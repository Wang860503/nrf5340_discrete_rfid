/*
 * 針對外部電源 Duty 55 調優版：
 * 1. 暴力過濾 125kHz 載波殘留雜訊
 * 2. 修正曼徹斯特判定區間以匹配實測之 305~397us 脈衝
 */

#include <errno.h>
#include <hal/nrf_gpio.h>
#include <nrfx_pwm.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/comparator.h>
#include <zephyr/drivers/comparator/nrf_comp.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#if DT_NODE_EXISTS(DT_NODELABEL(comp)) && \
    DT_NODE_HAS_STATUS(DT_NODELABEL(comp), okay)
BUILD_ASSERT(DT_PROP(DT_NODELABEL(comp), enable_hyst) != 0,
             "app.overlay: &comp 請保留 enable-hyst");
#endif

LOG_MODULE_REGISTER(rfid_main);

/* PWM 載波：DAMP(center-aligned) 下 f = 16MHz / (2 * TOP)，TOP=64 => 125kHz */
#define RFID_PWM_TOP 64U
#define RFID_PWM_CH_A 10U
#define RFID_PWM_CH_B 53U /* 與 CH_A 相差 16 ticks，約 1us @ 16MHz */
#define TICK_BUFFER_SIZE 2048
#define EM_DECODED_BITS_CAP 1024

/* --- BSP 核心調優區 --- */

/* 兩次「採納邊緣」至少間隔如此，否則視為載波漣波（125kHz 週期約 8us） */
#define EM_EDGE_MIN_US 500

/* 125kHz/64 曼徹斯特：半位元約 256us、全位元約 512us（留邊界給 LC 與卡片誤差）
 */
#define EM_SHORT_MIN 500
#define EM_SHORT_MAX 750
#define EM_LONG_MIN 800
#define EM_LONG_MAX 1300

/* 保持偵測穩定性的參數 */
#define EM_MIN_STORED_TO_DECODE 50
#define RFID_CAPTURE_WINDOW_MS 600
#define EM_SKIP_FIRST_GAP_US 2500
#define EM_MANCHESTER_GAP_HOLD_US 10000
#define EM_MANCHESTER_RESYNC_US 15000

static const struct device* comp_dev = DEVICE_DT_GET(DT_NODELABEL(comp));
static const nrfx_pwm_t rfid_pwm = NRFX_PWM_INSTANCE(0);
static nrf_pwm_values_individual_t rfid_seq_values[] = {
    {
        .channel_0 = (uint16_t)(RFID_PWM_CH_A | 0x8000U),
        .channel_1 = (uint16_t)(RFID_PWM_CH_B | 0x8000U),
        .channel_2 = 0,
        .channel_3 = 0,
    },
};
static const nrf_pwm_sequence_t rfid_seq = {
    .values.p_individual = rfid_seq_values,
    .length = NRF_PWM_VALUES_LENGTH(rfid_seq_values),
    .repeats = 0,
    .end_delay = 0,
};

static uint32_t tick_buffer[TICK_BUFFER_SIZE];
static uint8_t dec_raw_bits[EM_DECODED_BITS_CAP];
static bool pwm_ready;
static bool carrier_running;

static int start_carrier_with_pwm(void) {
  nrfx_err_t err;

  if (!pwm_ready) {
    nrfx_pwm_config_t const config = {
        .output_pins =
            {
                NRF_GPIO_PIN_MAP(1, 9),  /* P1.09 */
                NRF_GPIO_PIN_MAP(1, 10), /* P1.10 */
                NRF_PWM_PIN_NOT_CONNECTED,
                NRF_PWM_PIN_NOT_CONNECTED,
            },
        .irq_priority = NRFX_PWM_DEFAULT_CONFIG_IRQ_PRIORITY,
        .base_clock = NRF_PWM_CLK_16MHz,
        .count_mode = NRF_PWM_MODE_UP_AND_DOWN,
        .top_value = RFID_PWM_TOP,
        .load_mode = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode = NRF_PWM_STEP_AUTO,
        .skip_gpio_cfg = false,
        .skip_psel_cfg = false,
    };

    err = nrfx_pwm_init(&rfid_pwm, &config, NULL, NULL);
    if (!(err == NRFX_SUCCESS || err == NRFX_ERROR_ALREADY ||
          err == NRFX_ERROR_INVALID_STATE)) {
      LOG_ERR("pwm init failed: %d", err);
      return -EIO;
    }
    pwm_ready = true;
  }

  if (!carrier_running) {
    (void)nrfx_pwm_simple_playback(&rfid_pwm, &rfid_seq, 1, NRFX_PWM_FLAG_LOOP);
    carrier_running = true;
  }

  LOG_INF("PWM carrier ON: top=%u mode=DAMP ch=%u/%u (~125kHz)", RFID_PWM_TOP,
          RFID_PWM_CH_A, RFID_PWM_CH_B);
  return 0;
}

static void stop_carrier_with_pwm(void) {
  if (!pwm_ready || !carrier_running) return;
  (void)nrfx_pwm_stop(&rfid_pwm, true);
  carrier_running = false;
}

void invert_bits(uint8_t* bits, int len) {
  for (int i = 0; i < len; i++) {
    bits[i] = !bits[i];
  }
}

int find_em4100_header(uint8_t* bits, int len) {
  if (len < 64) return -1;
  for (int i = 0; i <= len - 64; i++) {
    if (i > 0 && bits[i - 1] == 1) continue;
    int ones = 0;
    for (int j = 0; j < 9; j++) {
      if (bits[i + j] == 1)
        ones++;
      else
        break;
    }
    if (ones == 9) return i;
  }
  return -1;
}

int EM4100_Full_Check(uint8_t* bits) {
  for (int j = 0; j < 9; j++)
    if (bits[j] != 1) return 0;
  if (bits[63] != 0) return 0;

  uint8_t col_parity[4] = {0};
  for (int row = 0; row < 10; row++) {
    int base = 9 + row * 5;
    int row_sum = 0;
    for (int col = 0; col < 4; col++) {
      uint8_t val = bits[base + col];
      row_sum += val;
      col_parity[col] += val;
    }
    if ((row_sum + bits[base + 4]) % 2 != 0) return 0;
  }
  for (int col = 0; col < 4; col++) {
    if ((col_parity[col] + bits[59 + col]) % 2 != 0) return 0;
  }
  return 1;
}

static int em4100_valid_at(const uint8_t* bits, int len, int off) {
  if (off < 0 || off + 64 > len) return 0;
  if (off > 0 && bits[off - 1] != 0) return 0;
  return EM4100_Full_Check((uint8_t*)&bits[off]);
}

uint64_t decode_card_code(uint8_t* bits) {
  uint64_t code = 0;
  for (int j = 0; j < 10; j++) {
    unsigned int digit = (bits[9 + 5 * j] << 3) | (bits[9 + 5 * j + 1] << 2) |
                         (bits[9 + 5 * j + 2] << 1) |
                         (bits[9 + 5 * j + 3] << 0);
    code |= ((uint64_t)digit << (4 * (9 - j)));
  }
  return (code & 0xFFFFFFFF);
}

static int try_em4100_slide(const uint8_t* bits, int len, uint64_t* out_code) {
  if (len < 64) return -1;
  for (int i = 0; i <= len - 64; i++) {
    if (em4100_valid_at(bits, len, i)) {
      *out_code = decode_card_code((uint8_t*)&bits[i]);
      return i;
    }
  }
  return -1;
}

void decode_bitstream(uint16_t total_ticks, uint8_t* out_bits, int* out_len) {
  int bit_idx = 0;
  uint8_t current_val = 1;
  int state = 0;

  memset(out_bits, 0, EM_DECODED_BITS_CAP);
  *out_len = 0;

  for (int i = 0; i < total_ticks; i++) {
    uint32_t T = tick_buffer[i];
    if (T < EM_EDGE_MIN_US) continue;

    if (T >= EM_LONG_MIN && T <= EM_LONG_MAX) {
      current_val = !current_val;
      if (bit_idx < EM_DECODED_BITS_CAP) out_bits[bit_idx++] = current_val;
      state = 0;
    } else if (T >= EM_SHORT_MIN && T <= EM_SHORT_MAX) {
      if (state == 0) {
        state = 1;
      } else {
        if (bit_idx < EM_DECODED_BITS_CAP) out_bits[bit_idx++] = current_val;
        state = 0;
      }
    } else {
      state = 0;
    }
  }
  *out_len = bit_idx;
}

static void tick_buffer_append(uint32_t diff_us, int* demod_counter,
                               bool* tick_window_slid) {
  if (*demod_counter >= TICK_BUFFER_SIZE) {
    const size_t half = TICK_BUFFER_SIZE / 2;
    memmove(tick_buffer, tick_buffer + half, half * sizeof(tick_buffer[0]));
    *demod_counter = (int)half;
    *tick_window_slid = true;
  }
  tick_buffer[(*demod_counter)++] = diff_us;
}

int em4095_comp_receiver(void) {
  int demod_counter = 0;
  uint32_t raw_edges = 0;
  uint32_t max_gap_us = 0;
  uint32_t gap_lt_100 = 0;
  uint32_t gap_100_230 = 0;
  uint32_t gap_231_520 = 0;
  uint32_t gap_gt_520 = 0;

  if (!device_is_ready(comp_dev)) return 0;

  k_sleep(K_MSEC(50));

  int stable = comparator_get_output(comp_dev);
  uint8_t last_val = (stable > 0) ? 1 : 0;
  uint32_t t0 = k_cycle_get_32();
  uint32_t last_phy_cycle = t0;
  uint32_t last_accepted_cycle = t0;
  uint64_t stop_time = k_uptime_get() + RFID_CAPTURE_WINDOW_MS;
  bool lead_sync_dropped = false;
  bool tick_window_slid = false;

  while (k_uptime_get() < stop_time) {
    int current_val = comparator_get_output(comp_dev);
    if (current_val < 0) continue;

    if (current_val != last_val) {
      uint32_t now = k_cycle_get_32();
      uint32_t diff_phy = k_cyc_to_us_near32(now - last_phy_cycle);

      raw_edges++;
      if (diff_phy > max_gap_us) max_gap_us = diff_phy;
      if (diff_phy < 100) {
        gap_lt_100++;
      } else if (diff_phy <= 230) {
        gap_100_230++;
      } else if (diff_phy <= 520) {
        gap_231_520++;
      } else {
        gap_gt_520++;
      }
      last_phy_cycle = now;

      /* 僅在距「上次採納邊緣」夠遠時記錄 gap；短間隔仍同步
       * last_val，避免載波毛刺造成忙等 */
      uint32_t diff_acc = k_cyc_to_us_near32(now - last_accepted_cycle);
      if (diff_acc > EM_EDGE_MIN_US) {
        if (!lead_sync_dropped) {
          lead_sync_dropped = true;
          if (diff_acc <= EM_SKIP_FIRST_GAP_US) {
            tick_buffer_append(diff_acc, &demod_counter, &tick_window_slid);
          }
        } else {
          tick_buffer_append(diff_acc, &demod_counter, &tick_window_slid);
        }
        last_accepted_cycle = now;
      }
      last_val = (uint8_t)(current_val > 0 ? 1 : 0);
    }
  }

  LOG_INF("scan: raw_edges=%u stored=%d max_gap_us=%u", raw_edges,
          demod_counter, max_gap_us);
  LOG_INF("gap_hist: <100=%u 100-230=%u 231-520=%u >520=%u", gap_lt_100,
          gap_100_230, gap_231_520, gap_gt_520);

  if (demod_counter >= EM_MIN_STORED_TO_DECODE) {
    int raw_len = 0;
    decode_bitstream(demod_counter, dec_raw_bits, &raw_len);
    LOG_INF("Captured: %d ticks, Decoded: %d bits", demod_counter, raw_len);

    uint64_t code;
    // 嘗試正向
    int slide = try_em4100_slide(dec_raw_bits, raw_len, &code);
    if (slide >= 0) {
      LOG_WRN("[SUCCESS] ID: %08llX", code);
      return 1;
    }
    // 嘗試反轉
    invert_bits(dec_raw_bits, raw_len);
    slide = try_em4100_slide(dec_raw_bits, raw_len, &code);
    if (slide >= 0) {
      LOG_WRN("[SUCCESS-Inv] ID: %08llX", code);
      return 1;
    }

    LOG_INF("Fingerprint: %u, %u, %u, %u, %u, %u, %u, %u, %u, %u",
            tick_buffer[0], tick_buffer[1], tick_buffer[2], tick_buffer[3],
            tick_buffer[4], tick_buffer[5], tick_buffer[6], tick_buffer[7],
            tick_buffer[8], tick_buffer[9]);
  }
  return 0;
}

/* 參數須與 app.overlay &comp 一致：DIFF(AIN2 vs AIN3) + hysteresis。 */
static void comp_reapply_diff_ain2_ain3(void) {
  if (!device_is_ready(comp_dev)) return;
  const struct comp_nrf_comp_diff_config cfg = {
      .psel = COMP_NRF_COMP_PSEL_AIN2,
      .sp_mode = COMP_NRF_COMP_SP_MODE_HIGH,
      .isource = COMP_NRF_COMP_ISOURCE_DISABLED,
      .extrefsel = COMP_NRF_COMP_EXTREFSEL_AIN3,
      .enable_hyst = true,
  };
  int r = comp_nrf_comp_configure_diff(comp_dev, &cfg);
  if (r != 0) {
    LOG_WRN("comp re-apply diff AIN2/AIN3 failed: %d", r);
  } else {
    LOG_INF("COMP: DIFF AIN2 vs AIN3 + hysteresis ON");
  }
}

int main(void) {
  int carrier_ret;

  LOG_INF("nRF5340 Discrete RFID Starting...");

  comp_reapply_diff_ain2_ain3();

  while (1) {
    carrier_ret = start_carrier_with_pwm();
    if (carrier_ret != 0) {
      LOG_ERR("Carrier PWM wave start failed, ret=%d", carrier_ret);
      k_sleep(K_MSEC(1500));
      continue;
    }

    if (!em4095_comp_receiver()) {
      LOG_INF("No card detected.");
    }

    /* 只在掃描窗口內發射，待機時關閉載波。 */
    stop_carrier_with_pwm();
    k_sleep(K_MSEC(1500));
  }
  return 0;
}