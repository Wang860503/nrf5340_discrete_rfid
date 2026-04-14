/*
 * 針對外部電源 Duty 55 調優版：
 * 1. 暴力過濾 125kHz 載波殘留雜訊
 * 2. 修正曼徹斯特判定區間以匹配實測之 305~397us 脈衝
 */

#include <errno.h>
#include <hal/nrf_gpiote.h>
#include <hal/nrf_timer.h>
#include <nrfx_dppi.h>
#include <nrfx_gpiote.h>
#include <nrfx_timer.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/comparator.h>
#include <zephyr/drivers/comparator/nrf_comp.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#if DT_NODE_EXISTS(DT_NODELABEL(comp)) && \
    DT_NODE_HAS_STATUS(DT_NODELABEL(comp), okay)
BUILD_ASSERT(
    DT_PROP(DT_NODELABEL(comp), enable_hyst) != 0,
    "app.overlay: &comp 請保留 enable-hyst（nRF5340 差動模式即晶片最大遲滯）");
#endif

LOG_MODULE_REGISTER(rfid_main);

/* 載波：計時器在 PWM_PERIOD tick CLEAR → 125kHz；A 延後開啟騰出 B→A 死區 */
#define PWM_PERIOD 128
#define RFID_DEAD_TICKS 44
#define RFID_CH_A_ON_LEAD 26U /* 約 1.25µs @ 16MHz；與 2*dead 幾何搭配 */
#define RFID_GPIOTE_PIN_A 41  /* P1.09 */
#define RFID_GPIOTE_PIN_B 42  /* P1.10 */
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
static const nrfx_timer_t rfid_timer = NRFX_TIMER_INSTANCE(2);
static const nrfx_gpiote_t rfid_gpiote = NRFX_GPIOTE_INSTANCE(0);
static const nrfx_dppi_t rfid_dppi = NRFX_DPPI_INSTANCE(0);

static uint32_t tick_buffer[TICK_BUFFER_SIZE];
static uint8_t dec_raw_bits[EM_DECODED_BITS_CAP];

static bool nrfx_ok_or_already(nrfx_err_t err) {
  return (err == NRFX_SUCCESS) || (err == NRFX_ERROR_INVALID_STATE) ||
         (err == NRFX_ERROR_ALREADY);
}

static int start_carrier_with_dppi_deadtime(void) {
  uint32_t ch_a_on;
  uint32_t ch_a_off;
  uint32_t ch_b_on;
  uint32_t ch_b_off;
  uint32_t active_ticks_a;
  uint32_t active_ticks_b;
  uint8_t te_ch_a;
  uint8_t te_ch_b;
  uint8_t dppi_ch_a_on;
  uint8_t dppi_ch_a_off;
  uint8_t dppi_ch_b_on;
  uint8_t dppi_ch_b_off;
  nrfx_err_t err;

  LOG_INF("dppi: setup enter dead=%u period=%u lead_A=%u", RFID_DEAD_TICKS,
          PWM_PERIOD, RFID_CH_A_ON_LEAD);

  /* 125kHz / 對稱死區幾何：總導通 = PWM - 2*dead，A 延後 RFID_CH_A_ON_LEAD 再開
   */
  {
    uint32_t total_active = PWM_PERIOD - (RFID_DEAD_TICKS * 2U);
    if (total_active < 4U) {
      LOG_ERR("dead too long for period (2*dead=%u period=%u)",
              2U * RFID_DEAD_TICKS, PWM_PERIOD);
      return -EINVAL;
    }
    active_ticks_a = total_active / 2U;
    active_ticks_b = total_active - active_ticks_a;
    ch_a_on = RFID_CH_A_ON_LEAD;
    ch_a_off = ch_a_on + active_ticks_a;
    ch_b_on = ch_a_off + RFID_DEAD_TICKS;
    ch_b_off = ch_b_on + active_ticks_b;
    if (ch_b_off > PWM_PERIOD) {
      LOG_ERR("ch_b_off %u > PWM_PERIOD %u (增大 lead 或減 dead)", ch_b_off,
              PWM_PERIOD);
      return -EINVAL;
    }
  }

  /* Zephyr gpio_nrfx 會先 init GPIOTE0，此處再呼叫會得到
   * NRFX_ERROR_ALREADY（須視為成功） */
  err = nrfx_gpiote_init(&rfid_gpiote, NRFX_GPIOTE_DEFAULT_CONFIG_IRQ_PRIORITY);
  if (!nrfx_ok_or_already(err)) {
    LOG_ERR("gpiote_init failed: %d", err);
    return -EIO;
  }

  err = nrfx_gpiote_channel_alloc(&rfid_gpiote, &te_ch_a);
  if (err != NRFX_SUCCESS) {
    LOG_ERR("gpiote ch alloc A: %d", err);
    return -ENOMEM;
  }
  err = nrfx_gpiote_channel_alloc(&rfid_gpiote, &te_ch_b);
  if (err != NRFX_SUCCESS) {
    LOG_ERR("gpiote ch alloc B: %d", err);
    return -ENOMEM;
  }

  nrfx_gpiote_output_config_t out_cfg = {
      .drive = NRF_GPIO_PIN_H0H1,
      .input_connect = NRF_GPIO_PIN_INPUT_DISCONNECT,
      .pull = NRF_GPIO_PIN_NOPULL,
  };
  /* Task 模式 + LoToHi：由 DPPI 改訂閱 SET/CLR（非 OUT
   * toggle），避免漏脈衝造成相位反轉 */
  nrfx_gpiote_task_config_t task_a = {
      .task_ch = te_ch_a,
      .polarity = NRF_GPIOTE_POLARITY_LOTOHI,
      .init_val = NRF_GPIOTE_INITIAL_VALUE_LOW,
  };
  nrfx_gpiote_task_config_t task_b = {
      .task_ch = te_ch_b,
      .polarity = NRF_GPIOTE_POLARITY_LOTOHI,
      .init_val = NRF_GPIOTE_INITIAL_VALUE_LOW,
  };

  err = nrfx_gpiote_output_configure(&rfid_gpiote, RFID_GPIOTE_PIN_A, &out_cfg,
                                     &task_a);
  if (err != NRFX_SUCCESS) {
    LOG_ERR("gpiote out cfg A pin=%u: %d", (unsigned)RFID_GPIOTE_PIN_A, err);
    return -EIO;
  }
  err = nrfx_gpiote_output_configure(&rfid_gpiote, RFID_GPIOTE_PIN_B, &out_cfg,
                                     &task_b);
  if (err != NRFX_SUCCESS) {
    LOG_ERR("gpiote out cfg B pin=%u: %d", (unsigned)RFID_GPIOTE_PIN_B, err);
    return -EIO;
  }

  nrfx_gpiote_out_clear(&rfid_gpiote, RFID_GPIOTE_PIN_A);
  nrfx_gpiote_out_clear(&rfid_gpiote, RFID_GPIOTE_PIN_B);

  /* 以 channel index 啟用 Task（避免 nrfx 依 pin 查表時與 Zephyr
   * 內部狀態不一致） */
  nrf_gpiote_task_enable(rfid_gpiote.p_reg, te_ch_a);
  nrf_gpiote_task_enable(rfid_gpiote.p_reg, te_ch_b);
  LOG_INF("dppi: gpiote TE ch=%u,%u enabled", te_ch_a, te_ch_b);

  if (ch_b_off < PWM_PERIOD && rfid_timer.cc_channel_count < 5U) {
    LOG_ERR("TIMER2 needs >=5 CC (B_CLR@ch_b_off=%u, CLEAR@%u)", ch_b_off,
            PWM_PERIOD);
    return -ENOTSUP;
  }

  /* NRFX_TIMER_DEFAULT_CONFIG 要的是 Hz，勿用
   * nrf_timer_frequency_t（NRF_TIMER_FREQ_16MHz==0 會除零） */
  nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG(16000000U);
  err = nrfx_timer_init(&rfid_timer, &timer_cfg, NULL);
  if (!nrfx_ok_or_already(err)) {
    LOG_ERR("timer_init failed: %d", err);
    return -EIO;
  }

  nrfx_timer_clear(&rfid_timer);
  nrfx_timer_extended_compare(&rfid_timer, NRF_TIMER_CC_CHANNEL0, ch_a_on, 0,
                              false);
  nrfx_timer_extended_compare(&rfid_timer, NRF_TIMER_CC_CHANNEL1, ch_a_off, 0,
                              false);
  nrfx_timer_extended_compare(&rfid_timer, NRF_TIMER_CC_CHANNEL2, ch_b_on, 0,
                              false);
  /* B 關在 ch_b_off；週期務必在 PWM_PERIOD 才 CLEAR（ch_b_off<128 時須 CC4） */
  if (ch_b_off < PWM_PERIOD) {
    nrfx_timer_extended_compare(&rfid_timer, NRF_TIMER_CC_CHANNEL3, ch_b_off, 0,
                                false);
    nrfx_timer_extended_compare(&rfid_timer, NRF_TIMER_CC_CHANNEL4, PWM_PERIOD,
                                NRF_TIMER_SHORT_COMPARE4_CLEAR_MASK, false);
  } else {
    nrfx_timer_extended_compare(&rfid_timer, NRF_TIMER_CC_CHANNEL3, ch_b_off,
                                NRF_TIMER_SHORT_COMPARE3_CLEAR_MASK, false);
  }

  err = nrfx_dppi_channel_alloc(&rfid_dppi, &dppi_ch_a_on);
  if (err != NRFX_SUCCESS) {
    LOG_ERR("dppi alloc a_on: %d", err);
    return -ENOMEM;
  }
  err = nrfx_dppi_channel_alloc(&rfid_dppi, &dppi_ch_a_off);
  if (err != NRFX_SUCCESS) {
    LOG_ERR("dppi alloc a_off: %d", err);
    return -ENOMEM;
  }
  err = nrfx_dppi_channel_alloc(&rfid_dppi, &dppi_ch_b_on);
  if (err != NRFX_SUCCESS) {
    LOG_ERR("dppi alloc b_on: %d", err);
    return -ENOMEM;
  }
  err = nrfx_dppi_channel_alloc(&rfid_dppi, &dppi_ch_b_off);
  if (err != NRFX_SUCCESS) {
    LOG_ERR("dppi alloc b_off: %d", err);
    return -ENOMEM;
  }

  nrf_timer_publish_set(rfid_timer.p_reg, NRF_TIMER_EVENT_COMPARE0,
                        dppi_ch_a_on);
  nrf_gpiote_subscribe_set(rfid_gpiote.p_reg, nrf_gpiote_set_task_get(te_ch_a),
                           dppi_ch_a_on);

  nrf_timer_publish_set(rfid_timer.p_reg, NRF_TIMER_EVENT_COMPARE1,
                        dppi_ch_a_off);
  nrf_gpiote_subscribe_set(rfid_gpiote.p_reg, nrf_gpiote_clr_task_get(te_ch_a),
                           dppi_ch_a_off);

  nrf_timer_publish_set(rfid_timer.p_reg, NRF_TIMER_EVENT_COMPARE2,
                        dppi_ch_b_on);
  nrf_gpiote_subscribe_set(rfid_gpiote.p_reg, nrf_gpiote_set_task_get(te_ch_b),
                           dppi_ch_b_on);

  /* B_CLR 僅綁 CC3@ch_b_off；CC4@PWM_PERIOD 只做 TIMER CLEAR，不發 DPPI */
  nrf_timer_publish_set(rfid_timer.p_reg, NRF_TIMER_EVENT_COMPARE3,
                        dppi_ch_b_off);
  nrf_gpiote_subscribe_set(rfid_gpiote.p_reg, nrf_gpiote_clr_task_get(te_ch_b),
                           dppi_ch_b_off);

  (void)nrfx_dppi_channel_enable(&rfid_dppi, dppi_ch_a_on);
  (void)nrfx_dppi_channel_enable(&rfid_dppi, dppi_ch_a_off);
  (void)nrfx_dppi_channel_enable(&rfid_dppi, dppi_ch_b_on);
  (void)nrfx_dppi_channel_enable(&rfid_dppi, dppi_ch_b_off);

  nrfx_timer_enable(&rfid_timer);

  LOG_INF("DPPI carrier: pwm=%u dead=%u active=%u+%u b_off=%u tail=%u%s",
          PWM_PERIOD, RFID_DEAD_TICKS, active_ticks_a, active_ticks_b, ch_b_off,
          PWM_PERIOD - ch_b_off,
          (ch_b_off < PWM_PERIOD) ? " CC4=CLEAR" : " CC3=CLEAR");
  LOG_INF("Timing ticks: A[%u,%u] B[%u,%u] te=%u,%u dppi=%u,%u,%u,%u", ch_a_on,
          ch_a_off, ch_b_on, ch_b_off, te_ch_a, te_ch_b, dppi_ch_a_on,
          dppi_ch_a_off, dppi_ch_b_on, dppi_ch_b_off);
  return 0;
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

/* 參數須與 app.overlay &comp 一致：SE + VDD 參考 + th_down/th_up 門檻。 */
static void comp_reapply_se_threshold(void) {
  if (!device_is_ready(comp_dev)) return;
  const struct comp_nrf_comp_se_config cfg = {
      .psel = COMP_NRF_COMP_PSEL_AIN2,
      .sp_mode = COMP_NRF_COMP_SP_MODE_HIGH,
      .isource = COMP_NRF_COMP_ISOURCE_DISABLED,
      .extrefsel = COMP_NRF_COMP_EXTREFSEL_AIN3,
      .refsel = COMP_NRF_COMP_REFSEL_VDD,
      .th_down = 23,
      .th_up = 25,
  };
  int r = comp_nrf_comp_configure_se(comp_dev, &cfg);
  if (r != 0) {
    LOG_WRN("comp re-apply se threshold failed: %d", r);
  } else {
    LOG_INF("COMP: SE ref=VDD th=[33,35] applied");
  }
}

int main(void) {
  int dppi_ret;

  LOG_INF("nRF5340 Discrete RFID Starting...");

  comp_reapply_se_threshold();

  dppi_ret = start_carrier_with_dppi_deadtime();
  if (dppi_ret == 0) {
    LOG_INF("Carrier DPPI wave started.");
  } else {
    LOG_ERR("Carrier DPPI wave start failed, ret=%d", dppi_ret);
  }

  while (1) {
    if (!em4095_comp_receiver()) {
      LOG_INF("No card detected.");
    }
    k_sleep(K_MSEC(1500));
  }
  return 0;
}