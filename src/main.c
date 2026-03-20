/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <helpers/nrfx_gppi.h>
#include <nrfx_saadc.h>
#include <nrfx_timer.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(discrete_rfid, LOG_LEVEL_INF);

/* 物理參數定義 */
#define PWM_PERIOD_NS 8000      // 125kHz
#define ADC_BUF_SIZE 64         // 32組(Env+Ref)數據
#define SAMPLING_RATE_HZ 50000  // 50kHz 採樣
#define TICK_BUFFER_SIZE 400
#define EM_SHORT_MIN 200
#define EM_SHORT_MAX 310
#define EM_LONG_MIN 450
#define EM_LONG_MAX 600
#define CALIB_SAMPLES_REQUIRED 2000

/* 硬體實例 */
static const nrfx_timer_t m_timer = NRFX_TIMER_INSTANCE(1);
static uint8_t m_gppi_ch;
/* 修正：針對 nrfx 3.2.0+ 使用 int16_t 定義緩衝區 */
static int16_t adc_buffer[2][ADC_BUF_SIZE];
static uint8_t active_buf = 0;

/* 數據快取與狀態 */
static uint32_t tick_buffer[TICK_BUFFER_SIZE];
static uint16_t demod_counter = 0;
static bool last_env_state = false;
static uint32_t last_edge_cycle = 0;
static struct k_work rfid_decode_work;

// calibration parameter
static bool is_calibrating = true;
static int32_t calib_sample_count = 0;
static int16_t calib_max = 0;
static int16_t calib_min = 4095;
static int16_t fixed_hysteresis = 60;

int EM4100_Full_Check(uint8_t* bits) {
  // 檢查 Header (9個1)
  for (int i = 0; i < 9; i++)
    if (bits[i] != 1) return 0;
  // 檢查 Stop Bit (1個0)
  if (bits[63] != 0) return 0;

  uint8_t col_parity_calc[4] = {0};

  // Row Parity
  for (int row = 0; row < 10; row++) {
    int base = 9 + row * 5;
    int row_sum = 0;
    for (int col = 0; col < 4; col++) {
      uint8_t val = bits[base + col];
      row_sum += val;
      col_parity_calc[col] += val;
    }
    // 偶同位檢查
    if ((row_sum + bits[base + 4]) % 2 != 0) return 0;
  }

  // 檢查 Column Parity
  for (int col = 0; col < 4; col++) {
    if ((col_parity_calc[col] + bits[59 + col]) % 2 != 0) return 0;
  }
  return 1;  // 校驗通過
}

uint64_t decode_card_code(uint8_t* bits) {
  uint64_t code = 0;
  for (int j = 0; j < 10; j++) {
    unsigned int digit = (bits[9 + 5 * j] << 3) | (bits[9 + 5 * j + 1] << 2) |
                         (bits[9 + 5 * j + 2] << 1) |
                         (bits[9 + 5 * j + 3] << 0);
    code |= ((uint64_t)digit << (4 * (9 - j)));
  }
  /*total 40 bits, 保留最後 32 bits (8個 Hex)，過濾掉前面的 Customer ID*/
  return (code & 0xFFFFFFFF);
}

void decode_bitstream(uint16_t total_ticks, uint8_t* out_bits, int* out_len) {
  int bit_idx = 0;
  uint8_t current_val = 1;  // 假設 Header 為 1
  int state = 0;

  memset(out_bits, 0, 300);
  *out_len = 0;

  for (int i = 0; i < total_ticks; i++) {
    uint32_t T = tick_buffer[i];
    if (T == 0) continue;

    if (T > EM_LONG_MIN && T < EM_LONG_MAX) {
      // Long Pulse: 翻轉數值 (例如 0->1 或 1->0)
      if (state == 1) state = 0;  // 重置狀態
      current_val = !current_val;
      if (bit_idx < 300) out_bits[bit_idx++] = current_val;
      state = 0;
    } else if (T > EM_SHORT_MIN && T < EM_SHORT_MAX) {
      // Short Pulse: 保持數值
      if (state == 0) {
        state = 1;  // 等待第二個 Short
      } else {
        // 第二個 Short 到達，確認一個 Bit
        if (bit_idx < 300) out_bits[bit_idx++] = current_val;
        state = 0;
      }
    } else {
      // 異常長度，重置狀態
      state = 0;
    }
  }
  *out_len = bit_idx;
}

static void saadc_handler(nrfx_saadc_evt_t const* p_event) {
  if (p_event->type == NRFX_SAADC_EVT_DONE) {
    // 修正：強制轉型為 HAL 要求的 void*
    nrfx_saadc_buffer_set((nrf_saadc_value_t*)adc_buffer[active_buf],
                          ADC_BUF_SIZE);
    active_buf = (active_buf == 0) ? 1 : 0;

    int16_t* p_buf = (int16_t*)p_event->data.done.p_buffer;

    if (is_calibrating) {
      /* --- 校準模式：只找峰值，不解碼 --- */
      for (int i = 0; i < ADC_BUF_SIZE; i++) {
        if (p_buf[i] > calib_max) calib_max = p_buf[i];
        if (p_buf[i] < calib_min) calib_min = p_buf[i];
      }
      calib_sample_count += ADC_BUF_SIZE;

      if (calib_sample_count >= CALIB_SAMPLES_REQUIRED) {
        // 計算環境雜訊振幅
        int16_t noise_vpp = calib_max - calib_min;

        // 設定固定遲滯：雜訊的一半再加 20 單位安全餘量 (可根據需求調整)
        fixed_hysteresis = (noise_vpp / 2) + 20;

        // 防呆保護：避免門檻過低或過高
        if (fixed_hysteresis < 30) fixed_hysteresis = 30;

        is_calibrating = false;
        LOG_INF("calibrate done！ Vpp: %d, 固定遲滯設定為: %d", noise_vpp,
                fixed_hysteresis);
      }
      return;  // 校準期間不執行後續解碼
    }

    for (int i = 0; i < ADC_BUF_SIZE; i += 2) {
      int16_t env = p_buf[i], ref = p_buf[i + 1];
      bool current = (env > (ref + fixed_hysteresis))   ? true
                     : (env < (ref - fixed_hysteresis)) ? false
                                                        : last_env_state;

      if (current != last_env_state) {
        uint32_t now = k_cycle_get_32();
        if (last_edge_cycle != 0) {
          uint32_t diff = k_cyc_to_us_near32(now - last_edge_cycle);
          if (demod_counter < TICK_BUFFER_SIZE && diff > 100)
            tick_buffer[demod_counter++] = diff;
        }
        last_edge_cycle = now;
        last_env_state = current;
      }
    }
  }
}

void hardware_init(void) {
  // 1. PWM TX
  const struct device* pwm = DEVICE_DT_GET(DT_NODELABEL(pwm0));
  pwm_set_cycles(pwm, 0, PWM_PERIOD_NS, PWM_PERIOD_NS / 2, 0);
  pwm_set_cycles(pwm, 1, PWM_PERIOD_NS, PWM_PERIOD_NS / 2,
                 PWM_POLARITY_INVERTED);

  // 2. SAADC
  nrfx_err_t err = nrfx_saadc_init(NRFX_SAADC_DEFAULT_CONFIG_IRQ_PRIORITY);
  if (err != NRFX_SUCCESS) {
    LOG_ERR("SAADC init failed: %d", err);
    return;
  }

  nrfx_saadc_channel_t channels[] = {
      NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN0, 0),
      NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN1, 1),
  };

  // 手動微調硬體參數 (Gain 1/4, 內部參考電壓)
  for (int i = 0; i < 2; i++) {
    channels[i].channel_config.gain = NRF_SAADC_GAIN1_4;
    channels[i].channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL;
    channels[i].channel_config.acq_time = NRF_SAADC_ACQTIME_10US;
  }

  err = nrfx_saadc_channels_config(channels, 2);
  if (err != NRFX_SUCCESS) {
    return;
  }

  nrfx_saadc_adv_config_t adv_config = {
      .oversampling = NRF_SAADC_OVERSAMPLE_DISABLED,
      .burst = NRF_SAADC_BURST_DISABLED,
      .internal_timer_cc = 0,  // 我們使用外部 Timer 觸發，所以設為 0
      .start_on_end = false};

  // 呼叫正式的 v3 API 註冊 Handler
  // 參數：通道遮罩(BIT 0&1), 解析度, 配置結構, 回呼函數
  err = nrfx_saadc_advanced_mode_set(
      BIT(0) | BIT(1), NRF_SAADC_RESOLUTION_12BIT, &adv_config, saadc_handler);
  if (err != NRFX_SUCCESS) return;

  nrfx_saadc_buffer_set((nrf_saadc_value_t*)adc_buffer[0], ADC_BUF_SIZE);
  active_buf = 1;

  // 5. Timer 1 配置 (修正 DEFAULT_CONFIG)
  nrfx_timer_config_t timer_cfg =
      NRFX_TIMER_DEFAULT_CONFIG(NRF_TIMER_FREQ_16MHz);
  nrfx_timer_init(&m_timer, &timer_cfg, NULL);

  uint32_t ticks = nrfx_timer_ms_to_ticks(&m_timer, 1000.0 / SAMPLING_RATE_HZ);
  nrfx_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, ticks,
                              NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);

  // 6. GPPI 連結 (使用 HAL 層獲取 Task 地址)
  nrfx_gppi_channel_alloc(&m_gppi_ch);
  uint32_t timer_event_addr =
      nrfx_timer_event_address_get(&m_timer, NRF_TIMER_EVENT_COMPARE0);

  // 修正：直接從 HAL 獲取暫存器地址，避開 nrfx v3 驅動層的 API 不一致
  uint32_t saadc_task_addr =
      nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE);

  nrfx_gppi_channel_endpoints_setup(m_gppi_ch, timer_event_addr,
                                    saadc_task_addr);
  nrfx_gppi_channels_enable(BIT(m_gppi_ch));

  // 7. 啟動定時器
  nrfx_timer_enable(&m_timer);
  LOG_INF("RFID Hardware initialized with SAADC v3 API.");
}

void rfid_decode_work_handler(struct k_work* work) {
  uint8_t raw[350], valid[64];
  int len = 0;
  if (demod_counter < 128) {
    demod_counter = 0;
    return;
  }

  decode_bitstream(demod_counter, raw, &len);
  demod_counter = 0;

  for (int i = 0; i <= len - 64; i++) {
    if (raw[i] == 1 && raw[i + 1] == 1) {  // 尋找 Header
      memcpy(valid, &raw[i], 64);
      if (EM4100_Full_Check(valid)) {
        LOG_INF("Card Found: %010llu ", decode_card_code(valid));
        last_edge_cycle = 0;
        return;
      }
    }
  }
}

int main(void) {
  LOG_INF("Discrete RFID");
  hardware_init();
  k_work_init(&rfid_decode_work, rfid_decode_work_handler);

  while (1) {
    if (demod_counter > 200) k_work_submit(&rfid_decode_work);
    k_sleep(K_MSEC(100));
  }

  return 0;
}
