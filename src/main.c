/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(rfid_main);

#define PWM_PERIOD 128  // 125kHz
#define RFID_50_PERCENT_DUTY 64
#define TICK_BUFFER_SIZE 400
#define ADC_HYSTERESIS 25  // 數位遲滯 LSB
#define EM_SHORT_MIN 180   // us
#define EM_SHORT_MAX 350   // us
#define EM_LONG_MIN 400    // us
#define EM_LONG_MAX 650    // us

static const struct device* adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc));
static const struct adc_dt_spec adc_chan_0 =
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);
static const struct adc_dt_spec adc_chan_1 =
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1);
static const struct device* pwm_dev = DEVICE_DT_GET(DT_NODELABEL(pwm0));

static uint32_t tick_buffer[TICK_BUFFER_SIZE];
static uint8_t data_valid[64];
static uint64_t em_card_code = 0;

static int16_t adc_raw[2];
static const struct adc_sequence sequence = {
    .channels = BIT(0) | BIT(1),
    .buffer = adc_raw,
    .buffer_size = sizeof(adc_raw),
    .resolution = 12,
};

int EM4100_Full_Check(uint8_t* bits) {
  // 檢查 Header (9個1)
  for (int i = 0; i < 9; i++)
    if (bits[i] != 1) return 0;
  // 檢查 Stop Bit (1個0)
  if (bits[63] != 0) return 0;

  uint8_t col_parity_calc[4] = {0};

  // 檢查 Row Parity
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

int em4095_adc_receiver(void) {
  uint8_t raw_bits[350];
  int raw_bit_count = 0;
  int demod_counter = 0;

  /* start 125KHZ */
  pwm_set_cycles(pwm_dev, 0, PWM_PERIOD, RFID_50_PERCENT_DUTY, 0);
  pwm_set_cycles(pwm_dev, 1, PWM_PERIOD, RFID_50_PERCENT_DUTY,
                 PWM_POLARITY_INVERTED);

  k_sleep(K_MSEC(10));

  adc_read(adc_dev, &sequence);
  LOG_DBG("ADC Test - CH0: %d, CH1: %d\n", adc_raw[0], adc_raw[1]);

  /*start sampling*/
  bool last_state = false;
  uint32_t start_cycle = k_cycle_get_32();
  uint32_t last_edge_cycle = start_cycle;

  uint64_t stop_time = k_uptime_get() + 120;
  while (k_uptime_get() < stop_time) {
    if (adc_read(adc_dev, &sequence) == 0) {
      bool current_state = (adc_raw[1] > (adc_raw[0] + ADC_HYSTERESIS));

      if (current_state != last_state) {
        uint32_t now = k_cycle_get_32();
        uint32_t diff_us = k_cyc_to_us_near32(now - last_edge_cycle);

        if (demod_counter < TICK_BUFFER_SIZE && diff_us > 50) {  // 簡單去噪
          tick_buffer[demod_counter++] = diff_us;
        }
        last_edge_cycle = now;
        last_state = current_state;
      }
    }
    k_busy_wait(20);  // 控制採樣頻率約 50kHz
  }

  /*stop sampling*/
  pwm_set_cycles(pwm_dev, 0, PWM_PERIOD, 0, 0);
  pwm_set_cycles(pwm_dev, 1, PWM_PERIOD, 0, 0);

  LOG_INF("Sampling done. Captured ticks: %d", demod_counter);

  /*decode*/
  decode_bitstream(demod_counter, raw_bits, &raw_bit_count);
  if (raw_bit_count < 64) return 0;

  for (int i = 0; i <= raw_bit_count - 64; i++) {
    int header_match = 1;
    for (int k = 0; k < 9; k++) {
      header_match &= raw_bits[i + k];
      if (!header_match) {
        break;
      }
    }

    if (!header_match) continue;

    for (int k = 0; k < 64; k++) data_valid[k] = raw_bits[i + k];

    if (EM4100_Full_Check(data_valid) == 1) {
      em_card_code = decode_card_code(data_valid);
      LOG_INF("Card Found: (%010llu)", em_card_code);
      return 1;
    }
  }
  return 0;
}

int main(void) {
  int err;
  LOG_INF("nRF5340 Discrete RFID Starting...");
  err = adc_channel_setup_dt(&adc_chan_0);
  if (err < 0) {
    LOG_ERR("Could not setup channel (%d)\n", err);
    return err;
  }
  err = adc_channel_setup_dt(&adc_chan_1);
  if (err < 0) {
    LOG_ERR("Could not setup channel (%d)\n", err);
    return err;
  }

  while (1) {
    if (!em4095_adc_receiver()) {
      LOG_INF("No card detected.");
    }
    k_sleep(K_MSEC(1000));
  }

  return 0;
}
