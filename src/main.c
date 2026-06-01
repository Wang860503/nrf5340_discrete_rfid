/*
 * nRF5340 SAADC 差動接收 + PWM 載波 + FSK/HID Prox 解碼（精簡版，無 COMP 路徑）
 */

#include <errno.h>
#include <hal/nrf_gpio.h>
#include <limits.h>
#include <nrfx_pwm.h>
#include <nrfx_saadc.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(rfid_main);

#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/* ==================== 解碼模式切換 ==================== */
#define DECODE_MODE_ASK 0
#define DECODE_MODE_FSK 1

/* 0=ASK(EM4100)；1=FSK(HID Prox) */
#define ACTIVE_DECODE_MODE DECODE_MODE_FSK
/* 1=僅 FSK：首次載波穩定 + 縮短輪詢（不改 6 視窗解調／擷取長度） */
#define RFID_FSK_RECOGNITION_FAST 1
#if ACTIVE_DECODE_MODE == DECODE_MODE_FSK && RFID_FSK_RECOGNITION_FAST
#define RFID_FSK_FAST_ACTIVE 1
#else
#define RFID_FSK_FAST_ACTIVE 0
#endif
#if ACTIVE_DECODE_MODE == DECODE_MODE_FSK && RFID_FSK_FAST_ACTIVE
/* 1=上電即用實測 half/phase，略過首輪 ~8s 全掃（換卡/天線可改下面兩值或設 0）
 */
#define RFID_FSK_BOOTSTRAP_PHASE_LOCK 1
#define RFID_FSK_BOOTSTRAP_HALF_Q8 1600U
#define RFID_FSK_BOOTSTRAP_PHASE_Q8 96U
#else
#define RFID_FSK_BOOTSTRAP_PHASE_LOCK 0
#endif

#if ACTIVE_DECODE_MODE == DECODE_MODE_FSK
static int fsk_window_crosses;
static int16_t last_fsk_sample;
/* 量測此 CN 從本輪擷取開始到 HID 解碼成功的耗時 */
#define RFID_TIMING_TARGET_CN 20495U
static int64_t rfid_timing_round_start_ms;
static int64_t rfid_timing_decode_start_ms;
static uint32_t rfid_timing_capture_us;
/* 上一輪 HID 已成功：保留 half/phase，下一輪勿從 640 全掃 */
static bool rfid_saadc_hid_phase_hold;
/* bootstrap 已預設 hold：仍要在第一次 HID 成功時印 log */
static bool rfid_fsk_log_next_match;
#endif
/* ====================================================== */

#define RFID_PWM_TOP 64U
/* UP_AND_DOWN + INDIVIDUAL；對齊 EFM32 TIMER DTI（rise/fall dead time） */
#define RFID_PWM_DTI_ENABLE 1
#define RFID_PWM_DTI_CENTER (RFID_PWM_TOP / 2U)
#define RFID_PWM_DTI_EDGE_GUARD 4U
/* 兩路皆低的最小關斷窗（4 tick @ 16MHz ≈ 250ns），防 ch_a/ch_b 交叉與
 * shoot-through */
#define RFID_PWM_DTI_MIN_GAP 4U
#define RFID_PWM_DTI_DUTY_MAX 22U
/* Dead Time Optimization：rise/fall 可分開（對齊 EFR DTI）。
 * 16MHz PWM → 1 tick = 62.5ns；8 ticks ≈ 500ns。 */
#define RFID_PWM_DTI_DEAD_RISE_TICKS 5U
#define RFID_PWM_DTI_DEAD_FALL_TICKS 5U
/* 五檔 rise/fall A/B：每段固定秒數輪巡（靠卡測 bad/clip 用） */
#define RFID_PWM_DTI_AB_SCAN_ENABLE 0
#define RFID_PWM_DTI_AB_SCAN_INTERVAL_MS 10000U
/* 1=僅 header 9 個 1 全對才噴 SUCCESS/ID */
#define RFID_SAADC_REQUIRE_HEADER_CLEAN 0
/* 0=width 跟 duty 線性（恢復 duty16 0% clip 甜點）；1=強制偶數 width */
#define RFID_PWM_DTI_WIDTH_EVEN_ALIGN 0
/* EFR-like：duty → width（能量）映射 */
#define RFID_PWM_DTI_WIDTH_REF 24U
#define RFID_PWM_DTI_WIDTH_MAX 28U
/* 舊線性縮放參考（DTI 關閉時使用） */
#define RFID_PWM_CH_A_REF 23U
#define RFID_PWM_CH_B_REF 39U
#define RFID_PWM_DUTY_REFERENCE 10U
#define RFID_PWM_DUTY_MIN 8U
#if RFID_PWM_DTI_ENABLE
#define RFID_PWM_DUTY_MAX RFID_PWM_DTI_DUTY_MAX
#else
#define RFID_PWM_DUTY_MAX 22U
#endif
#define RFID_PWM_CH_A RFID_PWM_CH_A_REF
#define RFID_PWM_CH_B RFID_PWM_CH_B_REF
#define RFID_CLK_P_PIN NRF_GPIO_PIN_MAP(1, 9)
#define RFID_CLK_N_PIN NRF_GPIO_PIN_MAP(1, 10)
/* DK Arduino A0/A1 → AIN0/AIN1（P0.04 / P0.05） */
#define RFID_RX_AIN0_PIN NRF_GPIO_PIN_MAP(0, 4)
#define RFID_RX_AIN1_PIN NRF_GPIO_PIN_MAP(0, 5)
/* SAADC 差動接收（AIN0–AIN1） */
#define RFID_USE_SAADC_RECEIVER 1
#define RFID_SAADC_CHANNEL_INDEX 0U
#define RFID_SAADC_SAMPLE_HZ 62500UL
#define RFID_SAADC_INTERNAL_TIMER_HZ 16000000UL
#define RFID_SAADC_TIMER_CC \
  (RFID_SAADC_INTERNAL_TIMER_HZ / RFID_SAADC_SAMPLE_HZ)
#define RFID_SAADC_ENV_WINDOW_US 64U
#define RFID_SAADC_SAMPLES_PER_WINDOW \
  ((RFID_SAADC_SAMPLE_HZ * RFID_SAADC_ENV_WINDOW_US) / 1000000UL)
#define RFID_SAADC_TARGET_WINDOWS \
  ((RFID_CAPTURE_WINDOW_MS * 1000U) / RFID_SAADC_ENV_WINDOW_US)
#define RFID_SAADC_BUFFER_SIZE 256U
#define RFID_SAADC_BUFFER_COUNT 2U
#define RFID_SAADC_MIN_RANGE_COUNTS 12
#define RFID_SAADC_CARD_MIN_ABS_AVG 200U
#define RFID_SAADC_CLIP_LEVEL 4090
/* 量產：硬體 47k+100pF 線性化後，RSHIFT=0、門檻 50% 正中 */
#define RFID_SAADC_SAMPLE_RSHIFT 0
#define RFID_SAADC_ENV_THRESHOLD_PCT 35U
#define RFID_SAADC_CARD_MIN_ABS_AVG_ACTIVE \
  (RFID_SAADC_CARD_MIN_ABS_AVG >> RFID_SAADC_SAMPLE_RSHIFT)
#define RFID_SAADC_CLIP_LEVEL_ACTIVE \
  (RFID_SAADC_CLIP_LEVEL >> RFID_SAADC_SAMPLE_RSHIFT)
#define RFID_SAADC_ENV_PP_CAP_ACTIVE \
  (RFID_SAADC_ENV_PP_CAP >> RFID_SAADC_SAMPLE_RSHIFT)
#define RFID_SAADC_MAX_CLIP_PERCENT 5U
#define RFID_SAADC_SAT_ENV_ENABLE 1
#define RFID_SAADC_ENV_PP_CAP 2800
#define RFID_SAADC_DECODE_ENABLE 1
#define RFID_SAADC_STRICT_MANCHESTER_ONLY 1
/* strict 下允許 frame_bad≤此值時改走 parity 容錯解碼器（量產抗噪） */
#define RFID_SAADC_STRICT_MAX_BAD_PAIRS 10U
/* 1=日常／量產：僅 [CARD_READY]、啟動摘要、錯誤、SAADC 飽和警示 */
#define RFID_LOG_MINIMAL 1

#if RFID_LOG_MINIMAL
#define RFID_SAADC_LOG_IMPORTANT_ONLY 1
#define RFID_SAADC_VERBOSE_ENV_LOG 0
#define RFID_DECODE_DIAG_LOG 0
#define RFID_MAIN_IDLE_LOG 0
#define RFID_BOOT_SELFTEST_LOG 0
#else
#define RFID_SAADC_LOG_IMPORTANT_ONLY 0
#define RFID_SAADC_VERBOSE_ENV_LOG 1
#define RFID_DECODE_DIAG_LOG 1
#define RFID_MAIN_IDLE_LOG 1
#define RFID_BOOT_SELFTEST_LOG 1
#endif

/* delta/phase 已逼近邊界時，允許少量 parity 誤差先吐碼觀察。 */
#define RFID_EM4100_ROW_PARITY_TOLERANCE 0
#define RFID_EM4100_COL_PARITY_TOLERANCE 0
/* 每段 capture 前 N 樣本學習差動 DC，之後扣除 */
#define RFID_SAADC_DC_BIAS_ENABLE 1
#define RFID_SAADC_DC_BIAS_LEARN_SAMPLES 1000U
/* RF/96 確認；SAADC Manchester 也縮小到 704..832 */
/* 640≈160µs、1800≈450µs：涵蓋 EM4100(256µs) 與 HID Prox FSK(400µs) */
#if ACTIVE_DECODE_MODE == DECODE_MODE_ASK
#define RFID_SAADC_MANCHESTER_HALF_MIN_Q8 1200U
#define RFID_SAADC_MANCHESTER_HALF_MAX_Q8 1600U
#else
#define RFID_SAADC_MANCHESTER_HALF_MIN_Q8 640U
#define RFID_SAADC_MANCHESTER_HALF_MAX_Q8 1800U
#endif

/* 步長 16：FSK/ASK 共用，粗掃過粗會漏 half_q8 */
#define RFID_SAADC_MANCHESTER_HALF_STEP_Q8 16U
#define RFID_SAADC_MANCHESTER_OFFSET_STEP_Q8 16U
#define RFID_SAADC_PHASE_WIDE_STEP_Q8 16U
/* 1=上一輪最佳 (half,phase) ±窗口，避免每輪從 0 掃描造成 bad 飄移 */
#define RFID_SAADC_PHASE_TRACK_ENABLE 1
#define RFID_SAADC_PHASE_TRACK_WINDOW_Q8 64U
/* lock 後僅在 tracked±半徑內掃 half/phase，避免 384 全窗搶走 400 鎖相 */
#define RFID_SAADC_PHASE_LOCK_NARROW 1
/* 新 bad 須優於 lock 至少此值才換相位（減少 528↔64↔616 跳動） */
#define RFID_SAADC_PHASE_LOCK_HYST_BAD 2
/* ≤此值進 narrow；log 常見 bad=15~17，14 過嚴導致永遠 wide */
#define RFID_SAADC_PHASE_LOCK_MAX_BAD 28U
/* tracked_bad 低於此用 guided wide（半窗+全相位），否則全 half */
#define RFID_SAADC_PHASE_GUIDED_BAD_LT 25
/* 每 N 輪 guided 穿插 1 次全 half 掃描 */
#define RFID_SAADC_PHASE_FULL_WIDE_EVERY 4U
/* 連續 hold 且 bad 仍高 → 解除 lock 做一輪全掃 */
#define RFID_SAADC_PHASE_HOLD_RESCAN_GT 6U
/* bad≤18 時加大 refine 半徑（涵蓋 392↔584 帶） */
#define RFID_SAADC_PHASE_REFINE_WIDE_RADIUS_Q8 96U
#define RFID_SAADC_PHASE_REFINE_ENABLE 1
#define RFID_SAADC_PHASE_REFINE_RADIUS_Q8 48U
#define RFID_SAADC_PHASE_REFINE_HALF_RADIUS_Q8 16U
#define RFID_SAADC_PHASE_REFINE_STEP_Q8 2U
/* 0=固定 CH_A_REF，避免 sweep 打斷 phase_lock；收斂後再開 1 微調 */
#define RFID_SAADC_CHA_REF_SWEEP_ENABLE 0
#define RFID_SAADC_CHA_REF_SWEEP_BAD_GT 10
/* 1=UART 輸出 RAW CSV（配合 capture_raw_adc_dump.py）；量產前改回 0 */
#define RFID_SAADC_EXPORT_RAW_WINDOWS 0
#define RFID_SAADC_RAW_DUMP_MAX \
  3U /* 與 capture_raw_adc_dump.py --max-blocks 對齊 */
#define EM_DECODED_BITS_CAP 2048

/* --- 🎯 BSP 核心調優區（景明哥硬核對齊版） --- */

/* 128µs 視窗：RF/64 SHORT=2T=256µs=2×128µs, LONG=4T=512µs=4×128µs，
 * 窗口為 SHORT 的整數倍→無量化模糊；比 256µs 窗更能精確分類。 */
#define EM_ENV_WINDOW_US 64
#define EM_ENV_EDGE_MA_TAPS 1
#define EM_ENV_DUTY_SAMPLE_US 8
#define EM_ENV_BOOT_WINDOWS 32
#define EM_ENV_THRESHOLD_X16 0
#define EM_ENV_DUTY_THRESHOLD_X16 1
#define EM_ENV_USE_DUTY_LEVEL 0
#define EM_ENV_STATE_STABLE_WINDOWS 1
#define EM_ENV_BASELINE_WEIGHT 15
#define EM_ENV_MIN_TICK_US 200
#define EM_ENV_LEVEL_CAP 8192
#define EM_ENV_HALF_MIN_WINDOWS 1
#define EM_ENV_HALF_MAX_WINDOWS 8
#define EM_RAW_CARD_MIN_EDGES \
  4200U /* 實測 SE ~4.5k/200ms；原 16500 永遠 gate skip */
#define EM_CARD_MIN_ENV_TRANSITIONS 350U
#define EM_CARD_MIN_ENV_SHORT 120U
#define EM_LEVEL_MAX_FRAME_BAD_PAIRS 24
#define EM_TICK_HALF_MAX_FRAME_BAD_PAIRS 24
#define EM_NOISY_MAX_FRAME_BAD_PAIRS 48
#define EM_FRAC_HALF_MIN_Q8 896U
#define EM_FRAC_HALF_MAX_Q8 1152U
#define EM_FRAC_HALF_STEP_Q8 32U
#define EM_FRAC_OFFSET_STEP_Q8 64U
/* RF/64(512) + RF/96(768) 全覆蓋；步長 64 共7個值: 448,512,576,640,704,768,832
 */
#define EM_DELTA_HALF_MIN_Q8 896U
#define EM_DELTA_HALF_MAX_Q8 1152U
#define EM_DELTA_HALF_STEP_Q8 64U
#define EM_DELTA_OFFSET_STEP_Q8 64U
#define EM_DELTA_MIN_DIFF 1
#define EM_DELTA_TRACK_OFF_WINDOW 12
#define EM_DELTA_SHIFT_LOG_EVERY 5U
#define EM_DELTA_FORCE_INV_ONE 0
#define EM_ALIGN_TARGET_LOW16 0U
#define EM_NEAR_TARGET_MAX_HAM 4U
#define EM_NEAR_TARGET_MAX_DELTA 32
#define EM_NEAR_TARGET_BRUTE_DELTA 64
/* 診斷模式：凍結 delta 參數，連續輸出 raw40 做統計 */
#define EM_DELTA_DEBUG_RAW_MODE 0
#define EM_DELTA_DEBUG_HALF_Q8 280U
#define EM_DELTA_DEBUG_OFF_Q8 64U
#define EM_ENABLE_EDGE_THRESHOLD_SWEEP 0
#define EM_EDGE_MIN_US 180 /* 濾 <180us 載波半週（示波器 250us/div 密波） */
/* 128µs 視窗：RF/64 SHORT=2窗≈274µs → num_1t=(274+128)/256=1 半位元 ✓
 *              RF/64 LONG =4窗≈548µs → num_1t=(548+128)/256=2 半位元 ✓ */
#define EM_CLOCK_BASE_1T_US 256U

/* 128µs 視窗量化：RF/64 SHORT=2窗≈274µs, LONG=4窗≈548µs */
#define EM_SHORT_MIN 200 /* 含 RF/64 SHORT ~274µs 下界 */
#define EM_SHORT_MAX 380 /* 1T 上界；>380 視為 2T（含 ~396µs 尷尬間距） */
#define EM_LONG_MIN 381
#define EM_LONG_MAX 1100 /* 含 RF/96 LONG 768µs 上界 */
/* env128 展開：1T=2 半位、2T=4 半位（305→2, 610→4；勿用 ÷128 得 5） */
#define EM_ENV_SHORT_HALVES 4U
#define EM_ENV_LONG_HALVES 8U
#define EM_SELFTEST_CODE 0x0123456789ULL
#define RFID_CAPTURE_WINDOW_MS_BALANCED 75U
#define RFID_CAPTURE_WINDOW_MS_STABLE 100U
#define RFID_FSK_CAPTURE_WINDOW_MS 85U /* FSK 保守加速：實測穩定可再略降 */
#if RFID_FSK_FAST_ACTIVE
#define RFID_CAPTURE_WINDOW_MS RFID_FSK_CAPTURE_WINDOW_MS
#else
#define RFID_CAPTURE_WINDOW_MS RFID_CAPTURE_WINDOW_MS_STABLE
#endif
#if RFID_FSK_FAST_ACTIVE
#define RFID_SAADC_CARRIER_SETTLE_MS 50U /* FSK：僅首次擷取前等載波 */
#define RFID_SAADC_POST_ABORT_SLEEP_MS 0U
#define RFID_SAADC_CAPTURE_POLL_MS 1U
#else
#define RFID_SAADC_CARRIER_SETTLE_MS 0U
#define RFID_SAADC_POST_ABORT_SLEEP_MS 2U
#define RFID_SAADC_CAPTURE_POLL_MS 2U
#endif
/* 診斷：驗證 frame 偏移是否只差 ±2 bits。 */
#define RFID_EM4100_SHIFT_SCAN_RADIUS 2
#define EM_SKIP_FIRST_GAP_US 2500
#define EM_MANCHESTER_GAP_HOLD_US 10000
#define EM_MANCHESTER_RESYNC_US 15000

static const nrfx_pwm_t rfid_pwm = NRFX_PWM_INSTANCE(0);
static nrf_pwm_values_individual_t rfid_seq_values[] = {
    {
#if RFID_PWM_DTI_ENABLE
        /* duty=RFID_PWM_DUTY_REFERENCE 的對齊值：對齊既有能量映射 */
        .channel_0 = (uint16_t)(RFID_PWM_CH_A_REF | 0x8000U),
        .channel_1 = (uint16_t)(RFID_PWM_CH_B_REF | 0x8000U),
#else
        .channel_0 = (uint16_t)(RFID_PWM_CH_A | 0x8000U),
        .channel_1 = (uint16_t)(RFID_PWM_CH_B | 0x8000U),
#endif
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

static uint8_t dec_raw_bits[EM_DECODED_BITS_CAP];
static uint8_t dec_pair_bad[EM_DECODED_BITS_CAP];
static uint16_t dec_bad_prefix[EM_DECODED_BITS_CAP + 1];
static uint8_t env_edge_samples[EM_ENV_LEVEL_CAP];
static uint8_t env_level_samples[EM_ENV_LEVEL_CAP];
#if ACTIVE_DECODE_MODE == DECODE_MODE_FSK
static uint8_t fsk_cross_history[EM_ENV_LEVEL_CAP];
#endif
static int16_t adc_window_avg_samples[EM_ENV_LEVEL_CAP];
static int16_t adc_window_pp_samples[EM_ENV_LEVEL_CAP];
static int16_t adc_window_signed_samples[EM_ENV_LEVEL_CAP];
static uint16_t reverse16(uint16_t value) {
  uint16_t reversed = 0U;
  for (int i = 0; i < 16; i++) {
    reversed = (uint16_t)((reversed << 1) | (value & 1U));
    value >>= 1;
  }
  return reversed;
}

typedef enum {
  RFID_SAADC_ENV_MODE_ABS = 0,
  RFID_SAADC_ENV_MODE_PP,
  RFID_SAADC_ENV_MODE_SIGNED,
  RFID_SAADC_ENV_MODE_BLEND,
  RFID_SAADC_ENV_MODE_PP_CAP_BLEND,
} rfid_saadc_env_mode_t;

/* 五檔相位對焦矩陣：{6,8}/{8,6} 補 MOSFET 充放電不對稱；{8,8} 基線；{7,9}/{9,7}
 * 微調 */
#define TEST_RISE_FALL_PAIRS_COUNT 5

typedef struct {
  uint8_t rise;
  uint8_t fall;
} rfid_dti_profile_t;

static const rfid_dti_profile_t dti_profiles[TEST_RISE_FALL_PAIRS_COUNT] = {
    {6U, 8U}, {8U, 6U}, {8U, 8U}, {7U, 9U}, {9U, 7U},
};

static rfid_saadc_env_mode_t rfid_saadc_active_env_mode =
    RFID_SAADC_ENV_MODE_ABS;
static uint16_t rfid_saadc_buffers[RFID_SAADC_BUFFER_COUNT]
                                  [RFID_SAADC_BUFFER_SIZE];
static bool pwm_ready;
static bool carrier_running;
static uint8_t rfid_pwm_active_duty = RFID_PWM_DUTY_REFERENCE;
static uint8_t rfid_pwm_dead_rise_ticks = RFID_PWM_DTI_DEAD_RISE_TICKS;
static uint8_t rfid_pwm_dead_fall_ticks = RFID_PWM_DTI_DEAD_FALL_TICKS;
/* 運行時 CH_A 相位（RFID_PWM_CH_A_REF 為預設）；sweep 時微調載波/取樣對齊 */
static uint8_t rfid_pwm_ch_a_ref_runtime = RFID_PWM_CH_A_REF;
#if RFID_SAADC_PHASE_TRACK_ENABLE
static bool rfid_saadc_phase_valid;
static uint32_t rfid_saadc_tracked_half_q8;
static uint32_t rfid_saadc_tracked_phase_q8;
static int rfid_saadc_tracked_bad = 1000;
static int rfid_saadc_tracked_score = 1000;
static uint8_t rfid_saadc_phase_hold_streak;
static uint8_t rfid_saadc_wide_round;
#endif
static volatile uint32_t rfid_saadc_target_windows = RFID_SAADC_TARGET_WINDOWS;
static bool rfid_saadc_ready;
static bool rfid_saadc_irq_connected;
static volatile bool rfid_saadc_capture_done;
#if RFID_SAADC_CARRIER_SETTLE_MS > 0U
static bool rfid_saadc_carrier_settled;
#endif
static volatile uint32_t rfid_saadc_raw_samples;
static volatile uint32_t rfid_saadc_sample_errors;
static volatile uint32_t rfid_saadc_clip_samples;
static volatile uint32_t rfid_saadc_clip_pos_samples;
static volatile uint32_t rfid_saadc_clip_neg_samples;
static volatile uint32_t rfid_saadc_next_buffer;
static volatile uint32_t rfid_saadc_window_samples;
static volatile uint32_t rfid_saadc_window_abs_sum;
static volatile int16_t rfid_saadc_window_min;
static volatile int16_t rfid_saadc_window_max;
static volatile int32_t rfid_saadc_window_signed_sum;
static volatile uint32_t rfid_saadc_raw_abs_sum;
static volatile int32_t rfid_saadc_raw_sum;
static volatile int16_t rfid_saadc_raw_min;
static volatile int16_t rfid_saadc_raw_max;
static volatile int rfid_saadc_env_level_count;
#if RFID_SAADC_DC_BIAS_ENABLE
static int32_t saadc_dc_bias_accumulator;
static uint32_t saadc_dc_bias_sample_count;
static int16_t saadc_dc_offset;
#endif
#if RFID_USE_SAADC_RECEIVER
static int rfid_saadc_configure(void);
static void rfid_saadc_capture_reset(void);
static int em4095_try_level_decoders(int env_level_count, const char* frac_tag,
                                     int32_t sample_min, int32_t sample_max);
static int rfid_saadc_decode_from_envelope(int env_level_count,
                                           const char* tag);
#if RFID_SAADC_EXPORT_RAW_WINDOWS
static void rfid_saadc_dump_window_csv(int env_level_count);
#endif
#endif
static int start_carrier_with_pwm(void);

static void carrier_gpio_high_drive_configure(void) {
  nrf_gpio_cfg(RFID_CLK_P_PIN, NRF_GPIO_PIN_DIR_OUTPUT,
               NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,
               NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_cfg(RFID_CLK_N_PIN, NRF_GPIO_PIN_DIR_OUTPUT,
               NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,
               NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);
}

/* PWM 停止後強制把輸出鎖到固定電平，避免 nrfx_pwm_stop() 瞬間殘留 */
static void carrier_gpio_force_off(void) {
  nrf_gpio_cfg(RFID_CLK_P_PIN, NRF_GPIO_PIN_DIR_OUTPUT,
               NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,
               NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_pin_clear(RFID_CLK_P_PIN);

  nrf_gpio_cfg(RFID_CLK_N_PIN, NRF_GPIO_PIN_DIR_OUTPUT,
               NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,
               NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_pin_clear(RFID_CLK_N_PIN);
}

#if RFID_COMP_RX_GPIO_PINCNF_ENABLE
static const char* rfid_rx_gpio_drive_name(nrf_gpio_pin_drive_t drive) {
#if defined(GPIO_PIN_CNF_DRIVE_E0E1)
  if (drive == NRF_GPIO_PIN_E0E1) {
    return "E0E1";
  }
#endif
  if (drive == NRF_GPIO_PIN_S0S1) {
    return "S0S1";
  }
  return "other";
}

static void rfid_rx_ain_gpio_configure(void) {
  const nrf_gpio_pin_drive_t drive = RFID_COMP_RX_GPIO_DRIVE;

  /*
   * 類比輸入：數位輸入緩衝關閉、無上下拉；DRIVE 依 PIN_CNF 改變 pad
   * 特性（實測用）。 nRF5340 PS 將 E0E1 標為 extra-high
   * drive，與「低斜率」命名不同，以 comp_edges 為準。
   */
  nrf_gpio_cfg(RFID_RX_AIN0_PIN, NRF_GPIO_PIN_DIR_INPUT,
               NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, drive,
               NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_cfg(RFID_RX_AIN1_PIN, NRF_GPIO_PIN_DIR_INPUT,
               NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, drive,
               NRF_GPIO_PIN_NOSENSE);
}
#endif

static int rfid_pwm_apply_duty(uint8_t duty) {
  uint16_t ch_a;
  uint16_t ch_b;
  uint32_t width;

  if (duty < RFID_PWM_DUTY_MIN) {
    duty = RFID_PWM_DUTY_MIN;
  }
#if RFID_PWM_DTI_ENABLE
  else if (duty > RFID_PWM_DTI_DUTY_MAX) {
    duty = RFID_PWM_DTI_DUTY_MAX;
  }
  /*
   * EFR-like 單旋鈕：duty → 互補間距 width，保留 CH_A_REF 相位偏置。
   * 固定 dead-time ticks；width 線性跟 duty，不偶數修齊。
   */
  width = duty;
#if RFID_PWM_DTI_WIDTH_EVEN_ALIGN
  width = (width + 1U) & ~1U;
#endif
  if (width > (uint32_t)RFID_PWM_DTI_DUTY_MAX) {
    width = (uint32_t)RFID_PWM_DTI_DUTY_MAX;
  }
  if (width < (uint32_t)RFID_PWM_DTI_MIN_GAP) {
    width = (uint32_t)RFID_PWM_DTI_MIN_GAP;
  }

  /* Center-aligned 互補：以 TOP/2 為中心對稱展開 half_width，
   * 再各自扣除/加上 dead time，確保雙臂 ON 時間對稱且不隨 duty 偏移。 */
  {
    uint32_t half_width = (width + 1U) >> 1U;
    uint32_t center = (uint32_t)RFID_PWM_DTI_CENTER;
    ch_a = (uint16_t)(center > half_width ? center - half_width : 0U);
    ch_b = (uint16_t)(center + half_width);
  }

  if (ch_a > rfid_pwm_dead_rise_ticks) {
    ch_a = (uint16_t)(ch_a - rfid_pwm_dead_rise_ticks);
  }
  ch_b = (uint16_t)(ch_b + rfid_pwm_dead_fall_ticks);

  if (ch_a < RFID_PWM_DTI_EDGE_GUARD) {
    ch_a = RFID_PWM_DTI_EDGE_GUARD;
  }
  if (ch_b > (RFID_PWM_TOP - RFID_PWM_DTI_EDGE_GUARD)) {
    ch_b = (uint16_t)(RFID_PWM_TOP - RFID_PWM_DTI_EDGE_GUARD);
  }
  /* 剛性保護：dead-time + width 邊界下仍保留 MIN_GAP，必要時整體平移 */
  if (ch_b <= ch_a + RFID_PWM_DTI_MIN_GAP) {
    ch_b = (uint16_t)(ch_a + RFID_PWM_DTI_MIN_GAP);
  }
  if (ch_b > (RFID_PWM_TOP - RFID_PWM_DTI_EDGE_GUARD)) {
    ch_b = (uint16_t)(RFID_PWM_TOP - RFID_PWM_DTI_EDGE_GUARD);
    ch_a = (uint16_t)(ch_b - RFID_PWM_DTI_MIN_GAP);
    if (ch_a < RFID_PWM_DTI_EDGE_GUARD) {
      ch_a = RFID_PWM_DTI_EDGE_GUARD;
      ch_b = (uint16_t)(ch_a + RFID_PWM_DTI_MIN_GAP);
    }
  }
#else
  else if (duty > RFID_PWM_DUTY_MAX) {
    duty = (uint8_t)RFID_PWM_DUTY_MAX;
  }
  ch_a = (uint16_t)((rfid_pwm_ch_a_ref_runtime * (uint32_t)duty +
                     RFID_PWM_DUTY_REFERENCE / 2U) /
                    RFID_PWM_DUTY_REFERENCE);
  ch_b = (uint16_t)((RFID_PWM_CH_B_REF * (uint32_t)duty +
                     RFID_PWM_DUTY_REFERENCE / 2U) /
                    RFID_PWM_DUTY_REFERENCE);
  if (ch_a < 4U) {
    ch_a = 4U;
  }
  if (ch_b <= ch_a + 2U) {
    ch_b = ch_a + 2U;
  }
  if (ch_b >= RFID_PWM_TOP) {
    ch_b = (uint16_t)(RFID_PWM_TOP - 1U);
  }
#endif

  rfid_seq_values[0].channel_0 = ch_a | 0x8000U;
  rfid_seq_values[0].channel_1 = ch_b | 0x8000U;
  rfid_pwm_active_duty = duty;

  if (carrier_running) {
    (void)nrfx_pwm_stop(&rfid_pwm, true);
    carrier_gpio_force_off();
    carrier_running = false;
  }

  return start_carrier_with_pwm();
}

static void rfid_pwm_apply_dti_profile(size_t index) {
  if (index >= TEST_RISE_FALL_PAIRS_COUNT) {
    index = 0U;
  }
  rfid_pwm_dead_rise_ticks = dti_profiles[index].rise;
  rfid_pwm_dead_fall_ticks = dti_profiles[index].fall;
}

static int start_carrier_with_pwm(void) {
  nrfx_err_t err;

  if (!pwm_ready) {
    nrfx_pwm_config_t const config = {
        .output_pins =
            {
                RFID_CLK_P_PIN, /* P1.09 */
                RFID_CLK_N_PIN, /* P1.10 */
                NRF_PWM_PIN_NOT_CONNECTED,
                NRF_PWM_PIN_NOT_CONNECTED,
            },
        .pin_inverted = {false, false, false, false},
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
    carrier_gpio_high_drive_configure();
    pwm_ready = true;
  }

  if (!carrier_running) {
    (void)nrfx_pwm_simple_playback(&rfid_pwm, &rfid_seq, 1, NRFX_PWM_FLAG_LOOP);
    carrier_running = true;
  }

  return 0;
}

static void __attribute__((unused)) stop_carrier_with_pwm(void) {
  if (!pwm_ready || !carrier_running) return;
  (void)nrfx_pwm_stop(&rfid_pwm, true);
  carrier_gpio_force_off();
  carrier_running = false;
}

void invert_bits(uint8_t* bits, int len) {
  for (int i = 0; i < len; i++) {
    bits[i] = !bits[i];
  }
}

int EM4100_Full_Check(uint8_t* bits) {
  for (int j = 0; j < 9; j++)
    if (bits[j] != 1) return 0;
  if (bits[63] != 0) return 0;

  uint8_t col_parity[4] = {0};
  int row_err = 0;
  int col_err = 0;
  for (int row = 0; row < 10; row++) {
    int base = 9 + row * 5;
    int row_sum = 0;
    for (int col = 0; col < 4; col++) {
      uint8_t val = bits[base + col];
      row_sum += val;
      col_parity[col] += val;
    }
    if ((row_sum + bits[base + 4]) % 2 != 0) row_err++;
  }
  for (int col = 0; col < 4; col++) {
    if ((col_parity[col] + bits[59 + col]) % 2 != 0) col_err++;
  }
  if (row_err > RFID_EM4100_ROW_PARITY_TOLERANCE) return 0;
  if (col_err > RFID_EM4100_COL_PARITY_TOLERANCE) return 0;
  return 1;
}

static int em4100_valid_at(const uint8_t* bits, int len, int off) {
  if (off < 0 || off + 64 > len) return 0;
  if (EM4100_Full_Check((uint8_t*)&bits[off])) return 1;
  for (int delta = 1; delta <= RFID_EM4100_SHIFT_SCAN_RADIUS; delta++) {
    int left = off - delta;
    int right = off + delta;
    if (left >= 0 && left + 64 <= len &&
        EM4100_Full_Check((uint8_t*)&bits[left])) {
      return 1;
    }
    if (right >= 0 && right + 64 <= len &&
        EM4100_Full_Check((uint8_t*)&bits[right])) {
      return 1;
    }
  }
  return 0;
}

uint64_t decode_card_code(uint8_t* bits) {
  uint64_t code = 0;
  for (int j = 0; j < 10; j++) {
    unsigned int digit = (bits[9 + 5 * j] << 3) | (bits[9 + 5 * j + 1] << 2) |
                         (bits[9 + 5 * j + 2] << 1) |
                         (bits[9 + 5 * j + 3] << 0);
    code |= ((uint64_t)digit << (4 * (9 - j)));
  }
  return code;
}

/* 🎯 HID Prox
 * (Wiegand-26)：依靠零容錯曼徹斯特解碼與幀內雙重驗證，徹底消滅幽靈！ */
static bool decode_hid_prox(const uint8_t* bits, int len, uint64_t* out_raw44,
                            uint32_t* facility_code, uint32_t* card_number) {
  int zero_count = 0;
  uint32_t w26_candidates[6] = {0};  // 儲存同一個陣列中找到的候選封包
  int candidate_count = 0;
  int total_bad = 0;

  if (facility_code == NULL || card_number == NULL || len < 64) {
    return false;
  }

  /* 🛡️ 第一道防線：全域相位鎖定。
   * 如果這個相位的總 Manchester 錯誤率超過 12.5%，直接踢掉！ */
  for (int i = 0; i < len; i++) {
    if (dec_pair_bad[i]) total_bad++;
  }
  if (total_bad > (len / 8)) {
    return false;
  }

  /* 1. 掃描整個波形陣列，抓出 Wiegand 26 封包 */
  for (int i = 0; i < len; i++) {
    if (bits[i] == 0) {
      zero_count++;
    } else {
      /* 🎯 放寬 Preamble：真實波形大約只有 6 個 0，我們設定 >= 5 */
      if (zero_count >= 5) {
        const int start_idx = i;

        if (start_idx + 26 < len) {
          uint32_t w26 = 0;
          int bad_manchester = 0;

          /* 提取 26 位元，並統計該封包內的 Manchester 錯誤數 */
          for (int j = 1; j <= 26; j++) {
            w26 = (w26 << 1) | (bits[start_idx + j] ? 1U : 0U);
            if (dec_pair_bad[start_idx + j]) {
              bad_manchester++;
            }
          }

          const uint8_t p_even = (uint8_t)((w26 >> 25) & 1U);
          const uint8_t p_odd = (uint8_t)(w26 & 1U);
          const uint32_t cn = (uint32_t)((w26 >> 1) & 0xFFFFU);

          uint8_t calc_even = 0U;
          for (int k = 13; k <= 24; k++)
            calc_even ^= (uint8_t)((w26 >> k) & 1U);

          uint8_t calc_odd = 1U;
          for (int k = 1; k <= 12; k++) calc_odd ^= (uint8_t)((w26 >> k) & 1U);

          /* 🛡️ 核心防線：Parity 正確 + 曼徹斯特錯誤必須為 0！
           * 因為現在 FSK 6 視窗很完美，真正的封包 bad 一定是 0。
           * 我們拿掉了不準確的尾部檢測，靠這條嚴格規則來過濾雜訊。 */
          if (calc_even == p_even && calc_odd == p_odd && cn != 0 &&
              bad_manchester == 0) {
            if (candidate_count < (int)ARRAY_SIZE(w26_candidates)) {
              w26_candidates[candidate_count++] = w26;
            }
            i = start_idx + 26; /* 推進指標，跳過已解碼資料 */
          }
        }
      }
      zero_count = 0;
    }
  }

  /* 🛡️ 最終防線：閃電雙重防偽。
   * 同一個陣列內，一模一樣的卡號必須重複出現 >= 2 次，100% 是真卡！ */
  for (int c1 = 0; c1 < candidate_count; c1++) {
    int match = 1;
    for (int c2 = c1 + 1; c2 < candidate_count; c2++) {
      if (w26_candidates[c1] == w26_candidates[c2]) {
        match++;
      }
    }

    if (match >= 2) {
      uint32_t final_w26 = w26_candidates[c1];
      if (out_raw44 != NULL) *out_raw44 = final_w26;
      *facility_code = (final_w26 >> 17) & 0xFFU;
      *card_number = (final_w26 >> 1) & 0xFFFFU;
      return true;
    }
  }

  return false;
}

static uint8_t env_majority_level(const uint8_t* levels, int start, int width);

static void em4100_build_frame(uint64_t code, uint8_t* bits) {
  uint8_t col_sum[4] = {0};

  memset(bits, 0, 64);

  for (int i = 0; i < 9; i++) {
    bits[i] = 1;
  }

  for (int row = 0; row < 10; row++) {
    int base = 9 + row * 5;
    uint8_t digit = (code >> (4 * (9 - row))) & 0x0F;
    uint8_t row_sum = 0;

    for (int col = 0; col < 4; col++) {
      uint8_t val = (digit >> (3 - col)) & 1U;
      bits[base + col] = val;
      row_sum += val;
      col_sum[col] += val;
    }
    bits[base + 4] = row_sum & 1U;
  }

  for (int col = 0; col < 4; col++) {
    bits[59 + col] = col_sum[col] & 1U;
  }
  bits[63] = 0;
}

static void em4100_decode_self_test(void) {
  uint8_t bits[64];

  em4100_build_frame(EM_SELFTEST_CODE, bits);

  int valid = EM4100_Full_Check(bits);
  uint64_t decoded = decode_card_code(bits);
#if RFID_BOOT_SELFTEST_LOG
  LOG_WRN("[SELFTEST-frame] valid=%d raw40=%010llX", valid,
          (unsigned long long)decoded);
#endif
  if (!valid || decoded != EM_SELFTEST_CODE) {
    LOG_ERR("[SELFTEST-frame] failed");
  }

  uint8_t levels[64 * 4];
  const int half_windows = 2;
  int bad_pairs = 0;

  for (int bit = 0; bit < 64; bit++) {
    uint8_t expected_bit = bits[bit];
    int base = bit * half_windows * 2;

    for (int i = 0; i < half_windows; i++) {
      levels[base + i] = expected_bit ? 0U : 1U;
      levels[base + half_windows + i] = expected_bit;
    }
  }

  memset(dec_raw_bits, 0, EM_DECODED_BITS_CAP);
  for (int bit = 0; bit < 64; bit++) {
    int base = bit * half_windows * 2;
    uint8_t first = env_majority_level(levels, base, half_windows);
    uint8_t second =
        env_majority_level(levels, base + half_windows, half_windows);

    if (first == second) bad_pairs++;
    dec_raw_bits[bit] = second;
  }

  valid = EM4100_Full_Check(dec_raw_bits);
  decoded = decode_card_code(dec_raw_bits);
#if RFID_BOOT_SELFTEST_LOG
  LOG_WRN("[SELFTEST-manchester] valid=%d bad=%d raw40=%010llX", valid,
          bad_pairs, (unsigned long long)decoded);
#endif
  if (!valid || bad_pairs != 0 || decoded != EM_SELFTEST_CODE) {
    LOG_ERR("[SELFTEST-manchester] failed");
  }
}

#if EM_ALIGN_TARGET_LOW16 != 0U
static int em4100_low16_delta(uint16_t low16);
#endif

static bool em4100_candidate_from_parity(const uint8_t* bits, int off,
                                         uint64_t* out_code,
                                         const char** reason) {
  int row_bad = -1;
  int row_bad_count = 0;
  int col_bad = -1;
  int col_bad_count = 0;
  int row_bad_idx[10] = {0};
  int col_bad_idx[4] = {0};
  uint8_t col_parity[4] = {0};

  int hdr_err = 0;
  for (int j = 0; j < 9; j++) {
    if (bits[off + j] != 1) hdr_err++;
  }
  if (hdr_err > 1) return false;
  if (bits[off + 63] != 0) return false;

  for (int row = 0; row < 10; row++) {
    int base = off + 9 + row * 5;
    int row_sum = 0;
    for (int col = 0; col < 4; col++) {
      uint8_t val = bits[base + col];
      row_sum += val;
      col_parity[col] += val;
    }
    if ((row_sum + bits[base + 4]) % 2 != 0) {
      row_bad = row;
      if (row_bad_count < (int)ARRAY_SIZE(row_bad_idx)) {
        row_bad_idx[row_bad_count] = row;
      }
      row_bad_count++;
    }
  }

  for (int col = 0; col < 4; col++) {
    if ((col_parity[col] + bits[off + 59 + col]) % 2 != 0) {
      col_bad = col;
      if (col_bad_count < (int)ARRAY_SIZE(col_bad_idx)) {
        col_bad_idx[col_bad_count] = col;
      }
      col_bad_count++;
    }
  }

  uint8_t corrected[64];
  memcpy(corrected, &bits[off], sizeof(corrected));

  if (row_bad_count == 0 && col_bad_count == 0) {
    *reason = "clean";
  } else if (row_bad_count > 0 && row_bad_count <= 3 && col_bad_count == 0) {
    for (int i = 0; i < row_bad_count; i++) {
      corrected[9 + row_bad_idx[i] * 5 + 4] ^= 1U;
    }
    if (!EM4100_Full_Check(corrected)) return false;
    *reason = "rowparity";
  } else if (row_bad_count == 0 && col_bad_count > 0 && col_bad_count <= 2) {
    for (int i = 0; i < col_bad_count; i++) {
      corrected[59 + col_bad_idx[i]] ^= 1U;
    }
    if (!EM4100_Full_Check(corrected)) return false;
    *reason = "colparity";
  } else if (row_bad_count == 1 && col_bad_count == 1) {
    corrected[9 + row_bad * 5 + col_bad] ^= 1U;
    if (!EM4100_Full_Check(corrected)) return false;
    *reason = "data1";
  } else if (row_bad_count == 2 && col_bad_count == 2) {
    for (int swap = 0; swap <= 1; swap++) {
      memcpy(corrected, &bits[off], sizeof(corrected));
      corrected[9 + row_bad_idx[0] * 5 + col_bad_idx[swap]] ^= 1U;
      corrected[9 + row_bad_idx[1] * 5 + col_bad_idx[1 - swap]] ^= 1U;
      if (EM4100_Full_Check(corrected)) {
        *out_code = decode_card_code(corrected);
        *reason = "data2";
        return true;
      }
    }
    return false;
  } else {
    /* A lone row/column parity error cannot locate a data bit reliably. */
    return false;
  }

  *out_code = decode_card_code(corrected);
  return true;
}

static uint8_t env_majority_level(const uint8_t* levels, int start, int width) {
  int sum = 0;
  for (int i = 0; i < width; i++) {
    sum += levels[start + i] ? 1 : 0;
  }
  return (sum * 2 >= width) ? 1 : 0;
}

static uint8_t env_sample_level_q8(const uint8_t* levels, int len,
                                   uint32_t pos_q8) {
  int idx = (int)((pos_q8 + 128U) >> 8);

  if (idx < 0) return levels[0];
  if (idx >= len) return levels[len - 1];
  return levels[idx];
}

static uint8_t env_majority_level_q8(const uint8_t* levels, int len,
                                     uint32_t start_q8, uint32_t width_q8) {
  uint32_t quarter_q8 = width_q8 >> 2;
  uint32_t center_q8 = width_q8 >> 1;
  int sum = 0;

  sum += env_sample_level_q8(levels, len, start_q8 + quarter_q8) ? 1 : 0;
  sum += env_sample_level_q8(levels, len, start_q8 + center_q8) ? 1 : 0;
  sum += env_sample_level_q8(levels, len, start_q8 + width_q8 - quarter_q8) ? 1
                                                                            : 0;
  return sum >= 2 ? 1U : 0U;
}

static int em4100_frame_error_score(const uint8_t* bits, int off,
                                    int* header_err, int* row_err, int* col_err,
                                    int* stop_err) {
  uint8_t col_parity[4] = {0};

  *header_err = 0;
  *row_err = 0;
  *col_err = 0;
  *stop_err = bits[off + 63] != 0 ? 1 : 0;

  for (int j = 0; j < 9; j++) {
    if (bits[off + j] != 1) (*header_err)++;
  }

  for (int row = 0; row < 10; row++) {
    int base = off + 9 + row * 5;
    int row_sum = 0;
    for (int col = 0; col < 4; col++) {
      uint8_t val = bits[base + col];
      row_sum += val;
      col_parity[col] += val;
    }
    if ((row_sum + bits[base + 4]) % 2 != 0) (*row_err)++;
  }

  for (int col = 0; col < 4; col++) {
    if ((col_parity[col] + bits[off + 59 + col]) % 2 != 0) (*col_err)++;
  }

  return (*header_err * 2) + *row_err + *col_err + (*stop_err * 2);
}

static bool saadc_pair_parity_rescue_at(uint8_t* bits, const uint8_t* pair_bad,
                                        int bit_len, int frame_off,
                                        int max_bad_pairs, uint64_t* out_code,
                                        const char** out_reason) {
  int bad_pos[12];
  int n = 0;

  for (int i = 0; i < 64; i++) {
    const int idx = frame_off + i;

    if (idx >= bit_len) {
      break;
    }
    if (pair_bad[idx]) {
      if (n < (int)ARRAY_SIZE(bad_pos)) {
        bad_pos[n++] = idx;
      }
    }
  }
  if (n > max_bad_pairs) {
    return false;
  }

  const int masks = 1 << n;

  for (int mask = 0; mask < masks; mask++) {
    for (int b = 0; b < n; b++) {
      if (mask & (1 << b)) {
        bits[bad_pos[b]] ^= 1U;
      }
    }

    const char* reason = NULL;

    if (em4100_candidate_from_parity(bits, frame_off, out_code, &reason) &&
        reason != NULL) {
      *out_reason = reason;
      for (int b = 0; b < n; b++) {
        if (mask & (1 << b)) {
          bits[bad_pos[b]] ^= 1U;
        }
      }
      return true;
    }

    for (int b = 0; b < n; b++) {
      if (mask & (1 << b)) {
        bits[bad_pos[b]] ^= 1U;
      }
    }
  }

  return false;
}

#if ACTIVE_DECODE_MODE == DECODE_MODE_FSK
/* printk 直出 UART，避免 LOG 緩衝導致 LED 已亮、串口延遲 */
static void rfid_hid_match_report(uint32_t hid_fc, uint32_t hid_cn,
                                  uint64_t hid_raw44, int inv, uint32_t half_q8,
                                  uint32_t phase_q8) {
  const bool first_hid_lock = !rfid_saadc_hid_phase_hold;
  const int64_t now_ms = k_uptime_get();
  const int64_t scan_ms = now_ms - rfid_timing_decode_start_ms;
  const bool log_this_match =
      first_hid_lock || rfid_fsk_log_next_match || (scan_ms >= 500);

  rfid_saadc_hid_phase_hold = true;
  rfid_saadc_phase_valid = true;
  rfid_saadc_tracked_half_q8 = half_q8;
  rfid_saadc_tracked_phase_q8 = phase_q8;
  rfid_saadc_tracked_bad = 0;
  rfid_saadc_tracked_score = 0;

  /* 每次 HID 成功都印卡號；TIMING 僅首次／bootstrap／慢掃（避免刷屏） */
  if (log_this_match && hid_cn == RFID_TIMING_TARGET_CN) {
    printk(
        "[TIMING-CN%u] total_ms=%lld capture_ms=%u scan_ms=%lld "
        "half_q8=%u phase_q8=%u inv=%d%s\n",
        (unsigned)RFID_TIMING_TARGET_CN,
        (long long)(now_ms - rfid_timing_round_start_ms),
        rfid_timing_capture_us / 1000U, (long long)scan_ms, (unsigned)half_q8,
        (unsigned)phase_q8, inv,
        first_hid_lock ? " (first_lock)"
                       : (rfid_fsk_log_next_match ? " (bootstrap)" : ""));
    rfid_fsk_log_next_match = false;
  }
  printk(
      "!!! [HID PROX MATCH] FC: %u, CN: %u, w26_raw: %08llX (inv=%d, "
      "half=%u)\n",
      hid_fc, hid_cn, (unsigned long long)hid_raw44, inv, (unsigned)half_q8);

  gpio_pin_set_dt(&led, (hid_cn == 20495U) ? 1 : 0);
}
#endif

/* 單一 (half_q8, phase_q8) 曼徹斯特解碼；成功回傳 frame offset，否則 -1 */
static int saadc_pair_try_phase_config(
    int level_len, uint32_t level_limit_q8, uint32_t half_q8, uint32_t phase_q8,
    int* best_score, int* best_bad, uint32_t* best_half_q8,
    uint32_t* best_phase_q8, int* best_off, int* best_inv, int* best_header,
    int* best_row, int* best_col, int* best_stop, int* best_bits,
    uint64_t* best_code, uint64_t* out_code, int* out_bits,
    int* out_half_windows, int* out_offset, bool* out_inverted,
    int* out_bad_pairs) {
  const uint32_t bit_q8 = half_q8 * 2U;
  int bit_len = 0;

  memset(dec_raw_bits, 0, EM_DECODED_BITS_CAP);
  memset(dec_pair_bad, 0, EM_DECODED_BITS_CAP);

  for (uint32_t pos_q8 = phase_q8;
       pos_q8 + bit_q8 <= level_limit_q8 && bit_len < EM_DECODED_BITS_CAP;
       pos_q8 += bit_q8) {
    uint8_t first =
        env_majority_level_q8(env_level_samples, level_len, pos_q8, half_q8);
    uint8_t second = env_majority_level_q8(env_level_samples, level_len,
                                           pos_q8 + half_q8, half_q8);

    if (first == second) {
      dec_pair_bad[bit_len] = 1U;
    }
    dec_raw_bits[bit_len++] = second;
  }

  if (bit_len < 64) {
    return -1;
  }

  dec_bad_prefix[0] = 0U;
  for (int i = 0; i < bit_len; i++) {
    dec_bad_prefix[i + 1] =
        (uint16_t)(dec_bad_prefix[i] + (dec_pair_bad[i] ? 1U : 0U));
  }

  for (int inv = 0; inv <= 1; inv++) {
    if (inv) {
      invert_bits(dec_raw_bits, bit_len);
    }

    /* 👇 優先執行 HID Prox 嚴格校驗（具備幀內雙重校驗，保證 0 誤判） */
    {
      uint64_t hid_raw44 = 0ULL;
      uint32_t hid_fc = 0;
      uint32_t hid_cn = 0;

      if (decode_hid_prox(dec_raw_bits, bit_len, &hid_raw44, &hid_fc,
                          &hid_cn)) {
        rfid_hid_match_report(hid_fc, hid_cn, hid_raw44, inv, half_q8,
                              phase_q8);

        *out_code = ((uint64_t)hid_fc << 16) | (uint64_t)hid_cn;
        *out_bits = bit_len;
        *out_half_windows = (int)half_q8;
        *out_offset = (int)phase_q8;
        *out_inverted = inv != 0;
        *out_bad_pairs = 0;

        if (inv) {
          invert_bits(dec_raw_bits, bit_len);
        }
        return 0; /* 🚀 解除封印！不用等 5
                     次了，只要幀內驗證通過，瞬間回傳成功！ */
      }
    }

    for (int off = 0; off <= bit_len - 64; off++) {
      int header_err;
      int row_err;
      int col_err;
      int stop_err;
      int frame_bad = dec_bad_prefix[off + 64] - dec_bad_prefix[off];

      if (frame_bad > 0) {
        if (frame_bad <= *best_bad) {
          int bad_header;
          int bad_row;
          int bad_col;
          int bad_stop;
          int bad_score = em4100_frame_error_score(
              dec_raw_bits, off, &bad_header, &bad_row, &bad_col, &bad_stop);

          if (frame_bad < *best_bad ||
              (frame_bad == *best_bad && bad_score < *best_score)) {
            *best_bad = frame_bad;
            *best_half_q8 = half_q8;
            *best_phase_q8 = phase_q8;
            *best_off = off;
            *best_inv = inv;
            *best_header = bad_header;
            *best_row = bad_row;
            *best_col = bad_col;
            *best_stop = bad_stop;
            *best_bits = bit_len;
            *best_code = decode_card_code((uint8_t*)&dec_raw_bits[off]);
            *best_score = bad_score;
          }
        }
        continue;
      }

      int score = em4100_frame_error_score(dec_raw_bits, off, &header_err,
                                           &row_err, &col_err, &stop_err);
      uint64_t code = decode_card_code((uint8_t*)&dec_raw_bits[off]);

      if (score < *best_score ||
          (score == *best_score && frame_bad < *best_bad)) {
        *best_score = score;
        *best_bad = frame_bad;
        *best_half_q8 = half_q8;
        *best_phase_q8 = phase_q8;
        *best_off = off;
        *best_inv = inv;
        *best_header = header_err;
        *best_row = row_err;
        *best_col = col_err;
        *best_stop = stop_err;
        *best_bits = bit_len;
        *best_code = code;
      }

      if (frame_bad == 0) {
        uint64_t parity_code;
        const char* parity_reason = NULL;

        if (em4100_candidate_from_parity(dec_raw_bits, off, &parity_code,
                                         &parity_reason)) {
#if RFID_SAADC_REQUIRE_HEADER_CLEAN
          if (!em4100_header_clean(dec_raw_bits, off)) {
            continue;
          }
#endif
          *out_code = parity_code;
          *out_bits = bit_len;
          *out_half_windows = (int)half_q8;
          *out_offset = (int)phase_q8;
          *out_inverted = inv != 0;
          *out_bad_pairs = frame_bad;
          if (inv) {
            invert_bits(dec_raw_bits, bit_len);
          }
#if !RFID_LOG_MINIMAL
          LOG_WRN(
              "[SUCCESS-SAADC-01-10 half_q8=%d phase_q8=%d inv=%d bad=%d "
              "parity=%s header=0]",
              (int)half_q8, (int)phase_q8, inv, frame_bad,
              parity_reason != NULL ? parity_reason : "?");
          LOG_WRN("[SUCCESS-SAADC-01-10] ID: %010llX",
                  (unsigned long long)parity_code);
#endif
          return off;
        }
        if (em4100_valid_at(dec_raw_bits, bit_len, off)) {
#if RFID_SAADC_REQUIRE_HEADER_CLEAN
          if (!em4100_header_clean(dec_raw_bits, off)) {
            continue;
          }
#endif
          *out_code = code;
          *out_bits = bit_len;
          *out_half_windows = (int)half_q8;
          *out_offset = (int)phase_q8;
          *out_inverted = inv != 0;
          *out_bad_pairs = frame_bad;
          if (inv) {
            invert_bits(dec_raw_bits, bit_len);
          }
#if !RFID_LOG_MINIMAL
          LOG_WRN(
              "[SUCCESS-SAADC-01-10 half_q8=%d phase_q8=%d inv=%d bad=%d "
              "parity=clean header=0]",
              (int)half_q8, (int)phase_q8, inv, frame_bad);
          LOG_WRN("[SUCCESS-SAADC-01-10] ID: %010llX",
                  (unsigned long long)code);
#endif
          return off;
        }
      }
    }

    if (inv) {
      invert_bits(dec_raw_bits, bit_len);
    }
  }

  return -1;
}

static int try_em4100_saadc_pair_decode(int level_len, uint64_t* out_code,
                                        int* out_bits, int* out_half_windows,
                                        int* out_offset, bool* out_inverted,
                                        int* out_bad_pairs, bool log_best) {
  /* 供 strict fail 時做「穩定候選」收斂展示用（不代表 strict success）。 */
  extern uint64_t rfid_saadc_pair_last_best_code;
  extern int rfid_saadc_pair_last_best_bad;
  extern int rfid_saadc_pair_last_best_score;
  int best_score = 1000;
  int best_bad = 1000;
  uint32_t best_half_q8 = 0;
  uint32_t best_phase_q8 = 0;
  int best_off = -1;
  int best_inv = 0;
  int best_header = 0;
  int best_row = 0;
  int best_col = 0;
  int best_stop = 0;
  int best_bits = 0;
  uint64_t best_code = 0;

  if (level_len < 128) return -1;

  const uint32_t level_limit_q8 = (uint32_t)(level_len - 1) << 8;
  uint32_t half_lo = RFID_SAADC_MANCHESTER_HALF_MIN_Q8;
  uint32_t half_hi = RFID_SAADC_MANCHESTER_HALF_MAX_Q8;
#if ACTIVE_DECODE_MODE == DECODE_MODE_FSK
  const bool narrow_scan = rfid_saadc_hid_phase_hold && rfid_saadc_phase_valid;
#else
  const bool narrow_scan = rfid_saadc_phase_valid;
#endif
  bool guided_wide = false;
  const uint32_t phase_step_q8 = narrow_scan
                                     ? RFID_SAADC_MANCHESTER_OFFSET_STEP_Q8
                                     : RFID_SAADC_PHASE_WIDE_STEP_Q8;

#if ACTIVE_DECODE_MODE == DECODE_MODE_FSK
  if (rfid_saadc_hid_phase_hold && rfid_saadc_phase_valid) {
    int hit = saadc_pair_try_phase_config(
        level_len, level_limit_q8, rfid_saadc_tracked_half_q8,
        rfid_saadc_tracked_phase_q8, &best_score, &best_bad, &best_half_q8,
        &best_phase_q8, &best_off, &best_inv, &best_header, &best_row,
        &best_col, &best_stop, &best_bits, &best_code, out_code, out_bits,
        out_half_windows, out_offset, out_inverted, out_bad_pairs);
    if (hit >= 0) {
      return hit;
    }
  }
#endif

#if RFID_SAADC_PHASE_TRACK_ENABLE && RFID_SAADC_PHASE_LOCK_NARROW
  if (rfid_saadc_phase_valid) {
    half_lo =
        (rfid_saadc_tracked_half_q8 > RFID_SAADC_PHASE_REFINE_HALF_RADIUS_Q8)
            ? (rfid_saadc_tracked_half_q8 -
               RFID_SAADC_PHASE_REFINE_HALF_RADIUS_Q8)
            : RFID_SAADC_MANCHESTER_HALF_MIN_Q8;
    half_hi =
        rfid_saadc_tracked_half_q8 + RFID_SAADC_PHASE_REFINE_HALF_RADIUS_Q8;
    if (half_hi > RFID_SAADC_MANCHESTER_HALF_MAX_Q8) {
      half_hi = RFID_SAADC_MANCHESTER_HALF_MAX_Q8;
    }
  } else if (rfid_saadc_tracked_bad < RFID_SAADC_PHASE_GUIDED_BAD_LT) {
    rfid_saadc_wide_round++;
    if ((rfid_saadc_wide_round % RFID_SAADC_PHASE_FULL_WIDE_EVERY) != 0U) {
      guided_wide = true;
      half_lo =
          (rfid_saadc_tracked_half_q8 > RFID_SAADC_PHASE_REFINE_HALF_RADIUS_Q8)
              ? (rfid_saadc_tracked_half_q8 -
                 RFID_SAADC_PHASE_REFINE_HALF_RADIUS_Q8)
              : RFID_SAADC_MANCHESTER_HALF_MIN_Q8;
      half_hi =
          rfid_saadc_tracked_half_q8 + RFID_SAADC_PHASE_REFINE_HALF_RADIUS_Q8;
      if (half_hi > RFID_SAADC_MANCHESTER_HALF_MAX_Q8) {
        half_hi = RFID_SAADC_MANCHESTER_HALF_MAX_Q8;
      }
    }
  } else {
    rfid_saadc_wide_round++;
  }
#endif

  if (log_best && RFID_DECODE_DIAG_LOG) {
    const char* mode = narrow_scan ? "narrow" : guided_wide ? "guided" : "wide";

    LOG_WRN("saadc_decode: %s half=%u..%u step=%u trk=%u/%u bad=%d", mode,
            half_lo, half_hi, (unsigned)phase_step_q8,
            rfid_saadc_tracked_half_q8, rfid_saadc_tracked_phase_q8,
            rfid_saadc_tracked_bad);
  }

#if RFID_SAADC_PHASE_REFINE_ENABLE
  if (!narrow_scan && rfid_saadc_tracked_bad <= 20 &&
      rfid_saadc_tracked_bad < 1000) {
    uint32_t hint_half_lo =
        (rfid_saadc_tracked_half_q8 > RFID_SAADC_PHASE_REFINE_HALF_RADIUS_Q8)
            ? (rfid_saadc_tracked_half_q8 -
               RFID_SAADC_PHASE_REFINE_HALF_RADIUS_Q8)
            : RFID_SAADC_MANCHESTER_HALF_MIN_Q8;
    uint32_t hint_half_hi =
        rfid_saadc_tracked_half_q8 + RFID_SAADC_PHASE_REFINE_HALF_RADIUS_Q8;

    if (hint_half_hi > RFID_SAADC_MANCHESTER_HALF_MAX_Q8) {
      hint_half_hi = RFID_SAADC_MANCHESTER_HALF_MAX_Q8;
    }

    for (uint32_t half_q8 = hint_half_lo; half_q8 <= hint_half_hi;
         half_q8 += RFID_SAADC_MANCHESTER_HALF_STEP_Q8) {
      const uint32_t bit_q8 = half_q8 * 2U;
      uint32_t phase_lo =
          (rfid_saadc_tracked_phase_q8 > RFID_SAADC_PHASE_REFINE_RADIUS_Q8)
              ? (rfid_saadc_tracked_phase_q8 -
                 RFID_SAADC_PHASE_REFINE_RADIUS_Q8)
              : 0U;
      uint32_t phase_hi =
          rfid_saadc_tracked_phase_q8 + RFID_SAADC_PHASE_REFINE_RADIUS_Q8;

      if (phase_hi >= bit_q8) {
        phase_hi = bit_q8 - 1U;
      }

      for (uint32_t phase_q8 = phase_lo; phase_q8 <= phase_hi;
           phase_q8 += RFID_SAADC_PHASE_REFINE_STEP_Q8) {
        int hit = saadc_pair_try_phase_config(
            level_len, level_limit_q8, half_q8, phase_q8, &best_score,
            &best_bad, &best_half_q8, &best_phase_q8, &best_off, &best_inv,
            &best_header, &best_row, &best_col, &best_stop, &best_bits,
            &best_code, out_code, out_bits, out_half_windows, out_offset,
            out_inverted, out_bad_pairs);
        if (hit >= 0) {
          return hit;
        }
      }
    }
  }
#endif

  for (uint32_t half_q8 = half_lo; half_q8 <= half_hi;
       half_q8 += RFID_SAADC_MANCHESTER_HALF_STEP_Q8) {
    const uint32_t bit_q8 = half_q8 * 2U;
    uint32_t phase_start = 0U;
    uint32_t phase_end = bit_q8;

#if RFID_SAADC_PHASE_TRACK_ENABLE
    if (rfid_saadc_phase_valid) {
      phase_start =
          (rfid_saadc_tracked_phase_q8 > RFID_SAADC_PHASE_TRACK_WINDOW_Q8)
              ? (rfid_saadc_tracked_phase_q8 - RFID_SAADC_PHASE_TRACK_WINDOW_Q8)
              : 0U;
      phase_end =
          rfid_saadc_tracked_phase_q8 + RFID_SAADC_PHASE_TRACK_WINDOW_Q8;
      if (phase_end > bit_q8) {
        phase_end = bit_q8;
      }
    }
#endif

    for (uint32_t phase_q8 = phase_start; phase_q8 < phase_end;
         phase_q8 += phase_step_q8) {
      int hit = saadc_pair_try_phase_config(
          level_len, level_limit_q8, half_q8, phase_q8, &best_score, &best_bad,
          &best_half_q8, &best_phase_q8, &best_off, &best_inv, &best_header,
          &best_row, &best_col, &best_stop, &best_bits, &best_code, out_code,
          out_bits, out_half_windows, out_offset, out_inverted, out_bad_pairs);
      if (hit >= 0) {
        return hit;
      }
    }
  }

#if RFID_SAADC_PHASE_REFINE_ENABLE
  if (best_off >= 0) {
    const uint32_t refine_radius_q8 =
        (best_bad <= (int)RFID_SAADC_PHASE_LOCK_MAX_BAD)
            ? RFID_SAADC_PHASE_REFINE_WIDE_RADIUS_Q8
            : RFID_SAADC_PHASE_REFINE_RADIUS_Q8;
    uint32_t half_lo =
        (best_half_q8 > RFID_SAADC_PHASE_REFINE_HALF_RADIUS_Q8)
            ? (best_half_q8 - RFID_SAADC_PHASE_REFINE_HALF_RADIUS_Q8)
            : RFID_SAADC_MANCHESTER_HALF_MIN_Q8;
    uint32_t half_hi = best_half_q8 + RFID_SAADC_PHASE_REFINE_HALF_RADIUS_Q8;

    if (half_hi > RFID_SAADC_MANCHESTER_HALF_MAX_Q8) {
      half_hi = RFID_SAADC_MANCHESTER_HALF_MAX_Q8;
    }

    for (uint32_t half_q8 = half_lo; half_q8 <= half_hi;
         half_q8 += RFID_SAADC_MANCHESTER_HALF_STEP_Q8) {
      const uint32_t bit_q8 = half_q8 * 2U;
      uint32_t phase_lo = (best_phase_q8 > refine_radius_q8)
                              ? (best_phase_q8 - refine_radius_q8)
                              : 0U;
      uint32_t phase_hi = best_phase_q8 + refine_radius_q8;

      if (phase_hi >= bit_q8) {
        phase_hi = bit_q8 - 1U;
      }

      for (uint32_t phase_q8 = phase_lo; phase_q8 <= phase_hi;
           phase_q8 += RFID_SAADC_PHASE_REFINE_STEP_Q8) {
        int hit = saadc_pair_try_phase_config(
            level_len, level_limit_q8, half_q8, phase_q8, &best_score,
            &best_bad, &best_half_q8, &best_phase_q8, &best_off, &best_inv,
            &best_header, &best_row, &best_col, &best_stop, &best_bits,
            &best_code, out_code, out_bits, out_half_windows, out_offset,
            out_inverted, out_bad_pairs);
        if (hit >= 0) {
          return hit;
        }
      }
    }
  }
#endif

  if (best_off >= 0 && best_bad > 0 &&
      best_bad <= (int)RFID_SAADC_STRICT_MAX_BAD_PAIRS) {
    const uint32_t bit_q8 = best_half_q8 * 2U;
    int rescue_bit_len = 0;

    memset(dec_raw_bits, 0, EM_DECODED_BITS_CAP);
    memset(dec_pair_bad, 0, EM_DECODED_BITS_CAP);

    for (uint32_t pos_q8 = best_phase_q8; pos_q8 + bit_q8 <= level_limit_q8 &&
                                          rescue_bit_len < EM_DECODED_BITS_CAP;
         pos_q8 += bit_q8) {
      uint8_t first = env_majority_level_q8(env_level_samples, level_len,
                                            pos_q8, best_half_q8);
      uint8_t second = env_majority_level_q8(
          env_level_samples, level_len, pos_q8 + best_half_q8, best_half_q8);

      if (first == second) {
        dec_pair_bad[rescue_bit_len] = 1U;
      }
      dec_raw_bits[rescue_bit_len++] = second;
    }

    if (rescue_bit_len >= 64 && best_off + 64 <= rescue_bit_len) {
      if (best_inv) {
        invert_bits(dec_raw_bits, rescue_bit_len);
      }

      uint64_t rescued_code;
      const char* rescue_reason = NULL;

#if RFID_DECODE_DIAG_LOG
      LOG_WRN(
          "[RESCUE-ATTEMPT] best_off=%d best_bad=%d best_inv=%d half=%u "
          "phase=%u",
          best_off, best_bad, best_inv, best_half_q8, best_phase_q8);
#endif
      if (saadc_pair_parity_rescue_at(dec_raw_bits, dec_pair_bad,
                                      rescue_bit_len, best_off,
                                      (int)RFID_SAADC_STRICT_MAX_BAD_PAIRS,
                                      &rescued_code, &rescue_reason)) {
#if RFID_SAADC_REQUIRE_HEADER_CLEAN
        if (!em4100_header_clean(dec_raw_bits, best_off)) {
          if (best_inv) {
            invert_bits(dec_raw_bits, rescue_bit_len);
          }
        } else
#endif
        {
          *out_code = rescued_code;
          *out_bits = rescue_bit_len;
          *out_half_windows = (int)best_half_q8;
          *out_offset = (int)best_phase_q8;
          *out_inverted = best_inv != 0;
          *out_bad_pairs = best_bad;
          if (best_inv) {
            invert_bits(dec_raw_bits, rescue_bit_len);
          }
#if !RFID_LOG_MINIMAL
          LOG_WRN(
              "[SUCCESS-SAADC-01-10 half_q8=%d phase_q8=%d inv=%d bad=%d "
              "parity=%s header=0]",
              (int)best_half_q8, (int)best_phase_q8, best_inv, best_bad,
              rescue_reason != NULL ? rescue_reason : "?");
          LOG_WRN("[SUCCESS-SAADC-01-10] ID: %010llX",
                  (unsigned long long)rescued_code);
#endif
          return best_off;
        }
      }

      if (best_inv) {
        invert_bits(dec_raw_bits, rescue_bit_len);
      }
    }
  }

  if (log_best && !RFID_SAADC_LOG_IMPORTANT_ONLY) {
    LOG_INF(
        "saadc_pair_best: score=%d bad=%d half_q8=%u phase_q8=%u frame=%d "
        "inv=%d",
        best_score, best_bad, best_half_q8, best_phase_q8, best_off, best_inv);
    LOG_INF("saadc_pair_best: bits=%d header=%d row=%d col=%d stop=%d",
            best_bits, best_header, best_row, best_col, best_stop);
    LOG_INF("saadc_pair_best: raw40=%010llX low16=%u",
            (unsigned long long)best_code, (uint16_t)(best_code & 0xFFFFULL));
  }

#if RFID_SAADC_PHASE_TRACK_ENABLE
  if (best_off >= 0) {
    bool update_lock = !rfid_saadc_phase_valid ||
                       best_bad < rfid_saadc_tracked_bad -
                                      (int)RFID_SAADC_PHASE_LOCK_HYST_BAD ||
                       (best_bad < rfid_saadc_tracked_bad &&
                        best_score < rfid_saadc_tracked_score) ||
                       (best_bad == rfid_saadc_tracked_bad &&
                        best_score < rfid_saadc_tracked_score);

    if (update_lock) {
      rfid_saadc_tracked_half_q8 = best_half_q8;
      rfid_saadc_tracked_phase_q8 = best_phase_q8;
      rfid_saadc_tracked_bad = best_bad;
      rfid_saadc_tracked_score = best_score;
      rfid_saadc_phase_hold_streak = 0U;
#if ACTIVE_DECODE_MODE == DECODE_MODE_FSK
      if (best_bad <= (int)RFID_SAADC_PHASE_LOCK_MAX_BAD &&
          best_half_q8 >= 1400U) {
#else
      if (best_bad <= (int)RFID_SAADC_PHASE_LOCK_MAX_BAD) {
#endif
        rfid_saadc_phase_valid = true;
        if (log_best) {
          LOG_WRN(
              "saadc_phase_lock: half_q8=%u phase_q8=%u bad=%d score=%d "
              "CH_A=%u",
              best_half_q8, best_phase_q8, best_bad, best_score,
              (unsigned)rfid_pwm_ch_a_ref_runtime);
        }
      } else {
        rfid_saadc_phase_valid = false;
        if (log_best) {
          LOG_WRN(
              "saadc_phase_probe: half_q8=%u phase_q8=%u bad=%d "
              "(>%u, wide next)",
              best_half_q8, best_phase_q8, best_bad,
              (unsigned)RFID_SAADC_PHASE_LOCK_MAX_BAD);
        }
      }
    } else if (log_best) {
      rfid_saadc_phase_hold_streak++;
      LOG_WRN(
          "saadc_phase_hold: half_q8=%u phase_q8=%u bad=%d (cand "
          "bad=%d score=%d)",
          rfid_saadc_tracked_half_q8, rfid_saadc_tracked_phase_q8,
          rfid_saadc_tracked_bad, best_bad, best_score);
      if (rfid_saadc_phase_hold_streak >= RFID_SAADC_PHASE_HOLD_RESCAN_GT &&
          rfid_saadc_tracked_bad > (int)RFID_SAADC_PHASE_LOCK_MAX_BAD) {
        rfid_saadc_phase_valid = false;
        rfid_saadc_phase_hold_streak = 0U;
        LOG_WRN("saadc_phase_rescan: bad=%d stuck, next=wide",
                rfid_saadc_tracked_bad);
      }
    }
  }
#endif

#if RFID_SAADC_CHA_REF_SWEEP_ENABLE
  if (best_off >= 0 && best_bad > (int)RFID_SAADC_CHA_REF_SWEEP_BAD_GT) {
    static const uint8_t cha_table[] = {22U, 23U, 24U};
    static uint8_t cha_idx;

    cha_idx = (uint8_t)((cha_idx + 1U) % (uint8_t)ARRAY_SIZE(cha_table));
    if (rfid_pwm_ch_a_ref_runtime != cha_table[cha_idx]) {
      rfid_pwm_ch_a_ref_runtime = cha_table[cha_idx];
#if RFID_SAADC_PHASE_TRACK_ENABLE
      rfid_saadc_phase_valid = false;
#endif
      (void)rfid_pwm_apply_duty(rfid_pwm_active_duty);
      LOG_WRN("saadc_cha_ref_sweep: CH_A=%u (bad=%d > %u)",
              (unsigned)rfid_pwm_ch_a_ref_runtime, best_bad,
              (unsigned)RFID_SAADC_CHA_REF_SWEEP_BAD_GT);
    }
  }
#endif

  rfid_saadc_pair_last_best_code = best_code;
  rfid_saadc_pair_last_best_bad = best_bad;
  rfid_saadc_pair_last_best_score = best_score;

  if (best_half_q8 > 0 && best_bad <= 20) {
    int bit_len = 0;
    memset(dec_raw_bits, 0, EM_DECODED_BITS_CAP);
    const uint32_t bit_q8 = best_half_q8 * 2U;

    for (uint32_t pos_q8 = best_phase_q8;
         pos_q8 + bit_q8 <= level_limit_q8 && bit_len < EM_DECODED_BITS_CAP;
         pos_q8 += bit_q8) {
      dec_raw_bits[bit_len++] = env_majority_level_q8(
          env_level_samples, level_len, pos_q8 + best_half_q8, best_half_q8);
    }
#if !RFID_LOG_MINIMAL
    LOG_WRN(
        "====== [ROUND BITS DUMP] half=%u phase=%u len=%d inv=%d bad=%d ======",
        best_half_q8, best_phase_q8, bit_len, best_inv, best_bad);
#endif
    if (best_inv) {
      invert_bits(dec_raw_bits, bit_len);
    }
#if !RFID_LOG_MINIMAL
    char chunk[65];
    for (int i = 0; i < bit_len; i += 64) {
      int chunk_len = bit_len - i;
      if (chunk_len > 64) chunk_len = 64;
      for (int j = 0; j < chunk_len; j++) {
        chunk[j] = dec_raw_bits[i + j] ? '1' : '0';
      }
      chunk[chunk_len] = '\0';
      LOG_WRN("BITS[%03d]: %s", i, chunk);
    }
    LOG_WRN(
        "==================================================================");
#endif
  }

  return -1;
}

uint64_t rfid_saadc_pair_last_best_code;
int rfid_saadc_pair_last_best_bad;
int rfid_saadc_pair_last_best_score;

#if RFID_USE_SAADC_RECEIVER && RFID_FSK_BOOTSTRAP_PHASE_LOCK
static void rfid_saadc_fsk_bootstrap_phase_lock(void) {
  rfid_saadc_hid_phase_hold = true;
  rfid_saadc_phase_valid = true;
  rfid_saadc_tracked_half_q8 = RFID_FSK_BOOTSTRAP_HALF_Q8;
  rfid_saadc_tracked_phase_q8 = RFID_FSK_BOOTSTRAP_PHASE_Q8;
  rfid_saadc_tracked_bad = 0;
  rfid_saadc_tracked_score = 0;
  rfid_fsk_log_next_match = true;
}
#endif

#if RFID_USE_SAADC_RECEIVER
static void rfid_saadc_capture_reset(void) {
  rfid_saadc_capture_done = false;
  rfid_saadc_raw_samples = 0;
  rfid_saadc_sample_errors = 0;
  rfid_saadc_clip_samples = 0;
  rfid_saadc_clip_pos_samples = 0;
  rfid_saadc_clip_neg_samples = 0;
  rfid_saadc_next_buffer = 0;
  rfid_saadc_window_samples = 0;
  rfid_saadc_window_abs_sum = 0;
  rfid_saadc_window_min = INT16_MAX;
  rfid_saadc_window_max = INT16_MIN;
  rfid_saadc_window_signed_sum = 0;
  rfid_saadc_raw_abs_sum = 0;
  rfid_saadc_raw_sum = 0;
  rfid_saadc_raw_min = INT16_MAX;
  rfid_saadc_raw_max = INT16_MIN;
  rfid_saadc_env_level_count = 0;
#if RFID_SAADC_DC_BIAS_ENABLE
  saadc_dc_bias_accumulator = 0;
  saadc_dc_bias_sample_count = 0;
  saadc_dc_offset = 0;
#endif
#if ACTIVE_DECODE_MODE == DECODE_MODE_FSK
  fsk_window_crosses = 0;
  last_fsk_sample = 0;
  memset(fsk_cross_history, 0, sizeof(fsk_cross_history));
  if (!rfid_saadc_hid_phase_hold) {
    /* 尚未讀到 HID：全掃 640..1800 */
    rfid_saadc_phase_valid = false;
    rfid_saadc_tracked_bad = 1000;
    rfid_saadc_tracked_score = 1000;
  }
#endif
}

static void rfid_saadc_process_sample(int16_t sample) {
#if RFID_SAADC_DC_BIAS_ENABLE
  if (saadc_dc_bias_sample_count < RFID_SAADC_DC_BIAS_LEARN_SAMPLES) {
    saadc_dc_bias_accumulator += (int32_t)sample;
    saadc_dc_bias_sample_count++;
    saadc_dc_offset = (int16_t)(saadc_dc_bias_accumulator /
                                (int32_t)saadc_dc_bias_sample_count);
  }
  {
    int32_t corrected = (int32_t)sample - (int32_t)saadc_dc_offset;

    if (corrected > INT16_MAX) {
      corrected = INT16_MAX;
    } else if (corrected < INT16_MIN) {
      corrected = INT16_MIN;
    }
    sample = (int16_t)corrected;
  }
#endif
#if RFID_SAADC_SAMPLE_RSHIFT > 0
  sample = (int16_t)(sample >> RFID_SAADC_SAMPLE_RSHIFT);
#endif
  uint32_t abs_sample = (sample < 0) ? (uint32_t)(-sample) : (uint32_t)sample;

#if ACTIVE_DECODE_MODE == DECODE_MODE_FSK
  if ((sample >= 0 && last_fsk_sample < 0) ||
      (sample < 0 && last_fsk_sample >= 0)) {
    fsk_window_crosses++;
  }
  last_fsk_sample = sample;
#endif

  rfid_saadc_raw_samples++;
  rfid_saadc_raw_sum += sample;
  rfid_saadc_raw_abs_sum += abs_sample;
  if (sample <= -(int16_t)RFID_SAADC_CLIP_LEVEL_ACTIVE ||
      sample >= (int16_t)RFID_SAADC_CLIP_LEVEL_ACTIVE) {
    rfid_saadc_clip_samples++;
    if (sample >= (int16_t)RFID_SAADC_CLIP_LEVEL_ACTIVE) {
      rfid_saadc_clip_pos_samples++;
    }
    if (sample <= -(int16_t)RFID_SAADC_CLIP_LEVEL_ACTIVE) {
      rfid_saadc_clip_neg_samples++;
    }
  }
  if (sample < rfid_saadc_raw_min) rfid_saadc_raw_min = sample;
  if (sample > rfid_saadc_raw_max) rfid_saadc_raw_max = sample;

  if (rfid_saadc_window_samples == 0U) {
    rfid_saadc_window_min = sample;
    rfid_saadc_window_max = sample;
  } else {
    if (sample < rfid_saadc_window_min) {
      rfid_saadc_window_min = sample;
    }
    if (sample > rfid_saadc_window_max) {
      rfid_saadc_window_max = sample;
    }
  }

  rfid_saadc_window_signed_sum += sample;
  rfid_saadc_window_abs_sum += abs_sample;
  rfid_saadc_window_samples++;
  if (rfid_saadc_window_samples >= RFID_SAADC_SAMPLES_PER_WINDOW) {
    int count = rfid_saadc_env_level_count;

    if (count < EM_ENV_LEVEL_CAP) {
#if ACTIVE_DECODE_MODE == DECODE_MODE_FSK
      fsk_cross_history[count] =
          (uint8_t)((fsk_window_crosses > 255) ? 255 : fsk_window_crosses);
      fsk_window_crosses = 0;
#endif
      int16_t pp = (int16_t)(rfid_saadc_window_max - rfid_saadc_window_min);
      int32_t signed_avg =
          rfid_saadc_window_signed_sum / (int32_t)rfid_saadc_window_samples;

      if (pp < 0) {
        pp = 0;
      }
      if (signed_avg > INT16_MAX) {
        signed_avg = INT16_MAX;
      } else if (signed_avg < INT16_MIN) {
        signed_avg = INT16_MIN;
      }
      adc_window_avg_samples[count] =
          (int16_t)(rfid_saadc_window_abs_sum / rfid_saadc_window_samples);
      adc_window_pp_samples[count] = pp;
      adc_window_signed_samples[count] = (int16_t)signed_avg;
      rfid_saadc_env_level_count = count + 1;
    }

    rfid_saadc_window_abs_sum = 0;
    rfid_saadc_window_signed_sum = 0;
    rfid_saadc_window_samples = 0;
    rfid_saadc_window_min = INT16_MAX;
    rfid_saadc_window_max = INT16_MIN;

    if (rfid_saadc_env_level_count >= (int)rfid_saadc_target_windows) {
      rfid_saadc_capture_done = true;
    }
  }
}

static void rfid_saadc_event_handler(nrfx_saadc_evt_t const* p_event) {
  switch (p_event->type) {
    case NRFX_SAADC_EVT_BUF_REQ:
      if (!rfid_saadc_capture_done) {
        uint32_t idx = rfid_saadc_next_buffer++ % RFID_SAADC_BUFFER_COUNT;
        nrfx_err_t err =
            nrfx_saadc_buffer_set((nrf_saadc_value_t*)rfid_saadc_buffers[idx],
                                  RFID_SAADC_BUFFER_SIZE);
        if (err != NRFX_SUCCESS) {
          rfid_saadc_sample_errors++;
        }
      }
      break;

    case NRFX_SAADC_EVT_DONE:
      for (uint16_t i = 0; i < p_event->data.done.size; i++) {
        if (rfid_saadc_capture_done) break;
        int16_t sample = NRFX_SAADC_SAMPLE_GET(p_event->data.done.p_buffer, i);
        rfid_saadc_process_sample(sample);
      }
      break;

    default:
      break;
  }
}

static int16_t rfid_saadc_env_value_at(int idx) {
  const int16_t abs_v = adc_window_avg_samples[idx];
  const int16_t pp_v = adc_window_pp_samples[idx];
  const int16_t sgn_v = adc_window_signed_samples[idx];
  const int16_t sgn_abs = (int16_t)((sgn_v < 0) ? -sgn_v : sgn_v);

  switch (rfid_saadc_active_env_mode) {
    case RFID_SAADC_ENV_MODE_PP:
      return pp_v;
    case RFID_SAADC_ENV_MODE_SIGNED:
      return sgn_abs;
    case RFID_SAADC_ENV_MODE_BLEND:
      return (int16_t)((abs_v + pp_v) / 2);
    case RFID_SAADC_ENV_MODE_PP_CAP_BLEND: {
      int16_t ppc = pp_v;

      if (ppc > (int16_t)RFID_SAADC_ENV_PP_CAP_ACTIVE) {
        ppc = (int16_t)RFID_SAADC_ENV_PP_CAP_ACTIVE;
      }
      return (int16_t)((abs_v + ppc) / 2);
    }
    default:
      return abs_v;
  }
}

static int32_t rfid_adc_smoothed_env(int idx, int len) {
  int32_t left = rfid_saadc_env_value_at(idx > 0 ? idx - 1 : idx);
  int32_t mid = rfid_saadc_env_value_at(idx);
  int32_t right = rfid_saadc_env_value_at((idx + 1 < len) ? idx + 1 : idx);

  return (left + (mid * 2) + right) / 4;
}

#if RFID_SAADC_EXPORT_RAW_WINDOWS
static const struct device* rfid_raw_uart_dev;

static void rfid_raw_uart_putc(char c) {
  uart_poll_out(rfid_raw_uart_dev, (unsigned char)c);
}

static void rfid_raw_uart_puts(const char* s) {
  while (s != NULL && *s != '\0') {
    rfid_raw_uart_putc(*s++);
  }
}

static void rfid_saadc_dump_window_csv(int env_level_count) {
  static int dumps_left = (int)RFID_SAADC_RAW_DUMP_MAX;
  char line[80];

  if (dumps_left <= 0) {
    return;
  }
  dumps_left--;

  if (rfid_raw_uart_dev == NULL) {
    if (!device_is_ready(DEVICE_DT_GET(DT_CHOSEN(zephyr_console)))) {
      LOG_ERR("raw_dump: console UART not ready");
      return;
    }
    rfid_raw_uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
  }

  LOG_WRN("raw_dump_uart: exporting %d windows (bypass log buffer)",
          env_level_count);

  rfid_raw_uart_puts("\n====== [RAW-ADC-DATA-START] ======\n");
  snprintf(
      line, sizeof(line),
      "meta,window_us=%u,samples_per_window=%lu,sample_hz=%lu,env_count=%d\n",
      RFID_SAADC_ENV_WINDOW_US, (unsigned long)RFID_SAADC_SAMPLES_PER_WINDOW,
      (unsigned long)RFID_SAADC_SAMPLE_HZ, env_level_count);
  rfid_raw_uart_puts(line);
  rfid_raw_uart_puts("Index,Signed_Diff_Raw,Abs_Avg_Raw\n");

  for (int d_idx = 0; d_idx < env_level_count; d_idx++) {
    snprintf(line, sizeof(line), "%d,%d,%d\n", d_idx,
             (int)adc_window_signed_samples[d_idx],
             (int)adc_window_avg_samples[d_idx]);
    rfid_raw_uart_puts(line);
    if ((d_idx & 0x0F) == 0x0F) {
      k_sleep(K_MSEC(2));
    }
  }

  snprintf(line, sizeof(line),
           "====== [RAW-ADC-DATA-END] env_count=%d ======\n\n",
           env_level_count);
  rfid_raw_uart_puts(line);
  LOG_WRN("raw_dump_uart: done");
}
#endif

static int rfid_saadc_decode_from_envelope(int env_level_count,
                                           const char* tag) {
#if ACTIVE_DECODE_MODE == DECODE_MODE_FSK
  if (env_level_count < 128) {
    return 0;
  }

  /* 6 視窗 (384µs) + 門檻 11：FSK 實測可穩定分出 12.5k / 15.625k */
  const int FSK_WINDOW_COUNT = 6;
  const int FSK_CROSS_THRESHOLD = 11;

  for (int i = 0; i < env_level_count; i++) {
    int sum_cross = 0;
    int win_count = 0;

    for (int j = i; j < i + FSK_WINDOW_COUNT && j < env_level_count; j++) {
      sum_cross += (int)fsk_cross_history[j];
      win_count++;
    }

    if (win_count <= 0) {
      env_level_samples[i] = 0U;
      env_edge_samples[i] = 0U;
      continue;
    }

    const int normalized_cross = (sum_cross * FSK_WINDOW_COUNT) / win_count;

    env_level_samples[i] = (normalized_cross >= FSK_CROSS_THRESHOLD) ? 1U : 0U;
    env_edge_samples[i] = env_level_samples[i] ? 255U : 0U;
  }

#if RFID_SAADC_DECODE_ENABLE
  if (em4095_try_level_decoders(env_level_count, "fsk", 0, 0)) {
    return 1;
  }
#endif
  ARG_UNUSED(tag);
  return 2;

#else /* 👇 景明哥，您剛剛漏掉了這個 #else ！！！ */

  uint32_t adc_hist[8] = {0};
  uint32_t transitions = 0;
  uint32_t high_windows = 0;
  int32_t env_sum = 0;
  int16_t env_min = INT16_MAX;
  int16_t env_max = INT16_MIN;

  if (env_level_count < 128) {
    return 0;
  }

  for (int i = 0; i < env_level_count; i++) {
    int32_t smooth = rfid_adc_smoothed_env(i, env_level_count);

    env_sum += smooth;
    if (smooth < env_min) {
      env_min = (int16_t)smooth;
    }
    if (smooth > env_max) {
      env_max = (int16_t)smooth;
    }
  }

  if (env_min == INT16_MAX || env_max == INT16_MIN) {
    return 0;
  }

  const int32_t env_range = (int32_t)env_max - (int32_t)env_min;
  uint32_t env_pct_hist[64] = {0};
  int32_t p20 = env_min;
  int32_t p80 = env_max;

  if (env_range > 0) {
    for (int i = 0; i < env_level_count; i++) {
      int32_t avg = rfid_adc_smoothed_env(i, env_level_count);
      uint32_t bin = (uint32_t)(((avg - env_min) * 63) / env_range);

      if (bin > 63U) {
        bin = 63U;
      }
      env_pct_hist[bin]++;
    }

    uint32_t acc = 0;
    uint32_t target20 = (uint32_t)env_level_count / 5U;
    uint32_t target80 = ((uint32_t)env_level_count * 4U) / 5U;
    bool got20 = false;

    for (uint32_t bin = 0; bin < 64U; bin++) {
      acc += env_pct_hist[bin];
      if (!got20 && acc >= target20) {
        p20 = env_min + ((int32_t)bin * env_range) / 63;
        got20 = true;
      }
      if (acc >= target80) {
        p80 = env_min + ((int32_t)bin * env_range) / 63;
        break;
      }
    }
  }

  const int32_t pct_range = p80 - p20;
  int32_t active_threshold_pct = RFID_SAADC_ENV_THRESHOLD_PCT;

  const int32_t threshold =
      p20 + ((pct_range * (int32_t)active_threshold_pct) / 100);
  const int32_t hysteresis = pct_range > 64 ? (pct_range / 8) : 4;
  uint8_t last_level = 0;
  bool have_last_level = false;
  uint8_t level_state =
      (rfid_adc_smoothed_env(0, env_level_count) > threshold) ? 1U : 0U;

  for (int i = 0; i < env_level_count; i++) {
    int32_t avg = rfid_adc_smoothed_env(i, env_level_count);
    uint32_t norm = 0;

    if (avg > threshold + hysteresis) {
      level_state = 1U;
    } else if (avg + hysteresis < threshold) {
      level_state = 0U;
    }

    if (env_range > 0) {
      norm = (uint32_t)(((avg - env_min) * 255) / env_range);
      if (norm > 255U) {
        norm = 255U;
      }
    }

    env_edge_samples[i] = (uint8_t)norm;
    env_level_samples[i] = level_state;
    adc_hist[(norm >= 255U) ? 7U : (norm / 32U)]++;
    if (level_state) {
      high_windows++;
    }
    if (have_last_level && level_state != last_level) {
      transitions++;
    }
    last_level = level_state;
    have_last_level = true;
  }

#if RFID_SAADC_VERBOSE_ENV_LOG
  LOG_INF("saadc_env[%s]: avg=%d min=%d max=%d range=%d p20=%d p80=%d", tag,
          env_level_count ? (env_sum / env_level_count) : 0, env_min, env_max,
          env_range, p20, p80);
  LOG_INF("saadc_thr[%s]: threshold=%d hyst=%d pct_range=%d", tag, threshold,
          hysteresis, pct_range);
  ARG_UNUSED(high_windows);
  ARG_UNUSED(transitions);
  ARG_UNUSED(adc_hist);
#else
  ARG_UNUSED(high_windows);
  ARG_UNUSED(transitions);
  ARG_UNUSED(adc_hist);
#endif

  if (env_range < RFID_SAADC_MIN_RANGE_COUNTS) {
    return 0;
  }

#if RFID_SAADC_DECODE_ENABLE
  if (em4095_try_level_decoders(env_level_count, tag, env_min, env_max)) {
    return 1;
  }
#endif
  return 2;

#endif /* 結束 FSK / ASK 判斷 */
}

static int em4095_try_level_decoders(int env_level_count, const char* frac_tag,
                                     int32_t sample_min, int32_t sample_max) {
  uint64_t level_code;
  int level_bits = 0;
  int half_windows = 0;
  int level_offset = 0;
  int level_bad_pairs = 0;
  bool level_inverted = false;

  int level_slide = try_em4100_saadc_pair_decode(
      env_level_count, &level_code, &level_bits, &half_windows, &level_offset,
      &level_inverted, &level_bad_pairs, RFID_DECODE_DIAG_LOG);
  if (level_slide >= 0) {
#if !RFID_LOG_MINIMAL
    LOG_WRN(
        "[SUCCESS-SAADC-01-10 half_q8=%d phase_q8=%d inv=%d bad=%d "
        "slide=%d bits=%d]",
        half_windows, level_offset, level_inverted, level_bad_pairs,
        level_slide, level_bits);
    LOG_WRN("[SUCCESS-SAADC-01-10] ID: %010llX",
            (unsigned long long)level_code);
#endif
    return 1;
  }

#if RFID_SAADC_STRICT_MANCHESTER_ONLY
  if (rfid_saadc_pair_last_best_bad > (int)RFID_SAADC_STRICT_MAX_BAD_PAIRS) {
    if (!RFID_SAADC_LOG_IMPORTANT_ONLY) {
      LOG_INF(
          "%s_strict_pair_fail: levels=%d bad=%d max=%u half_q8=%u..%u "
          "sample=%d..%d",
          frac_tag, env_level_count, rfid_saadc_pair_last_best_bad,
          (unsigned)RFID_SAADC_STRICT_MAX_BAD_PAIRS,
          RFID_SAADC_MANCHESTER_HALF_MIN_Q8, RFID_SAADC_MANCHESTER_HALF_MAX_Q8,
          sample_min, sample_max);
    }
    return 0;
  }
#endif

  if (!RFID_SAADC_LOG_IMPORTANT_ONLY) {
    LOG_INF("%s_decode_fail: levels=%d half=%u..%u bad_max=%u sample=%d..%d",
            frac_tag, env_level_count, EM_ENV_HALF_MIN_WINDOWS,
            EM_ENV_HALF_MAX_WINDOWS, EM_LEVEL_MAX_FRAME_BAD_PAIRS, sample_min,
            sample_max);
  }
  return 0;
}

static int rfid_saadc_configure(void) {
  nrfx_err_t err;

  if (rfid_saadc_ready) return 0;

#if !RFID_USE_SAADC_RECEIVER && RFID_COMP_RX_GPIO_PINCNF_ENABLE
  rfid_rx_ain_gpio_configure();
#endif

#if defined(__ZEPHYR__)
  if (!rfid_saadc_irq_connected) {
    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SAADC), IRQ_PRIO_LOWEST,
                nrfx_saadc_irq_handler, 0, 0);
    irq_enable(NRFX_IRQ_NUMBER_GET(NRF_SAADC));
    rfid_saadc_irq_connected = true;
  }
#endif

  if (!nrfx_saadc_init_check()) {
    err = nrfx_saadc_init(NRFX_SAADC_DEFAULT_CONFIG_IRQ_PRIORITY);
    if (err != NRFX_SUCCESS && err != NRFX_ERROR_ALREADY) {
      LOG_ERR("SAADC init failed: %d", err);
      return -EIO;
    }
  }

  /* P0.04=AIN0=CMP_P, P0.05=AIN1=1.65V ref（nRF5340 AIN0=P0.04, AIN1=P0.05）
   */
  /*
  nrfx_saadc_channel_t channel = NRFX_SAADC_DEFAULT_CHANNEL_DIFFERENTIAL(
      NRF_SAADC_INPUT_AIN0, NRF_SAADC_INPUT_AIN1, RFID_SAADC_CHANNEL_INDEX);
      */
  nrfx_saadc_channel_t channel = NRFX_SAADC_DEFAULT_CHANNEL_SE(
      NRF_SAADC_INPUT_AIN0, RFID_SAADC_CHANNEL_INDEX);
  channel.channel_config.gain = NRF_SAADC_GAIN1_6;
  channel.channel_config.acq_time = NRF_SAADC_ACQTIME_3US;

  err = nrfx_saadc_channel_config(&channel);
  if (err != NRFX_SUCCESS) {
    LOG_ERR("SAADC channel config failed: %d", err);
    return -EIO;
  }

  nrfx_saadc_adv_config_t adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;
  adv_config.internal_timer_cc = RFID_SAADC_TIMER_CC;
  adv_config.start_on_end = true;

  err = nrfx_saadc_advanced_mode_set(1UL << RFID_SAADC_CHANNEL_INDEX,
                                     NRF_SAADC_RESOLUTION_12BIT, &adv_config,
                                     rfid_saadc_event_handler);
  if (err != NRFX_SUCCESS) {
    LOG_ERR("SAADC advanced mode failed: %d", err);
    return -EIO;
  }
  /*
    err = nrfx_saadc_offset_calibrate(NULL);
    if (err != NRFX_SUCCESS) {
      LOG_WRN("SAADC offset calibration skipped/failed: %d", err);
    }
  */
  rfid_saadc_ready = true;
  LOG_INF(
      "SAADC: differential AIN0(P0.04)-AIN1(P0.05), %luHz timer_cc=%lu "
      "env=%uus",
      (unsigned long)RFID_SAADC_SAMPLE_HZ, (unsigned long)RFID_SAADC_TIMER_CC,
      RFID_SAADC_ENV_WINDOW_US);
  LOG_INF("SAADC scale: rshift=%u clip=%u card_min=%u thr_pct=%u%%",
          RFID_SAADC_SAMPLE_RSHIFT, RFID_SAADC_CLIP_LEVEL_ACTIVE,
          RFID_SAADC_CARD_MIN_ABS_AVG_ACTIVE, RFID_SAADC_ENV_THRESHOLD_PCT);
  return 0;
}

int em4095_saadc_receiver(void) {
  int env_level_count;

  if (rfid_saadc_configure() != 0) return 0;

#if ACTIVE_DECODE_MODE == DECODE_MODE_FSK
  rfid_timing_round_start_ms = k_uptime_get();
#endif

#if RFID_SAADC_CARRIER_SETTLE_MS > 0U
  if (!rfid_saadc_carrier_settled) {
    k_sleep(K_MSEC(RFID_SAADC_CARRIER_SETTLE_MS));
    rfid_saadc_carrier_settled = true;
  }
#elif ACTIVE_DECODE_MODE == DECODE_MODE_ASK
  k_sleep(K_MSEC(50));
#endif
  rfid_saadc_target_windows = RFID_SAADC_TARGET_WINDOWS;
  rfid_saadc_capture_reset();

  uint32_t capture_start_cycle = k_cycle_get_32();
  nrfx_err_t err = nrfx_saadc_buffer_set(
      (nrf_saadc_value_t*)rfid_saadc_buffers[0], RFID_SAADC_BUFFER_SIZE);
  if (err != NRFX_SUCCESS) {
    LOG_ERR("SAADC buffer0 set failed: %d", err);
    return 0;
  }

  err = nrfx_saadc_buffer_set((nrf_saadc_value_t*)rfid_saadc_buffers[1],
                              RFID_SAADC_BUFFER_SIZE);
  if (err != NRFX_SUCCESS) {
    LOG_ERR("SAADC buffer1 set failed: %d", err);
    return 0;
  }
  rfid_saadc_next_buffer = 0;

  err = nrfx_saadc_mode_trigger();
  if (err != NRFX_SUCCESS) {
    LOG_ERR("SAADC trigger failed: %d", err);
    return 0;
  }

  uint64_t timeout = k_uptime_get() + RFID_CAPTURE_WINDOW_MS + 100U;
  while (!rfid_saadc_capture_done && k_uptime_get() < timeout) {
    k_sleep(K_MSEC(RFID_SAADC_CAPTURE_POLL_MS));
  }

  nrfx_saadc_abort();
#if RFID_SAADC_POST_ABORT_SLEEP_MS > 0U
  k_sleep(K_MSEC(RFID_SAADC_POST_ABORT_SLEEP_MS));
#endif
  env_level_count = rfid_saadc_env_level_count;

  uint32_t capture_us =
      k_cyc_to_us_near32(k_cycle_get_32() - capture_start_cycle);
#if ACTIVE_DECODE_MODE == DECODE_MODE_FSK
  rfid_timing_capture_us = capture_us;
  rfid_timing_decode_start_ms = k_uptime_get();
#endif
  const int32_t raw_avg =
      rfid_saadc_raw_samples
          ? (rfid_saadc_raw_sum / (int32_t)rfid_saadc_raw_samples)
          : 0;
  const uint32_t raw_abs_avg =
      rfid_saadc_raw_samples ? (rfid_saadc_raw_abs_sum / rfid_saadc_raw_samples)
                             : 0;
  const uint32_t clip_pct =
      rfid_saadc_raw_samples
          ? (rfid_saadc_clip_samples * 100U) / rfid_saadc_raw_samples
          : 0U;
  const uint32_t clip_pos_pct =
      rfid_saadc_raw_samples
          ? (rfid_saadc_clip_pos_samples * 100U) / rfid_saadc_raw_samples
          : 0U;
  const uint32_t clip_neg_pct =
      rfid_saadc_raw_samples
          ? (rfid_saadc_clip_neg_samples * 100U) / rfid_saadc_raw_samples
          : 0U;
#if RFID_RX_CAPTURE_STATS_LOG
  LOG_INF("saadc: raw=%u windows=%d capture_us=%u errors=%u",
          rfid_saadc_raw_samples, env_level_count, capture_us,
          rfid_saadc_sample_errors);
  LOG_INF("saadc_raw: avg=%d abs_avg=%u min=%d max=%d clip=%u (%u%%)", raw_avg,
          raw_abs_avg, rfid_saadc_raw_samples ? rfid_saadc_raw_min : 0,
          rfid_saadc_raw_samples ? rfid_saadc_raw_max : 0,
          rfid_saadc_clip_samples, clip_pct);
  LOG_WRN("saadc_clip_dir: pos=%u (%u%%) neg=%u (%u%%)",
          rfid_saadc_clip_pos_samples, clip_pos_pct,
          rfid_saadc_clip_neg_samples, clip_neg_pct);
#endif

  const bool saadc_saturated =
      (rfid_saadc_raw_samples > 0U) &&
      ((rfid_saadc_clip_samples * 100U) >
       (rfid_saadc_raw_samples * RFID_SAADC_MAX_CLIP_PERCENT));

  if (saadc_saturated) {
    LOG_WRN("saadc_saturated: clip=%u/%u > %u%%; lower carrier drive",
            rfid_saadc_clip_samples, rfid_saadc_raw_samples,
            RFID_SAADC_MAX_CLIP_PERCENT);
  }

  if (raw_abs_avg < RFID_SAADC_CARD_MIN_ABS_AVG_ACTIVE) {
#if RFID_RX_CAPTURE_STATS_LOG
    LOG_INF("saadc_card_gate_skip: abs_avg=%u/%u", raw_abs_avg,
            RFID_SAADC_CARD_MIN_ABS_AVG_ACTIVE);
#endif
    return 0;
  }

  if (env_level_count < 128) {
#if RFID_RX_CAPTURE_STATS_LOG
    LOG_INF("saadc_skip: not enough windows=%d", env_level_count);
#endif
    return 0;
  }

#if RFID_SAADC_EXPORT_RAW_WINDOWS
  rfid_saadc_dump_window_csv(env_level_count);
#endif

#if RFID_SAADC_SAT_ENV_ENABLE
  if (saadc_saturated) {
    static const struct {
      rfid_saadc_env_mode_t mode;
      const char* tag;
    } sat_modes[] = {
        {RFID_SAADC_ENV_MODE_SIGNED, "adc_signed"},
        {RFID_SAADC_ENV_MODE_BLEND, "adc_blend"},
        {RFID_SAADC_ENV_MODE_PP_CAP_BLEND, "adc_ppcb"},
        {RFID_SAADC_ENV_MODE_PP, "adc_pp"},
        {RFID_SAADC_ENV_MODE_ABS, "adc_abs"},
    };
    int last_ret = 0;

    if (!RFID_SAADC_LOG_IMPORTANT_ONLY) {
      LOG_WRN("saadc_sat_env: trying %u envelope modes",
              (unsigned)ARRAY_SIZE(sat_modes));
    }
    for (size_t mi = 0; mi < ARRAY_SIZE(sat_modes); mi++) {
      int ret;

      rfid_saadc_active_env_mode = sat_modes[mi].mode;
      ret = rfid_saadc_decode_from_envelope(env_level_count, sat_modes[mi].tag);
      if (ret == 1) {
        if (!RFID_SAADC_LOG_IMPORTANT_ONLY) {
          LOG_WRN("saadc_sat_env: success mode=%s", sat_modes[mi].tag);
        }
        return 1;
      }
      if (ret > last_ret) {
        last_ret = ret;
      }
    }

#if !RFID_LOG_MINIMAL
    LOG_WRN("[CARD-PRESENT-SAADC] sat modes failed, best_ret=%d", last_ret);
#endif
    return last_ret > 0 ? last_ret : 2;
  }
#endif

  rfid_saadc_active_env_mode = RFID_SAADC_ENV_MODE_ABS;
  {
    int ret = rfid_saadc_decode_from_envelope(env_level_count, "adc_frac");

    if (ret == 1) {
      return 1;
    }
    if (ret == 2) {
#if !RFID_LOG_MINIMAL
      LOG_WRN("[CARD-PRESENT-SAADC] no valid EM4100 frame yet");
#endif
      return 2;
    }
    return 0;
  }
}
#endif /* RFID_USE_SAADC_RECEIVER */

int main(void) {
  int carrier_ret;
#if RFID_PWM_DTI_AB_SCAN_ENABLE
  size_t dti_ab_index = 0U;
  int64_t dti_ab_next_switch_ms = 0;
#endif

  LOG_INF("nRF5340 Discrete RFID Starting...");
  if (!gpio_is_ready_dt(&led)) return -1;
  gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
  gpio_pin_set_dt(&led, 0);
#if RFID_BOOT_SELFTEST_LOG
  em4100_decode_self_test();
#endif
  LOG_INF("RX path: SAADC differential AIN0-AIN1");

#if RFID_PWM_DTI_AB_SCAN_ENABLE
  rfid_pwm_apply_dti_profile(dti_ab_index);
  dti_ab_next_switch_ms =
      k_uptime_get() + (int64_t)RFID_PWM_DTI_AB_SCAN_INTERVAL_MS;
#endif

  carrier_ret = rfid_pwm_apply_duty(RFID_PWM_DUTY_REFERENCE);
  if (carrier_ret != 0) {
    LOG_ERR("Carrier PWM wave start failed, ret=%d", carrier_ret);
    k_sleep(K_MSEC(1500));
  } else {
    LOG_INF("PWM carrier: duty=%u dti=%d dead_rise=%u dead_fall=%u",
            RFID_PWM_DUTY_REFERENCE, RFID_PWM_DTI_ENABLE,
            (unsigned)rfid_pwm_dead_rise_ticks,
            (unsigned)rfid_pwm_dead_fall_ticks);
#if RFID_PWM_DTI_AB_SCAN_ENABLE
    LOG_INF("DTI A/B scan: %u profiles x %u ms (card on antenna)",
            (unsigned)TEST_RISE_FALL_PAIRS_COUNT,
            (unsigned)RFID_PWM_DTI_AB_SCAN_INTERVAL_MS);
#endif
#if RFID_FSK_BOOTSTRAP_PHASE_LOCK
    rfid_saadc_fsk_bootstrap_phase_lock();
    LOG_INF("FSK phase bootstrap: half_q8=%u phase_q8=%u",
            (unsigned)RFID_FSK_BOOTSTRAP_HALF_Q8,
            (unsigned)RFID_FSK_BOOTSTRAP_PHASE_Q8);
#endif
  }

  while (!carrier_ret) {
#if RFID_PWM_DTI_AB_SCAN_ENABLE
    if (k_uptime_get() >= dti_ab_next_switch_ms) {
      dti_ab_index = (dti_ab_index + 1U) % TEST_RISE_FALL_PAIRS_COUNT;
      rfid_pwm_apply_dti_profile(dti_ab_index);
      carrier_ret = rfid_pwm_apply_duty(RFID_PWM_DUTY_REFERENCE);
      if (carrier_ret != 0) {
        LOG_ERR("DTI A/B switch failed ret=%d profile=%u", carrier_ret,
                (unsigned)dti_ab_index);
        break;
      }
      LOG_INF("DTI A/B profile[%u/%u]: rise=%u fall=%u duty=%u",
              (unsigned)dti_ab_index + 1U, (unsigned)TEST_RISE_FALL_PAIRS_COUNT,
              (unsigned)rfid_pwm_dead_rise_ticks,
              (unsigned)rfid_pwm_dead_fall_ticks,
              (unsigned)RFID_PWM_DUTY_REFERENCE);
      dti_ab_next_switch_ms =
          k_uptime_get() + (int64_t)RFID_PWM_DTI_AB_SCAN_INTERVAL_MS;
    }
#endif
    if (!em4095_saadc_receiver()) {
#if RFID_MAIN_IDLE_LOG
      LOG_INF("No card detected.");
#endif
    }
  }
  return 0;
}