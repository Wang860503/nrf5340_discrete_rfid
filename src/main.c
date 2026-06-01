/*
 * 針對 SAADC 飽和壓低載波 Duty 14 調優版：
 * 1. raw edge 仍過濾 125kHz 載波殘留，避免直接進解碼
 * 2. 包絡密度保留全部 COMP edge，用 window 聚合後產生 Manchester tick
 */

#include <errno.h>
#include <hal/nrf_comp.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_timer.h>
#include <limits.h>
#include <nrfx_comp.h>
#include <nrfx_pwm.h>
#include <nrfx_saadc.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/comparator.h>
#include <zephyr/drivers/comparator/nrf_comp.h>
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
/* 第四招：COMP/SAADC 前重設 PIN_CNF（類比腳 INPUT_DISCONNECT + 可選 DRIVE）
 * 2.8V 總攻：這一招先暫停，避免變因太多。 */
#define RFID_COMP_RX_GPIO_PINCNF_ENABLE 0
#if defined(GPIO_PIN_CNF_DRIVE_E0E1)
#define RFID_COMP_RX_GPIO_DRIVE NRF_GPIO_PIN_E0E1
#else
#define RFID_COMP_RX_GPIO_DRIVE NRF_GPIO_PIN_S0S1
#endif
/* 量產主路徑：SAADC 差動；改 0 可切回 COMP 診斷（nrfx 直連） */
#define RFID_USE_SAADC_RECEIVER 1
/* COMP：nRF5340 僅 TIMER0–2；用 TIMER1 @ 1MHz（TIMER2 常被 Zephyr timing 使用）
 */
#if !RFID_USE_SAADC_RECEIVER
#define RFID_USE_HIGH_RES_TIMER 1
#if defined(NRF_TIMER1)
#define RFID_HIGH_RES_TIMER NRF_TIMER1
#elif defined(NRF_TIMER1_NS)
#define RFID_HIGH_RES_TIMER NRF_TIMER1_NS
#else
#define RFID_HIGH_RES_TIMER NRF_TIMER0
#endif
#endif
/* 1=啟用 COMP 擷取 + EdgeCap；tick 來源見 RFID_COMP_TICK_SOURCE */
#define RFID_COMP_DIRECT_EDGE_DECODE 1
/* tick 來源：ENV=128µs 包絡跳變（有 305/610µs，可分出 1T/2T）
 * DEBOUNCE=每 N µs 抽樣 CMP（間隔幾乎全為 1T，難解 EM4100） */
#define RFID_COMP_TICK_SOURCE_ENV 0
#define RFID_COMP_TICK_SOURCE_DEBOUNCE 1
#define RFID_COMP_TICK_SOURCE RFID_COMP_TICK_SOURCE_ENV
#define RFID_COMP_DEBOUNCE_TICK_US 256U
/* 直接邊緣模式下：短/長半週期容許範圍（µs），含 RF/64 和 RF/96 */
#define RFID_COMP_DIRECT_SHORT_MIN_US 200U
#define RFID_COMP_DIRECT_SHORT_MAX_US 480U
#define RFID_COMP_DIRECT_LONG_MIN_US 500U
#define RFID_COMP_DIRECT_LONG_MAX_US 1100U
/* 至少需要這麼多邊緣才視為卡在場（200ms 捕獲，RF/96 約 520 個邊緣） */
#define RFID_COMP_DIRECT_CARD_MIN_EDGES 80U
#define RFID_COMP_USE_NRFX_DIRECT \
  1 /* 1=nrfx_comp 滿血（對齊 EFM32 PD3/PD4）；0=Zephyr comparator API */
#define RFID_COMP_USE_INTERNAL_THRESHOLD \
  1 /* 0=DIFF AIN0/AIN1；1=SE AIN0 vs INT1V8 */
#define RFID_COMP_REFSEL COMP_NRF_COMP_REFSEL_INT_1V8
/* 景明哥 COMP 三招（SE INT1V8，1.65V 虛地 / 頂部削平避開） */
/* 2.8V 總攻：回到 th53/63 + sp=LOW，並關閉 isource（基線在 2.8V 條件下更穩） */
#define RFID_COMP_THRESHOLD_DOWN 24U
#define RFID_COMP_THRESHOLD_UP 30U
#define RFID_COMP_SP_MODE NRF_COMP_SP_MODE_LOW
#define RFID_COMP_ISOURCE NRF_COMP_ISOURCE_OFF
/* 第三招：關閉電流源（OFF） */
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
#define RFID_VOTE_DIAG_LOG 0
#define RFID_RX_CAPTURE_STATS_LOG 0
#define RFID_MAIN_IDLE_LOG 0
#define RFID_BOOT_SELFTEST_LOG 0
#else
#define RFID_SAADC_LOG_IMPORTANT_ONLY 0
#define RFID_SAADC_VERBOSE_ENV_LOG 1
#define RFID_DECODE_DIAG_LOG 1
#define RFID_VOTE_DIAG_LOG 1
#define RFID_RX_CAPTURE_STATS_LOG 1
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
#define TICK_BUFFER_SIZE 2048
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
#define EM_ENABLE_TICK_FALLBACK_DIAG 1
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
#define EM_TICK_SPLIT_NOM_US EM_CLOCK_BASE_1T_US
#define EM_TICK_SPLIT_MAX_US 1900
#define EM_TICK_SPLIT_MAX_PARTS 16
#define EM_TICK_HALFCELL_MIN_US 200
#define EM_TICK_HALFCELL_MAX_US 340
#define EM_TICK_HALFCELL_STEP_US 5
#define EM_TICK_HALFCELL_MAX_REPEAT 6
/* RF/64 半週期≈256µs；勿掃到 420（會把 305/610 都壓成 1T→假低分） */
#define EM_EDGECAP_UNIT_MIN_US 250
#define EM_EDGECAP_UNIT_MAX_US 340
#define EM_EDGECAP_UNIT_STEP_US 8
#define EM_EDGECAP_TOL_MIN_PCT 30
#define EM_EDGECAP_TOL_STEP_PCT 5
#define EM_EDGECAP_TOL_PCT 45
#define EM_EDGECAP_MAX_REPEAT 16
#define EM_EDGECAP_MIN_GAP_US 100
/* unit 以 ~256µs 計（RF/64）：stretched_1T≈384µs, repeat1≈512µs ... */
#define EM_EDGECAP_STRETCHED_1T_MIN_US 350
#define EM_EDGECAP_STRETCHED_1T_MAX_US 600
#define EM_EDGECAP_REPEAT1_MAX_US 620
#define EM_EDGECAP_REPEAT2_MAX_US 900
#define EM_EDGECAP_REPEAT3_MAX_US 1300
#define EM_EDGECAP_REPEAT4_MAX_US 1700
#define EM_EDGECAP_RESCUE_LONG_HALVES 38
#define EM_EDGECAP_RESCUE_BITS 128

/* 保持偵測穩定性的參數 */
#define EM_MIN_STORED_TO_DECODE 300
/* 128µs 窗：有卡時 long 常見 96–120；100 會誤觸 decode_skip */
#define EM_MIN_LONG_TICKS_TO_DECODE 80
/* 同一 raw40 在單次擷取窗內至少 N 次才進入 streak（僅 EM4100 舊路徑） */
#define EM_CANDIDATE_CONFIRM_COUNT 5
#define EM_ID_STREAK_CONFIRM_COUNT 5
#define EM_CONFIRM_TRACKED_CODES 8
/* 0=少印 [CANDIDATE-*] 雜訊；1=除錯 */
#define EM_CANDIDATE_LOG_VERBOSE 0
#define EM_SELFTEST_CODE 0x0123456789ULL
#define EM_CROSS_SCAN_VOTE_FRAMES 7
#define EM_CROSS_SCAN_VOTE_MAX_FEED_SCORE 26
#define EM_CROSS_SCAN_VOTE_MAX_BAD_PAIRS 10
#define EM_SCAN_VOTE_CANDIDATES 12
#define EM_SCAN_VOTE_MAX_SCORE 30
#define EM_ROW_VOTE_GROUPS 16
#define EM_ROW_VOTE_GROUP_MIN_MATCH_ROWS 8
#define EM_ROW_VOTE_ROWS 10
#define EM_ROW_VOTE_BITS 5
#define EM_ROW_VOTE_DATA_BITS 4
#define EM_ROW_VOTE_DEPTH 7
#define EM_WHOLE_FEED_MIN_CONSENSUS_ROWS 6
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

#if !RFID_USE_SAADC_RECEIVER && !RFID_COMP_USE_NRFX_DIRECT
static const struct device* comp_dev = DEVICE_DT_GET(DT_NODELABEL(comp));
#endif
#if !RFID_USE_SAADC_RECEIVER && RFID_COMP_USE_NRFX_DIRECT
static volatile uint32_t comp_nrfx_irq_ready;
static volatile uint32_t comp_nrfx_irq_cross;
static volatile uint32_t comp_nrfx_irq_up;
static volatile uint32_t comp_nrfx_irq_down;
static bool comp_nrfx_armed;
#endif
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

static uint32_t tick_buffer[TICK_BUFFER_SIZE];
static uint8_t tick_edge_to_level[TICK_BUFFER_SIZE];
#if RFID_COMP_DIRECT_EDGE_DECODE
/* env128：僅在高分/近目標時記住 Manchester 極性 inv */
static bool comp_phase_valid;
static int comp_last_best_inv;
#define COMP_PHASE_LOCK_MAX_SCORE 6
#endif
static uint8_t edgecap_half_bits[EM_DECODED_BITS_CAP];
static uint16_t edgecap_half_tick_idx[EM_DECODED_BITS_CAP];
static uint8_t edgecap_half_sub_idx[EM_DECODED_BITS_CAP];
static uint8_t dec_raw_bits[EM_DECODED_BITS_CAP];
static uint8_t dec_pair_bad[EM_DECODED_BITS_CAP];
static uint16_t dec_bad_prefix[EM_DECODED_BITS_CAP + 1];
static uint16_t dec_bit_half_pos[EM_DECODED_BITS_CAP];
static uint8_t em_delta_best_bits_snapshot[EM_DECODED_BITS_CAP];
static uint8_t em_delta_inv_bits[EM_DECODED_BITS_CAP];
static uint8_t env_edge_samples[EM_ENV_LEVEL_CAP];
/* 邊緣計時解碼器專用緩衝區 */
static int16_t em_et_edge_pos[800]; /* edge window 位置 */
static int8_t em_et_edge_dir[800];  /* +1=rising, -1=falling */
static uint8_t em_et_bits[800];     /* 解碼後 bit 串 */
static uint8_t env_level_samples[EM_ENV_LEVEL_CAP];
#if ACTIVE_DECODE_MODE == DECODE_MODE_FSK
static uint8_t fsk_cross_history[EM_ENV_LEVEL_CAP];
#endif
static int16_t adc_window_avg_samples[EM_ENV_LEVEL_CAP];
static int16_t adc_window_pp_samples[EM_ENV_LEVEL_CAP];
static int16_t adc_window_signed_samples[EM_ENV_LEVEL_CAP];
static int em_delta_tracked_frame_off = -1;
static uint8_t em_delta_shift_log_div;

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
static uint64_t confirm_candidate_codes[EM_CONFIRM_TRACKED_CODES];
static uint8_t confirm_candidate_counts[EM_CONFIRM_TRACKED_CODES];
static uint64_t em4100_streak_id;
static uint8_t em4100_streak_count;
static uint64_t em4100_latched_ready_id;
static uint8_t cross_scan_vote_ones[64];
static uint8_t cross_scan_vote_frames;
static bool row_vote_group_used[EM_ROW_VOTE_GROUPS];
static uint16_t row_vote_keys[EM_ROW_VOTE_GROUPS];
static uint8_t row_vote_seed_nibbles[EM_ROW_VOTE_GROUPS][EM_ROW_VOTE_ROWS];
static uint8_t row_vote_feed_nibbles[EM_ROW_VOTE_GROUPS][EM_ROW_VOTE_DEPTH]
                                    [EM_ROW_VOTE_ROWS];
static uint8_t row_vote_ones[EM_ROW_VOTE_GROUPS][EM_ROW_VOTE_ROWS]
                            [EM_ROW_VOTE_DATA_BITS];
static uint8_t row_vote_counts[EM_ROW_VOTE_GROUPS][EM_ROW_VOTE_ROWS];

typedef struct {
  bool used;
  uint8_t bits[64];
  const char* source;
  int score;
  int bad;
  int origin;
  int inv;
  uint64_t code;
} scan_vote_candidate_t;

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
          const uint32_t fc = (uint32_t)((w26 >> 17) & 0xFFU);
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

static int confirm_em4100_candidate(uint64_t code, const char* tag) {
  int slot = -1;
  int replace_slot = 0;
  uint8_t count;

  if (code == 0) {
#if EM_CANDIDATE_LOG_VERBOSE
    LOG_WRN("[CANDIDATE-%s] reject zero raw40", tag);
#endif
    return 0;
  }

#if EM_ALIGN_TARGET_LOW16 != 0U
  if (em4100_low16_delta((uint16_t)(code & 0xFFFFULL)) > 256) {
#if EM_CANDIDATE_LOG_VERBOSE
    LOG_WRN("[CANDIDATE-%s] reject far tgt low16=%04X", tag,
            (uint16_t)(code & 0xFFFFULL));
#endif
    return 0;
  }
#endif

  for (int i = 0; i < (int)ARRAY_SIZE(confirm_candidate_codes); i++) {
    if (confirm_candidate_counts[i] == 0) {
      replace_slot = i;
      break;
    }
    if (confirm_candidate_counts[i] < confirm_candidate_counts[replace_slot]) {
      replace_slot = i;
    }
    if (confirm_candidate_codes[i] == code) {
      slot = i;
      break;
    }
  }

  if (slot < 0) {
    slot = replace_slot;
    confirm_candidate_codes[slot] = code;
    confirm_candidate_counts[slot] = 0;
  }

  if (confirm_candidate_counts[slot] < UINT8_MAX) {
    confirm_candidate_counts[slot]++;
  }
  count = confirm_candidate_counts[slot];

#if EM_CANDIDATE_LOG_VERBOSE
  LOG_WRN("[CANDIDATE-%s] raw40=%010llX low16=%04X pool=%u/%u", tag,
          (unsigned long long)code, (uint16_t)(code & 0xFFFFULL), count,
          EM_CANDIDATE_CONFIRM_COUNT);
#endif

#if EM_ALIGN_TARGET_LOW16 != 0U
  if ((uint16_t)(code & 0xFFFFULL) == EM_ALIGN_TARGET_LOW16) {
    LOG_WRN("[CARD_READY] raw40=%010llX low16=%04X tag=%s (target)",
            (unsigned long long)code, (uint16_t)(code & 0xFFFFULL), tag);
    em4100_streak_id = code;
    em4100_streak_count = EM_ID_STREAK_CONFIRM_COUNT;
    return 1;
  }
#endif

  if (count < EM_CANDIDATE_CONFIRM_COUNT) {
    return 0;
  }

  if (code == em4100_streak_id) {
    if (em4100_streak_count < UINT8_MAX) {
      em4100_streak_count++;
    }
  } else {
    em4100_streak_id = code;
    em4100_streak_count = 1U;
    em4100_latched_ready_id = 0ULL;
  }

  if (em4100_streak_count < EM_ID_STREAK_CONFIRM_COUNT) {
    return 0;
  }

  if (code != em4100_latched_ready_id) {
    LOG_WRN("[CARD_READY] raw40=%010llX low16=%04X pool=%u streak=%u via=%s",
            (unsigned long long)code, (uint16_t)(code & 0xFFFFULL), count,
            em4100_streak_count, tag);
    em4100_latched_ready_id = code;
  }
  return 1;
}

#if EM_ENABLE_TICK_FALLBACK_DIAG
static int count_bad_pairs_in_frame(int off);
static int em4100_frame_error_score(const uint8_t* bits, int off,
                                    int* header_err, int* row_err, int* col_err,
                                    int* stop_err);
static bool em4100_candidate_from_parity(const uint8_t* bits, int off,
                                         uint64_t* out_code,
                                         const char** reason);

static int __attribute__((unused)) try_em4100_slide(const uint8_t* bits,
                                                    int len,
                                                    uint64_t* out_code) {
  if (len < 64) return -1;
  for (int i = 0; i <= len - 64; i++) {
    if (em4100_valid_at(bits, len, i)) {
      *out_code = decode_card_code((uint8_t*)&bits[i]);
      return i;
    }
  }
  return -1;
}
#endif

static int count_bad_pairs_in_frame(int off) {
  int bad = 0;

  for (int i = 0; i < 64; i++) {
    bad += dec_pair_bad[off + i] ? 1 : 0;
  }
  return bad;
}

static int try_em4100_level_slide(const uint8_t* bits, int len,
                                  uint64_t* out_code, int* out_frame_bad) {
  if (len < 64) return -1;

  for (int i = 0; i <= len - 64; i++) {
    if (em4100_valid_at(bits, len, i)) {
      int frame_bad = count_bad_pairs_in_frame(i);
      if (frame_bad <= EM_LEVEL_MAX_FRAME_BAD_PAIRS) {
        *out_code = decode_card_code((uint8_t*)&bits[i]);
        *out_frame_bad = frame_bad;
        return i;
      }
    }
  }
  return -1;
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

static void em4100_log_shift_window(const uint8_t* bits, int len,
                                    int center_off, const char* tag) {
  int lo = center_off - RFID_EM4100_SHIFT_SCAN_RADIUS;
  int hi = center_off + RFID_EM4100_SHIFT_SCAN_RADIUS;

  if (len < 64 || center_off < 0) return;
  if (lo < 0) lo = 0;
  if (hi > len - 64) hi = len - 64;

#if RFID_DECODE_DIAG_LOG
  for (int off = lo; off <= hi; off++) {
    int header_err;
    int row_err;
    int col_err;
    int stop_err;
    int score = em4100_frame_error_score(bits, off, &header_err, &row_err,
                                         &col_err, &stop_err);
    uint64_t code = decode_card_code((uint8_t*)&bits[off]);
    LOG_WRN(
        "[%s_shift off=%d] score=%d hdr=%d row=%d col=%d stop=%d raw40=%010llX",
        tag, off, score, header_err, row_err, col_err, stop_err,
        (unsigned long long)code);
  }
#endif
}

static int find_header_offset(const uint8_t* bits, int len) {
  if (len < 64) return -1;

  for (int i = 0; i <= len - 64; i++) {
    bool match = true;
    for (int j = 0; j < 9; j++) {
      if (bits[i + j] != 1U) {
        match = false;
        break;
      }
    }
    if (match) return i;
  }
  return -1;
}

static bool em4100_header_clean(const uint8_t* bits, int off) {
  for (int j = 0; j < 9; j++) {
    if (bits[off + j] != 1U) {
      return false;
    }
  }
  return true;
}

static void reset_em4100_confirm_pool(void) {
  memset(confirm_candidate_codes, 0, sizeof(confirm_candidate_codes));
  memset(confirm_candidate_counts, 0, sizeof(confirm_candidate_counts));
}

static void reset_em4100_card_session(void) {
  reset_em4100_confirm_pool();
  em4100_streak_id = 0ULL;
  em4100_streak_count = 0U;
  em4100_latched_ready_id = 0ULL;
}

static void reset_cross_scan_vote_pool(void) {
  memset(cross_scan_vote_ones, 0, sizeof(cross_scan_vote_ones));
  cross_scan_vote_frames = 0;
}

static void reset_row_vote_pool(void) {
  memset(row_vote_group_used, 0, sizeof(row_vote_group_used));
  memset(row_vote_keys, 0, sizeof(row_vote_keys));
  memset(row_vote_seed_nibbles, 0, sizeof(row_vote_seed_nibbles));
  memset(row_vote_feed_nibbles, 0, sizeof(row_vote_feed_nibbles));
  memset(row_vote_ones, 0, sizeof(row_vote_ones));
  memset(row_vote_counts, 0, sizeof(row_vote_counts));
}

static void reset_row_vote_group(int group) {
  if (group < 0 || group >= EM_ROW_VOTE_GROUPS) return;
  row_vote_group_used[group] = false;
  row_vote_keys[group] = 0;
  memset(row_vote_seed_nibbles[group], 0, sizeof(row_vote_seed_nibbles[group]));
  memset(row_vote_feed_nibbles[group], 0, sizeof(row_vote_feed_nibbles[group]));
  memset(row_vote_ones[group], 0, sizeof(row_vote_ones[group]));
  memset(row_vote_counts[group], 0, sizeof(row_vote_counts[group]));
}

static uint8_t em4100_row_data_nibble(const uint8_t* bits, int row) {
  uint8_t nibble = 0;
  int base = 9 + row * EM_ROW_VOTE_BITS;

  for (int bit = 0; bit < EM_ROW_VOTE_DATA_BITS; bit++) {
    nibble = (uint8_t)((nibble << 1) | (bits[base + bit] ? 1U : 0U));
  }
  return nibble;
}

static void seed_row_vote_group(int group, const uint8_t* bits, uint16_t key) {
  reset_row_vote_group(group);
  row_vote_group_used[group] = true;
  row_vote_keys[group] = key;
  for (int row = 0; row < EM_ROW_VOTE_ROWS; row++) {
    row_vote_seed_nibbles[group][row] = em4100_row_data_nibble(bits, row);
  }
}

static int row_vote_seed_match_rows(int group, const uint8_t* bits) {
  int matches = 0;

  if (group < 0 || group >= EM_ROW_VOTE_GROUPS || !row_vote_group_used[group]) {
    return 0;
  }

  for (int row = 0; row < EM_ROW_VOTE_ROWS; row++) {
    if (row_vote_seed_nibbles[group][row] ==
        em4100_row_data_nibble(bits, row)) {
      matches++;
    }
  }
  return matches;
}

static void row_vote_bits_to_hex(const uint8_t* bits, char* out_hex) {
  static const char hex[] = "0123456789ABCDEF";

  for (int row = 0; row < EM_ROW_VOTE_ROWS; row++) {
    out_hex[row] = hex[em4100_row_data_nibble(bits, row) & 0x0FU];
  }
  out_hex[EM_ROW_VOTE_ROWS] = '\0';
}

static void row_vote_feed_to_hex(int group, int feed, char* out_hex) {
  static const char hex[] = "0123456789ABCDEF";

  for (int row = 0; row < EM_ROW_VOTE_ROWS; row++) {
    out_hex[row] = hex[row_vote_feed_nibbles[group][feed][row] & 0x0FU];
  }
  out_hex[EM_ROW_VOTE_ROWS] = '\0';
}

static int row_vote_feed_support(int group, int feed, int* consensus_rows) {
  int score = 0;
  int rows_with_majority = 0;

  for (int row = 0; row < EM_ROW_VOTE_ROWS; row++) {
    int support = 0;
    uint8_t nibble = row_vote_feed_nibbles[group][feed][row];

    for (int other = 0; other < EM_ROW_VOTE_DEPTH; other++) {
      if (row_vote_feed_nibbles[group][other][row] == nibble) {
        support++;
      }
    }

    score += support;
    if (support > (EM_ROW_VOTE_DEPTH / 2)) {
      rows_with_majority++;
    }
  }

  if (consensus_rows != NULL) {
    *consensus_rows = rows_with_majority;
  }
  return score;
}

static int row_vote_select_best_feed(int group, int* best_score,
                                     int* best_consensus_rows,
                                     int* runner_up_score) {
  int selected_feed = -1;
  int selected_score = -1;
  int selected_consensus = -1;
  int second_score = -1;

  for (int feed = 0; feed < EM_ROW_VOTE_DEPTH; feed++) {
    int consensus_rows = 0;
    int score = row_vote_feed_support(group, feed, &consensus_rows);

    if (score > selected_score ||
        (score == selected_score && consensus_rows > selected_consensus)) {
      second_score = selected_score;
      selected_score = score;
      selected_consensus = consensus_rows;
      selected_feed = feed;
    } else if (score > second_score) {
      second_score = score;
    }
  }

  if (best_score != NULL) {
    *best_score = selected_score;
  }
  if (best_consensus_rows != NULL) {
    *best_consensus_rows = selected_consensus;
  }
  if (runner_up_score != NULL) {
    *runner_up_score = second_score;
  }
  return selected_feed;
}

static void stitch_row_vote_feed(int group, int feed, uint8_t* stitched) {
  uint8_t col_sum[EM_ROW_VOTE_DATA_BITS] = {0};

  memset(stitched, 0, 64);
  for (int bit = 0; bit < 9; bit++) {
    stitched[bit] = 1U;
  }

  for (int row = 0; row < EM_ROW_VOTE_ROWS; row++) {
    int base = 9 + row * EM_ROW_VOTE_BITS;
    uint8_t nibble = row_vote_feed_nibbles[group][feed][row];
    uint8_t row_sum = 0;

    for (int bit = 0; bit < EM_ROW_VOTE_DATA_BITS; bit++) {
      uint8_t data_bit = (nibble >> (EM_ROW_VOTE_DATA_BITS - 1 - bit)) & 1U;
      stitched[base + bit] = data_bit;
      row_sum += data_bit;
      col_sum[bit] += data_bit;
    }
    stitched[base + 4] = row_sum & 1U;
  }

  for (int col = 0; col < EM_ROW_VOTE_DATA_BITS; col++) {
    stitched[59 + col] = col_sum[col] & 1U;
  }
  stitched[63] = 0U;
}

static void dump_row_vote_nibbles(int group, const char* reason,
                                  const uint8_t* stitched, int header_err,
                                  int row_err, int col_err, int stop_err,
                                  int weak_data_bits, uint64_t code) {
  char final_hex[EM_ROW_VOTE_ROWS + 1];
  char feed_hex[EM_ROW_VOTE_ROWS + 1];

  if (group < 0 || group >= EM_ROW_VOTE_GROUPS) return;

  row_vote_bits_to_hex(stitched, final_hex);

#if RFID_VOTE_DIAG_LOG
  LOG_WRN("============== [NIBBLE-VOTE-DUMP-START] ==============");
  LOG_WRN("[NIBBLE-VOTE-DUMP] reason=%s group=%d seed=%u", reason, group,
          row_vote_keys[group]);
  LOG_WRN(
      "[NIBBLE-VOTE-DUMP] err header=%d row=%d col=%d stop=%d weak=%d "
      "raw40=%010llX low16=%u",
      header_err, row_err, col_err, stop_err, weak_data_bits,
      (unsigned long long)code, (uint16_t)(code & 0xFFFFULL));
  LOG_WRN("[NIBBLE-VOTE-DUMP] final=%s", final_hex);

  for (int feed = 0; feed < EM_ROW_VOTE_DEPTH; feed++) {
    row_vote_feed_to_hex(group, feed, feed_hex);
    LOG_WRN("[NIBBLE-VOTE-DUMP] feed[%d]=%s", feed, feed_hex);
  }
  LOG_WRN("==============  [NIBBLE-VOTE-DUMP-END]  ==============");
#endif
}

static void consider_scan_vote_candidate(scan_vote_candidate_t* candidates,
                                         const uint8_t* bits,
                                         const char* source, int score,
                                         int bad_pairs, int origin, int inv,
                                         uint64_t code) {
  int empty = -1;
  int worst = -1;
  int worst_score = -1;

  if (score > EM_SCAN_VOTE_MAX_SCORE ||
      bad_pairs > EM_CROSS_SCAN_VOTE_MAX_BAD_PAIRS || code == 0) {
    return;
  }

  for (int i = 0; i < EM_SCAN_VOTE_CANDIDATES; i++) {
    if (!candidates[i].used) {
      if (empty < 0) empty = i;
      continue;
    }
    if (candidates[i].code == code) {
      if (score >= candidates[i].score) return;
      empty = i;
      break;
    }
    if (candidates[i].score > worst_score) {
      worst_score = candidates[i].score;
      worst = i;
    }
  }

  if (empty < 0) {
    if (worst < 0 || score >= worst_score) return;
    empty = worst;
  }

  candidates[empty].used = true;
  candidates[empty].source = source;
  candidates[empty].score = score;
  candidates[empty].bad = bad_pairs;
  candidates[empty].origin = origin;
  candidates[empty].inv = inv;
  candidates[empty].code = code;
  memcpy(candidates[empty].bits, bits, sizeof(candidates[empty].bits));
}

static bool __attribute__((unused)) feed_cross_scan_vote_frame(
    const uint8_t* bits, const char* source, int score, int bad_pairs,
    uint64_t* out_code) {
  uint8_t voted[64] = {0};
  int header_err;
  int row_err;
  int col_err;
  int stop_err;
  uint64_t code = decode_card_code((uint8_t*)bits);

  if (score > EM_CROSS_SCAN_VOTE_MAX_FEED_SCORE ||
      bad_pairs > EM_CROSS_SCAN_VOTE_MAX_BAD_PAIRS) {
    return false;
  }

  for (int bit = 0; bit < 64; bit++) {
    if (bits[bit]) cross_scan_vote_ones[bit]++;
  }
  cross_scan_vote_frames++;

#if RFID_VOTE_DIAG_LOG
  LOG_WRN("[VOTE-FEED-%s] frames=%u/%u score=%d bad=%d raw40=%010llX low16=%u",
          source, cross_scan_vote_frames, EM_CROSS_SCAN_VOTE_FRAMES, score,
          bad_pairs, (unsigned long long)code, (uint16_t)(code & 0xFFFFULL));
#endif

  if (cross_scan_vote_frames < EM_CROSS_SCAN_VOTE_FRAMES) {
    return false;
  }

  for (int bit = 0; bit < 64; bit++) {
    voted[bit] =
        cross_scan_vote_ones[bit] > (EM_CROSS_SCAN_VOTE_FRAMES / 2) ? 1U : 0U;
  }

  (void)em4100_frame_error_score(voted, 0, &header_err, &row_err, &col_err,
                                 &stop_err);
  code = decode_card_code(voted);

  if (EM4100_Full_Check(voted) && code != 0) {
#if RFID_VOTE_DIAG_LOG
    LOG_WRN("[VOTE-SUCCESS] raw40=%010llX low32=%08llX low16=%u",
            (unsigned long long)code,
            (unsigned long long)(code & 0xFFFFFFFFULL),
            (uint16_t)(code & 0xFFFFULL));
#endif
    *out_code = code;
    reset_cross_scan_vote_pool();
    return true;
  }

#if RFID_VOTE_DIAG_LOG
  LOG_WRN(
      "[VOTE-FAILED] frames=%u header=%d row=%d col=%d stop=%d raw40=%010llX "
      "low16=%u",
      cross_scan_vote_frames, header_err, row_err, col_err, stop_err,
      (unsigned long long)code, (uint16_t)(code & 0xFFFFULL));
#endif
  reset_cross_scan_vote_pool();
  return false;
}

static bool feed_row_vote_frame(const uint8_t* bits, const char* source,
                                int score, int bad_pairs, uint64_t* out_code) {
  uint8_t stitched[64] = {0};
  uint8_t accepted_rows = 0;
  bool all_rows_ready = true;
  int group = -1;
  int empty_group = -1;
  int best_group = -1;
  int best_matches = -1;
  int match_rows = 0;
  int header_err;
  int row_err;
  int col_err;
  int stop_err;
  int weak_data_bits = 0;
  int selected_feed = -1;
  int selected_score = -1;
  int selected_consensus_rows = 0;
  int runner_up_score = -1;
  uint64_t input_code = decode_card_code((uint8_t*)bits);
  uint16_t key = (uint16_t)(input_code & 0xFFFFULL);
  uint64_t code;

  if (score > EM_CROSS_SCAN_VOTE_MAX_FEED_SCORE ||
      bad_pairs > EM_CROSS_SCAN_VOTE_MAX_BAD_PAIRS || input_code == 0) {
    return false;
  }

  for (int g = 0; g < EM_ROW_VOTE_GROUPS; g++) {
    int group_count = 0;
    for (int row = 0; row < EM_ROW_VOTE_ROWS; row++) {
      group_count += row_vote_counts[g][row];
    }

    if (!row_vote_group_used[g] && group_count == 0) {
      if (empty_group < 0) empty_group = g;
      continue;
    }

    int matches = row_vote_seed_match_rows(g, bits);
    if (matches > best_matches) {
      best_matches = matches;
      best_group = g;
    }
  }

  if (group < 0) {
    if (best_group >= 0 && best_matches >= EM_ROW_VOTE_GROUP_MIN_MATCH_ROWS) {
      group = best_group;
      match_rows = best_matches;
    } else if (empty_group >= 0) {
      group = empty_group;
      seed_row_vote_group(group, bits, key);
      match_rows = EM_ROW_VOTE_ROWS;
    } else {
#if RFID_VOTE_DIAG_LOG
      LOG_WRN("[ROW-VOTE-DROP-%s] key=%u best_match=%d groups_full", source,
              key, best_matches);
#endif
      return false;
    }
  }

  for (int row = 0; row < EM_ROW_VOTE_ROWS; row++) {
    int base = 9 + row * EM_ROW_VOTE_BITS;
    int feed_slot = row_vote_counts[group][row];
    if (row_vote_counts[group][row] >= EM_ROW_VOTE_DEPTH) continue;

    row_vote_feed_nibbles[group][feed_slot][row] =
        em4100_row_data_nibble(bits, row);
    for (int bit = 0; bit < EM_ROW_VOTE_DATA_BITS; bit++) {
      if (bits[base + bit]) row_vote_ones[group][row][bit]++;
    }
    row_vote_counts[group][row]++;
    accepted_rows++;
  }

  uint8_t min_count = EM_ROW_VOTE_DEPTH;
  uint8_t max_count = 0;
  int total_count = 0;
  for (int row = 0; row < EM_ROW_VOTE_ROWS; row++) {
    uint8_t count = row_vote_counts[group][row];
    if (count < min_count) min_count = count;
    if (count > max_count) max_count = count;
    total_count += count;
  }

#if RFID_VOTE_DIAG_LOG
  LOG_WRN(
      "[ROW-VOTE-FEED-%s] key=%u seed=%u match=%d group=%d accepted=%u "
      "score=%d bad=%d total=%d min=%u max=%u",
      source, key, row_vote_keys[group], match_rows, group, accepted_rows,
      score, bad_pairs, total_count, min_count, max_count);
#endif

  for (int row = 0; row < EM_ROW_VOTE_ROWS; row++) {
    if (row_vote_counts[group][row] < EM_ROW_VOTE_DEPTH) {
      all_rows_ready = false;
      break;
    }
  }
  if (!all_rows_ready) return false;

  for (int row = 0; row < EM_ROW_VOTE_ROWS; row++) {
    for (int bit = 0; bit < EM_ROW_VOTE_DATA_BITS; bit++) {
      uint8_t ones = row_vote_ones[group][row][bit];
      if (ones == (EM_ROW_VOTE_DEPTH / 2) ||
          ones == ((EM_ROW_VOTE_DEPTH / 2) + 1)) {
        weak_data_bits++;
      }
    }
  }

  selected_feed = row_vote_select_best_feed(
      group, &selected_score, &selected_consensus_rows, &runner_up_score);
  if (selected_feed < 0) {
    reset_row_vote_group(group);
    return false;
  }
  stitch_row_vote_feed(group, selected_feed, stitched);

  (void)em4100_frame_error_score(stitched, 0, &header_err, &row_err, &col_err,
                                 &stop_err);
  code = decode_card_code(stitched);

  char selected_hex[EM_ROW_VOTE_ROWS + 1];
  row_vote_feed_to_hex(group, selected_feed, selected_hex);
#if RFID_VOTE_DIAG_LOG
  LOG_WRN(
      "[WHOLE-FEED-VOTE] group=%d seed=%u feed=%d score=%d runner=%d "
      "consensus=%d/%d raw40=%s",
      group, row_vote_keys[group], selected_feed, selected_score,
      runner_up_score, selected_consensus_rows, EM_ROW_VOTE_ROWS, selected_hex);
#endif

  if (EM4100_Full_Check(stitched) && code != 0) {
    int stitched_matches = row_vote_seed_match_rows(group, stitched);

    if (selected_consensus_rows < EM_WHOLE_FEED_MIN_CONSENSUS_ROWS) {
#if RFID_VOTE_DIAG_LOG
      LOG_WRN(
          "[WHOLE-FEED-REJECT] seed=%u match=%d consensus=%d raw40=%010llX "
          "low16=%u weak=%d",
          row_vote_keys[group], stitched_matches, selected_consensus_rows,
          (unsigned long long)code, (uint16_t)(code & 0xFFFFULL),
          weak_data_bits);
#endif
      dump_row_vote_nibbles(group, "reject", stitched, header_err, row_err,
                            col_err, stop_err, weak_data_bits, code);
      reset_row_vote_group(group);
      return false;
    }

#if RFID_VOTE_DIAG_LOG
    LOG_WRN(
        "[WHOLE-FEED-CANDIDATE] seed=%u match=%d consensus=%d raw40=%010llX "
        "low32=%08llX low16=%u weak=%d",
        row_vote_keys[group], stitched_matches, selected_consensus_rows,
        (unsigned long long)code, (unsigned long long)(code & 0xFFFFFFFFULL),
        (uint16_t)(code & 0xFFFFULL), weak_data_bits);
#endif
    if (!confirm_em4100_candidate(code, "whole-feed")) {
      reset_row_vote_group(group);
      return false;
    }
    *out_code = code;
    reset_row_vote_group(group);
    return true;
  }

#if RFID_VOTE_DIAG_LOG
  LOG_WRN(
      "[ROW-VOTE-FAILED] key=%u header=%d row=%d col=%d stop=%d weak=%d "
      "raw40=%010llX low16=%u",
      key, header_err, row_err, col_err, stop_err, weak_data_bits,
      (unsigned long long)code, (uint16_t)(code & 0xFFFFULL));
#endif
  dump_row_vote_nibbles(group, "failed", stitched, header_err, row_err, col_err,
                        stop_err, weak_data_bits, code);
  reset_row_vote_group(group);
  return false;
}

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

static bool em4100_candidate_from_relaxed_header_clean(const uint8_t* bits,
                                                       int off,
                                                       uint64_t* out_code,
                                                       const char** reason) {
  int header_err;
  int row_err;
  int col_err;
  int stop_err;

  (void)em4100_frame_error_score(bits, off, &header_err, &row_err, &col_err,
                                 &stop_err);
  if (stop_err != 0 || row_err != 0 || col_err != 0 || header_err > 1) {
    return false;
  }

  *out_code = decode_card_code((uint8_t*)&bits[off]);
  *reason = header_err == 0 ? "clean" : "header1-clean";
  return true;
}

#if EM_ALIGN_TARGET_LOW16 != 0U
static int em4100_low16_delta(uint16_t low16) {
  int delta = (int)low16 - (int)EM_ALIGN_TARGET_LOW16;

  if (delta < 0) {
    delta = -delta;
  }
  return delta;
}

static bool em4100_frame_near_target_low16(uint64_t code) {
  uint16_t low16 = (uint16_t)(code & 0xFFFFULL);
  unsigned ham =
      (unsigned)__builtin_popcount((unsigned)(low16 ^ EM_ALIGN_TARGET_LOW16));

  return ham <= EM_NEAR_TARGET_MAX_HAM ||
         em4100_low16_delta(low16) <= EM_NEAR_TARGET_MAX_DELTA;
}

static int em4100_decode_rank(int frame_score, uint64_t code) {
  int tgt_pen = em4100_low16_delta((uint16_t)(code & 0xFFFFULL));

  if (tgt_pen > 4095) {
    tgt_pen = 4095;
  }
  return frame_score * 4096 + tgt_pen;
}

static bool em4100_try_target_code(const uint8_t* trial, uint64_t* out_code) {
  uint64_t code;

  if (!EM4100_Full_Check((uint8_t*)trial)) {
    return false;
  }
  code = decode_card_code((uint8_t*)trial);
  if ((uint16_t)(code & 0xFFFFULL) != EM_ALIGN_TARGET_LOW16) {
    return false;
  }
  *out_code = code;
  return true;
}

static bool em4100_force_parity_repair(const uint8_t* frame64,
                                       uint64_t* out_code) {
  uint8_t trial[64];
  int hdr_bad = 0;

  memcpy(trial, frame64, 64);
  for (int j = 0; j < 9; j++) {
    if (trial[j] != 1U) {
      hdr_bad++;
    }
  }
  if (hdr_bad > 2) {
    return false;
  }

  for (int j = 0; j < 9; j++) {
    trial[j] = 1U;
  }
  trial[63] = 0U;

  for (int row = 0; row < 10; row++) {
    int base = 9 + row * 5;
    int row_sum = 0;

    for (int col = 0; col < 4; col++) {
      row_sum += trial[base + col];
    }
    trial[base + 4] = (uint8_t)(row_sum & 1);
  }

  {
    uint8_t col_parity[4] = {0};

    for (int row = 0; row < 10; row++) {
      int base = 9 + row * 5;

      for (int col = 0; col < 4; col++) {
        col_parity[col] += trial[base + col];
      }
    }
    for (int col = 0; col < 4; col++) {
      trial[59 + col] = (uint8_t)(col_parity[col] & 1);
    }
  }

  return em4100_try_target_code(trial, out_code);
}

static bool em4100_bruteforce_flip_search(uint8_t* trial, int max_flips,
                                          uint64_t* out_code) {
  if (max_flips >= 1) {
    for (int b = 9; b < 64; b++) {
      trial[b] ^= 1U;
      if (em4100_try_target_code(trial, out_code)) {
        return true;
      }
      trial[b] ^= 1U;
    }
  }

  if (max_flips < 2) {
    return false;
  }

  for (int b1 = 9; b1 < 63; b1++) {
    trial[b1] ^= 1U;
    for (int b2 = b1 + 1; b2 < 64; b2++) {
      trial[b2] ^= 1U;
      if (em4100_try_target_code(trial, out_code)) {
        trial[b2] ^= 1U;
        trial[b1] ^= 1U;
        return true;
      }
      trial[b2] ^= 1U;
    }
    trial[b1] ^= 1U;
  }

  if (max_flips < 3) {
    return false;
  }

  for (int b1 = 9; b1 < 62; b1++) {
    trial[b1] ^= 1U;
    for (int b2 = b1 + 1; b2 < 63; b2++) {
      trial[b2] ^= 1U;
      for (int b3 = b2 + 1; b3 < 64; b3++) {
        trial[b3] ^= 1U;
        if (em4100_try_target_code(trial, out_code)) {
          trial[b3] ^= 1U;
          trial[b2] ^= 1U;
          trial[b1] ^= 1U;
          return true;
        }
        trial[b3] ^= 1U;
      }
      trial[b2] ^= 1U;
    }
    trial[b1] ^= 1U;
  }

  return false;
}

static bool em4100_recover_align_target(const uint8_t* frame64,
                                        uint64_t* out_code) {
  uint8_t trial[64];
  uint64_t code;
  uint16_t low16;
  unsigned ham;
  int delta;
  int max_flips = 2;

  memcpy(trial, frame64, 64);
  code = decode_card_code(trial);
  if (!em4100_frame_near_target_low16(code)) {
    return false;
  }

  low16 = (uint16_t)(code & 0xFFFFULL);
  ham = (unsigned)__builtin_popcount((unsigned)(low16 ^ EM_ALIGN_TARGET_LOW16));
  delta = em4100_low16_delta(low16);
  if (delta <= (int)EM_NEAR_TARGET_BRUTE_DELTA || ham <= 6U) {
    max_flips = 3;
  }

  if (em4100_try_target_code(trial, out_code)) {
    return true;
  }
  if (em4100_force_parity_repair(frame64, out_code)) {
    return true;
  }

  invert_bits(trial, 64);
  if (em4100_try_target_code(trial, out_code)) {
    return true;
  }
  if (em4100_force_parity_repair(trial, out_code)) {
    return true;
  }
  invert_bits(trial, 64);

  if (em4100_bruteforce_flip_search(trial, max_flips, out_code)) {
    return true;
  }

  invert_bits(trial, 64);
  if (em4100_bruteforce_flip_search(trial, max_flips, out_code)) {
    return true;
  }

#if RFID_DECODE_DIAG_LOG
  LOG_WRN("tick_lvl_recover_fail: low16=%04X ham=%u delta=%d flips=%d", low16,
          ham, delta, max_flips);
#endif
  return false;
}
#endif /* EM_ALIGN_TARGET_LOW16 != 0U */

static bool em4100_candidate_from_synced_header(const uint8_t* bits, int len,
                                                int off, uint64_t* out_code,
                                                const char** reason) {
  const char* strict_reason;

  if (off <= 0 || off + 64 > len || bits[off - 1] != 0) return false;

  if (em4100_candidate_from_parity(bits, off, out_code, &strict_reason) &&
      strcmp(strict_reason, "data2") != 0) {
    *reason = strict_reason;
    return true;
  }

  return em4100_candidate_from_relaxed_header_clean(bits, off, out_code,
                                                    reason);
}

static bool edgecap_decode_bit_pair(int half_len, int hb_start,
                                    uint8_t* out_bit) {
  if (hb_start < 0 || hb_start + 1 >= half_len) return false;

  uint8_t first = edgecap_half_bits[hb_start];
  uint8_t second = edgecap_half_bits[hb_start + 1];

  if (first == 1U && second == 0U) {
    *out_bit = 1U;
  } else if (first == 0U && second == 1U) {
    *out_bit = 0U;
  } else {
    *out_bit = second;
  }

  return true;
}

/* EM4100 僅 1T/2T：用 EM_SHORT_MAX 硬切，勿用 unit 除法（396µs 會被誤判為 1T）
 */
static bool edgecap_repeat_from_gap_mode(uint32_t T, bool allow_long,
                                         uint32_t unit_1t_us,
                                         uint32_t* repeat) {
  (void)unit_1t_us;

  if (T < EM_EDGECAP_MIN_GAP_US) {
    return false;
  }

  if (T <= EM_SHORT_MAX) {
    *repeat = 1U;
  } else if (!allow_long) {
    *repeat = 1U;
  } else if (T <= EM_LONG_MAX) {
    *repeat = 2U;
  } else {
    *repeat = 3U;
    if (*repeat > EM_EDGECAP_MAX_REPEAT) {
      *repeat = EM_EDGECAP_MAX_REPEAT;
    }
  }

  return true;
}

static bool edgecap_repeat_from_gap(uint32_t T, uint32_t unit_1t_us,
                                    uint32_t* repeat) {
  return edgecap_repeat_from_gap_mode(T, false, unit_1t_us, repeat);
}

static bool edgecap_decode_header_rescue_stream(
    int tick_count, int tick_start, uint8_t tick_sub_start, uint32_t unit_1t_us,
    uint8_t* out_bits, uint8_t* out_pair_bad, uint16_t* out_half_tick,
    int* out_bit_len) {
  uint8_t half_bits[EM_EDGECAP_RESCUE_BITS * 2];
  uint16_t half_tick[EM_EDGECAP_RESCUE_BITS * 2];
  int half_len = 0;

  if (tick_start < 0 || tick_start >= tick_count) return false;
  memset(out_bits, 0, EM_EDGECAP_RESCUE_BITS);
  memset(out_pair_bad, 0, EM_EDGECAP_RESCUE_BITS);
  memset(out_half_tick, 0, EM_EDGECAP_RESCUE_BITS * sizeof(out_half_tick[0]));

  for (int i = tick_start;
       i < tick_count && half_len < (EM_EDGECAP_RESCUE_BITS * 2); i++) {
    uint32_t repeat;
    uint32_t start_n = (i == tick_start) ? tick_sub_start : 0U;
    bool allow_long = half_len < EM_EDGECAP_RESCUE_LONG_HALVES;

    if (!edgecap_repeat_from_gap_mode(tick_buffer[i], allow_long, unit_1t_us,
                                      &repeat)) {
      continue;
    }
    if (start_n >= repeat) continue;

    uint8_t half_bit = tick_edge_to_level[i] ? 0U : 1U;
    for (uint32_t n = start_n; n < repeat && half_len < 128; n++) {
      half_bits[half_len] = half_bit;
      half_tick[half_len] = (uint16_t)i;
      half_len++;
    }
  }

  if (half_len < 128) return false;

  *out_bit_len = half_len / 2;
  for (int bit = 0, pos = 0; bit < *out_bit_len; bit++, pos += 2) {
    uint8_t first = half_bits[pos];
    uint8_t second = half_bits[pos + 1];

    out_half_tick[bit] = half_tick[pos];
    if (first == 1U && second == 0U) {
      out_bits[bit] = 1U;
    } else if (first == 0U && second == 1U) {
      out_bits[bit] = 0U;
    } else {
      out_pair_bad[bit] = 1U;
      out_bits[bit] = second;
    }
  }

  return true;
}

static bool __attribute__((unused)) em4100_candidate_repeats_nearby(
    const uint8_t* bits, int len, int off, uint64_t code) {
  if (len < 128) return false;

  for (int frame_step = 1; frame_step <= 3; frame_step++) {
    const int centers[2] = {off + (64 * frame_step), off - (64 * frame_step)};

    for (int c = 0; c < 2; c++) {
      for (int delta = -2; delta <= 2; delta++) {
        int repeat_off = centers[c] + delta;
        uint64_t repeat_code;
        const char* repeat_reason;

        if (repeat_off < 0 || repeat_off + 64 > len || repeat_off == off) {
          continue;
        }

        if (em4100_candidate_from_parity(bits, repeat_off, &repeat_code,
                                         &repeat_reason) &&
            repeat_code == code) {
          return true;
        }
      }
    }
  }

  return false;
}

static bool em4100_candidate_from_repeated_frames(const uint8_t* bits, int len,
                                                  int off, uint64_t* out_code,
                                                  const char** reason,
                                                  int* out_frames,
                                                  int* out_weak_bits,
                                                  int* out_start_frame) {
  uint8_t voted[64];
  int total_frames;

  if (off < 0 || off + 64 > len) return false;
  if (off <= 0 || bits[off - 1] != 0) return false;

  total_frames = (len - off) / 64;
  if (total_frames < 3) return false;

  for (int start_frame = 0; start_frame < 1 && start_frame + 3 <= total_frames;
       start_frame++) {
    int max_frames = total_frames - start_frame;
    int frame_options[3] = {7, 5, 3};

    for (int option = 0; option < (int)ARRAY_SIZE(frame_options); option++) {
      int frames = frame_options[option];
      int weak_bits = 0;

      if (frames > max_frames) continue;

      for (int bit = 0; bit < 64; bit++) {
        int ones = 0;

        for (int frame = 0; frame < frames; frame++) {
          int idx = off + (start_frame + frame) * 64 + bit;
          ones += bits[idx] ? 1 : 0;
        }

        int zeros = frames - ones;
        if (ones == zeros ||
            (ones > zeros ? ones - zeros : zeros - ones) == 1) {
          weak_bits++;
        }
        voted[bit] = ones > zeros ? 1U : 0U;
      }

      if (weak_bits > 14) continue;

      const char* voted_reason;
      if (em4100_candidate_from_parity(voted, 0, out_code, &voted_reason)) {
        if (strcmp(voted_reason, "data2") == 0) continue;
      } else if (!em4100_candidate_from_relaxed_header_clean(voted, 0, out_code,
                                                             &voted_reason)) {
        continue;
      }

      *reason = "vote";
      *out_frames = frames;
      *out_weak_bits = weak_bits;
      *out_start_frame = start_frame;
      return true;
    }
  }

  return false;
}

static bool edgecap_decode_frame_from_halfbits(int half_len, int hb_start,
                                               int resync, uint8_t* out_bits,
                                               uint8_t* out_pair_bad,
                                               int* out_bad_pairs) {
  int pos = hb_start;
  int bad_pairs = 0;

  memset(out_bits, 0, 64);
  memset(out_pair_bad, 0, 64);

  for (int bit = 0; bit < 64; bit++) {
    if (pos + 1 >= half_len) return false;

    uint8_t first = edgecap_half_bits[pos];
    uint8_t second = edgecap_half_bits[pos + 1];
    bool bad_pair = false;

    if (first == 1U && second == 0U) {
      out_bits[bit] = 1U;
    } else if (first == 0U && second == 1U) {
      out_bits[bit] = 0U;
    } else {
      bad_pair = true;
      out_pair_bad[bit] = 1U;
      out_bits[bit] = second;
      bad_pairs++;
    }

    pos += (resync && bad_pair) ? 1 : 2;
  }

  *out_bad_pairs = bad_pairs;
  return true;
}

#if EM_ENABLE_TICK_FALLBACK_DIAG
static void __attribute__((unused)) log_em4100_best_score(const char* tag,
                                                          const uint8_t* bits,
                                                          int len) {
  int best_score = 1000;
  int best_off = -1;
  int best_header = 0;
  int best_row = 0;
  int best_col = 0;
  int best_stop = 0;
  uint64_t best_code = 0;

  if (len < 64) return;

  for (int off = 0; off <= len - 64; off++) {
    int header_err;
    int row_err;
    int col_err;
    int stop_err;
    int score = em4100_frame_error_score(bits, off, &header_err, &row_err,
                                         &col_err, &stop_err);

    if (score < best_score) {
      best_score = score;
      best_off = off;
      best_header = header_err;
      best_row = row_err;
      best_col = col_err;
      best_stop = stop_err;
      best_code = decode_card_code((uint8_t*)&bits[off]);
    }
  }

  LOG_INF("%s best: score=%d off=%d header=%d row=%d col=%d", tag, best_score,
          best_off, best_header, best_row, best_col);
  LOG_INF("%s best: stop=%d raw40=%010llX low16=%u", tag, best_stop,
          (unsigned long long)best_code, (uint16_t)(best_code & 0xFFFFULL));
  if (best_header <= 1 && best_row == 0 && best_col == 0 && best_stop == 0) {
    LOG_WRN("[NEAR-%s] header_err=%d raw40=%010llX low16=%u", tag, best_header,
            (unsigned long long)best_code, (uint16_t)(best_code & 0xFFFFULL));
  }
}
#endif

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

static uint8_t env_sample_edges_q8(int len, uint32_t pos_q8) {
  int idx = (int)((pos_q8 + 128U) >> 8);

  if (idx < 0) return env_edge_samples[0];
  if (idx >= len) return env_edge_samples[len - 1];
  return env_edge_samples[idx];
}

static int env_edge_sum_q8(int len, uint32_t start_q8, uint32_t width_q8) {
  uint32_t sixth_q8 = width_q8 / 6U;
  uint32_t center_q8 = width_q8 >> 1;
  int sum = 0;

  sum += env_sample_edges_q8(len, start_q8 + sixth_q8);
  sum += env_sample_edges_q8(len, start_q8 + center_q8);
  sum += env_sample_edges_q8(len, start_q8 + width_q8 - sixth_q8);
  return sum;
}

static int try_em4100_level_decode(int level_len, uint64_t* out_code,
                                   int* out_bits, int* out_half_windows,
                                   int* out_offset, bool* out_inverted,
                                   int* out_bad_pairs, bool log_best) {
  int best_score = 1000;
  int best_h = 0;
  int best_off = 0;
  int best_bits = 0;
  int best_bad = 0;
  int best_frame_off = -1;
  int best_inv = 0;
  int best_header = 0;
  int best_row = 0;
  int best_col = 0;
  int best_stop = 0;
  uint64_t best_code = 0;

  for (int half_windows = EM_ENV_HALF_MIN_WINDOWS;
       half_windows <= EM_ENV_HALF_MAX_WINDOWS; half_windows++) {
    const int bit_windows = half_windows * 2;

    for (int offset = 0; offset < bit_windows; offset++) {
      int bit_len = 0;

      memset(dec_raw_bits, 0, EM_DECODED_BITS_CAP);
      memset(dec_pair_bad, 0, EM_DECODED_BITS_CAP);
      for (int pos = offset;
           pos + bit_windows <= level_len && bit_len < EM_DECODED_BITS_CAP;
           pos += bit_windows) {
        uint8_t first =
            env_majority_level(env_level_samples, pos, half_windows);
        uint8_t second = env_majority_level(env_level_samples,
                                            pos + half_windows, half_windows);

        if (first == second) {
          dec_pair_bad[bit_len] = 1;
        }
        /* EM4100 polarity can be inverted later; preserve bit clock even for
         * bad pairs. */
        dec_raw_bits[bit_len++] = second;
      }

      if (bit_len < 64) continue;

      for (int inv = 0; inv <= 1; inv++) {
        if (inv) invert_bits(dec_raw_bits, bit_len);
        for (int off = 0; off <= bit_len - 64; off++) {
          uint64_t candidate_code;
          const char* candidate_reason;
          int header_err;
          int row_err;
          int col_err;
          int stop_err;
          int frame_bad = count_bad_pairs_in_frame(off);
          int score = em4100_frame_error_score(dec_raw_bits, off, &header_err,
                                               &row_err, &col_err, &stop_err);
          if (score < best_score) {
            best_score = score;
            best_h = half_windows;
            best_off = offset;
            best_bits = bit_len;
            best_bad = frame_bad;
            best_frame_off = off;
            best_inv = inv;
            best_header = header_err;
            best_row = row_err;
            best_col = col_err;
            best_stop = stop_err;
            best_code = decode_card_code((uint8_t*)&dec_raw_bits[off]);
          }

          if (frame_bad <= EM_LEVEL_MAX_FRAME_BAD_PAIRS &&
              em4100_candidate_from_parity(dec_raw_bits, off, &candidate_code,
                                           &candidate_reason) &&
              strcmp(candidate_reason, "data2") != 0) {
            if (confirm_em4100_candidate(candidate_code, candidate_reason)) {
              *out_code = candidate_code;
              *out_bits = bit_len;
              *out_half_windows = half_windows;
              *out_offset = offset;
              *out_inverted = inv != 0;
              *out_bad_pairs = frame_bad;
              return off;
            }
          }
        }
        if (inv) invert_bits(dec_raw_bits, bit_len);
      }

      int frame_bad = 0;
      int slide =
          try_em4100_level_slide(dec_raw_bits, bit_len, out_code, &frame_bad);
      if (slide >= 0) {
        *out_bits = bit_len;
        *out_half_windows = half_windows;
        *out_offset = offset;
        *out_inverted = false;
        *out_bad_pairs = frame_bad;
        return slide;
      }

      invert_bits(dec_raw_bits, bit_len);
      slide =
          try_em4100_level_slide(dec_raw_bits, bit_len, out_code, &frame_bad);
      if (slide >= 0) {
        *out_bits = bit_len;
        *out_half_windows = half_windows;
        *out_offset = offset;
        *out_inverted = true;
        *out_bad_pairs = frame_bad;
        return slide;
      }
    }
  }

  if (!log_best) return -1;

  LOG_INF("level_best: score=%d h=%d off=%d frame=%d inv=%d", best_score,
          best_h, best_off, best_frame_off, best_inv);
  LOG_INF("level_best: bits=%d bad=%d header=%d row=%d col=%d stop=%d",
          best_bits, best_bad, best_header, best_row, best_col, best_stop);
  LOG_INF("level_best: raw40=%010llX low16=%u", (unsigned long long)best_code,
          (uint16_t)(best_code & 0xFFFFULL));
  if (best_header <= 1 && best_row == 0 && best_col == 0 && best_stop == 0) {
    LOG_WRN("[NEAR-Level h=%d off=%d frame=%d inv=%d] header_err=%d", best_h,
            best_off, best_frame_off, best_inv, best_header);
    LOG_WRN("[NEAR-Level] raw40=%010llX low16=%u",
            (unsigned long long)best_code, (uint16_t)(best_code & 0xFFFFULL));
  }
  return -1;
}

/* frame_bad≤max：先翻轉曼徹斯特壞點再跑 EM4100 parity 救援 */
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
  printk("!!! [HID PROX MATCH] FC: %u, CN: %u, w26_raw: %08llX (inv=%d, "
         "half=%u)\n",
         hid_fc, hid_cn, (unsigned long long)hid_raw44, inv,
         (unsigned)half_q8);

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
      uint8_t first = env_majority_level_q8(env_level_samples, level_len,
                                            pos_q8, best_half_q8);
      uint8_t second = env_majority_level_q8(
          env_level_samples, level_len, pos_q8 + best_half_q8, best_half_q8);
      dec_raw_bits[bit_len++] = second;
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

static void rfid_saadc_candidate_consensus(const char* tag, uint64_t code,
                                           int score, int bad) {
  /* 連續一致候選即報告（展示/診斷用）。 */
  static uint64_t last_code;
  static uint8_t streak;
  static int last_score;
  static int last_bad;

  if (code == 0 || score >= 1000 || bad >= 1000) {
    streak = 0;
    last_code = 0;
    return;
  }

  if (code == last_code) {
    if (streak < UINT8_MAX) streak++;
    if (score < last_score) last_score = score;
    if (bad < last_bad) last_bad = bad;
  } else {
    last_code = code;
    streak = 1;
    last_score = score;
    last_bad = bad;
  }

#if RFID_DECODE_DIAG_LOG
  if (streak >= 3) {
    LOG_WRN("[CANDIDATE-SAADC tag=%s streak=%u score<=%d bad<=%d] ID: %010llX",
            tag, (unsigned)streak, last_score, last_bad,
            (unsigned long long)last_code);
  }
#endif
}

static int try_em4100_fractional_level_decode(
    int level_len, uint64_t* out_code, int* out_bits, int* out_half_windows,
    int* out_offset, bool* out_inverted, int* out_bad_pairs, bool log_best,
    const char* tag) {
  int best_score = 1000;
  uint32_t best_half_q8 = 0;
  uint32_t best_offset_q8 = 0;
  int best_bits = 0;
  int best_bad = 0;
  int best_frame_off = -1;
  int best_inv = 0;
  int best_header = 0;
  int best_row = 0;
  int best_col = 0;
  int best_stop = 0;
  uint64_t best_code = 0;
  int noisy_bad = 1000;
  int noisy_bits = 0;
  uint32_t noisy_half_q8 = 0;
  uint32_t noisy_offset_q8 = 0;
  int noisy_frame_off = -1;
  int noisy_inv = 0;
  uint64_t noisy_code = 0;
  const char* noisy_reason = NULL;

  if (level_len < 128) return -1;

  const uint32_t level_limit_q8 = (uint32_t)(level_len - 1) << 8;

  for (uint32_t half_q8 = EM_FRAC_HALF_MIN_Q8; half_q8 <= EM_FRAC_HALF_MAX_Q8;
       half_q8 += EM_FRAC_HALF_STEP_Q8) {
    const uint32_t bit_q8 = half_q8 * 2U;

    for (uint32_t offset_q8 = 0; offset_q8 < bit_q8;
         offset_q8 += EM_FRAC_OFFSET_STEP_Q8) {
      int bit_len = 0;

      memset(dec_raw_bits, 0, EM_DECODED_BITS_CAP);
      memset(dec_pair_bad, 0, EM_DECODED_BITS_CAP);

      for (uint32_t pos_q8 = offset_q8;
           pos_q8 + bit_q8 <= level_limit_q8 && bit_len < EM_DECODED_BITS_CAP;
           pos_q8 += bit_q8) {
        uint8_t first = env_majority_level_q8(env_level_samples, level_len,
                                              pos_q8, half_q8);
        uint8_t second = env_majority_level_q8(env_level_samples, level_len,
                                               pos_q8 + half_q8, half_q8);

        if (first == second) {
          dec_pair_bad[bit_len] = 1;
        }
        dec_raw_bits[bit_len++] = second;
      }

      if (bit_len < 64) continue;

      for (int inv = 0; inv <= 1; inv++) {
        if (inv) invert_bits(dec_raw_bits, bit_len);

        for (int off = 0; off <= bit_len - 64; off++) {
          uint64_t candidate_code;
          const char* candidate_reason;
          int header_err;
          int row_err;
          int col_err;
          int stop_err;
          int frame_bad = count_bad_pairs_in_frame(off);
          int score = em4100_frame_error_score(dec_raw_bits, off, &header_err,
                                               &row_err, &col_err, &stop_err);
          uint64_t code = decode_card_code((uint8_t*)&dec_raw_bits[off]);

          if (score < best_score) {
            best_score = score;
            best_half_q8 = half_q8;
            best_offset_q8 = offset_q8;
            best_bits = bit_len;
            best_bad = frame_bad;
            best_frame_off = off;
            best_inv = inv;
            best_header = header_err;
            best_row = row_err;
            best_col = col_err;
            best_stop = stop_err;
            best_code = code;
          }

          if (frame_bad <= EM_LEVEL_MAX_FRAME_BAD_PAIRS &&
              em4100_candidate_from_parity(dec_raw_bits, off, &candidate_code,
                                           &candidate_reason) &&
              strcmp(candidate_reason, "data2") != 0) {
            if (confirm_em4100_candidate(candidate_code, candidate_reason)) {
              *out_code = candidate_code;
              *out_bits = bit_len;
              *out_half_windows = (int)half_q8;
              *out_offset = (int)offset_q8;
              *out_inverted = inv != 0;
              *out_bad_pairs = frame_bad;
              return off;
            }
          }

          if (score == 0 && frame_bad < noisy_bad &&
              frame_bad <= EM_NOISY_MAX_FRAME_BAD_PAIRS) {
            noisy_bad = frame_bad;
            noisy_bits = bit_len;
            noisy_half_q8 = half_q8;
            noisy_offset_q8 = offset_q8;
            noisy_frame_off = off;
            noisy_inv = inv;
            noisy_code = code;
            noisy_reason = "frac-noisy-clean";
          }
        }

        if (inv) invert_bits(dec_raw_bits, bit_len);
      }
    }
  }

  if (noisy_reason != NULL) {
#if RFID_DECODE_DIAG_LOG
    LOG_WRN("[NOISY-%s] bad=%d raw40=%010llX low16=%u", noisy_reason, noisy_bad,
            (unsigned long long)noisy_code, (uint16_t)(noisy_code & 0xFFFFULL));
#endif
    if (confirm_em4100_candidate(noisy_code, noisy_reason)) {
      *out_code = noisy_code;
      *out_bits = noisy_bits;
      *out_half_windows = (int)noisy_half_q8;
      *out_offset = (int)noisy_offset_q8;
      *out_inverted = noisy_inv != 0;
      *out_bad_pairs = noisy_bad;
      return noisy_frame_off;
    }
  }

  if (log_best) {
    LOG_INF("%s_best: score=%d half_q8=%u off_q8=%u frame=%d inv=%d", tag,
            best_score, best_half_q8, best_offset_q8, best_frame_off, best_inv);
    LOG_INF("%s_best: bits=%d bad=%d header=%d row=%d col=%d stop=%d", tag,
            best_bits, best_bad, best_header, best_row, best_col, best_stop);
    LOG_INF("%s_best: raw40=%010llX low16=%u", tag,
            (unsigned long long)best_code, (uint16_t)(best_code & 0xFFFFULL));
  }

  return -1;
}

static int try_em4100_clocked_resync_decode(int level_len, uint64_t* out_code,
                                            int* out_bits,
                                            int* out_half_windows,
                                            int* out_offset, bool* out_inverted,
                                            int* out_bad_pairs, bool log_best) {
  int best_score = 1000;
  uint32_t best_half_q8 = 0;
  uint32_t best_offset_q8 = 0;
  int best_bits = 0;
  int best_bad = 0;
  int best_frame_off = -1;
  int best_inv = 0;
  int best_header = 0;
  int best_row = 0;
  int best_col = 0;
  int best_stop = 0;
  uint64_t best_code = 0;

  if (level_len < 128) return -1;

  const uint32_t level_limit_q8 = (uint32_t)(level_len - 1) << 8;

  for (uint32_t half_q8 = EM_FRAC_HALF_MIN_Q8; half_q8 <= EM_FRAC_HALF_MAX_Q8;
       half_q8 += EM_FRAC_HALF_STEP_Q8) {
    const uint32_t bit_q8 = half_q8 * 2U;

    for (uint32_t offset_q8 = 0; offset_q8 < bit_q8;
         offset_q8 += EM_FRAC_OFFSET_STEP_Q8) {
      int bit_len = 0;
      uint32_t pos_q8 = offset_q8;

      memset(dec_raw_bits, 0, EM_DECODED_BITS_CAP);
      memset(dec_pair_bad, 0, EM_DECODED_BITS_CAP);

      while (pos_q8 + bit_q8 <= level_limit_q8 &&
             bit_len < EM_DECODED_BITS_CAP) {
        uint8_t first = env_majority_level_q8(env_level_samples, level_len,
                                              pos_q8, half_q8);
        uint8_t second = env_majority_level_q8(env_level_samples, level_len,
                                               pos_q8 + half_q8, half_q8);
        bool bad_pair = first == second;

        if (bad_pair) dec_pair_bad[bit_len] = 1;
        dec_raw_bits[bit_len++] = second;

        /* EM4095-style resync: when the sampled bit repeats unexpectedly,
         * shift the next sampling point by a half bit instead of staying
         * permanently out of phase. */
        pos_q8 += bad_pair ? half_q8 : bit_q8;
      }

      if (bit_len < 64) continue;

      for (int inv = 0; inv <= 1; inv++) {
        if (inv) invert_bits(dec_raw_bits, bit_len);

        for (int off = 0; off <= bit_len - 64; off++) {
          uint64_t candidate_code;
          const char* candidate_reason;
          int header_err;
          int row_err;
          int col_err;
          int stop_err;
          int frame_bad = count_bad_pairs_in_frame(off);
          int score = em4100_frame_error_score(dec_raw_bits, off, &header_err,
                                               &row_err, &col_err, &stop_err);
          uint64_t code = decode_card_code((uint8_t*)&dec_raw_bits[off]);

          if (score < best_score) {
            best_score = score;
            best_half_q8 = half_q8;
            best_offset_q8 = offset_q8;
            best_bits = bit_len;
            best_bad = frame_bad;
            best_frame_off = off;
            best_inv = inv;
            best_header = header_err;
            best_row = row_err;
            best_col = col_err;
            best_stop = stop_err;
            best_code = code;
          }

          if (frame_bad <= EM_LEVEL_MAX_FRAME_BAD_PAIRS &&
              em4100_candidate_from_parity(dec_raw_bits, off, &candidate_code,
                                           &candidate_reason)) {
            if (confirm_em4100_candidate(candidate_code, candidate_reason)) {
              *out_code = candidate_code;
              *out_bits = bit_len;
              *out_half_windows = (int)half_q8;
              *out_offset = (int)offset_q8;
              *out_inverted = inv != 0;
              *out_bad_pairs = frame_bad;
              return off;
            }
          }
        }

        if (inv) invert_bits(dec_raw_bits, bit_len);
      }
    }
  }

  if (log_best) {
    LOG_INF("clocked_best: score=%d half_q8=%u off_q8=%u frame=%d inv=%d",
            best_score, best_half_q8, best_offset_q8, best_frame_off, best_inv);
    LOG_INF("clocked_best: bits=%d bad=%d header=%d row=%d col=%d stop=%d",
            best_bits, best_bad, best_header, best_row, best_col, best_stop);
    LOG_INF("clocked_best: raw40=%010llX low16=%u",
            (unsigned long long)best_code, (uint16_t)(best_code & 0xFFFFULL));
  }

  return -1;
}

/* ============================================================
 * 邊緣計時 EM4100 Manchester 解碼器
 * 不需要預先知道 bit rate，直接從 edge 間距推算 T/2。
 * 適用 RF/64(T/2=2win=256µs) 和 RF/96(T/2=3win=384µs) 等不同速率。
 *
 * Manchester 規則（Thomas convention，EM4100 採用）：
 *   "1" bit = 前半 HIGH、後半 LOW；mid-bit edge = H→L
 *   "0" bit = 前半 LOW、後半 HIGH；mid-bit edge = L→H
 *   同類 bit 相連 → bit boundary 有 edge（SHORT 間距）
 *   不同類 bit 相連 → boundary 無 edge（LONG 間距 ≈ T = 2×T/2）
 * ============================================================ */
static void saadc_edge_timing_decode_log(int level_len) {
#if !RFID_DECODE_DIAG_LOG
  return;
#endif
  if (level_len < 64) return;

  /* ---- Step 1: 門限 127，找出所有邊緣 ---- */
  int n_edges = 0;
  uint8_t prev = env_edge_samples[0] > 127U ? 1U : 0U;
  for (int i = 1; i < level_len && n_edges < 799; i++) {
    uint8_t cur = env_edge_samples[i] > 127U ? 1U : 0U;
    if (cur != prev) {
      em_et_edge_pos[n_edges] = (int16_t)i;
      em_et_edge_dir[n_edges] = (int8_t)(cur ? 1 : -1);
      n_edges++;
    }
    prev = cur;
  }
  if (n_edges < 16) {
    LOG_WRN("!!! [EDGE-DEC] edges=%d too few (threshold=127)", n_edges);
    return;
  }

  /* ---- Step 2: 統計間距直方圖，找出 T/2 ---- */
  int hist[16] = {0};
  for (int i = 1; i < n_edges; i++) {
    int iv = em_et_edge_pos[i] - em_et_edge_pos[i - 1];
    if (iv >= 1 && iv < 16) hist[iv]++;
  }
  int half_t = 2;
  int max_cnt = 0;
  for (int t = 1; t <= 8; t++) {
    if (hist[t] > max_cnt) {
      max_cnt = hist[t];
      half_t = t;
    }
  }
  /* 印出間距分佈（bins 1..10） */
  LOG_WRN(
      "!!! [EDGE-DEC] edges=%d half_t=%d (cnt=%d) hist:"
      " 1=%d 2=%d 3=%d 4=%d 5=%d 6=%d 7=%d 8=%d 9=%d 10=%d",
      n_edges, half_t, max_cnt, hist[1], hist[2], hist[3], hist[4], hist[5],
      hist[6], hist[7], hist[8], hist[9], hist[10]);

  /* SHORT 門限：< 1.5 × T/2；LONG：≥ 1.5 × T/2 */
  /* 整數版：SHORT if (2*iv) < 3*half_t */
  int short_limit2 = 3 * half_t; /* 2×interval < 3×half_t → SHORT */

  /* ---- Step 3: 嘗試兩種起始 edge (0 = 假設 mid-bit, 1 = 假設從 boundary 後)
   * ---- */
  int best_row_err = 99, best_col_err = 99;
  uint64_t best_code = 0;
  int best_half_t = half_t, best_n_bits = 0;

  for (int start_ei = 0; start_ei <= 1; start_ei++) {
    /* 解碼 bit 串 */
    int n_bits = 0;
    int ei = start_ei;
    while (ei < n_edges && n_bits < 799) {
      /* 視 ei 為 mid-bit edge */
      uint8_t bit = (em_et_edge_dir[ei] < 0) ? 1U : 0U; /* H→L="1", L→H="0" */
      em_et_bits[n_bits++] = bit;

      if (ei + 1 >= n_edges) break;
      int iv = em_et_edge_pos[ei + 1] - em_et_edge_pos[ei];

      if ((2 * iv) < short_limit2) {
        /* SHORT：同類 bit → boundary edge 存在，跳過 boundary */
        ei += 2;
      } else {
        /* LONG：不同類 bit → 直接到下一個 mid-bit edge */
        ei += 1;
      }
    }
    if (n_bits < 64) continue;

    /* ---- Step 4: 在 bit 串中搜尋 EM4100 header (9+ 個 "1") ---- */
    int found_in_stream = 0;
    for (int s = 0; s + 64 <= n_bits; s++) {
      int hdr = 0;
      while (s + hdr < n_bits && em_et_bits[s + hdr]) hdr++;
      if (hdr < 9) continue;

      int P = s + hdr;
      if (P + 55 > n_bits) continue;

      /* 解 10 rows × 5 bits + 5 col/stop */
      uint8_t rd[10][4];
      int row_err = 0;
      for (int r = 0; r < 10; r++) {
        uint8_t par = 0;
        for (int b = 0; b < 4; b++) {
          rd[r][b] = em_et_bits[P + r * 5 + b];
          par ^= rd[r][b];
        }
        if (par != em_et_bits[P + r * 5 + 4]) row_err++;
      }
      int col_err = 0;
      for (int c = 0; c < 4; c++) {
        uint8_t cp = 0;
        for (int r = 0; r < 10; r++) cp ^= rd[r][c];
        if (cp != em_et_bits[P + 50 + c]) col_err++;
      }

      uint64_t code = 0;
      for (int r = 0; r < 10; r++) {
        for (int b = 0; b < 4; b++) code = (code << 1) | rd[r][b];
      }
      uint16_t low16 = (uint16_t)(code & 0xFFFFULL);

      /* 只印 parity 相對好的候選（≤ 3 total errors） */
      if (row_err + col_err <= 4) {
        LOG_WRN(
            "!!! [EDGE-DEC] si=%d bits=%d hdr=%d P=%d"
            " row=%d col=%d raw40=%010llX low16=%04X",
            start_ei, n_bits, hdr, P, row_err, col_err,
            (unsigned long long)code, (unsigned int)low16);
        if (low16 == EM_ALIGN_TARGET_LOW16) {
          LOG_WRN("!!! [EDGE-MATCH] TARGET 0x%04X FOUND by edge-timing!",
                  EM_ALIGN_TARGET_LOW16);
        }
        if (row_err + col_err < best_row_err + best_col_err) {
          best_row_err = row_err;
          best_col_err = col_err;
          best_code = code;
          best_n_bits = n_bits;
          best_half_t = half_t;
        }
        found_in_stream++;
        if (found_in_stream >= 4) break;
      }
    }
    if (!found_in_stream && start_ei == 1) {
      LOG_WRN("!!! [EDGE-DEC] si=%d bits=%d no valid frame", start_ei, n_bits);
    }
  } /* end start_ei loop */

  if (best_row_err + best_col_err <= 3) {
    LOG_WRN(
        "!!! [EDGE-BEST] half_t=%d bits=%d row=%d col=%d"
        " raw40=%010llX low16=%04X",
        best_half_t, best_n_bits, best_row_err, best_col_err,
        (unsigned long long)best_code, (unsigned int)(best_code & 0xFFFFULL));
  }
}

static int try_em4100_edge_delta_decode(int level_len, uint64_t* out_code,
                                        int* out_bits, int* out_half_windows,
                                        int* out_offset, bool* out_inverted,
                                        int* out_bad_pairs, bool log_best) {
  int best_score = 1000;
  uint32_t best_half_q8 = 0;
  uint32_t best_offset_q8 = 0;
  int best_bits = 0;
  int best_bad = 0;
  int best_frame_off = -1;
  int best_inv = 0;
  int best_header = 0;
  int best_row = 0;
  int best_col = 0;
  int best_stop = 0;
  uint64_t best_code = 0;
  int best_bits_snapshot_len = 0;
  uint32_t best_half_q8_snap = 0;
  int best_inv_snap = 0;
  uint32_t best_off_q8_snap = 0;
  int inv_best_hits[2] = {0, 0};
  int inv_parity_hits[2] = {0, 0};
  int inv_bad_sum[2] = {0, 0};

  if (level_len < 128) return -1;

  const uint32_t level_limit_q8 = (uint32_t)(level_len - 1) << 8;

  const uint32_t delta_half_start_q8 =
#if EM_DELTA_DEBUG_RAW_MODE
      EM_DELTA_DEBUG_HALF_Q8;
#else
      EM_DELTA_HALF_MIN_Q8;
#endif
  const uint32_t delta_half_end_q8 =
#if EM_DELTA_DEBUG_RAW_MODE
      EM_DELTA_DEBUG_HALF_Q8;
#else
      EM_DELTA_HALF_MAX_Q8;
#endif

  for (uint32_t half_q8 = delta_half_start_q8; half_q8 <= delta_half_end_q8;
       half_q8 += EM_DELTA_HALF_STEP_Q8) {
    const uint32_t bit_q8 = half_q8 * 2U;

    for (uint32_t offset_q8 =
#if EM_DELTA_DEBUG_RAW_MODE
             EM_DELTA_DEBUG_OFF_Q8;
#else
             0;
#endif
         offset_q8 <
#if EM_DELTA_DEBUG_RAW_MODE
         (EM_DELTA_DEBUG_OFF_Q8 + 1U);
#else
         bit_q8;
#endif
         offset_q8 += EM_DELTA_OFFSET_STEP_Q8) {
      int bit_len = 0;

      memset(dec_raw_bits, 0, EM_DECODED_BITS_CAP);
      memset(dec_pair_bad, 0, EM_DECODED_BITS_CAP);

      for (uint32_t pos_q8 = offset_q8;
           pos_q8 + bit_q8 <= level_limit_q8 && bit_len < EM_DECODED_BITS_CAP;
           pos_q8 += bit_q8) {
        int first_sum = env_edge_sum_q8(level_len, pos_q8, half_q8);
        int second_sum = env_edge_sum_q8(level_len, pos_q8 + half_q8, half_q8);
        int diff = second_sum - first_sum;

        if (diff < 0) diff = -diff;
        if (diff <= EM_DELTA_MIN_DIFF) {
          dec_pair_bad[bit_len] = 1;
        }
        dec_raw_bits[bit_len++] = second_sum > first_sum ? 1U : 0U;
      }

      if (bit_len < 64) continue;

      for (int inv = 0; inv <= 1; inv++) {
        if (inv) invert_bits(dec_raw_bits, bit_len);
        int pass_begin = 0;
        int pass_end = 1;
        int off_min = 0;
        int off_max = bit_len - 64;
        bool have_focus = em_delta_tracked_frame_off >= off_min &&
                          em_delta_tracked_frame_off <= off_max;
        if (!have_focus) {
          pass_begin = 1;
          pass_end = 1;
        }

        for (int pass = pass_begin; pass <= pass_end; pass++) {
          int focus_lo = off_min;
          int focus_hi = off_max;

          if (pass == 0) {
            focus_lo = em_delta_tracked_frame_off - EM_DELTA_TRACK_OFF_WINDOW;
            focus_hi = em_delta_tracked_frame_off + EM_DELTA_TRACK_OFF_WINDOW;
            if (focus_lo < off_min) focus_lo = off_min;
            if (focus_hi > off_max) focus_hi = off_max;
          }

          for (int off = focus_lo; off <= focus_hi; off++) {
            if (pass == 1 && have_focus) {
              int d = off - em_delta_tracked_frame_off;
              if (d < 0) d = -d;
              if (d <= EM_DELTA_TRACK_OFF_WINDOW) {
                continue;
              }
            }

            uint64_t candidate_code;
            const char* candidate_reason;
            int header_err;
            int row_err;
            int col_err;
            int stop_err;
            int frame_bad = count_bad_pairs_in_frame(off);
            int score = em4100_frame_error_score(dec_raw_bits, off, &header_err,
                                                 &row_err, &col_err, &stop_err);
            uint64_t code = decode_card_code((uint8_t*)&dec_raw_bits[off]);

            if (header_err <= 1 && !RFID_SAADC_LOG_IMPORTANT_ONLY) {
              LOG_WRN(
                  "[FRAME-CANDIDATE] off=%d inv=%d raw40=%010llX hdr=%d "
                  "row=%d "
                  "col=%d stop=%d bad=%d score=%d",
                  off, inv, (unsigned long long)code, header_err, row_err,
                  col_err, stop_err, frame_bad, score);
            }

            inv_bad_sum[inv] += frame_bad;

            if (score < best_score) {
              best_score = score;
              best_half_q8 = half_q8;
              best_offset_q8 = offset_q8;
              best_bits = bit_len;
              best_bad = frame_bad;
              best_frame_off = off;
              best_inv = inv;
              best_header = header_err;
              best_row = row_err;
              best_col = col_err;
              best_stop = stop_err;
              best_code = code;
              inv_best_hits[inv]++;
              memcpy(em_delta_best_bits_snapshot, dec_raw_bits,
                     (size_t)bit_len);
              best_bits_snapshot_len = bit_len;
              best_half_q8_snap = half_q8;
              best_inv_snap = inv;
              best_off_q8_snap = offset_q8;
            }

            if (frame_bad <= EM_LEVEL_MAX_FRAME_BAD_PAIRS &&
                em4100_candidate_from_parity(dec_raw_bits, off, &candidate_code,
                                             &candidate_reason)) {
              inv_parity_hits[inv]++;
              if (confirm_em4100_candidate(candidate_code, candidate_reason)) {
                *out_code = candidate_code;
                *out_bits = bit_len;
                *out_half_windows = (int)half_q8;
                *out_offset = (int)offset_q8;
                *out_inverted = inv != 0;
                *out_bad_pairs = frame_bad;
                em_delta_tracked_frame_off = off;
#if EM_DELTA_DEBUG_RAW_MODE
                LOG_WRN("[DEBUG-RAW] raw40=%010llX bits=%d off=%d inv=%d",
                        (unsigned long long)candidate_code, bit_len, off, inv);
                if (inv) invert_bits(dec_raw_bits, bit_len);
                return -1;
#else
                return off;
#endif
              }
            }
          }
        }

        if (inv) invert_bits(dec_raw_bits, bit_len);
      }
    }
  }

  if (log_best) {
    LOG_INF("delta_best: score=%d half_q8=%u off_q8=%u frame=%d inv=%d",
            best_score, best_half_q8, best_offset_q8, best_frame_off, best_inv);
    LOG_INF("delta_best: bits=%d bad=%d header=%d row=%d col=%d stop=%d",
            best_bits, best_bad, best_header, best_row, best_col, best_stop);
    LOG_INF("delta_best: raw40=%010llX low16=%u", (unsigned long long)best_code,
            (uint16_t)(best_code & 0xFFFFULL));
    LOG_WRN(
        "delta_inv_stats: inv0 best=%d parity=%d bad_sum=%d | inv1 best=%d "
        "parity=%d bad_sum=%d",
        inv_best_hits[0], inv_parity_hits[0], inv_bad_sum[0], inv_best_hits[1],
        inv_parity_hits[1], inv_bad_sum[1]);
    if (best_bits_snapshot_len >= 64) {
      int h_off = find_header_offset(em_delta_best_bits_snapshot,
                                     best_bits_snapshot_len);
      if (h_off >= 0) {
        uint64_t code =
            decode_card_code((uint8_t*)&em_delta_best_bits_snapshot[h_off]);
        LOG_WRN("!!! [MANUAL-DECODE] Found Header at offset=%d, ID=%010llX",
                h_off, (unsigned long long)code);
        (void)confirm_em4100_candidate(code, "manual-delta");
      } else {
        LOG_WRN(
            "!!! [MANUAL-DECODE] No header found. First bits: "
            "%d%d%d%d%d%d%d%d",
            em_delta_best_bits_snapshot[0], em_delta_best_bits_snapshot[1],
            em_delta_best_bits_snapshot[2], em_delta_best_bits_snapshot[3],
            em_delta_best_bits_snapshot[4], em_delta_best_bits_snapshot[5],
            em_delta_best_bits_snapshot[6], em_delta_best_bits_snapshot[7]);
      }
#if EM_DELTA_DEBUG_RAW_MODE
      char raw_bits_inv[65];
      uint16_t head_hex = 0;

      memcpy(em_delta_inv_bits, em_delta_best_bits_snapshot,
             (size_t)best_bits_snapshot_len);
      invert_bits(em_delta_inv_bits, best_bits_snapshot_len);
      for (int k = 0; k < 64; k++) {
        raw_bits_inv[k] = em_delta_inv_bits[k] ? '1' : '0';
      }
      raw_bits_inv[64] = '\0';
      for (int k = 0; k < 16; k++) {
        head_hex =
            (uint16_t)((head_hex << 1) | (em_delta_inv_bits[k] ? 1U : 0U));
      }
      LOG_WRN("!!! [RAW-BITS-INV] %s", raw_bits_inv);
      LOG_WRN("!!! [RAW-HEAD-HEX] %04X", head_hex);
#endif
    }
    if (best_frame_off >= 0) {
      em_delta_tracked_frame_off = best_frame_off;
    }
    em_delta_shift_log_div++;
    if (best_frame_off >= 0 && best_bits_snapshot_len >= 64 &&
        (em_delta_shift_log_div % EM_DELTA_SHIFT_LOG_EVERY) == 0U) {
      em4100_log_shift_window(em_delta_best_bits_snapshot,
                              best_bits_snapshot_len, best_frame_off, "delta");
    }
    if (best_frame_off >= 0 && best_bits_snapshot_len >= 64) {
      /* 暴力位移掃描：0~127 shift，命中目標 ID 就印出 */
      for (int shift = 0; shift < 128; shift++) {
        uint64_t test_code = 0ULL;
        for (int i = 0; i < 40; i++) {
          int idx = (best_frame_off + shift + i) % best_bits_snapshot_len;
          test_code = (test_code << 1) |
                      (em_delta_best_bits_snapshot[idx] ? 1ULL : 0ULL);
        }
        uint16_t low16 = (uint16_t)(test_code & 0xFFFFULL);
        uint16_t low16_rev = reverse16(low16);
        if ((low16 == EM_ALIGN_TARGET_LOW16) ||
            (low16_rev == EM_ALIGN_TARGET_LOW16)) {
          LOG_WRN(
              "!!! [FOUND-ALIGNMENT] shift=%d, code=%010llX low16=%04X "
              "low16_rev=%04X",
              shift, (unsigned long long)test_code, (unsigned int)low16,
              (unsigned int)low16_rev);
          /* 完整循環掃描 snapshot 所有起點（兩種極性）。
           * 重要：FOUND-ALIGNMENT 的 low16=500F 是比對「原始 40
           * 位元」的最後16位元， 這不等於解碼後的卡片 ID
           * low16！所以要分開追蹤： (A) 全局最佳 score frame（任何 ID） (B)
           * 解碼後 low16==TARGET 的最佳 score frame 兩者都回報，(B)
           * 才是真正找到目標卡。 */
          {
            int fa_len2 = best_bits_snapshot_len;
            int fa_scan_found = 0;
            /* (A) 全局最佳 */
            int fa_best_sc = 99, fa_best_s = -1, fa_best_pol = 0;
            uint64_t fa_best_code = 0;
            /* (B) 目標 ID 最佳 */
            int fa_tgt_sc = 99, fa_tgt_s = -1, fa_tgt_pol = 0;
            uint64_t fa_tgt_code = 0;
            for (int fa_pol = 0; fa_pol <= 1; fa_pol++) {
              for (int fa_s = 0; fa_s < fa_len2; fa_s++) {
                int fa_hok = 1;
                for (int k = 0; k < 9 && fa_hok; k++) {
                  if ((em_delta_best_bits_snapshot[(fa_s + k) % fa_len2] ^
                       fa_pol) == 0)
                    fa_hok = 0;
                }
                if (!fa_hok) continue;
                uint8_t fa_f[64];
                for (int k = 0; k < 64; k++)
                  fa_f[k] = em_delta_best_bits_snapshot[(fa_s + k) % fa_len2] ^
                            (uint8_t)fa_pol;
                int fa_row = 0;
                uint8_t fa_cs[4] = {0, 0, 0, 0};
                for (int r = 0; r < 10; r++) {
                  int b = 9 + r * 5;
                  uint8_t p = 0;
                  for (int c = 0; c < 4; c++) {
                    p ^= fa_f[b + c];
                    fa_cs[c] ^= fa_f[b + c];
                  }
                  if (p != fa_f[b + 4]) fa_row++;
                }
                int fa_col = 0;
                for (int c = 0; c < 4; c++)
                  if (fa_cs[c] != fa_f[59 + c]) fa_col++;
                int fa_sc = fa_row + fa_col + (fa_f[63] ? 1 : 0);
                uint64_t fc = decode_card_code(fa_f);
                uint16_t fl16 = (uint16_t)(fc & 0xFFFFULL);
                /* (A) 追蹤全域最佳 score */
                if (fa_sc < fa_best_sc) {
                  fa_best_sc = fa_sc;
                  fa_best_s = fa_s;
                  fa_best_pol = fa_pol;
                  fa_best_code = fc;
                }
                /* (B) 追蹤解碼後 low16=TARGET 的最佳 score */
                if (fl16 == EM_ALIGN_TARGET_LOW16 && fa_sc < fa_tgt_sc) {
                  fa_tgt_sc = fa_sc;
                  fa_tgt_s = fa_s;
                  fa_tgt_pol = fa_pol;
                  fa_tgt_code = fc;
                }
                /* score=0 直接 confirm */
                if (fa_sc == 0 && fa_scan_found < 4) {
                  LOG_WRN("!!! [FA-SCAN p=%d s=%d] ID=%010llX l16=%04X", fa_pol,
                          fa_s, (unsigned long long)fc, (unsigned int)fl16);
                  fa_scan_found++;
                  (void)confirm_em4100_candidate(fc, "fa-scan");
                }
              }
            }
            /* 目標卡：回報解碼後 low16=TARGET 的最佳 frame（不管 score） */
            if (fa_tgt_s >= 0) {
              uint16_t tl16 = (uint16_t)(fa_tgt_code & 0xFFFFULL);
              LOG_WRN("!!! [FA-TARGET sc=%d p=%d s=%d] ID=%010llX l16=%04X",
                      fa_tgt_sc, fa_tgt_pol, fa_tgt_s,
                      (unsigned long long)fa_tgt_code, (unsigned int)tl16);
              if (fa_tgt_sc == 0)
                (void)confirm_em4100_candidate(fa_tgt_code, "fa-target");
            }
            /* 若未找到 score=0，回報全域最佳 */
            if (!fa_scan_found) {
              uint16_t fl16b = (uint16_t)(fa_best_code & 0xFFFFULL);
              LOG_WRN("!!! [FA-SCAN] best sc=%d p=%d s=%d ID=%010llX l16=%04X",
                      fa_best_sc, fa_best_pol, fa_best_s,
                      (unsigned long long)fa_best_code, (unsigned int)fl16b);
            }
          }
        }
      }
      /* 診斷：找出最長的連續 1 串 (應≥9 才能包含 EM4100 header) 以及最長 0 串
       */
      int max_run1 = 0, cur_run1 = 0, max_run1_pos = -1, cur_run1_start = 0;
      int max_run0 = 0, cur_run0 = 0, max_run0_pos = -1, cur_run0_start = 0;
      for (int i = 0; i < best_bits_snapshot_len; i++) {
        if (em_delta_best_bits_snapshot[i]) {
          if (cur_run1 == 0) cur_run1_start = i;
          cur_run1++;
          if (cur_run1 > max_run1) {
            max_run1 = cur_run1;
            max_run1_pos = cur_run1_start;
          }
          cur_run0 = 0;
        } else {
          if (cur_run0 == 0) cur_run0_start = i;
          cur_run0++;
          if (cur_run0 > max_run0) {
            max_run0 = cur_run0;
            max_run0_pos = cur_run0_start;
          }
          cur_run1 = 0;
        }
      }
      LOG_WRN("!!! [MAX-RUN-1] len=%d pos=%d bits_total=%d half_q8=%u inv=%d",
              max_run1, max_run1_pos, best_bits_snapshot_len,
              (unsigned)best_half_q8_snap, best_inv_snap);
      LOG_WRN("!!! [MAX-RUN-0] len=%d pos=%d", max_run0, max_run0_pos);
      /* 印出從 MAX-RUN-1 位置起的 48 bits (= 真正 header 起點) */
      {
        char bits48[49];
        int snap_start = (max_run1_pos >= 0) ? max_run1_pos : 0;
        int snap_avail = best_bits_snapshot_len - snap_start;
        int snap_print = (snap_avail < 48) ? snap_avail : 48;
        for (int i = 0; i < snap_print; i++) {
          bits48[i] = em_delta_best_bits_snapshot[snap_start + i] ? '1' : '0';
        }
        bits48[snap_print] = '\0';
        LOG_WRN("!!! [SNAP-HEAD@%d] 48bits: %s", snap_start, bits48);
      }
      /* 從最長 1-run header 位置重新解碼 (最可靠的 header 起點) */
      if (max_run1 >= 9 && (max_run1_pos + 64) <= best_bits_snapshot_len) {
        uint64_t code_bh = decode_card_code(
            (uint8_t*)&em_delta_best_bits_snapshot[max_run1_pos]);
        uint16_t bh_low16 = (uint16_t)(code_bh & 0xFFFFULL);
        LOG_WRN("!!! [BEST-HDR-DEC] hdr_pos=%d len=%d ID=%010llX low16=%04X",
                max_run1_pos, max_run1, (unsigned long long)code_bh,
                (unsigned int)bh_low16);
        if (bh_low16 == EM_ALIGN_TARGET_LOW16) {
          LOG_WRN("!!! [BEST-HDR-MATCH] TARGET %04X FOUND!",
                  EM_ALIGN_TARGET_LOW16);
        }
        (void)confirm_em4100_candidate(code_bh, "best-hdr");
      }
      /* Differential Manchester 解碼診斷：計算相鄰 bit XOR 串的最長連續 1 */
      /* DM 編碼中 "1 bit" = 方向轉換，若卡片使用 DM，header 9個"1"會在 XOR
       * 串中呈現為 9個連續的1（每個相鄰 bit 都不同）或9個連續的0（每個相鄰
       * bit 都相同） */
      {
        int dm_max1 = 0, dm_cur1 = 0, dm_max1_pos = -1, dm_cur1_start = 0;
        int dm_max0 = 0, dm_cur0 = 0, dm_max0_pos = -1, dm_cur0_start = 0;
        for (int i = 1; i < best_bits_snapshot_len; i++) {
          uint8_t xor_bit = em_delta_best_bits_snapshot[i] ^
                            em_delta_best_bits_snapshot[i - 1];
          if (xor_bit) {
            if (dm_cur1 == 0) dm_cur1_start = i;
            dm_cur1++;
            if (dm_cur1 > dm_max1) {
              dm_max1 = dm_cur1;
              dm_max1_pos = dm_cur1_start;
            }
            dm_cur0 = 0;
          } else {
            if (dm_cur0 == 0) dm_cur0_start = i;
            dm_cur0++;
            if (dm_cur0 > dm_max0) {
              dm_max0 = dm_cur0;
              dm_max0_pos = dm_cur0_start;
            }
            dm_cur1 = 0;
          }
        }
        LOG_WRN(
            "!!! [DM-RUN] trans-1=%d pos=%d same-0=%d pos=%d (DM header "
            "needs "
            "8+)",
            dm_max1, dm_max1_pos, dm_max0, dm_max0_pos);

        /* 嘗試 Differential Manchester EM4100 解碼：
         * XOR stream dm_bit[i] = snap[i+1] XOR snap[i] = DM 解碼後資料位元。
         * 掃描所有資料起始位置 P：要求 P 前面有 7..9 個連續 XOR=1 (DM
         * header)。 每個候選位置嘗試解 10 rows × 5 bit + 5 col/stop。 */
        {
          int xlen = best_bits_snapshot_len - 1;
          /* 建立 XOR 串 (reuse em_delta_inv_bits as temp buffer) */
          for (int i = 0; i < xlen; i++) {
            em_delta_inv_bits[i] = em_delta_best_bits_snapshot[i + 1] ^
                                   em_delta_best_bits_snapshot[i];
          }
          int dm_found = 0; /* 本輪找到有效 DM frame 次數 */
          /* 對每個候選資料起始位置 P 做測試 */
          for (int P = 7; P <= xlen - 55; P++) {
            /* 計算 P 前面連續 XOR=1 的長度（最多回看 10 位） */
            int hlen = 0;
            for (int h = 1; h <= 10 && (P - h) >= 0; h++) {
              if (em_delta_inv_bits[P - h])
                hlen++;
              else
                break;
            }
            if (hlen < 7) continue; /* header 太短，跳過 */
            /* 確認 P-hlen-1 位置（header 前一位）為 XOR=0 或超出邊界
             * 若沒有這個邊界，說明 P 不是資料起始而是 header 中間 */
            int before_hdr = P - hlen - 1;
            if (before_hdr >= 0 && em_delta_inv_bits[before_hdr]) {
              continue; /* header 前面仍是 XOR=1 → P 不是資料起始 */
            }
            /* 嘗試解碼 EM4100 資料：10 rows × 5 bits + 5 col/stop */
            uint8_t row_data[10][4];
            int row_err = 0;
            for (int r = 0; r < 10; r++) {
              uint8_t parity = 0;
              for (int b = 0; b < 4; b++) {
                uint8_t d = em_delta_inv_bits[P + r * 5 + b];
                row_data[r][b] = d;
                parity ^= d;
              }
              if (parity != em_delta_inv_bits[P + r * 5 + 4]) row_err++;
            }
            if (row_err > 2) continue;
            /* Column parities */
            int col_err = 0;
            for (int c = 0; c < 4; c++) {
              uint8_t cparity = 0;
              for (int r = 0; r < 10; r++) cparity ^= row_data[r][c];
              if (cparity != em_delta_inv_bits[P + 50 + c]) col_err++;
            }
            if (col_err > 2) continue;
            uint8_t stop_bit = em_delta_inv_bits[P + 54];
            /* 組合 raw40 */
            uint64_t dm_raw40 = 0;
            for (int r = 0; r < 10; r++) {
              for (int b = 0; b < 4; b++) {
                dm_raw40 = (dm_raw40 << 1) | row_data[r][b];
              }
            }
            uint16_t dm_low16 = (uint16_t)(dm_raw40 & 0xFFFFULL);
            dm_found++;
            LOG_WRN(
                "!!! [DM-DECODE] hdr=%d P=%d row_err=%d col_err=%d"
                " stop=%d raw40=%010llX low16=%04X",
                hlen, P, row_err, col_err, (int)stop_bit,
                (unsigned long long)dm_raw40, (unsigned int)dm_low16);
            if (dm_low16 == EM_ALIGN_TARGET_LOW16) {
              LOG_WRN("!!! [DM-MATCH] TARGET FOUND! low16=0x%04X (%u)",
                      (unsigned int)dm_low16, (unsigned int)dm_low16);
            }
            if (dm_found >= 6) break; /* 最多印 6 個，避免 log 爆炸 */
          }
          if (dm_found == 0) {
            LOG_WRN("!!! [DM-DECODE] no valid frame (row+col<=2 each)");
          }
        }

        /* 印出 snapshot 起始位置的前 24 個 env_edge_samples 原始振幅值
         * （4 windows/bit × 6 bits = 24 值，可看出 Manchester / DM 波形特徵）
         */
        {
          int win_start = (int)(best_off_q8_snap >> 8U);
          if (win_start < 0) win_start = 0;
          char env_buf[128];
          int ew = 0;
          int print_cnt = 0;
          for (int w = win_start;
               w < win_start + 24 && w < level_len && print_cnt < 24;
               w++, print_cnt++) {
            int written = snprintf(env_buf + ew, (int)sizeof(env_buf) - ew - 1,
                                   "%u ", (unsigned)env_edge_samples[w]);
            if (written < 0 || ew + written >= (int)sizeof(env_buf) - 1) break;
            ew += written;
          }
          env_buf[ew] = '\0';
          LOG_WRN("!!! [ENV-DUMP] win=%d(4win/bit): %s", win_start, env_buf);
        }
      }
    }
#if EM_DELTA_DEBUG_RAW_MODE
    LOG_WRN("[DEBUG-RAW] raw40=%010llX bits=%d frame=%d inv=%d",
            (unsigned long long)best_code, best_bits, best_frame_off, best_inv);
#endif
  }

#if EM_DELTA_DEBUG_RAW_MODE
  return -1;
#else
  return -1;
#endif
}

#if EM_ENABLE_EDGE_THRESHOLD_SWEEP
static int try_em4100_edge_threshold_decode(
    int level_len, uint32_t min_edges, uint32_t max_edges, uint64_t* out_code,
    int* out_bits, int* out_half_windows, int* out_offset, bool* out_inverted,
    int* out_bad_pairs, uint32_t* out_threshold) {
  if (level_len < 128 || max_edges <= min_edges) return -1;

  for (uint32_t threshold = min_edges; threshold < max_edges; threshold++) {
    for (int i = 0; i < level_len; i++) {
      env_level_samples[i] = env_edge_samples[i] > threshold ? 1U : 0U;
    }

    int slide = try_em4100_fractional_level_decode(
        level_len, out_code, out_bits, out_half_windows, out_offset,
        out_inverted, out_bad_pairs, false, "edge_frac");
    if (slide >= 0) {
      *out_threshold = threshold;
      return slide;
    }
  }

  return -1;
}
#endif

#if EM_ENABLE_TICK_FALLBACK_DIAG
void decode_bitstream_variant(uint16_t total_ticks, uint8_t* out_bits,
                              int* out_len, uint8_t initial_val,
                              int initial_short_state) {
  int bit_idx = 0;
  uint8_t current_val = initial_val;
  int state = initial_short_state;

  memset(out_bits, 0, EM_DECODED_BITS_CAP);
  *out_len = 0;

  for (int i = 0; i < total_ticks; i++) {
    uint32_t T = tick_buffer[i];
    uint32_t num_1t;

    if (T < EM_EDGE_MIN_US) {
      state = 0;
      continue;
    }

    num_1t = (T + (EM_CLOCK_BASE_1T_US / 2U)) / EM_CLOCK_BASE_1T_US;
    if (num_1t == 0U) {
      state = 0;
      continue;
    }
    if (num_1t > EM_TICK_SPLIT_MAX_PARTS) {
      num_1t = EM_TICK_SPLIT_MAX_PARTS;
    }

    for (uint32_t part = 0; part < num_1t; part++) {
      if (part + 1U < num_1t) {
        current_val = !current_val;
        if (bit_idx < EM_DECODED_BITS_CAP) out_bits[bit_idx++] = current_val;
        state = 0;
        part++;
      } else {
        if (state == 0) {
          state = 1;
        } else {
          if (bit_idx < EM_DECODED_BITS_CAP) out_bits[bit_idx++] = current_val;
          state = 0;
        }
      }
    }
  }
  *out_len = bit_idx;
}

void decode_bitstream(uint16_t total_ticks, uint8_t* out_bits, int* out_len) {
  decode_bitstream_variant(total_ticks, out_bits, out_len, 1, 0);
}

static int try_em4100_edge_capture_decode(int tick_count, uint64_t* out_code,
                                          int* out_bits, bool* out_inverted) {
  int best_score = 1000;
  uint32_t best_unit_us = 0;
  uint32_t best_tol_pct = 0;
  int best_resync = 0;
  int best_phase = 0;
  int best_bits = 0;
  int best_bad = 0;
  int best_frame = -1;
  int best_inv = 0;
  int best_header = 0;
  int best_row = 0;
  int best_col = 0;
  int best_stop = 0;
  uint64_t best_code = 0;
  uint64_t reported_codes[8] = {0};
  int reported_count = 0;
  int rescue_best_score = 1000;
  int rescue_best_hb = -1;
  int rescue_best_off = -1;
  int rescue_best_inv = 0;
  int rescue_best_bad = 0;
  int rescue_best_header = 0;
  int rescue_best_row = 0;
  int rescue_best_col = 0;
  int rescue_best_stop = 0;
  uint64_t rescue_best_code = 0;
  scan_vote_candidate_t scan_vote_candidates[EM_SCAN_VOTE_CANDIDATES] = {0};

  for (uint32_t unit_us = EM_EDGECAP_UNIT_MIN_US;
       unit_us <= EM_EDGECAP_UNIT_MAX_US; unit_us += EM_EDGECAP_UNIT_STEP_US) {
    for (uint32_t tol_pct = EM_EDGECAP_TOL_MIN_PCT;
         tol_pct <= EM_EDGECAP_TOL_PCT; tol_pct += EM_EDGECAP_TOL_STEP_PCT) {
      int half_len = 0;

      memset(edgecap_half_bits, 0, EM_DECODED_BITS_CAP);
      memset(edgecap_half_tick_idx, 0, sizeof(edgecap_half_tick_idx));
      memset(edgecap_half_sub_idx, 0, sizeof(edgecap_half_sub_idx));

      for (int i = 0; i < tick_count && half_len < EM_DECODED_BITS_CAP; i++) {
        uint32_t T = tick_buffer[i];
        /* Paper flow: falling edge stores 1, rising edge stores 0. */
        uint8_t half_bit = tick_edge_to_level[i] ? 0U : 1U;
        uint32_t repeat;

        if (!edgecap_repeat_from_gap(T, unit_us, &repeat)) continue;
        if (repeat > EM_EDGECAP_MAX_REPEAT) continue;

        for (uint32_t n = 0; n < repeat && half_len < EM_DECODED_BITS_CAP;
             n++) {
          edgecap_half_bits[half_len] = half_bit;
          edgecap_half_tick_idx[half_len] = (uint16_t)i;
          edgecap_half_sub_idx[half_len] = (uint8_t)n;
          half_len++;
        }
      }

      if (half_len < 128) continue;

      for (int hb_start = 0; hb_start + 127 < half_len; hb_start++) {
        uint8_t rescue_bits[EM_EDGECAP_RESCUE_BITS];
        uint8_t rescue_pair_bad[EM_EDGECAP_RESCUE_BITS];
        uint16_t rescue_half_tick[EM_EDGECAP_RESCUE_BITS];
        int rescue_len = 0;
        int tick_start = edgecap_half_tick_idx[hb_start];
        uint8_t tick_sub_start = edgecap_half_sub_idx[hb_start];

        if (!edgecap_decode_header_rescue_stream(
                tick_count, tick_start, tick_sub_start, unit_us, rescue_bits,
                rescue_pair_bad, rescue_half_tick, &rescue_len)) {
          continue;
        }

        for (int inv = 0; inv <= 1; inv++) {
          uint64_t candidate_code;
          const char* candidate_reason;
          uint8_t sentinel_bit = 1U;

          if (inv) invert_bits(rescue_bits, rescue_len);
          for (int rescue_off = 0; rescue_off + 64 <= rescue_len;
               rescue_off++) {
            int rescue_bad = 0;
            for (int b = 0; b < 64; b++) {
              rescue_bad += rescue_pair_bad[rescue_off + b] ? 1 : 0;
            }
            sentinel_bit = 1U;
            if (rescue_off > 0) {
              sentinel_bit = rescue_bits[rescue_off - 1];
            } else if (edgecap_decode_bit_pair(half_len, hb_start - 2,
                                               &sentinel_bit)) {
              if (inv) sentinel_bit ^= 1U;
            }

            int rescue_header;
            int rescue_row;
            int rescue_col;
            int rescue_stop;
            int rescue_frame_score = em4100_frame_error_score(
                rescue_bits, rescue_off, &rescue_header, &rescue_row,
                &rescue_col, &rescue_stop);
            int rescue_score =
                rescue_frame_score + rescue_bad + (sentinel_bit == 0U ? 0 : 2);
            uint64_t rescue_code =
                decode_card_code((uint8_t*)&rescue_bits[rescue_off]);

            if (rescue_stop == 0) {
              consider_scan_vote_candidate(
                  scan_vote_candidates, &rescue_bits[rescue_off],
                  "rescue-group", rescue_score, rescue_bad,
                  (hb_start / 2) + rescue_off, inv, rescue_code);
            }

            if (rescue_score < rescue_best_score) {
              rescue_best_score = rescue_score;
              rescue_best_hb = hb_start;
              rescue_best_off = rescue_off;
              rescue_best_inv = inv;
              rescue_best_bad = rescue_bad;
              rescue_best_header = rescue_header;
              rescue_best_row = rescue_row;
              rescue_best_col = rescue_col;
              rescue_best_stop = rescue_stop;
              rescue_best_code = rescue_code;
            }

            if (rescue_bad > EM_LEVEL_MAX_FRAME_BAD_PAIRS) continue;

            if (sentinel_bit == 0U &&
                ((em4100_candidate_from_parity(rescue_bits, rescue_off,
                                               &candidate_code,
                                               &candidate_reason) &&
                  strcmp(candidate_reason, "data2") != 0) ||
                 em4100_candidate_from_relaxed_header_clean(
                     rescue_bits, rescue_off, &candidate_code,
                     &candidate_reason))) {
              bool already_reported = false;
              for (int r = 0; r < reported_count; r++) {
                if (reported_codes[r] == candidate_code) {
                  already_reported = true;
                  break;
                }
              }

              if (!already_reported) {
                if (reported_count < (int)ARRAY_SIZE(reported_codes)) {
                  reported_codes[reported_count++] = candidate_code;
                }
#if RFID_DECODE_DIAG_LOG
                LOG_WRN(
                    "[HEADER-RESCUE hb=%d off=%d tick=%d sub=%u bad=%d "
                    "unit_us=%u tol=%u]",
                    hb_start, rescue_off, rescue_half_tick[rescue_off],
                    tick_sub_start, rescue_bad, unit_us, tol_pct);
#endif
                if (confirm_em4100_candidate(candidate_code,
                                             candidate_reason)) {
                  *out_code = candidate_code;
                  *out_bits = 64;
                  *out_inverted = inv != 0;
                  return (hb_start / 2) + rescue_off;
                }
              }
            }
          }

          if (inv) invert_bits(rescue_bits, rescue_len);
        }
      }

      for (int frame_resync = 0; frame_resync <= 1; frame_resync++) {
        uint8_t frame_bits[64];
        uint8_t frame_pair_bad[64];

        for (int hb_start = 0; hb_start + 127 < half_len; hb_start++) {
          int frame_bad = 0;

          if (!edgecap_decode_frame_from_halfbits(half_len, hb_start,
                                                  frame_resync, frame_bits,
                                                  frame_pair_bad, &frame_bad)) {
            continue;
          }
          if (frame_bad > 2) continue;

          for (int inv = 0; inv <= 1; inv++) {
            uint64_t candidate_code;
            const char* candidate_reason;
            uint8_t sentinel_bit = 1U;

            if (inv) invert_bits(frame_bits, 64);
            if (edgecap_decode_bit_pair(half_len, hb_start - 2,
                                        &sentinel_bit)) {
              if (inv) sentinel_bit ^= 1U;
            }
            if (sentinel_bit != 0U) {
              if (inv) invert_bits(frame_bits, 64);
              continue;
            }

            if ((em4100_candidate_from_parity(frame_bits, 0, &candidate_code,
                                              &candidate_reason) &&
                 strcmp(candidate_reason, "data2") != 0) ||
                em4100_candidate_from_relaxed_header_clean(
                    frame_bits, 0, &candidate_code, &candidate_reason)) {
              bool already_reported = false;
              for (int r = 0; r < reported_count; r++) {
                if (reported_codes[r] == candidate_code) {
                  already_reported = true;
                  break;
                }
              }

              if (!already_reported) {
                if (reported_count < (int)ARRAY_SIZE(reported_codes)) {
                  reported_codes[reported_count++] = candidate_code;
                }

#if RFID_DECODE_DIAG_LOG
                LOG_WRN(
                    "[FRAME-edgecap hb=%d resync=%d bad=%d unit_us=%u "
                    "tol=%u]",
                    hb_start, frame_resync, frame_bad, unit_us, tol_pct);
#endif
                if (confirm_em4100_candidate(candidate_code,
                                             candidate_reason)) {
                  *out_code = candidate_code;
                  *out_bits = 64;
                  *out_inverted = inv != 0;
                  return hb_start / 2;
                }
              }
            }

            if (inv) invert_bits(frame_bits, 64);
          }
        }
      }

      for (int resync = 0; resync <= 1; resync++) {
        for (int phase = 0; phase <= 1; phase++) {
          int bit_len = 0;
          int pos = phase;

          memset(dec_raw_bits, 0, EM_DECODED_BITS_CAP);
          memset(dec_pair_bad, 0, EM_DECODED_BITS_CAP);
          memset(dec_bit_half_pos, 0, sizeof(dec_bit_half_pos));

          while (pos + 1 < half_len && bit_len < EM_DECODED_BITS_CAP) {
            uint8_t first = edgecap_half_bits[pos];
            uint8_t second = edgecap_half_bits[pos + 1];
            bool bad_pair = false;
            int bit_idx = bit_len;

            if (first == 1U && second == 0U) {
              dec_raw_bits[bit_len++] = 1U;
            } else if (first == 0U && second == 1U) {
              dec_raw_bits[bit_len++] = 0U;
            } else {
              bad_pair = true;
              dec_pair_bad[bit_len] = 1U;
              dec_raw_bits[bit_len++] = second;
            }
            dec_bit_half_pos[bit_idx] = (uint16_t)pos;

            pos += (resync && bad_pair) ? 1 : 2;
          }

          if (bit_len < 64) continue;

          for (int inv = 0; inv <= 1; inv++) {
            if (inv) invert_bits(dec_raw_bits, bit_len);

            for (int off = 0; off <= bit_len - 64; off++) {
              uint64_t candidate_code;
              const char* candidate_reason;
              uint64_t vote_code;
              const char* vote_reason;
              int vote_frames = 0;
              int vote_weak_bits = 0;
              int vote_start_frame = 0;
              int header_err;
              int row_err;
              int col_err;
              int stop_err;
              int frame_bad = count_bad_pairs_in_frame(off);
              int frame_score =
                  em4100_frame_error_score(dec_raw_bits, off, &header_err,
                                           &row_err, &col_err, &stop_err);
              int unit_penalty = (int)((unit_us > EM_CLOCK_BASE_1T_US)
                                           ? (unit_us - EM_CLOCK_BASE_1T_US)
                                           : (EM_CLOCK_BASE_1T_US - unit_us)) /
                                 8;
              int score = frame_score + frame_bad + unit_penalty;
              uint64_t code = decode_card_code((uint8_t*)&dec_raw_bits[off]);

              if (stop_err == 0) {
                consider_scan_vote_candidate(scan_vote_candidates,
                                             &dec_raw_bits[off], "edge-group",
                                             score, frame_bad, off, inv, code);
              }

              if (score < best_score) {
                best_score = score;
                best_unit_us = unit_us;
                best_tol_pct = tol_pct;
                best_resync = resync;
                best_phase = phase;
                best_bits = bit_len;
                best_bad = frame_bad;
                best_frame = off;
                best_inv = inv;
                best_header = header_err;
                best_row = row_err;
                best_col = col_err;
                best_stop = stop_err;
                best_code = code;
              }

              if (off < 64 &&
                  em4100_candidate_from_repeated_frames(
                      dec_raw_bits, bit_len, off, &vote_code, &vote_reason,
                      &vote_frames, &vote_weak_bits, &vote_start_frame)) {
                bool already_reported = false;
                for (int r = 0; r < reported_count; r++) {
                  if (reported_codes[r] == vote_code) {
                    already_reported = true;
                    break;
                  }
                }
                if (!already_reported) {
                  if (reported_count < (int)ARRAY_SIZE(reported_codes)) {
                    reported_codes[reported_count++] = vote_code;
                  }

                  LOG_WRN(
                      "[VOTE-edgecap frames=%d start=%d weak=%d off=%d "
                      "unit_us=%u]",
                      vote_frames, vote_start_frame, vote_weak_bits, off,
                      unit_us);
                  if (confirm_em4100_candidate(vote_code, vote_reason)) {
                    *out_code = vote_code;
                    *out_bits = bit_len;
                    *out_inverted = inv != 0;
                    return off;
                  }
                }
              }

              if (frame_bad <= EM_LEVEL_MAX_FRAME_BAD_PAIRS &&
                  em4100_candidate_from_synced_header(dec_raw_bits, bit_len,
                                                      off, &candidate_code,
                                                      &candidate_reason)) {
                bool already_reported = false;
                for (int r = 0; r < reported_count; r++) {
                  if (reported_codes[r] == candidate_code) {
                    already_reported = true;
                    break;
                  }
                }
                if (already_reported) continue;

                if (reported_count < (int)ARRAY_SIZE(reported_codes)) {
                  reported_codes[reported_count++] = candidate_code;
                }

                if (confirm_em4100_candidate(candidate_code,
                                             candidate_reason)) {
                  *out_code = candidate_code;
                  *out_bits = bit_len;
                  *out_inverted = inv != 0;
                  return off;
                }
              }
            }

            if (inv) invert_bits(dec_raw_bits, bit_len);
          }
        }
      }
    }
  }

#if RFID_DECODE_DIAG_LOG
  LOG_INF(
      "edgecap_best: score=%d unit_us=%u tol=%u resync=%d phase=%d bits=%d "
      "bad=%d",
      best_score, best_unit_us, best_tol_pct, best_resync, best_phase,
      best_bits, best_bad);
  LOG_INF("edgecap_best: frame=%d inv=%d header=%d row=%d col=%d stop=%d",
          best_frame, best_inv, best_header, best_row, best_col, best_stop);
  LOG_INF("edgecap_best: raw40=%010llX low16=%u", (unsigned long long)best_code,
          (uint16_t)(best_code & 0xFFFFULL));
  if (rescue_best_hb >= 0) {
    LOG_WRN(
        "RESCUE-best: score=%d hb=%d off=%d inv=%d bad=%d header=%d row=%d "
        "col=%d stop=%d",
        rescue_best_score, rescue_best_hb, rescue_best_off, rescue_best_inv,
        rescue_best_bad, rescue_best_header, rescue_best_row, rescue_best_col,
        rescue_best_stop);
    LOG_WRN("RESCUE-best: raw40=%010llX low16=%u",
            (unsigned long long)rescue_best_code,
            (uint16_t)(rescue_best_code & 0xFFFFULL));
  }
#endif
  for (int i = 0; i < EM_SCAN_VOTE_CANDIDATES; i++) {
    uint64_t voted_code;
    if (!scan_vote_candidates[i].used) continue;
    if (feed_row_vote_frame(scan_vote_candidates[i].bits,
                            scan_vote_candidates[i].source,
                            scan_vote_candidates[i].score,
                            scan_vote_candidates[i].bad, &voted_code)) {
      *out_code = voted_code;
      *out_bits = 64;
      *out_inverted = scan_vote_candidates[i].inv != 0;
      return scan_vote_candidates[i].origin >= 0
                 ? scan_vote_candidates[i].origin
                 : 0;
    }
  }

  return -1;
}

static int try_em4100_tick_halfcell_decode(int tick_count, uint64_t* out_code,
                                           int* out_bits, int* out_half_us,
                                           int* out_offset, bool* out_inverted,
                                           int* out_bad_pairs) {
  int best_score = 1000;
  int best_half_us = 0;
  int best_offset = 0;
  int best_bits = 0;
  int best_bad = 0;
  int best_frame = -1;
  int best_inv = 0;
  int best_header = 0;
  int best_row = 0;
  int best_col = 0;
  int best_stop = 0;
  uint64_t best_code = 0;

  for (int half_us = EM_TICK_HALFCELL_MIN_US;
       half_us <= EM_TICK_HALFCELL_MAX_US;
       half_us += EM_TICK_HALFCELL_STEP_US) {
    for (uint8_t initial_level = 0; initial_level <= 1; initial_level++) {
      int half_len = 0;
      uint8_t level = initial_level;

      memset(env_level_samples, 0, EM_ENV_LEVEL_CAP);
      for (int i = 0; i < tick_count && half_len < EM_ENV_LEVEL_CAP; i++) {
        uint32_t repeat =
            (tick_buffer[i] + (uint32_t)(half_us / 2)) / (uint32_t)half_us;

        if (repeat < 1U) repeat = 1U;
        if (repeat > EM_TICK_HALFCELL_MAX_REPEAT) {
          repeat = EM_TICK_HALFCELL_MAX_REPEAT;
        }
        for (uint32_t n = 0; n < repeat && half_len < EM_ENV_LEVEL_CAP; n++) {
          env_level_samples[half_len++] = level;
        }
        level = !level;
      }

      for (int offset = 0; offset <= 1; offset++) {
        int bit_len = 0;

        memset(dec_raw_bits, 0, EM_DECODED_BITS_CAP);
        memset(dec_pair_bad, 0, EM_DECODED_BITS_CAP);
        for (int pos = offset;
             pos + 1 < half_len && bit_len < EM_DECODED_BITS_CAP; pos += 2) {
          uint8_t first = env_level_samples[pos];
          uint8_t second = env_level_samples[pos + 1];

          if (first == second) dec_pair_bad[bit_len] = 1;
          dec_raw_bits[bit_len++] = second;
        }
        if (bit_len < 64) continue;

        for (int inv = 0; inv <= 1; inv++) {
          if (inv) invert_bits(dec_raw_bits, bit_len);
          for (int off = 0; off <= bit_len - 64; off++) {
            uint64_t candidate_code;
            const char* candidate_reason;
            int header_err;
            int row_err;
            int col_err;
            int stop_err;
            int frame_bad = count_bad_pairs_in_frame(off);
            int score = em4100_frame_error_score(dec_raw_bits, off, &header_err,
                                                 &row_err, &col_err, &stop_err);
            uint64_t code = decode_card_code((uint8_t*)&dec_raw_bits[off]);

            if (score < best_score) {
              best_score = score;
              best_half_us = half_us;
              best_offset = offset;
              best_bits = bit_len;
              best_bad = frame_bad;
              best_frame = off;
              best_inv = inv;
              best_header = header_err;
              best_row = row_err;
              best_col = col_err;
              best_stop = stop_err;
              best_code = code;
            }

            if (frame_bad <= EM_TICK_HALF_MAX_FRAME_BAD_PAIRS) {
              bool parity_ok = em4100_candidate_from_parity(
                  dec_raw_bits, off, &candidate_code, &candidate_reason);
              bool relaxed_ok = em4100_candidate_from_relaxed_header_clean(
                  dec_raw_bits, off, &candidate_code, &candidate_reason);

              if (parity_ok || relaxed_ok) {
                if (confirm_em4100_candidate(candidate_code,
                                             candidate_reason)) {
                  *out_code = candidate_code;
                  *out_bits = bit_len;
                  *out_half_us = half_us;
                  *out_offset = offset;
                  *out_inverted = inv != 0;
                  *out_bad_pairs = frame_bad;
                  return off;
                }
              }
            }
          }
          if (inv) invert_bits(dec_raw_bits, bit_len);
        }
      }
    }
  }

#if RFID_DECODE_DIAG_LOG
  LOG_INF("tick_half_best: score=%d half_us=%d off=%d frame=%d inv=%d",
          best_score, best_half_us, best_offset, best_frame, best_inv);
  LOG_INF("tick_half_best: bits=%d bad=%d header=%d row=%d col=%d stop=%d",
          best_bits, best_bad, best_header, best_row, best_col, best_stop);
  LOG_INF("tick_half_best: raw40=%010llX low16=%u",
          (unsigned long long)best_code, (uint16_t)(best_code & 0xFFFFULL));
#endif
  return -1;
}

/* env128：依 short/long 分桶固定半位數（與 env_short/env_long 統計一致） */
static uint32_t env_tick_gap_to_half_repeat(uint32_t gap_us) {
  if (gap_us >= EM_LONG_MIN) {
    if (gap_us <= EM_LONG_MAX) {
      return EM_ENV_LONG_HALVES;
    }
    /* 超長 gap：按 512µs bit 週期延伸，每週期 4 半位 */
    uint32_t n = ((gap_us + EM_CLOCK_BASE_1T_US) / (EM_CLOCK_BASE_1T_US * 2U)) *
                 EM_ENV_LONG_HALVES;

    if (n < EM_ENV_LONG_HALVES) {
      n = EM_ENV_LONG_HALVES;
    }
    if (n > EM_TICK_HALFCELL_MAX_REPEAT) {
      n = EM_TICK_HALFCELL_MAX_REPEAT;
    }
    return n;
  }
  if (gap_us >= EM_SHORT_MIN) {
    return EM_ENV_SHORT_HALVES;
  }
  return 1U;
}

static int env_tick_expand_stored_levels(int tick_count) {
  int half_len = 0;

  memset(env_level_samples, 0, EM_ENV_LEVEL_CAP);
  for (int i = 0; i < tick_count && half_len < EM_ENV_LEVEL_CAP; i++) {
    uint32_t repeat = env_tick_gap_to_half_repeat(tick_buffer[i]);
    uint8_t half_val = tick_edge_to_level[i] ? 0U : 1U;

    for (uint32_t n = 0; n < repeat && half_len < EM_ENV_LEVEL_CAP; n++) {
      env_level_samples[half_len++] = half_val;
    }
  }
  return half_len;
}

/* env128 tick：包絡電平 + 固定門檻間距展開半位（不掃 half_us） */
static int try_em4100_tick_stored_level_decode(
    int tick_count, uint64_t* out_code, int* out_bits, int* out_half_us,
    int* out_offset, bool* out_inverted, int* out_bad_pairs) {
  int best_score = 1000;
#if EM_ALIGN_TARGET_LOW16 != 0U
  int best_rank = 1000000;
#endif
  const int fixed_half_us = (int)EM_ENV_WINDOW_US;
  int best_half_us = fixed_half_us;
  int best_offset = 0;
  int best_bits = 0;
  int best_bad = 0;
  int best_frame = -1;
  int best_inv = 0;
  int best_header = 0;
  int best_row = 0;
  int best_col = 0;
  int best_stop = 0;
  uint64_t best_code = 0;
  uint8_t best_frame_bits[64];
  bool have_best_frame = false;

#if RFID_COMP_DIRECT_EDGE_DECODE
  {
    const int half_len = env_tick_expand_stored_levels(tick_count);

    if (half_len >= 64) {
      for (int offset = 0; offset <= 1; offset++) {
        for (int inv_pass = 0; inv_pass <= 1; inv_pass++) {
          const int inv = comp_phase_valid
                              ? (inv_pass == 0 ? comp_last_best_inv
                                               : (comp_last_best_inv ^ 1))
                              : inv_pass;
          int bit_len = 0;

          memset(dec_raw_bits, 0, EM_DECODED_BITS_CAP);
          memset(dec_pair_bad, 0, EM_DECODED_BITS_CAP);
          for (int pos = offset;
               pos + 1 < half_len && bit_len < EM_DECODED_BITS_CAP; pos += 2) {
            uint8_t first = env_level_samples[pos];
            uint8_t second = env_level_samples[pos + 1];

            if (inv) {
              first ^= 1U;
              second ^= 1U;
            }
            if (first == 1U && second == 0U) {
              dec_raw_bits[bit_len++] = 1U;
            } else if (first == 0U && second == 1U) {
              dec_raw_bits[bit_len++] = 0U;
            } else {
              dec_pair_bad[bit_len] = 1U;
              dec_raw_bits[bit_len++] = second;
            }
          }
          if (bit_len < 64) {
            continue;
          }

          for (int off = 0; off <= bit_len - 64; off++) {
            uint64_t candidate_code;
            const char* candidate_reason;
            int header_err;
            int row_err;
            int col_err;
            int stop_err;
            int frame_bad = count_bad_pairs_in_frame(off);
            int score = em4100_frame_error_score(dec_raw_bits, off, &header_err,
                                                 &row_err, &col_err, &stop_err);
            uint64_t code = decode_card_code((uint8_t*)&dec_raw_bits[off]);
#if EM_ALIGN_TARGET_LOW16 != 0U
            int rank = em4100_decode_rank(score, code);
#endif

#if EM_ALIGN_TARGET_LOW16 != 0U
            if (rank < best_rank) {
              best_rank = rank;
#else
            if (score < best_score) {
#endif
              best_score = score;
              best_half_us = fixed_half_us;
              best_offset = offset;
              best_bits = bit_len;
              best_bad = frame_bad;
              best_frame = off;
              best_inv = inv;
              best_header = header_err;
              best_row = row_err;
              best_col = col_err;
              best_stop = stop_err;
              best_code = code;
              memcpy(best_frame_bits, &dec_raw_bits[off],
                     sizeof(best_frame_bits));
              have_best_frame = true;
            }

            if (frame_bad <= EM_TICK_HALF_MAX_FRAME_BAD_PAIRS) {
              bool parity_ok =
                  em4100_candidate_from_parity(
                      dec_raw_bits, off, &candidate_code, &candidate_reason) &&
                  strcmp(candidate_reason, "data2") != 0;
              bool relaxed_ok = em4100_candidate_from_relaxed_header_clean(
                  dec_raw_bits, off, &candidate_code, &candidate_reason);

              if (parity_ok || relaxed_ok) {
                if (confirm_em4100_candidate(candidate_code,
                                             candidate_reason)) {
#if RFID_COMP_DIRECT_EDGE_DECODE
                  comp_last_best_inv = inv;
                  comp_phase_valid = true;
#endif
                  *out_code = candidate_code;
                  *out_bits = bit_len;
                  *out_half_us = fixed_half_us;
                  *out_offset = offset;
                  *out_inverted = inv != 0;
                  *out_bad_pairs = frame_bad;
                  return off;
                }
              }
            }
          }
        }
      }
    }
  }
#endif

#if EM_COMP_LOG_LEVEL_SNAPSHOT
  if (have_best_frame) {
    int snap_len = 0;
    const int snap_n = 20;

    for (int i = 0; i < tick_count && snap_len < EM_ENV_LEVEL_CAP; i++) {
      uint32_t repeat = env_tick_gap_to_half_repeat(tick_buffer[i]);
      uint8_t half_val = tick_edge_to_level[i] ? 0U : 1U;

      for (uint32_t n = 0; n < repeat && snap_len < EM_ENV_LEVEL_CAP; n++) {
        env_level_samples[snap_len++] = half_val;
      }
    }
    if (snap_len > snap_n) {
      snap_len = snap_n;
    }
    LOG_WRN(
        "lvl_snap inv=%d half_us=%d: %u,%u,%u,%u,%u,%u,%u,%u,%u,%u,"
        "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u",
        best_inv, best_half_us, snap_len > 0 ? env_level_samples[0] : 0U,
        snap_len > 1 ? env_level_samples[1] : 0U,
        snap_len > 2 ? env_level_samples[2] : 0U,
        snap_len > 3 ? env_level_samples[3] : 0U,
        snap_len > 4 ? env_level_samples[4] : 0U,
        snap_len > 5 ? env_level_samples[5] : 0U,
        snap_len > 6 ? env_level_samples[6] : 0U,
        snap_len > 7 ? env_level_samples[7] : 0U,
        snap_len > 8 ? env_level_samples[8] : 0U,
        snap_len > 9 ? env_level_samples[9] : 0U,
        snap_len > 10 ? env_level_samples[10] : 0U,
        snap_len > 11 ? env_level_samples[11] : 0U,
        snap_len > 12 ? env_level_samples[12] : 0U,
        snap_len > 13 ? env_level_samples[13] : 0U,
        snap_len > 14 ? env_level_samples[14] : 0U,
        snap_len > 15 ? env_level_samples[15] : 0U,
        snap_len > 16 ? env_level_samples[16] : 0U,
        snap_len > 17 ? env_level_samples[17] : 0U,
        snap_len > 18 ? env_level_samples[18] : 0U,
        snap_len > 19 ? env_level_samples[19] : 0U);
  }
#endif

#if RFID_DECODE_DIAG_LOG
  LOG_INF("tick_lvl_best: score=%d sh=%u lg=%u off=%d frame=%d inv=%d bad=%d",
          best_score, (unsigned)EM_ENV_SHORT_HALVES,
          (unsigned)EM_ENV_LONG_HALVES, best_offset, best_frame, best_inv,
          best_bad);
  LOG_INF("tick_lvl_best: raw40=%010llX low16=%u",
          (unsigned long long)best_code, (uint16_t)(best_code & 0xFFFFULL));
#endif

#if RFID_COMP_DIRECT_EDGE_DECODE
  if (have_best_frame &&
#if EM_ALIGN_TARGET_LOW16 != 0U
      (best_score <= 4 || em4100_frame_near_target_low16(best_code))) {
#else
      (best_score <= 4)) {
#endif
    comp_last_best_inv = best_inv;
    comp_phase_valid = true;
#if RFID_DECODE_DIAG_LOG
    LOG_INF("comp_phase_lock: sh=%u lg=%u inv=%d score=%d",
            (unsigned)EM_ENV_SHORT_HALVES, (unsigned)EM_ENV_LONG_HALVES,
            best_inv, best_score);
#endif
  } else if (best_score > 18) {
    comp_phase_valid = false;
  }
#endif

#if EM_ALIGN_TARGET_LOW16 != 0U
  {
    uint16_t low16 = (uint16_t)(best_code & 0xFFFFULL);
    unsigned ham =
        (unsigned)__builtin_popcount((unsigned)(low16 ^ EM_ALIGN_TARGET_LOW16));
    int delta = em4100_low16_delta(low16);

    if (ham <= EM_NEAR_TARGET_MAX_HAM || delta <= EM_NEAR_TARGET_MAX_DELTA) {
      LOG_WRN(
          "tick_lvl_near_tgt: ham=%u delta=%d low16=%04X tgt=%04X score=%d "
          "hdr=%d row=%d col=%d stop=%d bad=%d",
          ham, delta, low16, EM_ALIGN_TARGET_LOW16, best_score, best_header,
          best_row, best_col, best_stop, best_bad);
    }
  }
#endif

#if EM_ALIGN_TARGET_LOW16 != 0U
  if (have_best_frame &&
      (em4100_frame_near_target_low16(best_code) || best_score <= 4)) {
    uint64_t repaired;

    if (em4100_recover_align_target(best_frame_bits, &repaired)) {
#if RFID_DECODE_DIAG_LOG
      LOG_WRN("[TickLvl-recover] raw40=%010llX low16=%04X",
              (unsigned long long)repaired, (uint16_t)(repaired & 0xFFFFULL));
#endif
#if RFID_COMP_DIRECT_EDGE_DECODE
      comp_last_best_inv = best_inv;
      comp_phase_valid = true;
#endif
      *out_code = repaired;
      *out_bits = best_bits;
      *out_half_us = fixed_half_us;
      *out_offset = best_offset;
      *out_inverted = best_inv != 0;
      *out_bad_pairs = best_bad;
      return best_frame;
    }
  }
#endif

  if (have_best_frame && best_score <= 12) {
    uint64_t candidate_code;
    const char* candidate_reason;

    if ((em4100_candidate_from_parity(best_frame_bits, 0, &candidate_code,
                                      &candidate_reason) &&
         strcmp(candidate_reason, "data2") != 0) ||
        em4100_candidate_from_relaxed_header_clean(
            best_frame_bits, 0, &candidate_code, &candidate_reason)) {
#if RFID_DECODE_DIAG_LOG
      LOG_WRN("[TickLvl-retry] raw40=%010llX low16=%u reason=%s",
              (unsigned long long)candidate_code,
              (uint16_t)(candidate_code & 0xFFFFULL), candidate_reason);
#endif
      if (confirm_em4100_candidate(candidate_code, candidate_reason)) {
        *out_code = candidate_code;
        *out_bits = best_bits;
        *out_half_us = best_half_us;
        *out_offset = best_offset;
        *out_inverted = best_inv != 0;
        *out_bad_pairs = best_bad;
        return best_frame;
      }
    }
  }

  return -1;
}

static int try_comp_env_tick_decode(int tick_count, uint64_t* out_code,
                                    int* out_bits, bool* out_inverted) {
  int slide;
  int bad_pairs = 0;
  int half_us = 0;
  int offset = 0;

  /* env128 固定分桶解碼先跑（快）；EdgeCap 全掃 unit 很慢 */
  slide = try_em4100_tick_stored_level_decode(tick_count, out_code, out_bits,
                                              &half_us, &offset, out_inverted,
                                              &bad_pairs);
  if (slide >= 0) {
    return slide;
  }

  slide =
      try_em4100_tick_halfcell_decode(tick_count, out_code, out_bits, &half_us,
                                      &offset, out_inverted, &bad_pairs);
  if (slide >= 0) {
    return slide;
  }

  slide = try_em4100_edge_capture_decode(tick_count, out_code, out_bits,
                                         out_inverted);
  if (slide >= 0) {
    return slide;
  }

  return -1;
}
#endif

static void tick_buffer_append(uint32_t diff_us, uint8_t edge_to_level,
                               int* demod_counter, bool* tick_window_slid) {
  if (*demod_counter >= TICK_BUFFER_SIZE) {
    const size_t half = TICK_BUFFER_SIZE / 2;
    memmove(tick_buffer, tick_buffer + half, half * sizeof(tick_buffer[0]));
    memmove(tick_edge_to_level, tick_edge_to_level + half,
            half * sizeof(tick_edge_to_level[0]));
    *demod_counter = (int)half;
    *tick_window_slid = true;
  }
  tick_buffer[*demod_counter] = diff_us;
  tick_edge_to_level[*demod_counter] = edge_to_level;
  (*demod_counter)++;
}

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
  bool strict_pair_blocked = false;
  rfid_saadc_candidate_consensus(frac_tag, rfid_saadc_pair_last_best_code,
                                 rfid_saadc_pair_last_best_score,
                                 rfid_saadc_pair_last_best_bad);
  if (rfid_saadc_pair_last_best_bad > (int)RFID_SAADC_STRICT_MAX_BAD_PAIRS) {
    strict_pair_blocked = true;
    if (!RFID_SAADC_LOG_IMPORTANT_ONLY) {
      LOG_INF(
          "%s_strict_pair_fail: levels=%d bad=%d max=%u half_q8=%u..%u "
          "sample=%d..%d",
          frac_tag, env_level_count, rfid_saadc_pair_last_best_bad,
          (unsigned)RFID_SAADC_STRICT_MAX_BAD_PAIRS,
          RFID_SAADC_MANCHESTER_HALF_MIN_Q8, RFID_SAADC_MANCHESTER_HALF_MAX_Q8,
          sample_min, sample_max);
    }
#if RFID_DECODE_DIAG_LOG
    LOG_WRN("%s_strict_pair_fail: continue delta diagnostics", frac_tag);
#endif
  }
  /* bad≤STRICT_MAX：放行 parity 容錯解碼路徑 */
#endif
  /*
    level_slide = try_em4100_clocked_resync_decode(
        env_level_count, &level_code, &level_bits, &half_windows,
    &level_offset, &level_inverted, &level_bad_pairs, true); if (level_slide
    >= 0) { LOG_WRN("[SUCCESS-Clocked half_q8=%d off_q8=%d inv=%d bad=%d
    slide=%d]", half_windows, level_offset, level_inverted, level_bad_pairs,
              level_slide);
      LOG_WRN("[SUCCESS-Clocked] ID: %08llX", (unsigned long long)level_code);
      return 1;
    }

    level_slide = try_em4100_fractional_level_decode(
        env_level_count, &level_code, &level_bits, &half_windows,
    &level_offset, &level_inverted, &level_bad_pairs, true, frac_tag); if
    (level_slide >= 0) { LOG_WRN("[SUCCESS-%s half_q8=%d off_q8=%d inv=%d
    bad=%d slide=%d]", frac_tag, half_windows, level_offset, level_inverted,
              level_bad_pairs, level_slide);
      LOG_WRN("[SUCCESS-%s] ID: %08llX", frac_tag,
              (unsigned long long)level_code);
      return 1;
    }

    level_slide = try_em4100_level_decode(
        env_level_count, &level_code, &level_bits, &half_windows,
    &level_offset, &level_inverted, &level_bad_pairs, true); if (level_slide
    >= 0) { LOG_WRN("[SUCCESS-Level h=%d off=%d inv=%d bad=%d slide=%d
    bits=%d]", half_windows, level_offset, level_inverted, level_bad_pairs,
              level_slide, level_bits);
      LOG_WRN("[SUCCESS-Level] ID: %08llX", (unsigned long long)level_code);
      return 1;
    }
  */
  /* 邊緣計時解碼：不依賴 bit rate，從 edge 間距自動推算 T/2 */
  saadc_edge_timing_decode_log(env_level_count);

#if ACTIVE_DECODE_MODE == DECODE_MODE_ASK
  level_slide = try_em4100_edge_delta_decode(
      env_level_count, &level_code, &level_bits, &half_windows, &level_offset,
      &level_inverted, &level_bad_pairs, RFID_DECODE_DIAG_LOG);
  if (level_slide >= 0) {
#if !RFID_LOG_MINIMAL
    LOG_WRN("[SUCCESS-Delta half_q8=%d off_q8=%d inv=%d bad=%d slide=%d]",
            half_windows, level_offset, level_inverted, level_bad_pairs,
            level_slide);
    LOG_WRN("[SUCCESS-Delta] ID: %08llX", (unsigned long long)level_code);
#endif
    return 1;
  }
#endif
#if RFID_SAADC_STRICT_MANCHESTER_ONLY
  if (strict_pair_blocked) {
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

#if !RFID_USE_SAADC_RECEIVER
static uint32_t env_edge_window_smooth(uint32_t raw, uint32_t* prev1,
                                       uint32_t* prev2, uint8_t* warmup) {
#if EM_ENV_EDGE_MA_TAPS >= 3
  uint32_t sum = raw;
  uint32_t n = 1U;

  if (*warmup >= 1U) {
    sum += *prev1;
    n++;
  }
  if (*warmup >= 2U) {
    sum += *prev2;
    n++;
  }
  *prev2 = *prev1;
  *prev1 = raw;
  if (*warmup < 2U) {
    (*warmup)++;
  }
  return sum / n;
#else
  ARG_UNUSED(prev1);
  ARG_UNUSED(prev2);
  ARG_UNUSED(warmup);
  return raw;
#endif
}

static void comp_log_silent_hint(uint32_t env_windows,
                                 const uint32_t* env_duty_hist,
                                 uint32_t comp_edges);

#if RFID_USE_HIGH_RES_TIMER
static void rfid_high_res_timer_init(void) {
  nrf_timer_task_trigger(RFID_HIGH_RES_TIMER, NRF_TIMER_TASK_STOP);
  nrf_timer_mode_set(RFID_HIGH_RES_TIMER, NRF_TIMER_MODE_TIMER);
  nrf_timer_bit_width_set(RFID_HIGH_RES_TIMER, NRF_TIMER_BIT_WIDTH_32);
  /* nRF53 HAL：PRESCALER 指數；NRF_TIMER_FREQ_1MHz → 16MHz/2^4 = 1MHz */
  nrf_timer_prescaler_set(RFID_HIGH_RES_TIMER, (uint32_t)NRF_TIMER_FREQ_1MHz);
  nrf_timer_task_trigger(RFID_HIGH_RES_TIMER, NRF_TIMER_TASK_CLEAR);
  nrf_timer_task_trigger(RFID_HIGH_RES_TIMER, NRF_TIMER_TASK_START);
}

static uint32_t rfid_high_res_timer_get_us(void) {
  nrf_timer_task_trigger(RFID_HIGH_RES_TIMER, NRF_TIMER_TASK_CAPTURE0);
  return nrf_timer_cc_get(RFID_HIGH_RES_TIMER, NRF_TIMER_CC_CHANNEL0);
}
#endif

int em4095_comp_receiver(void) {
  int demod_counter = 0;
  uint32_t all_edges = 0;
  uint32_t comp_edges = 0;
  uint32_t phy_edges = 0;
  uint32_t raw_edges = 0;
  uint32_t max_gap_us = 0;
  uint32_t gap_lt_100 = 0;
  uint32_t gap_100_230 = 0;
  uint32_t gap_231_520 = 0;
  uint32_t gap_gt_520 = 0;
  uint32_t raw_accepted = 0;
  uint32_t raw_rejected = 0;
  uint32_t raw_short = 0;
  uint32_t raw_long = 0;
  uint32_t raw_other = 0;
  uint32_t env_windows = 0;
  uint32_t env_boot_sum = 0;
  uint32_t env_sum_edges = 0;
  uint32_t env_min_edges = 0xFFFFFFFFU;
  uint32_t env_max_edges = 0;
  uint32_t env_duty_boot_sum_x16 = 0;
  uint32_t env_duty_sum_x16 = 0;
  uint32_t env_duty_min_x16 = 0xFFFFFFFFU;
  uint32_t env_duty_max_x16 = 0;
  uint32_t env_duty_baseline_q8 = 0;
  uint32_t env_transitions = 0;
  uint32_t env_short = 0;
  uint32_t env_long = 0;
  uint32_t env_other = 0;
  uint32_t env_window_edges = 0;
  uint32_t env_window_samples = 0;
  uint32_t env_window_high_samples = 0;
  uint32_t env_baseline_q8 = 0;
  uint32_t env_high_windows = 0;
  uint32_t env_edge_hist[10] = {0};
  uint32_t env_duty_hist[17] = {0};
  int env_level_count = 0;

#if RFID_COMP_USE_NRFX_DIRECT
  if (!comp_nrfx_armed) {
    return 0;
  }
  comp_nrfx_irq_ready = 0;
  comp_nrfx_irq_cross = 0;
  comp_nrfx_irq_up = 0;
  comp_nrfx_irq_down = 0;
#else
  if (!device_is_ready(comp_dev)) {
    return 0;
  }
#endif

  k_sleep(K_MSEC(50));
  reset_em4100_confirm_pool();

#if RFID_COMP_USE_NRFX_DIRECT
  int stable = (int)nrfx_comp_sample();
#else
  int stable = comparator_get_output(comp_dev);
#endif
  uint8_t last_val = (stable > 0) ? 1 : 0;
#if RFID_USE_HIGH_RES_TIMER
  uint32_t t0 = rfid_high_res_timer_get_us();
#else
  uint32_t t0 = k_cycle_get_32();
#endif
  uint32_t last_phy_us = t0;
  uint32_t last_duty_sample_us = t0;
#if RFID_COMP_DIRECT_EDGE_DECODE && \
    (RFID_COMP_TICK_SOURCE == RFID_COMP_TICK_SOURCE_DEBOUNCE)
  uint32_t last_tick_store_us = 0;
#endif
  uint32_t env_window_start_us = t0;
  uint32_t last_env_transition_us = t0;
  uint64_t stop_time = k_uptime_get() + RFID_CAPTURE_WINDOW_MS;
  bool lead_sync_dropped = false;
  bool tick_window_slid = false;
  bool env_ready = false;
  uint8_t env_state = 0;
  uint8_t pending_env_state = 0;
  uint8_t pending_env_windows = 0;
  uint32_t env_ma_prev1 = 0;
  uint32_t env_ma_prev2 = 0;
  uint8_t env_ma_warmup = 0;

  while (k_uptime_get() < stop_time) {
#if RFID_COMP_USE_NRFX_DIRECT
    int current_val = (int)nrfx_comp_sample();
#else
    int current_val = comparator_get_output(comp_dev);
#endif
    if (current_val < 0) {
      continue;
    }

#if RFID_USE_HIGH_RES_TIMER
    uint32_t now = rfid_high_res_timer_get_us();
#else
    uint32_t now = k_cycle_get_32();
#endif

    if ((now - last_duty_sample_us) >= EM_ENV_DUTY_SAMPLE_US) {
      last_duty_sample_us = now;
      env_window_samples++;
      if (current_val > 0) {
        env_window_high_samples++;
      }
    }

    if (current_val != last_val) {
#if RFID_USE_HIGH_RES_TIMER
      uint32_t diff_phy = now - last_phy_us;
#else
      uint32_t diff_phy = k_cyc_to_us_near32(now - last_phy_us);
#endif

      all_edges++;
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
      env_window_edges++;
      comp_edges++;
      phy_edges++;
      raw_edges++;
      if (diff_phy >= EM_EDGE_MIN_US) {
        raw_accepted++;
        if (diff_phy >= EM_SHORT_MIN && diff_phy <= EM_SHORT_MAX) {
          raw_short++;
        } else if (diff_phy >= EM_LONG_MIN && diff_phy <= EM_LONG_MAX) {
          raw_long++;
        } else {
          raw_other++;
        }
      } else {
        raw_rejected++;
      }
#if RFID_COMP_DIRECT_EDGE_DECODE && \
    (RFID_COMP_TICK_SOURCE == RFID_COMP_TICK_SOURCE_DEBOUNCE)
      /* 診斷：固定週期抽樣 CMP；間隔幾乎恆為 debounce_us（缺 2T） */
      {
        const uint32_t since_store =
            last_tick_store_us == 0U ? 0U : (now - last_tick_store_us);

        if (last_tick_store_us == 0U ||
            since_store >= RFID_COMP_DEBOUNCE_TICK_US) {
          if (last_tick_store_us != 0U) {
            const uint8_t lvl = (uint8_t)(current_val > 0 ? 1 : 0);

            tick_buffer_append(since_store, lvl, &demod_counter,
                               &tick_window_slid);
            raw_accepted++;
            if (since_store >= EM_SHORT_MIN && since_store <= EM_SHORT_MAX) {
              env_short++;
            } else if (since_store >= EM_LONG_MIN &&
                       since_store <= EM_LONG_MAX) {
              env_long++;
            } else {
              env_other++;
            }
          }
          last_tick_store_us = now;
        }
      }
#endif
      last_phy_us = now;
      last_val = (uint8_t)(current_val > 0 ? 1 : 0);
    }

    if ((now - env_window_start_us) >= EM_ENV_WINDOW_US) {
      const uint32_t edge_smooth = env_edge_window_smooth(
          env_window_edges, &env_ma_prev1, &env_ma_prev2, &env_ma_warmup);
      const uint32_t edge_count_x16 = edge_smooth << 4;
      const uint32_t duty_x16 =
          env_window_samples
              ? ((env_window_high_samples << 4) / env_window_samples)
              : 0;

      env_windows++;
      env_sum_edges += edge_smooth;
      if (edge_smooth < env_min_edges) env_min_edges = edge_smooth;
      if (edge_smooth > env_max_edges) env_max_edges = edge_smooth;
      env_edge_hist[edge_smooth < 9U ? edge_smooth : 9U]++;
      env_duty_sum_x16 += duty_x16;
      if (duty_x16 < env_duty_min_x16) env_duty_min_x16 = duty_x16;
      if (duty_x16 > env_duty_max_x16) env_duty_max_x16 = duty_x16;
      env_duty_hist[duty_x16 <= 16U ? duty_x16 : 16U]++;

      if (!env_ready) {
        env_boot_sum += edge_smooth;
        env_duty_boot_sum_x16 += duty_x16;
        if (env_windows >= EM_ENV_BOOT_WINDOWS) {
          env_baseline_q8 = (env_boot_sum << 12) / env_windows;
          env_duty_baseline_q8 = (env_duty_boot_sum_x16 << 8) / env_windows;
#if EM_ENV_USE_DUTY_LEVEL
          env_state = ((duty_x16 << 8) >
                       env_duty_baseline_q8 + (EM_ENV_DUTY_THRESHOLD_X16 << 8))
                          ? 1
                          : 0;
#else
          env_state = ((edge_count_x16 << 8) >
                       env_baseline_q8 + (EM_ENV_THRESHOLD_X16 << 8))
                          ? 1
                          : 0;
#endif
          pending_env_state = env_state;
          pending_env_windows = 0;
          last_env_transition_us = now;
          env_ready = true;
        }
      } else {
        uint8_t next_env_state = env_state;

#if EM_ENV_USE_DUTY_LEVEL
        const uint32_t duty_q8 = duty_x16 << 8;
        const uint32_t duty_threshold_q8 = EM_ENV_DUTY_THRESHOLD_X16 << 8;
        if (duty_q8 > env_duty_baseline_q8 + duty_threshold_q8) {
          next_env_state = 1;
        } else if (duty_q8 + duty_threshold_q8 < env_duty_baseline_q8) {
          next_env_state = 0;
        }
#else
        const uint32_t edge_count_q8 = edge_count_x16 << 8;
        const uint32_t edge_threshold_q8 = EM_ENV_THRESHOLD_X16 << 8;
        if (edge_count_q8 > env_baseline_q8 + edge_threshold_q8) {
          next_env_state = 1;
        } else if (edge_count_q8 + edge_threshold_q8 < env_baseline_q8) {
          next_env_state = 0;
        }
#endif

        if (next_env_state != env_state) {
          if (next_env_state == pending_env_state) {
            if (pending_env_windows < UINT8_MAX) pending_env_windows++;
          } else {
            pending_env_state = next_env_state;
            pending_env_windows = 1;
          }
        } else {
          pending_env_state = env_state;
          pending_env_windows = 0;
        }

        if (next_env_state != env_state &&
            pending_env_windows >= EM_ENV_STATE_STABLE_WINDOWS) {
          uint32_t diff_env = now - last_env_transition_us;

          if (diff_env >= EM_ENV_MIN_TICK_US) {
            env_state = next_env_state;
            pending_env_state = env_state;
            pending_env_windows = 0;
            last_env_transition_us = now;
            env_transitions++;

            if (diff_env >= EM_SHORT_MIN && diff_env <= EM_SHORT_MAX) {
              env_short++;
            } else if (diff_env >= EM_LONG_MIN && diff_env <= EM_LONG_MAX) {
              env_long++;
            } else {
              env_other++;
            }

#if RFID_COMP_DIRECT_EDGE_DECODE && \
    (RFID_COMP_TICK_SOURCE == RFID_COMP_TICK_SOURCE_ENV)
            /* env HIGH=載波 ON；EM4100 Manchester：HIGH=carrier OFF */
            {
              bool store_env_tick = true;

              if (!lead_sync_dropped) {
                lead_sync_dropped = true;
                if (diff_env <= EM_SKIP_FIRST_GAP_US) {
                  store_env_tick = false;
                }
              }
              if (store_env_tick) {
                tick_buffer_append(diff_env, env_state ^ 1U, &demod_counter,
                                   &tick_window_slid);
              }
            }
#endif
          }
        }

        env_baseline_q8 = ((env_baseline_q8 * EM_ENV_BASELINE_WEIGHT) +
                           (edge_count_x16 << 8)) /
                          (EM_ENV_BASELINE_WEIGHT + 1U);
        env_duty_baseline_q8 =
            ((env_duty_baseline_q8 * EM_ENV_BASELINE_WEIGHT) +
             (duty_x16 << 8)) /
            (EM_ENV_BASELINE_WEIGHT + 1U);
      }

      if (env_ready && env_level_count < EM_ENV_LEVEL_CAP) {
        env_edge_samples[env_level_count] =
            edge_smooth > UINT8_MAX ? UINT8_MAX : (uint8_t)edge_smooth;
        env_level_samples[env_level_count++] = env_state;
        if (env_state) env_high_windows++;
      }

      env_window_edges = 0;
      env_window_samples = 0;
      env_window_high_samples = 0;
      env_window_start_us = now;
    }
  }

#if RFID_DECODE_DIAG_LOG
  LOG_INF(
      "scan: raw_edges=%u phy_edges=%u comp_edges=%u all_edges=%u stored=%d "
      "max_gap_us=%u",
      raw_edges, phy_edges, comp_edges, all_edges, demod_counter, max_gap_us);
#if RFID_COMP_USE_NRFX_DIRECT
  LOG_INF("comp_nrfx_irq: ready=%u cross=%u up=%u down=%u",
          (unsigned)comp_nrfx_irq_ready, (unsigned)comp_nrfx_irq_cross,
          (unsigned)comp_nrfx_irq_up, (unsigned)comp_nrfx_irq_down);
  comp_log_silent_hint(env_windows, env_duty_hist, comp_edges);
#endif
  LOG_INF("gap_hist: <100=%u 100-230=%u 231-520=%u >520=%u", gap_lt_100,
          gap_100_230, gap_231_520, gap_gt_520);
  LOG_INF("raw_ticks: accepted=%u rejected_lt_%u=%u short=%u long=%u other=%u",
          raw_accepted, EM_EDGE_MIN_US, raw_rejected, raw_short, raw_long,
          raw_other);
  LOG_INF(
      "env: windows=%u win_us=%u ma=%u edge_avg_x16=%u min=%u max=%u "
      "baseline_x16=%u",
      env_windows, EM_ENV_WINDOW_US, EM_ENV_EDGE_MA_TAPS,
      env_windows ? ((env_sum_edges << 4) / env_windows) : 0,
      env_min_edges == 0xFFFFFFFFU ? 0 : env_min_edges, env_max_edges,
      (env_baseline_q8 + 128U) >> 8);
  LOG_INF("env_duty: avg_x16=%u min=%u max=%u baseline_x16=%u",
          env_windows ? (env_duty_sum_x16 / env_windows) : 0,
          env_duty_min_x16 == 0xFFFFFFFFU ? 0 : env_duty_min_x16,
          env_duty_max_x16, (env_duty_baseline_q8 + 128U) >> 8);
  LOG_INF("duty_hist: 0=%u 1=%u 2=%u 3=%u 4=%u 5=%u 6=%u 7=%u",
          env_duty_hist[0], env_duty_hist[1], env_duty_hist[2],
          env_duty_hist[3], env_duty_hist[4], env_duty_hist[5],
          env_duty_hist[6], env_duty_hist[7]);
  LOG_INF("duty_hist: 8=%u 9=%u 10=%u 11=%u 12=%u 13=%u 14=%u 15=%u 16=%u",
          env_duty_hist[8], env_duty_hist[9], env_duty_hist[10],
          env_duty_hist[11], env_duty_hist[12], env_duty_hist[13],
          env_duty_hist[14], env_duty_hist[15], env_duty_hist[16]);
  LOG_INF("env_hist: 0=%u 1=%u 2=%u 3=%u 4=%u 5=%u 6=%u 7=%u 8=%u 9+=%u",
          env_edge_hist[0], env_edge_hist[1], env_edge_hist[2],
          env_edge_hist[3], env_edge_hist[4], env_edge_hist[5],
          env_edge_hist[6], env_edge_hist[7], env_edge_hist[8],
          env_edge_hist[9]);
  LOG_INF(
      "env_ticks: transitions=%u short=%u long=%u other=%u levels=%d high=%u "
      "slid=%d",
      env_transitions, env_short, env_long, env_other, env_level_count,
      env_high_windows, tick_window_slid);

  if (demod_counter > 0) {
    LOG_INF("Env fingerprint: %u, %u, %u, %u, %u, %u, %u, %u, %u, %u",
            demod_counter > 0 ? tick_buffer[0] : 0,
            demod_counter > 1 ? tick_buffer[1] : 0,
            demod_counter > 2 ? tick_buffer[2] : 0,
            demod_counter > 3 ? tick_buffer[3] : 0,
            demod_counter > 4 ? tick_buffer[4] : 0,
            demod_counter > 5 ? tick_buffer[5] : 0,
            demod_counter > 6 ? tick_buffer[6] : 0,
            demod_counter > 7 ? tick_buffer[7] : 0,
            demod_counter > 8 ? tick_buffer[8] : 0,
            demod_counter > 9 ? tick_buffer[9] : 0);
  }
#endif /* RFID_DECODE_DIAG_LOG */

#if RFID_COMP_DIRECT_EDGE_DECODE
#if RFID_DECODE_DIAG_LOG
  LOG_INF(
      "tick_path: %s stored=%d short=%u long=%u env_trans=%u carrier_rej=%u",
#if RFID_COMP_TICK_SOURCE == RFID_COMP_TICK_SOURCE_DEBOUNCE
      "debounce",
#else
      "env128",
#endif
      demod_counter, env_short, env_long, env_transitions, raw_rejected);
#if RFID_COMP_TICK_SOURCE == RFID_COMP_TICK_SOURCE_DEBOUNCE
  LOG_INF("tick_path: debounce_us=%u (expect mostly 1T gaps)",
          RFID_COMP_DEBOUNCE_TICK_US);
#else
  if (demod_counter == 0) {
    LOG_INF("tick_path: no env ticks (card coupling / env threshold)");
  }
#endif
#endif /* RFID_DECODE_DIAG_LOG */
  if ((uint32_t)demod_counter < RFID_COMP_DIRECT_CARD_MIN_EDGES) {
#if RFID_DECODE_DIAG_LOG
    LOG_INF("card_gate_skip(direct): stored=%d/%u", demod_counter,
            RFID_COMP_DIRECT_CARD_MIN_EDGES);
#endif
    reset_cross_scan_vote_pool();
    reset_row_vote_pool();
    reset_em4100_card_session();
    return 0;
  }
#else
  if (all_edges < EM_RAW_CARD_MIN_EDGES) {
#if RFID_DECODE_DIAG_LOG
    LOG_INF("card_gate_skip: all=%u/%u comp=%u phy=%u raw=%u trans=%u short=%u",
            all_edges, EM_RAW_CARD_MIN_EDGES, comp_edges, phy_edges, raw_edges,
            env_transitions, env_short);
#endif
    reset_cross_scan_vote_pool();
    reset_row_vote_pool();
    reset_em4100_card_session();
    return 0;
  }

  if (env_transitions < EM_CARD_MIN_ENV_TRANSITIONS ||
      env_short < EM_CARD_MIN_ENV_SHORT) {
#if RFID_DECODE_DIAG_LOG
    LOG_INF(
        "card_gate_skip: all=%u comp=%u phy=%u raw=%u trans=%u/%u "
        "short=%u/%u",
        all_edges, comp_edges, phy_edges, raw_edges, env_transitions,
        EM_CARD_MIN_ENV_TRANSITIONS, env_short, EM_CARD_MIN_ENV_SHORT);
#endif
    reset_cross_scan_vote_pool();
    reset_row_vote_pool();
    reset_em4100_card_session();
    return 0;
  }
#endif

#if RFID_DECODE_DIAG_LOG
  LOG_INF(
      "[CARD-ACTIVE] all=%u comp=%u phy=%u raw=%u trans=%u short=%u long=%u",
      all_edges, comp_edges, phy_edges, raw_edges, env_transitions, env_short,
      env_long);
#endif

#if RFID_COMP_DIRECT_EDGE_DECODE
  /* 直接邊緣模式：需要至少 20 個 LONG 間隔才能可靠解碼 */
  if (env_long < EM_MIN_LONG_TICKS_TO_DECODE) {
#if RFID_DECODE_DIAG_LOG
    LOG_INF("decode_skip(direct): stored=%d long=%u/%u short=%u", demod_counter,
            env_long, EM_MIN_LONG_TICKS_TO_DECODE, env_short);
#endif
#if !RFID_LOG_MINIMAL
    LOG_WRN(
        "[CARD-PRESENT] direct_edges=%d, but not enough LONG intervals (%u)",
        demod_counter, env_long);
#endif
    reset_cross_scan_vote_pool();
    return 2;
  }
#else
  if (demod_counter < EM_MIN_STORED_TO_DECODE ||
      env_long < EM_MIN_LONG_TICKS_TO_DECODE) {
#if RFID_DECODE_DIAG_LOG
    LOG_INF("decode_skip: stored=%d/%u env_long=%u/%u", demod_counter,
            EM_MIN_STORED_TO_DECODE, env_long, EM_MIN_LONG_TICKS_TO_DECODE);
#endif
#if !RFID_LOG_MINIMAL
    LOG_WRN(
        "[CARD-PRESENT] all_edges=%u comp_edges=%u phy_edges=%u "
        "raw_edges=%u, "
        "but envelope ticks are not decodable",
        all_edges, comp_edges, phy_edges, raw_edges);
#endif
    reset_cross_scan_vote_pool();
    return 2;
  }
#endif

#if EM_ENABLE_TICK_FALLBACK_DIAG
  {
    uint64_t code;
    int bits = 0;
    bool inverted = false;
    int slide =
        try_comp_env_tick_decode(demod_counter, &code, &bits, &inverted);

    if (slide >= 0) {
      reset_cross_scan_vote_pool();
      return 1;
    }
  }
#endif

#if RFID_DECODE_DIAG_LOG
  LOG_INF("Fingerprint: %u, %u, %u, %u, %u, %u, %u, %u, %u, %u", tick_buffer[0],
          tick_buffer[1], tick_buffer[2], tick_buffer[3], tick_buffer[4],
          tick_buffer[5], tick_buffer[6], tick_buffer[7], tick_buffer[8],
          tick_buffer[9]);
#endif
#if !RFID_LOG_MINIMAL
  LOG_WRN(
      "[CARD-PRESENT] all_edges=%u comp_edges=%u phy_edges=%u raw_edges=%u, "
      "but no valid EM4100 frame yet",
      all_edges, comp_edges, phy_edges, raw_edges);
#endif
  return 2;
}
#endif

#if !RFID_USE_SAADC_RECEIVER
static void comp_log_customer_hw_expectations(void) {
  LOG_INF(
      "RX HW: P0.04=CMP_P slow (R16/C43, TP2); P0.05=CMP_N fast (R19/C42, "
      "TP3)");
  LOG_INF(
      "RX HW: 1V65_LF biases op-amp only — do NOT tie P0.05 to fixed 1.65V");
  LOG_INF(
      "RX scope: TP2/TP3 = envelope data slice (~256/512us), NOT 125kHz "
      "carrier");
#if RFID_COMP_DIRECT_EDGE_DECODE
  LOG_INF(
      "RX FW: DIFF th=0/0 hyst~50mV; edge gaps %u..%u us (DEMOD_OUT / ACMP "
      "PD3/PD4)",
      EM_EDGE_MIN_US, RFID_COMP_DIRECT_LONG_MAX_US);
#else
  LOG_INF("RX FW: ENV %uus windows count carrier edges (non-Mavericks path)",
          EM_ENV_WINDOW_US);
#endif
}

static void comp_log_silent_hint(uint32_t env_windows,
                                 const uint32_t* env_duty_hist,
                                 uint32_t comp_edges) {
  const char* stuck = "no-toggle";

  if (comp_edges != 0U) {
    return;
  }
  if (env_windows > 0U && env_duty_hist[0] == env_windows) {
    stuck = "stuck-LOW(CMP_P<CMP_N)";
  } else if (env_windows > 0U && env_duty_hist[16] == env_windows) {
    stuck = "stuck-HIGH(CMP_P>CMP_N)";
  }
  LOG_WRN("COMP silent (%s): verify TP2/TP3 waveforms cross; PWM carrier on",
          stuck);
}

static void comp_force_max_hysteresis(void) {
#if defined(NRF_COMP) && defined(NRF_COMP_HYST_50MV)
  nrf_comp_hysteresis_set(NRF_COMP, NRF_COMP_HYST_50MV);
  LOG_INF("COMP: HAL hysteresis forced to 50mV (Nordic max, vs SL 30mV sym)");
#elif defined(NRF_COMP) && defined(NRF_COMP_HYST_40MV)
  nrf_comp_hysteresis_set(NRF_COMP, NRF_COMP_HYST_40MV);
  LOG_INF("COMP: HAL hysteresis forced to 40mV");
#else
  LOG_INF("COMP: HAL hysteresis force unavailable on this target");
#endif
}

#if RFID_COMP_USE_NRFX_DIRECT
static void comp_nrfx_event_handler(nrf_comp_event_t event) {
  switch (event) {
    case NRF_COMP_EVENT_READY:
      comp_nrfx_irq_ready++;
      break;
    case NRF_COMP_EVENT_CROSS:
      comp_nrfx_irq_cross++;
      break;
    case NRF_COMP_EVENT_UP:
      comp_nrfx_irq_up++;
      break;
    case NRF_COMP_EVENT_DOWN:
      comp_nrfx_irq_down++;
      break;
    default:
      break;
  }
}

static nrfx_comp_config_t comp_nrfx_build_config(void) {
#if RFID_COMP_USE_INTERNAL_THRESHOLD
  return (nrfx_comp_config_t){
      .reference = NRF_COMP_REF_INT_1V8,
      .ext_ref = NRF_COMP_EXT_REF_1,
      .main_mode = NRF_COMP_MAIN_MODE_SE,
      .threshold =
          {
              .th_down = RFID_COMP_THRESHOLD_DOWN,
              .th_up = RFID_COMP_THRESHOLD_UP,
          },
      .speed_mode = RFID_COMP_SP_MODE,
      .hyst = NRF_COMP_HYST_NO_HYST,
      .isource = RFID_COMP_ISOURCE,
      /* P0.04=AIN0=CMP_P；nRF5340 COMP INPUT_0 對應 P0.04 */
      .input = NRF_COMP_INPUT_0,
      .interrupt_priority = NRFX_COMP_DEFAULT_CONFIG_IRQ_PRIORITY,
  };
#else
  /* DIFF：VIN+=AIN0(CMP_P 慢)、VIN-=AIN1(CMP_N 快)。須 REF_AREF 才會寫入
   * EXTREFSEL； 若用 INT_1V8，nrfx 不會接 AIN1，等於單端對內部 1.8V（與示波器
   * CH1-CH2 不一致）。 */
  return (nrfx_comp_config_t){
      .reference = NRF_COMP_REF_AREF,
      .ext_ref = NRF_COMP_EXT_REF_1,
      .main_mode = NRF_COMP_MAIN_MODE_DIFF,
      .threshold = {.th_down = 0, .th_up = 0},
      .speed_mode = NRF_COMP_SP_MODE_HIGH,
      .hyst = NRF_COMP_HYST_50MV,
      .isource = RFID_COMP_ISOURCE,
      .input = NRF_COMP_INPUT_0,
      .interrupt_priority = NRFX_COMP_DEFAULT_CONFIG_IRQ_PRIORITY,
  };
#endif
}

static int comp_nrfx_arm_receiver(void) {
  nrfx_err_t err;
  nrfx_comp_config_t comp_cfg = comp_nrfx_build_config();

  comp_nrfx_armed = false;

#if RFID_COMP_RX_GPIO_PINCNF_ENABLE
  rfid_rx_ain_gpio_configure();
#endif

#if defined(__ZEPHYR__)
  if (!nrfx_comp_init_check()) {
    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_COMP), IRQ_PRIO_LOWEST,
                nrfx_comp_irq_handler, 0, 0);
    irq_enable(NRFX_IRQ_NUMBER_GET(NRF_COMP));
  }
#endif

  if (!nrfx_comp_init_check()) {
    err = nrfx_comp_init(&comp_cfg, comp_nrfx_event_handler);
    if (err != NRFX_SUCCESS) {
      LOG_ERR("nrfx_comp_init failed: %d", err);
      return -EIO;
    }
  } else {
    err = nrfx_comp_reconfigure(&comp_cfg);
    if (err != NRFX_SUCCESS) {
      LOG_ERR("nrfx_comp_reconfigure failed: %d", err);
      return -EIO;
    }
  }

  comp_force_max_hysteresis();
#if !RFID_COMP_USE_INTERNAL_THRESHOLD
  /* 再寫一次，避免 reconfigure 路徑漏接 AIN1（對齊 Zephyr
   * comp_nrf_comp_configure_diff） */
  nrf_comp_ref_set(NRF_COMP, NRF_COMP_REF_AREF);
  nrf_comp_ext_ref_set(NRF_COMP, NRF_COMP_EXT_REF_1);
  nrf_comp_input_select(NRF_COMP, NRF_COMP_INPUT_0);
  nrf_comp_main_mode_set(NRF_COMP, NRF_COMP_MAIN_MODE_DIFF);
#endif
#if RFID_COMP_USE_INTERNAL_THRESHOLD
#if RFID_COMP_RX_GPIO_PINCNF_ENABLE
  LOG_INF("COMP nrfx: SE th=%u/%u sp=%s isource=%d gpio=%s",
          RFID_COMP_THRESHOLD_DOWN, RFID_COMP_THRESHOLD_UP,
          (RFID_COMP_SP_MODE == NRF_COMP_SP_MODE_LOW) ? "LOW" : "HIGH",
          (int)RFID_COMP_ISOURCE,
          rfid_rx_gpio_drive_name(RFID_COMP_RX_GPIO_DRIVE));
#else
  LOG_INF("COMP nrfx: SE th=%u/%u sp=%s isource=%d gpio=off",
          RFID_COMP_THRESHOLD_DOWN, RFID_COMP_THRESHOLD_UP,
          (RFID_COMP_SP_MODE == NRF_COMP_SP_MODE_LOW) ? "LOW" : "HIGH",
          (int)RFID_COMP_ISOURCE);
#endif
#else
  LOG_INF(
      "COMP nrfx: DIFF AREF AIN0(CMP_P) vs AIN1(CMP_N) th=0/0 sp=HIGH "
      "hyst=50mV");
  comp_log_customer_hw_expectations();
#endif

  nrfx_comp_start(NRF_COMP_INT_CROSS_MASK | NRF_COMP_INT_UP_MASK |
                      NRF_COMP_INT_DOWN_MASK | NRF_COMP_INT_READY_MASK,
                  0);
  comp_nrfx_armed = true;
  LOG_INF("nRF5340 COMP receiver armed (nrfx, poll + IRQ stats)");
  return 0;
}
#endif /* RFID_COMP_USE_NRFX_DIRECT */

static void comp_reapply_receiver_config(void) {
#if RFID_COMP_USE_NRFX_DIRECT
  nrf_gpio_cfg(RFID_RX_AIN0_PIN, NRF_GPIO_PIN_DIR_INPUT,
               NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,
               NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);

  nrf_gpio_cfg(RFID_RX_AIN1_PIN, NRF_GPIO_PIN_DIR_INPUT,
               NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,
               NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);
  (void)comp_nrfx_arm_receiver();
#else
  if (!device_is_ready(comp_dev)) {
    return;
  }
#if RFID_COMP_USE_INTERNAL_THRESHOLD
  const struct comp_nrf_comp_se_config cfg = {
      .psel = COMP_NRF_COMP_PSEL_AIN0,
      .sp_mode = COMP_NRF_COMP_SP_MODE_HIGH,
      .isource = COMP_NRF_COMP_ISOURCE_DISABLED,
      .extrefsel = COMP_NRF_COMP_EXTREFSEL_AIN1,
      .refsel = RFID_COMP_REFSEL,
      .th_down = RFID_COMP_THRESHOLD_DOWN,
      .th_up = RFID_COMP_THRESHOLD_UP,
  };
  int r = comp_nrf_comp_configure_se(comp_dev, &cfg);
  if (r != 0) {
    LOG_WRN("comp re-apply SE AIN0/internal-ref failed: %d", r);
  } else {
    comp_force_max_hysteresis();
    LOG_INF("COMP: SE AIN0 vs INT1V8 th_up=%u th_down=%u",
            RFID_COMP_THRESHOLD_UP, RFID_COMP_THRESHOLD_DOWN);
  }
#else
  const struct comp_nrf_comp_diff_config cfg = {
      .psel = COMP_NRF_COMP_PSEL_AIN0,
      .sp_mode = COMP_NRF_COMP_SP_MODE_HIGH,
      .isource = COMP_NRF_COMP_ISOURCE_DISABLED,
      .extrefsel = COMP_NRF_COMP_EXTREFSEL_AIN1,
      .enable_hyst = true,
  };
  int r = comp_nrf_comp_configure_diff(comp_dev, &cfg);
  if (r != 0) {
    LOG_WRN("comp re-apply diff AIN0/AIN1 failed: %d", r);
  } else {
    LOG_INF("COMP: DIFF AIN0 vs AIN1 + hysteresis ON (Zephyr shim)");
  }
#endif
#endif /* !RFID_COMP_USE_NRFX_DIRECT */
}
#endif /* !RFID_USE_SAADC_RECEIVER */

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
#if RFID_USE_HIGH_RES_TIMER
  rfid_high_res_timer_init();
  LOG_INF("RF timing: TIMER1 1MHz (1us/tick)");
#endif

#if RFID_USE_SAADC_RECEIVER
  LOG_INF("RX path: SAADC differential AIN0-AIN1");
#else
  LOG_INF(
      "RX path: COMP %s (%s) th=%u/%u sp=%s gpio=%s duty=%u",
      RFID_COMP_USE_INTERNAL_THRESHOLD ? "SE AIN0/INT1V8" : "DIFF AIN0/AIN1",
      RFID_COMP_USE_NRFX_DIRECT ? "nrfx" : "zephyr", RFID_COMP_THRESHOLD_DOWN,
      RFID_COMP_THRESHOLD_UP,
      RFID_COMP_USE_INTERNAL_THRESHOLD
          ? ((RFID_COMP_SP_MODE == NRF_COMP_SP_MODE_LOW) ? "LOW" : "HIGH")
          : "HIGH",
#if RFID_COMP_RX_GPIO_PINCNF_ENABLE
      rfid_rx_gpio_drive_name(RFID_COMP_RX_GPIO_DRIVE),
#else
      "off",
#endif
      RFID_PWM_DUTY_REFERENCE);
  comp_log_customer_hw_expectations();
  comp_reapply_receiver_config();
#endif

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
#if !RFID_USE_SAADC_RECEIVER && RFID_COMP_USE_NRFX_DIRECT
    {
      int boot_sample = (int)nrfx_comp_sample();

      LOG_INF(
          "COMP sample after PWM=%d (1=CMP_P>CMP_N); card@TP2/TP3 should "
          "toggle",
          boot_sample);
    }
#endif
#if RFID_USE_SAADC_RECEIVER && RFID_FSK_BOOTSTRAP_PHASE_LOCK
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
#if RFID_USE_SAADC_RECEIVER
    if (!em4095_saadc_receiver()) {
#else
    if (!em4095_comp_receiver()) {
#endif
#if RFID_MAIN_IDLE_LOG
      LOG_INF("No card detected.");
#endif
    }
  }
  return 0;
}