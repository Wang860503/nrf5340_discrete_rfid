/*
 * 最終修訂版：優化抗噪門檻，解決 176 bits 無法對齊 Header 的問題
 */

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/comparator.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(rfid_main);

#define PWM_PERIOD 128
/*
 * PWM pulse width (cycles out of PWM_PERIOD). Vendor example often uses
 * ~60/128. Higher duty drives the coil harder (more current on CLK_P/CLK_N
 * half-bridge). On a marginal supply (e.g. 3.0 V, thin USB cable, small bulk
 * caps), that extra current drops VDD and the SoC brown-out resets — "reboot
 * loop" when set to 60. Keep 40 for bring-up; move toward 60 only after:
 * solid 3.3 V rail, adequate current rating, short/heavy GND return, and
 * bulk/decoupling at MCU and analog front-end.
 */
#define RFID_50_PERCENT_DUTY 35
/* Logs showed stored==1000 (saturated); later edges dropped → misaligned frame
 */
#define TICK_BUFFER_SIZE 2048
/* Manchester output cap; logs often hit 512 while ticks still saturate */
#define EM_DECODED_BITS_CAP 1024

/*
 * Ignore edge-to-edge gaps <= this (us). Too low (e.g. 55) admits 125kHz
 * ripple: huge raw_edges/stored but decoded bits never show EM4100's 9 leading
 * 1s (see max run log).
 */
#define EM_EDGE_MIN_US 60

/* Try decode when at least this many intervals are stored (was hard-coded >64;
 * logs peaked ~60). */
#define EM_MIN_STORED_TO_DECODE 50

#define RFID_CAPTURE_WINDOW_MS 600

/*
 * First interval after arm often spans idle+settle (tens of ms) and must not
 * enter the buffer.
 */
#define EM_SKIP_FIRST_GAP_US 2500

/*
 * Manchester-ish buckets (us). Non-overlapping SHORT then LONG.
 * Fingerprint had gaps >2000us (e.g. 4–7ms): those used to hit else and reset
 * state every time, yielding max run ones/zeros ~2–4 (quasi-alternating
 * garbage).
 */
#define EM_SHORT_MIN 70
#define EM_SHORT_MAX 400
#define EM_LONG_MIN 410
#define EM_LONG_MAX 2200

/* Gaps above LONG but below this: ignore (no emit, no state reset) — envelope
 * dropout */
#define EM_MANCHESTER_GAP_HOLD_US 10000
/* Very long idle: resync decoder */
#define EM_MANCHESTER_RESYNC_US 15000

#define RFID_COMP_SAMPLE_INTERVAL_US 0
#define RFID_ENABLE_CARRIER_PWM 1

static const struct device* comp_dev = DEVICE_DT_GET(DT_NODELABEL(comp));
#if RFID_ENABLE_CARRIER_PWM
static const struct device* pwm_dev = DEVICE_DT_GET(DT_NODELABEL(pwm0));
#endif

static uint32_t tick_buffer[TICK_BUFFER_SIZE];
static uint8_t dec_raw_bits[EM_DECODED_BITS_CAP];
static uint8_t dec_saved_bits[EM_DECODED_BITS_CAP];
static uint8_t dec_work_bits[EM_DECODED_BITS_CAP];

void invert_bits(uint8_t* bits, int len) {
    for (int i = 0; i < len; i++) {
        bits[i] = !bits[i];
    }
}

/* 智慧搜尋：在位元流中尋找 EM4100 的 9 個連續 '1' */
int find_em4100_header(uint8_t* bits, int len) {
    if (len < 64) return -1;
    for (int i = 0; i <= len - 64; i++) {
        if (i > 0 && bits[i - 1] == 1) {
            continue; /* nine 1s must start a new run, not extend a longer run */
        }
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

static void em4100_max_runs(const uint8_t* bits, int len, int* max_ones,
                            int* max_zeros) {
    int best_o = 0;
    int best_z = 0;
    int run_o = 0;
    int run_z = 0;

    for (int i = 0; i < len; i++) {
        if (bits[i] == 1) {
            run_o++;
            run_z = 0;
            if (run_o > best_o) best_o = run_o;
        } else {
            run_z++;
            run_o = 0;
            if (run_z > best_z) best_z = run_z;
        }
    }
    *max_ones = best_o;
    *max_zeros = best_z;
}

int EM4100_Full_Check(uint8_t* bits) {
    /* Without this, try_em4100_slide matches random windows (parity is linear). */
    for (int j = 0; j < 9; j++) {
        if (bits[j] != 1) return 0;
    }
    /* Stop bit */
    if (bits[63] != 0) return 0;

    uint8_t col_parity[4] = {0};

    // 檢查 10 組 Row (Customer ID + Data)
    for (int row = 0; row < 10; row++) {
        int base = 9 + row * 5;
        int row_sum = 0;
        for (int col = 0; col < 4; col++) {
            uint8_t val = bits[base + col];
            row_sum += val;
            col_parity[col] += val;
        }
        // 偶同位檢查：數據和 + 同位位元
        if ((row_sum + bits[base + 4]) % 2 != 0) return 0;
    }

    // 檢查 Column Parity
    for (int col = 0; col < 4; col++) {
        if ((col_parity[col] + bits[59 + col]) % 2 != 0) return 0;
    }
    return 1;
}

/* Preamble aligned to field boundary (bit before frame must be 0 when present). */
static int em4100_valid_at(const uint8_t* bits, int len, int off) {
    if (off < 0 || off + 64 > len) return 0;
    if (off > 0 && bits[off - 1] != 0) return 0;
    return EM4100_Full_Check((uint8_t*)&bits[off]);
}

uint64_t decode_card_code(uint8_t* bits) {
    uint64_t code = 0;
    for (int j = 0; j < 10; j++) {
        unsigned int digit =
            (bits[9 + 5 * j] << 3) | (bits[9 + 5 * j + 1] << 2) |
            (bits[9 + 5 * j + 2] << 1) | (bits[9 + 5 * j + 3] << 0);
        code |= ((uint64_t)digit << (4 * (9 - j)));
    }
    return (code & 0xFFFFFFFF);
}

/*
 * Slide a 64-bit window; em4100_valid_at adds bit-before-preamble==0 + Full_Check.
 */
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

/* 9-run of 1s is not unique; try small bit slip around that index. */
static int try_em4100_near_header(const uint8_t* bits, int len, int h,
                                  uint64_t* out_code) {
    const int span = 16;

    if (len < 64 || h < 0) return -1;
    for (int d = -span; d <= span; d++) {
        int i = h + d;

        if (i < 0 || i + 64 > len) continue;
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
        } else if (T > EM_LONG_MAX && T < EM_MANCHESTER_GAP_HOLD_US) {
            /* Between-bit / AFE dropout; do not treat as symbol (avoids
             * constant resync) */
        } else if (T >= EM_MANCHESTER_RESYNC_US) {
            state = 0;
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

    LOG_DBG("scan start");

    if (!device_is_ready(comp_dev)) return 0;
#if RFID_ENABLE_CARRIER_PWM
    if (!device_is_ready(pwm_dev)) {
        LOG_ERR("pwm not ready");
        return 0;
    }
#endif

    if (comparator_set_trigger(comp_dev, COMPARATOR_TRIGGER_NONE) != 0) {
        LOG_ERR("comparator_set_trigger failed");
        return 0;
    }

#if RFID_ENABLE_CARRIER_PWM
    pwm_set_cycles(pwm_dev, 0, PWM_PERIOD, RFID_50_PERCENT_DUTY, 0);
    pwm_set_cycles(pwm_dev, 1, PWM_PERIOD, RFID_50_PERCENT_DUTY,
                   PWM_POLARITY_INVERTED);
#endif

    k_sleep(K_MSEC(50));

    int stable = comparator_get_output(comp_dev);
    uint8_t last_val = (stable > 0) ? 1 : 0;
    uint32_t last_edge_cycle = k_cycle_get_32();
    /* EM4100 會重複送框；約 400ms 採樣視窗 */
    uint64_t stop_time = k_uptime_get() + RFID_CAPTURE_WINDOW_MS;
    /*
     * Drop at most ONE leading interval > EM_SKIP_FIRST_GAP_US (idle after
     * arm). Do NOT use demod_counter==0 for this: buffer stays empty until
     * first store, so a second long gap would be wrongly skipped forever
     * (stored << raw_edges).
     */
    bool lead_sync_dropped = false;
    bool tick_window_slid = false;

    while (k_uptime_get() < stop_time) {
        int current_val = comparator_get_output(comp_dev);
        if (current_val < 0) continue;

        if (current_val != last_val) {
            uint32_t now = k_cycle_get_32();
            uint32_t diff_us = k_cyc_to_us_near32(now - last_edge_cycle);

            raw_edges++;
            if (diff_us > max_gap_us) {
                max_gap_us = diff_us;
            }
            if (diff_us > EM_EDGE_MIN_US) {
                if (!lead_sync_dropped) {
                    lead_sync_dropped = true;
                    if (diff_us <= EM_SKIP_FIRST_GAP_US) {
                        tick_buffer_append(diff_us, &demod_counter,
                                           &tick_window_slid);
                    }
                } else {
                    tick_buffer_append(diff_us, &demod_counter, &tick_window_slid);
                }
            }
            last_edge_cycle = now;
            last_val = current_val;
        }
    }

    if (tick_window_slid) {
        LOG_WRN(
            "High edge rate: tick window slid (oldest half discarded when "
            "full); decode uses last ~%d stored intervals",
            demod_counter);
    }
    LOG_INF("scan: raw_edges=%u stored=%d max_gap_us=%u (decode if stored>=%d)",
            raw_edges, demod_counter, max_gap_us, EM_MIN_STORED_TO_DECODE);
    if (raw_edges > 80 && demod_counter < EM_MIN_STORED_TO_DECODE) {
        LOG_WRN(
            "Most edge gaps <= EM_EDGE_MIN_US (carrier ripple or threshold too "
            "high); "
            "lower EM_EDGE_MIN_US or improve analog envelope");
    }
    if (raw_edges == 0) {
        LOG_WRN(
            "No edges: check EN/3V3_LF, CMP_P/CMP_N, carrier at coil, COMP "
            "enabled");
    }

#if RFID_ENABLE_CARRIER_PWM
    pwm_set_cycles(pwm_dev, 0, PWM_PERIOD, 0, 0);
    pwm_set_cycles(pwm_dev, 1, PWM_PERIOD, 0, 0);
#endif

    if (demod_counter >= EM_MIN_STORED_TO_DECODE) {
        int in_short = 0;
        int in_long = 0;
        int in_other = 0;

        for (int i = 0; i < demod_counter; i++) {
            uint32_t T = tick_buffer[i];

            if (T < EM_EDGE_MIN_US) {
                continue;
            }
            if (T >= EM_LONG_MIN && T <= EM_LONG_MAX) {
                in_long++;
            } else if (T >= EM_SHORT_MIN && T <= EM_SHORT_MAX) {
                in_short++;
            } else {
                in_other++;
            }
        }

        int raw_len = 0;
        decode_bitstream(demod_counter, dec_raw_bits, &raw_len);

        LOG_INF(
            "Captured: %d ticks, Decoded: %d bits, buckets short/long/other: "
            "%d/%d/%d",
            demod_counter, raw_len, in_short, in_long, in_other);
        if (raw_len >= EM_DECODED_BITS_CAP) {
            LOG_WRN("Decoded bit stream at cap (%d); late frames may be missing",
                    EM_DECODED_BITS_CAP);
        }

        if (raw_len < 64) {
            LOG_INF(
                "Bits < 64: pulse widths mostly outside SHORT/LONG; tune "
                "EM_SHORT_* / "
                "EM_LONG_* from buckets (RF/64 ~256us half-bit, ~512us "
                "full-bit scale)");
        }

        memcpy(dec_saved_bits, dec_raw_bits, (size_t)raw_len);

        int max_o;
        int max_z;
        em4100_max_runs(dec_saved_bits, raw_len, &max_o, &max_z);
        LOG_INF(
            "Bit stream: max run ones=%d zeros=%d (EM4100 preamble needs 9 "
            "ones)",
            max_o, max_z);
        if (max_o < 9) {
            LOG_WRN(
                "No EM4100-like preamble: raise EM_EDGE_MIN_US, fix analog "
                "filter/resonance, "
                "or verify EM4100 tag + 125kHz field");
        }

        /* Normal polarity */
        int hn = find_em4100_header(dec_saved_bits, raw_len);
        if (hn >= 0) {
            if (em4100_valid_at(dec_saved_bits, raw_len, hn)) {
                LOG_WRN("Card ID: %08llX",
                        decode_card_code(&dec_saved_bits[hn]));
                return 1;
            }
            LOG_INF("Header at %d but parity/stop check failed", hn);
            uint64_t code_near;
            int nn =
                try_em4100_near_header(dec_saved_bits, raw_len, hn, &code_near);
            if (nn >= 0) {
                LOG_WRN("Card ID (near hdr %d -> %d): %08llX", hn, nn,
                        code_near);
                return 1;
            }
        }

        /* Inverted copy */
        memcpy(dec_work_bits, dec_saved_bits, (size_t)raw_len);
        invert_bits(dec_work_bits, raw_len);
        int hi = find_em4100_header(dec_work_bits, raw_len);
        if (hi >= 0) {
            if (em4100_valid_at(dec_work_bits, raw_len, hi)) {
                LOG_WRN("Card ID (inverted): %08llX",
                        decode_card_code(&dec_work_bits[hi]));
                return 1;
            }
            LOG_INF("Header (inverted) at %d but parity/stop check failed", hi);
            uint64_t code_near_inv;
            int ni = try_em4100_near_header(dec_work_bits, raw_len, hi,
                                            &code_near_inv);
            if (ni >= 0) {
                LOG_WRN("Card ID (near inv hdr %d -> %d): %08llX", hi, ni,
                        code_near_inv);
                return 1;
            }
        }

        if (hn < 0 && hi < 0) {
            LOG_INF("No 9-run header; trying sliding 64-bit parity match");
        }

        uint64_t code;
        int slide = try_em4100_slide(dec_saved_bits, raw_len, &code);
        if (slide >= 0) {
            LOG_WRN("Card ID (slide offset %d): %08llX", slide, code);
            return 1;
        }
        memcpy(dec_work_bits, dec_saved_bits, (size_t)raw_len);
        invert_bits(dec_work_bits, raw_len);
        slide = try_em4100_slide(dec_work_bits, raw_len, &code);
        if (slide >= 0) {
            LOG_WRN("Card ID (slide inv offset %d): %08llX", slide, code);
            return 1;
        }

        /* Fingerprint of first five stored intervals (us) */
        LOG_INF("Fingerprint: %u, %u, %u, %u, %u", tick_buffer[0],
                tick_buffer[1], tick_buffer[2], tick_buffer[3], tick_buffer[4]);
    }
    return 0;
}

int main(void) {
    LOG_INF("nRF5340 Discrete RFID Starting...");
    LOG_INF(
        "Tuning: edge_min_us=%u min_stored=%u capture_ms=%u (if scan line "
        "differs, rebuild/flash)",
        EM_EDGE_MIN_US, EM_MIN_STORED_TO_DECODE, RFID_CAPTURE_WINDOW_MS);
    while (1) {
        if (!em4095_comp_receiver()) {
            LOG_INF("No card detected.");
        }
        k_sleep(K_MSEC(1500));
    }
    return 0;
}