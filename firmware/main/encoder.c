/**
 * encoder.c — Quadrature encoder velocity feedback (008-encoder-feedback)
 *
 * Uses ESP32-S3 PCNT hardware units (4X quadrature).
 *
 * Architecture:
 *   encoder_init()      — allocate PCNT units/channels; clear counters
 *   encoder_start()     — enable PCNT units
 *   encoder_sample_now() — task-context periodic sampler:
 *     1. Read Δcount = current_count - last_count for each PCNT unit
 *     2. Apply noise-floor suppression (|Δcount| < NOISE_FLOOR → ω = 0)
 *     3. Compute ω = Δcount × 2π / (CPR × Δt_s)
 *     4. Clamp to ±MAX_SPEED_RAD_S; set fault flag on clamp
 *     5. Write to g_encoder_velocities[], g_encoder_counts[], g_encoder_faults[]
 *     6. Apply dir_sign from g_cal_params (009 integration)
 *
 * Concurrency constraints:
 *   - No malloc / free in the sampling hot-path
 *   - All shared globals are volatile, 32-bit aligned
 */

#include "encoder.h"
#include "motor.h"          /* MAX_SPEED_RAD_S */
#include "calibration.h"    /* g_cal_params.dir_sign[] (009) */

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "encoder";

/* ─── Shared globals (ISR-safe) ─────────────────────────────────────────── */
volatile float   g_encoder_velocities[4] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile int32_t g_encoder_counts[4]     = {0, 0, 0, 0};
volatile int32_t g_encoder_last_delta[4] = {0, 0, 0, 0};
volatile uint32_t g_encoder_sample_seq   = 0;
volatile bool    g_encoder_faults[4]     = {false, false, false, false};

/* ─── Private state ─────────────────────────────────────────────────────── */

typedef struct {
    pcnt_unit_handle_t    unit;
    pcnt_channel_handle_t chan_a;
    pcnt_channel_handle_t chan_b;
    int                   pin_a;
    int                   pin_b;
    int                   last_count;
} EncoderChannel;

static EncoderChannel s_enc[4];
static bool s_encoder_enabled = false;

/* GPIO pin table indexed by wheel (FL=0, FR=1, RL=2, RR=3) */
static const int k_pin_a[4] = {
    CONFIG_ENCODER_FL_PIN_A,
    CONFIG_ENCODER_FR_PIN_A,
    CONFIG_ENCODER_RL_PIN_A,
    CONFIG_ENCODER_RR_PIN_A,
};
static const int k_pin_b[4] = {
    CONFIG_ENCODER_FL_PIN_B,
    CONFIG_ENCODER_FR_PIN_B,
    CONFIG_ENCODER_RL_PIN_B,
    CONFIG_ENCODER_RR_PIN_B,
};

/* Precomputed: 2π / (CPR × Δt_s) — computed once at init */
static float s_omega_scale = 0.0f;

/* Max Δcount before declaring a fault: 1.5× the expected max ticks/period */
static int s_max_ticks_per_period = 0;

void encoder_sample_now(void)
{
    if (!s_encoder_enabled) {
        return;
    }

    for (int i = 0; i < 4; i++) {
        int current = 0;
        pcnt_unit_get_count(s_enc[i].unit, &current);

        int delta = current - s_enc[i].last_count;
        g_encoder_last_delta[i] = delta;
        s_enc[i].last_count = current;

        /* Update cumulative count (signed) */
        g_encoder_counts[i] += (int32_t)delta;

        /* Noise-floor suppression */
        if (delta > -CONFIG_ENCODER_NOISE_FLOOR && delta < CONFIG_ENCODER_NOISE_FLOOR) {
            g_encoder_velocities[i] = 0.0f;
            g_encoder_faults[i] = false;
            continue;
        }

        /* Fault detection: implausibly high tick rate */
        if (delta > s_max_ticks_per_period || delta < -s_max_ticks_per_period) {
            g_encoder_faults[i] = true;
            g_encoder_velocities[i] = 0.0f;
            continue;
        }

        g_encoder_faults[i] = false;

        float omega = (float)delta * s_omega_scale;

        /* Apply calibration direction sign (+1/-1) from 009 params. */
        int8_t sign = g_cal_params.dir_sign[i];
        if (sign == 0) sign = 1;
        omega *= (float)sign;

        if (omega > MAX_SPEED_RAD_S) {
            omega = MAX_SPEED_RAD_S;
            g_encoder_faults[i] = true;
        } else if (omega < -MAX_SPEED_RAD_S) {
            omega = -MAX_SPEED_RAD_S;
            g_encoder_faults[i] = true;
        }

        g_encoder_velocities[i] = omega;
    }

    g_encoder_sample_seq++;
}

/* ─── encoder_init() ─────────────────────────────────────────────────────── */

void encoder_init(void)
{
    s_encoder_enabled = false;

    /* Validate Kconfig parameters */
    if (CONFIG_ENCODER_CPR <= 0) {
        ESP_LOGE(TAG, "invalid CONFIG_ENCODER_CPR=%d", CONFIG_ENCODER_CPR);
        return;
    }

    /* Precompute omega scale factor: 2π / (CPR × Δt_s) */
    float dt_s = CONFIG_ENCODER_SAMPLE_PERIOD_MS / 1000.0f;
    s_omega_scale = (2.0f * (float)M_PI) / ((float)CONFIG_ENCODER_CPR * dt_s);

    /* Max ticks per period = MAX_SPEED / omega_scale × 1.5 (fault threshold) */
    s_max_ticks_per_period = (int)(MAX_SPEED_RAD_S / s_omega_scale * 1.5f) + 1;

    /* Ensure stable logic levels for encoder inputs. Many quadrature encoder
     * boards expose open-collector/open-drain outputs and require pull-ups. */
    gpio_config_t enc_gpio_cfg = {
        .pin_bit_mask =
            (1ULL << CONFIG_ENCODER_FL_PIN_A) |
            (1ULL << CONFIG_ENCODER_FL_PIN_B) |
            (1ULL << CONFIG_ENCODER_FR_PIN_A) |
            (1ULL << CONFIG_ENCODER_FR_PIN_B) |
            (1ULL << CONFIG_ENCODER_RL_PIN_A) |
            (1ULL << CONFIG_ENCODER_RL_PIN_B) |
            (1ULL << CONFIG_ENCODER_RR_PIN_A) |
            (1ULL << CONFIG_ENCODER_RR_PIN_B),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&enc_gpio_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config failed: %s", esp_err_to_name(err));
        return;
    }

    for (int i = 0; i < 4; i++) {
        s_enc[i].pin_a = k_pin_a[i];
        s_enc[i].pin_b = k_pin_b[i];
        s_enc[i].last_count = 0;

        /* ── Allocate PCNT unit ── */
        pcnt_unit_config_t unit_cfg = {
            .low_limit  = -32768,
            .high_limit =  32767,
            .flags = { .accum_count = 1 },   /* Accumulate across ±32767 watchpoints */
        };
        err = pcnt_new_unit(&unit_cfg, &s_enc[i].unit);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "pcnt_new_unit[%d] failed: %s", i, esp_err_to_name(err));
            return;
        }

        /* ── Glitch filter ── */
        if (CONFIG_ENCODER_GLITCH_NS > 0) {
            pcnt_glitch_filter_config_t filter_cfg = {
                .max_glitch_ns = CONFIG_ENCODER_GLITCH_NS,
            };
            err = pcnt_unit_set_glitch_filter(s_enc[i].unit, &filter_cfg);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "pcnt_glitch_filter[%d] failed: %s", i, esp_err_to_name(err));
                return;
            }
        }

        /* ── Channel A: edge on pin_a, level on pin_b ── */
        pcnt_chan_config_t chan_a_cfg = {
            .edge_gpio_num  = s_enc[i].pin_a,
            .level_gpio_num = s_enc[i].pin_b,
        };
        err = pcnt_new_channel(s_enc[i].unit, &chan_a_cfg, &s_enc[i].chan_a);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "pcnt_new_channel A[%d] failed: %s", i, esp_err_to_name(err));
            return;
        }
        err = pcnt_channel_set_edge_action(s_enc[i].chan_a,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "pcnt_edge_action A[%d] failed: %s", i, esp_err_to_name(err));
            return;
        }
        err = pcnt_channel_set_level_action(s_enc[i].chan_a,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "pcnt_level_action A[%d] failed: %s", i, esp_err_to_name(err));
            return;
        }

        /* ── Channel B: edge on pin_b, level on pin_a ── */
        pcnt_chan_config_t chan_b_cfg = {
            .edge_gpio_num  = s_enc[i].pin_b,
            .level_gpio_num = s_enc[i].pin_a,
        };
        err = pcnt_new_channel(s_enc[i].unit, &chan_b_cfg, &s_enc[i].chan_b);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "pcnt_new_channel B[%d] failed: %s", i, esp_err_to_name(err));
            return;
        }
        err = pcnt_channel_set_edge_action(s_enc[i].chan_b,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "pcnt_edge_action B[%d] failed: %s", i, esp_err_to_name(err));
            return;
        }
        err = pcnt_channel_set_level_action(s_enc[i].chan_b,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "pcnt_level_action B[%d] failed: %s", i, esp_err_to_name(err));
            return;
        }

        /* ── Add ±32767 watchpoints to enable accumulation ── */
        err = pcnt_unit_add_watch_point(s_enc[i].unit, 32767);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "pcnt_watch_point +[%d] failed: %s", i, esp_err_to_name(err));
            return;
        }
        err = pcnt_unit_add_watch_point(s_enc[i].unit, -32768);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "pcnt_watch_point -[%d] failed: %s", i, esp_err_to_name(err));
            return;
        }

        /* ── Clear counter ── */
        err = pcnt_unit_clear_count(s_enc[i].unit);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "pcnt_clear_count[%d] failed: %s", i, esp_err_to_name(err));
            return;
        }
    }

    s_encoder_enabled = true;

    ESP_LOGI(TAG, "encoder_init: CPR=%d period=%dms scale=%.4f max_ticks=%d",
             CONFIG_ENCODER_CPR, CONFIG_ENCODER_SAMPLE_PERIOD_MS,
             (double)s_omega_scale, s_max_ticks_per_period);
}

/* ─── encoder_start() ────────────────────────────────────────────────────── */

void encoder_start(void)
{
    if (!s_encoder_enabled) {
        ESP_LOGW(TAG, "encoder_start skipped: encoder not initialized");
        return;
    }

    /* Enable and start all 4 PCNT units */
    for (int i = 0; i < 4; i++) {
        esp_err_t err = pcnt_unit_enable(s_enc[i].unit);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "pcnt_unit_enable[%d] failed: %s", i, esp_err_to_name(err));
            s_encoder_enabled = false;
            return;
        }
        err = pcnt_unit_start(s_enc[i].unit);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "pcnt_unit_start[%d] failed: %s", i, esp_err_to_name(err));
            s_encoder_enabled = false;
            return;
        }
    }

    ESP_LOGI(TAG, "encoder_start: PCNT counting active; sample by task timer at %d Hz",
             1000 / CONFIG_ENCODER_SAMPLE_PERIOD_MS);
}
