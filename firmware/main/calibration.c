/**
 * calibration.c — Encoder auto-calibration (009-encoder-auto-cal-brat-on)
 *
 * Implements:
 *   calibration_params_load()        — NVS load at boot (T005)
 *   cal_nvs_keys_absent()            — fast NVS probe (T006)
 *   cal_gpio_trigger_active()        — GPIO 22 active-low check (T007)
 *   calibration_run()                — direction + speed passes (T010–T015, T026)
 *   calibration_reset_callback()     — /calibration_reset service (T018)
 *
 * T026 motion guard: calibration_run() checks g_encoder_velocities[] > 0.5 rad/s
 * and aborts if rover is not stationary.
 *
 * NVS key layout (all under "rover_cfg" namespace):
 *   cal_dir_0..3  — int8_t direction signs (±1)
 *   cal_spd_0..3  — float speed scales [0.5, 2.0] stored as u32 (memcpy)
 *
 * Timing per wheel burst:
 *   Direction pass: set_pwm → 350ms settle → read → stop_all → 150ms coast
 *   Speed LOW:      set_pwm → 350ms settle → read → stop_all → 150ms coast
 *   Speed HIGH:     set_pwm → 350ms settle → read → stop_all → 150ms coast
 * Total: 4 wheels × 3 bursts × 500ms = ~6 s (well within SC-004 ≤30 s).
 */

#include "calibration.h"
#include "encoder.h"        /* g_encoder_velocities[] */
#include "motor.h"          /* motor_set_pwm, motor_stop_all, PWM_MAX_TICKS, MAX_SPEED_RAD_S */
#include "nvs_config.h"     /* NVS_NAMESPACE, NVS_KEY_CAL_DIR_*, NVS_KEY_CAL_SPD_* */

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "driver/gpio.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "calibration";

/* ─── Global calibration state ──────────────────────────────────────────── */

volatile CalibrationParams g_cal_params = {
    .dir_sign    = {+1, +1, +1, +1},
    .speed_scale = {1.0f, 1.0f, 1.0f, 1.0f},
    .calibrated  = false,
};

/* ─── NVS key tables ─────────────────────────────────────────────────────── */

static const char * const k_dir_keys[4] = {
    NVS_KEY_CAL_DIR_0, NVS_KEY_CAL_DIR_1,
    NVS_KEY_CAL_DIR_2, NVS_KEY_CAL_DIR_3,
};
static const char * const k_spd_keys[4] = {
    NVS_KEY_CAL_SPD_0, NVS_KEY_CAL_SPD_1,
    NVS_KEY_CAL_SPD_2, NVS_KEY_CAL_SPD_3,
};

/* ─── calibration_params_load() ─────────────────────────────────────────── */

void calibration_params_load(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "calibration_params_load: nvs_open failed (%s) — using defaults",
                 esp_err_to_name(err));
        return;
    }

    bool all_dirs_found = true;

    for (int i = 0; i < 4; i++) {
        int8_t sign = +1;
        err = nvs_get_i8(h, k_dir_keys[i], &sign);
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            sign = +1;
            all_dirs_found = false;
            ESP_LOGD(TAG, "cal_dir_%d absent — defaulting to +1", i);
        } else if (err != ESP_OK) {
            ESP_LOGW(TAG, "cal_dir_%d read error: %s — defaulting to +1", i,
                     esp_err_to_name(err));
            sign = +1;
            all_dirs_found = false;
        }
        g_cal_params.dir_sign[i] = sign;
    }

    /* g_cal_params.calibrated = true only when ALL 4 direction keys present.
     * Partial NVS corruption is treated as uncalibrated (T005 spec).       */
    g_cal_params.calibrated = all_dirs_found;

    /* Only load speed scales when direction calibration is complete.
     * If not fully calibrated, speed scale defaults stay 1.0 — prevents
     * stale or garbage NVS float values from being applied to motor output. */
    if (g_cal_params.calibrated) {
        for (int i = 0; i < 4; i++) {
            uint32_t bits = 0;
            float scale = 1.0f;
            err = nvs_get_u32(h, k_spd_keys[i], &bits);
            if (err == ESP_OK) {
                memcpy(&scale, &bits, sizeof(float));
                /* Sanity check: reject NaN/Inf or out-of-range values.
                 * Valid calibration range is inclusive [0.5, 2.0]. */
                if (!isfinite(scale) || scale < 0.5f || scale > 2.0f) {
                    ESP_LOGW(TAG, "cal_spd_%d invalid value %.4f — defaulting to 1.0",
                             i, (double)scale);
                    scale = 1.0f;
                }
            } else if (err != ESP_ERR_NVS_NOT_FOUND) {
                ESP_LOGW(TAG, "cal_spd_%d read error: %s — defaulting to 1.0", i,
                         esp_err_to_name(err));
            }
            g_cal_params.speed_scale[i] = scale;
        }
    } else {
        /* Uncalibrated — ensure speed scales are 1.0 (safe default) */
        for (int i = 0; i < 4; i++) {
            g_cal_params.speed_scale[i] = 1.0f;
        }
        ESP_LOGD(TAG, "calibration_params_load: not calibrated — speed scales set to 1.0");
    }

    nvs_close(h);

    ESP_LOGI(TAG, "calibration loaded: calibrated=%s "
             "dir=[%+d %+d %+d %+d] scale=[%.3f %.3f %.3f %.3f]",
             g_cal_params.calibrated ? "yes" : "no",
             (int)g_cal_params.dir_sign[0], (int)g_cal_params.dir_sign[1],
             (int)g_cal_params.dir_sign[2], (int)g_cal_params.dir_sign[3],
             (double)g_cal_params.speed_scale[0], (double)g_cal_params.speed_scale[1],
             (double)g_cal_params.speed_scale[2], (double)g_cal_params.speed_scale[3]);
}

/* ─── cal_nvs_keys_absent() ─────────────────────────────────────────────── */

bool cal_nvs_keys_absent(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) {
        return true;   /* NVS unavailable → treat as absent */
    }

    int8_t dummy = 0;
    err = nvs_get_i8(h, NVS_KEY_CAL_DIR_0, &dummy);
    nvs_close(h);

    return (err == ESP_ERR_NVS_NOT_FOUND);
}

/* ─── cal_gpio_trigger_active() ─────────────────────────────────────────── */

bool cal_gpio_trigger_active(void)
{
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << CONFIG_CAL_TRIGGER_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io_cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "cal trigger gpio init failed (gpio=%d, err=%s) -> ignored",
                 CONFIG_CAL_TRIGGER_GPIO, esp_err_to_name(err));
        return false;
    }

    /* Active-low: return true when pin is driven LOW */
    return (gpio_get_level(CONFIG_CAL_TRIGGER_GPIO) == 0);
}

/* ─── calibration_run() ─────────────────────────────────────────────────── */

void calibration_run(void)
{
    /* T026: Motion guard — abort if rover is not stationary */
    for (int i = 0; i < 4; i++) {
        if (fabsf(g_encoder_velocities[i]) > 0.5f) {
            ESP_LOGE(TAG, "CAL: rover not stationary (wheel %d: %.2f rad/s) — calibration aborted",
                     i, (double)g_encoder_velocities[i]);
            return;
        }
    }

    ESP_LOGI(TAG, "CAL: starting calibration sequence");
    motor_stop_all();

    /* ── Direction calibration pass (T010) ── */

    /* Reset all dir_sign to +1 before measuring, so g_encoder_velocities[]
     * reflects the raw PCNT direction without any previously-stored sign
     * correction applied. Without this, a re-calibration with a stored
     * sign of -1 would flip the detected direction and write the wrong sign. */
    for (int ch = 0; ch < 4; ch++) {
        g_cal_params.dir_sign[ch] = +1;
    }

    uint32_t duty_low = (uint32_t)(CONFIG_CAL_DUTY_LOW_PCT * PWM_MAX_TICKS / 100);

    int8_t  detected_sign[4];
    bool    dir_valid[4];

    for (int ch = 0; ch < 4; ch++) {
        /* Drive wheel forward (LPWM) at low duty — forward per motor.h convention */
        motor_set_pwm((MotorChannel)ch, 0, duty_low);
        vTaskDelay(pdMS_TO_TICKS(350));

        float vel = g_encoder_velocities[ch];
        motor_stop_all();
        vTaskDelay(pdMS_TO_TICKS(150));   /* coast-down */

        /* T012: Zero-count guard */
        if (vel == 0.0f) {
            ESP_LOGW(TAG, "CAL: wheel %d zero count — skipping, default +1", ch);
            detected_sign[ch] = +1;
            dir_valid[ch]     = false;
        } else {
            detected_sign[ch] = (vel >= 0.0f) ? +1 : -1;
            dir_valid[ch]     = true;
        }

        ESP_LOGI(TAG, "CAL: wheel %d direction %+d (vel=%.3f rad/s, valid=%s)",
                 ch, (int)detected_sign[ch], (double)vel,
                 dir_valid[ch] ? "yes" : "no");
    }

    /* T011: Write direction signs to NVS */
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CAL: nvs_open failed for direction write: %s",
                 esp_err_to_name(err));
        goto update_globals_dir;
    }

    for (int ch = 0; ch < 4; ch++) {
        err = nvs_set_i8(h, k_dir_keys[ch], detected_sign[ch]);
        if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "CAL: nvs_set_i8 wheel %d: %s", ch, esp_err_to_name(err));
        }
    }
    err = nvs_commit(h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "CAL: nvs_commit (direction) failed: %s", esp_err_to_name(err));
    }
    nvs_close(h);

update_globals_dir:
    /* Update live g_cal_params regardless of NVS result */
    for (int ch = 0; ch < 4; ch++) {
        g_cal_params.dir_sign[ch] = detected_sign[ch];
    }
    g_cal_params.calibrated = true;

    /* ── Speed calibration pass (T014) ── */

    uint32_t duty_high = (uint32_t)(CONFIG_CAL_DUTY_HIGH_PCT * PWM_MAX_TICKS / 100);
    float commanded_low  = (CONFIG_CAL_DUTY_LOW_PCT  / 100.0f) * MAX_SPEED_RAD_S;
    float commanded_high = (CONFIG_CAL_DUTY_HIGH_PCT / 100.0f) * MAX_SPEED_RAD_S;

    float computed_scales[4];

    for (int ch = 0; ch < 4; ch++) {
        /* (a) LOW duty burst */
        motor_set_pwm((MotorChannel)ch, 0, duty_low);
        vTaskDelay(pdMS_TO_TICKS(350));
        float measured_low = fabsf(g_encoder_velocities[ch]);
        motor_stop_all();
        vTaskDelay(pdMS_TO_TICKS(150));   /* coast-down between LOW and HIGH */

        /* (b) HIGH duty burst */
        motor_set_pwm((MotorChannel)ch, 0, duty_high);
        vTaskDelay(pdMS_TO_TICKS(350));
        float measured_high = fabsf(g_encoder_velocities[ch]);
        motor_stop_all();
        vTaskDelay(pdMS_TO_TICKS(150));   /* inter-wheel coast-down */

        /* Compute raw scale — guard against stall (zero measured speed) */
        float raw_scale = 1.0f;
        if (measured_low < 0.1f && measured_high < 0.1f) {
            /* Both measurements zero — stall; default to 0.5 with warning */
            ESP_LOGW(TAG, "CAL: wheel %d stall detected (both zero) — clamping scale to 0.5", ch);
            raw_scale = 0.5f;
        } else if (measured_low < 0.1f) {
            /* Only high valid */
            raw_scale = commanded_high / measured_high;
        } else if (measured_high < 0.1f) {
            /* Only low valid */
            raw_scale = commanded_low / measured_low;
        } else {
            raw_scale = (commanded_low / measured_low + commanded_high / measured_high) / 2.0f;
        }

        /* T015: Clamp to [0.5, 2.0] */
        float clamped_scale = fmaxf(0.5f, fminf(2.0f, raw_scale));
        if (clamped_scale != raw_scale) {
            ESP_LOGW(TAG, "CAL: wheel %d scale clamped %.2f\u2192%.2f \u2014 check mechanism",
                     ch, (double)raw_scale, (double)clamped_scale);
        }

        computed_scales[ch] = clamped_scale;

        ESP_LOGI(TAG, "CAL: wheel %d speed low=%.2f high=%.2f scale=%.3f",
                 ch, (double)measured_low, (double)measured_high, (double)clamped_scale);
    }

    /* T015: Write speed scales to NVS */
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CAL: nvs_open failed for speed write: %s", esp_err_to_name(err));
        goto update_globals_spd;
    }

    for (int ch = 0; ch < 4; ch++) {
        uint32_t bits = 0;
        memcpy(&bits, &computed_scales[ch], sizeof(float));
        err = nvs_set_u32(h, k_spd_keys[ch], bits);
        if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "CAL: nvs_set_u32 speed wheel %d: %s", ch, esp_err_to_name(err));
        }
    }
    err = nvs_commit(h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "CAL: nvs_commit (speed) failed: %s", esp_err_to_name(err));
    }
    nvs_close(h);

update_globals_spd:
    for (int ch = 0; ch < 4; ch++) {
        g_cal_params.speed_scale[ch] = computed_scales[ch];
    }

    ESP_LOGI(TAG, "CAL: calibration complete — rover ready");
}

/* ─── calibration_reset_callback() ─────────────────────────────────────────*/

void calibration_reset_callback(const void *req, void *res)
{
    (void)req;
    (void)res;

    ESP_LOGI(TAG, "calibration_reset: erasing all calibration NVS keys");

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "calibration_reset: nvs_open failed: %s", esp_err_to_name(err));
        return;
    }

    /* Erase all 8 calibration keys; ESP_ERR_NVS_NOT_FOUND is treated as success */
    for (int i = 0; i < 4; i++) {
        err = nvs_erase_key(h, k_dir_keys[i]);
        if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "erase %s: %s", k_dir_keys[i], esp_err_to_name(err));
        }
        err = nvs_erase_key(h, k_spd_keys[i]);
        if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "erase %s: %s", k_spd_keys[i], esp_err_to_name(err));
        }
    }

    err = nvs_commit(h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "calibration_reset: nvs_commit failed: %s", esp_err_to_name(err));
    }
    nvs_close(h);

    /* Reset live g_cal_params to safe defaults */
    for (int i = 0; i < 4; i++) {
        g_cal_params.dir_sign[i]    = +1;
        g_cal_params.speed_scale[i] = 1.0f;
    }
    g_cal_params.calibrated = false;

    ESP_LOGI(TAG, "calibration_reset: complete — defaults restored (all +1, scale 1.0)");
}
