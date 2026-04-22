/**
 * motor.c — BTS7960B motor driver implementation using ESP-IDF LEDC
 *
 * See motor.h for the full API contract and direction conventions.
 */

#include "motor.h"

#include <math.h>
#include <string.h>

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "motor";

/* ─── LEDC channel mapping ──────────────────────────────────────────────── */
/* Two LEDC channels per motor (RPWM + LPWM), 8 channels total.
 * Channel order is stable across calls; indexed by MotorChannel × 2.        */

typedef struct {
    gpio_num_t rpwm_gpio;
    gpio_num_t lpwm_gpio;
    ledc_channel_t rpwm_ch;
    ledc_channel_t lpwm_ch;
} MotorCfg;

static const MotorCfg s_motors[MOTOR_COUNT] = {
    /* FL */ { CONFIG_MOTOR_FL_RPWM_GPIO, CONFIG_MOTOR_FL_LPWM_GPIO,
               LEDC_CHANNEL_0, LEDC_CHANNEL_1 },
    /* FR */ { CONFIG_MOTOR_FR_RPWM_GPIO, CONFIG_MOTOR_FR_LPWM_GPIO,
               LEDC_CHANNEL_2, LEDC_CHANNEL_3 },
    /* RL */ { CONFIG_MOTOR_RL_RPWM_GPIO, CONFIG_MOTOR_RL_LPWM_GPIO,
               LEDC_CHANNEL_4, LEDC_CHANNEL_5 },
    /* RR */ { CONFIG_MOTOR_RR_RPWM_GPIO, CONFIG_MOTOR_RR_LPWM_GPIO,
               LEDC_CHANNEL_6, LEDC_CHANNEL_7 },
};

/* ─── Public API ────────────────────────────────────────────────────────── */

void motor_init(void)
{
    /* Configure a single LEDC timer shared by all 8 channels */
    ledc_timer_config_t timer_cfg = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_10_BIT,   /* 0 … 1023 ticks */
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 1000,                 /* 1 kHz PWM */
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    for (int i = 0; i < MOTOR_COUNT; i++) {
        const MotorCfg *m = &s_motors[i];

        ledc_channel_config_t rpwm = {
            .gpio_num   = m->rpwm_gpio,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = m->rpwm_ch,
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0,
            .hpoint     = 0,
            .intr_type  = LEDC_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&rpwm));

        ledc_channel_config_t lpwm = {
            .gpio_num   = m->lpwm_gpio,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = m->lpwm_ch,
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0,
            .hpoint     = 0,
            .intr_type  = LEDC_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&lpwm));
    }

    ESP_LOGI(TAG, "motor_init: all 8 LEDC channels configured at 0%% duty");
}

void motor_set_pwm(MotorChannel channel, uint32_t rpwm_duty, uint32_t lpwm_duty)
{
    if (channel >= MOTOR_COUNT) {
        ESP_LOGE(TAG, "motor_set_pwm: invalid channel %d", channel);
        return;
    }

    /* Shoot-through guard: simultaneous non-zero RPWM+LPWM is a firmware fault */
    if (rpwm_duty > 0 && lpwm_duty > 0) {
        ESP_LOGE(TAG, "motor_set_pwm: shoot-through on ch%d — forcing stop", channel);
        rpwm_duty = 0;
        lpwm_duty = 0;
    }

    /* Clamp to hardware range */
    if (rpwm_duty > PWM_MAX_TICKS) rpwm_duty = PWM_MAX_TICKS;
    if (lpwm_duty > PWM_MAX_TICKS) lpwm_duty = PWM_MAX_TICKS;

    const MotorCfg *m = &s_motors[channel];

    /* Safe order: zero the outgoing channel before driving the incoming one.
     * This guarantees the dead time even if both are being changed.          */
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, m->rpwm_ch, rpwm_duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, m->rpwm_ch));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, m->lpwm_ch, lpwm_duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, m->lpwm_ch));
}

void motor_stop_all(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_set_pwm((MotorChannel)i, 0, 0);
    }
    ESP_LOGI(TAG, "motor_stop_all: all wheels stopped");
}

uint32_t speed_to_duty(float speed_rad_s)
{
    float magnitude = fabsf(speed_rad_s);
    /* clamp magnitude before conversion */
    if (magnitude > MAX_SPEED_RAD_S) magnitude = MAX_SPEED_RAD_S;

    /* duty = round_half_up(magnitude / MAX_SPEED_RAD_S × PWM_MAX_TICKS) */
    float duty_f = (magnitude / MAX_SPEED_RAD_S) * (float)PWM_MAX_TICKS;
    uint32_t duty = (uint32_t)(duty_f + 0.5f);   /* round-half-up */

    if (duty > PWM_MAX_TICKS) duty = PWM_MAX_TICKS;
    return duty;
}

float clamp_speed(float speed)
{
    if (speed > MAX_SPEED_RAD_S)  return  MAX_SPEED_RAD_S;
    if (speed < -MAX_SPEED_RAD_S) return -MAX_SPEED_RAD_S;
    return speed;
}
