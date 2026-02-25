#pragma once
/**
 * motor.h — BTS7960B motor driver abstraction
 *
 * Manages 4× BTS7960B H-bridge drivers (one per GB37-520 wheel).
 * Each driver is controlled by two LEDC PWM channels: RPWM (reverse) and
 * LPWM (forward). The EN (enable) pin is held HIGH after init.
 *
 * Shoot-through guard: motor_set_pwm() ALWAYS drives one channel to 0
 * before driving the other to a non-zero value, preventing simultaneous
 * RPWM+LPWM, which would short the BTS7960B output stage.
 *
 * Direction convention (matches FR-002, FR-003):
 *   speed > 0  → LPWM driven, RPWM = 0  (forward)
 *   speed < 0  → RPWM driven, LPWM = 0  (reverse)
 *   speed == 0 → both = 0               (stop / coast)
 *
 * Wheel order: FL=0, FR=1, RL=2, RR=3  (matches /wheel_velocities array index)
 */

#include <stdint.h>

/* ─── PWM parameters ────────────────────────────────────────────────────── */

/** LEDC timer resolution: 10-bit → 1024 ticks, ~1 kHz at 80 MHz APB clock */
#define PWM_MAX_TICKS       1023u

/** Maximum angular speed in rad/s — used for duty-cycle mapping (FR-009).
 *  Matched to max_linear_speed/wheel_radius: 1.5 m/s ÷ 0.05 m = 30 rad/s.
 *  Override via NVS key "max_speed_rads" at provisioning time.             */
#define MAX_SPEED_RAD_S     30.0f

/* ─── GPIO pin assignments ───────────────────────────────────────────────── */
/* Configurable at compile time via Kconfig (firmware/main/Kconfig.projbuild).
 * Defaults below are the physical wiring layout on the rover PCB.
 * RPWM = reverse PWM channel, LPWM = forward PWM channel.                  */

#ifndef CONFIG_MOTOR_FL_RPWM_GPIO
#define CONFIG_MOTOR_FL_RPWM_GPIO   4
#endif
#ifndef CONFIG_MOTOR_FL_LPWM_GPIO
#define CONFIG_MOTOR_FL_LPWM_GPIO   5
#endif
#ifndef CONFIG_MOTOR_FR_RPWM_GPIO
#define CONFIG_MOTOR_FR_RPWM_GPIO   6
#endif
#ifndef CONFIG_MOTOR_FR_LPWM_GPIO
#define CONFIG_MOTOR_FR_LPWM_GPIO   7
#endif
#ifndef CONFIG_MOTOR_RL_RPWM_GPIO
#define CONFIG_MOTOR_RL_RPWM_GPIO   15
#endif
#ifndef CONFIG_MOTOR_RL_LPWM_GPIO
#define CONFIG_MOTOR_RL_LPWM_GPIO   16
#endif
#ifndef CONFIG_MOTOR_RR_RPWM_GPIO
#define CONFIG_MOTOR_RR_RPWM_GPIO   17
#endif
#ifndef CONFIG_MOTOR_RR_LPWM_GPIO
#define CONFIG_MOTOR_RR_LPWM_GPIO   18
#endif

/* ─── Motor channel enumeration ─────────────────────────────────────────── */
typedef enum {
    MOTOR_FL = 0,   /**< Front-Left  — /wheel_velocities index 0 */
    MOTOR_FR = 1,   /**< Front-Right — /wheel_velocities index 1 */
    MOTOR_RL = 2,   /**< Rear-Left   — /wheel_velocities index 2 */
    MOTOR_RR = 3,   /**< Rear-Right  — /wheel_velocities index 3 */
    MOTOR_COUNT = 4
} MotorChannel;

/* ─── API ────────────────────────────────────────────────────────────────── */

/**
 * motor_init() — Configure LEDC PWM and GPIO for all 4 motor channels.
 *
 * MUST be called before any other motor function.
 * Drives all RPWM and LPWM pins LOW (0% duty) on return — safe state.
 * EN pins are driven HIGH to enable the BTS7960B output stages.
 */
void motor_init(void);

/**
 * motor_set_pwm() — Drive one motor channel with shoot-through guard.
 *
 * @param channel    Motor to drive (MOTOR_FL … MOTOR_RR)
 * @param rpwm_duty  Reverse PWM duty cycle [0, PWM_MAX_TICKS]
 * @param lpwm_duty  Forward PWM duty cycle [0, PWM_MAX_TICKS]
 *
 * Shoot-through guard: if both rpwm_duty and lpwm_duty are non-zero,
 * this function asserts and drives both to 0 (firmware fault, FR-002 note).
 */
void motor_set_pwm(MotorChannel channel, uint32_t rpwm_duty, uint32_t lpwm_duty);

/**
 * motor_stop_all() — Drive all 8 PWM channels to 0% immediately.
 *
 * Called by the watchdog expiry callback, session loss handler, and on boot.
 * Safe to call from an ISR context (LEDC duty update is non-blocking).
 */
void motor_stop_all(void);

/**
 * speed_to_duty() — Map rad/s magnitude to LEDC duty cycle ticks.
 *
 * Formula: duty = round_half_up(|speed| / MAX_SPEED_RAD_S × PWM_MAX_TICKS)
 * Result is clamped to [0, PWM_MAX_TICKS].
 *
 * @param speed_rad_s   Signed speed (sign ignored; use magnitude only)
 * @return              Duty cycle in ticks [0, PWM_MAX_TICKS]
 */
uint32_t speed_to_duty(float speed_rad_s);

/**
 * clamp_speed() — Enforce [-MAX_SPEED_RAD_S, +MAX_SPEED_RAD_S] range.
 *
 * @param speed   Input speed in rad/s (any value)
 * @return        Clamped speed in rad/s
 */
float clamp_speed(float speed);
