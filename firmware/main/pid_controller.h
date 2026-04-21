#pragma once
/**
 * pid_controller.h — Per-wheel PID velocity controller (010-pid-closed-loop)
 *
 * Provides a discrete PID controller that closes the loop between commanded
 * wheel velocity (rad/s from ROS2) and measured wheel velocity (rad/s from
 * encoder PCNT hardware).
 *
 * Features:
 *   - Anti-windup: integral term clamped to ±integral_max
 *   - Output clamping: result clamped to [output_min, output_max]
 *   - Derivative-on-error (not derivative-on-measurement)
 *   - Reset function for zero-crossing / mode changes
 *
 * Usage:
 *   PidState pid;
 *   pid_init(&pid, kp, ki, kd, out_min, out_max, integral_max);
 *   ...
 *   float duty = pid_update(&pid, setpoint, measurement, dt_s);
 */

#include <stdbool.h>

/* ─── Kconfig defaults (÷1000 scaling for float representation) ──────── */

#ifndef CONFIG_PID_KP
#define CONFIG_PID_KP   50    /* ÷1000 → 0.050 */
#endif
#ifndef CONFIG_PID_KI
#define CONFIG_PID_KI   100   /* ÷1000 → 0.100 */
#endif
#ifndef CONFIG_PID_KD
#define CONFIG_PID_KD   5     /* ÷1000 → 0.005 */
#endif
#ifndef CONFIG_PID_INTEGRAL_MAX
#define CONFIG_PID_INTEGRAL_MAX   500   /* ÷1000 → 0.500 */
#endif

/* ─── PidState ──────────────────────────────────────────────────────────── */

typedef struct {
    float kp;             /**< Proportional gain */
    float ki;             /**< Integral gain */
    float kd;             /**< Derivative gain */
    float integral;       /**< Accumulated integral term */
    float prev_error;     /**< Previous error for derivative computation */
    float output_min;     /**< Minimum output clamp (e.g. -PWM_MAX_TICKS) */
    float output_max;     /**< Maximum output clamp (e.g. +PWM_MAX_TICKS) */
    float integral_max;   /**< Anti-windup: max |integral| accumulation */
} PidState;

/* ─── Globals ───────────────────────────────────────────────────────────── */

/** Per-wheel PID error (commanded - measured), readable by status_reporter */
extern volatile float g_pid_errors[4];

/** Per-wheel PID output (duty ticks as float), readable by status_reporter */
extern volatile float g_pid_outputs[4];

/* ─── API ───────────────────────────────────────────────────────────────── */

/**
 * pid_init() — Initialise a PID controller with gains and limits.
 *
 * @param pid           PID state to initialise
 * @param kp            Proportional gain
 * @param ki            Integral gain
 * @param kd            Derivative gain
 * @param output_min    Minimum output (typically -PWM_MAX_TICKS as float)
 * @param output_max    Maximum output (typically +PWM_MAX_TICKS as float)
 * @param integral_max  Anti-windup clamp for |integral| accumulation
 */
void pid_init(PidState *pid, float kp, float ki, float kd,
              float output_min, float output_max, float integral_max);

/**
 * pid_update() — Compute one PID iteration.
 *
 * @param pid          PID state (updated in place)
 * @param setpoint     Desired value (commanded speed in rad/s)
 * @param measurement  Current value (encoder-measured speed in rad/s)
 * @param dt_s         Time step in seconds since last call
 * @return             Control output, clamped to [output_min, output_max]
 */
float pid_update(PidState *pid, float setpoint, float measurement, float dt_s);

/**
 * pid_reset() — Zero the integral and previous error.
 *
 * Call when transitioning to/from zero speed to prevent integral windup
 * from causing creep or overshoot on direction changes.
 */
void pid_reset(PidState *pid);
