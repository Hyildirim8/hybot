/**
 * pid_controller.c — Discrete PID velocity controller (010-pid-closed-loop)
 *
 * Simple PID with anti-windup and output clamping.
 * Runs in the micro-ROS executor task context (not ISR).
 */

#include "pid_controller.h"

#include <math.h>

/* ─── Shared globals ────────────────────────────────────────────────────── */

volatile float g_pid_errors[4]  = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float g_pid_outputs[4] = {0.0f, 0.0f, 0.0f, 0.0f};

/* ─── API ───────────────────────────────────────────────────────────────── */

void pid_init(PidState *pid, float kp, float ki, float kd,
              float output_min, float output_max, float integral_max)
{
    pid->kp           = kp;
    pid->ki           = ki;
    pid->kd           = kd;
    pid->integral     = 0.0f;
    pid->prev_error   = 0.0f;
    pid->output_min   = output_min;
    pid->output_max   = output_max;
    pid->integral_max = integral_max;
}

float pid_update(PidState *pid, float setpoint, float measurement, float dt_s)
{
    /* Guard against degenerate dt (first call, timer glitch) */
    if (dt_s <= 0.0f) {
        dt_s = 0.001f;   /* fallback: 1 ms */
    }

    float error = setpoint - measurement;

    /* ── Proportional ── */
    float p_term = pid->kp * error;

    /* ── Integral with anti-windup ── */
    pid->integral += error * dt_s;
    if (pid->integral >  pid->integral_max) pid->integral =  pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    float i_term = pid->ki * pid->integral;

    /* ── Derivative (on error) ── */
    float d_term = pid->kd * (error - pid->prev_error) / dt_s;
    pid->prev_error = error;

    /* ── Sum and clamp ── */
    float output = p_term + i_term + d_term;
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    return output;
}

void pid_reset(PidState *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}
