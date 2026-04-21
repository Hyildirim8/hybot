#pragma once
/**
 * encoder.h — Quadrature encoder velocity feedback (008-encoder-feedback)
 *
 * Manages 4× quadrature encoders (one per GB37-520 wheel) using ESP32-S3
 * PCNT hardware units with full 4X decoding.
 *
 * Velocity sampling is performed in task context by encoder_sample_now()
 * at CONFIG_ENCODER_SAMPLE_PERIOD_MS cadence (called from wheel_publisher
 * timer callback), avoiding ISR-time driver calls.
 *
 * Each PCNT unit uses two channels for 4X quadrature:
 *   channel A: edge on pin_a, level on pin_b  (detects A transitions)
 *   channel B: edge on pin_b, level on pin_a  (detects B transitions)
 *
 * Direction convention:
 *   positive g_encoder_velocities[i]  → forward (matches motor FR-002)
 *   negative g_encoder_velocities[i]  → reverse
 *   zero                              → stationary or below noise floor
 *
 * Wheel order: FL=0, FR=1, RL=2, RR=3  (matches MotorChannel enum)
 *
 * Concurrency safety: g_encoder_velocities[], g_encoder_counts[], g_encoder_faults[]
 * are written by encoder_sample_now() and read by other FreeRTOS tasks.
 * Single 32-bit aligned read/write on LX7 is atomic; no mutex required.
 */

#include <stdbool.h>
#include <stdint.h>

/* ─── Compile-time Kconfig defaults (ifdef guards match sdkconfig symbols) ─ */

#ifndef CONFIG_ENCODER_CPR
#define CONFIG_ENCODER_CPR              1980    /* 11 lines × 4 × 45 gear */
#endif
#ifndef CONFIG_ENCODER_NOISE_FLOOR
#define CONFIG_ENCODER_NOISE_FLOOR      2       /* ticks below → zero velocity */
#endif
#ifndef CONFIG_ENCODER_SAMPLE_PERIOD_MS
#define CONFIG_ENCODER_SAMPLE_PERIOD_MS 20      /* 50 Hz sampling */
#endif
#ifndef CONFIG_ENCODER_GLITCH_NS
#define CONFIG_ENCODER_GLITCH_NS        1000    /* suppress sub-1µs noise */
#endif

/* GPIO pin defaults (FL, FR, RL, RR) */
#ifndef CONFIG_ENCODER_FL_PIN_A
#define CONFIG_ENCODER_FL_PIN_A   8
#endif
#ifndef CONFIG_ENCODER_FL_PIN_B
#define CONFIG_ENCODER_FL_PIN_B   9
#endif
#ifndef CONFIG_ENCODER_FR_PIN_A
#define CONFIG_ENCODER_FR_PIN_A   10
#endif
#ifndef CONFIG_ENCODER_FR_PIN_B
#define CONFIG_ENCODER_FR_PIN_B   11
#endif
#ifndef CONFIG_ENCODER_RL_PIN_A
#define CONFIG_ENCODER_RL_PIN_A   12
#endif
#ifndef CONFIG_ENCODER_RL_PIN_B
#define CONFIG_ENCODER_RL_PIN_B   13
#endif
#ifndef CONFIG_ENCODER_RR_PIN_A
#define CONFIG_ENCODER_RR_PIN_A   14
#endif
#ifndef CONFIG_ENCODER_RR_PIN_B
#define CONFIG_ENCODER_RR_PIN_B   21
#endif

/* ─── Shared globals (ISR-safe, aligned 32-bit) ─────────────────────────── */

/** Last computed wheel velocities in rad/s. Written by encoder_sample_now() every
 *  CONFIG_ENCODER_SAMPLE_PERIOD_MS ms. Read by wheel_publisher, status_reporter,
 *  and calibration_run(). Index = MotorChannel (FL=0, FR=1, RL=2, RR=3). */
extern volatile float   g_encoder_velocities[4];

/** Cumulative signed tick count per wheel. Written by encoder_sample_now(), read by status.
 *  Does NOT reset on micro-ROS disconnect — persists across sessions. */
extern volatile int32_t g_encoder_counts[4];

/** Last raw delta tick count measured per sample period (pre-noise-floor).
 * Useful for wiring diagnostics when velocities stay at zero. */
extern volatile int32_t g_encoder_last_delta[4];

/** Monotonic sequence incremented on every velocity sample.
 * If this value is not changing, encoder_sample_now() is not being called. */
extern volatile uint32_t g_encoder_sample_seq;

/** Per-wheel fault flag: set when |Δcount| exceeds MAX_SPEED_RAD_S equivalent.
 *  Indicates implausibly high tick rate (e.g., noise/EMI), NOT open-pin.   */
extern volatile bool    g_encoder_faults[4];

/* ─── API ────────────────────────────────────────────────────────────────── */

/**
 * encoder_init() — Allocate and configure PCNT units.
 *
 * Configures 4 PCNT units with 4X quadrature decoding, glitch filters,
 * ±32767 watchpoints, and clears all counters.
 *
 * MUST be called before encoder_start() and before the micro-ROS retry loop.
 * Safe to call even if NVS calibration has not yet loaded.
 */
void encoder_init(void);

/**
 * encoder_start() — Enable and start all 4 PCNT units.
 *
 * Call after encoder_init(). PCNT units begin accumulating counts.
 *
 * Separating init from start allows the boot sequence to complete NVS load
 * and calibration before the first velocity sample is taken.
 */
void encoder_start(void);

/**
 * encoder_sample_now() — Sample PCNT counters and update velocity globals.
 *
 * Must be called periodically at approximately CONFIG_ENCODER_SAMPLE_PERIOD_MS.
 * This function is intended to run in task context (not ISR).
 */
void encoder_sample_now(void);
