/*
 * SPDX-FileCopyrightText: 2026 GMaster contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef GZRL_CONTROL_H_
#define GZRL_CONTROL_H_

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @file
 * @brief Allocation-free control primitives with explicit time inputs.
 *
 * Objects have one owner. Public fields are available for static allocation;
 * modify them only through these functions. All step functions leave the object
 * and output unchanged on error. No function reads a clock or accesses hardware.
 */

/** PID recurrence. Both modes use time-based, rather than per-call, gains. */
enum gzrl_pid_mode {
	GZRL_PID_POSITION = 0,
	GZRL_PID_INCREMENTAL,
};

/** Differentiating the measurement avoids derivative kick on a target change. */
enum gzrl_pid_derivative {
	GZRL_PID_D_MEASUREMENT = 0,
	GZRL_PID_D_ERROR,
};

/** PID parameters. Limits and accepted dt bounds are inclusive. */
struct gzrl_pid_config {
	enum gzrl_pid_mode mode;
	enum gzrl_pid_derivative derivative;
	float kp;
	float ki;
	float kd;
	float output_min;
	float output_max;
	float integral_min; /**< Limits apply to the integral contribution, in output units. */
	float integral_max;
	float dt_min; /**< Smallest accepted positive time step, in seconds. */
	float dt_max; /**< Largest accepted time step, in seconds. */
};

/** PID diagnostics. Saturation and conditional integration can make output differ from p + i + d.
 */
struct gzrl_pid_state {
	float output;
	float p;
	float i;
	float d;
	float previous_error;
	float previous_measurement;
	bool initialized;
};

/** Caller-owned PID object. */
struct gzrl_pid {
	struct gzrl_pid_config config;
	struct gzrl_pid_state state;
	bool configured;
};

/**
 * @brief Initialize or reconfigure a PID and reset its history.
 *
 * Gains must be finite and nonnegative. Output and integral intervals must
 * contain zero. This avoids applying a nonzero command during a reset. Set
 * integral_min = integral_max = 0 to disable integral accumulation.
 *
 * @retval 0 Success.
 * @retval -EINVAL Invalid pointer, enumeration, gain, limit, or dt interval.
 */
int gzrl_pid_init(struct gzrl_pid *pid, const struct gzrl_pid_config *config);

/** @brief Clear PID history and output; the next derivative contribution is zero. */
int gzrl_pid_reset(struct gzrl_pid *pid);

/**
 * @brief Update a PID using explicit dt in seconds.
 *
 * Uses backward difference differentiation and rectangular integration.
 * Conditional integration prevents the integral from driving output further
 * into saturation, while still allowing it to unwind. The first derivative is
 * zero. Incremental mode includes changes in P and D and the accepted change
 * in I, accumulating into the previous saturated output.
 *
 * @retval 0 Success; output is within configured limits.
 * @retval -EINVAL Unconfigured object or invalid pointer.
 * @retval -ERANGE Nonfinite input, unacceptable dt, or unrepresentable calculation.
 */
int gzrl_pid_update(struct gzrl_pid *pid, float setpoint, float measurement, float dt,
		    float *output);

/** @brief Copy the current parameters without changing state. */
int gzrl_pid_get_config(const struct gzrl_pid *pid, struct gzrl_pid_config *config);

/** @brief Copy the current diagnostics without changing state. */
int gzrl_pid_get_state(const struct gzrl_pid *pid, struct gzrl_pid_state *state);

/** Two independent PIDs; outer output is held between explicitly scheduled updates. */
struct gzrl_cascade {
	struct gzrl_pid outer;
	struct gzrl_pid inner;
	float outer_output;
	bool outer_ready;
};

/** @brief Initialize a cascade; both loops are configured and reset atomically. */
int gzrl_cascade_init(struct gzrl_cascade *cascade, const struct gzrl_pid_config *outer,
		      const struct gzrl_pid_config *inner);

/** @brief Reset both loops, including the held outer-loop reference. */
int gzrl_cascade_reset(struct gzrl_cascade *cascade);

/**
 * @brief Update independently scheduled outer and inner loops.
 *
 * inner_dt is the elapsed time since the previous inner update. If outer_due
 * is true, outer_dt is the elapsed time since the previous outer update. If
 * false, setpoint, outer_measurement and outer_dt are ignored. Each accepted
 * call advances the inner loop. A failure in either loop changes neither.
 *
 * @retval -EAGAIN Outer loop has not run yet and outer_due is false.
 * @return Otherwise the same status codes as gzrl_pid_update().
 */
int gzrl_cascade_update(struct gzrl_cascade *cascade, float setpoint, float outer_measurement,
			float inner_measurement, bool outer_due, float outer_dt, float inner_dt,
			float *output);

/** First-order low-pass state. First sample after init/reset seeds the output. */
struct gzrl_lowpass {
	float time_constant;
	float dt_min;
	float dt_max;
	float value;
	bool initialized;
	bool configured;
};

/** @brief Initialize a low-pass filter with positive time constant and dt bounds, in seconds. */
int gzrl_lowpass_init(struct gzrl_lowpass *filter, float time_constant, float dt_min, float dt_max);

/** @brief Reset low-pass history; retain its configuration. */
int gzrl_lowpass_reset(struct gzrl_lowpass *filter);

/** @brief Filter a sample using alpha = dt / (time_constant + dt). */
int gzrl_lowpass_update(struct gzrl_lowpass *filter, float input, float dt, float *output);

/** Moving-average state backed by caller-owned storage. */
struct gzrl_moving_average {
	float *storage;
	size_t capacity;
	size_t count;
	size_t next;
	double sum;
};

/** @brief Initialize with storage for capacity floats; storage must outlive the filter. */
int gzrl_moving_average_init(struct gzrl_moving_average *filter, float *storage, size_t capacity);

/** @brief Empty the moving-average window without reading or clearing its storage. */
int gzrl_moving_average_reset(struct gzrl_moving_average *filter);

/** @brief Average available samples until the window fills, then maintain a sliding window. */
int gzrl_moving_average_update(struct gzrl_moving_average *filter, float input, float *output);

/** Median state backed by two separate caller-owned arrays of capacity floats each. */
struct gzrl_median {
	float *storage;
	float *scratch;
	size_t capacity;
	size_t count;
	size_t next;
};

/**
 * @brief Initialize a sliding median with non-overlapping storage and scratch arrays.
 *
 * Uses insertion sort in caller-provided scratch: O(capacity squared) worst-case
 * work, no allocation. Choose a small fixed capacity appropriate to the control
 * period. Even window sizes return the mean of the two middle samples.
 */
int gzrl_median_init(struct gzrl_median *filter, float *storage, float *scratch, size_t capacity);

/** @brief Empty the median window without reading or clearing its arrays. */
int gzrl_median_reset(struct gzrl_median *filter);

/** @brief Compute the median of available samples or of the full sliding window. */
int gzrl_median_update(struct gzrl_median *filter, float input, float *output);

/**
 * @brief Compute the shortest signed angular error, in [-pi, pi), using radians.
 *
 * The antipodal tie is -pi. Rejects nonfinite inputs. With derivative-on-error,
 * a wrapped-angle loop can pass error as the PID setpoint and zero as its
 * measurement. Derivative-on-measurement instead requires a continuous,
 * unwrapped measurement and a target constructed as measurement + error.
 */
int gzrl_angle_error(float target, float measurement, float *error);

#ifdef __cplusplus
}
#endif

#endif /* GZRL_CONTROL_H_ */
