/*
 * SPDX-FileCopyrightText: 2026 GMaster contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <gzrl/control.h>

#include <errno.h>
#include <float.h>
#include <math.h>
#include <string.h>

#include <zephyr/ztest.h>

int control_cpp_probe(void);

static struct gzrl_pid_config config_for(enum gzrl_pid_mode mode)
{
	struct gzrl_pid_config config = {
		.mode = mode,
		.derivative = GZRL_PID_D_MEASUREMENT,
		.kp = 2.0f,
		.ki = 1.0f,
		.kd = 0.5f,
		.output_min = -100.0f,
		.output_max = 100.0f,
		.integral_min = -50.0f,
		.integral_max = 50.0f,
		.dt_min = 0.001f,
		.dt_max = 1.0f,
	};

	return config;
}

ZTEST(control, test_configuration_and_cpp)
{
	struct gzrl_pid_config config = config_for(GZRL_PID_POSITION);
	struct gzrl_pid pid;
	struct gzrl_pid_config copy;
	struct gzrl_pid_state state;

	zassert_ok(control_cpp_probe());
	zassert_ok(gzrl_pid_init(&pid, &config));
	zassert_ok(gzrl_pid_get_config(&pid, &copy));
	zassert_equal(copy.kp, config.kp);
	zassert_ok(gzrl_pid_get_state(&pid, &state));
	zassert_false(state.initialized);
	struct gzrl_pid before = pid;

	config.dt_min = 0.0f;
	zassert_equal(gzrl_pid_init(&pid, &config), -EINVAL);
	zassert_mem_equal(&pid, &before, sizeof(pid));
	config = config_for(GZRL_PID_POSITION);
	config.integral_min = 1.0f;
	zassert_equal(gzrl_pid_init(&pid, &config), -EINVAL);
	config = config_for(GZRL_PID_POSITION);
	config.kd = -1.0f;
	zassert_equal(gzrl_pid_init(&pid, &config), -EINVAL);
	config = config_for((enum gzrl_pid_mode)42);
	zassert_equal(gzrl_pid_init(&pid, &config), -EINVAL);
	config = config_for(GZRL_PID_POSITION);
	config.derivative = (enum gzrl_pid_derivative)42;
	zassert_equal(gzrl_pid_init(&pid, &config), -EINVAL);
	config = config_for(GZRL_PID_POSITION);
	config.output_max = NAN;
	zassert_equal(gzrl_pid_init(&pid, &config), -EINVAL);
}

ZTEST(control, test_pid_numeric_sequence)
{
	for (enum gzrl_pid_mode mode = GZRL_PID_POSITION; mode <= GZRL_PID_INCREMENTAL; ++mode) {
		struct gzrl_pid_config config = config_for(mode);
		struct gzrl_pid pid;
		float output;

		zassert_ok(gzrl_pid_init(&pid, &config));
		zassert_ok(gzrl_pid_update(&pid, 1.0f, 0.0f, 0.1f, &output));
		zassert_within(output, 2.1f, 0.00001f);
		zassert_equal(pid.state.d, 0.0f);
		zassert_ok(gzrl_pid_update(&pid, 1.0f, 0.25f, 0.1f, &output));
		/* P = 1.5; I = .175; D = -.5 * .25 / .1 = -1.25. */
		zassert_within(output, 0.425f, 0.00001f);
		zassert_ok(gzrl_pid_update(&pid, 1.0f, 0.5f, 0.2f, &output));
		/* P = 1; I = .275; D = -.625 despite the changed period. */
		zassert_within(output, 0.65f, 0.00001f);
	}
}

ZTEST(control, test_derivative_source_and_reset)
{
	struct gzrl_pid_config config = config_for(GZRL_PID_POSITION);
	struct gzrl_pid pid;
	float output;

	config.kp = 0.0f;
	config.ki = 0.0f;
	config.kd = 1.0f;
	zassert_ok(gzrl_pid_init(&pid, &config));
	zassert_ok(gzrl_pid_update(&pid, 0.0f, 0.0f, 0.1f, &output));
	zassert_ok(gzrl_pid_update(&pid, 5.0f, 0.0f, 0.1f, &output));
	zassert_equal(output, 0.0f);
	config.derivative = GZRL_PID_D_ERROR;
	zassert_ok(gzrl_pid_init(&pid, &config));
	zassert_ok(gzrl_pid_update(&pid, 0.0f, 0.0f, 0.1f, &output));
	zassert_ok(gzrl_pid_update(&pid, 5.0f, 0.0f, 0.1f, &output));
	zassert_within(output, 50.0f, 0.0001f);
	zassert_ok(gzrl_pid_reset(&pid));
	zassert_ok(gzrl_pid_update(&pid, 8.0f, -2.0f, 0.1f, &output));
	zassert_equal(output, 0.0f);
}

ZTEST(control, test_saturation_and_unwinding)
{
	for (enum gzrl_pid_mode mode = GZRL_PID_POSITION; mode <= GZRL_PID_INCREMENTAL; ++mode) {
		struct gzrl_pid_config config = config_for(mode);
		struct gzrl_pid pid;
		float output;

		config.kp = 0.0f;
		config.kd = 0.0f;
		config.output_min = -1.0f;
		config.output_max = 1.0f;
		zassert_ok(gzrl_pid_init(&pid, &config));
		zassert_ok(gzrl_pid_update(&pid, 0.5f, 0.0f, 1.0f, &output));
		zassert_equal(pid.state.i, 0.5f);
		for (int i = 0; i < 100; ++i) {
			zassert_ok(gzrl_pid_update(&pid, 5.0f, 0.0f, 1.0f, &output));
			zassert_equal(output, 1.0f);
			zassert_equal(pid.state.i, 0.5f);
		}
		zassert_ok(gzrl_pid_update(&pid, -0.25f, 0.0f, 1.0f, &output));
		zassert_equal(pid.state.i, 0.25f);
		zassert_true(output < 1.0f);
		zassert_ok(gzrl_pid_reset(&pid));
		/* Even a first pure-I step larger than the output range reaches the rail. */
		zassert_ok(gzrl_pid_update(&pid, -5.0f, 0.0f, 1.0f, &output));
		zassert_equal(output, -1.0f);
		zassert_equal(pid.state.i, 0.0f);
	}
}

ZTEST(control, test_integral_limits)
{
	struct gzrl_pid_config config = config_for(GZRL_PID_POSITION);
	struct gzrl_pid pid;
	float output;

	config.kp = 0.0f;
	config.kd = 0.0f;
	config.integral_min = -0.25f;
	config.integral_max = 0.25f;
	zassert_ok(gzrl_pid_init(&pid, &config));
	zassert_ok(gzrl_pid_update(&pid, 5.0f, 0.0f, 1.0f, &output));
	zassert_equal(output, 0.25f);
	zassert_ok(gzrl_pid_update(&pid, -5.0f, 0.0f, 1.0f, &output));
	zassert_equal(output, -0.25f);
}

ZTEST(control, test_invalid_step_is_transactional)
{
	struct gzrl_pid_config config = config_for(GZRL_PID_POSITION);
	struct gzrl_pid pid;
	float output;
	const float bad_dt[] = {0.0f, -1.0f, NAN, INFINITY, 0.0001f, 1.001f};

	zassert_ok(gzrl_pid_init(&pid, &config));
	zassert_ok(gzrl_pid_update(&pid, 1.0f, 0.0f, 0.1f, &output));
	struct gzrl_pid before = pid;

	for (size_t i = 0; i < ARRAY_SIZE(bad_dt); ++i) {
		output = 123.0f;
		zassert_equal(gzrl_pid_update(&pid, 1.0f, 0.0f, bad_dt[i], &output), -ERANGE);
		zassert_mem_equal(&pid, &before, sizeof(pid));
		zassert_equal(output, 123.0f);
	}
	zassert_equal(gzrl_pid_update(&pid, NAN, 0.0f, 0.1f, &output), -ERANGE);
	zassert_equal(gzrl_pid_update(&pid, 0.0f, INFINITY, 0.1f, &output), -ERANGE);
	zassert_equal(gzrl_pid_update(&pid, FLT_MAX, -FLT_MAX, 0.1f, &output), -ERANGE);
	zassert_mem_equal(&pid, &before, sizeof(pid));
	zassert_equal(output, 123.0f);
	zassert_ok(gzrl_pid_update(&pid, 0.0f, 0.0f, config.dt_min, &output));
	zassert_ok(gzrl_pid_update(&pid, 0.0f, 0.0f, config.dt_max, &output));
}

ZTEST(control, test_cascade_multirate_and_failure)
{
	struct gzrl_pid_config outer = config_for(GZRL_PID_POSITION);
	struct gzrl_pid_config inner = config_for(GZRL_PID_INCREMENTAL);
	struct gzrl_cascade cascade;
	float output = 1000.0f;

	outer.ki = 0.0f;
	outer.kd = 0.0f;
	inner.kp = 3.0f;
	inner.ki = 0.0f;
	inner.kd = 0.0f;
	zassert_ok(gzrl_cascade_init(&cascade, &outer, &inner));
	zassert_equal(gzrl_cascade_update(&cascade, 1.0f, 0.0f, 0.0f, false, 0.1f, 0.01f, &output),
		      -EAGAIN);
	zassert_equal(output, 1000.0f);
	zassert_ok(gzrl_cascade_update(&cascade, 1.0f, 0.0f, 0.0f, true, 0.1f, 0.01f, &output));
	zassert_equal(output, 6.0f);
	zassert_ok(gzrl_cascade_update(&cascade, NAN, NAN, 1.0f, false, NAN, 0.01f, &output));
	zassert_equal(output, 3.0f);
	struct gzrl_cascade before = cascade;

	output = 1000.0f;
	zassert_equal(gzrl_cascade_update(&cascade, 4.0f, 1.0f, 0.0f, true, 0.1f, 0.0f, &output),
		      -ERANGE);
	zassert_mem_equal(&cascade, &before, sizeof(cascade));
	zassert_equal(output, 1000.0f);
	zassert_ok(gzrl_cascade_reset(&cascade));
	zassert_false(cascade.outer_ready);
	zassert_equal(cascade.inner.state.output, 0.0f);
}

ZTEST(control, test_variable_period_plant_converges)
{
	for (enum gzrl_pid_mode mode = GZRL_PID_POSITION; mode <= GZRL_PID_INCREMENTAL; ++mode) {
		struct gzrl_pid_config config = config_for(mode);
		struct gzrl_pid pid;
		float speed = 0.0f;
		float output;

		config.kp = 1.0f;
		config.ki = 3.0f;
		config.kd = 0.0f;
		config.output_min = -2.0f;
		config.output_max = 2.0f;
		zassert_ok(gzrl_pid_init(&pid, &config));
		for (int step = 0; step < 1500; ++step) {
			float dt = step % 2 == 0 ? 0.005f : 0.015f;

			zassert_ok(gzrl_pid_update(&pid, 1.0f, speed, dt, &output));
			zassert_true(fabsf(output) <= 2.0f);
			/* Independent first-order plant: tau = .2 s, static gain = 1. */
			speed = output + (speed - output) * expf(-dt / 0.2f);
		}
		zassert_within(speed, 1.0f, 0.002f);
	}
}

ZTEST(control, test_lowpass_seeding_step_and_reset)
{
	struct gzrl_lowpass filter;
	float output;

	zassert_ok(gzrl_lowpass_init(&filter, 0.1f, 0.001f, 0.2f));
	zassert_ok(gzrl_lowpass_update(&filter, 0.0f, 0.1f, &output));
	zassert_ok(gzrl_lowpass_update(&filter, 1.0f, 0.1f, &output));
	zassert_within(output, 0.5f, 0.000001f);
	zassert_ok(gzrl_lowpass_update(&filter, 1.0f, 0.2f, &output));
	zassert_within(output, 5.0f / 6.0f, 0.000001f);
	struct gzrl_lowpass before = filter;

	zassert_equal(gzrl_lowpass_update(&filter, NAN, 0.1f, &output), -ERANGE);
	zassert_equal(gzrl_lowpass_update(&filter, 1.0f, 0.0f, &output), -ERANGE);
	zassert_mem_equal(&filter, &before, sizeof(filter));
	zassert_ok(gzrl_lowpass_reset(&filter));
	zassert_ok(gzrl_lowpass_update(&filter, 10.0f, 0.1f, &output));
	zassert_equal(output, 10.0f);
}

ZTEST(control, test_moving_average_window)
{
	float storage[3];
	struct gzrl_moving_average filter;
	float output;

	zassert_ok(gzrl_moving_average_init(&filter, storage, ARRAY_SIZE(storage)));
	zassert_ok(gzrl_moving_average_update(&filter, 1.0f, &output));
	zassert_equal(output, 1.0f);
	zassert_ok(gzrl_moving_average_update(&filter, 4.0f, &output));
	zassert_equal(output, 2.5f);
	zassert_ok(gzrl_moving_average_update(&filter, 7.0f, &output));
	zassert_equal(output, 4.0f);
	zassert_ok(gzrl_moving_average_update(&filter, 10.0f, &output));
	zassert_equal(output, 7.0f);
	struct gzrl_moving_average before = filter;
	float storage_before[3];

	memcpy(storage_before, storage, sizeof(storage));
	zassert_equal(gzrl_moving_average_update(&filter, INFINITY, &output), -ERANGE);
	zassert_mem_equal(&filter, &before, sizeof(filter));
	zassert_mem_equal(storage, storage_before, sizeof(storage));
	zassert_equal(output, 7.0f);
	zassert_ok(gzrl_moving_average_reset(&filter));
	zassert_ok(gzrl_moving_average_update(&filter, -2.0f, &output));
	zassert_equal(output, -2.0f);
}

ZTEST(control, test_median_impulses_even_and_overlap)
{
	float storage[3];
	float scratch[3];
	struct gzrl_median filter;
	float output;

	zassert_equal(gzrl_median_init(&filter, storage, &storage[1], 2), -EINVAL);
	zassert_ok(gzrl_median_init(&filter, storage, scratch, ARRAY_SIZE(storage)));
	zassert_ok(gzrl_median_update(&filter, 1.0f, &output));
	zassert_equal(output, 1.0f);
	zassert_ok(gzrl_median_update(&filter, 3.0f, &output));
	zassert_equal(output, 2.0f);
	zassert_ok(gzrl_median_update(&filter, 1000.0f, &output));
	zassert_equal(output, 3.0f);
	zassert_ok(gzrl_median_update(&filter, 2.0f, &output));
	zassert_equal(output, 3.0f);
	zassert_ok(gzrl_median_update(&filter, -1000.0f, &output));
	zassert_equal(output, 2.0f);
	struct gzrl_median before = filter;
	float storage_before[3];

	memcpy(storage_before, storage, sizeof(storage));
	zassert_equal(gzrl_median_update(&filter, NAN, &output), -ERANGE);
	zassert_mem_equal(&filter, &before, sizeof(filter));
	zassert_mem_equal(storage, storage_before, sizeof(storage));
	zassert_equal(output, 2.0f);
	zassert_ok(gzrl_median_reset(&filter));
	zassert_ok(gzrl_median_update(&filter, FLT_MAX, &output));
	zassert_ok(gzrl_median_update(&filter, FLT_MAX, &output));
	zassert_equal(output, FLT_MAX);
}

ZTEST(control, test_angle_wrap_and_external_feedback)
{
	const float pi = 3.14159265358979323846f;
	float error;
	float output;
	struct gzrl_pid_config config = config_for(GZRL_PID_POSITION);
	struct gzrl_pid pid;

	zassert_ok(gzrl_angle_error(0.1f, 2.0f * pi - 0.1f, &error));
	zassert_within(error, 0.2f, 0.00001f);
	config.ki = 0.0f;
	config.kd = 0.0f;
	zassert_ok(gzrl_pid_init(&pid, &config));
	zassert_ok(gzrl_pid_update(&pid, error, 0.0f, 0.1f, &output));
	zassert_within(output, 0.4f, 0.00001f);
	zassert_ok(gzrl_angle_error(2.0f * pi - 0.1f, 0.1f, &error));
	zassert_within(error, -0.2f, 0.00001f);
	zassert_ok(gzrl_angle_error(-4.0f * pi + 0.1f, 0.0f, &error));
	zassert_within(error, 0.1f, 0.00001f);
	error = 10.0f;
	zassert_equal(gzrl_angle_error(NAN, 0.0f, &error), -ERANGE);
	zassert_equal(error, 10.0f);
}

ZTEST_SUITE(control, NULL, NULL, NULL, NULL, NULL);
