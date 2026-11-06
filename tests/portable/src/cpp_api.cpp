/*
 * SPDX-FileCopyrightText: 2026 GMaster contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#include <gzrl/control.h>
#include <gzrl/motor/core.h>

#include <cmath>
#include <cstdio>

extern "C" int gzrl_portable_c_smoke(void);

static bool check_control_interfaces()
{
	gzrl_pid_config outer = {};
	gzrl_pid_config inner;
	gzrl_cascade cascade;
	float output;

	outer.kp = 2.0f;
	outer.output_min = -100.0f;
	outer.output_max = 100.0f;
	outer.dt_min = 0.001f;
	outer.dt_max = 0.1f;
	inner = outer;
	inner.kp = 1.0f;
	if (gzrl_cascade_init(&cascade, &outer, &inner) != 0 ||
	    gzrl_cascade_update(&cascade, 2.0f, 0.0f, 1.0f, true, 0.01f, 0.01f, &output) != 0 ||
	    output != 3.0f ||
	    gzrl_cascade_update(&cascade, 99.0f, 99.0f, 2.0f, false, 0.0f, 0.01f, &output) != 0 ||
	    output != 2.0f) {
		return false;
	}
	gzrl_lowpass lowpass;
	gzrl_moving_average average;
	gzrl_median median;
	float average_storage[3];
	float median_storage[3];
	float median_scratch[3];

	if (gzrl_lowpass_init(&lowpass, 1.0f, 0.1f, 1.0f) != 0 ||
	    gzrl_lowpass_update(&lowpass, 1.0f, 1.0f, &output) != 0 ||
	    gzrl_lowpass_update(&lowpass, 3.0f, 1.0f, &output) != 0 || output != 2.0f ||
	    gzrl_moving_average_init(&average, average_storage, 3) != 0 ||
	    gzrl_moving_average_update(&average, 1.0f, &output) != 0 ||
	    gzrl_moving_average_update(&average, 3.0f, &output) != 0 || output != 2.0f ||
	    gzrl_median_init(&median, median_storage, median_scratch, 3) != 0 ||
	    gzrl_median_update(&median, 1.0f, &output) != 0 ||
	    gzrl_median_update(&median, 9.0f, &output) != 0 ||
	    gzrl_median_update(&median, 2.0f, &output) != 0 || output != 2.0f) {
		return false;
	}
	return gzrl_angle_error(6.1831853f, 0.1f, &output) == 0 &&
	       std::fabs(output + 0.2f) < 0.00001f;
}

static bool check_motor_interfaces()
{
	for (int profile = 0; profile < GZRL_MOTOR_MODEL_COUNT; ++profile) {
		gzrl_motor_config config = {};
		gzrl_motor motor;
		gzrl_motor_command command;
		gzrl_motor_frame frame;

		config.model = static_cast<gzrl_motor_model>(profile);
		config.id = 1;
		config.gear_ratio = 1.0f;
		config.effort_limit = gzrl_motor_is_mit(config.model) ? 1.0f : 100.0f;
		config.pmax = 3.14159265f;
		config.vmax = 30.0f;
		config.tmax = 10.0f;
		config.feedback_timeout_us = 10000;
		if (gzrl_motor_init(&motor, &config) != 0 || gzrl_motor_online(&motor, 0) ||
		    gzrl_motor_stop_command(&motor, &command) != 0 ||
		    gzrl_motor_encode(&motor, &command, &frame) != 0 || frame.length != 8 ||
		    (gzrl_motor_capabilities(config.model) & GZRL_MOTOR_CAP(command.type)) == 0) {
			return false;
		}
	}
	return true;
}

int main()
{
	const int c_result = gzrl_portable_c_smoke();

	if (c_result != 0 || !check_control_interfaces() || !check_motor_interfaces()) {
		std::fprintf(stderr, "Portable C/C++ interface test failed (C result %d)\n",
			     c_result);
		return 1;
	}
	std::puts("Portable C17/C++11 API and linkage checks passed");
	return 0;
}
