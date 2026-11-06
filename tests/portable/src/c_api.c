/*
 * SPDX-FileCopyrightText: 2026 GMaster contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#include <gzrl/control.h>
#include <gzrl/motor/core.h>

#include <math.h>

/* Called by C++ to verify C linkage as well as independent controller/protocol composition. */
int gzrl_portable_c_smoke(void)
{
	struct gzrl_pid pid;
	const struct gzrl_pid_config pid_config = {
		.mode = GZRL_PID_POSITION,
		.derivative = GZRL_PID_D_MEASUREMENT,
		.kp = 2.0f,
		.output_min = -100.0f,
		.output_max = 100.0f,
		.dt_min = 0.001f,
		.dt_max = 0.1f,
	};
	struct gzrl_motor motor;
	const struct gzrl_motor_config motor_config = {
		.model = GZRL_MOTOR_M3508,
		.id = 1,
		.gear_ratio = 1.0f,
		.effort_limit = 100.0f,
		.feedback_timeout_us = 10000,
	};
	struct gzrl_motor_command command = {.type = GZRL_MOTOR_CMD_EFFORT};
	struct gzrl_motor_frame frame;
	float output;

	if (gzrl_pid_init(&pid, &pid_config) != 0 || gzrl_motor_init(&motor, &motor_config) != 0 ||
	    gzrl_pid_update(&pid, 5.0f, 2.0f, 0.01f, &output) != 0 || output != 6.0f) {
		return 1;
	}
	command.effort = output;
	if (gzrl_motor_encode(&motor, &command, &frame) != 0 || frame.id != 0x200 ||
	    frame.length != 8 || frame.data[0] != 0 || frame.data[1] != 6) {
		return 2;
	}
	frame = (struct gzrl_motor_frame){
		.id = 0x201,
		.length = 8,
		.data = {0x10, 0x00, 0, 60, 0, 100, 42, 0},
	};
	if (gzrl_motor_receive(&motor, &frame, 0) != 0 || !gzrl_motor_online(&motor, 0) ||
	    fabsf(motor.feedback.rotor_angle_rad - 3.14159265f) > 0.00001f ||
	    motor.feedback.effort != 100.0f || motor.feedback.position_rad != 0.0) {
		return 3;
	}
	return 0;
}
