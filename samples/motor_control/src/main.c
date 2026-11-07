/*
 * SPDX-FileCopyrightText: 2026 GMaster contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <gzrl/control.h>
#include <gzrl/motor/bus.h>

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <string.h>

#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_string_conv.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(motor_control);

enum request_type {
	REQUEST_INFO,
	REQUEST_START,
	REQUEST_REFRESH,
	REQUEST_TARGET,
	REQUEST_STATUS,
	REQUEST_CLEAR,
	REQUEST_ZERO,
	REQUEST_DRIVE_ZERO,
	REQUEST_BRAKE,
	REQUEST_SHUTDOWN,
};

enum control_mode {
	MODE_IDLE,
	MODE_DIRECT,
	MODE_SPEED,
	MODE_ANGLE,
	MODE_POSITION,
	MODE_EFFORT,
	MODE_EXTERNAL,
	MODE_DRIVE_SPEED,
	MODE_DRIVE_SINGLE,
	MODE_DRIVE_MULTI,
};

struct operator_request {
	uint64_t received_us;
	enum request_type type;
	enum control_mode mode;
	float value;
	float auxiliary;
	bool clockwise;
};

struct controller {
	struct gzrl_pid pid;
	struct gzrl_cascade cascade;
	struct operator_request target;
	uint64_t authorized_us;
	uint64_t last_step_us;
	uint64_t last_outer_us;
	uint32_t step_min_us;
	uint32_t step_max_us;
	bool authorized;
};

K_MSGQ_DEFINE(request_queue, sizeof(struct operator_request), 8, 4);
static atomic_t stop_requested;
static atomic_t ready;
static struct gzrl_motor_bus motor_bus;
static struct controller controller;
static const struct device *const motor = DEVICE_DT_GET(DT_ALIAS(motor0));
static struct gzrl_motor_config motor_config;

static int parse_milli(const char *text, float *value)
{
	int error = 0;
	long number = shell_strtol(text, 10, &error);

	if (error != 0 || number < -1000000000L || number > 1000000000L) {
		return -EINVAL;
	}
	*value = (float)number / 1000.0f;
	return 0;
}

static int command_handler(const struct shell *shell, size_t argc, char **argv)
{
	struct operator_request request = {.received_us = gzrl_motor_bus_now_us(),
					   .type = REQUEST_TARGET};

	if (!atomic_get(&ready)) {
		shell_error(shell, "Motor sample is not ready; check the startup log.");
		return -ENODEV;
	}
	if (strcmp(argv[0], "stop") == 0) {
		atomic_set(&stop_requested, 1);
		shell_print(shell, "Stop requested; pending operator commands will be discarded.");
		return 0;
	}
	if (strcmp(argv[0], "info") == 0) {
		request.type = REQUEST_INFO;
	} else if (strcmp(argv[0], "start") == 0) {
		request.type = REQUEST_START;
	} else if (strcmp(argv[0], "refresh") == 0) {
		request.type = REQUEST_REFRESH;
	} else if (strcmp(argv[0], "status") == 0) {
		request.type = REQUEST_STATUS;
	} else if (strcmp(argv[0], "clear_error") == 0) {
		request.type = REQUEST_CLEAR;
	} else if (strcmp(argv[0], "zero") == 0) {
		request.type = REQUEST_ZERO;
	} else if (strcmp(argv[0], "drive_zero") == 0) {
		request.type = REQUEST_DRIVE_ZERO;
	} else if (strcmp(argv[0], "brake") == 0) {
		request.type = REQUEST_BRAKE;
	} else if (strcmp(argv[0], "shutdown") == 0) {
		request.type = REQUEST_SHUTDOWN;
	} else if (strcmp(argv[0], "direct") == 0) {
		request.mode = MODE_DIRECT;
	} else if (strcmp(argv[0], "speed") == 0) {
		request.mode = MODE_SPEED;
	} else if (strcmp(argv[0], "angle") == 0) {
		request.mode = MODE_ANGLE;
	} else if (strcmp(argv[0], "position") == 0) {
		request.mode = MODE_POSITION;
	} else if (strcmp(argv[0], "effort") == 0) {
		request.mode = MODE_EFFORT;
	} else if (strcmp(argv[0], "external_demo") == 0) {
		request.mode = MODE_EXTERNAL;
	} else if (strcmp(argv[0], "drive_speed") == 0) {
		request.mode = MODE_DRIVE_SPEED;
	} else if (strcmp(argv[0], "drive_single") == 0) {
		request.mode = MODE_DRIVE_SINGLE;
	} else if (strcmp(argv[0], "drive_multi") == 0) {
		request.mode = MODE_DRIVE_MULTI;
	} else {
		return -EINVAL;
	}
	if (request.type == REQUEST_TARGET) {
		if (parse_milli(argv[1], &request.value) != 0 ||
		    (argc > 2 && parse_milli(argv[2], &request.auxiliary) != 0)) {
			atomic_set(&stop_requested, 1);
			return -EINVAL;
		}
		if (argc > 3) {
			if (strcmp(argv[3], "0") != 0 && strcmp(argv[3], "1") != 0) {
				atomic_set(&stop_requested, 1);
				return -EINVAL;
			}
			request.clockwise = strcmp(argv[3], "1") == 0;
		}
	}
	int ret = k_msgq_put(&request_queue, &request, K_NO_WAIT);

	if (ret != 0) {
		shell_error(shell, "Operator queue full; request was not accepted.");
	}
	return ret;
}

/* Zephyr Shell macros intentionally expand identical NULL branches for leaf commands. */
/* NOLINTBEGIN(bugprone-branch-clone) */
SHELL_STATIC_SUBCMD_SET_CREATE(
	motor_commands,
	SHELL_CMD_ARG(info, NULL, "Read status; numbers use milli-units.", command_handler, 1, 0),
	SHELL_CMD_ARG(start, NULL, "Authorize startup for a bounded lease.", command_handler, 1, 0),
	SHELL_CMD_ARG(stop, NULL, "Stop and revoke authorization.", command_handler, 1, 0),
	SHELL_CMD_ARG(refresh, NULL, "Renew existing authorization; never restart.",
		      command_handler, 1, 0),
	SHELL_CMD_ARG(direct, NULL, "<native-effort * 1000>", command_handler, 2, 0),
	SHELL_CMD_ARG(speed, NULL, "<output speed in mrad/s>: software PID.", command_handler, 2,
		      0),
	SHELL_CMD_ARG(angle, NULL, "<rotor angle in mrad>: wrapped software PID.", command_handler,
		      2, 0),
	SHELL_CMD_ARG(position, NULL, "<output displacement in mrad>: position/speed cascade.",
		      command_handler, 2, 0),
	SHELL_CMD_ARG(effort, NULL, "<native feedback effort * 1000>: software PID.",
		      command_handler, 2, 0),
	SHELL_CMD_ARG(external_demo, NULL, "<target * 1000> <synthetic feedback * 1000>",
		      command_handler, 3, 0),
	SHELL_CMD_ARG(drive_speed, NULL, "<output speed in mrad/s>: MG internal loop.",
		      command_handler, 2, 0),
	SHELL_CMD_ARG(drive_single, NULL,
		      "<rotor angle mrad> <max output speed mrad/s> <clockwise 0|1>",
		      command_handler, 4, 0),
	SHELL_CMD_ARG(drive_multi, NULL, "<absolute output angle mrad> <max output speed mrad/s>",
		      command_handler, 3, 0),
	SHELL_CMD_ARG(status, NULL, "Request MG status while stopped.", command_handler, 1, 0),
	SHELL_CMD_ARG(clear_error, NULL, "Request supported drive error clear while stopped.",
		      command_handler, 1, 0),
	SHELL_CMD_ARG(zero, NULL, "Reset software displacement origin while stopped.",
		      command_handler, 1, 0),
	SHELL_CMD_ARG(drive_zero, NULL,
		      "Explicitly request supported drive-side zero while stopped.",
		      command_handler, 1, 0),
	SHELL_CMD_ARG(brake, NULL, "Engage supported mechanical brake while stopped.",
		      command_handler, 1, 0),
	SHELL_CMD_ARG(shutdown, NULL, "Stop, then shut down this sample's CAN controller.",
		      command_handler, 1, 0),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(motor, &motor_commands, "Explicitly authorized motor control sample.", NULL);
/* NOLINTEND(bugprone-branch-clone) */

static void reset_control(uint64_t now_us)
{
	(void)gzrl_pid_reset(&controller.pid);
	(void)gzrl_cascade_reset(&controller.cascade);
	controller.last_step_us = now_us;
	controller.last_outer_us = now_us;
	controller.step_min_us = UINT32_MAX;
	controller.step_max_us = 0;
}

static void revoke_control(uint64_t now_us)
{
	controller.authorized = false;
	controller.target.mode = MODE_IDLE;
	reset_control(now_us);
	(void)gzrl_motor_device_stop(motor);
}

static int milli_value(double value)
{
	value *= 1000.0;
	if (!isfinite(value)) {
		return 0;
	}
	if (value > INT_MAX) {
		return INT_MAX;
	}
	if (value < INT_MIN) {
		return INT_MIN;
	}
	return (int)value;
}

static void report(uint64_t now_us)
{
	struct gzrl_motor_snapshot snapshot;
	int ret = gzrl_motor_device_snapshot(motor, now_us, &snapshot);

	if (ret != 0) {
		LOG_ERR("Snapshot failed: %d", ret);
		return;
	}
	LOG_INF("model=%u state=%u fault=%u online=%u valid=0x%x error=%d",
		(unsigned int)motor_config.model, (unsigned int)snapshot.state,
		(unsigned int)snapshot.fault, snapshot.online, snapshot.feedback.valid,
		snapshot.last_error);
	LOG_INF("rotor_mrad=%d position_mrad=%d speed_mrad_s=%d effort_milli=%d unit=%u",
		milli_value((double)snapshot.feedback.rotor_angle_rad),
		milli_value(snapshot.feedback.position_rad),
		milli_value((double)snapshot.feedback.velocity_rad_s),
		milli_value((double)snapshot.feedback.effort),
		(unsigned int)snapshot.feedback.effort_unit);
	LOG_INF("authorized=%u mode=%u rx=%u drop=%u tx=%u done=%u fail=%u busy=%u stall=%u",
		controller.authorized, (unsigned int)controller.target.mode,
		motor_bus.stats.received, motor_bus.stats.dropped, motor_bus.stats.submitted,
		motor_bus.stats.completed, motor_bus.stats.failed, motor_bus.stats.mailbox_busy,
		motor_bus.stats.stalled);
	LOG_INF("step_min_us=%u step_max_us=%u temperature_c=%d",
		controller.step_max_us != 0 ? controller.step_min_us : 0, controller.step_max_us,
		(int)snapshot.feedback.temperature_c);
}

static int accept_target(const struct operator_request *request, uint64_t now_us)
{
	float speed_limit = (float)CONFIG_SAMPLE_MOTOR_SPEED_LIMIT_MILLI / 1000.0f;
	float position_limit = (float)CONFIG_SAMPLE_MOTOR_POSITION_LIMIT_MILLI / 1000.0f;

	if (!controller.authorized) {
		return -EACCES;
	}
	if ((request->mode == MODE_DIRECT && fabsf(request->value) > motor_config.effort_limit) ||
	    ((request->mode == MODE_SPEED || request->mode == MODE_DRIVE_SPEED) &&
	     fabsf(request->value) > speed_limit) ||
	    ((request->mode == MODE_ANGLE || request->mode == MODE_POSITION ||
	      request->mode == MODE_DRIVE_SINGLE || request->mode == MODE_DRIVE_MULTI) &&
	     fabsf(request->value) > position_limit) ||
	    ((request->mode == MODE_DRIVE_SINGLE || request->mode == MODE_DRIVE_MULTI) &&
	     (request->auxiliary <= 0.0f || request->auxiliary > speed_limit))) {
		return -ERANGE;
	}
	if (request->mode != controller.target.mode) {
		struct gzrl_pid_config config;

		reset_control(now_us);
		(void)gzrl_pid_get_config(&controller.pid, &config);
		config.derivative =
			request->mode == MODE_ANGLE ? GZRL_PID_D_ERROR : GZRL_PID_D_MEASUREMENT;
		(void)gzrl_pid_init(&controller.pid, &config);
	}
	controller.target = *request;
	controller.authorized_us = request->received_us;
	return 0;
}

static int handle_request(const struct operator_request *request, uint64_t now_us, bool *shutdown)
{
	struct gzrl_motor_command command = {0};
	int ret;

	if (now_us < request->received_us ||
	    now_us - request->received_us >= (uint64_t)CONFIG_SAMPLE_MOTOR_LEASE_MS * 1000U) {
		return -ESTALE;
	}
	switch (request->type) {
	case REQUEST_INFO:
		report(now_us);
		return 0;
	case REQUEST_START:
		ret = gzrl_motor_device_start(motor, now_us);
		if (ret == 0) {
			controller.authorized = true;
			controller.authorized_us = request->received_us;
			controller.target.mode = MODE_IDLE;
			reset_control(now_us);
		}
		return ret;
	case REQUEST_REFRESH:
		if (!controller.authorized) {
			return -EACCES;
		}
		controller.authorized_us = request->received_us;
		return 0;
	case REQUEST_TARGET:
		return accept_target(request, now_us);
	case REQUEST_ZERO:
		return gzrl_motor_device_reset_position(motor, now_us);
	case REQUEST_STATUS:
		command.type = GZRL_MOTOR_CMD_STATUS;
		break;
	case REQUEST_CLEAR:
		command.type = GZRL_MOTOR_CMD_CLEAR_ERROR;
		break;
	case REQUEST_DRIVE_ZERO:
		command.type = GZRL_MOTOR_CMD_SET_ZERO;
		break;
	case REQUEST_BRAKE:
		command.type = GZRL_MOTOR_CMD_BRAKE;
		command.brake_engaged = true;
		break;
	case REQUEST_SHUTDOWN:
		revoke_control(now_us);
		*shutdown = true;
		return 0;
	default:
		return -EINVAL;
	}
	return gzrl_motor_device_operation(motor, &command, now_us);
}

static int calculate_command(const struct gzrl_motor_feedback *feedback, uint64_t now_us,
			     struct gzrl_motor_command *command)
{
	const struct operator_request *target = &controller.target;
	float dt = (float)(now_us - controller.last_step_us) / 1000000.0f;
	float reference = target->value;
	float measurement;
	int ret;

	*command = (struct gzrl_motor_command){
		.type = GZRL_MOTOR_CMD_EFFORT,
		.effort_unit = gzrl_motor_command_unit(motor_config.model),
	};
	switch (target->mode) {
	case MODE_IDLE:
		return 0;
	case MODE_DIRECT:
		command->effort = reference;
		return 0;
	case MODE_DRIVE_SPEED:
		command->type = GZRL_MOTOR_CMD_SPEED;
		command->velocity_rad_s = reference;
		return 0;
	case MODE_DRIVE_SINGLE:
	case MODE_DRIVE_MULTI:
		command->type = target->mode == MODE_DRIVE_SINGLE ? GZRL_MOTOR_CMD_SINGLE_ANGLE
								  : GZRL_MOTOR_CMD_MULTI_ANGLE;
		command->position_rad = reference;
		command->max_velocity_rad_s = target->auxiliary;
		command->clockwise = target->clockwise;
		return 0;
	case MODE_SPEED:
		if (!(feedback->valid & GZRL_MOTOR_VALID_VELOCITY)) {
			return -ENODATA;
		}
		measurement = feedback->velocity_rad_s;
		break;
	case MODE_ANGLE:
		if (!(feedback->valid & GZRL_MOTOR_VALID_ANGLE)) {
			return -ENODATA;
		}
		ret = gzrl_angle_error(reference, feedback->rotor_angle_rad, &reference);
		if (ret != 0) {
			return ret;
		}
		measurement = 0.0f;
		break;
	case MODE_POSITION: {
		uint32_t needed = GZRL_MOTOR_VALID_POSITION | GZRL_MOTOR_VALID_VELOCITY;
		bool outer_due = !controller.cascade.outer_ready ||
				 now_us - controller.last_outer_us >= 10000;
		float outer_dt = (float)(now_us - controller.last_outer_us) / 1000000.0f;

		if ((feedback->valid & needed) != needed || !isfinite(feedback->position_rad) ||
		    fabs(feedback->position_rad) > 1000000.0) {
			return -ENODATA;
		}
		ret = gzrl_cascade_update(&controller.cascade, reference,
					  (float)feedback->position_rad, feedback->velocity_rad_s,
					  outer_due, outer_dt, dt, &command->effort);
		if (ret == 0 && outer_due) {
			controller.last_outer_us = now_us;
		}
		return ret;
	}
	case MODE_EFFORT:
		if (!(feedback->valid & GZRL_MOTOR_VALID_EFFORT)) {
			return -ENODATA;
		}
		measurement = feedback->effort;
		break;
	case MODE_EXTERNAL:
		/* Replace this explicitly synthetic input with an application sensor sample. */
		measurement = target->auxiliary;
		break;
	default:
		return -EINVAL;
	}
	return gzrl_pid_update(&controller.pid, reference, measurement, dt, &command->effort);
}

static int setup(const struct device **can)
{
	const struct gzrl_motor_policy policy = {
		.command_timeout_us = 50000,
		.startup_timeout_us = 500000,
		.retry_interval_us = 50000,
		.tx_timeout_us = 50000,
		.max_attempts = 5,
	};
	const struct device *devices[] = {motor};
	int ret = gzrl_motor_device_topology(motor, &motor_config, can);

	if (ret != 0) {
		return ret;
	}
	motor_config.encoder_offset = CONFIG_SAMPLE_MOTOR_ENCODER_OFFSET;
	motor_config.gear_ratio = (float)CONFIG_SAMPLE_MOTOR_GEAR_RATIO_MILLI / 1000.0f;
	motor_config.effort_limit = (float)CONFIG_SAMPLE_MOTOR_EFFORT_LIMIT_MILLI / 1000.0f;
	motor_config.pmax = (float)CONFIG_SAMPLE_MOTOR_PMAX_MILLI / 1000.0f;
	motor_config.vmax = (float)CONFIG_SAMPLE_MOTOR_VMAX_MILLI / 1000.0f;
	motor_config.tmax = (float)CONFIG_SAMPLE_MOTOR_TMAX_MILLI / 1000.0f;
	motor_config.position_wrap_rad = (float)CONFIG_SAMPLE_MOTOR_POSITION_WRAP_MILLI / 1000.0f;
	motor_config.feedback_timeout_us = 100000;
	ret = gzrl_motor_device_configure(motor, &motor_config, &policy);
	if (ret != 0) {
		return ret;
	}
	struct gzrl_pid_config inner = {
		.mode = GZRL_PID_POSITION,
		.derivative = GZRL_PID_D_MEASUREMENT,
		.kp = (float)CONFIG_SAMPLE_MOTOR_KP_MILLI / 1000.0f,
		.ki = (float)CONFIG_SAMPLE_MOTOR_KI_MILLI / 1000.0f,
		.kd = (float)CONFIG_SAMPLE_MOTOR_KD_MILLI / 1000.0f,
		.output_min = -motor_config.effort_limit,
		.output_max = motor_config.effort_limit,
		.integral_min = -motor_config.effort_limit,
		.integral_max = motor_config.effort_limit,
		.dt_min = 0.0001f,
		.dt_max = 0.02f,
	};
	struct gzrl_pid_config outer = inner;

	outer.kp = (float)CONFIG_SAMPLE_MOTOR_OUTER_KP_MILLI / 1000.0f;
	outer.ki = 0.0f;
	outer.kd = 0.0f;
	outer.output_max = (float)CONFIG_SAMPLE_MOTOR_SPEED_LIMIT_MILLI / 1000.0f;
	outer.output_min = -outer.output_max;
	outer.integral_min = 0.0f;
	outer.integral_max = 0.0f;
	outer.dt_max = 0.05f;
	ret = gzrl_pid_init(&controller.pid, &inner);
	if (ret == 0) {
		ret = gzrl_cascade_init(&controller.cascade, &outer, &inner);
	}
	if (ret == 0) {
		ret = can_set_mode(*can, CAN_MODE_NORMAL);
	}
	if (ret == 0) {
		ret = can_set_bitrate(*can, CONFIG_SAMPLE_MOTOR_BITRATE);
	}
	if (ret == 0) {
		ret = gzrl_motor_bus_attach(&motor_bus, *can, devices, ARRAY_SIZE(devices));
	}
	if (ret == 0) {
		ret = can_start(*can);
		if (ret != 0) {
			(void)gzrl_motor_bus_detach(&motor_bus);
		}
	}
	return ret;
}

int main(void)
{
	const struct device *can;
	int ret = setup(&can);
	bool shutdown = false;
	uint64_t shutdown_us = 0;
	int64_t next_tick = k_uptime_get();
	enum gzrl_motor_run_state previous_state = GZRL_MOTOR_STOPPED;

	if (ret != 0) {
		LOG_ERR("Motor sample setup failed: %d. Check topology and explicit runtime "
			"limits.",
			ret);
		return 0;
	}
	atomic_set(&ready, 1);
	LOG_INF("Motor sample ready: stopped, no motion authorized. Run motor info.");
	while (true) {
		uint64_t now_us = gzrl_motor_bus_now_us();
		struct gzrl_motor_snapshot snapshot;
		struct operator_request request;

		(void)gzrl_motor_bus_process(&motor_bus, now_us, 32);
		if (atomic_cas(&stop_requested, 1, 0)) {
			k_msgq_purge(&request_queue);
			revoke_control(now_us);
		}
		(void)gzrl_motor_device_snapshot(motor, now_us, &snapshot);
		if (controller.authorized &&
		    (snapshot.state == GZRL_MOTOR_FAULT ||
		     (!snapshot.online && snapshot.state == GZRL_MOTOR_RUNNING))) {
			LOG_ERR("Control revoked: motor fault=%u error=%d.",
				(unsigned int)snapshot.fault, snapshot.last_error);
			controller.authorized = false;
			controller.target.mode = MODE_IDLE;
			reset_control(now_us);
		}
		if (controller.authorized &&
		    now_us - controller.authorized_us >=
			    (uint64_t)CONFIG_SAMPLE_MOTOR_LEASE_MS * 1000U) {
			LOG_INF("Operator lease expired; explicit start is required.");
			revoke_control(now_us);
		}
		for (size_t i = 0; i < 8 && !shutdown; ++i) {
			if (k_msgq_get(&request_queue, &request, K_NO_WAIT) != 0) {
				break;
			}
			now_us = gzrl_motor_bus_now_us();
			ret = handle_request(&request, now_us, &shutdown);
			if (ret != 0) {
				LOG_WRN("Request rejected: %d", ret);
				if (request.type == REQUEST_TARGET) {
					revoke_control(now_us);
				}
			} else if (request.type != REQUEST_INFO) {
				LOG_INF("Accepted; use motor info to inspect completion.");
			}
		}
		now_us = gzrl_motor_bus_now_us();
		(void)gzrl_motor_device_snapshot(motor, now_us, &snapshot);
		if (snapshot.state == GZRL_MOTOR_RUNNING && previous_state != GZRL_MOTOR_RUNNING) {
			reset_control(now_us);
		}
		previous_state = snapshot.state;
		if (controller.authorized && snapshot.state == GZRL_MOTOR_RUNNING &&
		    now_us > controller.last_step_us) {
			struct gzrl_motor_command command;
			uint32_t step_us = (uint32_t)MIN(now_us - controller.last_step_us,
							 (uint64_t)UINT32_MAX);

			controller.step_min_us = MIN(controller.step_min_us, step_us);
			controller.step_max_us = MAX(controller.step_max_us, step_us);

			ret = calculate_command(&snapshot.feedback, now_us, &command);
			if (ret == 0) {
				ret = gzrl_motor_device_command(motor, &command, now_us);
			}
			controller.last_step_us = now_us;
			if (ret != 0) {
				LOG_ERR("Control update failed: %d; stopping.", ret);
				revoke_control(now_us);
			}
		}
		(void)gzrl_motor_bus_flush(&motor_bus, now_us);
		if (shutdown) {
			atomic_set(&ready, 0);
			if (shutdown_us == 0) {
				shutdown_us = now_us;
			}
			(void)gzrl_motor_device_snapshot(motor, now_us, &snapshot);
			if (!snapshot.operation_pending || now_us - shutdown_us >= 100000) {
				if (snapshot.operation_pending) {
					LOG_ERR("Stop delivery unconfirmed; drive timeout remains "
						"required.");
				}
				ret = can_stop(can);
				int detach_ret = gzrl_motor_bus_detach(&motor_bus);

				LOG_INF("CAN stop=%d, detach=%d. Reset board to restart sample.",
					ret, detach_ret);
				return 0;
			}
		}
		next_tick += 1;
		if (next_tick <= k_uptime_get()) {
			next_tick = k_uptime_get() + 1;
		}
		k_sleep(K_TIMEOUT_ABS_MS(next_tick));
	}
}
