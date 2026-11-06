/*
 * SPDX-FileCopyrightText: 2026 GMaster contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#include <gzrl/motor/core.h>

#include <errno.h>
#include <float.h>
#include <math.h>
#include <string.h>
#include <zephyr/ztest.h>

#define PI_F  3.14159265358979323846f
#define TAU_F (2.0f * PI_F)

static struct gzrl_motor make_motor(enum gzrl_motor_model model, uint16_t id)
{
	struct gzrl_motor motor;
	struct gzrl_motor_config config = {
		.model = model,
		.id = id,
		.master_id = gzrl_motor_is_mit(model) ? 0x100 : 0,
		.gear_ratio = 1.0f,
		.effort_limit = gzrl_motor_is_mit(model) ? 12.0f : 1000.0f,
		.pmax = PI_F,
		.vmax = 30.0f,
		.tmax = 12.0f,
		.feedback_timeout_us = 10000,
	};

	zassert_ok(gzrl_motor_init(&motor, &config));
	return motor;
}

static struct gzrl_motor_frame grouped_feedback(const struct gzrl_motor *motor, uint16_t encoder)
{
	struct gzrl_motor_frame frame = {
		.id = motor->rx_id,
		.length = 8,
		.data = {(uint8_t)(encoder >> 8), (uint8_t)encoder, 0xff, 0xc4, 0xff, 0x85, 50, 0},
	};

	return frame;
}

ZTEST(motor_core, test_profile_ids_and_slots)
{
	const uint16_t maximum[] = {7, 8, 8, 15, 15, 8, 32};
	const uint16_t feedback_base[] = {0x204, 0x200, 0x200, 0, 0, 0x300, 0x140};
	const uint16_t first[] = {0x1ff, 0x200, 0x200, 0, 0, 0x3fe, 0};
	const uint16_t second[] = {0x2ff, 0x1ff, 0x1ff, 0, 0, 0x4fe, 0};

	for (int model = 0; model < GZRL_MOTOR_MODEL_COUNT; ++model) {
		for (uint16_t id = 1; id <= maximum[model]; ++id) {
			struct gzrl_motor motor = make_motor(model, id);

			zassert_false(gzrl_motor_online(&motor, 0));
			zassert_equal(motor.feedback.valid, 0);
			if (gzrl_motor_is_mit(model)) {
				zassert_equal(motor.tx_id, id);
				zassert_equal(motor.rx_id, 0x100);
				zassert_equal(motor.group_slot, -1);
			} else if (model == GZRL_MOTOR_LKMG) {
				zassert_equal(motor.tx_id, 0x140 + id);
				zassert_equal(motor.rx_id, motor.tx_id);
				zassert_equal(motor.group_slot, -1);
			} else {
				zassert_equal(motor.tx_id, id <= 4 ? first[model] : second[model]);
				zassert_equal(motor.rx_id, feedback_base[model] + id);
				zassert_equal(motor.group_slot, (id - 1) % 4);
			}
		}
		struct gzrl_motor motor = make_motor(model, 1);
		struct gzrl_motor_config config = motor.config;
		struct gzrl_motor before = motor;

		config.id = maximum[model] + 1;
		zassert_equal(gzrl_motor_init(&motor, &config), -EINVAL);
		zassert_mem_equal(&motor, &before, sizeof(motor));
		config.id = 0;
		zassert_equal(gzrl_motor_init(&motor, &config), -EINVAL);
	}
}

ZTEST(motor_core, test_configuration_limits)
{
	struct gzrl_motor motor = make_motor(GZRL_MOTOR_M2006, 1);
	struct gzrl_motor_config config = motor.config;
	const float bad_ratios[] = {0.0f, -1.0f, NAN, INFINITY};

	for (size_t i = 0; i < ARRAY_SIZE(bad_ratios); ++i) {
		config.gear_ratio = bad_ratios[i];
		zassert_equal(gzrl_motor_init(&motor, &config), -EINVAL);
	}
	config = motor.config;
	config.effort_limit = 10001;
	zassert_equal(gzrl_motor_init(&motor, &config), -EINVAL);
	config.effort_limit = 10000;
	zassert_ok(gzrl_motor_init(&motor, &config));
	config.model = GZRL_MOTOR_M3508;
	config.effort_limit = 16384;
	zassert_ok(gzrl_motor_init(&motor, &config));
	config.effort_limit = 16385;
	zassert_equal(gzrl_motor_init(&motor, &config), -EINVAL);
	config.model = GZRL_MOTOR_GM6020;
	config.effort_limit = 30000;
	zassert_ok(gzrl_motor_init(&motor, &config));
	config.effort_limit = 30001;
	zassert_equal(gzrl_motor_init(&motor, &config), -EINVAL);
	config = motor.config;
	config.encoder_offset = 8192;
	zassert_equal(gzrl_motor_init(&motor, &config), -EINVAL);
	config = motor.config;
	config.feedback_timeout_us = 0;
	zassert_equal(gzrl_motor_init(&motor, &config), -EINVAL);
	config = motor.config;
	config.model = GZRL_MOTOR_MODEL_COUNT;
	zassert_equal(gzrl_motor_init(&motor, &config), -EINVAL);

	motor = make_motor(GZRL_MOTOR_DM4310, 1);
	config = motor.config;
	config.pmax = INFINITY;
	zassert_equal(gzrl_motor_init(&motor, &config), -EINVAL);
	config = motor.config;
	config.tmax = 0;
	zassert_equal(gzrl_motor_init(&motor, &config), -EINVAL);
	config = motor.config;
	config.effort_limit = config.tmax + 1;
	zassert_equal(gzrl_motor_init(&motor, &config), -EINVAL);
	config = motor.config;
	config.master_id = 0x800;
	zassert_equal(gzrl_motor_init(&motor, &config), -EINVAL);
}

ZTEST(motor_core, test_grouped_wire_vectors_and_transactional_merge)
{
	const enum gzrl_motor_model models[] = {GZRL_MOTOR_GM6020, GZRL_MOTOR_M3508,
						GZRL_MOTOR_M2006, GZRL_MOTOR_DM_MULTI};
	const uint8_t big_endian[] = {0x01, 0x23, 0xfe, 0xdc, 0x00, 0x01, 0xff, 0xff};
	const uint8_t little_endian[] = {0x23, 0x01, 0xdc, 0xfe, 0x01, 0x00, 0xff, 0xff};
	const float values[] = {291, -292, 1, -1};

	for (size_t i = 0; i < ARRAY_SIZE(models); ++i) {
		struct gzrl_motor_group group = {0};
		struct gzrl_motor_command command = {.type = GZRL_MOTOR_CMD_EFFORT};

		for (uint16_t id = 1; id <= 4; ++id) {
			struct gzrl_motor motor = make_motor(models[i], id);

			command.effort = values[id - 1];
			zassert_ok(gzrl_motor_group_add(&group, &motor, &command));
		}
		zassert_mem_equal(group.frame.data,
				  models[i] == GZRL_MOTOR_DM_MULTI ? little_endian : big_endian, 8);
		zassert_equal(group.occupied, 15);
		struct gzrl_motor duplicate = make_motor(models[i], 1);
		struct gzrl_motor wrong_group = make_motor(models[i], 5);
		struct gzrl_motor_group before = group;

		zassert_equal(gzrl_motor_group_add(&group, &duplicate, &command), -EEXIST);
		zassert_equal(gzrl_motor_group_add(&group, &wrong_group, &command), -EINVAL);
		zassert_mem_equal(&group, &before, sizeof(group));
	}
}

ZTEST(motor_core, test_grouped_feedback_precision_and_validity)
{
	const enum gzrl_motor_model models[] = {GZRL_MOTOR_GM6020, GZRL_MOTOR_M3508,
						GZRL_MOTOR_M2006, GZRL_MOTOR_DM_MULTI};

	for (size_t i = 0; i < ARRAY_SIZE(models); ++i) {
		struct gzrl_motor motor = make_motor(models[i], 1);
		struct gzrl_motor_config config = motor.config;

		config.gear_ratio = 2.0f;
		config.encoder_offset = 100;
		zassert_ok(gzrl_motor_init(&motor, &config));
		struct gzrl_motor_frame frame = grouped_feedback(&motor, 4196);

		zassert_ok(gzrl_motor_receive(&motor, &frame, 100));
		zassert_within(motor.feedback.rotor_angle_rad, PI_F, 1e-6f);
		zassert_equal(motor.feedback.position_rad, 0.0);
		zassert_within(motor.feedback.velocity_rad_s,
			       models[i] == GZRL_MOTOR_DM_MULTI ? -PI_F / 100 : -PI_F, 1e-6f);
		zassert_equal(motor.feedback.effort, -123.0f);
		zassert_equal((motor.feedback.valid & GZRL_MOTOR_VALID_TEMPERATURE) != 0,
			      models[i] != GZRL_MOTOR_M2006);
		zassert_equal(motor.feedback.effort_unit, models[i] == GZRL_MOTOR_DM_MULTI
								  ? GZRL_MOTOR_EFFORT_MILLIAMP
								  : GZRL_MOTOR_EFFORT_RAW);
		frame.data[7] = 1;
		zassert_ok(gzrl_motor_receive(&motor, &frame, 101));
		zassert_equal(gzrl_motor_faulted(&motor), models[i] == GZRL_MOTOR_DM_MULTI);
	}
}

ZTEST(motor_core, test_position_wrap_loss_reset_and_timestamp)
{
	struct gzrl_motor motor = make_motor(GZRL_MOTOR_M3508, 1);
	struct gzrl_motor_frame frame = grouped_feedback(&motor, 8190);

	zassert_ok(gzrl_motor_receive(&motor, &frame, 100));
	frame = grouped_feedback(&motor, 2);
	zassert_ok(gzrl_motor_receive(&motor, &frame, 200));
	zassert_within(motor.feedback.position_rad, 4.0 * (double)TAU_F / 8192.0, 1e-6);
	zassert_true(gzrl_motor_online(&motor, 10199));
	zassert_false(gzrl_motor_online(&motor, 10200));
	zassert_false(gzrl_motor_online(&motor, 199));
	struct gzrl_motor before = motor;

	zassert_equal(gzrl_motor_receive(&motor, &frame, 199), -ESTALE);
	zassert_mem_equal(&motor, &before, sizeof(motor));
	frame = grouped_feedback(&motor, 20);
	zassert_ok(gzrl_motor_receive(&motor, &frame, 10200));
	zassert_false((motor.feedback.valid & GZRL_MOTOR_VALID_POSITION) != 0);
	zassert_equal(motor.feedback.position_rad, before.feedback.position_rad);
	zassert_ok(gzrl_motor_reset_position(&motor));
	frame = grouped_feedback(&motor, 16);
	zassert_ok(gzrl_motor_receive(&motor, &frame, 10201));
	zassert_within(motor.feedback.position_rad, -4.0 * (double)TAU_F / 8192.0, 1e-6);
	gzrl_motor_invalidate_position(&motor);
	zassert_false((motor.feedback.valid & GZRL_MOTOR_VALID_POSITION) != 0);
	zassert_ok(gzrl_motor_receive(&motor, &frame, 10202));
	zassert_false((motor.feedback.valid & GZRL_MOTOR_VALID_POSITION) != 0);
}

ZTEST(motor_core, test_rejected_frames_never_update_state)
{
	struct gzrl_motor motor = make_motor(GZRL_MOTOR_GM6020, 1);
	struct gzrl_motor_frame frame = grouped_feedback(&motor, 1);

	zassert_ok(gzrl_motor_receive(&motor, &frame, 1));
	struct gzrl_motor before = motor;

	for (uint8_t length = 0; length <= 9; ++length) {
		if (length == 8) {
			continue;
		}
		frame.length = length;
		zassert_equal(gzrl_motor_receive(&motor, &frame, 2), -EINVAL);
	}
	frame.length = 8;
	for (uint8_t flag = 1; flag <= 4; flag <<= 1) {
		frame.flags = flag;
		zassert_equal(gzrl_motor_receive(&motor, &frame, 2), -EINVAL);
	}
	frame.flags = 0;
	frame.id++;
	zassert_equal(gzrl_motor_receive(&motor, &frame, 2), -ENOENT);
	frame.id = 0x800;
	zassert_equal(gzrl_motor_receive(&motor, &frame, 2), -EINVAL);
	frame.id = motor.rx_id;
	frame.data[0] = 0x20;
	zassert_equal(gzrl_motor_receive(&motor, &frame, 2), -EINVAL);
	zassert_mem_equal(&motor, &before, sizeof(motor));
}

ZTEST(motor_core, test_mit_encoding_vectors_and_special_commands)
{
	const uint8_t zero_effort[] = {0x7f, 0xff, 0x7f, 0xf0, 0, 0, 0x07, 0xff};
	const uint8_t mid_gains[] = {0x7f, 0xff, 0x7f, 0xf7, 0xff, 0x7f, 0xf7, 0xff};
	const enum gzrl_motor_command_type specials[] = {
		GZRL_MOTOR_CMD_CLEAR_ERROR, GZRL_MOTOR_CMD_ENABLE, GZRL_MOTOR_CMD_DISABLE,
		GZRL_MOTOR_CMD_SET_ZERO};
	const uint8_t tails[] = {0xfb, 0xfc, 0xfd, 0xfe};

	for (int model = GZRL_MOTOR_DM4310; model <= GZRL_MOTOR_DM2325; ++model) {
		struct gzrl_motor motor = make_motor(model, 3);
		struct gzrl_motor_frame frame;
		struct gzrl_motor_command command = {
			.type = GZRL_MOTOR_CMD_EFFORT,
			.effort_unit = GZRL_MOTOR_EFFORT_NEWTON_METRE,
		};

		zassert_ok(gzrl_motor_encode(&motor, &command, &frame));
		zassert_equal(frame.id, 3);
		zassert_mem_equal(frame.data, zero_effort, 8);
		command.type = GZRL_MOTOR_CMD_MIT;
		command.kp = 250;
		command.kd = 2.5f;
		zassert_ok(gzrl_motor_encode(&motor, &command, &frame));
		zassert_mem_equal(frame.data, mid_gains, 8);
		command.position_rad = -motor.config.pmax;
		command.velocity_rad_s = -motor.config.vmax;
		command.kp = 0;
		command.kd = 0;
		command.effort = -motor.config.tmax;
		zassert_ok(gzrl_motor_encode(&motor, &command, &frame));
		for (size_t i = 0; i < 8; ++i) {
			zassert_equal(frame.data[i], 0);
		}
		command.position_rad = motor.config.pmax;
		command.velocity_rad_s = motor.config.vmax;
		command.kp = 500;
		command.kd = 5;
		command.effort = motor.config.tmax;
		zassert_ok(gzrl_motor_encode(&motor, &command, &frame));
		for (size_t i = 0; i < 8; ++i) {
			zassert_equal(frame.data[i], 0xff);
		}
		for (size_t i = 0; i < ARRAY_SIZE(specials); ++i) {
			command.type = specials[i];
			zassert_ok(gzrl_motor_encode(&motor, &command, &frame));
			zassert_equal(frame.data[7], tails[i]);
			for (size_t j = 0; j < 7; ++j) {
				zassert_equal(frame.data[j], 0xff);
			}
		}
	}
}

ZTEST(motor_core, test_mit_feedback_fractional_torque_and_payload_id)
{
	struct gzrl_motor motor = make_motor(GZRL_MOTOR_DM4310, 1);
	struct gzrl_motor_frame frame = {
		.id = 0x100,
		.length = 8,
		.data = {0x11, 0x80, 0, 0x80, 0x08, 0, 40, 41},
	};

	zassert_ok(gzrl_motor_receive(&motor, &frame, 0));
	zassert_true(gzrl_motor_online(&motor, 0));
	zassert_within(motor.feedback.native_position_rad, PI_F / 65535.0f, 1e-7f);
	zassert_within(motor.feedback.velocity_rad_s, 30.0f / 4095.0f, 1e-7f);
	zassert_within(motor.feedback.effort, 12.0f / 4095.0f, 1e-7f);
	zassert_equal(motor.feedback.effort_unit, GZRL_MOTOR_EFFORT_NEWTON_METRE);
	zassert_equal(motor.feedback.temperature_c, 41);
	zassert_equal(motor.feedback.mos_temperature_c, 40);
	zassert_false(gzrl_motor_faulted(&motor));
	struct gzrl_motor before = motor;

	frame.data[0] = 0x12;
	zassert_equal(gzrl_motor_receive(&motor, &frame, 1), -ENOENT);
	frame.data[0] = 0x21;
	zassert_equal(gzrl_motor_receive(&motor, &frame, 1), -EINVAL);
	zassert_mem_equal(&motor, &before, sizeof(motor));
	for (uint8_t fault = 8; fault <= 14; ++fault) {
		frame.data[0] = (uint8_t)((fault << 4) | 1);
		zassert_ok(gzrl_motor_receive(&motor, &frame, fault));
		zassert_true(gzrl_motor_faulted(&motor));
	}
}

ZTEST(motor_core, test_mit_dual_encoder_faults)
{
	struct gzrl_motor motor = make_motor(GZRL_MOTOR_DM4310, 2);
	struct gzrl_motor_frame frame = {
		.id = 0x100,
		.length = 8,
		.data = {0, 0x80, 0, 0x80, 0x08, 0, 25, 26},
	};

	/* DM-J4310-2EC V1.4: output calibration, sensor and motor encoder faults. */
	for (uint8_t state = 3; state <= 5; ++state) {
		frame.data[0] = (uint8_t)((state << 4) | 2);
		zassert_ok(gzrl_motor_receive(&motor, &frame, state));
		zassert_equal(motor.feedback.drive_state, state);
		zassert_true(gzrl_motor_faulted(&motor));
	}
	struct gzrl_motor before = motor;
	const uint8_t reserved[] = {2, 6, 7, 15};

	for (size_t i = 0; i < ARRAY_SIZE(reserved); ++i) {
		frame.data[0] = (uint8_t)((reserved[i] << 4) | 2);
		zassert_equal(gzrl_motor_receive(&motor, &frame, 10 + i), -EINVAL);
		zassert_mem_equal(&motor, &before, sizeof(motor));
	}
}

ZTEST(motor_core, test_mit_motion_cannot_alias_lifecycle_commands)
{
	struct gzrl_motor motor = make_motor(GZRL_MOTOR_DM4310, 1);
	struct gzrl_motor_command command = {
		.type = GZRL_MOTOR_CMD_MIT,
		.effort_unit = GZRL_MOTOR_EFFORT_NEWTON_METRE,
		.position_rad = motor.config.pmax,
		.velocity_rad_s = motor.config.vmax,
		.kp = 500,
		.kd = 5,
	};
	struct gzrl_motor_frame frame;

	for (uint16_t code = 4091; code <= 4094; ++code) {
		command.effort = ((float)code + 0.25f) / 4095.0f * 24.0f - 12.0f;
		zassert_equal(gzrl_motor_encode(&motor, &command, &frame), -ERANGE);
		zassert_equal(frame.length, 0);
	}
	command.type = GZRL_MOTOR_CMD_SET_ZERO;
	zassert_ok(gzrl_motor_encode(&motor, &command, &frame));
	zassert_equal(frame.data[7], 0xfe);
}

ZTEST(motor_core, test_mit_does_not_assume_pmax_is_physical_wrap)
{
	struct gzrl_motor motor = make_motor(GZRL_MOTOR_DM4310, 1);
	struct gzrl_motor_frame frame = {
		.id = 0x100,
		.length = 8,
		.data = {0x11, 0xff, 0xfe, 0, 0, 0, 40, 41},
	};

	zassert_ok(gzrl_motor_receive(&motor, &frame, 1));
	frame.data[1] = 0;
	frame.data[2] = 1;
	zassert_ok(gzrl_motor_receive(&motor, &frame, 2));
	zassert_false((motor.feedback.valid & GZRL_MOTOR_VALID_POSITION) != 0);
	struct gzrl_motor_config config = motor.config;

	config.position_wrap_rad = TAU_F;
	zassert_ok(gzrl_motor_init(&motor, &config));
	frame.data[1] = 0xff;
	frame.data[2] = 0xfe;
	zassert_ok(gzrl_motor_receive(&motor, &frame, 1));
	frame.data[1] = 0;
	frame.data[2] = 1;
	zassert_ok(gzrl_motor_receive(&motor, &frame, 2));
	zassert_true((motor.feedback.valid & GZRL_MOTOR_VALID_POSITION) != 0);
	zassert_within(motor.feedback.position_rad, 2.0 * (double)TAU_F / 65535.0, 1e-6);
}

ZTEST(motor_core, test_lkmg_commands_native_and_output_axis)
{
	struct gzrl_motor motor = make_motor(GZRL_MOTOR_LKMG, 32);
	struct gzrl_motor_config config = motor.config;
	struct gzrl_motor_frame frame;
	struct gzrl_motor_command command = {.type = GZRL_MOTOR_CMD_EFFORT, .effort = -291};
	const uint8_t torque[] = {0xa1, 0, 0, 0, 0xdd, 0xfe, 0, 0};
	const uint8_t speed[] = {0xa2, 0, 0, 0, 0x60, 0x73, 0xff, 0xff};
	const uint8_t angle[] = {0xa6, 0, 0x68, 0x01, 0x50, 0x46, 0, 0};
	const uint8_t multi[] = {0xa4, 0, 0x68, 0x01, 0x60, 0x73, 0xff, 0xff};

	config.gear_ratio = 2;
	zassert_ok(gzrl_motor_init(&motor, &config));
	zassert_ok(gzrl_motor_encode(&motor, &command, &frame));
	zassert_equal(frame.id, 0x160);
	zassert_mem_equal(frame.data, torque, 8);
	command.type = GZRL_MOTOR_CMD_SPEED;
	command.velocity_rad_s = -PI_F;
	zassert_ok(gzrl_motor_encode(&motor, &command, &frame));
	zassert_mem_equal(frame.data, speed, 8);
	command.type = GZRL_MOTOR_CMD_SINGLE_ANGLE;
	command.position_rad = PI_F;
	command.max_velocity_rad_s = PI_F;
	command.clockwise = true;
	zassert_ok(gzrl_motor_encode(&motor, &command, &frame));
	zassert_mem_equal(frame.data, angle, 8);
	command.clockwise = false;
	zassert_ok(gzrl_motor_encode(&motor, &command, &frame));
	zassert_equal(frame.data[1], 1);
	command.type = GZRL_MOTOR_CMD_MULTI_ANGLE;
	command.position_rad = -PI_F;
	zassert_ok(gzrl_motor_encode(&motor, &command, &frame));
	zassert_mem_equal(frame.data, multi, 8);
	command.type = GZRL_MOTOR_CMD_BRAKE;
	command.brake_engaged = false;
	zassert_ok(gzrl_motor_encode(&motor, &command, &frame));
	zassert_equal(frame.data[0], 0x8c);
	zassert_equal(frame.data[1], 1);
	command.brake_engaged = true;
	zassert_ok(gzrl_motor_encode(&motor, &command, &frame));
	zassert_equal(frame.data[1], 0);
	command.type = GZRL_MOTOR_CMD_STATUS;
	zassert_ok(gzrl_motor_encode(&motor, &command, &frame));
	zassert_equal(frame.data[0], 0x9c);
}

ZTEST(motor_core, test_lkmg_feedback_and_ack_age)
{
	struct gzrl_motor motor = make_motor(GZRL_MOTOR_LKMG, 1);
	struct gzrl_motor_frame frame = {
		.id = 0x141,
		.length = 8,
		.data = {0x9c, 0xfb, 0x85, 0xff, 0x4c, 0xff, 0xff, 0xff},
	};

	zassert_ok(gzrl_motor_receive(&motor, &frame, 0));
	zassert_equal(motor.feedback.temperature_c, -5);
	zassert_equal(motor.feedback.effort, -123);
	zassert_within(motor.feedback.velocity_rad_s, -PI_F, 1e-6f);
	zassert_true(motor.feedback.rotor_angle_rad < TAU_F);
	frame.data[6] = 0;
	frame.data[7] = 0;
	zassert_ok(gzrl_motor_receive(&motor, &frame, 1));
	zassert_within(motor.feedback.position_rad, (double)TAU_F / 65536.0, 1e-6);
	uint32_t motion_sequence = motor.feedback.sequence;

	frame.data[0] = 0x8c;
	frame.data[1] = 1;
	zassert_ok(gzrl_motor_receive(&motor, &frame, 10001));
	zassert_equal(motor.feedback.sequence, motion_sequence);
	zassert_equal(motor.feedback.timestamp_us, 1);
	zassert_equal(motor.feedback.status_timestamp_us, 10001);
	zassert_true((motor.feedback.valid & GZRL_MOTOR_VALID_BRAKE) != 0);
	zassert_false(motor.feedback.brake_engaged);
	zassert_false(gzrl_motor_online(&motor, 10001));
	struct gzrl_motor before = motor;

	frame.data[1] = 2;
	zassert_equal(gzrl_motor_receive(&motor, &frame, 10002), -EINVAL);
	frame.data[0] = 0x99;
	zassert_equal(gzrl_motor_receive(&motor, &frame, 10002), -ENOTSUP);
	zassert_mem_equal(&motor, &before, sizeof(motor));
}

ZTEST(motor_core, test_encode_errors_clear_output)
{
	const float invalid[] = {NAN, INFINITY, -INFINITY, 1001.0f, -1001.0f};
	struct gzrl_motor motor = make_motor(GZRL_MOTOR_M3508, 1);
	struct gzrl_motor_frame frame;
	struct gzrl_motor_frame empty = {0};
	struct gzrl_motor_command command = {.type = GZRL_MOTOR_CMD_EFFORT};

	for (size_t i = 0; i < ARRAY_SIZE(invalid); ++i) {
		memset(&frame, 0xff, sizeof(frame));
		command.effort = invalid[i];
		zassert_equal(gzrl_motor_encode(&motor, &command, &frame), -ERANGE);
		zassert_mem_equal(&frame, &empty, sizeof(frame));
	}
	command.effort = 0;
	command.effort_unit = GZRL_MOTOR_EFFORT_NEWTON_METRE;
	zassert_equal(gzrl_motor_encode(&motor, &command, &frame), -EINVAL);
	command.type = GZRL_MOTOR_CMD_SPEED;
	zassert_equal(gzrl_motor_encode(&motor, &command, &frame), -ENOTSUP);
	command.type = GZRL_MOTOR_CMD_COUNT;
	zassert_equal(gzrl_motor_encode(&motor, &command, &frame), -EINVAL);
	motor = make_motor(GZRL_MOTOR_LKMG, 1);
	command.type = GZRL_MOTOR_CMD_SPEED;
	command.velocity_rad_s = FLT_MAX;
	zassert_equal(gzrl_motor_encode(&motor, &command, &frame), -ERANGE);
	command.type = GZRL_MOTOR_CMD_SINGLE_ANGLE;
	command.position_rad = TAU_F;
	zassert_equal(gzrl_motor_encode(&motor, &command, &frame), -ERANGE);
	command.position_rad = 0;
	command.max_velocity_rad_s = 0;
	zassert_equal(gzrl_motor_encode(&motor, &command, &frame), -ERANGE);
}

ZTEST(motor_core, test_safe_command_and_capability_matrix)
{
	for (int model = 0; model < GZRL_MOTOR_MODEL_COUNT; ++model) {
		struct gzrl_motor motor = make_motor(model, 1);
		struct gzrl_motor_command command;
		struct gzrl_motor_frame frame;

		zassert_ok(gzrl_motor_stop_command(&motor, &command));
		zassert_ok(gzrl_motor_encode(&motor, &command, &frame));
		zassert_equal(command.type, gzrl_motor_is_mit(model) ? GZRL_MOTOR_CMD_DISABLE
								     : GZRL_MOTOR_CMD_EFFORT);
		zassert_true((gzrl_motor_capabilities(model) & GZRL_MOTOR_CAP(command.type)) != 0);
		for (int type = 0; type < GZRL_MOTOR_CMD_COUNT; ++type) {
			if ((gzrl_motor_capabilities(model) & GZRL_MOTOR_CAP(type)) == 0) {
				command.type = type;
				zassert_equal(gzrl_motor_encode(&motor, &command, &frame),
					      -ENOTSUP);
			}
		}
	}
	zassert_equal(gzrl_motor_capabilities(GZRL_MOTOR_MODEL_COUNT), 0);
}

ZTEST_SUITE(motor_core, NULL, NULL, NULL, NULL, NULL);
