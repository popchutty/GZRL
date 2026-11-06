/*
 * SPDX-FileCopyrightText: 2026 GMaster contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#include "internal.h"

int motor_grouped_decode(const struct gzrl_motor *motor, const uint8_t *data,
			 struct motor_decoded *decoded)
{
	uint16_t encoder = motor_be16(data);
	struct gzrl_motor_feedback *feedback = &decoded->feedback;

	if (encoder >= 8192) {
		return -EINVAL;
	}
	encoder = (encoder + 8192 - motor->config.encoder_offset) % 8192;
	decoded->native_position = (float)((double)encoder * (MOTOR_TAU / 8192.0));
	feedback->velocity_rad_s =
		(float)((double)motor_signed16(motor_be16(&data[2])) * (MOTOR_TAU / 60.0));
	feedback->effort = (float)motor_signed16(motor_be16(&data[4]));
	feedback->effort_unit = GZRL_MOTOR_EFFORT_RAW;
	if (motor->config.model != GZRL_MOTOR_M2006) {
		feedback->temperature_c = data[6];
		feedback->valid |= GZRL_MOTOR_VALID_TEMPERATURE;
	}
	if (motor->config.model == GZRL_MOTOR_DM_MULTI) {
		feedback->velocity_rad_s /= 100.0f;
		feedback->effort_unit = GZRL_MOTOR_EFFORT_MILLIAMP;
		feedback->drive_state = data[7];
		feedback->valid |= GZRL_MOTOR_VALID_DRIVE_STATE;
	}
	return 0;
}

int motor_grouped_encode(const struct gzrl_motor *motor, const struct gzrl_motor_command *command,
			 uint8_t *data)
{
	if (!motor_in_range(command->effort, -motor->config.effort_limit,
			    motor->config.effort_limit)) {
		return -ERANGE;
	}
	uint16_t value = (uint16_t)(int16_t)command->effort;
	uint8_t *slot = &data[2 * motor->group_slot];

	if (motor->config.model == GZRL_MOTOR_DM_MULTI) {
		motor_put_le16(value, slot);
	} else {
		motor_put_be16(value, slot);
	}
	return 0;
}
