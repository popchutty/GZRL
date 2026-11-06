/*
 * SPDX-FileCopyrightText: 2026 GMaster contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#include "internal.h"

int motor_lkmg_decode(const struct gzrl_motor *motor, const uint8_t *data,
		      struct motor_decoded *decoded)
{
	struct gzrl_motor_feedback *feedback = &decoded->feedback;

	if (data[0] == 0x8c) {
		if (data[1] > 1) {
			return -EINVAL;
		}
		feedback->brake_engaged = data[1] == 0;
		feedback->valid |= GZRL_MOTOR_VALID_BRAKE;
		decoded->motion = false;
		return 0;
	}
	switch (data[0]) {
	case 0x9c:
	case 0xa0:
	case 0xa1:
	case 0xa2:
	case 0xad:
	case 0xa3:
	case 0xa4:
	case 0xa5:
	case 0xa6:
	case 0xa7:
	case 0xa8:
		break;
	default:
		return -ENOTSUP;
	}
	uint16_t encoder = (uint16_t)(motor_le16(&data[6]) - motor->config.encoder_offset);

	decoded->native_position = (float)((double)encoder * (MOTOR_TAU / 65536.0));
	feedback->velocity_rad_s =
		(float)((double)motor_signed16(motor_le16(&data[4])) * (MOTOR_PI / 180.0));
	feedback->effort = (float)motor_signed16(motor_le16(&data[2]));
	feedback->effort_unit = GZRL_MOTOR_EFFORT_RAW;
	feedback->temperature_c = data[1] < 128 ? data[1] : (int)data[1] - 256;
	feedback->valid |= GZRL_MOTOR_VALID_TEMPERATURE;
	return 0;
}

int motor_lkmg_encode(const struct gzrl_motor *motor, const struct gzrl_motor_command *command,
		      uint8_t *data)
{
	double ratio = (double)motor->config.gear_ratio;
	double value;
	double speed;

	switch (command->type) {
	case GZRL_MOTOR_CMD_STATUS:
		data[0] = 0x9c;
		return 0;
	case GZRL_MOTOR_CMD_BRAKE:
		data[0] = 0x8c;
		data[1] = command->brake_engaged ? 0 : 1;
		return 0;
	case GZRL_MOTOR_CMD_EFFORT:
		if (!motor_in_range(command->effort, -motor->config.effort_limit,
				    motor->config.effort_limit)) {
			return -ERANGE;
		}
		data[0] = 0xa1;
		motor_put_le16((uint16_t)(int16_t)command->effort, &data[4]);
		return 0;
	case GZRL_MOTOR_CMD_SPEED:
		value = (double)command->velocity_rad_s * ratio * (18000.0 / MOTOR_PI);
		data[0] = 0xa2;
		break;
	case GZRL_MOTOR_CMD_SINGLE_ANGLE:
	case GZRL_MOTOR_CMD_MULTI_ANGLE:
		if (command->type == GZRL_MOTOR_CMD_SINGLE_ANGLE &&
		    (!motor_in_range(command->position_rad, 0, (float)MOTOR_TAU) ||
		     command->position_rad == (float)MOTOR_TAU)) {
			return -ERANGE;
		}
		value = (double)command->position_rad * (18000.0 / MOTOR_PI);
		if (command->type == GZRL_MOTOR_CMD_MULTI_ANGLE) {
			value *= ratio;
		}
		speed = (double)command->max_velocity_rad_s * ratio * (180.0 / MOTOR_PI);
		if (!isfinite(speed) || speed < 1.0 || speed > UINT16_MAX) {
			return -ERANGE;
		}
		data[0] = command->type == GZRL_MOTOR_CMD_SINGLE_ANGLE ? 0xa6 : 0xa4;
		data[1] = command->type == GZRL_MOTOR_CMD_SINGLE_ANGLE && !command->clockwise;
		motor_put_le16((uint16_t)speed, &data[2]);
		break;
	default:
		return -ENOTSUP;
	}
	if (!isfinite(value) || value < INT32_MIN || value > INT32_MAX) {
		return -ERANGE;
	}
	motor_put_le32((uint32_t)(int32_t)value, &data[4]);
	return 0;
}
