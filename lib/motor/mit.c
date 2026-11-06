/*
 * SPDX-FileCopyrightText: 2026 GMaster contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#include "internal.h"

#include <string.h>

static float decode_unsigned(uint32_t value, float limit, uint32_t maximum)
{
	return (float)(((double)value / maximum * 2.0 - 1.0) * (double)limit);
}

static uint16_t encode_unsigned(float value, float minimum, float maximum, uint32_t mask)
{
	return (uint16_t)(((double)value - (double)minimum) / ((double)maximum - (double)minimum) *
			  mask);
}

int motor_mit_decode(const struct gzrl_motor *motor, const uint8_t *data,
		     struct motor_decoded *decoded)
{
	struct gzrl_motor_feedback *feedback = &decoded->feedback;
	uint8_t state = data[0] >> 4;

	if ((data[0] & 0x0f) != motor->config.id) {
		return -ENOENT;
	}
	if (state != 0 && state != 1 && (state < 3 || state > 5) && (state < 8 || state > 14)) {
		return -EINVAL;
	}
	decoded->native_position = decode_unsigned(motor_be16(&data[1]), motor->config.pmax, 65535);
	decoded->position_span = motor->config.position_wrap_rad;
	feedback->velocity_rad_s = decode_unsigned(((uint16_t)data[3] << 4) | (data[4] >> 4),
						   motor->config.vmax, 4095);
	feedback->effort = decode_unsigned(((uint16_t)(data[4] & 0x0f) << 8) | data[5],
					   motor->config.tmax, 4095);
	feedback->effort_unit = GZRL_MOTOR_EFFORT_NEWTON_METRE;
	feedback->temperature_c = data[7];
	feedback->mos_temperature_c = data[6];
	feedback->drive_state = state;
	feedback->valid |= GZRL_MOTOR_VALID_TEMPERATURE | GZRL_MOTOR_VALID_MOS_TEMPERATURE |
			   GZRL_MOTOR_VALID_DRIVE_STATE;
	return 0;
}

int motor_mit_encode(const struct gzrl_motor *motor, const struct gzrl_motor_command *command,
		     uint8_t *data)
{
	const struct gzrl_motor_config *config = &motor->config;
	float position = 0.0f;
	float velocity = 0.0f;
	float kp = 0.0f;
	float kd = 0.0f;
	uint8_t special;

	switch (command->type) {
	case GZRL_MOTOR_CMD_ENABLE:
		special = 0xfc;
		break;
	case GZRL_MOTOR_CMD_DISABLE:
		special = 0xfd;
		break;
	case GZRL_MOTOR_CMD_CLEAR_ERROR:
		special = 0xfb;
		break;
	case GZRL_MOTOR_CMD_SET_ZERO:
		special = 0xfe;
		break;
	default:
		special = 0;
		break;
	}
	if (special != 0) {
		memset(data, 0xff, 8);
		data[7] = special;
		return 0;
	}
	if (command->type == GZRL_MOTOR_CMD_MIT) {
		position = command->position_rad;
		velocity = command->velocity_rad_s;
		kp = command->kp;
		kd = command->kd;
	}
	if (!motor_in_range(position, -config->pmax, config->pmax) ||
	    !motor_in_range(velocity, -config->vmax, config->vmax) ||
	    !motor_in_range(kp, 0.0f, 500.0f) || !motor_in_range(kd, 0.0f, 5.0f) ||
	    !motor_in_range(command->effort, -config->effort_limit, config->effort_limit)) {
		return -ERANGE;
	}
	uint16_t p = encode_unsigned(position, -config->pmax, config->pmax, 65535);
	uint16_t v = encode_unsigned(velocity, -config->vmax, config->vmax, 4095);
	uint16_t k_p = encode_unsigned(kp, 0.0f, 500.0f, 4095);
	uint16_t k_d = encode_unsigned(kd, 0.0f, 5.0f, 4095);
	uint16_t t = encode_unsigned(command->effort, -config->tmax, config->tmax, 4095);

	data[0] = (uint8_t)(p >> 8);
	data[1] = (uint8_t)p;
	data[2] = (uint8_t)(v >> 4);
	data[3] = (uint8_t)((v << 4) | (k_p >> 8));
	data[4] = (uint8_t)k_p;
	data[5] = (uint8_t)(k_d >> 4);
	data[6] = (uint8_t)((k_d << 4) | (t >> 8));
	data[7] = (uint8_t)t;
	if (data[7] >= 0xfb && data[7] <= 0xfe) {
		bool reserved = true;

		for (size_t i = 0; i < 7; ++i) {
			reserved = reserved && data[i] == 0xff;
		}
		if (reserved) {
			return -ERANGE;
		}
	}
	return 0;
}
