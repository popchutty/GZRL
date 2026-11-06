/*
 * SPDX-FileCopyrightText: 2026 GMaster contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#include "internal.h"

#include <float.h>
#include <string.h>

struct motor_profile {
	uint16_t maximum_id;
	float maximum_effort;
	uint16_t rx_base;
	uint16_t tx_first;
	uint16_t tx_second;
};

static const struct motor_profile profiles[GZRL_MOTOR_MODEL_COUNT] = {
	[GZRL_MOTOR_GM6020] = {7, 30000.0f, 0x204, 0x1ff, 0x2ff},
	[GZRL_MOTOR_M3508] = {8, 16384.0f, 0x200, 0x200, 0x1ff},
	[GZRL_MOTOR_M2006] = {8, 10000.0f, 0x200, 0x200, 0x1ff},
	[GZRL_MOTOR_DM4310] = {15, 0.0f, 0, 0, 0},
	[GZRL_MOTOR_DM2325] = {15, 0.0f, 0, 0, 0},
	[GZRL_MOTOR_DM_MULTI] = {8, 32767.0f, 0x300, 0x3fe, 0x4fe},
	[GZRL_MOTOR_LKMG] = {32, 2048.0f, 0x140, 0, 0},
};

bool gzrl_motor_is_mit(enum gzrl_motor_model model)
{
	return model == GZRL_MOTOR_DM4310 || model == GZRL_MOTOR_DM2325;
}

enum gzrl_motor_effort_unit gzrl_motor_command_unit(enum gzrl_motor_model model)
{
	return gzrl_motor_is_mit(model) ? GZRL_MOTOR_EFFORT_NEWTON_METRE : GZRL_MOTOR_EFFORT_RAW;
}

uint32_t gzrl_motor_capabilities(enum gzrl_motor_model model)
{
	if ((unsigned int)model >= GZRL_MOTOR_MODEL_COUNT) {
		return 0;
	}
	uint32_t capabilities = GZRL_MOTOR_CAP(GZRL_MOTOR_CMD_EFFORT);

	if (gzrl_motor_is_mit(model)) {
		capabilities |= GZRL_MOTOR_CAP(GZRL_MOTOR_CMD_MIT) |
				GZRL_MOTOR_CAP(GZRL_MOTOR_CMD_ENABLE) |
				GZRL_MOTOR_CAP(GZRL_MOTOR_CMD_DISABLE) |
				GZRL_MOTOR_CAP(GZRL_MOTOR_CMD_CLEAR_ERROR) |
				GZRL_MOTOR_CAP(GZRL_MOTOR_CMD_SET_ZERO);
	} else if (model == GZRL_MOTOR_LKMG) {
		capabilities |= GZRL_MOTOR_CAP(GZRL_MOTOR_CMD_STATUS) |
				GZRL_MOTOR_CAP(GZRL_MOTOR_CMD_BRAKE) |
				GZRL_MOTOR_CAP(GZRL_MOTOR_CMD_SPEED) |
				GZRL_MOTOR_CAP(GZRL_MOTOR_CMD_SINGLE_ANGLE) |
				GZRL_MOTOR_CAP(GZRL_MOTOR_CMD_MULTI_ANGLE);
	}
	return capabilities;
}

static bool positive(float value)
{
	return isfinite(value) && value > 0.0f;
}

int gzrl_motor_init(struct gzrl_motor *motor, const struct gzrl_motor_config *config)
{
	struct gzrl_motor next = {0};

	if (motor == NULL || config == NULL ||
	    (unsigned int)config->model >= GZRL_MOTOR_MODEL_COUNT ||
	    !positive(config->gear_ratio) || !positive(config->effort_limit) ||
	    config->feedback_timeout_us == 0) {
		return -EINVAL;
	}
	const struct motor_profile *profile = &profiles[config->model];

	if (config->id == 0 || config->id > profile->maximum_id) {
		return -EINVAL;
	}
	next.config = *config;
	next.group_slot = -1;
	if (gzrl_motor_is_mit(config->model)) {
		if (config->master_id > 0x7ff || config->encoder_offset != 0 ||
		    !positive(config->pmax) || !positive(config->vmax) || !positive(config->tmax) ||
		    config->pmax > FLT_MAX / 2.0f || config->effort_limit > config->tmax ||
		    !motor_in_range(config->position_wrap_rad, 0.0f, 2.0f * config->pmax)) {
			return -EINVAL;
		}
		next.tx_id = config->id;
		next.rx_id = config->master_id;
	} else {
		if (config->effort_limit > profile->maximum_effort || config->master_id != 0 ||
		    config->position_wrap_rad != 0.0f) {
			return -EINVAL;
		}
		next.rx_id = profile->rx_base + config->id;
		if (config->model == GZRL_MOTOR_LKMG) {
			next.tx_id = next.rx_id;
		} else {
			if (config->encoder_offset >= 8192) {
				return -EINVAL;
			}
			next.tx_id = config->id <= 4 ? profile->tx_first : profile->tx_second;
			next.group_slot = (int8_t)((config->id - 1) % 4);
		}
	}
	next.initialized = true;
	*motor = next;
	return 0;
}

bool gzrl_motor_online(const struct gzrl_motor *motor, uint64_t now_us)
{
	return motor != NULL && motor->initialized &&
	       (motor->feedback.valid & GZRL_MOTOR_VALID_ANGLE) != 0 &&
	       now_us >= motor->feedback.timestamp_us &&
	       now_us - motor->feedback.timestamp_us < motor->config.feedback_timeout_us;
}

bool gzrl_motor_faulted(const struct gzrl_motor *motor)
{
	if (motor == NULL || !motor->initialized) {
		return true;
	}
	if ((motor->feedback.valid & GZRL_MOTOR_VALID_DRIVE_STATE) == 0) {
		return false;
	}
	return gzrl_motor_is_mit(motor->config.model) ? motor->feedback.drive_state > 1
						      : motor->feedback.drive_state != 0;
}

void gzrl_motor_invalidate_position(struct gzrl_motor *motor)
{
	if (motor != NULL && motor->initialized) {
		motor->continuity_lost = true;
		motor->feedback.valid &= ~GZRL_MOTOR_VALID_POSITION;
	}
}

int gzrl_motor_reset_position(struct gzrl_motor *motor)
{
	if (motor == NULL || !motor->initialized ||
	    (motor->feedback.valid & GZRL_MOTOR_VALID_ANGLE) == 0) {
		return -ENODATA;
	}
	motor->feedback.position_rad = 0.0;
	motor->feedback.valid |= GZRL_MOTOR_VALID_POSITION;
	motor->continuity_lost = false;
	return 0;
}

static float wrap_angle(float value)
{
	double angle = fmod((double)value, MOTOR_TAU);

	if (angle < 0.0) {
		angle += MOTOR_TAU;
	}
	float result = (float)angle;

	/* Rounding a value just below 2*pi to float must not produce an endpoint. */
	return result >= (float)MOTOR_TAU ? 0.0f : result;
}

int gzrl_motor_receive(struct gzrl_motor *motor, const struct gzrl_motor_frame *frame,
		       uint64_t timestamp_us)
{
	struct motor_decoded decoded;
	int result;

	if (motor == NULL || !motor->initialized || frame == NULL || frame->length != 8 ||
	    frame->flags != 0 || frame->id > 0x7ff) {
		return -EINVAL;
	}
	if (frame->id != motor->rx_id) {
		return -ENOENT;
	}
	if (timestamp_us < motor->feedback.status_timestamp_us) {
		return -ESTALE;
	}
	decoded = (struct motor_decoded){
		.feedback = motor->feedback,
		.position_span = (float)MOTOR_TAU,
		.motion = true,
	};
	if (gzrl_motor_is_mit(motor->config.model)) {
		result = motor_mit_decode(motor, frame->data, &decoded);
	} else if (motor->config.model == GZRL_MOTOR_LKMG) {
		result = motor_lkmg_decode(motor, frame->data, &decoded);
	} else {
		result = motor_grouped_decode(motor, frame->data, &decoded);
	}
	if (result < 0) {
		return result;
	}
	bool lost = motor->continuity_lost;
	struct gzrl_motor_feedback *next = &decoded.feedback;

	if (decoded.motion) {
		next->velocity_rad_s /= motor->config.gear_ratio;
		next->rotor_angle_rad = wrap_angle(decoded.native_position);
		next->native_position_rad = decoded.native_position;
		if ((motor->feedback.valid & GZRL_MOTOR_VALID_ANGLE) == 0) {
			next->position_rad = 0.0;
		} else if (!gzrl_motor_online(motor, timestamp_us)) {
			lost = true;
		} else if (!lost) {
			double delta = (double)decoded.native_position -
				       (double)motor->last_native_position;
			double span = (double)decoded.position_span;

			if (decoded.position_span == 0.0f) {
				if (fabs(delta) > (double)motor->config.pmax) {
					lost = true;
				}
			} else if (delta > span / 2.0) {
				delta -= span;
			} else if (delta < -span / 2.0) {
				delta += span;
			} else if (fabs(delta) == span / 2.0) {
				lost = true;
			}
			if (!lost) {
				next->position_rad += delta / (double)motor->config.gear_ratio;
			}
		}
		if (!isfinite(next->velocity_rad_s) || !isfinite(next->position_rad)) {
			return -ERANGE;
		}
		next->valid |= MOTOR_MOTION_VALID;
		if (lost) {
			next->valid &= ~GZRL_MOTOR_VALID_POSITION;
		} else {
			next->valid |= GZRL_MOTOR_VALID_POSITION;
		}
		next->sequence++;
		next->timestamp_us = timestamp_us;
	}
	memcpy(next->raw, frame->data, sizeof(next->raw));
	next->status_timestamp_us = timestamp_us;
	next->status_sequence++;
	motor->feedback = *next;
	if (decoded.motion) {
		motor->last_native_position = decoded.native_position;
		motor->continuity_lost = lost;
	}
	return 0;
}

bool gzrl_motor_command_is_motion(const struct gzrl_motor_command *command)
{
	return command != NULL &&
	       (command->type == GZRL_MOTOR_CMD_EFFORT || command->type == GZRL_MOTOR_CMD_MIT ||
		command->type == GZRL_MOTOR_CMD_SPEED ||
		command->type == GZRL_MOTOR_CMD_SINGLE_ANGLE ||
		command->type == GZRL_MOTOR_CMD_MULTI_ANGLE);
}

int gzrl_motor_stop_command(const struct gzrl_motor *motor, struct gzrl_motor_command *command)
{
	if (motor == NULL || !motor->initialized || command == NULL) {
		return -EINVAL;
	}
	*command = (struct gzrl_motor_command){
		.type = gzrl_motor_is_mit(motor->config.model) ? GZRL_MOTOR_CMD_DISABLE
							       : GZRL_MOTOR_CMD_EFFORT,
		.effort_unit = gzrl_motor_command_unit(motor->config.model),
	};
	return 0;
}

int gzrl_motor_encode(const struct gzrl_motor *motor, const struct gzrl_motor_command *command,
		      struct gzrl_motor_frame *frame)
{
	struct gzrl_motor_frame next = {.length = 8};
	int result;

	if (frame == NULL) {
		return -EINVAL;
	}
	memset(frame, 0, sizeof(*frame));
	if (motor == NULL || !motor->initialized || command == NULL ||
	    (unsigned int)command->type >= GZRL_MOTOR_CMD_COUNT) {
		return -EINVAL;
	}
	if ((gzrl_motor_capabilities(motor->config.model) & GZRL_MOTOR_CAP(command->type)) == 0) {
		return -ENOTSUP;
	}
	if ((command->type == GZRL_MOTOR_CMD_EFFORT || command->type == GZRL_MOTOR_CMD_MIT) &&
	    command->effort_unit != gzrl_motor_command_unit(motor->config.model)) {
		return -EINVAL;
	}
	next.id = motor->tx_id;
	if (gzrl_motor_is_mit(motor->config.model)) {
		result = motor_mit_encode(motor, command, next.data);
	} else if (motor->config.model == GZRL_MOTOR_LKMG) {
		result = motor_lkmg_encode(motor, command, next.data);
	} else {
		result = motor_grouped_encode(motor, command, next.data);
	}
	if (result == 0) {
		*frame = next;
	}
	return result;
}

int gzrl_motor_group_add(struct gzrl_motor_group *group, const struct gzrl_motor *motor,
			 const struct gzrl_motor_command *command)
{
	struct gzrl_motor_frame encoded;
	int result;

	if (group == NULL || motor == NULL || !motor->initialized || motor->group_slot < 0) {
		return -EINVAL;
	}
	bool little_endian = motor->config.model == GZRL_MOTOR_DM_MULTI;
	uint8_t mask = (uint8_t)(1U << motor->group_slot);

	if (group->occupied != 0 &&
	    (group->frame.id != motor->tx_id || group->little_endian != little_endian)) {
		return -EINVAL;
	}
	if ((group->occupied & mask) != 0) {
		return -EEXIST;
	}
	result = gzrl_motor_encode(motor, command, &encoded);
	if (result != 0) {
		return result;
	}
	if (group->occupied == 0) {
		memset(&group->frame, 0, sizeof(group->frame));
		group->frame.id = motor->tx_id;
		group->frame.length = 8;
		group->little_endian = little_endian;
	}
	memcpy(&group->frame.data[2 * motor->group_slot], &encoded.data[2 * motor->group_slot], 2);
	group->occupied |= mask;
	return 0;
}
