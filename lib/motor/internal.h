/*
 * SPDX-FileCopyrightText: 2026 GMaster contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef GZRL_MOTOR_CORE_INTERNAL_H_
#define GZRL_MOTOR_CORE_INTERNAL_H_

#include <gzrl/motor/core.h>

#include <errno.h>
#include <limits.h>
#include <math.h>

#define MOTOR_PI  3.14159265358979323846
#define MOTOR_TAU (2.0 * MOTOR_PI)
#define MOTOR_MOTION_VALID                                                                         \
	(GZRL_MOTOR_VALID_ANGLE | GZRL_MOTOR_VALID_VELOCITY | GZRL_MOTOR_VALID_EFFORT)

struct motor_decoded {
	struct gzrl_motor_feedback feedback;
	float native_position;
	float position_span;
	bool motion;
};

static inline uint16_t motor_be16(const uint8_t *data)
{
	return ((uint16_t)data[0] << 8) | data[1];
}

static inline uint16_t motor_le16(const uint8_t *data)
{
	return ((uint16_t)data[1] << 8) | data[0];
}

static inline int32_t motor_signed16(uint16_t value)
{
	return value <= INT16_MAX ? (int32_t)value : (int32_t)value - 65536;
}

static inline void motor_put_le16(uint16_t value, uint8_t *data)
{
	data[0] = (uint8_t)value;
	data[1] = (uint8_t)(value >> 8);
}

static inline void motor_put_be16(uint16_t value, uint8_t *data)
{
	data[0] = (uint8_t)(value >> 8);
	data[1] = (uint8_t)value;
}

static inline void motor_put_le32(uint32_t value, uint8_t *data)
{
	for (size_t i = 0; i < 4; ++i) {
		data[i] = (uint8_t)(value >> (8 * i));
	}
}

static inline bool motor_in_range(float value, float minimum, float maximum)
{
	return isfinite(value) && value >= minimum && value <= maximum;
}

int motor_grouped_decode(const struct gzrl_motor *motor, const uint8_t *data,
			 struct motor_decoded *decoded);
int motor_grouped_encode(const struct gzrl_motor *motor, const struct gzrl_motor_command *command,
			 uint8_t *data);
int motor_mit_decode(const struct gzrl_motor *motor, const uint8_t *data,
		     struct motor_decoded *decoded);
int motor_mit_encode(const struct gzrl_motor *motor, const struct gzrl_motor_command *command,
		     uint8_t *data);
int motor_lkmg_decode(const struct gzrl_motor *motor, const uint8_t *data,
		      struct motor_decoded *decoded);
int motor_lkmg_encode(const struct gzrl_motor *motor, const struct gzrl_motor_command *command,
		      uint8_t *data);

#endif /* GZRL_MOTOR_CORE_INTERNAL_H_ */
