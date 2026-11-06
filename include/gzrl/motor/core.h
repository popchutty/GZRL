/*
 * SPDX-FileCopyrightText: 2026 GMaster contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef GZRL_MOTOR_CORE_H_
#define GZRL_MOTOR_CORE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Supported motor/firmware profiles. Numeric values are stable DT model indices. */
enum gzrl_motor_model {
	GZRL_MOTOR_GM6020,
	GZRL_MOTOR_M3508,
	GZRL_MOTOR_M2006,
	GZRL_MOTOR_DM4310,
	GZRL_MOTOR_DM2325,
	GZRL_MOTOR_DM_MULTI,
	GZRL_MOTOR_LKMG,
	GZRL_MOTOR_MODEL_COUNT,
};

/** @brief Units of a native drive effort value, never scaled by a gearbox ratio. */
enum gzrl_motor_effort_unit {
	GZRL_MOTOR_EFFORT_RAW,
	GZRL_MOTOR_EFFORT_MILLIAMP,
	GZRL_MOTOR_EFFORT_NEWTON_METRE,
};

/** @brief Portable frame flags. Motor codecs accept only flags == 0. */
enum gzrl_motor_frame_flag {
	GZRL_MOTOR_FRAME_EXTENDED = 1U << 0,
	GZRL_MOTOR_FRAME_REMOTE = 1U << 1,
	GZRL_MOTOR_FRAME_FD = 1U << 2,
};

/** @brief Byte-wise classic CAN frame; no native driver or wire-layout dependency. */
struct gzrl_motor_frame {
	uint32_t id;
	uint8_t flags;
	uint8_t length;
	uint8_t data[8];
};

/** @brief Initialization parameters. All limits are explicit application choices. */
struct gzrl_motor_config {
	enum gzrl_motor_model model;
	uint16_t id;
	uint16_t master_id;      /**< DM MIT feedback CAN identifier; zero is permitted. */
	uint16_t encoder_offset; /**< Counts; ignored offsets are rejected. */
	float gear_ratio;        /**< Motor revolutions per output revolution, strictly positive. */
	float effort_limit;      /**< Symmetric native command limit, in command_unit() units. */
	float pmax;              /**< DM MIT firmware position limit, motor-side rad. */
	float vmax;              /**< DM MIT firmware velocity limit, motor-side rad/s. */
	float tmax;              /**< DM MIT firmware torque limit, N m. */
	float position_wrap_rad; /**< DM MIT verified native wrap period; 0 assumes no wrap. */
	uint64_t feedback_timeout_us;
};

/** @brief Feedback field validity; unavailable quantities are not fabricated. */
enum gzrl_motor_feedback_valid {
	GZRL_MOTOR_VALID_ANGLE = 1U << 0,
	GZRL_MOTOR_VALID_POSITION = 1U << 1,
	GZRL_MOTOR_VALID_VELOCITY = 1U << 2,
	GZRL_MOTOR_VALID_EFFORT = 1U << 3,
	GZRL_MOTOR_VALID_TEMPERATURE = 1U << 4,
	GZRL_MOTOR_VALID_MOS_TEMPERATURE = 1U << 5,
	GZRL_MOTOR_VALID_DRIVE_STATE = 1U << 6,
	GZRL_MOTOR_VALID_BRAKE = 1U << 7,
};

/** @brief Latest decoded feedback; timestamp_us always refers to motion feedback. */
struct gzrl_motor_feedback {
	uint32_t valid;
	uint32_t sequence;
	uint32_t status_sequence;
	uint64_t timestamp_us;
	uint64_t status_timestamp_us;
	float rotor_angle_rad;     /**< Offset-adjusted motor angle, [0, 2*pi). */
	float native_position_rad; /**< Native reported position before modulo/output conversion. */
	double position_rad;       /**< Output displacement since first sample or software reset. */
	float velocity_rad_s;      /**< Output shaft velocity. */
	float effort;
	enum gzrl_motor_effort_unit effort_unit;
	float temperature_c;
	float mos_temperature_c;
	uint8_t drive_state; /**< DM MIT state nibble or DM multi fault byte. */
	bool brake_engaged;
	uint8_t raw[8]; /**< Last accepted packet, including status-only acknowledgements. */
};

/** @brief Native command operations; availability is exposed by capabilities(). */
enum gzrl_motor_command_type {
	GZRL_MOTOR_CMD_EFFORT,
	GZRL_MOTOR_CMD_MIT,
	GZRL_MOTOR_CMD_ENABLE,
	GZRL_MOTOR_CMD_DISABLE,
	GZRL_MOTOR_CMD_CLEAR_ERROR,
	GZRL_MOTOR_CMD_SET_ZERO,
	GZRL_MOTOR_CMD_STATUS,
	GZRL_MOTOR_CMD_BRAKE,
	GZRL_MOTOR_CMD_SPEED,
	GZRL_MOTOR_CMD_SINGLE_ANGLE,
	GZRL_MOTOR_CMD_MULTI_ANGLE,
	GZRL_MOTOR_CMD_COUNT,
};

#define GZRL_MOTOR_CAP(command) (UINT32_C(1) << (command))

/** @brief Command payload. Only fields relevant to type are consumed. */
struct gzrl_motor_command {
	enum gzrl_motor_command_type type;
	enum gzrl_motor_effort_unit effort_unit;
	float effort;
	float position_rad;       /**< MIT/single-angle: motor side; MG multi-angle: output side. */
	float velocity_rad_s;     /**< MIT: motor side; MG speed: output side. */
	float kp;                 /**< MIT drive-side gain, [0, 500]. */
	float kd;                 /**< MIT drive-side gain, [0, 5]. */
	float max_velocity_rad_s; /**< MG position commands: positive output-side speed limit. */
	bool clockwise;           /**< MG single-angle direction. */
	bool brake_engaged;
};

/** @brief Caller-owned single-thread state. Fields are read-only outside core operations. */
struct gzrl_motor {
	struct gzrl_motor_config config;
	struct gzrl_motor_feedback feedback;
	uint16_t tx_id;
	uint16_t rx_id;
	int8_t group_slot; /**< -1 for individual frames; otherwise slot index [0, 3]. */
	float last_native_position;
	bool initialized;
	bool continuity_lost;
};

/** @brief Validate and initialize an object; on error its previous contents are unchanged. */
int gzrl_motor_init(struct gzrl_motor *motor, const struct gzrl_motor_config *config);

/** @brief Get supported command bits; returns zero for an invalid model. */
uint32_t gzrl_motor_capabilities(enum gzrl_motor_model model);

/** @brief Get effort command units (RAW for non-MIT profiles, N m for DM MIT). */
enum gzrl_motor_effort_unit gzrl_motor_command_unit(enum gzrl_motor_model model);

/** @brief Identify the supported DM MIT family. */
bool gzrl_motor_is_mit(enum gzrl_motor_model model);

/**
 * @brief Decode a frame at its receive time, never at queue-drain time.
 *
 * Returns 0 on success, -ENOENT for a different recipient, -ESTALE for backwards time,
 * -ENOTSUP for an unsupported MG response, or -EINVAL/-ERANGE for malformed data.
 * Rejected packets never change state. Brake acknowledgements do not refresh motion age.
 */
int gzrl_motor_receive(struct gzrl_motor *motor, const struct gzrl_motor_frame *frame,
		       uint64_t timestamp_us);

/** @brief Pure online query: fresh motion feedback is required; timeout equality is offline. */
bool gzrl_motor_online(const struct gzrl_motor *motor, uint64_t now_us);

/** @brief Report decoded drive errors; invalid objects are considered faulted. */
bool gzrl_motor_faulted(const struct gzrl_motor *motor);

/** @brief Mark accumulated position invalid after packet loss, without altering its value. */
void gzrl_motor_invalidate_position(struct gzrl_motor *motor);

/** @brief Reset displacement origin using the last sample; fails if no motion sample exists. */
int gzrl_motor_reset_position(struct gzrl_motor *motor);

/** @brief Distinguish motion commands from lifecycle/status operations. */
bool gzrl_motor_command_is_motion(const struct gzrl_motor_command *command);

/**
 * @brief Encode a command without I/O, automatic enabling, or state mutation.
 *
 * The output frame is cleared before validation so failure cannot leave a sendable frame.
 * Grouped profiles encode their own slot only; use group_add() to combine all peers.
 */
int gzrl_motor_encode(const struct gzrl_motor *motor, const struct gzrl_motor_command *command,
		      struct gzrl_motor_frame *frame);

/** @brief Build a native safe-output command: DM MIT disable, otherwise zero effort. */
int gzrl_motor_stop_command(const struct gzrl_motor *motor, struct gzrl_motor_command *command);

/** @brief A zero-initialized group has all unpopulated slots at zero output. */
struct gzrl_motor_group {
	struct gzrl_motor_frame frame;
	uint8_t occupied;
	bool little_endian;
};

/** @brief Combine one slot, rejecting duplicate slots, unlike IDs or unlike byte ordering. */
int gzrl_motor_group_add(struct gzrl_motor_group *group, const struct gzrl_motor *motor,
			 const struct gzrl_motor_command *command);

#ifdef __cplusplus
}
#endif
#endif /* GZRL_MOTOR_CORE_H_ */
