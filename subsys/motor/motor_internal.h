/*
 * SPDX-FileCopyrightText: 2026 GMaster contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef GZRL_MOTOR_INTERNAL_H_
#define GZRL_MOTOR_INTERNAL_H_

#include <gzrl/motor/bus.h>

enum motor_action {
	MOTOR_ACTION_NONE,
	MOTOR_ACTION_MOTION,
	MOTOR_ACTION_START,
	MOTOR_ACTION_SAFE,
	MOTOR_ACTION_OPERATION,
};

struct motor_device_config {
	const struct device *can;
	enum gzrl_motor_model model;
	uint16_t id;
	uint16_t master_id;
};

struct motor_device_data {
	struct gzrl_motor motor;
	struct gzrl_motor_policy policy;
	struct gzrl_motor_bus *bus;
	struct gzrl_motor_command command;
	struct gzrl_motor_command operation;
	enum gzrl_motor_run_state state;
	enum gzrl_motor_fault fault;
	uint64_t state_since_us;
	uint64_t command_us;
	uint64_t operation_us;
	uint64_t last_attempt_us;
	uint64_t safe_since_us;
	uint32_t generation;
	uint32_t start_motion_sequence;
	uint32_t start_status_sequence;
	uint32_t start_rx_serial;
	uint8_t attempts;
	int last_error;
	bool configured;
	bool command_valid;
	bool operation_pending;
	bool safe_pending;
	bool attempted;
	bool startup_transmitted;
	bool startup_brake_confirmed;
	bool startup_feedback_confirmed;
};

struct motor_device_api {
	uint32_t (*capabilities)(const struct device *dev);
};

struct motor_device_data *motor_data(const struct device *dev);
void motor_fault(struct motor_device_data *data, enum gzrl_motor_fault fault, int error);
bool motor_inflight(const struct motor_device_data *data);
size_t motor_devices_on_can(const struct device *can);

#endif /* GZRL_MOTOR_INTERNAL_H_ */
