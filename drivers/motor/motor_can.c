/*
 * SPDX-FileCopyrightText: 2026 GMaster contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT gmaster_can_motor

#include "../../subsys/motor/motor_internal.h"

#include <errno.h>
#include <string.h>
#include <zephyr/devicetree.h>

static uint32_t capabilities(const struct device *dev)
{
	const struct motor_device_config *config = dev->config;

	return gzrl_motor_capabilities(config->model);
}

static const struct motor_device_api driver_api = {.capabilities = capabilities};

struct motor_device_data *motor_data(const struct device *dev)
{
	if (dev == NULL || dev->api != &driver_api || !device_is_ready(dev)) {
		return NULL;
	}
	return dev->data;
}

bool motor_inflight(const struct motor_device_data *data)
{
	if (data->bus == NULL) {
		return false;
	}
	for (size_t i = 0; i < data->bus->slot_count; ++i) {
		const struct gzrl_motor_tx_slot *slot = &data->bus->slots[i];

		if (slot->frame.id == data->motor.tx_id && atomic_get(&slot->completion) != 0) {
			return true;
		}
	}
	return false;
}

void motor_fault(struct motor_device_data *data, enum gzrl_motor_fault fault, int error)
{
	if (data->state == GZRL_MOTOR_STARTING || data->state == GZRL_MOTOR_RUNNING) {
		data->safe_pending = true;
		data->safe_since_us = data->bus->last_process_us;
	}
	if (data->state != GZRL_MOTOR_FAULT) {
		data->generation++;
		data->attempts = 0;
		data->attempted = false;
		data->fault = fault;
	}
	data->state = GZRL_MOTOR_FAULT;
	data->command_valid = false;
	data->operation_pending = false;
	data->last_error = error;
}

int gzrl_motor_device_topology(const struct device *dev, struct gzrl_motor_config *topology,
			       const struct device **can)
{
	if (motor_data(dev) == NULL || topology == NULL || can == NULL) {
		return -EINVAL;
	}
	const struct motor_device_config *config = dev->config;

	memset(topology, 0, sizeof(*topology));
	topology->model = config->model;
	topology->id = config->id;
	topology->master_id = config->master_id;
	*can = config->can;
	return 0;
}

uint32_t gzrl_motor_device_capabilities(const struct device *dev)
{
	return motor_data(dev) == NULL ? 0 : driver_api.capabilities(dev);
}

int gzrl_motor_device_configure(const struct device *dev, const struct gzrl_motor_config *config,
				const struct gzrl_motor_policy *policy)
{
	struct motor_device_data *data = motor_data(dev);
	struct gzrl_motor next;
	int result;

	if (data == NULL || config == NULL || policy == NULL) {
		return -EINVAL;
	}
	if (data->bus != NULL) {
		return -EBUSY;
	}
	const struct motor_device_config *topology = dev->config;

	if (topology->model != config->model || topology->id != config->id ||
	    topology->master_id != config->master_id || policy->command_timeout_us == 0 ||
	    policy->startup_timeout_us == 0 || policy->retry_interval_us == 0 ||
	    policy->tx_timeout_us == 0 || policy->max_attempts == 0 ||
	    policy->retry_interval_us > policy->startup_timeout_us) {
		return -EINVAL;
	}
	result = gzrl_motor_init(&next, config);
	if (result != 0) {
		return result;
	}
	memset(data, 0, sizeof(*data));
	data->motor = next;
	data->policy = *policy;
	data->configured = true;
	return 0;
}

int gzrl_motor_device_start(const struct device *dev, uint64_t now_us)
{
	struct motor_device_data *data = motor_data(dev);

	if (data == NULL || !data->configured || data->bus == NULL) {
		return -ENODEV;
	}
	if (data->state == GZRL_MOTOR_STARTING || data->state == GZRL_MOTOR_RUNNING) {
		return -EALREADY;
	}
	if (motor_inflight(data) || data->safe_pending || data->operation_pending) {
		return -EBUSY;
	}
	if (gzrl_motor_faulted(&data->motor)) {
		return -EIO;
	}
	if (data->bus->time_valid && now_us < data->bus->last_process_us) {
		return -ESTALE;
	}
	data->state = GZRL_MOTOR_STARTING;
	data->fault = GZRL_MOTOR_FAULT_NONE;
	data->last_error = 0;
	data->state_since_us = now_us;
	data->start_motion_sequence = data->motor.feedback.sequence;
	data->start_status_sequence = data->motor.feedback.status_sequence;
	data->start_rx_serial = (uint32_t)atomic_get(&data->bus->rx_serial);
	data->startup_transmitted = false;
	data->startup_brake_confirmed = false;
	data->startup_feedback_confirmed = false;
	data->command_valid = false;
	data->generation++;
	data->attempts = 0;
	data->attempted = false;
	return 0;
}

int gzrl_motor_device_stop(const struct device *dev)
{
	struct motor_device_data *data = motor_data(dev);

	if (data == NULL || !data->configured || data->bus == NULL) {
		return -ENODEV;
	}
	if (data->state == GZRL_MOTOR_STARTING || data->state == GZRL_MOTOR_RUNNING) {
		data->safe_pending = true;
		data->safe_since_us = data->bus->last_process_us;
	}
	data->state = GZRL_MOTOR_STOPPED;
	data->command_valid = false;
	data->operation_pending = false;
	data->generation++;
	data->attempts = 0;
	data->attempted = false;
	return 0;
}

static bool invalid_time(const struct motor_device_data *data, uint64_t now_us)
{
	return data->bus->time_valid && now_us < data->bus->last_process_us;
}

int gzrl_motor_device_command(const struct device *dev, const struct gzrl_motor_command *command,
			      uint64_t now_us)
{
	struct motor_device_data *data = motor_data(dev);
	struct gzrl_motor_frame frame;
	int result;

	if (data == NULL || data->bus == NULL) {
		return -ENODEV;
	}
	if (data->state != GZRL_MOTOR_RUNNING) {
		return -EACCES;
	}
	if (invalid_time(data, now_us) || (data->command_valid && now_us < data->command_us)) {
		motor_fault(data, GZRL_MOTOR_FAULT_TIME, -ESTALE);
		return -ESTALE;
	}
	if (!gzrl_motor_online(&data->motor, now_us) || gzrl_motor_faulted(&data->motor)) {
		motor_fault(data, GZRL_MOTOR_FAULT_FEEDBACK, -ENODATA);
		return -ENODATA;
	}
	if (!gzrl_motor_command_is_motion(command)) {
		motor_fault(data, GZRL_MOTOR_FAULT_COMMAND, -EINVAL);
		return -EINVAL;
	}
	result = gzrl_motor_encode(&data->motor, command, &frame);
	if (result != 0) {
		motor_fault(data, GZRL_MOTOR_FAULT_COMMAND, result);
		return result;
	}
	data->command = *command;
	data->command_us = now_us;
	data->command_valid = true;
	return 0;
}

int gzrl_motor_device_operation(const struct device *dev, const struct gzrl_motor_command *command,
				uint64_t now_us)
{
	struct motor_device_data *data = motor_data(dev);
	struct gzrl_motor_frame frame;
	int result;

	if (data == NULL || data->bus == NULL || command == NULL) {
		return -EINVAL;
	}
	if (data->state == GZRL_MOTOR_RUNNING || data->state == GZRL_MOTOR_STARTING ||
	    data->safe_pending || data->operation_pending || motor_inflight(data)) {
		return -EBUSY;
	}
	if (gzrl_motor_command_is_motion(command) || command->type == GZRL_MOTOR_CMD_ENABLE ||
	    command->type == GZRL_MOTOR_CMD_DISABLE ||
	    (command->type == GZRL_MOTOR_CMD_BRAKE && !command->brake_engaged)) {
		return -EACCES;
	}
	if (invalid_time(data, now_us)) {
		return -ESTALE;
	}
	result = gzrl_motor_encode(&data->motor, command, &frame);
	if (result != 0) {
		return result;
	}
	data->operation = *command;
	data->operation_pending = true;
	data->operation_us = now_us;
	data->generation++;
	data->attempts = 0;
	data->attempted = false;
	return 0;
}

int gzrl_motor_device_reset_position(const struct device *dev, uint64_t now_us)
{
	struct motor_device_data *data = motor_data(dev);

	if (data == NULL || !data->configured || data->bus == NULL) {
		return -ENODEV;
	}
	if (data->state == GZRL_MOTOR_RUNNING || data->state == GZRL_MOTOR_STARTING ||
	    data->safe_pending || data->operation_pending || motor_inflight(data)) {
		return -EBUSY;
	}
	if (invalid_time(data, now_us)) {
		return -ESTALE;
	}
	if (!gzrl_motor_online(&data->motor, now_us)) {
		return -ENODATA;
	}
	return gzrl_motor_reset_position(&data->motor);
}

int gzrl_motor_device_snapshot(const struct device *dev, uint64_t now_us,
			       struct gzrl_motor_snapshot *snapshot)
{
	struct motor_device_data *data = motor_data(dev);

	if (data == NULL || !data->configured || snapshot == NULL) {
		return -EINVAL;
	}
	*snapshot = (struct gzrl_motor_snapshot){
		.feedback = data->motor.feedback,
		.state = data->state,
		.fault = data->fault,
		.online = gzrl_motor_online(&data->motor, now_us),
		.operation_pending =
			data->operation_pending || data->safe_pending || motor_inflight(data),
		.last_error = data->last_error,
	};
	return 0;
}

static int motor_init(const struct device *dev)
{
	const struct motor_device_config *config = dev->config;

	return device_is_ready(config->can) ? 0 : -ENODEV;
}

#define MOTOR_DEFINE(inst)                                                                         \
	BUILD_ASSERT(DT_INST_PROP(inst, motor_id) > 0 && DT_INST_PROP(inst, motor_id) <= 0x7ff);   \
	BUILD_ASSERT(DT_INST_PROP(inst, feedback_id) >= 0 &&                                       \
		     DT_INST_PROP(inst, feedback_id) <= 0x7ff);                                    \
	static struct motor_device_data motor_data_##inst;                                         \
	static const struct motor_device_config motor_config_##inst = {                            \
		.can = DEVICE_DT_GET(DT_INST_PHANDLE(inst, can)),                                  \
		.model = DT_INST_ENUM_IDX(inst, model),                                            \
		.id = DT_INST_PROP(inst, motor_id),                                                \
		.master_id = DT_INST_PROP(inst, feedback_id),                                      \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, motor_init, NULL, &motor_data_##inst, &motor_config_##inst,    \
			      POST_KERNEL, CONFIG_GZRL_MOTOR_INIT_PRIORITY, &driver_api);

DT_INST_FOREACH_STATUS_OKAY(MOTOR_DEFINE)

#define MOTOR_COUNT(inst) count += DEVICE_DT_GET(DT_INST_PHANDLE(inst, can)) == can;

size_t motor_devices_on_can(const struct device *can)
{
	size_t count = 0;

	DT_INST_FOREACH_STATUS_OKAY(MOTOR_COUNT)
	return count;
}
