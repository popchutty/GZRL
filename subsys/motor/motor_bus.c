/*
 * SPDX-FileCopyrightText: 2026 GMaster contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#include "motor_internal.h"

#include <errno.h>
#include <string.h>

uint64_t gzrl_motor_bus_now_us(void)
{
	return k_ticks_to_us_floor64(k_uptime_ticks());
}

static void receive_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{
	struct gzrl_motor_bus *bus = user_data;
	struct gzrl_motor_rx_event event = {0};

	ARG_UNUSED(dev);
	if (!atomic_get(&bus->accepting)) {
		return;
	}
	event.timestamp_us = gzrl_motor_bus_now_us();
	event.serial = (uint32_t)atomic_inc(&bus->rx_serial) + 1U;
	event.frame.id = frame->id;
	event.frame.flags = frame->flags == 0 ? 0 : GZRL_MOTOR_FRAME_EXTENDED;
	event.frame.length = frame->dlc;
	memcpy(event.frame.data, frame->data, MIN(frame->dlc, sizeof(event.frame.data)));
	if (k_msgq_put(&bus->rx_queue, &event, K_NO_WAIT) != 0) {
		atomic_inc(&bus->dropped);
	}
}

static void transmit_callback(const struct device *dev, int error, void *user_data)
{
	struct gzrl_motor_tx_slot *slot = user_data;

	ARG_UNUSED(dev);
	atomic_set(&slot->error, error);
	atomic_set(&slot->completion, 2);
}

static void fault_all(struct gzrl_motor_bus *bus, enum gzrl_motor_fault reason, int error,
		      bool invalidate)
{
	for (size_t i = 0; i < bus->count; ++i) {
		struct motor_device_data *data = motor_data(bus->devices[i]);

		if (invalidate) {
			gzrl_motor_invalidate_position(&data->motor);
		}
		motor_fault(data, reason, error);
	}
}

static bool incompatible(const struct gzrl_motor *left, const struct gzrl_motor *right)
{
	if (left->rx_id == right->rx_id &&
	    !(gzrl_motor_is_mit(left->config.model) && gzrl_motor_is_mit(right->config.model) &&
	      left->config.id != right->config.id)) {
		return true;
	}
	/* Commands must not alias another endpoint's feedback or individual command ID. */
	if (left->tx_id == right->rx_id || right->tx_id == left->rx_id) {
		return true;
	}
	if (left->tx_id != right->tx_id) {
		return false;
	}
	return left->group_slot < 0 || right->group_slot < 0 ||
	       left->group_slot == right->group_slot ||
	       (left->config.model == GZRL_MOTOR_DM_MULTI) !=
		       (right->config.model == GZRL_MOTOR_DM_MULTI);
}

static void remove_filters(struct gzrl_motor_bus *bus)
{
	atomic_clear(&bus->accepting);
	for (size_t i = 0; i < bus->filter_count; ++i) {
		can_remove_rx_filter(bus->can, bus->filters[i]);
	}
	bus->filter_count = 0;
}

int gzrl_motor_bus_attach(struct gzrl_motor_bus *bus, const struct device *can,
			  const struct device *const *devices, size_t count)
{
	enum can_state state;
	int result;

	if (bus == NULL || can == NULL || devices == NULL || count == 0 ||
	    count > CONFIG_GZRL_MOTOR_MAX_DEVICES || !device_is_ready(can)) {
		return -EINVAL;
	}
	if (bus->attached) {
		return -EALREADY;
	}
	result = can_get_state(can, &state, NULL);
	if (result != 0) {
		return result;
	}
	if (state != CAN_STATE_STOPPED) {
		return -EBUSY;
	}
	if (motor_devices_on_can(can) != count) {
		return -EINVAL;
	}
	for (size_t i = 0; i < count; ++i) {
		struct motor_device_data *data = motor_data(devices[i]);

		if (data == NULL || !data->configured) {
			return -ENODEV;
		}
		const struct motor_device_config *config = devices[i]->config;

		if (config->can != can || data->bus != NULL) {
			return -EBUSY;
		}
		for (size_t j = 0; j < i; ++j) {
			if (incompatible(&data->motor, &motor_data(devices[j])->motor)) {
				return -EADDRINUSE;
			}
		}
	}
	memset(bus, 0, sizeof(*bus));
	bus->can = can;
	bus->count = count;
	memcpy(bus->devices, devices, count * sizeof(*devices));
	k_msgq_init(&bus->rx_queue, bus->rx_storage, sizeof(struct gzrl_motor_rx_event),
		    CONFIG_GZRL_MOTOR_RX_DEPTH);
	for (size_t i = 0; i < count; ++i) {
		struct gzrl_motor *motor = &motor_data(devices[i])->motor;
		bool duplicate = false;
		size_t slot;

		for (slot = 0; slot < bus->slot_count; ++slot) {
			if (bus->slots[slot].frame.id == motor->tx_id) {
				break;
			}
		}
		if (slot == bus->slot_count) {
			bus->slots[slot].bus = bus;
			bus->slots[slot].frame.id = motor->tx_id;
			bus->slots[slot].frame.dlc = 8;
			bus->slot_count++;
		}
		bus->slots[slot].members |= BIT(i);
		for (size_t j = 0; j < i; ++j) {
			duplicate |= motor_data(devices[j])->motor.rx_id == motor->rx_id;
		}
		if (duplicate) {
			continue;
		}
		struct can_filter filter = {.id = motor->rx_id, .mask = CAN_STD_ID_MASK};

		result = can_add_rx_filter(can, receive_callback, bus, &filter);
		if (result < 0) {
			remove_filters(bus);
			bus->count = 0;
			return result;
		}
		bus->filters[bus->filter_count++] = result;
	}
	for (size_t i = 0; i < count; ++i) {
		motor_data(devices[i])->bus = bus;
	}
	bus->attached = true;
	atomic_set(&bus->accepting, 1);
	return 0;
}

static int collect_completions(struct gzrl_motor_bus *bus, uint64_t now_us)
{
	int first_error = 0;

	for (size_t i = 0; i < bus->slot_count; ++i) {
		struct gzrl_motor_tx_slot *slot = &bus->slots[i];
		atomic_val_t completion = atomic_get(&slot->completion);

		if (completion == 0) {
			continue;
		}
		if (completion == 1) {
			for (size_t j = 0; j < bus->count; ++j) {
				if ((slot->members & BIT(j)) == 0) {
					continue;
				}
				struct motor_device_data *data = motor_data(bus->devices[j]);

				if (now_us >= slot->submitted_us &&
				    now_us - slot->submitted_us >= data->policy.tx_timeout_us) {
					motor_fault(data, GZRL_MOTOR_FAULT_TX, -ETIMEDOUT);
					if (!slot->stalled) {
						bus->stats.stalled++;
						slot->stalled = true;
					}
					first_error = -ETIMEDOUT;
				}
			}
			continue;
		}
		int error = (int)atomic_get(&slot->error);

		if (error == 0) {
			bus->stats.completed++;
		} else {
			bus->stats.failed++;
			first_error = error;
		}
		for (size_t j = 0; j < bus->count; ++j) {
			if ((slot->members & BIT(j)) == 0) {
				continue;
			}
			struct motor_device_data *data = motor_data(bus->devices[j]);

			if (error != 0) {
				motor_fault(data, GZRL_MOTOR_FAULT_TX, error);
			} else if (slot->generations[j] == data->generation) {
				if (slot->actions[j] == MOTOR_ACTION_SAFE) {
					data->safe_pending = false;
				} else if (slot->actions[j] == MOTOR_ACTION_START) {
					data->startup_transmitted = true;
				} else if (slot->actions[j] == MOTOR_ACTION_OPERATION) {
					data->operation_pending = false;
					if (data->operation.type == GZRL_MOTOR_CMD_SET_ZERO) {
						gzrl_motor_invalidate_position(&data->motor);
					}
				}
			}
		}
		atomic_clear(&slot->completion);
		slot->stalled = false;
	}
	return first_error;
}

static bool start_confirmed(const struct motor_device_data *data, uint64_t now_us)
{
	const struct gzrl_motor *motor = &data->motor;
	const struct gzrl_motor_feedback *feedback = &motor->feedback;

	if (!gzrl_motor_online(motor, now_us) || feedback->timestamp_us < data->state_since_us ||
	    feedback->sequence == data->start_motion_sequence ||
	    !data->startup_feedback_confirmed) {
		return false;
	}
	if (gzrl_motor_is_mit(motor->config.model)) {
		return data->startup_transmitted && feedback->drive_state == 1;
	}
	if (motor->config.model == GZRL_MOTOR_LKMG) {
		return data->startup_transmitted && data->startup_brake_confirmed &&
		       (feedback->valid & GZRL_MOTOR_VALID_BRAKE) && !feedback->brake_engaged;
	}
	return true;
}

static void update_state(struct motor_device_data *data, uint64_t now_us)
{
	if (data->safe_pending && now_us >= data->safe_since_us &&
	    now_us - data->safe_since_us >= data->policy.startup_timeout_us) {
		data->safe_pending = false;
		motor_fault(data, GZRL_MOTOR_FAULT_TX, -ETIMEDOUT);
	}
	if (data->state == GZRL_MOTOR_STARTING || data->state == GZRL_MOTOR_RUNNING) {
		if (now_us < data->state_since_us) {
			motor_fault(data, GZRL_MOTOR_FAULT_TIME, -ESTALE);
			return;
		}
		if (gzrl_motor_faulted(&data->motor)) {
			motor_fault(data, GZRL_MOTOR_FAULT_DRIVE, -EIO);
			return;
		}
	}
	if (data->state == GZRL_MOTOR_STARTING) {
		if (start_confirmed(data, now_us)) {
			data->state = GZRL_MOTOR_RUNNING;
			data->state_since_us = now_us;
			data->attempts = 0;
			data->attempted = false;
		} else if (now_us - data->state_since_us >= data->policy.startup_timeout_us ||
			   (data->attempts >= data->policy.max_attempts && data->attempted &&
			    now_us - data->last_attempt_us >= data->policy.retry_interval_us)) {
			motor_fault(data, GZRL_MOTOR_FAULT_START, -ETIMEDOUT);
		}
	} else if (data->state == GZRL_MOTOR_RUNNING) {
		if (!gzrl_motor_online(&data->motor, now_us)) {
			motor_fault(data, GZRL_MOTOR_FAULT_FEEDBACK, -ETIMEDOUT);
		} else if ((gzrl_motor_is_mit(data->motor.config.model) &&
			    data->motor.feedback.drive_state != 1) ||
			   (data->motor.config.model == GZRL_MOTOR_LKMG &&
			    data->motor.feedback.brake_engaged)) {
			motor_fault(data, GZRL_MOTOR_FAULT_DRIVE, -EIO);
		} else {
			uint64_t since =
				data->command_valid ? data->command_us : data->state_since_us;

			if (now_us < since || now_us - since >= data->policy.command_timeout_us) {
				motor_fault(data, GZRL_MOTOR_FAULT_COMMAND, -ETIMEDOUT);
			}
		}
	}
	if (data->operation_pending &&
	    (now_us < data->operation_us ||
	     now_us - data->operation_us >= data->policy.startup_timeout_us)) {
		data->operation_pending = false;
		data->last_error = -ETIMEDOUT;
		data->generation++;
	}
}

static int check_time(struct gzrl_motor_bus *bus, uint64_t now_us)
{
	if (bus->time_valid && now_us < bus->last_process_us) {
		fault_all(bus, GZRL_MOTOR_FAULT_TIME, -ESTALE, true);
		return -ESTALE;
	}
	bus->last_process_us = now_us;
	bus->time_valid = true;
	return 0;
}

static int check_controller(struct gzrl_motor_bus *bus)
{
	enum can_state state;
	int result = can_get_state(bus->can, &state, NULL);

	if (result != 0 || state == CAN_STATE_BUS_OFF || state == CAN_STATE_STOPPED) {
		int error = result != 0                  ? result
			    : state == CAN_STATE_BUS_OFF ? -ENETUNREACH
							 : -ENETDOWN;

		fault_all(bus, GZRL_MOTOR_FAULT_BUS, error, true);
		return error;
	}
	return 0;
}

int gzrl_motor_bus_process(struct gzrl_motor_bus *bus, uint64_t now_us, size_t rx_budget)
{
	struct gzrl_motor_rx_event event;
	int result;
	int first_error;

	if (bus == NULL || !bus->attached) {
		return -ENODEV;
	}
	result = check_time(bus, now_us);
	if (result != 0) {
		return result;
	}
	first_error = collect_completions(bus, now_us);
	atomic_val_t lost = atomic_set(&bus->dropped, 0);

	if (lost != 0) {
		bus->stats.dropped += (uint32_t)lost;
		fault_all(bus, GZRL_MOTOR_FAULT_RX_LOSS, -EOVERFLOW, true);
		first_error = -EOVERFLOW;
	}
	for (size_t n = 0; n < rx_budget; ++n) {
		if (k_msgq_peek(&bus->rx_queue, &event) != 0 || event.timestamp_us > now_us) {
			break;
		}
		if (k_msgq_get(&bus->rx_queue, &event, K_NO_WAIT) != 0) {
			break;
		}
		bool accepted = false;

		bus->stats.received++;
		for (size_t i = 0; i < bus->count; ++i) {
			struct motor_device_data *data = motor_data(bus->devices[i]);

			if (data->motor.rx_id != event.frame.id) {
				continue;
			}
			uint32_t previous_sequence = data->motor.feedback.sequence;
			uint64_t previous_timestamp = data->motor.feedback.timestamp_us;
			bool had_motion =
				(data->motor.feedback.valid & GZRL_MOTOR_VALID_ANGLE) != 0;
			uint32_t serial_delta = event.serial - data->start_rx_serial;
			bool after_start = serial_delta != 0 && serial_delta < UINT32_C(0x80000000);

			result = gzrl_motor_receive(&data->motor, &event.frame, event.timestamp_us);
			accepted |= result == 0;
			bool new_motion =
				result == 0 && data->motor.feedback.sequence != previous_sequence;

			/* A later healthy packet must never erase an observed fault or feedback
			 * gap. */
			if (result == 0 &&
			    (data->state == GZRL_MOTOR_STARTING ||
			     data->state == GZRL_MOTOR_RUNNING) &&
			    gzrl_motor_faulted(&data->motor)) {
				motor_fault(data, GZRL_MOTOR_FAULT_DRIVE, -EIO);
			}
			if (result == 0 && data->state == GZRL_MOTOR_RUNNING) {
				if (new_motion && had_motion &&
				    event.timestamp_us >= previous_timestamp &&
				    event.timestamp_us - previous_timestamp >=
					    data->motor.config.feedback_timeout_us) {
					motor_fault(data, GZRL_MOTOR_FAULT_FEEDBACK, -ETIMEDOUT);
				} else if ((gzrl_motor_is_mit(data->motor.config.model) &&
					    data->motor.feedback.drive_state != 1) ||
					   (data->motor.config.model == GZRL_MOTOR_LKMG &&
					    data->motor.feedback.brake_engaged)) {
					motor_fault(data, GZRL_MOTOR_FAULT_DRIVE, -EIO);
				}
			}
			if (result == 0 && data->state == GZRL_MOTOR_STARTING && after_start &&
			    new_motion) {
				data->startup_feedback_confirmed = true;
			}
			if (result == 0 && after_start && data->state == GZRL_MOTOR_STARTING &&
			    data->attempted && data->motor.config.model == GZRL_MOTOR_LKMG &&
			    event.frame.data[0] == 0x8c &&
			    event.timestamp_us >= data->state_since_us &&
			    data->motor.feedback.status_sequence != data->start_status_sequence &&
			    !data->motor.feedback.brake_engaged) {
				data->startup_brake_confirmed = true;
				data->startup_feedback_confirmed = false;
				data->start_motion_sequence = data->motor.feedback.sequence;
			}
		}
		if (!accepted) {
			bus->stats.rejected++;
		}
	}
	result = check_controller(bus);
	if (result != 0) {
		first_error = result;
	}
	for (size_t i = 0; i < bus->count; ++i) {
		update_state(motor_data(bus->devices[i]), now_us);
	}
	return first_error;
}

static enum motor_action select_command(struct motor_device_data *data, uint64_t now_us,
					struct gzrl_motor_command *command)
{
	memset(command, 0, sizeof(*command));
	command->effort_unit = gzrl_motor_command_unit(data->motor.config.model);
	if (data->safe_pending) {
		if (data->attempts >= data->policy.max_attempts) {
			data->safe_pending = false;
			motor_fault(data, GZRL_MOTOR_FAULT_TX, -ETIMEDOUT);
			return MOTOR_ACTION_NONE;
		}
		if (data->attempted &&
		    now_us - data->last_attempt_us < data->policy.retry_interval_us) {
			return MOTOR_ACTION_NONE;
		}
		gzrl_motor_stop_command(&data->motor, command);
		return MOTOR_ACTION_SAFE;
	}
	if (data->state == GZRL_MOTOR_RUNNING) {
		if (data->command_valid) {
			*command = data->command;
			return MOTOR_ACTION_MOTION;
		}
		return MOTOR_ACTION_NONE;
	}
	if (data->attempted && now_us - data->last_attempt_us < data->policy.retry_interval_us) {
		return MOTOR_ACTION_NONE;
	}
	if (data->attempts >= data->policy.max_attempts) {
		if (data->operation_pending) {
			data->operation_pending = false;
			data->last_error = -ETIMEDOUT;
		}
		return MOTOR_ACTION_NONE;
	}
	if (data->operation_pending) {
		*command = data->operation;
		return MOTOR_ACTION_OPERATION;
	}
	if (data->state == GZRL_MOTOR_STARTING) {
		if (gzrl_motor_is_mit(data->motor.config.model)) {
			command->type = GZRL_MOTOR_CMD_ENABLE;
		} else if (data->motor.config.model == GZRL_MOTOR_LKMG) {
			if (data->startup_brake_confirmed) {
				command->type = GZRL_MOTOR_CMD_STATUS;
			} else {
				command->type = GZRL_MOTOR_CMD_BRAKE;
				command->brake_engaged = false;
			}
		} else {
			return MOTOR_ACTION_NONE; /* Grouped motors broadcast feedback autonomously.
						   */
		}
		return MOTOR_ACTION_START;
	}
	return MOTOR_ACTION_NONE;
}

static int prepare_slot(struct gzrl_motor_bus *bus, struct gzrl_motor_tx_slot *slot,
			uint64_t now_us)
{
	struct gzrl_motor_group group = {0};
	struct gzrl_motor_frame frame = {0};
	bool any_action = false;
	bool grouped = false;

	for (size_t i = 0; i < bus->count; ++i) {
		if ((slot->members & BIT(i)) == 0) {
			continue;
		}
		struct motor_device_data *data = motor_data(bus->devices[i]);
		struct gzrl_motor_command command;
		enum motor_action action = select_command(data, now_us, &command);
		int result;

		grouped = data->motor.group_slot >= 0;
		slot->actions[i] = action;
		slot->generations[i] = data->generation;
		any_action |= action != MOTOR_ACTION_NONE;
		if (grouped) {
			if (action == MOTOR_ACTION_NONE) {
				gzrl_motor_stop_command(&data->motor, &command);
			}
			result = gzrl_motor_group_add(&group, &data->motor, &command);
		} else if (action != MOTOR_ACTION_NONE) {
			result = gzrl_motor_encode(&data->motor, &command, &frame);
		} else {
			result = 0;
		}
		if (result != 0) {
			motor_fault(data, GZRL_MOTOR_FAULT_COMMAND, result);
			return result;
		}
	}
	if (!any_action) {
		return 0;
	}
	if (grouped) {
		frame = group.frame;
	}
	slot->frame.flags = 0;
	slot->frame.dlc = frame.length;
	slot->frame.id = frame.id;
	memcpy(slot->frame.data, frame.data, sizeof(frame.data));
	return 1;
}

int gzrl_motor_bus_flush(struct gzrl_motor_bus *bus, uint64_t now_us)
{
	int result;
	int first_error;

	if (bus == NULL || !bus->attached) {
		return -ENODEV;
	}
	result = check_time(bus, now_us);
	if (result != 0) {
		return result;
	}
	first_error = collect_completions(bus, now_us);
	result = check_controller(bus);
	if (result != 0) {
		return result;
	}
	/* Overflow between process and flush must not permit another motion submission. */
	if (atomic_get(&bus->dropped) != 0) {
		fault_all(bus, GZRL_MOTOR_FAULT_RX_LOSS, -EOVERFLOW, true);
		first_error = -EOVERFLOW;
	}
	for (size_t i = 0; i < bus->count; ++i) {
		update_state(motor_data(bus->devices[i]), now_us);
	}
	size_t begin = bus->next_slot;

	for (size_t n = 0; n < bus->slot_count; ++n) {
		size_t index = (begin + n) % bus->slot_count;
		struct gzrl_motor_tx_slot *slot = &bus->slots[index];

		if (atomic_get(&slot->completion) != 0) {
			continue;
		}
		result = prepare_slot(bus, slot, now_us);
		if (result <= 0) {
			if (result < 0) {
				first_error = result;
			}
			continue;
		}
		slot->submitted_us = now_us;
		atomic_set(&slot->completion, 1);
		result = can_send(bus->can, &slot->frame, K_NO_WAIT, transmit_callback, slot);
		if (result == -EAGAIN || result == -EBUSY) {
			atomic_clear(&slot->completion);
			bus->stats.mailbox_busy++;
			bus->next_slot = index;
			return first_error != 0 ? first_error : result;
		}
		if (result != 0) {
			atomic_clear(&slot->completion);
			bus->stats.failed++;
			first_error = result;
		}
		for (size_t j = 0; j < bus->count; ++j) {
			if ((slot->members & BIT(j)) == 0) {
				continue;
			}
			struct motor_device_data *data = motor_data(bus->devices[j]);

			if (slot->actions[j] == MOTOR_ACTION_SAFE ||
			    slot->actions[j] == MOTOR_ACTION_START ||
			    slot->actions[j] == MOTOR_ACTION_OPERATION) {
				data->attempts++;
				data->attempted = true;
				data->last_attempt_us = now_us;
			}
			if (result != 0) {
				motor_fault(data, GZRL_MOTOR_FAULT_TX, result);
			}
		}
		if (result == 0) {
			bus->stats.submitted++;
		}
		bus->next_slot = (index + 1) % bus->slot_count;
	}
	return first_error;
}

int gzrl_motor_bus_detach(struct gzrl_motor_bus *bus)
{
	enum can_state state;
	int result;

	if (bus == NULL || !bus->attached) {
		return -ENODEV;
	}
	result = can_get_state(bus->can, &state, NULL);
	if (result != 0) {
		return result;
	}
	if (state != CAN_STATE_STOPPED) {
		return -EBUSY;
	}
	collect_completions(bus, bus->last_process_us);
	for (size_t i = 0; i < bus->slot_count; ++i) {
		if (atomic_get(&bus->slots[i].completion) != 0) {
			return -EBUSY;
		}
	}
	remove_filters(bus);
	for (size_t i = 0; i < bus->count; ++i) {
		struct motor_device_data *data = motor_data(bus->devices[i]);

		data->bus = NULL;
		data->state = GZRL_MOTOR_STOPPED;
		data->command_valid = false;
		data->operation_pending = false;
		data->safe_pending = false;
		gzrl_motor_invalidate_position(&data->motor);
	}
	k_msgq_purge(&bus->rx_queue);
	bus->attached = false;
	return 0;
}
