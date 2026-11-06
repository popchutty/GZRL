/*
 * SPDX-FileCopyrightText: 2026 GMaster contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>
#include <math.h>
#include <string.h>

#include <gzrl/motor/bus.h>
#include <zephyr/drivers/can/can_fake.h>
#include <zephyr/fff.h>
#include <zephyr/ztest.h>

DEFINE_FFF_GLOBALS;

static const struct device *const controllers[] = {
	DEVICE_DT_GET(DT_NODELABEL(fake_can0)),
	DEVICE_DT_GET(DT_NODELABEL(fake_can1)),
	DEVICE_DT_GET(DT_NODELABEL(conflict_can)),
};
static const struct device *const motors[] = {
	DEVICE_DT_GET(DT_NODELABEL(motor_a)),  DEVICE_DT_GET(DT_NODELABEL(motor_b)),
	DEVICE_DT_GET(DT_NODELABEL(motor_dm)), DEVICE_DT_GET(DT_NODELABEL(motor_dm2)),
	DEVICE_DT_GET(DT_NODELABEL(motor_mg)),
};
static const struct device *const isolated[] = {DEVICE_DT_GET(DT_NODELABEL(isolated_motor))};
static const struct device *const conflicting[] = {
	DEVICE_DT_GET(DT_NODELABEL(conflict_a)),
	DEVICE_DT_GET(DT_NODELABEL(conflict_b)),
};
static const struct gzrl_motor_policy policy = {
	.command_timeout_us = 20000,
	.startup_timeout_us = 50000,
	.retry_interval_us = 5000,
	.tx_timeout_us = 10000,
	.max_attempts = 3,
};
static struct gzrl_motor_bus buses[3];
static enum can_state states[3];
static int add_failure;
static int send_error;
static int send_budget;
static bool synchronous_completion;

struct recorded_filter {
	const struct device *dev;
	struct can_filter filter;
	can_rx_callback_t callback;
	void *arg;
	bool active;
};

struct recorded_send {
	const struct device *dev;
	struct can_frame frame;
	can_tx_callback_t callback;
	void *arg;
	bool pending;
};

static struct recorded_filter filters[16];
static struct recorded_send sends[64];
static size_t filter_count;
static size_t send_count;

extern uint32_t motor_cpp_capabilities(const struct device *dev);

static size_t controller_index(const struct device *dev)
{
	for (size_t i = 0; i < ARRAY_SIZE(controllers); ++i) {
		if (controllers[i] == dev) {
			return i;
		}
	}
	zassert_unreachable("unknown controller");
	return 0;
}

static int get_state(const struct device *dev, enum can_state *state,
		     struct can_bus_err_cnt *errors)
{
	if (state != NULL) {
		*state = states[controller_index(dev)];
	}
	if (errors != NULL) {
		*errors = (struct can_bus_err_cnt){0};
	}
	return 0;
}

static int add_filter(const struct device *dev, can_rx_callback_t callback, void *arg,
		      const struct can_filter *filter)
{
	if (add_failure >= 0 && filter_count == (size_t)add_failure) {
		return -ENOSPC;
	}
	zassert_true(filter_count < ARRAY_SIZE(filters));
	filters[filter_count] = (struct recorded_filter){
		.dev = dev,
		.filter = *filter,
		.callback = callback,
		.arg = arg,
		.active = true,
	};
	return (int)filter_count++;
}

static void remove_filter(const struct device *dev, int id)
{
	zassert_true(id >= 0 && (size_t)id < filter_count);
	zassert_equal(filters[id].dev, dev);
	zassert_true(filters[id].active, "filter removed twice");
	filters[id].active = false;
}

static int send_frame(const struct device *dev, const struct can_frame *frame, k_timeout_t wait,
		      can_tx_callback_t callback, void *arg)
{
	zassert_true(K_TIMEOUT_EQ(wait, K_NO_WAIT), "motor submission may not wait for mailboxes");
	zassert_not_null(callback, "send must be asynchronous");
	if (send_error != 0) {
		return send_error;
	}
	if (send_budget == 0) {
		return -EAGAIN;
	}
	if (send_budget > 0) {
		send_budget--;
	}
	zassert_true(send_count < ARRAY_SIZE(sends));
	sends[send_count++] = (struct recorded_send){
		.dev = dev,
		.frame = *frame,
		.callback = callback,
		.arg = arg,
		.pending = !synchronous_completion,
	};
	if (synchronous_completion) {
		callback(dev, 0, arg);
	}
	return 0;
}

static void complete(size_t index, int error)
{
	zassert_true(index < send_count);
	zassert_true(sends[index].pending, "completion must occur exactly once");
	sends[index].pending = false;
	sends[index].callback(sends[index].dev, error, sends[index].arg);
}

static void complete_all(int error)
{
	for (size_t i = 0; i < send_count; ++i) {
		if (sends[i].pending) {
			complete(i, error);
		}
	}
}

static void inject(const struct device *can, struct can_frame frame)
{
	bool matched = false;

	for (size_t i = 0; i < filter_count; ++i) {
		if (filters[i].active && filters[i].dev == can &&
		    (frame.id & filters[i].filter.mask) == filters[i].filter.id) {
			filters[i].callback(can, &frame, filters[i].arg);
			matched = true;
		}
	}
	zassert_true(matched, "feedback must reach a registered filter");
}

static void dji_feedback(const struct device *can, uint16_t id, uint16_t angle)
{
	inject(can, (struct can_frame){
			    .id = 0x200U + id,
			    .dlc = 8,
			    .data = {angle >> 8, angle & 0xff, 0, 60, 0, 1, 25, 0},
		    });
}

static void mit_feedback(uint8_t id, uint8_t state)
{
	inject(controllers[0], (struct can_frame){
				       .id = 0x50,
				       .dlc = 8,
				       .data = {(state << 4) | id, 0x80, 0, 0x80, 0x08, 0, 30, 25},
			       });
}

static struct gzrl_motor_snapshot snapshot(const struct device *dev, uint64_t now)
{
	struct gzrl_motor_snapshot value;

	zassert_ok(gzrl_motor_device_snapshot(dev, now, &value));
	return value;
}

static void configure(const struct device *dev)
{
	struct gzrl_motor_config config;
	const struct device *can;

	zassert_true(device_is_ready(dev));
	zassert_ok(gzrl_motor_device_topology(dev, &config, &can));
	zassert_true(device_is_ready(can));
	config.gear_ratio = 1.0f;
	config.effort_limit = gzrl_motor_is_mit(config.model) ? 5.0f : 1000.0f;
	config.feedback_timeout_us = 100000;
	if (gzrl_motor_is_mit(config.model)) {
		config.pmax = 3.14159265358979323846f;
		config.vmax = 30.0f;
		config.tmax = 10.0f;
	}
	zassert_ok(gzrl_motor_device_configure(dev, &config, &policy));
}

static void attach_primary(void)
{
	zassert_ok(gzrl_motor_bus_attach(&buses[0], controllers[0], motors, ARRAY_SIZE(motors)));
	states[0] = CAN_STATE_ERROR_ACTIVE;
}

static uint64_t start_dji(void)
{
	uint64_t now = gzrl_motor_bus_now_us();

	zassert_ok(gzrl_motor_device_start(motors[0], now));
	dji_feedback(controllers[0], 1, 1000);
	now = gzrl_motor_bus_now_us();
	zassert_ok(gzrl_motor_bus_process(&buses[0], now, 4));
	zassert_equal(snapshot(motors[0], now).state, GZRL_MOTOR_RUNNING);
	return now;
}

static struct gzrl_motor_command effort(float value)
{
	return (struct gzrl_motor_command){
		.type = GZRL_MOTOR_CMD_EFFORT,
		.effort_unit = GZRL_MOTOR_EFFORT_RAW,
		.effort = value,
	};
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	memset(buses, 0, sizeof(buses));
	memset(filters, 0, sizeof(filters));
	memset(sends, 0, sizeof(sends));
	for (size_t i = 0; i < ARRAY_SIZE(states); ++i) {
		states[i] = CAN_STATE_STOPPED;
	}
	filter_count = 0;
	send_count = 0;
	add_failure = -1;
	send_error = 0;
	send_budget = -1;
	synchronous_completion = false;
	fake_can_get_state_fake.custom_fake = get_state;
	fake_can_add_rx_filter_fake.custom_fake = add_filter;
	fake_can_remove_rx_filter_fake.custom_fake = remove_filter;
	fake_can_send_fake.custom_fake = send_frame;
	for (size_t i = 0; i < ARRAY_SIZE(motors); ++i) {
		configure(motors[i]);
	}
	configure(isolated[0]);
	configure(conflicting[0]);
	configure(conflicting[1]);
}

static void after(void *fixture)
{
	ARG_UNUSED(fixture);
	complete_all(-ENETDOWN);
	for (size_t i = 0; i < ARRAY_SIZE(buses); ++i) {
		states[i] = CAN_STATE_STOPPED;
		if (buses[i].attached) {
			zassert_ok(gzrl_motor_bus_detach(&buses[i]), "failed to release test bus");
		}
	}
	for (size_t i = 0; i < filter_count; ++i) {
		zassert_false(filters[i].active, "receive filter leaked");
	}
}

ZTEST(motor_bus, test_attach_idle_and_cpp_api)
{
	attach_primary();
	uint64_t now = gzrl_motor_bus_now_us();

	zassert_equal(filter_count, 4, "shared MIT feedback ID requires one filter");
	zassert_equal(fake_can_start_fake.call_count, 0, "application owns CAN lifecycle");
	zassert_ok(gzrl_motor_bus_process(&buses[0], now, 4));
	zassert_ok(gzrl_motor_bus_flush(&buses[0], now));
	zassert_equal(send_count, 0, "initialization must not enable or move motors");
	for (size_t i = 0; i < ARRAY_SIZE(motors); ++i) {
		struct gzrl_motor_snapshot value = snapshot(motors[i], now);

		zassert_equal(value.state, GZRL_MOTOR_STOPPED);
		zassert_false(value.online);
		zassert_false(value.operation_pending);
		zassert_equal(motor_cpp_capabilities(motors[i]),
			      gzrl_motor_device_capabilities(motors[i]));
	}
	zassert_equal(gzrl_motor_bus_detach(&buses[0]), -EBUSY);
	zassert_equal(fake_can_stop_fake.call_count, 0);
}

ZTEST(motor_bus, test_attach_rejects_partial_duplicate_foreign_and_conflicting_topology)
{
	const struct device *partial[] = {motors[0]};
	const struct device *duplicate[] = {motors[0], motors[0], motors[2], motors[3], motors[4]};
	const struct device *foreign[] = {motors[0], motors[1], motors[2], motors[3], isolated[0]};

	zassert_not_equal(gzrl_motor_bus_attach(&buses[0], controllers[0], partial, 1), 0);
	zassert_not_equal(gzrl_motor_bus_attach(&buses[0], controllers[0], duplicate, 5), 0);
	zassert_not_equal(gzrl_motor_bus_attach(&buses[0], controllers[0], foreign, 5), 0);
	zassert_not_equal(gzrl_motor_bus_attach(&buses[2], controllers[2], conflicting, 2), 0);
	zassert_equal(filter_count, 0, "invalid topology may not consume filters");
	attach_primary();
}

ZTEST(motor_bus, test_partial_filter_failure_rolls_back)
{
	add_failure = 2;
	zassert_equal(gzrl_motor_bus_attach(&buses[0], controllers[0], motors, 5), -ENOSPC);
	zassert_false(buses[0].attached);
	zassert_equal(fake_can_remove_rx_filter_fake.call_count, 2);
	zassert_false(filters[0].active);
	zassert_false(filters[1].active);
	add_failure = -1;
	attach_primary();
	zassert_equal(fake_can_send_fake.call_count, 0);
}

ZTEST(motor_bus, test_group_slots_stopped_peer_and_safe_stop)
{
	attach_primary();
	uint64_t now = start_dji();
	struct gzrl_motor_command command = effort(-123.0f);

	zassert_ok(gzrl_motor_device_command(motors[0], &command, now));
	zassert_ok(gzrl_motor_bus_flush(&buses[0], now));
	zassert_equal(send_count, 1);
	zassert_equal(sends[0].frame.id, 0x200);
	const uint8_t expected[] = {0xff, 0x85, 0, 0, 0, 0, 0, 0};

	zassert_mem_equal(sends[0].frame.data, expected, 8);
	zassert_equal(buses[0].stats.submitted, 1);
	zassert_equal(buses[0].stats.completed, 0);
	complete(0, 0);
	zassert_ok(gzrl_motor_bus_process(&buses[0], now, 0));
	zassert_equal(buses[0].stats.completed, 1);
	zassert_ok(gzrl_motor_device_stop(motors[0]));
	zassert_equal(gzrl_motor_device_command(motors[0], &command, now), -EACCES);
	zassert_ok(gzrl_motor_bus_flush(&buses[0], now));
	zassert_equal(send_count, 2);
	const uint8_t zero[8] = {0};

	zassert_mem_equal(sends[1].frame.data, zero, 8);
	zassert_equal(snapshot(motors[0], now).state, GZRL_MOTOR_STOPPED);
}

ZTEST(motor_bus, test_latest_pending_and_synchronous_completion)
{
	attach_primary();
	uint64_t now = start_dji();
	struct gzrl_motor_command command = effort(100.0f);

	zassert_ok(gzrl_motor_device_command(motors[0], &command, now));
	zassert_ok(gzrl_motor_bus_flush(&buses[0], now));
	command.effort = 200;
	zassert_ok(gzrl_motor_device_command(motors[0], &command, now));
	zassert_ok(gzrl_motor_bus_flush(&buses[0], now));
	command.effort = 300;
	zassert_ok(gzrl_motor_device_command(motors[0], &command, now));
	zassert_ok(gzrl_motor_bus_flush(&buses[0], now));
	zassert_equal(send_count, 1, "one in-flight frame per control ID");
	complete(0, 0);
	zassert_ok(gzrl_motor_bus_process(&buses[0], now, 0));
	synchronous_completion = true;
	zassert_ok(gzrl_motor_bus_flush(&buses[0], now));
	zassert_equal(send_count, 2);
	zassert_equal(sends[1].frame.data[0], 1);
	zassert_equal(sends[1].frame.data[1], 44);
	zassert_ok(gzrl_motor_bus_process(&buses[0], now, 0));
	zassert_equal(buses[0].stats.completed, 2, "early callback must not be overwritten");
}

ZTEST(motor_bus, test_mailbox_backpressure_and_round_robin)
{
	attach_primary();
	uint64_t now = start_dji();
	struct gzrl_motor_command command = effort(100);
	struct gzrl_motor_command status = {.type = GZRL_MOTOR_CMD_STATUS};

	zassert_ok(gzrl_motor_device_command(motors[0], &command, now));
	zassert_ok(gzrl_motor_device_operation(motors[4], &status, now));
	send_error = -EAGAIN;
	zassert_equal(gzrl_motor_bus_flush(&buses[0], now), -EAGAIN);
	zassert_equal(send_count, 0);
	zassert_true(buses[0].stats.mailbox_busy > 0);
	zassert_equal(snapshot(motors[0], now).state, GZRL_MOTOR_RUNNING);
	send_error = 0;
	send_budget = 1;
	zassert_equal(gzrl_motor_bus_flush(&buses[0], now), -EAGAIN);
	zassert_equal(send_count, 1);
	zassert_equal(sends[0].frame.id, 0x200);
	complete(0, 0);
	zassert_ok(gzrl_motor_bus_process(&buses[0], now, 0));
	send_budget = 1;
	zassert_equal(gzrl_motor_bus_flush(&buses[0], now), -EAGAIN);
	zassert_equal(send_count, 2);
	zassert_equal(sends[1].frame.id, 0x141,
		      "continual group traffic must not starve the other pending ID");
}

ZTEST(motor_bus, test_async_failure_stall_and_stale_completion)
{
	attach_primary();
	uint64_t now = start_dji();
	struct gzrl_motor_command command = effort(100);

	zassert_ok(gzrl_motor_device_command(motors[0], &command, now));
	zassert_ok(gzrl_motor_bus_flush(&buses[0], now));
	zassert_ok(gzrl_motor_device_stop(motors[0]));
	complete(0, 0);
	zassert_ok(gzrl_motor_bus_process(&buses[0], now, 0));
	zassert_equal(snapshot(motors[0], now).state, GZRL_MOTOR_STOPPED);
	zassert_ok(gzrl_motor_bus_flush(&buses[0], now));
	complete(1, -EIO);
	(void)gzrl_motor_bus_process(&buses[0], now, 0);
	zassert_equal(buses[0].stats.failed, 1);
	zassert_not_equal(snapshot(motors[0], now).state, GZRL_MOTOR_RUNNING);
}

ZTEST(motor_bus, test_safe_output_mailbox_wait_has_expiry)
{
	attach_primary();
	uint64_t now = start_dji();

	zassert_ok(gzrl_motor_device_stop(motors[0]));
	send_error = -EAGAIN;
	zassert_equal(gzrl_motor_bus_flush(&buses[0], now), -EAGAIN);
	now += policy.startup_timeout_us;
	(void)gzrl_motor_bus_flush(&buses[0], now);
	struct gzrl_motor_snapshot value = snapshot(motors[0], now);

	zassert_false(value.operation_pending, "mailbox shortage must not queue safe work forever");
	zassert_equal(value.last_error, -ETIMEDOUT);
	unsigned int calls = fake_can_send_fake.call_count;

	now += policy.retry_interval_us;
	(void)gzrl_motor_bus_flush(&buses[0], now);
	zassert_equal(fake_can_send_fake.call_count, calls, "expired work must not be retried");
}

ZTEST(motor_bus, test_persistent_send_failure_has_bounded_safe_attempts)
{
	attach_primary();
	uint64_t now = start_dji();
	struct gzrl_motor_command command = effort(100);

	zassert_ok(gzrl_motor_device_command(motors[0], &command, now));
	send_error = -EIO;
	zassert_equal(gzrl_motor_bus_flush(&buses[0], now), -EIO);
	for (size_t i = 0; i < policy.max_attempts + 2U; ++i) {
		now += policy.retry_interval_us;
		(void)gzrl_motor_bus_flush(&buses[0], now);
	}
	zassert_true(fake_can_send_fake.call_count <= policy.max_attempts + 1U,
		     "failed safe sends must consume a finite retry budget");
	zassert_equal(snapshot(motors[0], now).fault, GZRL_MOTOR_FAULT_TX);
	zassert_false(snapshot(motors[0], now).operation_pending);
}

ZTEST(motor_bus, test_stalled_hardware_frame_is_not_resubmitted)
{
	attach_primary();
	uint64_t now = start_dji();
	struct gzrl_motor_command command = effort(100);

	zassert_ok(gzrl_motor_device_command(motors[0], &command, now));
	zassert_ok(gzrl_motor_bus_flush(&buses[0], now));
	now += policy.tx_timeout_us;
	(void)gzrl_motor_bus_process(&buses[0], now, 0);
	struct gzrl_motor_snapshot value = snapshot(motors[0], now);

	zassert_equal(value.state, GZRL_MOTOR_FAULT);
	zassert_equal(value.fault, GZRL_MOTOR_FAULT_TX);
	zassert_equal(buses[0].stats.stalled, 1);
	(void)gzrl_motor_bus_flush(&buses[0], now);
	zassert_equal(send_count, 1, "software cannot cancel the hardware frame");
	zassert_equal(gzrl_motor_device_start(motors[0], now), -EBUSY);
}

ZTEST(motor_bus, test_command_expiry_and_invalid_control_revoke_permission)
{
	attach_primary();
	uint64_t now = start_dji();
	struct gzrl_motor_command command = effort(100);

	zassert_ok(gzrl_motor_device_command(motors[0], &command, now));
	now += policy.command_timeout_us;
	(void)gzrl_motor_bus_process(&buses[0], now, 0);
	struct gzrl_motor_snapshot value = snapshot(motors[0], now);

	zassert_equal(value.state, GZRL_MOTOR_FAULT);
	zassert_equal(value.fault, GZRL_MOTOR_FAULT_COMMAND);
	zassert_ok(gzrl_motor_bus_flush(&buses[0], now));
	zassert_equal(send_count, 1);
	const uint8_t zero[8] = {0};

	zassert_mem_equal(sends[0].frame.data, zero, 8);
	complete(0, 0);
	(void)gzrl_motor_bus_process(&buses[0], now, 0);
	zassert_equal(snapshot(motors[0], now).state, GZRL_MOTOR_FAULT,
		      "successful safe output must not restart motion");
}

ZTEST(motor_bus, test_nan_command_is_a_fault)
{
	attach_primary();
	uint64_t now = start_dji();
	struct gzrl_motor_command command = effort(NAN);

	zassert_not_equal(gzrl_motor_device_command(motors[0], &command, now), 0);
	zassert_equal(snapshot(motors[0], now).fault, GZRL_MOTOR_FAULT_COMMAND);
	zassert_ok(gzrl_motor_bus_flush(&buses[0], now));
	const uint8_t zero[8] = {0};

	zassert_equal(send_count, 1);
	zassert_mem_equal(sends[0].frame.data, zero, 8);
}

ZTEST(motor_bus, test_receive_budget_timestamp_malformed_and_shared_id)
{
	attach_primary();
	mit_feedback(1, 0);
	mit_feedback(2, 1);
	dji_feedback(controllers[0], 1, 1000);
	uint64_t receipt = gzrl_motor_bus_now_us();
	uint64_t drain = receipt + 1000;

	zassert_ok(gzrl_motor_bus_process(&buses[0], drain, 1));
	zassert_equal(snapshot(motors[2], drain).feedback.sequence, 1);
	zassert_equal(snapshot(motors[3], drain).feedback.sequence, 0);
	zassert_ok(gzrl_motor_bus_process(&buses[0], drain, 2));
	zassert_equal(snapshot(motors[3], drain).feedback.sequence, 1);
	struct gzrl_motor_snapshot original = snapshot(motors[0], drain);

	zassert_true(original.feedback.timestamp_us <= receipt);
	zassert_true(original.feedback.timestamp_us < drain, "do not timestamp queue drain");
	inject(controllers[0], (struct can_frame){.id = 0x201, .dlc = 7});
	(void)gzrl_motor_bus_process(&buses[0], drain, 1);
	struct gzrl_motor_snapshot value = snapshot(motors[0], drain);

	zassert_equal(value.feedback.sequence, original.feedback.sequence);
	zassert_mem_equal(value.feedback.raw, original.feedback.raw, 8);
	zassert_equal(buses[0].stats.rejected, 1);
}

ZTEST(motor_bus, test_overflow_invalidates_continuity_and_running_permission)
{
	attach_primary();
	uint64_t now = start_dji();

	for (size_t i = 0; i < CONFIG_GZRL_MOTOR_RX_DEPTH + 1; ++i) {
		dji_feedback(controllers[0], 1, 1001 + i);
	}
	now = gzrl_motor_bus_now_us();
	(void)gzrl_motor_bus_process(&buses[0], now, CONFIG_GZRL_MOTOR_RX_DEPTH);
	struct gzrl_motor_snapshot value = snapshot(motors[0], now);

	zassert_equal(buses[0].stats.dropped, 1);
	zassert_equal(value.fault, GZRL_MOTOR_FAULT_RX_LOSS);
	zassert_equal(value.state, GZRL_MOTOR_FAULT);
	zassert_false(value.feedback.valid & GZRL_MOTOR_VALID_POSITION);
	dji_feedback(controllers[0], 1, 1009);
	now = gzrl_motor_bus_now_us();
	(void)gzrl_motor_bus_process(&buses[0], now, 1);
	zassert_false(snapshot(motors[0], now).feedback.valid & GZRL_MOTOR_VALID_POSITION,
		      "later packets cannot restore evidence lost in queue overflow");
}

ZTEST(motor_bus, test_multibus_isolation_and_bus_off)
{
	attach_primary();
	zassert_ok(gzrl_motor_bus_attach(&buses[1], controllers[1], isolated, 1));
	states[1] = CAN_STATE_ERROR_ACTIVE;
	uint64_t now = start_dji();

	zassert_ok(gzrl_motor_device_start(isolated[0], now));
	dji_feedback(controllers[1], 1, 2222);
	now = gzrl_motor_bus_now_us();
	zassert_ok(gzrl_motor_bus_process(&buses[1], now, 1));
	zassert_equal(snapshot(motors[0], now).feedback.raw[0], 1000 >> 8);
	zassert_equal(snapshot(isolated[0], now).feedback.raw[0], 2222 >> 8);
	states[0] = CAN_STATE_BUS_OFF;
	(void)gzrl_motor_bus_process(&buses[0], now, 0);
	zassert_equal(snapshot(motors[0], now).fault, GZRL_MOTOR_FAULT_BUS);
	zassert_equal(snapshot(isolated[0], now).state, GZRL_MOTOR_RUNNING);
	zassert_equal(fake_can_recover_fake.call_count, 0, "application owns recovery");
	states[0] = CAN_STATE_ERROR_ACTIVE;
	(void)gzrl_motor_bus_process(&buses[0], now, 0);
	zassert_equal(snapshot(motors[0], now).state, GZRL_MOTOR_FAULT);
}

ZTEST(motor_bus, test_mit_start_needs_enabled_feedback_and_times_out)
{
	attach_primary();
	uint64_t now = gzrl_motor_bus_now_us();

	mit_feedback(1, 0);
	zassert_ok(gzrl_motor_bus_process(&buses[0], now, 1));
	zassert_ok(gzrl_motor_device_start(motors[2], now));
	zassert_ok(gzrl_motor_bus_flush(&buses[0], now));
	zassert_equal(send_count, 1);
	zassert_equal(sends[0].frame.id, 1);
	zassert_equal(sends[0].frame.data[7], 0xfc);
	complete(0, 0);
	zassert_ok(gzrl_motor_bus_process(&buses[0], now, 0));
	zassert_equal(snapshot(motors[2], now).state, GZRL_MOTOR_STARTING);
	mit_feedback(1, 1);
	now = gzrl_motor_bus_now_us();
	zassert_ok(gzrl_motor_bus_process(&buses[0], now, 1));
	zassert_equal(snapshot(motors[2], now).state, GZRL_MOTOR_RUNNING);
	zassert_ok(gzrl_motor_device_start(motors[3], now));
	(void)gzrl_motor_bus_process(&buses[0], now + policy.startup_timeout_us, 0);
	zassert_equal(snapshot(motors[3], now + policy.startup_timeout_us).fault,
		      GZRL_MOTOR_FAULT_START);
}

ZTEST(motor_bus, test_queued_pre_start_feedback_cannot_confirm_start)
{
	attach_primary();
	mit_feedback(1, 1);
	uint64_t now = gzrl_motor_bus_now_us();

	zassert_ok(gzrl_motor_device_start(motors[2], now));
	zassert_ok(gzrl_motor_bus_flush(&buses[0], now));
	zassert_equal(send_count, 1);
	complete(0, 0);
	zassert_ok(gzrl_motor_bus_process(&buses[0], now, 1));
	zassert_equal(snapshot(motors[2], now).state, GZRL_MOTOR_STARTING,
		      "queued preauthorization feedback cannot confirm a new startup");
}

ZTEST(motor_bus, test_old_same_tick_feedback_cannot_confirm_start)
{
	attach_primary();
	mit_feedback(1, 1);
	uint64_t now = gzrl_motor_bus_now_us();

	zassert_ok(gzrl_motor_bus_process(&buses[0], now, 1));
	zassert_equal(snapshot(motors[2], now).feedback.sequence, 1);
	zassert_ok(gzrl_motor_device_start(motors[2], now));
	zassert_ok(gzrl_motor_bus_process(&buses[0], now, 0));
	zassert_equal(snapshot(motors[2], now).state, GZRL_MOTOR_STARTING,
		      "feedback predating the request cannot confirm startup in the same tick");
}

ZTEST(motor_bus, test_maintenance_permissions_and_software_zero)
{
	attach_primary();
	uint64_t now = gzrl_motor_bus_now_us();
	struct gzrl_motor_command operation = {.type = GZRL_MOTOR_CMD_ENABLE};

	zassert_equal(gzrl_motor_device_operation(motors[2], &operation, now), -EACCES);
	operation = (struct gzrl_motor_command){.type = GZRL_MOTOR_CMD_BRAKE};
	zassert_equal(gzrl_motor_device_operation(motors[4], &operation, now), -EACCES);
	operation = (struct gzrl_motor_command){.type = GZRL_MOTOR_CMD_SET_ZERO};
	zassert_equal(gzrl_motor_device_operation(motors[0], &operation, now), -ENOTSUP);
	zassert_equal(gzrl_motor_device_reset_position(motors[0], now), -ENODATA);
	dji_feedback(controllers[0], 1, 1000);
	dji_feedback(controllers[0], 1, 1200);
	now = gzrl_motor_bus_now_us();
	zassert_ok(gzrl_motor_bus_process(&buses[0], now, 2));
	zassert_true(snapshot(motors[0], now).feedback.position_rad > 0);
	zassert_ok(gzrl_motor_device_reset_position(motors[0], now));
	zassert_equal(snapshot(motors[0], now).feedback.position_rad, 0);
	operation = (struct gzrl_motor_command){.type = GZRL_MOTOR_CMD_CLEAR_ERROR};
	zassert_ok(gzrl_motor_device_operation(motors[2], &operation, now));
	zassert_ok(gzrl_motor_bus_flush(&buses[0], now));
	zassert_equal(sends[0].frame.data[7], 0xfb);
	zassert_equal(snapshot(motors[2], now).state, GZRL_MOTOR_STOPPED);
}

ZTEST(motor_bus, test_mg_brake_evidence_and_internal_speed)
{
	attach_primary();
	uint64_t now = gzrl_motor_bus_now_us();

	zassert_ok(gzrl_motor_device_start(motors[4], now));
	zassert_ok(gzrl_motor_bus_flush(&buses[0], now));
	zassert_equal(send_count, 1);
	zassert_equal(sends[0].frame.id, 0x141);
	zassert_equal(sends[0].frame.data[0], 0x8c);
	zassert_equal(sends[0].frame.data[1], 1);
	complete(0, 0);
	zassert_ok(gzrl_motor_bus_process(&buses[0], now, 0));
	zassert_equal(snapshot(motors[4], now).state, GZRL_MOTOR_STARTING);
	inject(controllers[0], (struct can_frame){.id = 0x141, .dlc = 8, .data = {0x8c, 1}});
	now = gzrl_motor_bus_now_us();
	zassert_ok(gzrl_motor_bus_process(&buses[0], now, 1));
	zassert_false(snapshot(motors[4], now).online, "brake ACK is not motion feedback");
	zassert_equal(snapshot(motors[4], now).state, GZRL_MOTOR_STARTING);
	inject(controllers[0], (struct can_frame){
				       .id = 0x141,
				       .dlc = 8,
				       .data = {0x9c, 25, 0, 0, 0, 0, 0, 0},
			       });
	now = gzrl_motor_bus_now_us();
	zassert_ok(gzrl_motor_bus_process(&buses[0], now, 1));
	zassert_equal(snapshot(motors[4], now).state, GZRL_MOTOR_RUNNING);
	struct gzrl_motor_command command = {
		.type = GZRL_MOTOR_CMD_SPEED,
		.velocity_rad_s = 1.0f,
	};

	zassert_ok(gzrl_motor_device_command(motors[4], &command, now));
	zassert_ok(gzrl_motor_bus_flush(&buses[0], now));
	zassert_equal(send_count, 2);
	zassert_equal(sends[1].frame.data[0], 0xa2);
	zassert_equal(sends[1].frame.data[4], 0x61);
	zassert_equal(sends[1].frame.data[5], 0x16);
	complete(1, 0);
	zassert_ok(gzrl_motor_bus_process(&buses[0], now, 0));
	zassert_ok(gzrl_motor_device_stop(motors[4]));
	zassert_ok(gzrl_motor_bus_flush(&buses[0], now));
	zassert_equal(sends[2].frame.data[0], 0xa1, "normal stop must not engage brake");
	zassert_equal(sends[2].frame.data[4], 0);
	zassert_equal(sends[2].frame.data[5], 0);
	complete(2, 0);
	zassert_ok(gzrl_motor_bus_process(&buses[0], now, 0));
	command = (struct gzrl_motor_command){
		.type = GZRL_MOTOR_CMD_BRAKE,
		.brake_engaged = true,
	};
	zassert_ok(gzrl_motor_device_operation(motors[4], &command, now));
	zassert_ok(gzrl_motor_bus_flush(&buses[0], now));
	zassert_equal(sends[3].frame.data[0], 0x8c);
	zassert_equal(sends[3].frame.data[1], 0);
	complete(3, 0);
	zassert_ok(gzrl_motor_bus_process(&buses[0], now, 0));
	zassert_false(snapshot(motors[4], now).feedback.brake_engaged,
		      "CAN completion must not fabricate a brake acknowledgement");
	inject(controllers[0], (struct can_frame){.id = 0x141, .dlc = 8, .data = {0x8c, 0}});
	zassert_ok(gzrl_motor_bus_process(&buses[0], gzrl_motor_bus_now_us(), 1));
	zassert_true(snapshot(motors[4], gzrl_motor_bus_now_us()).feedback.brake_engaged);
}

ZTEST(motor_bus, test_overflow_between_process_and_flush_prevents_motion)
{
	attach_primary();
	uint64_t now = start_dji();
	struct gzrl_motor_command command = effort(100);

	zassert_ok(gzrl_motor_device_command(motors[0], &command, now));
	for (size_t i = 0; i < CONFIG_GZRL_MOTOR_RX_DEPTH + 1; ++i) {
		dji_feedback(controllers[0], 1, 1000 + i);
	}
	zassert_equal(gzrl_motor_bus_flush(&buses[0], now), -EOVERFLOW);
	zassert_equal(snapshot(motors[0], now).fault, GZRL_MOTOR_FAULT_RX_LOSS);
	zassert_equal(send_count, 1);
	const uint8_t zero[8] = {0};

	zassert_mem_equal(sends[0].frame.data, zero, sizeof(zero));
}

ZTEST(motor_bus, test_expired_operation_never_reaches_controller)
{
	attach_primary();
	uint64_t now = gzrl_motor_bus_now_us();
	struct gzrl_motor_command operation = {.type = GZRL_MOTOR_CMD_SET_ZERO};

	zassert_ok(gzrl_motor_device_operation(motors[2], &operation, now));
	now += policy.startup_timeout_us;
	zassert_ok(gzrl_motor_bus_flush(&buses[0], now));
	zassert_equal(send_count, 0, "expired hardware zero must not be sent");
	struct gzrl_motor_snapshot value = snapshot(motors[2], now);

	zassert_false(value.operation_pending);
	zassert_equal(value.last_error, -ETIMEDOUT);
	zassert_equal(value.state, GZRL_MOTOR_STOPPED);
}

ZTEST(motor_bus, test_disconnect_requires_explicit_restart_and_new_command)
{
	attach_primary();
	uint64_t now = start_dji();
	struct gzrl_motor_command command = effort(100);

	zassert_ok(gzrl_motor_device_command(motors[0], &command, now));
	now += 100000;
	zassert_ok(gzrl_motor_bus_process(&buses[0], now, 0));
	zassert_equal(snapshot(motors[0], now).fault, GZRL_MOTOR_FAULT_FEEDBACK);
	zassert_ok(gzrl_motor_bus_flush(&buses[0], now));
	complete(0, 0);
	zassert_ok(gzrl_motor_bus_process(&buses[0], now, 0));
	/* Advance the simulator's real callback clock past the explicit offline deadline. */
	k_sleep(K_MSEC(110));
	dji_feedback(controllers[0], 1, 1020);
	now = gzrl_motor_bus_now_us();
	zassert_ok(gzrl_motor_bus_process(&buses[0], now, 1));
	struct gzrl_motor_snapshot value = snapshot(motors[0], now);

	zassert_true(value.online);
	zassert_equal(value.state, GZRL_MOTOR_FAULT);
	zassert_false(value.feedback.valid & GZRL_MOTOR_VALID_POSITION);
	zassert_ok(gzrl_motor_device_start(motors[0], now));
	dji_feedback(controllers[0], 1, 1021);
	now = gzrl_motor_bus_now_us();
	zassert_ok(gzrl_motor_bus_process(&buses[0], now, 1));
	zassert_equal(snapshot(motors[0], now).state, GZRL_MOTOR_RUNNING);
	zassert_ok(gzrl_motor_bus_flush(&buses[0], now));
	zassert_equal(send_count, 1, "restart must not reuse old command");
}

ZTEST(motor_bus, test_clock_regression_revokes_permission)
{
	attach_primary();
	uint64_t now = start_dji();
	struct gzrl_motor_command command = effort(100);

	zassert_ok(gzrl_motor_device_command(motors[0], &command, now));
	zassert_ok(gzrl_motor_bus_process(&buses[0], now + 1, 0));
	zassert_equal(gzrl_motor_bus_flush(&buses[0], now), -ESTALE);
	zassert_equal(snapshot(motors[0], now).fault, GZRL_MOTOR_FAULT_TIME);
	zassert_false(snapshot(motors[0], now).feedback.valid & GZRL_MOTOR_VALID_POSITION);
	zassert_equal(send_count, 0);
}

static uint64_t start_mit(void)
{
	uint64_t now = gzrl_motor_bus_now_us();

	zassert_ok(gzrl_motor_device_start(motors[2], now));
	zassert_ok(gzrl_motor_bus_flush(&buses[0], now));
	complete(0, 0);
	mit_feedback(1, 1);
	now = gzrl_motor_bus_now_us();
	zassert_ok(gzrl_motor_bus_process(&buses[0], now, 1));
	zassert_equal(snapshot(motors[2], now).state, GZRL_MOTOR_RUNNING);
	return now;
}

ZTEST(motor_bus, test_transient_drive_fault_cannot_be_erased_by_later_feedback)
{
	attach_primary();
	(void)start_mit();
	mit_feedback(1, 8);
	mit_feedback(1, 1);
	uint64_t now = gzrl_motor_bus_now_us();

	zassert_ok(gzrl_motor_bus_process(&buses[0], now, 2));
	struct gzrl_motor_snapshot value = snapshot(motors[2], now);

	zassert_equal(value.state, GZRL_MOTOR_FAULT,
		      "a healthy sample must not erase an earlier fault in the same FIFO batch");
	zassert_equal(value.fault, GZRL_MOTOR_FAULT_DRIVE);
}

ZTEST(motor_bus, test_transient_disabled_state_revokes_permission)
{
	attach_primary();
	(void)start_mit();
	mit_feedback(1, 0);
	mit_feedback(1, 1);
	uint64_t now = gzrl_motor_bus_now_us();

	zassert_ok(gzrl_motor_bus_process(&buses[0], now, 2));
	zassert_equal(snapshot(motors[2], now).fault, GZRL_MOTOR_FAULT_DRIVE,
		      "unexpected disable must latch even when another sample reports enabled");
}

ZTEST(motor_bus, test_unobserved_feedback_gap_cannot_be_healed_by_new_packet)
{
	struct gzrl_motor_config config;
	struct gzrl_motor_policy slow_command = policy;
	const struct device *can;

	zassert_ok(gzrl_motor_device_topology(motors[0], &config, &can));
	config.gear_ratio = 1;
	config.effort_limit = 1000;
	config.feedback_timeout_us = 10000;
	slow_command.command_timeout_us = 100000;
	zassert_ok(gzrl_motor_device_configure(motors[0], &config, &slow_command));
	attach_primary();
	uint64_t now = start_dji();
	struct gzrl_motor_command command = effort(100);

	zassert_ok(gzrl_motor_device_command(motors[0], &command, now));
	/* No process call observes the offline interval before new feedback is enqueued. */
	k_sleep(K_MSEC(10));
	dji_feedback(controllers[0], 1, 1100);
	now = gzrl_motor_bus_now_us();
	zassert_ok(gzrl_motor_bus_process(&buses[0], now, 1));
	struct gzrl_motor_snapshot value = snapshot(motors[0], now);

	zassert_true(value.online, "new packet itself is fresh");
	zassert_equal(value.state, GZRL_MOTOR_FAULT);
	zassert_equal(value.fault, GZRL_MOTOR_FAULT_FEEDBACK,
		      "an undrained gap must still require explicit restart");
}

ZTEST(motor_bus, test_group_running_peers_survive_one_motor_stop)
{
	attach_primary();
	uint64_t now = start_dji();

	zassert_ok(gzrl_motor_device_start(motors[1], now));
	dji_feedback(controllers[0], 2, 2000);
	now = gzrl_motor_bus_now_us();
	zassert_ok(gzrl_motor_bus_process(&buses[0], now, 1));
	struct gzrl_motor_command command_a = effort(123);
	struct gzrl_motor_command command_b = effort(-234);

	zassert_ok(gzrl_motor_device_command(motors[0], &command_a, now));
	zassert_ok(gzrl_motor_device_command(motors[1], &command_b, now));
	zassert_ok(gzrl_motor_bus_flush(&buses[0], now));
	const uint8_t expected[] = {0, 123, 0xff, 0x16, 0, 0, 0, 0};

	zassert_equal(send_count, 1);
	zassert_mem_equal(sends[0].frame.data, expected, sizeof(expected));
	zassert_ok(gzrl_motor_device_stop(motors[0]));
	complete(0, 0);
	zassert_ok(gzrl_motor_bus_process(&buses[0], now, 0));
	zassert_ok(gzrl_motor_bus_flush(&buses[0], now));
	const uint8_t stopped[] = {0, 0, 0xff, 0x16, 0, 0, 0, 0};

	zassert_equal(send_count, 2);
	zassert_mem_equal(sends[1].frame.data, stopped, sizeof(stopped));
	zassert_equal(snapshot(motors[1], now).state, GZRL_MOTOR_RUNNING);
}

ZTEST_SUITE(motor_bus, NULL, NULL, before, after, NULL);
