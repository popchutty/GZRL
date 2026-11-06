/*
 * SPDX-FileCopyrightText: 2026 GMaster contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#include <gzrl/motor/bus.h>
#include <zephyr/ztest.h>

static const struct device *const can = DEVICE_DT_GET(DT_NODELABEL(can_loopback0));
static const struct device *const motors[] = {
	DEVICE_DT_GET(DT_NODELABEL(motor_a)),
	DEVICE_DT_GET(DT_NODELABEL(motor_b)),
};
static struct gzrl_motor_bus bus;
CAN_MSGQ_DEFINE(observed_commands, 4);

ZTEST(motor_loopback, test_real_callbacks_grouping_completion_and_cleanup)
{
	const struct gzrl_motor_policy policy = {
		.command_timeout_us = 100000,
		.startup_timeout_us = 100000,
		.retry_interval_us = 10000,
		.tx_timeout_us = 50000,
		.max_attempts = 3,
	};
	const struct can_filter filter = {.id = 0x200, .mask = CAN_STD_ID_MASK};
	const struct gzrl_motor_command command = {
		.type = GZRL_MOTOR_CMD_EFFORT,
		.effort_unit = GZRL_MOTOR_EFFORT_RAW,
		.effort = 321,
	};
	struct can_frame feedback = {.id = 0x201, .dlc = 8, .data = {0x10, 0, 0, 60, 0, 3, 25, 0}};
	struct can_frame received;
	struct gzrl_motor_snapshot snapshot;

	zassert_true(device_is_ready(can));
	zassert_ok(can_set_mode(can, CAN_MODE_LOOPBACK));
	for (size_t i = 0; i < ARRAY_SIZE(motors); ++i) {
		struct gzrl_motor_config config;
		const struct device *controller;

		zassert_ok(gzrl_motor_device_topology(motors[i], &config, &controller));
		zassert_equal(controller, can);
		config.gear_ratio = 1;
		config.effort_limit = 1000;
		config.feedback_timeout_us = 500000;
		zassert_ok(gzrl_motor_device_configure(motors[i], &config, &policy));
	}
	zassert_ok(gzrl_motor_bus_attach(&bus, can, motors, ARRAY_SIZE(motors)));
	int filter_id = can_add_rx_filter_msgq(can, &observed_commands, &filter);

	zassert_true(filter_id >= 0);
	zassert_ok(can_start(can));
	uint64_t now = gzrl_motor_bus_now_us();

	zassert_ok(gzrl_motor_bus_flush(&bus, now));
	zassert_equal(k_msgq_num_used_get(&observed_commands), 0,
		      "attach must not cause a control frame");
	zassert_ok(gzrl_motor_device_start(motors[0], now));
	zassert_ok(can_send(can, &feedback, K_MSEC(100), NULL, NULL));
	/* Loopback reports TX completion before RX: wait for actual feedback evidence. */
	for (size_t attempt = 0; attempt < 20; ++attempt) {
		now = gzrl_motor_bus_now_us();
		zassert_ok(gzrl_motor_bus_process(&bus, now, 4));
		zassert_ok(gzrl_motor_device_snapshot(motors[0], now, &snapshot));
		if (snapshot.online) {
			break;
		}
		k_sleep(K_MSEC(1));
	}
	zassert_true(snapshot.online);
	zassert_equal(snapshot.state, GZRL_MOTOR_RUNNING);
	zassert_equal(snapshot.feedback.sequence, 1);
	zassert_ok(gzrl_motor_device_command(motors[0], &command, now));
	zassert_ok(gzrl_motor_bus_flush(&bus, now));
	zassert_ok(k_msgq_get(&observed_commands, &received, K_MSEC(100)));
	const uint8_t expected[] = {1, 0x41, 0, 0, 0, 0, 0, 0};

	zassert_mem_equal(received.data, expected, sizeof(expected));
	now = gzrl_motor_bus_now_us();
	zassert_ok(gzrl_motor_bus_process(&bus, now, 4));
	zassert_equal(bus.stats.completed, 1);
	zassert_ok(gzrl_motor_device_stop(motors[0]));
	zassert_ok(gzrl_motor_bus_flush(&bus, now));
	zassert_ok(k_msgq_get(&observed_commands, &received, K_MSEC(100)));
	const uint8_t zero[8] = {0};

	zassert_mem_equal(received.data, zero, sizeof(zero));
	zassert_ok(gzrl_motor_bus_process(&bus, gzrl_motor_bus_now_us(), 4));
	zassert_equal(bus.stats.completed, 2);
	zassert_ok(can_stop(can));
	can_remove_rx_filter(can, filter_id);
	zassert_ok(gzrl_motor_bus_detach(&bus));
	zassert_false(bus.attached);
}

ZTEST_SUITE(motor_loopback, NULL, NULL, NULL, NULL, NULL);
