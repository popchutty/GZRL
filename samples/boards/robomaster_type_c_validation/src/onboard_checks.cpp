/*
 * SPDX-FileCopyrightText: 2026 GMaster contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "validation.hpp"

#include <errno.h>
#include <inttypes.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys_clock.h>
#include <zephyr/version.h>

BUILD_ASSERT(DT_PROP(DT_NODELABEL(rcc), clock_frequency) == 168000000,
	     "The RoboMaster Type C system clock must be 168 MHz");

namespace type_c_validation
{

namespace
{

constexpr uint32_t system_clock_hz = 168000000U;
constexpr uint32_t color_step_ms = 400U;
constexpr uint32_t sensor_interactive_period_ms = 100U;
constexpr uint32_t magnetometer_retry_delay_ms = 20U;

K_SEM_DEFINE(button_sem, 0, 1);
CAN_MSGQ_DEFINE(can_loopback_msgq, 1);
K_SEM_DEFINE(can_loopback_tx_sem, 0, 1);
int can_loopback_tx_result;

void can_loopback_tx_callback(const struct device *device, int error, void *user_data)
{
	ARG_UNUSED(device);
	ARG_UNUSED(user_data);
	can_loopback_tx_result = error;
	k_sem_give(&can_loopback_tx_sem);
}

int can_loopback_send_bounded(const struct device *can, const struct can_frame &frame)
{
	k_sem_reset(&can_loopback_tx_sem);
	can_loopback_tx_result = -EINPROGRESS;
	int ret = can_send(can, &frame, K_MSEC(200), can_loopback_tx_callback, nullptr);
	if (ret != 0) {
		return ret;
	}

	ret = k_sem_take(&can_loopback_tx_sem, K_MSEC(500));
	if (ret != 0) {
		return -ETIMEDOUT;
	}
	return can_loopback_tx_result;
}

const struct gpio_dt_spec user_button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
const struct pwm_dt_spec rgb_leds[] = {
	PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0)),
	PWM_DT_SPEC_GET(DT_ALIAS(pwm_led1)),
	PWM_DT_SPEC_GET(DT_ALIAS(pwm_led2)),
};
const struct pwm_dt_spec buzzer = PWM_DT_SPEC_GET(DT_ALIAS(pwm_buzzer));

struct gpio_callback button_callback;

void button_callback_handler(const struct device *port, struct gpio_callback *callback,
			     uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(callback);
	ARG_UNUSED(pins);
	k_sem_give(&button_sem);
}

void report_device_ready(reporter &reporter, const char *name, const struct device *device)
{
	if (device_is_ready(device)) {
		reporter.record(result_kind::success, name,
				"controller initialized; pins not exercised");
	} else {
		reporter.record(result_kind::error, name, "device is not ready");
	}
}

struct three_axis_sensor_spec {
	const struct device *sensor;
	enum sensor_channel channel;
	const char *result_name;
	const char *axis_names[3];
	const char *unit;
	uint8_t retry_count;
	uint32_t retry_delay_ms;
};

const three_axis_sensor_spec accelerometer_spec = {
	DEVICE_DT_GET(DT_ALIAS(accel0)),
	SENSOR_CHAN_ACCEL_XYZ,
	"sensor.bmi088.accel",
	{"sensor.bmi088.accel.x", "sensor.bmi088.accel.y", "sensor.bmi088.accel.z"},
	"m/s^2",
	1U,
	0U,
};

const three_axis_sensor_spec gyroscope_spec = {
	DEVICE_DT_GET(DT_ALIAS(gyro0)),
	SENSOR_CHAN_GYRO_XYZ,
	"sensor.bmi088.gyro",
	{"sensor.bmi088.gyro.x", "sensor.bmi088.gyro.y", "sensor.bmi088.gyro.z"},
	"rad/s",
	1U,
	0U,
};

const three_axis_sensor_spec magnetometer_spec = {
	DEVICE_DT_GET(DT_ALIAS(magn0)),
	SENSOR_CHAN_MAGN_XYZ,
	"sensor.ist8310",
	{"sensor.ist8310.x", "sensor.ist8310.y", "sensor.ist8310.z"},
	"gauss",
	3U,
	magnetometer_retry_delay_ms,
};

int read_three_axis(const three_axis_sensor_spec &spec, int64_t values[3])
{
	struct sensor_value sensor_values[3] = {};
	int ret = sensor_sample_fetch(spec.sensor);
	if (ret == 0) {
		ret = sensor_channel_get(spec.sensor, spec.channel, sensor_values);
	}
	if (ret != 0) {
		return ret;
	}

	for (size_t i = 0U; i < ARRAY_SIZE(sensor_values); i++) {
		values[i] = sensor_value_to_micro(sensor_values[i].val1, sensor_values[i].val2);
	}
	return 0;
}

int read_three_axis_interactive(const three_axis_sensor_spec &spec, int64_t values[3])
{
	for (uint8_t attempt = 0U; attempt < spec.retry_count; attempt++) {
		const int ret = read_three_axis(spec, values);
		if (ret == 0) {
			return 0;
		}
		if (ret != -EIO || attempt + 1U >= spec.retry_count) {
			return ret;
		}
		k_sleep(K_MSEC(spec.retry_delay_ms));
	}

	return -EIO;
}

void print_three_axis(const three_axis_sensor_spec &spec, const int64_t values[3],
		      const reporter &reporter)
{
	for (size_t i = 0U; i < ARRAY_SIZE(spec.axis_names); i++) {
		print_fixed_point(reporter, spec.axis_names[i], values[i], spec.unit);
	}
}

bool fetch_three_axis(const three_axis_sensor_spec &spec, reporter &reporter)
{
	int64_t values[3] = {};

	if (!device_is_ready(spec.sensor)) {
		reporter.record(result_kind::error, spec.result_name, "device is not ready");
		return false;
	}

	const int ret = read_three_axis(spec, values);
	if (ret != 0) {
		reporter.record(result_kind::error, spec.result_name,
				"sample fetch or channel read failed", ret);
		return false;
	}

	print_three_axis(spec, values, reporter);
	reporter.record(result_kind::success, spec.result_name, "XYZ sample completed");
	return true;
}

bool can_frames_equal(const struct can_frame &expected, const struct can_frame &received)
{
	return expected.flags == received.flags && expected.id == received.id &&
	       expected.dlc == received.dlc &&
	       memcmp(expected.data, received.data, can_dlc_to_bytes(expected.dlc)) == 0;
}

int set_rgb(uint32_t green_pulse, uint32_t red_pulse, uint32_t blue_pulse)
{
	const uint32_t pulses[] = {green_pulse, red_pulse, blue_pulse};
	int first_error = 0;

	for (size_t i = 0; i < ARRAY_SIZE(rgb_leds); i++) {
		const int ret = pwm_set_dt(&rgb_leds[i], rgb_leds[i].period, pulses[i]);
		if (ret != 0 && first_error == 0) {
			first_error = ret;
		}
	}

	return first_error;
}

int turn_rgb_off()
{
	return set_rgb(0U, 0U, 0U);
}

} // namespace

void validate_system_info(reporter &reporter)
{
	shell_print(reporter.output(), "[INFO] system.info: board=%s zephyr=%s",
		    CONFIG_BOARD_TARGET, KERNEL_VERSION_STRING);
	reporter.info("safety", "laser, IMU heater, watchdog reset, flash/settings, camera, and "
				"USB host are excluded");
	reporter.record(result_kind::success, "shell.command", "validation shell command accepted");
}

void validate_clock(reporter &reporter)
{
	const uint32_t reported_hz = sys_clock_hw_cycles_per_sec();
	const int64_t start_ms = k_uptime_get();
	const uint64_t start_cycles = k_cycle_get_64();

	k_sleep(K_MSEC(200));

	const uint64_t elapsed_cycles = k_cycle_get_64() - start_cycles;
	const int64_t elapsed_ms = k_uptime_get() - start_ms;
	const uint64_t expected_cycles =
		(static_cast<uint64_t>(reported_hz) * static_cast<uint64_t>(elapsed_ms)) / 1000U;
	const uint64_t tolerance = expected_cycles / 20U;
	const uint64_t lower_bound = expected_cycles - tolerance;
	const uint64_t upper_bound = expected_cycles + tolerance;

	shell_print(reporter.output(),
		    "[INFO] clock: configured=%" PRIu32 " Hz elapsed=%" PRId64
		    " ms cycles=%" PRIu64,
		    reported_hz, elapsed_ms, elapsed_cycles);

	if (reported_hz != system_clock_hz || elapsed_ms < 190 || elapsed_ms > 300 ||
	    elapsed_cycles < lower_bound || elapsed_cycles > upper_bound) {
		reporter.record(result_kind::error, "clock.kernel_cadence",
				"clock frequency or cadence outside tolerance");
		return;
	}

	reporter.record(result_kind::success, "clock.kernel_cadence",
			"168 MHz clock and kernel timing agree");
}

void validate_devices(reporter &reporter)
{
	struct device_check {
		const char *name;
		const struct device *device;
	};

	const device_check checks[] = {
		{"uart.usart1", DEVICE_DT_GET(DT_NODELABEL(usart1))},
		{"uart.usart3.dbus", DEVICE_DT_GET(DT_NODELABEL(usart3))},
		{"uart.usart6", DEVICE_DT_GET(DT_NODELABEL(usart6))},
		{"can.can1", DEVICE_DT_GET(DT_NODELABEL(can1))},
		{"can.can2", DEVICE_DT_GET(DT_NODELABEL(can2))},
		{"spi.spi2", DEVICE_DT_GET(DT_NODELABEL(spi2))},
		{"i2c.i2c1", DEVICE_DT_GET(DT_NODELABEL(i2c1))},
		{"i2c.i2c2", DEVICE_DT_GET(DT_NODELABEL(i2c2))},
		{"pwm.servo1_4", DEVICE_DT_GET(DT_NODELABEL(pwm1))},
		{"pwm.buzzer", buzzer.dev},
		{"pwm.rgb", DEVICE_DT_GET(DT_NODELABEL(pwm5))},
		{"pwm.servo5_7", DEVICE_DT_GET(DT_NODELABEL(pwm8))},
		{"adc.battery", DEVICE_DT_GET(DT_ALIAS(volt_sensor0))},
		{"sensor.bmi088.accel", DEVICE_DT_GET(DT_ALIAS(accel0))},
		{"sensor.bmi088.gyro", DEVICE_DT_GET(DT_ALIAS(gyro0))},
		{"sensor.ist8310", DEVICE_DT_GET(DT_ALIAS(magn0))},
	};

	for (const device_check &check : checks) {
		report_device_ready(reporter, check.name, check.device);
	}

	if (gpio_is_ready_dt(&user_button)) {
		reporter.record(result_kind::success, "gpio.user_button",
				"GPIO controller initialized; button not exercised");
	} else {
		reporter.record(result_kind::error, "gpio.user_button", "GPIO device is not ready");
	}

	reporter.info("safety.exclusions",
		      "laser PWM, IMU-heater PWM, and watchdog are intentionally not exposed");
}

void validate_entropy(reporter &reporter)
{
	const struct device *const entropy = DEVICE_DT_GET(DT_CHOSEN(zephyr_entropy));
	struct guarded_buffer {
		uint8_t head_guard;
		uint8_t bytes[16];
		uint8_t tail_guard;
	};
	guarded_buffer first = {0x3c, {0}, 0x5a};
	guarded_buffer second = {0xc3, {0}, 0xa5};

	if (!device_is_ready(entropy)) {
		reporter.record(result_kind::error, "entropy.rng", "device is not ready");
		return;
	}

	memset(first.bytes, 0x5a, sizeof(first.bytes));
	memset(second.bytes, 0xa5, sizeof(second.bytes));

	int ret = entropy_get_entropy(entropy, first.bytes, sizeof(first.bytes));
	if (ret == 0) {
		ret = entropy_get_entropy(entropy, second.bytes, sizeof(second.bytes));
	}

	if (ret != 0 || first.head_guard != 0x3c || first.tail_guard != 0x5a ||
	    second.head_guard != 0xc3 || second.tail_guard != 0xa5 ||
	    memcmp(first.bytes, second.bytes, sizeof(first.bytes)) == 0) {
		reporter.record(result_kind::error, "entropy.rng",
				"entropy read verification failed", ret);
		return;
	}

	reporter.record(result_kind::success, "entropy.rng",
			"two guarded entropy reads completed and differed");
	reporter.info("entropy.rng", "functional check only; no statistical certification");
}

void validate_battery(reporter &reporter)
{
	const struct device *const battery = DEVICE_DT_GET(DT_ALIAS(volt_sensor0));
	struct sensor_value voltage = {};

	if (!device_is_ready(battery)) {
		reporter.record(result_kind::error, "adc.battery",
				"voltage-divider sensor is not ready");
		return;
	}

	int ret = sensor_sample_fetch(battery);
	if (ret == 0) {
		ret = sensor_channel_get(battery, SENSOR_CHAN_VOLTAGE, &voltage);
	}
	if (ret != 0) {
		reporter.record(result_kind::error, "adc.battery", "voltage sample failed", ret);
		return;
	}

	const int64_t microvolts = sensor_value_to_micro(voltage.val1, voltage.val2);
	const int64_t millivolts = microvolts / 1000LL;
	shell_print(reporter.output(), "[INFO] adc.battery: voltage=%" PRId64 " mV", millivolts);

	if (microvolts < 0 || microvolts > 35000000LL) {
		reporter.record(result_kind::error, "adc.battery",
				"voltage is outside a plausible range");
		return;
	}

	reporter.record(result_kind::success, "adc.battery",
			"ADC and voltage-divider sample completed");
	if (microvolts < 500000LL) {
		reporter.info("adc.battery",
			      "near-zero is expected when no battery input is present");
	}
}

void validate_accelerometer(reporter &reporter)
{
	(void)fetch_three_axis(accelerometer_spec, reporter);
	reporter.info("sensor.motion", "move the board and repeat the command to compare values");
}

void validate_gyroscope(reporter &reporter)
{
	(void)fetch_three_axis(gyroscope_spec, reporter);
	reporter.info("sensor.motion", "rotate the board and repeat the command to compare values");
}

void validate_magnetometer(reporter &reporter)
{
	(void)fetch_three_axis(magnetometer_spec, reporter);
	reporter.info("sensor.motion", "rotate the board and repeat the command to compare values");
}

void validate_can_internal(reporter &reporter)
{
	const struct device *const can = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
	struct can_filter filter = {};
	struct can_frame frame = {};
	struct can_frame received = {};
	int filter_id = -1;
	bool started = false;
	int test_ret = 0;

	if (!device_is_ready(can)) {
		reporter.record(result_kind::error, "can.can1.internal_loopback",
				"device is not ready");
		return;
	}

	test_ret = can_stop(can);
	if (test_ret == -EALREADY) {
		test_ret = 0;
	}
	if (test_ret == 0) {
		test_ret = can_set_mode(can, CAN_MODE_LOOPBACK);
	}
	if (test_ret == 0) {
		test_ret = can_set_bitrate(can, 1000000U);
	}
	if (test_ret == 0) {
		test_ret = can_start(can);
		started = test_ret == 0;
	}

	filter.flags = 0U;
	filter.id = 0x321U;
	filter.mask = CAN_STD_ID_MASK;
	if (test_ret == 0) {
		k_msgq_purge(&can_loopback_msgq);
		filter_id = can_add_rx_filter_msgq(can, &can_loopback_msgq, &filter);
		if (filter_id < 0) {
			test_ret = filter_id;
		}
	}

	frame.flags = 0U;
	frame.id = filter.id;
	frame.dlc = 8U;
	const uint8_t payload[8] = {0x00, 0x55, 0xaa, 0xff, 0x12, 0x34, 0x56, 0x78};
	memcpy(frame.data, payload, sizeof(payload));

	if (test_ret == 0) {
		test_ret = can_loopback_send_bounded(can, frame);
	}
	if (test_ret == 0) {
		test_ret = k_msgq_get(&can_loopback_msgq, &received, K_MSEC(500));
	}
	if (test_ret == 0 && !can_frames_equal(frame, received)) {
		test_ret = -EIO;
	}

	if (filter_id >= 0) {
		can_remove_rx_filter(can, filter_id);
	}

	int cleanup_ret = 0;
	if (started) {
		cleanup_ret = can_stop(can);
	}
	const int mode_ret = can_set_mode(can, 0U);
	if (cleanup_ret == 0 && mode_ret != 0) {
		cleanup_ret = mode_ret;
	}

	if (test_ret != 0) {
		reporter.record(result_kind::error, "can.can1.internal_loopback",
				"frame send, receive, or comparison failed", test_ret);
	} else if (cleanup_ret != 0) {
		reporter.record(result_kind::error, "can.can1.internal_loopback",
				"test passed but controller cleanup failed", cleanup_ret);
	} else {
		reporter.record(result_kind::success, "can.can1.internal_loopback",
				"frame matched");
	}

	reporter.info("can.can1.internal_loopback",
		      "transceiver, connector, termination, and physical bus are not tested");
}

void validate_rgb_colors(reporter &reporter)
{
	for (const struct pwm_dt_spec &led : rgb_leds) {
		if (!pwm_is_ready_dt(&led)) {
			reporter.record(result_kind::error, "pwm.rgb.api",
					"PWM device is not ready");
			return;
		}
	}

	int ret = turn_rgb_off();
	const uint32_t period = rgb_leds[0].period;
	const uint32_t quarter = period / 4U;
	const uint32_t low_white = period / 10U;
	const struct {
		const char *name;
		uint32_t green;
		uint32_t red;
		uint32_t blue;
	} colors[] = {
		{"green", quarter, 0U, 0U},
		{"red", 0U, quarter, 0U},
		{"blue", 0U, 0U, quarter},
		{"white", low_white, low_white, low_white},
	};

	for (const auto &color : colors) {
		if (ret != 0) {
			break;
		}
		shell_print(reporter.output(), "[INFO] pwm.rgb: showing %s", color.name);
		ret = set_rgb(color.green, color.red, color.blue);
		if (ret == 0) {
			k_sleep(K_MSEC(color_step_ms));
		}
	}

	const int cleanup_ret = turn_rgb_off();
	if (ret != 0) {
		reporter.record(result_kind::error, "pwm.rgb.api", "failed to drive RGB channels",
				ret);
	} else if (cleanup_ret != 0) {
		reporter.record(result_kind::error, "pwm.rgb.api",
				"failed to turn RGB channels off", cleanup_ret);
	} else {
		reporter.record(result_kind::success, "pwm.rgb.api",
				"all three PWM channels were driven");
		reporter.info("pwm.rgb.operator",
			      "visually confirm the green, red, blue, white order");
	}
}

void validate_button(reporter &reporter, uint32_t timeout_ms)
{
	if (!gpio_is_ready_dt(&user_button)) {
		reporter.record(result_kind::error, "gpio.user_button", "GPIO device is not ready");
		return;
	}

	int ret = gpio_pin_configure_dt(&user_button, GPIO_INPUT);
	if (ret != 0) {
		reporter.record(result_kind::error, "gpio.user_button", "pin configuration failed",
				ret);
		return;
	}

	k_sem_reset(&button_sem);
	gpio_init_callback(&button_callback, button_callback_handler, BIT(user_button.pin));
	ret = gpio_add_callback(user_button.port, &button_callback);
	if (ret != 0) {
		reporter.record(result_kind::error, "gpio.user_button",
				"callback registration failed", ret);
		return;
	}

	ret = gpio_pin_interrupt_configure_dt(&user_button, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		const int remove_ret = gpio_remove_callback(user_button.port, &button_callback);
		reporter.record(result_kind::error, "gpio.user_button",
				"interrupt configuration failed", ret);
		if (remove_ret != 0) {
			reporter.record(result_kind::error, "gpio.user_button",
					"failed to remove callback after setup error", remove_ret);
		}
		return;
	}

	shell_print(reporter.output(), "[INFO] gpio.user_button: press within %" PRIu32 " ms",
		    timeout_ms);
	const int wait_ret = k_sem_take(&button_sem, K_MSEC(timeout_ms));

	int cleanup_ret = gpio_pin_interrupt_configure_dt(&user_button, GPIO_INT_DISABLE);
	const int remove_ret = gpio_remove_callback(user_button.port, &button_callback);
	if (cleanup_ret == 0 && remove_ret != 0) {
		cleanup_ret = remove_ret;
	}

	if (cleanup_ret != 0) {
		reporter.record(result_kind::error, "gpio.user_button", "callback cleanup failed",
				cleanup_ret);
	} else if (wait_ret == 0) {
		reporter.record(result_kind::success, "gpio.user_button",
				"edge interrupt received");
	} else if (wait_ret == -EAGAIN) {
		reporter.record(result_kind::not_tested, "gpio.user_button",
				"no operator press received");
	} else {
		reporter.record(result_kind::error, "gpio.user_button", "button wait failed",
				wait_ret);
	}
}

void validate_i2c_readiness(reporter &reporter)
{
	report_device_ready(reporter, "i2c.i2c1", DEVICE_DT_GET(DT_NODELABEL(i2c1)));
	report_device_ready(reporter, "i2c.i2c2", DEVICE_DT_GET(DT_NODELABEL(i2c2)));
	reporter.info(
		"i2c.fixture",
		"readiness is not a bus transaction; use the I2C shell only with a known target");
}

namespace
{

void print_rgb_interactive_help(const struct shell *shell)
{
	shell_print(shell, "Commands:");
	shell_print(shell, "  set <red 0-100> <green 0-100> <blue 0-100>");
	shell_print(shell, "  off | status | help | pass | fail | quit");
	shell_print(shell,
		    "The setting remains active until changed, turned off, or the session exits.");
	shell_print(shell, "Tab completion and history are unavailable inside this session.");
}

} // namespace

namespace
{

int run_three_axis_interactive(const struct shell *shell, const char *test_id,
			       const three_axis_sensor_spec &spec)
{
	const test_descriptor *const test = find_test(test_id);
	if (test == nullptr) {
		return -ENOENT;
	}

	reporter reporter(shell);
	reporter.begin_test(*test);
	if (!device_is_ready(spec.sensor)) {
		reporter.record(result_kind::error, spec.result_name, "device is not ready");
		reporter.end_test();
		return reporter.finish();
	}

	int64_t previous[3] = {};
	int ret = read_three_axis_interactive(spec, previous);
	if (ret != 0) {
		reporter.record(result_kind::error, spec.result_name,
				"initial sample fetch or channel read failed", ret);
		reporter.end_test();
		return reporter.finish();
	}

	print_three_axis(spec, previous, reporter);
	shell_print(shell,
		    "Sensor stream active at %u ms; changed values are reported. Ctrl+C stops.",
		    sensor_interactive_period_ms);

	bool observed_change = false;
	uint8_t input_buffer[8] = {};
	int64_t next_sample_ms = k_uptime_get() + sensor_interactive_period_ms;

	for (;;) {
		const int64_t remaining_ms = next_sample_ms - k_uptime_get();
		const k_timeout_t timeout = remaining_ms > 0 ? K_MSEC(remaining_ms) : K_NO_WAIT;
		ret = shell_readline(shell, input_buffer, sizeof(input_buffer), timeout);
		if (ret == -ECANCELED) {
			break;
		}
		if (ret < 0 && ret != -ETIMEDOUT) {
			reporter.record(result_kind::error, spec.result_name,
					"interactive input failed", ret);
			break;
		}

		if (k_uptime_get() < next_sample_ms) {
			continue;
		}

		int64_t current[3] = {};
		ret = read_three_axis_interactive(spec, current);
		next_sample_ms = k_uptime_get() + sensor_interactive_period_ms;
		if (ret != 0) {
			reporter.record(result_kind::error, spec.result_name,
					"interactive sample fetch or channel read failed", ret);
			break;
		}

		for (size_t i = 0U; i < ARRAY_SIZE(current); i++) {
			if (current[i] != previous[i]) {
				print_fixed_point(reporter, spec.axis_names[i], current[i],
						  spec.unit);
				previous[i] = current[i];
				observed_change = true;
			}
		}
	}

	if (!reporter.current_test_has_error()) {
		if (observed_change) {
			reporter.record(result_kind::success, spec.result_name,
					"XYZ changes were observed");
		} else {
			reporter.record(result_kind::not_tested, spec.result_name,
					"session ended before a sensor change was observed",
					-ECANCELED);
		}
	}
	reporter.end_test();
	return reporter.finish();
}

} // namespace

int run_accelerometer_interactive(const struct shell *shell)
{
	return run_three_axis_interactive(shell, "accel.interactive", accelerometer_spec);
}

int run_gyroscope_interactive(const struct shell *shell)
{
	return run_three_axis_interactive(shell, "gyro.interactive", gyroscope_spec);
}

int run_magnetometer_interactive(const struct shell *shell)
{
	return run_three_axis_interactive(shell, "magn.interactive", magnetometer_spec);
}

int run_button_interactive(const struct shell *shell)
{
	const test_descriptor *const test = find_test("button.interactive");
	if (test == nullptr) {
		return -ENOENT;
	}

	reporter reporter(shell);
	reporter.begin_test(*test);
	if (!gpio_is_ready_dt(&user_button)) {
		reporter.record(result_kind::error, "gpio.user_button", "GPIO device is not ready");
		reporter.end_test();
		return reporter.finish();
	}

	int ret = gpio_pin_configure_dt(&user_button, GPIO_INPUT);
	if (ret != 0) {
		reporter.record(result_kind::error, "gpio.user_button", "pin configuration failed",
				ret);
		reporter.end_test();
		return reporter.finish();
	}

	int stable_state = gpio_pin_get_dt(&user_button);
	if (stable_state < 0) {
		reporter.record(result_kind::error, "gpio.user_button", "initial state read failed",
				stable_state);
		reporter.end_test();
		return reporter.finish();
	}

	int candidate_state = stable_state;
	uint32_t candidate_count = 0U;
	bool observed_change = false;
	uint8_t input_buffer[8] = {};
	shell_print(shell, "Button is %s. Press and release it; Ctrl+C stops.",
		    stable_state != 0 ? "pressed" : "released");

	for (;;) {
		ret = shell_readline(shell, input_buffer, sizeof(input_buffer), K_MSEC(50));
		if (ret == -ECANCELED) {
			break;
		}
		if (ret < 0 && ret != -ETIMEDOUT) {
			reporter.record(result_kind::error, "gpio.user_button",
					"interactive input failed", ret);
			break;
		}

		const int state = gpio_pin_get_dt(&user_button);
		if (state < 0) {
			reporter.record(result_kind::error, "gpio.user_button", "state read failed",
					state);
			break;
		}
		if (state != candidate_state) {
			candidate_state = state;
			candidate_count = 1U;
		} else if (candidate_count < 3U) {
			candidate_count++;
		}
		if (candidate_count >= 3U && stable_state != candidate_state) {
			stable_state = candidate_state;
			observed_change = true;
			shell_print(shell, "Button: %s",
				    stable_state != 0 ? "pressed" : "released");
		}
	}

	if (!reporter.current_test_has_error()) {
		if (observed_change) {
			reporter.record(result_kind::success, "gpio.user_button",
					"debounced state changes were observed");
		} else {
			reporter.record(result_kind::not_tested, "gpio.user_button",
					"session ended before a state change was observed",
					-ECANCELED);
		}
	}
	reporter.end_test();
	return reporter.finish();
}

int run_rgb_interactive(const struct shell *shell)
{
	const test_descriptor *const test = find_test("rgb.interactive");
	if (test == nullptr) {
		return -ENOENT;
	}

	reporter reporter(shell);
	reporter.begin_test(*test);
	for (const struct pwm_dt_spec &led : rgb_leds) {
		if (!pwm_is_ready_dt(&led)) {
			reporter.record(result_kind::error, "pwm.rgb", "PWM device is not ready");
			reporter.end_test();
			return reporter.finish();
		}
	}

	int ret = turn_rgb_off();
	if (ret != 0) {
		reporter.record(result_kind::error, "pwm.rgb", "failed to establish inactive state",
				ret);
		reporter.end_test();
		return reporter.finish();
	}

	uint32_t red = 0U;
	uint32_t green = 0U;
	uint32_t blue = 0U;
	bool active = false;
	bool operated = false;
	bool verdict_recorded = false;
	uint8_t line_buffer[64] = {};
	char *const line = reinterpret_cast<char *>(line_buffer);
	print_rgb_interactive_help(shell);

	for (;;) {
		ret = shell_readline(shell, line_buffer, sizeof(line_buffer), K_FOREVER);
		if (ret == -ECANCELED) {
			break;
		}
		if (ret < 0) {
			reporter.record(result_kind::error, "pwm.rgb", "interactive input failed",
					ret);
			break;
		}

		char *save = nullptr;
		char *command = strtok_r(line, " ", &save);
		if (command == nullptr) {
			continue;
		}
		if (strcmp(command, "help") == 0) {
			print_rgb_interactive_help(shell);
			continue;
		}
		if (strcmp(command, "status") == 0) {
			shell_print(shell, "RGB: red=%u%% green=%u%% blue=%u%% %s", red, green,
				    blue, active ? "active" : "off");
			continue;
		}
		if (strcmp(command, "off") == 0) {
			ret = turn_rgb_off();
			active = false;
			red = green = blue = 0U;
			if (ret != 0) {
				reporter.record(result_kind::error, "pwm.rgb",
						"failed to turn outputs off", ret);
				break;
			}
			continue;
		}
		if (strcmp(command, "pass") == 0 || strcmp(command, "fail") == 0) {
			verdict_recorded = true;
			if (strcmp(command, "pass") == 0 && operated) {
				reporter.record(result_kind::success, "pwm.rgb.operator",
						"operator confirmed RGB behavior");
			} else if (strcmp(command, "pass") == 0) {
				reporter.record(result_kind::not_tested, "pwm.rgb.operator",
						"no nonzero RGB setting was exercised", -EAGAIN);
			} else {
				reporter.record(result_kind::error, "pwm.rgb.operator",
						"operator observed incorrect RGB behavior", -EIO);
			}
			break;
		}
		if (strcmp(command, "quit") == 0) {
			break;
		}
		if (strcmp(command, "set") == 0) {
			char *red_text = strtok_r(nullptr, " ", &save);
			char *green_text = strtok_r(nullptr, " ", &save);
			char *blue_text = strtok_r(nullptr, " ", &save);
			char *extra = strtok_r(nullptr, " ", &save);
			uint32_t new_red = 0U;
			uint32_t new_green = 0U;
			uint32_t new_blue = 0U;
			if (red_text == nullptr || green_text == nullptr || blue_text == nullptr ||
			    extra != nullptr ||
			    parse_bounded_u32(red_text, 0U, 100U, new_red) != 0 ||
			    parse_bounded_u32(green_text, 0U, 100U, new_green) != 0 ||
			    parse_bounded_u32(blue_text, 0U, 100U, new_blue) != 0) {
				ret = turn_rgb_off();
				if (ret != 0) {
					reporter.record(result_kind::error, "pwm.rgb",
							"shutdown after invalid input failed", ret);
					break;
				}
				active = false;
				red = green = blue = 0U;
				shell_error(shell,
					    "Expected: set <red 0-100> <green 0-100> <blue 0-100>");
				continue;
			}

			const uint32_t period = rgb_leds[0].period;
			ret = set_rgb(static_cast<uint32_t>(
					      (static_cast<uint64_t>(period) * new_green) / 100U),
				      static_cast<uint32_t>(
					      (static_cast<uint64_t>(period) * new_red) / 100U),
				      static_cast<uint32_t>(
					      (static_cast<uint64_t>(period) * new_blue) / 100U));
			if (ret != 0) {
				(void)turn_rgb_off();
				reporter.record(result_kind::error, "pwm.rgb",
						"failed to apply RGB setting", ret);
				break;
			}
			red = new_red;
			green = new_green;
			blue = new_blue;
			active = red != 0U || green != 0U || blue != 0U;
			operated = operated || active;
			shell_print(shell, "RGB set: red=%u%% green=%u%% blue=%u%%", red, green,
				    blue);
			continue;
		}

		ret = turn_rgb_off();
		if (ret != 0) {
			reporter.record(result_kind::error, "pwm.rgb",
					"shutdown after unknown command failed", ret);
			break;
		}
		active = false;
		red = green = blue = 0U;
		shell_error(shell, "Unknown command; enter help");
	}

	const int cleanup_ret = turn_rgb_off();
	if (cleanup_ret != 0) {
		reporter.record(result_kind::error, "pwm.rgb", "final cleanup failed", cleanup_ret);
	} else if (!reporter.current_test_has_error() && !verdict_recorded) {
		reporter.record(result_kind::not_tested, "pwm.rgb.operator",
				"session ended without an operator verdict", -ECANCELED);
	}
	reporter.end_test();
	return reporter.finish();
}

} // namespace type_c_validation
