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
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

namespace type_c_validation
{

namespace
{

CAN_MSGQ_DEFINE(can_physical_msgq, 2);
K_SEM_DEFINE(can_physical_tx_sem, 0, 1);
int can_physical_tx_result;

void can_physical_tx_callback(const struct device *device, int error, void *user_data)
{
	ARG_UNUSED(device);
	ARG_UNUSED(user_data);
	can_physical_tx_result = error;
	k_sem_give(&can_physical_tx_sem);
}

int can_physical_send_bounded(const struct device *can, const struct can_frame &frame)
{
	k_sem_reset(&can_physical_tx_sem);
	can_physical_tx_result = -EINPROGRESS;
	int ret = can_send(can, &frame, K_MSEC(200), can_physical_tx_callback, nullptr);
	if (ret != 0) {
		return ret;
	}

	ret = k_sem_take(&can_physical_tx_sem, K_MSEC(500));
	if (ret != 0) {
		return -ETIMEDOUT;
	}
	return can_physical_tx_result;
}

constexpr uint8_t loopback_pattern[] = {
	0x00, 0x55, 0xaa, 0xff, 0x12, 0x34, 0x7e, 0x81,
};

struct servo_channel {
	const char *name;
	const struct device *pwm;
	uint32_t channel;
};

const struct device *const servo_pwm1 = DEVICE_DT_GET(DT_NODELABEL(pwm1));
const struct device *const servo_pwm8 = DEVICE_DT_GET(DT_NODELABEL(pwm8));
const struct pwm_dt_spec buzzer = PWM_DT_SPEC_GET(DT_ALIAS(pwm_buzzer));
const servo_channel servo_channels[] = {
	{"PWM1", servo_pwm1, 1U}, {"PWM2", servo_pwm1, 2U}, {"PWM3", servo_pwm1, 3U},
	{"PWM4", servo_pwm1, 4U}, {"PWM5", servo_pwm8, 1U}, {"PWM6", servo_pwm8, 2U},
	{"PWM7", servo_pwm8, 3U},
};

void drain_uart(const struct device *uart)
{
	unsigned char byte = 0U;
	const int64_t deadline = k_uptime_get() + 10;

	for (size_t count = 0; count < 32U && k_uptime_get() < deadline; count++) {
		if (uart_poll_in(uart, &byte) != 0) {
			break;
		}
	}
}

int uart_exchange(const struct device *uart)
{
	unsigned char byte = 0U;

	for (uint8_t expected : loopback_pattern) {
		uart_poll_out(uart, expected);
		const int64_t deadline = k_uptime_get() + 200;
		int ret;

		do {
			ret = uart_poll_in(uart, &byte);
			if (ret != 0) {
				k_yield();
			}
		} while (ret != 0 && k_uptime_get() < deadline);

		if (ret != 0) {
			return -ETIMEDOUT;
		}
		if (byte != expected) {
			return -EIO;
		}
	}

	return 0;
}

int spi_exchange(const struct device *spi, uint32_t frequency)
{
	uint8_t tx_data[sizeof(loopback_pattern)] = {};
	uint8_t rx_data[sizeof(loopback_pattern)] = {};
	memcpy(tx_data, loopback_pattern, sizeof(tx_data));

	struct spi_config config = {};
	config.frequency = frequency;
	config.operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8);
	config.slave = 0U;

	struct spi_buf tx_buffer = {tx_data, sizeof(tx_data)};
	struct spi_buf rx_buffer = {rx_data, sizeof(rx_data)};
	struct spi_buf_set tx_set = {&tx_buffer, 1U};
	struct spi_buf_set rx_set = {&rx_buffer, 1U};

	const int ret = spi_transceive(spi, &config, &tx_set, &rx_set);
	if (ret != 0) {
		return ret;
	}

	return memcmp(tx_data, rx_data, sizeof(tx_data)) == 0 ? 0 : -EIO;
}

bool can_frames_equal(const struct can_frame &expected, const struct can_frame &received)
{
	return expected.flags == received.flags && expected.id == received.id &&
	       expected.dlc == received.dlc &&
	       memcmp(expected.data, received.data, can_dlc_to_bytes(expected.dlc)) == 0;
}

int can_exchange(const struct device *transmitter, const struct device *receiver, uint32_t id)
{
	struct can_filter filter = {};
	struct can_frame frame = {};
	struct can_frame received = {};

	filter.flags = 0U;
	filter.id = id;
	filter.mask = CAN_STD_ID_MASK;

	k_msgq_purge(&can_physical_msgq);
	const int filter_id = can_add_rx_filter_msgq(receiver, &can_physical_msgq, &filter);
	if (filter_id < 0) {
		return filter_id;
	}

	frame.flags = 0U;
	frame.id = id;
	frame.dlc = 8U;
	memcpy(frame.data, loopback_pattern, sizeof(loopback_pattern));

	int ret = can_physical_send_bounded(transmitter, frame);
	if (ret == 0) {
		ret = k_msgq_get(&can_physical_msgq, &received, K_SECONDS(1));
	}

	can_remove_rx_filter(receiver, filter_id);
	if (ret != 0) {
		return ret;
	}

	return can_frames_equal(frame, received) ? 0 : -EIO;
}

void print_can_state(const reporter &reporter, const char *name, const struct device *can)
{
	enum can_state state;
	struct can_bus_err_cnt error_count = {};
	const int ret = can_get_state(can, &state, &error_count);

	if (ret == 0) {
		shell_print(reporter.output(), "[INFO] %s: state=%d tx_errors=%u rx_errors=%u",
			    name, static_cast<int>(state), error_count.tx_err_cnt,
			    error_count.rx_err_cnt);
	}
}

int prepare_can(const struct device *can, bool &started)
{
	started = false;
	int ret = can_stop(can);
	if (ret == -EALREADY) {
		ret = 0;
	}
	if (ret == 0) {
		ret = can_set_mode(can, 0U);
	}
	if (ret == 0) {
		ret = can_set_bitrate(can, 1000000U);
	}
	if (ret == 0) {
		ret = can_start(can);
		started = ret == 0;
	}

	return ret;
}

int cleanup_can(const struct device *can, bool started)
{
	int ret = 0;
	if (started) {
		ret = can_stop(can);
	}

	const int mode_ret = can_set_mode(can, 0U);
	if (ret == 0 && mode_ret != 0) {
		ret = mode_ret;
	}
	return ret;
}

int set_pwm(const struct device *pwm, uint32_t channel, uint32_t period, uint32_t pulse)
{
	return pwm_set(pwm, channel, period, pulse, PWM_POLARITY_NORMAL);
}

int set_buzzer(uint32_t period, uint32_t pulse)
{
	return pwm_set_dt(&buzzer, period, pulse);
}

int disable_all_servos()
{
	int first_error = 0;

	for (const servo_channel &servo : servo_channels) {
		const int ret = set_pwm(servo.pwm, servo.channel, PWM_MSEC(20), 0U);
		if (ret != 0 && first_error == 0) {
			first_error = ret;
		}
	}
	return first_error;
}

} // namespace

void validate_uart_loopback(reporter &reporter, uart_port port)
{
	const struct device *uart;
	const char *name;
	const char *fixture;

	if (port == uart_port::usart1) {
		uart = DEVICE_DT_GET(DT_NODELABEL(usart1));
		name = "uart.usart1.loopback";
		fixture = "jumper PA9 TX to PB7 RX (connector marked UART2)";
	} else {
		uart = DEVICE_DT_GET(DT_NODELABEL(usart6));
		name = "uart.usart6.loopback";
		fixture = "jumper PG14 TX to PG9 RX (connector marked UART1)";
	}

	reporter.info(name, fixture);
	if (!device_is_ready(uart)) {
		reporter.record(result_kind::error, name, "device is not ready");
		return;
	}

	struct uart_config original = {};
	int ret = uart_config_get(uart, &original);
	if (ret != 0) {
		reporter.record(result_kind::error, name, "failed to read current configuration",
				ret);
		return;
	}

	struct uart_config test_config = {};
	test_config.baudrate = 115200U;
	test_config.parity = UART_CFG_PARITY_NONE;
	test_config.stop_bits = UART_CFG_STOP_BITS_1;
	test_config.data_bits = UART_CFG_DATA_BITS_8;
	test_config.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;

	const int configure_ret = uart_configure(uart, &test_config);
	ret = configure_ret;
	if (ret == 0) {
		drain_uart(uart);
		ret = uart_exchange(uart);
	}

	const int restore_ret = uart_configure(uart, &original);
	if (ret != 0) {
		reporter.record(result_kind::error, name,
				configure_ret != 0 ? "failed to configure 115200 8N1"
						   : "115200 8N1 loopback did not match",
				ret);
		if (restore_ret != 0) {
			reporter.record(result_kind::error, name,
					"failed to restore UART configuration", restore_ret);
		}
	} else if (restore_ret != 0) {
		reporter.record(result_kind::error, name, "test passed but UART restoration failed",
				restore_ret);
	} else {
		reporter.record(result_kind::success, name, "115200 8N1 polling loopback matched");
	}
}

void validate_spi2_loopback(reporter &reporter)
{
	const struct device *const spi2 = DEVICE_DT_GET(DT_NODELABEL(spi2));

	reporter.info("fixture.spi2", "jumper PB15 MOSI to PB14 MISO; CS is not exercised");
	if (!device_is_ready(spi2)) {
		reporter.record(result_kind::error, "spi.spi2.loopback", "device is not ready");
		return;
	}

	const uint32_t frequencies[] = {500000U, 4000000U};
	for (uint32_t frequency : frequencies) {
		const int ret = spi_exchange(spi2, frequency);
		if (ret != 0) {
			reporter.record(result_kind::error, "spi.spi2.loopback",
					"transfer did not match", ret);
			return;
		}
		shell_print(reporter.output(),
			    "[INFO] spi.spi2.loopback: matched at %" PRIu32 " Hz", frequency);
	}

	reporter.record(result_kind::success, "spi.spi2.loopback",
			"MOSI-to-MISO transfers matched at two frequencies");
}

void validate_can_physical(reporter &reporter)
{
	const struct device *const can1 = DEVICE_DT_GET(DT_NODELABEL(can1));
	const struct device *const can2 = DEVICE_DT_GET(DT_NODELABEL(can2));
	bool can1_started = false;
	bool can2_started = false;

	reporter.info("fixture.can", "connect CAN1 H/L/GND to CAN2 H/L/GND; add no termination");
	if (!device_is_ready(can1) || !device_is_ready(can2)) {
		reporter.record(result_kind::error, "can.physical", "CAN controller is not ready");
		return;
	}

	int setup_ret = prepare_can(can1, can1_started);
	if (setup_ret == 0) {
		setup_ret = prepare_can(can2, can2_started);
	}

	int first_exchange_ret = 0;
	int second_exchange_ret = 0;
	if (setup_ret == 0) {
		first_exchange_ret = can_exchange(can1, can2, 0x521U);
		if (first_exchange_ret == 0) {
			second_exchange_ret = can_exchange(can2, can1, 0x522U);
		}
	}

	if (first_exchange_ret != 0 || second_exchange_ret != 0) {
		print_can_state(reporter, "can1", can1);
		print_can_state(reporter, "can2", can2);
	}

	int cleanup_ret = cleanup_can(can1, can1_started);
	const int can2_cleanup_ret = cleanup_can(can2, can2_started);
	if (cleanup_ret == 0 && can2_cleanup_ret != 0) {
		cleanup_ret = can2_cleanup_ret;
	}

	if (setup_ret != 0) {
		reporter.record(result_kind::error, "can.physical", "controller setup failed",
				setup_ret);
		if (cleanup_ret != 0) {
			reporter.record(result_kind::error, "can.physical",
					"controller cleanup failed", cleanup_ret);
		}
		return;
	}
	if (cleanup_ret != 0) {
		reporter.record(result_kind::error, "can.physical", "controller cleanup failed",
				cleanup_ret);
		return;
	}
	if (first_exchange_ret != 0) {
		reporter.record(result_kind::error, "can.physical", "CAN1-to-CAN2 exchange failed",
				first_exchange_ret);
		return;
	}

	reporter.record(result_kind::success, "can.can1_to_can2", "1 Mbit/s frame matched");
	if (second_exchange_ret != 0) {
		reporter.record(result_kind::error, "can.physical", "CAN2-to-CAN1 exchange failed",
				second_exchange_ret);
		return;
	}
	reporter.record(result_kind::success, "can.can2_to_can1", "1 Mbit/s frame matched");
}

void validate_buzzer_instrumentation(reporter &reporter)
{
	const uint32_t period = buzzer.period;
	const uint32_t pulse = period / 4U;

	if (!pwm_is_ready_dt(&buzzer)) {
		reporter.record(result_kind::error, "pwm.buzzer.instrumentation",
				"PWM controller is not ready");
		return;
	}

	shell_print(reporter.output(),
		    "[INFO] pwm.buzzer: period=%" PRIu32 " ns pulse=%" PRIu32
		    " ns duration=1000 ms",
		    period, pulse);
	int ret = set_buzzer(period, pulse);
	if (ret == 0) {
		k_sleep(K_SECONDS(1));
	}

	const int cleanup_ret = set_buzzer(period, 0U);
	if (ret != 0) {
		reporter.record(result_kind::error, "pwm.buzzer.instrumentation",
				"failed to generate bounded 4 kHz tone", ret);
	} else if (cleanup_ret != 0) {
		reporter.record(result_kind::error, "pwm.buzzer.instrumentation",
				"failed to disable buzzer output", cleanup_ret);
	} else {
		reporter.record(result_kind::success, "pwm.buzzer.instrumentation",
				"bounded 4 kHz, 25 percent duty tone generated and disabled");
	}
}

namespace
{

void print_buzzer_interactive_help(const struct shell *shell)
{
	shell_print(shell, "Commands:");
	shell_print(shell, "  set <frequency 100-5000 Hz> <duty 1-50 percent>");
	shell_print(shell, "  off | status | help | pass | fail | quit");
	shell_print(shell,
		    "The setting remains active until changed, turned off, or the session exits.");
	shell_print(shell, "Tab completion and history are unavailable inside this session.");
}

void print_servo_interactive_help(const struct shell *shell, uint32_t channel)
{
	shell_print(shell, "PWM%u signal only; do not attach a servo or load.", channel);
	shell_print(shell, "Commands:");
	shell_print(shell, "  pulse <1000-2000 microseconds>");
	shell_print(shell, "  off | status | help | pass | fail | quit");
	shell_print(shell,
		    "The setting remains active until changed, turned off, or the session exits.");
	shell_print(shell, "Tab completion and history are unavailable inside this session.");
}

} // namespace

int run_buzzer_interactive(const struct shell *shell)
{
	const test_descriptor *const test = find_test("buzzer.interactive");
	if (test == nullptr) {
		return -ENOENT;
	}

	reporter reporter(shell);
	reporter.begin_test(*test);
	if (!pwm_is_ready_dt(&buzzer)) {
		reporter.record(result_kind::error, "pwm.buzzer", "PWM controller is not ready");
		reporter.end_test();
		return reporter.finish();
	}

	uint32_t frequency = 4000U;
	uint32_t duty = 25U;
	uint32_t period = buzzer.period;
	bool active = false;
	bool operated = false;
	bool verdict_recorded = false;
	uint8_t line_buffer[64] = {};
	char *const line = reinterpret_cast<char *>(line_buffer);
	int ret = set_buzzer(period, 0U);
	if (ret != 0) {
		reporter.record(result_kind::error, "pwm.buzzer",
				"failed to establish inactive state", ret);
		reporter.end_test();
		return reporter.finish();
	}
	print_buzzer_interactive_help(shell);

	for (;;) {
		ret = shell_readline(shell, line_buffer, sizeof(line_buffer), K_FOREVER);
		if (ret == -ECANCELED) {
			break;
		}
		if (ret < 0) {
			reporter.record(result_kind::error, "pwm.buzzer",
					"interactive input failed", ret);
			break;
		}

		char *save = nullptr;
		char *command = strtok_r(line, " ", &save);
		if (command == nullptr) {
			continue;
		}
		if (strcmp(command, "help") == 0) {
			print_buzzer_interactive_help(shell);
			continue;
		}
		if (strcmp(command, "status") == 0) {
			shell_print(shell, "Buzzer: frequency=%u Hz duty=%u%% %s", frequency, duty,
				    active ? "active" : "off");
			continue;
		}
		if (strcmp(command, "off") == 0) {
			ret = set_buzzer(period, 0U);
			active = false;
			if (ret != 0) {
				reporter.record(result_kind::error, "pwm.buzzer",
						"failed to turn output off", ret);
				break;
			}
			continue;
		}
		if (strcmp(command, "pass") == 0 || strcmp(command, "fail") == 0) {
			verdict_recorded = true;
			if (strcmp(command, "pass") == 0 && operated) {
				reporter.record(result_kind::success, "pwm.buzzer.operator",
						"operator confirmed buzzer behavior");
			} else if (strcmp(command, "pass") == 0) {
				reporter.record(result_kind::not_tested, "pwm.buzzer.operator",
						"no tone was exercised", -EAGAIN);
			} else {
				reporter.record(result_kind::error, "pwm.buzzer.operator",
						"operator observed incorrect buzzer behavior",
						-EIO);
			}
			break;
		}
		if (strcmp(command, "quit") == 0) {
			break;
		}
		if (strcmp(command, "set") == 0) {
			char *frequency_text = strtok_r(nullptr, " ", &save);
			char *duty_text = strtok_r(nullptr, " ", &save);
			char *extra = strtok_r(nullptr, " ", &save);
			uint32_t new_frequency = 0U;
			uint32_t new_duty = 0U;
			if (frequency_text == nullptr || duty_text == nullptr || extra != nullptr ||
			    parse_bounded_u32(frequency_text, 100U, 5000U, new_frequency) != 0 ||
			    parse_bounded_u32(duty_text, 1U, 50U, new_duty) != 0) {
				ret = set_buzzer(period, 0U);
				if (ret != 0) {
					reporter.record(result_kind::error, "pwm.buzzer",
							"shutdown after invalid input failed", ret);
					break;
				}
				active = false;
				shell_error(shell,
					    "Expected: set <frequency 100-5000> <duty 1-50>");
				continue;
			}

			const uint32_t new_period =
				static_cast<uint32_t>(1000000000ULL / new_frequency);
			const uint32_t pulse = static_cast<uint32_t>(
				(static_cast<uint64_t>(new_period) * new_duty) / 100U);
			ret = set_buzzer(new_period, pulse);
			if (ret != 0) {
				(void)set_buzzer(period, 0U);
				reporter.record(result_kind::error, "pwm.buzzer",
						"failed to apply buzzer setting", ret);
				break;
			}
			frequency = new_frequency;
			duty = new_duty;
			period = new_period;
			active = true;
			operated = true;
			shell_print(shell, "Buzzer set: frequency=%u Hz duty=%u%%", frequency,
				    duty);
			continue;
		}

		ret = set_buzzer(period, 0U);
		if (ret != 0) {
			reporter.record(result_kind::error, "pwm.buzzer",
					"shutdown after unknown command failed", ret);
			break;
		}
		active = false;
		shell_error(shell, "Unknown command; enter help");
	}

	const int cleanup_ret = set_buzzer(period, 0U);
	if (cleanup_ret != 0) {
		reporter.record(result_kind::error, "pwm.buzzer", "final cleanup failed",
				cleanup_ret);
	} else if (!reporter.current_test_has_error() && !verdict_recorded) {
		reporter.record(result_kind::not_tested, "pwm.buzzer.operator",
				"session ended without an operator verdict", -ECANCELED);
	}
	reporter.end_test();
	return reporter.finish();
}

int run_servo_interactive(const struct shell *shell, uint32_t channel)
{
	const test_descriptor *const test = find_test("servo.interactive");
	if (test == nullptr) {
		return -ENOENT;
	}

	reporter reporter(shell);
	reporter.begin_test(*test);
	if (!device_is_ready(servo_pwm1) || !device_is_ready(servo_pwm8)) {
		reporter.record(result_kind::error, "pwm.servo", "PWM controller is not ready");
		reporter.end_test();
		return reporter.finish();
	}
	if (channel == 0U || channel > ARRAY_SIZE(servo_channels)) {
		reporter.record(result_kind::error, "pwm.servo", "channel is outside 1-7", -EINVAL);
		reporter.end_test();
		return reporter.finish();
	}

	const servo_channel &servo = servo_channels[channel - 1U];
	constexpr uint32_t period = PWM_MSEC(20);
	uint32_t pulse_us = 0U;
	bool active = false;
	bool operated = false;
	bool verdict_recorded = false;
	uint8_t line_buffer[48] = {};
	char *const line = reinterpret_cast<char *>(line_buffer);
	int ret = disable_all_servos();
	if (ret != 0) {
		reporter.record(result_kind::error, "pwm.servo",
				"failed to establish inactive state", ret);
		reporter.end_test();
		return reporter.finish();
	}
	print_servo_interactive_help(shell, channel);

	for (;;) {
		ret = shell_readline(shell, line_buffer, sizeof(line_buffer), K_FOREVER);
		if (ret == -ECANCELED) {
			break;
		}
		if (ret < 0) {
			reporter.record(result_kind::error, "pwm.servo", "interactive input failed",
					ret);
			break;
		}

		char *save = nullptr;
		char *command = strtok_r(line, " ", &save);
		if (command == nullptr) {
			continue;
		}
		if (strcmp(command, "help") == 0) {
			print_servo_interactive_help(shell, channel);
			continue;
		}
		if (strcmp(command, "status") == 0) {
			shell_print(shell, "PWM%u: pulse=%u us %s", channel, pulse_us,
				    active ? "active" : "off");
			continue;
		}
		if (strcmp(command, "off") == 0) {
			ret = disable_all_servos();
			active = false;
			pulse_us = 0U;
			if (ret != 0) {
				reporter.record(result_kind::error, "pwm.servo",
						"failed to turn all outputs off", ret);
				break;
			}
			continue;
		}
		if (strcmp(command, "pass") == 0 || strcmp(command, "fail") == 0) {
			verdict_recorded = true;
			if (strcmp(command, "pass") == 0 && operated) {
				reporter.record(result_kind::success, "pwm.servo.instrumentation",
						"operator confirmed the measured signal");
			} else if (strcmp(command, "pass") == 0) {
				reporter.record(result_kind::not_tested,
						"pwm.servo.instrumentation",
						"no pulse setting was exercised", -EAGAIN);
			} else {
				reporter.record(result_kind::error, "pwm.servo.instrumentation",
						"measured signal did not match", -EIO);
			}
			break;
		}
		if (strcmp(command, "quit") == 0) {
			break;
		}
		if (strcmp(command, "pulse") == 0) {
			char *pulse_text = strtok_r(nullptr, " ", &save);
			char *extra = strtok_r(nullptr, " ", &save);
			uint32_t new_pulse_us = 0U;
			if (pulse_text == nullptr || extra != nullptr ||
			    parse_bounded_u32(pulse_text, 1000U, 2000U, new_pulse_us) != 0) {
				ret = disable_all_servos();
				if (ret != 0) {
					reporter.record(result_kind::error, "pwm.servo",
							"shutdown after invalid input failed", ret);
					break;
				}
				active = false;
				pulse_us = 0U;
				shell_error(shell, "Expected: pulse <1000-2000>");
				continue;
			}

			ret = disable_all_servos();
			if (ret == 0) {
				ret = set_pwm(servo.pwm, servo.channel, period,
					      PWM_USEC(new_pulse_us));
			}
			if (ret != 0) {
				(void)disable_all_servos();
				reporter.record(result_kind::error, "pwm.servo",
						"failed to apply servo signal", ret);
				break;
			}
			pulse_us = new_pulse_us;
			active = true;
			operated = true;
			shell_print(shell, "PWM%u set: period=20000 us pulse=%u us", channel,
				    pulse_us);
			continue;
		}

		ret = disable_all_servos();
		if (ret != 0) {
			reporter.record(result_kind::error, "pwm.servo",
					"shutdown after unknown command failed", ret);
			break;
		}
		active = false;
		pulse_us = 0U;
		shell_error(shell, "Unknown command; enter help");
	}

	const int cleanup_ret = disable_all_servos();
	if (cleanup_ret != 0) {
		reporter.record(result_kind::error, "pwm.servo", "final cleanup failed",
				cleanup_ret);
	} else if (!reporter.current_test_has_error() && !verdict_recorded) {
		reporter.record(result_kind::not_tested, "pwm.servo.instrumentation",
				"session ended without a measurement verdict", -ECANCELED);
	}
	reporter.end_test();
	return reporter.finish();
}

} // namespace type_c_validation
