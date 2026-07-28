/*
 * SPDX-FileCopyrightText: 2026 GMaster contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "validation.hpp"

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

namespace type_c_validation
{

namespace
{

int parse_u32(const struct shell *shell, const char *text, uint32_t minimum, uint32_t maximum,
	      uint32_t &value)
{
	const int ret = parse_bounded_u32(text, minimum, maximum, value);
	if (ret == -EINVAL) {
		shell_error(shell, "Invalid integer: %s", text);
	} else if (ret != 0) {
		shell_error(shell, "Value must be between %u and %u", minimum, maximum);
	}

	return ret == 0 ? 0 : -EINVAL;
}

int run_test(const struct shell *shell, const char *id)
{
	const test_descriptor *const test = find_test(id);
	if (test == nullptr || test->function == nullptr) {
		shell_error(shell, "Internal test descriptor is missing: %s", id);
		return -ENOENT;
	}

	reporter reporter(shell);
	reporter.begin_test(*test);
	test->function(reporter);
	reporter.end_test();
	return reporter.finish();
}

void run_catalog_test(reporter &reporter, const test_descriptor &test)
{
	reporter.begin_test(test);
	if (test.function != nullptr) {
		test.function(reporter);
	} else {
		reporter.record(result_kind::error, test.id, "test has no default runner", -ENOSYS);
	}
	reporter.end_test();
}

int record_operator_verdict(reporter &reporter, const char *name, const char *question)
{
	if (reporter.current_test_has_error()) {
		return 0;
	}

	uint8_t response_buffer[16] = {};
	char *const response = reinterpret_cast<char *>(response_buffer);
	shell_print(reporter.output(), "%s", question);
	shell_print(reporter.output(), "Enter yes, no, or skip (Ctrl+C cancels):");

	for (;;) {
		const int ret = shell_readline(reporter.output(), response_buffer,
					       sizeof(response_buffer), K_SECONDS(30));
		if (ret == -ECANCELED) {
			reporter.record(result_kind::not_tested, name,
					"operator cancelled the verdict", -ECANCELED);
			return ret;
		}
		if (ret == -ETIMEDOUT) {
			reporter.record(result_kind::not_tested, name, "operator verdict timed out",
					-ETIMEDOUT);
			return 0;
		}
		if (ret < 0) {
			reporter.record(result_kind::error, name, "failed to read operator verdict",
					ret);
			return ret;
		}
		if (strcmp(response, "yes") == 0 || strcmp(response, "y") == 0) {
			reporter.record(result_kind::success, name,
					"operator confirmed expected behavior");
			return 0;
		}
		if (strcmp(response, "no") == 0 || strcmp(response, "n") == 0) {
			reporter.record(result_kind::error, name,
					"operator observed behavior that did not match", -EIO);
			return 0;
		}
		if (strcmp(response, "skip") == 0 || strcmp(response, "s") == 0) {
			reporter.record(result_kind::not_tested, name,
					"operator did not provide observation evidence", -EAGAIN);
			return 0;
		}

		shell_warn(reporter.output(), "Expected yes, no, or skip");
	}
}

int cmd_info(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return run_test(shell, "info");
}

int cmd_list(const struct shell *shell, size_t argc, char **argv)
{
	const char *const filter = argc == 2U ? argv[1] : nullptr;
	size_t count = 0U;
	const test_descriptor *const catalog = test_catalog(count);
	uint32_t shown = 0U;

	shell_print(shell,
		    "COMMAND                              QUICK  DEPTH                 SETUP");
	for (size_t i = 0U; i < count; i++) {
		const test_descriptor &test = catalog[i];
		if (filter != nullptr && strcmp(filter, test.name) != 0 &&
		    strcmp(filter, test.id) != 0) {
			continue;
		}

		shell_print(shell, "%-36s %-6s %-21s %s", test.command,
			    test_has_flag(test, test_flag_quick)
				    ? "auto"
				    : (test_has_flag(test, test_flag_guided) ? "guided" : "no"),
			    validation_depth_name(test.depth), test.setup);
		shell_print(shell, "  %s", test.description);
		shown++;
	}

	if (shown == 0U) {
		shell_error(shell, "No test matches: %s", filter);
		return -ENOENT;
	}

	return 0;
}

int cmd_quick(const struct shell *shell, size_t argc, char **argv)
{
	const bool guided = argc == 2U;
	if (guided && strcmp(argv[1], "guided") != 0) {
		shell_error(shell, "Mode must be 'guided'");
		return -EINVAL;
	}

	reporter reporter(shell);
	size_t count = 0U;
	const test_descriptor *const catalog = test_catalog(count);

	for (size_t i = 0U; i < count; i++) {
		if (test_has_flag(catalog[i], test_flag_quick)) {
			run_catalog_test(reporter, catalog[i]);
		}
	}

	if (guided) {
		const test_descriptor *rgb = find_test("rgb");
		const test_descriptor *buzzer = find_test("buzzer");
		const test_descriptor *button = find_test("button");

		if (rgb != nullptr) {
			reporter.begin_test(*rgb);
			validate_rgb_colors(reporter);
			const int verdict_ret = record_operator_verdict(
				reporter, "pwm.rgb.operator",
				"Did the LED show green, red, blue, then white?");
			reporter.end_test();
			if (verdict_ret != 0) {
				return reporter.finish();
			}
		}
		if (buzzer != nullptr) {
			reporter.begin_test(*buzzer);
			validate_buzzer_instrumentation(reporter);
			const int verdict_ret = record_operator_verdict(
				reporter, "pwm.buzzer.operator",
				"Did the onboard buzzer produce a clear tone?");
			reporter.end_test();
			if (verdict_ret != 0) {
				return reporter.finish();
			}
		}
		if (button != nullptr) {
			reporter.begin_test(*button);
			validate_button(reporter, 10000U);
			reporter.end_test();
		}
	}

	return reporter.finish();
}

int cmd_clock(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return run_test(shell, "clock");
}

int cmd_devices(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return run_test(shell, "devices");
}

int cmd_entropy(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return run_test(shell, "entropy");
}

int cmd_battery(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return run_test(shell, "battery");
}

int cmd_accel(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return run_test(shell, "accel");
}

int cmd_gyro(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return run_test(shell, "gyro");
}

int cmd_magn(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return run_test(shell, "magn");
}

int cmd_accel_interactive(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return run_accelerometer_interactive(shell);
}

int cmd_gyro_interactive(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return run_gyroscope_interactive(shell);
}

int cmd_magn_interactive(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return run_magnetometer_interactive(shell);
}

int cmd_uart(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	if (strcmp(argv[1], "1") == 0) {
		const test_descriptor *const test = find_test("uart1");
		reporter reporter(shell);
		reporter.begin_test(*test);
		validate_uart_loopback(reporter, uart_port::usart1);
		reporter.end_test();
		return reporter.finish();
	}
	if (strcmp(argv[1], "6") == 0) {
		const test_descriptor *const test = find_test("uart6");
		reporter reporter(shell);
		reporter.begin_test(*test);
		validate_uart_loopback(reporter, uart_port::usart6);
		reporter.end_test();
		return reporter.finish();
	}

	shell_error(shell, "UART must be 1 or 6");
	return -EINVAL;
}

int cmd_spi2(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return run_test(shell, "spi2");
}

int cmd_can(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	if (strcmp(argv[1], "internal") == 0) {
		return run_test(shell, "can.internal");
	}
	if (strcmp(argv[1], "physical") == 0) {
		return run_test(shell, "can.physical");
	}

	shell_error(shell, "CAN mode must be internal or physical");
	return -EINVAL;
}

int cmd_i2c(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return run_test(shell, "i2c");
}

int cmd_rgb(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	const test_descriptor *const test = find_test("rgb");
	reporter reporter(shell);
	reporter.begin_test(*test);
	validate_rgb_colors(reporter);
	record_operator_verdict(reporter, "pwm.rgb.operator",
				"Did the LED show green, red, blue, then white?");
	reporter.end_test();
	return reporter.finish();
}

int cmd_rgb_interactive(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return run_rgb_interactive(shell);
}

int cmd_button(const struct shell *shell, size_t argc, char **argv)
{
	uint32_t timeout_ms = 10000U;
	if (argc == 2U) {
		const int ret = parse_u32(shell, argv[1], 100U, 30000U, timeout_ms);
		if (ret != 0) {
			return ret;
		}
	}

	const test_descriptor *const test = find_test("button");
	reporter reporter(shell);
	reporter.begin_test(*test);
	validate_button(reporter, timeout_ms);
	reporter.end_test();
	return reporter.finish();
}

int cmd_button_interactive(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return run_button_interactive(shell);
}

int cmd_buzzer(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	const test_descriptor *const test = find_test("buzzer");
	reporter reporter(shell);
	reporter.begin_test(*test);
	validate_buzzer_instrumentation(reporter);
	record_operator_verdict(reporter, "pwm.buzzer.operator",
				"Did the onboard buzzer produce a clear tone?");
	reporter.end_test();
	return reporter.finish();
}

int cmd_buzzer_interactive(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return run_buzzer_interactive(shell);
}

int cmd_buzzer_score(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return run_buzzer_score(shell);
}

int cmd_servo(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	if (strcmp(argv[1], "unloaded") != 0) {
		shell_error(shell, "The literal 'unloaded' confirmation is required");
		return -EINVAL;
	}

	uint32_t channel = 0U;
	const int ret = parse_u32(shell, argv[2], 1U, 7U, channel);
	if (ret != 0) {
		return ret;
	}
	return run_servo_interactive(shell, channel);
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_accel,
			       SHELL_CMD_ARG(interactive, NULL,
					     "Report acceleration changes until Ctrl+C.",
					     cmd_accel_interactive, 1, 0),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_gyro,
			       SHELL_CMD_ARG(interactive, NULL,
					     "Report angular-rate changes until Ctrl+C.",
					     cmd_gyro_interactive, 1, 0),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_magn,
			       SHELL_CMD_ARG(interactive, NULL,
					     "Report magnetic-field changes until Ctrl+C.",
					     cmd_magn_interactive, 1, 0),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_rgb,
			       SHELL_CMD_ARG(interactive, NULL,
					     "Adjust RGB duty until the session exits.",
					     cmd_rgb_interactive, 1, 0),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_button,
			       SHELL_CMD_ARG(interactive, NULL,
					     "Report button state changes until Ctrl+C.",
					     cmd_button_interactive, 1, 0),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_buzzer,
			       SHELL_CMD_ARG(interactive, NULL,
					     "Adjust buzzer frequency and duty until exit.",
					     cmd_buzzer_interactive, 1, 0),
			       SHELL_CMD_ARG(score, NULL, buzzer_score_help, cmd_buzzer_score, 1,
					     0),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_test,
	SHELL_CMD_ARG(info, NULL, "Show board, firmware, shell, and safety information.", cmd_info,
		      1, 0),
	SHELL_CMD_ARG(list, NULL,
		      "List all tests or filter by peripheral/function. Usage: test list [name]",
		      cmd_list, 1, 1),
	SHELL_CMD_ARG(quick, NULL, "Run no-fixture checks. Usage: test quick [guided]", cmd_quick,
		      1, 1),
	SHELL_CMD_ARG(clock, NULL, "Check configured clock and kernel cadence.", cmd_clock, 1, 0),
	SHELL_CMD_ARG(devices, NULL, "Check curated onboard device initialization.", cmd_devices, 1,
		      0),
	SHELL_CMD_ARG(entropy, NULL, "Perform guarded functional entropy reads.", cmd_entropy, 1,
		      0),
	SHELL_CMD_ARG(battery, NULL, "Read the battery voltage divider.", cmd_battery, 1, 0),
	SHELL_CMD_ARG(accel, &sub_accel,
		      "Read or monitor BMI088 acceleration. Usage: test accel [interactive]",
		      cmd_accel, 1, 0),
	SHELL_CMD_ARG(gyro, &sub_gyro,
		      "Read or monitor BMI088 angular rate. Usage: test gyro [interactive]",
		      cmd_gyro, 1, 0),
	SHELL_CMD_ARG(magn, &sub_magn,
		      "Read or monitor IST8310 magnetic field. Usage: test magn [interactive]",
		      cmd_magn, 1, 0),
	SHELL_CMD_ARG(uart, NULL, "Run a physical 115200 8N1 loopback. Usage: test uart <1|6>",
		      cmd_uart, 2, 0),
	SHELL_CMD_ARG(spi2, NULL, "Run the PB15-MOSI to PB14-MISO loopback.", cmd_spi2, 1, 0),
	SHELL_CMD_ARG(can, NULL, "Run CAN validation. Usage: test can <internal|physical>", cmd_can,
		      2, 0),
	SHELL_CMD_ARG(i2c, NULL, "Report I2C1 and I2C2 controller readiness.", cmd_i2c, 1, 0),
	SHELL_CMD_ARG(rgb, &sub_rgb,
		      "Show colors or enter live control. Usage: test rgb [interactive]", cmd_rgb,
		      1, 0),
	SHELL_CMD_ARG(button, &sub_button,
		      "Wait or monitor. Usage: test button [timeout_ms|interactive]", cmd_button, 1,
		      1),
	SHELL_CMD_ARG(buzzer, &sub_buzzer,
		      "Play a tone or enter a special mode. Usage: test buzzer [interactive|score]",
		      cmd_buzzer, 1, 0),
	SHELL_CMD_ARG(servo, NULL, "Unloaded signal control only. Usage: test servo unloaded <1-7>",
		      cmd_servo, 3, 0),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(test, &sub_test,
		   "RoboMaster Type C validation. Run 'test list' or 'test quick'.", NULL);

} // namespace

} // namespace type_c_validation
