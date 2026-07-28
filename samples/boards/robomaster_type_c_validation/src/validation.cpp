/*
 * SPDX-FileCopyrightText: 2026 GMaster contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "validation.hpp"

#include <errno.h>
#include <inttypes.h>
#include <string.h>

#include <zephyr/shell/shell_string_conv.h>
#include <zephyr/sys/util.h>

namespace type_c_validation
{

namespace
{

const char *result_name(result_kind kind)
{
	switch (kind) {
	case result_kind::success:
		return "SUCCESS";
	case result_kind::error:
		return "ERROR";
	case result_kind::not_tested:
		return "NOT TESTED";
	}

	return "UNKNOWN";
}

const test_descriptor tests[] = {
	{"info", "info", "test info", "Show board, firmware, shell, and safety information.",
	 validation_depth::readiness, "none", test_flag_none, validate_system_info},
	{"clock", "clock", "test clock", "Check configured clock and kernel timing cadence.",
	 validation_depth::functional, "none", test_flag_quick, validate_clock},
	{"devices", "devices", "test devices", "Check curated onboard device initialization.",
	 validation_depth::readiness, "none", test_flag_quick, validate_devices},
	{"entropy", "entropy", "test entropy", "Perform two guarded entropy reads.",
	 validation_depth::functional, "none", test_flag_quick, validate_entropy},
	{"battery", "battery", "test battery", "Read the onboard battery voltage divider.",
	 validation_depth::functional, "none", test_flag_quick, validate_battery},
	{"accel", "accel", "test accel", "Read one BMI088 accelerometer XYZ sample.",
	 validation_depth::functional, "none", test_flag_quick, validate_accelerometer},
	{"accel.interactive", "accel", "test accel interactive",
	 "Report BMI088 accelerometer XYZ changes until Ctrl+C.", validation_depth::functional,
	 "move the board", test_flag_none, nullptr},
	{"gyro", "gyro", "test gyro", "Read one BMI088 gyroscope XYZ sample.",
	 validation_depth::functional, "none", test_flag_quick, validate_gyroscope},
	{"gyro.interactive", "gyro", "test gyro interactive",
	 "Report BMI088 gyroscope XYZ changes until Ctrl+C.", validation_depth::functional,
	 "rotate the board", test_flag_none, nullptr},
	{"magn", "magn", "test magn", "Read one IST8310 magnetometer XYZ sample.",
	 validation_depth::functional, "none", test_flag_quick, validate_magnetometer},
	{"magn.interactive", "magn", "test magn interactive",
	 "Report IST8310 magnetometer XYZ changes until Ctrl+C.", validation_depth::functional,
	 "rotate the board", test_flag_none, nullptr},
	{"uart1", "uart", "test uart 1", "Run the USART1 115200 8N1 physical loopback.",
	 validation_depth::physical_fixture, "jumper PA9 TX to PB7 RX", test_flag_none, nullptr},
	{"uart6", "uart", "test uart 6", "Run the USART6 115200 8N1 physical loopback.",
	 validation_depth::physical_fixture, "jumper PG14 TX to PG9 RX", test_flag_none, nullptr},
	{"spi2", "spi2", "test spi2", "Run SPI2 MOSI-to-MISO loopback at two rates.",
	 validation_depth::physical_fixture, "jumper PB15 MOSI to PB14 MISO", test_flag_none,
	 validate_spi2_loopback},
	{"can.internal", "can", "test can internal", "Run CAN1 controller internal loopback.",
	 validation_depth::internal_loopback, "none", test_flag_quick, validate_can_internal},
	{"can.physical", "can", "test can physical", "Exchange frames over CAN1 and CAN2.",
	 validation_depth::physical_fixture, "wire CAN1 H/L/GND to CAN2 H/L/GND", test_flag_none,
	 validate_can_physical},
	{"i2c", "i2c", "test i2c", "Check I2C1 and I2C2 controller initialization.",
	 validation_depth::readiness, "none", test_flag_none, validate_i2c_readiness},
	{"rgb", "rgb", "test rgb", "Show a bounded green, red, blue, and white sequence.",
	 validation_depth::operator_observation, "visually observe onboard RGB LED",
	 test_flag_guided, validate_rgb_colors},
	{"rgb.interactive", "rgb", "test rgb interactive",
	 "Adjust RGB duty percentages until Ctrl+C.", validation_depth::operator_observation,
	 "visually observe onboard RGB LED", test_flag_none, nullptr},
	{"button", "button", "test button [timeout_ms]", "Wait for one bounded user-button press.",
	 validation_depth::operator_observation, "press the onboard user button", test_flag_guided,
	 nullptr},
	{"button.interactive", "button", "test button interactive",
	 "Report debounced button state changes until Ctrl+C.",
	 validation_depth::operator_observation, "press and release the onboard user button",
	 test_flag_none, nullptr},
	{"buzzer", "buzzer", "test buzzer", "Play one bounded onboard buzzer tone.",
	 validation_depth::operator_observation, "listen for the onboard buzzer", test_flag_guided,
	 validate_buzzer_instrumentation},
	{"buzzer.interactive", "buzzer", "test buzzer interactive",
	 "Adjust buzzer frequency and duty until Ctrl+C.", validation_depth::operator_observation,
	 "listen to or measure the onboard buzzer", test_flag_none, nullptr},
	{"servo.interactive", "servo", "test servo unloaded <1-7>",
	 "Adjust one unloaded servo PWM signal until Ctrl+C.", validation_depth::instrumentation,
	 "no servo/load; use an oscilloscope or logic analyzer", test_flag_none, nullptr},
};

} // namespace

const char *validation_depth_name(validation_depth depth)
{
	switch (depth) {
	case validation_depth::readiness:
		return "readiness";
	case validation_depth::functional:
		return "functional";
	case validation_depth::internal_loopback:
		return "internal-loopback";
	case validation_depth::physical_fixture:
		return "physical-fixture";
	case validation_depth::operator_observation:
		return "operator-observation";
	case validation_depth::instrumentation:
		return "instrumentation";
	}

	return "unknown";
}

const test_descriptor *test_catalog(size_t &count)
{
	count = ARRAY_SIZE(tests);
	return tests;
}

const test_descriptor *find_test(const char *id)
{
	for (const test_descriptor &test : tests) {
		if (strcmp(test.id, id) == 0) {
			return &test;
		}
	}

	return nullptr;
}

bool test_has_flag(const test_descriptor &test, test_flags flag)
{
	return (test.flags & static_cast<uint32_t>(flag)) != 0U;
}

int parse_bounded_u32(const char *text, uint32_t minimum, uint32_t maximum, uint32_t &value)
{
	int error = 0;
	const unsigned long parsed = shell_strtoul(text, 10, &error);
	if (error != 0) {
		return -EINVAL;
	}
	if (parsed < minimum || parsed > maximum) {
		return -ERANGE;
	}

	value = static_cast<uint32_t>(parsed);
	return 0;
}

reporter::reporter(const struct shell *output) : output_(output)
{
}

void reporter::begin_test(const test_descriptor &test)
{
	current_test_ = &test;
	current_test_result_ = result_kind::success;
	current_test_has_result_ = false;
	shell_print(output_, "");
	shell_print(output_, "TEST %s", test.command);
	shell_print(output_, "  depth=%s setup=%s", validation_depth_name(test.depth), test.setup);
}

void reporter::end_test()
{
	if (current_test_ == nullptr) {
		return;
	}

	const char *outcome;
	if (!current_test_has_result_) {
		test_error_count_++;
		outcome = "ERROR";
		if (first_error_ == 0) {
			first_error_ = -EIO;
		}
		shell_error(output_, "No terminal result was recorded for %s", current_test_->id);
	} else {
		outcome = result_name(current_test_result_);
		switch (current_test_result_) {
		case result_kind::success:
			test_success_count_++;
			break;
		case result_kind::error:
			test_error_count_++;
			break;
		case result_kind::not_tested:
			test_not_tested_count_++;
			break;
		}
	}

	shell_print(output_, "RESULT: %s - %s [%s]", outcome, current_test_->id,
		    validation_depth_name(current_test_->depth));
	current_test_ = nullptr;
	current_test_has_result_ = false;
}

bool reporter::current_test_has_error() const
{
	return current_test_has_result_ && current_test_result_ == result_kind::error;
}

void reporter::record(result_kind kind, const char *name, const char *detail, int error)
{
	if (current_test_ != nullptr) {
		if (!current_test_has_result_ || kind == result_kind::error ||
		    (kind == result_kind::not_tested &&
		     current_test_result_ == result_kind::success)) {
			current_test_result_ = kind;
		}
		current_test_has_result_ = true;
	}

	if (kind == result_kind::error && first_error_ == 0) {
		first_error_ = error < 0 ? error : -EIO;
	} else if (kind == result_kind::not_tested && first_incomplete_error_ == 0 && error < 0) {
		first_incomplete_error_ = error;
	}

	const char *const depth = current_test_ != nullptr
					  ? validation_depth_name(current_test_->depth)
					  : "unspecified";
	if (detail != nullptr && error != 0) {
		shell_print(output_, "[%s][%s] %s: %s (error %d)", result_name(kind), depth, name,
			    detail, error);
	} else if (detail != nullptr) {
		shell_print(output_, "[%s][%s] %s: %s", result_name(kind), depth, name, detail);
	} else if (error != 0) {
		shell_print(output_, "[%s][%s] %s (error %d)", result_name(kind), depth, name,
			    error);
	} else {
		shell_print(output_, "[%s][%s] %s", result_name(kind), depth, name);
	}
}

void reporter::info(const char *name, const char *detail) const
{
	shell_print(output_, "[INFO] %s: %s", name, detail);
}

int reporter::finish() const
{
	const uint32_t selected = test_success_count_ + test_error_count_ + test_not_tested_count_;
	const char *outcome = "success";
	if (test_error_count_ != 0U) {
		outcome = "error";
	} else if (test_not_tested_count_ != 0U) {
		outcome = "not_tested";
	}

	shell_print(output_, "");
	shell_print(output_,
		    "Run result: %s - selected=%" PRIu32 " success=%" PRIu32 " error=%" PRIu32
		    " not_tested=%" PRIu32,
		    outcome, selected, test_success_count_, test_error_count_,
		    test_not_tested_count_);

	if (first_error_ != 0) {
		return first_error_;
	}
	if (test_not_tested_count_ != 0U) {
		return first_incomplete_error_ != 0 ? first_incomplete_error_ : -EAGAIN;
	}
	return 0;
}

const struct shell *reporter::output() const
{
	return output_;
}

int64_t sensor_value_to_micro(int32_t whole, int32_t fractional)
{
	return static_cast<int64_t>(whole) * 1000000LL + fractional;
}

void print_fixed_point(const reporter &reporter, const char *name, int64_t micro_value,
		       const char *unit)
{
	const bool negative = micro_value < 0;
	const uint64_t magnitude =
		negative ? static_cast<uint64_t>(-micro_value) : static_cast<uint64_t>(micro_value);

	shell_print(reporter.output(), "[INFO] %s: %s%" PRIu64 ".%06" PRIu64 " %s", name,
		    negative ? "-" : "", magnitude / 1000000ULL, magnitude % 1000000ULL, unit);
}

} // namespace type_c_validation
