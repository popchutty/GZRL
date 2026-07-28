/*
 * SPDX-FileCopyrightText: 2026 GMaster contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef GZRL_SAMPLES_ROBOMASTER_TYPE_C_VALIDATION_HPP_
#define GZRL_SAMPLES_ROBOMASTER_TYPE_C_VALIDATION_HPP_

#include <stddef.h>
#include <stdint.h>

#include <zephyr/shell/shell.h>

namespace type_c_validation
{

enum class result_kind {
	success,
	error,
	not_tested,
};

enum class validation_depth {
	readiness,
	functional,
	internal_loopback,
	physical_fixture,
	operator_observation,
	instrumentation,
};

enum test_flags : uint32_t {
	test_flag_none = 0U,
	test_flag_quick = 1U << 0,
	test_flag_guided = 1U << 1,
};

enum class uart_port {
	usart1,
	usart6,
};

class reporter;
using validation_function = void (*)(reporter &reporter);

struct test_descriptor {
	const char *id;
	const char *name;
	const char *command;
	const char *description;
	validation_depth depth;
	const char *setup;
	uint32_t flags;
	validation_function function;
};

class reporter
{
      public:
	explicit reporter(const struct shell *output);

	void begin_test(const test_descriptor &test);
	void end_test();
	bool current_test_has_error() const;
	void record(result_kind kind, const char *name, const char *detail = nullptr,
		    int error = 0);
	void info(const char *name, const char *detail) const;
	int finish() const;
	const struct shell *output() const;

      private:
	const struct shell *output_;
	const test_descriptor *current_test_ = nullptr;
	result_kind current_test_result_ = result_kind::success;
	bool current_test_has_result_ = false;
	uint32_t test_success_count_ = 0;
	uint32_t test_error_count_ = 0;
	uint32_t test_not_tested_count_ = 0;
	int first_error_ = 0;
	int first_incomplete_error_ = 0;
};

const test_descriptor *test_catalog(size_t &count);
const test_descriptor *find_test(const char *id);
const char *validation_depth_name(validation_depth depth);
bool test_has_flag(const test_descriptor &test, test_flags flag);
int parse_bounded_u32(const char *text, uint32_t minimum, uint32_t maximum, uint32_t &value);

void validate_system_info(reporter &reporter);
void validate_clock(reporter &reporter);
void validate_devices(reporter &reporter);
void validate_entropy(reporter &reporter);
void validate_battery(reporter &reporter);
void validate_accelerometer(reporter &reporter);
void validate_gyroscope(reporter &reporter);
void validate_magnetometer(reporter &reporter);
void validate_can_internal(reporter &reporter);
void validate_rgb_colors(reporter &reporter);
void validate_button(reporter &reporter, uint32_t timeout_ms);
void validate_i2c_readiness(reporter &reporter);

void validate_uart_loopback(reporter &reporter, uart_port port);
void validate_spi2_loopback(reporter &reporter);
void validate_can_physical(reporter &reporter);
void validate_buzzer_instrumentation(reporter &reporter);

int run_accelerometer_interactive(const struct shell *shell);
int run_gyroscope_interactive(const struct shell *shell);
int run_magnetometer_interactive(const struct shell *shell);
int run_button_interactive(const struct shell *shell);
int run_rgb_interactive(const struct shell *shell);
int run_buzzer_interactive(const struct shell *shell);
extern const char buzzer_score_help[];
int run_buzzer_score(const struct shell *shell);
int run_servo_interactive(const struct shell *shell, uint32_t channel);

int64_t sensor_value_to_micro(int32_t whole, int32_t fractional);
void print_fixed_point(const reporter &reporter, const char *name, int64_t micro_value,
		       const char *unit);

} // namespace type_c_validation

#endif /* GZRL_SAMPLES_ROBOMASTER_TYPE_C_VALIDATION_HPP_ */
