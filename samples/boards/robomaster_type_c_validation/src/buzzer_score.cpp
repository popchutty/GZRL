/*
 * SPDX-FileCopyrightText: 2026 GMaster contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "validation.hpp"

#include <errno.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

namespace type_c_validation
{

const char buzzer_score_help[] =
	"Enter the monophonic buzzer score player.\n"
	"Paste one score line and press Enter; playback starts immediately.\n"
	"\n"
	"Grammar:\n"
	"  score := T<BPM> <event> [<event> ...]\n"
	"  event := <MIDI>/<duration>[.] | R/<duration>[.]\n"
	"\n"
	"BPM: 40-240 quarter-note beats per minute\n"
	"MIDI: 21-108; 60=C4, 61=C#4/Db4, 69=A4\n"
	"Duration: 1=whole, 2=half, 4=quarter, 8=eighth, 16=sixteenth\n"
	"Add '.' for a dotted duration, for example 60/4.\n"
	"Separate tokens with spaces or tabs; do not use spaces inside an event.\n"
	"Limits: 100 events, 511 characters.\n"
	"Example: T120 67/8 67/8 69/4 67/4 72/4 71/2 R/8\n"
	"\n"
	"Mode commands: help | status | stop | quit\n"
	"Ctrl+C stops playback and exits. Paste another score to replace playback.";

namespace
{

const struct pwm_dt_spec buzzer = PWM_DT_SPEC_GET(DT_ALIAS(pwm_buzzer));
constexpr uint64_t expected_cycles_per_second = 1000000ULL;
constexpr uint32_t silence_period_cycles = 250U;
constexpr uint32_t duty_divisor = 4U;
constexpr size_t score_line_size = 512U;
constexpr size_t maximum_event_count = 100U;
constexpr uint8_t rest_midi = 0U;

struct buzzer_score_event {
	uint8_t midi;
	uint8_t duration_units;
};

struct buzzer_score {
	uint16_t bpm;
	uint8_t event_count;
	uint64_t total_duration_us;
	buzzer_score_event events[maximum_event_count];
};

enum class playback_phase {
	start_event,
	tone_gap,
	complete_event,
};

struct buzzer_score_session {
	const struct pwm_dt_spec *buzzer;
	struct k_work_delayable work;
	buzzer_score score;
	size_t event_index;
	uint64_t event_duration_us;
	uint64_t tone_duration_us;
	playback_phase phase;
	atomic_t active;
	atomic_t current_event;
	atomic_t error;
};

constexpr uint32_t octave_zero_frequency_millihz[] = {
	16352U, 17324U, 18354U, 19445U, 20602U, 21827U,
	23125U, 24500U, 25957U, 27500U, 29135U, 30868U,
};

BUILD_ASSERT(ARRAY_SIZE(octave_zero_frequency_millihz) == 12U);
BUILD_ASSERT(maximum_event_count <= UINT8_MAX);

bool is_decimal_digits(const char *text)
{
	if (text == nullptr || text[0] == '\0') {
		return false;
	}

	for (const char *cursor = text; *cursor != '\0'; ++cursor) {
		if (*cursor < '0' || *cursor > '9') {
			return false;
		}
	}
	return true;
}

uint64_t duration_units_to_us(uint8_t duration_units, uint16_t bpm)
{
	const uint64_t numerator = 7500000ULL * duration_units;
	return (numerator + (bpm / 2U)) / bpm;
}

int parse_duration(const char *text, bool dotted, uint8_t &duration_units)
{
	if (!is_decimal_digits(text)) {
		return -EINVAL;
	}

	uint32_t denominator = 0U;
	if (parse_bounded_u32(text, 1U, 16U, denominator) != 0) {
		return -EINVAL;
	}

	uint8_t base_units;
	switch (denominator) {
	case 1U:
		base_units = 32U;
		break;
	case 2U:
		base_units = 16U;
		break;
	case 4U:
		base_units = 8U;
		break;
	case 8U:
		base_units = 4U;
		break;
	case 16U:
		base_units = 2U;
		break;
	default:
		return -EINVAL;
	}

	duration_units = dotted ? static_cast<uint8_t>((base_units * 3U) / 2U) : base_units;
	return 0;
}

int parse_score_event(const struct shell *shell, char *token, size_t event_index,
		      buzzer_score_event &event)
{
	char *const slash = strchr(token, '/');
	if (slash == nullptr || strchr(slash + 1, '/') != nullptr) {
		shell_error(shell, "Event %zu must contain exactly one '/': %s", event_index,
			    token);
		return -EINVAL;
	}

	*slash = '\0';
	char *const duration_text = slash + 1;
	const size_t duration_length = strlen(duration_text);
	bool dotted = false;
	if (duration_length > 0U && duration_text[duration_length - 1U] == '.') {
		duration_text[duration_length - 1U] = '\0';
		dotted = true;
	}

	if (strcmp(token, "R") == 0) {
		event.midi = rest_midi;
	} else {
		if (!is_decimal_digits(token)) {
			shell_error(shell, "Event %zu has an invalid MIDI note: %s", event_index,
				    token);
			return -EINVAL;
		}

		uint32_t midi = 0U;
		if (parse_bounded_u32(token, 21U, 108U, midi) != 0) {
			shell_error(shell, "Event %zu MIDI note must be between 21 and 108",
				    event_index);
			return -EINVAL;
		}
		event.midi = static_cast<uint8_t>(midi);
	}

	if (parse_duration(duration_text, dotted, event.duration_units) != 0) {
		shell_error(shell, "Event %zu duration must be 1, 2, 4, 8, or 16 with optional '.'",
			    event_index);
		return -EINVAL;
	}
	return 0;
}

int parse_buzzer_score(const struct shell *shell, char *line, buzzer_score &score)
{
	char *save = nullptr;
	char *tempo = strtok_r(line, " \t", &save);
	if (tempo == nullptr || tempo[0] != 'T' || !is_decimal_digits(tempo + 1)) {
		shell_error(shell, "Score must begin with T<BPM>, for example T120");
		return -EINVAL;
	}

	uint32_t bpm = 0U;
	if (parse_bounded_u32(tempo + 1, 40U, 240U, bpm) != 0) {
		shell_error(shell, "BPM must be between 40 and 240");
		return -EINVAL;
	}
	score.bpm = static_cast<uint16_t>(bpm);

	uint64_t total_duration_us = 0U;
	size_t event_count = 0U;
	for (char *token = strtok_r(nullptr, " \t", &save); token != nullptr;
	     token = strtok_r(nullptr, " \t", &save)) {
		if (event_count >= maximum_event_count) {
			shell_error(shell, "Score exceeds the %zu-event limit",
				    maximum_event_count);
			return -E2BIG;
		}

		buzzer_score_event &event = score.events[event_count];
		const int ret = parse_score_event(shell, token, event_count + 1U, event);
		if (ret != 0) {
			return ret;
		}

		total_duration_us += duration_units_to_us(event.duration_units, score.bpm);
		event_count++;
	}

	if (event_count == 0U) {
		shell_error(shell, "Score must contain at least one event");
		return -EINVAL;
	}

	score.event_count = static_cast<uint8_t>(event_count);
	score.total_duration_us = total_duration_us;
	return 0;
}

uint32_t midi_period_cycles(uint8_t midi)
{
	const uint32_t base_frequency_millihz = octave_zero_frequency_millihz[midi % 12U];
	const uint8_t octave_shift = static_cast<uint8_t>((midi / 12U) - 1U);
	const uint32_t frequency_millihz = base_frequency_millihz << octave_shift;
	const uint64_t period_numerator = expected_cycles_per_second * 1000ULL;

	return static_cast<uint32_t>((period_numerator + (frequency_millihz / 2U)) /
				     frequency_millihz);
}

int silence_buzzer(const struct pwm_dt_spec &buzzer_spec)
{
	return pwm_set_cycles(buzzer_spec.dev, buzzer_spec.channel, silence_period_cycles, 0U,
			      buzzer_spec.flags);
}

int play_midi_note(const struct pwm_dt_spec &buzzer_spec, uint8_t midi)
{
	const uint32_t period_cycles = midi_period_cycles(midi);
	return pwm_set_cycles(buzzer_spec.dev, buzzer_spec.channel, period_cycles,
			      period_cycles / duty_divisor, buzzer_spec.flags);
}

void finish_playback(buzzer_score_session &session, int error)
{
	const int cleanup_ret = silence_buzzer(*session.buzzer);
	if (error == 0 && cleanup_ret != 0) {
		error = cleanup_ret;
	}

	atomic_set(&session.error, error);
	atomic_set(&session.active, 0);
}

int schedule_transition(buzzer_score_session &session, uint64_t delay_us)
{
	if (atomic_get(&session.active) == 0) {
		return 0;
	}

	const int ret = k_work_reschedule(&session.work, K_USEC(delay_us));
	if (ret < 0) {
		finish_playback(session, ret);
		return ret;
	}
	return 0;
}

void buzzer_score_work_handler(struct k_work *work)
{
	struct k_work_delayable *const delayable = k_work_delayable_from_work(work);
	buzzer_score_session *const session = CONTAINER_OF(delayable, buzzer_score_session, work);

	if (atomic_get(&session->active) == 0) {
		return;
	}

	if (session->phase == playback_phase::complete_event) {
		session->event_index++;
		if (session->event_index >= session->score.event_count) {
			finish_playback(*session, 0);
			return;
		}
		session->phase = playback_phase::start_event;
	}

	if (session->phase == playback_phase::start_event) {
		const buzzer_score_event &event = session->score.events[session->event_index];
		atomic_set(&session->current_event,
			   static_cast<atomic_val_t>(session->event_index) + 1);
		session->event_duration_us =
			duration_units_to_us(event.duration_units, session->score.bpm);

		int ret;
		if (event.midi == rest_midi) {
			ret = silence_buzzer(*session->buzzer);
			session->phase = playback_phase::complete_event;
			if (ret == 0) {
				ret = schedule_transition(*session, session->event_duration_us);
			}
		} else {
			ret = play_midi_note(*session->buzzer, event.midi);
			session->tone_duration_us = (session->event_duration_us * 7U) / 8U;
			session->phase = playback_phase::tone_gap;
			if (ret == 0) {
				ret = schedule_transition(*session, session->tone_duration_us);
			}
		}

		if (ret != 0 && atomic_get(&session->active) != 0) {
			finish_playback(*session, ret);
		}
		return;
	}

	const int ret = silence_buzzer(*session->buzzer);
	if (ret != 0) {
		finish_playback(*session, ret);
		return;
	}

	session->phase = playback_phase::complete_event;
	const uint64_t gap_duration_us = session->event_duration_us - session->tone_duration_us;
	(void)schedule_transition(*session, gap_duration_us);
}

int stop_playback(buzzer_score_session &session)
{
	atomic_set(&session.active, 0);
	struct k_work_sync sync;
	(void)k_work_cancel_delayable_sync(&session.work, &sync);

	int ret = static_cast<int>(atomic_get(&session.error));
	const int cleanup_ret = silence_buzzer(*session.buzzer);
	if (ret == 0 && cleanup_ret != 0) {
		ret = cleanup_ret;
	}
	atomic_set(&session.error, ret);
	return ret;
}

int start_playback(buzzer_score_session &session, const buzzer_score &score)
{
	int ret = stop_playback(session);
	if (ret != 0) {
		return ret;
	}

	session.score = score;
	session.event_index = 0U;
	session.event_duration_us = 0U;
	session.tone_duration_us = 0U;
	session.phase = playback_phase::start_event;
	atomic_set(&session.current_event, 0);
	atomic_set(&session.error, 0);
	atomic_set(&session.active, 1);

	ret = k_work_schedule(&session.work, K_NO_WAIT);
	if (ret < 0) {
		finish_playback(session, ret);
		return ret;
	}
	return 0;
}

bool line_is_command(const char *line, const char *command)
{
	while (*line == ' ' || *line == '\t') {
		line++;
	}

	const size_t length = strlen(command);
	if (strncmp(line, command, length) != 0) {
		return false;
	}
	line += length;

	while (*line == ' ' || *line == '\t') {
		line++;
	}
	return *line == '\0';
}

void print_score_status(const struct shell *shell, const buzzer_score_session &session)
{
	const uint64_t duration_ms = (session.score.total_duration_us + 500U) / 1000U;
	if (atomic_get(&session.active) != 0) {
		shell_print(shell, "Score: playing event %ld/%u, total=%" PRIu64 " ms",
			    static_cast<long>(atomic_get(&session.current_event)),
			    session.score.event_count, duration_ms);
	} else if (session.score.event_count != 0U) {
		shell_print(shell, "Score: idle, last score events=%u total=%" PRIu64 " ms",
			    session.score.event_count, duration_ms);
	} else {
		shell_print(shell, "Score: idle, no score loaded");
	}
}

} // namespace

int run_buzzer_score(const struct shell *shell)
{
	if (!pwm_is_ready_dt(&buzzer)) {
		shell_error(shell, "Buzzer PWM controller is not ready");
		return -ENODEV;
	}

	uint64_t cycles_per_second = 0U;
	int ret = pwm_get_cycles_per_sec(buzzer.dev, buzzer.channel, &cycles_per_second);
	if (ret != 0) {
		shell_error(shell, "Failed to read buzzer PWM clock (%d)", ret);
		return ret;
	}
	if (cycles_per_second != expected_cycles_per_second) {
		(void)silence_buzzer(buzzer);
		shell_error(shell,
			    "Buzzer score requires a 1 MHz PWM clock; got %" PRIu64
			    " Hz. Check the board Devicetree configuration.",
			    cycles_per_second);
		return -ENOTSUP;
	}

	buzzer_score_session session = {};
	session.buzzer = &buzzer;
	k_work_init_delayable(&session.work, buzzer_score_work_handler);
	atomic_set(&session.active, 0);
	atomic_set(&session.current_event, 0);
	atomic_set(&session.error, 0);

	ret = silence_buzzer(buzzer);
	if (ret != 0) {
		shell_error(shell, "Failed to establish inactive buzzer output (%d)", ret);
		return ret;
	}

	shell_print(shell, "%s", buzzer_score_help);
	shell_print(shell, "Tab completion and history are unavailable inside this session.");

	uint8_t line_buffer[score_line_size] = {};
	char *const line = reinterpret_cast<char *>(line_buffer);
	int result = 0;

	for (;;) {
		ret = shell_readline(shell, line_buffer, sizeof(line_buffer), K_FOREVER);
		if (ret == -ECANCELED) {
			break;
		}
		if (ret < 0) {
			shell_error(shell, "Score input failed (%d)", ret);
			result = ret;
			break;
		}

		const int playback_error = static_cast<int>(atomic_get(&session.error));
		if (playback_error != 0) {
			shell_error(shell, "Buzzer playback failed (%d)", playback_error);
			result = playback_error;
			break;
		}

		if (line_is_command(line, "help")) {
			shell_print(shell, "%s", buzzer_score_help);
			continue;
		}
		if (line_is_command(line, "status")) {
			print_score_status(shell, session);
			continue;
		}
		if (line_is_command(line, "stop")) {
			ret = stop_playback(session);
			if (ret != 0) {
				shell_error(shell, "Failed to stop buzzer playback (%d)", ret);
				result = ret;
				break;
			}
			shell_print(shell, "Buzzer score playback stopped");
			continue;
		}
		if (line_is_command(line, "quit")) {
			break;
		}

		buzzer_score candidate = {};
		ret = parse_buzzer_score(shell, line, candidate);
		if (ret != 0) {
			continue;
		}

		ret = start_playback(session, candidate);
		if (ret != 0) {
			shell_error(shell, "Failed to start buzzer playback (%d)", ret);
			result = ret;
			break;
		}
		shell_print(shell, "Playing %u events for %" PRIu64 " ms", candidate.event_count,
			    (candidate.total_duration_us + 500U) / 1000U);
	}

	const int cleanup_ret = stop_playback(session);
	if (result == 0 && cleanup_ret != 0) {
		shell_error(shell, "Final buzzer cleanup failed (%d)", cleanup_ret);
		result = cleanup_ret;
	}
	return result;
}

} // namespace type_c_validation
