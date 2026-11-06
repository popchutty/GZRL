/*
 * SPDX-FileCopyrightText: 2026 GMaster contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <gzrl/control.h>

#include <errno.h>
#include <math.h>
#include <stdint.h>

int gzrl_lowpass_init(struct gzrl_lowpass *filter, float time_constant, float dt_min, float dt_max)
{
	if (filter == NULL || !isfinite(time_constant) || time_constant <= 0.0f ||
	    !isfinite(dt_min) || !isfinite(dt_max) || dt_min <= 0.0f || dt_max < dt_min) {
		return -EINVAL;
	}
	struct gzrl_lowpass next = {
		.time_constant = time_constant,
		.dt_min = dt_min,
		.dt_max = dt_max,
		.configured = true,
	};

	*filter = next;
	return 0;
}

int gzrl_lowpass_reset(struct gzrl_lowpass *filter)
{
	if (filter == NULL || !filter->configured) {
		return -EINVAL;
	}
	filter->value = 0.0f;
	filter->initialized = false;
	return 0;
}

int gzrl_lowpass_update(struct gzrl_lowpass *filter, float input, float dt, float *output)
{
	if (filter == NULL || !filter->configured || output == NULL) {
		return -EINVAL;
	}
	if (!isfinite(input) || !isfinite(dt) || dt < filter->dt_min || dt > filter->dt_max) {
		return -ERANGE;
	}
	double value = (double)input;

	if (filter->initialized) {
		double alpha = (double)dt / ((double)filter->time_constant + (double)dt);

		value = (double)filter->value + alpha * ((double)input - (double)filter->value);
	}
	filter->value = (float)value;
	filter->initialized = true;
	*output = filter->value;
	return 0;
}

int gzrl_moving_average_init(struct gzrl_moving_average *filter, float *storage, size_t capacity)
{
	if (filter == NULL || storage == NULL || capacity == 0 ||
	    capacity > SIZE_MAX / sizeof(float)) {
		return -EINVAL;
	}
	struct gzrl_moving_average next = {.storage = storage, .capacity = capacity};

	*filter = next;
	return 0;
}

int gzrl_moving_average_reset(struct gzrl_moving_average *filter)
{
	if (filter == NULL || filter->storage == NULL || filter->capacity == 0) {
		return -EINVAL;
	}
	filter->count = 0;
	filter->next = 0;
	filter->sum = 0.0;
	return 0;
}

int gzrl_moving_average_update(struct gzrl_moving_average *filter, float input, float *output)
{
	if (filter == NULL || filter->storage == NULL || filter->capacity == 0 || output == NULL) {
		return -EINVAL;
	}
	if (!isfinite(input)) {
		return -ERANGE;
	}
	double sum = filter->sum;
	size_t count = filter->count;

	if (count == filter->capacity) {
		sum -= (double)filter->storage[filter->next];
	} else {
		++count;
	}
	sum += (double)input;
	filter->storage[filter->next] = input;
	filter->next = (filter->next + 1) % filter->capacity;
	filter->count = count;
	filter->sum = sum;
	*output = (float)(sum / (double)count);
	return 0;
}

int gzrl_median_init(struct gzrl_median *filter, float *storage, float *scratch, size_t capacity)
{
	if (filter == NULL || storage == NULL || scratch == NULL || capacity == 0 ||
	    capacity > SIZE_MAX / sizeof(float)) {
		return -EINVAL;
	}
	uintptr_t a = (uintptr_t)storage;
	uintptr_t b = (uintptr_t)scratch;
	size_t span = capacity * sizeof(float);

	if ((a <= b ? b - a : a - b) < span) {
		return -EINVAL;
	}
	struct gzrl_median next = {.storage = storage, .scratch = scratch, .capacity = capacity};

	*filter = next;
	return 0;
}

int gzrl_median_reset(struct gzrl_median *filter)
{
	if (filter == NULL || filter->storage == NULL || filter->scratch == NULL ||
	    filter->capacity == 0) {
		return -EINVAL;
	}
	filter->count = 0;
	filter->next = 0;
	return 0;
}

int gzrl_median_update(struct gzrl_median *filter, float input, float *output)
{
	if (filter == NULL || filter->storage == NULL || filter->scratch == NULL ||
	    filter->capacity == 0 || output == NULL) {
		return -EINVAL;
	}
	if (!isfinite(input)) {
		return -ERANGE;
	}
	filter->storage[filter->next] = input;
	filter->next = (filter->next + 1) % filter->capacity;
	if (filter->count < filter->capacity) {
		++filter->count;
	}
	for (size_t i = 0; i < filter->count; ++i) {
		float value = filter->storage[i];
		size_t j = i;

		while (j > 0 && filter->scratch[j - 1] > value) {
			filter->scratch[j] = filter->scratch[j - 1];
			--j;
		}
		filter->scratch[j] = value;
	}
	size_t middle = filter->count / 2;

	*output = filter->count % 2 != 0 ? filter->scratch[middle]
					 : (float)(((double)filter->scratch[middle - 1] +
						    (double)filter->scratch[middle]) /
						   2.0);
	return 0;
}

int gzrl_angle_error(float target, float measurement, float *error)
{
	const double pi = 3.14159265358979323846;
	const double tau = 2.0 * pi;

	if (error == NULL) {
		return -EINVAL;
	}
	if (!isfinite(target) || !isfinite(measurement)) {
		return -ERANGE;
	}
	double value = fmod((double)target - (double)measurement + pi, tau);

	if (value < 0.0) {
		value += tau;
	}
	*error = (float)(value - pi);
	return 0;
}
