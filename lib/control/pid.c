/*
 * SPDX-FileCopyrightText: 2026 GMaster contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <gzrl/control.h>

#include <errno.h>
#include <float.h>
#include <math.h>
#include <string.h>

static bool finite_float(double value)
{
	return isfinite(value) && value >= -(double)FLT_MAX && value <= (double)FLT_MAX;
}

static double bound(double value, float minimum, float maximum)
{
	if (value > (double)maximum) {
		return (double)maximum;
	}
	if (value < (double)minimum) {
		return (double)minimum;
	}
	return value;
}

static bool valid_limits(float minimum, float maximum)
{
	return isfinite(minimum) && isfinite(maximum) && minimum <= 0.0f && maximum >= 0.0f;
}

static bool valid_config(const struct gzrl_pid_config *config)
{
	return config != NULL &&
	       (config->mode == GZRL_PID_POSITION || config->mode == GZRL_PID_INCREMENTAL) &&
	       (config->derivative == GZRL_PID_D_MEASUREMENT ||
		config->derivative == GZRL_PID_D_ERROR) &&
	       isfinite(config->kp) && config->kp >= 0.0f && isfinite(config->ki) &&
	       config->ki >= 0.0f && isfinite(config->kd) && config->kd >= 0.0f &&
	       valid_limits(config->output_min, config->output_max) &&
	       valid_limits(config->integral_min, config->integral_max) &&
	       isfinite(config->dt_min) && isfinite(config->dt_max) && config->dt_min > 0.0f &&
	       config->dt_max >= config->dt_min;
}

int gzrl_pid_init(struct gzrl_pid *pid, const struct gzrl_pid_config *config)
{
	if (pid == NULL || !valid_config(config)) {
		return -EINVAL;
	}
	struct gzrl_pid next = {.config = *config, .configured = true};

	*pid = next;
	return 0;
}

int gzrl_pid_reset(struct gzrl_pid *pid)
{
	if (pid == NULL || !pid->configured) {
		return -EINVAL;
	}
	memset(&pid->state, 0, sizeof(pid->state));
	return 0;
}

int gzrl_pid_update(struct gzrl_pid *pid, float setpoint, float measurement, float dt,
		    float *output)
{
	if (pid == NULL || !pid->configured || output == NULL) {
		return -EINVAL;
	}
	const struct gzrl_pid_config *config = &pid->config;
	struct gzrl_pid_state next = pid->state;

	if (!isfinite(setpoint) || !isfinite(measurement) || !isfinite(dt) || dt < config->dt_min ||
	    dt > config->dt_max) {
		return -ERANGE;
	}
	double error = (double)setpoint - (double)measurement;
	double p = (double)config->kp * error;
	double d = 0.0;

	if (next.initialized) {
		double difference =
			config->derivative == GZRL_PID_D_ERROR
				? error - (double)next.previous_error
				: (double)next.previous_measurement - (double)measurement;

		d = (double)config->kd * difference / (double)dt;
	}
	if (!finite_float(error) || !finite_float(p) || !finite_float(d)) {
		return -ERANGE;
	}
	double integral = bound((double)next.i + (double)config->ki * error * (double)dt,
				config->integral_min, config->integral_max);
	double change_i = integral - (double)next.i;
	double without_i;

	if (config->mode == GZRL_PID_INCREMENTAL) {
		without_i = (double)next.output + (p - (double)next.p) + (d - (double)next.d);
	} else {
		without_i = p + (double)next.i + d;
	}
	double result = without_i + change_i;

	if ((result > (double)config->output_max && change_i > 0.0) ||
	    (result < (double)config->output_min && change_i < 0.0)) {
		integral = (double)next.i;
	}
	result = bound(result, config->output_min, config->output_max);
	if (!finite_float(integral) || !finite_float(result)) {
		return -ERANGE;
	}
	next.output = (float)result;
	next.p = (float)p;
	next.i = (float)integral;
	next.d = (float)d;
	next.previous_error = (float)error;
	next.previous_measurement = measurement;
	next.initialized = true;
	pid->state = next;
	*output = next.output;
	return 0;
}

int gzrl_pid_get_config(const struct gzrl_pid *pid, struct gzrl_pid_config *config)
{
	if (pid == NULL || !pid->configured || config == NULL) {
		return -EINVAL;
	}
	*config = pid->config;
	return 0;
}

int gzrl_pid_get_state(const struct gzrl_pid *pid, struct gzrl_pid_state *state)
{
	if (pid == NULL || !pid->configured || state == NULL) {
		return -EINVAL;
	}
	*state = pid->state;
	return 0;
}

int gzrl_cascade_init(struct gzrl_cascade *cascade, const struct gzrl_pid_config *outer,
		      const struct gzrl_pid_config *inner)
{
	struct gzrl_cascade next = {0};
	int ret;

	if (cascade == NULL) {
		return -EINVAL;
	}
	ret = gzrl_pid_init(&next.outer, outer);
	if (ret == 0) {
		ret = gzrl_pid_init(&next.inner, inner);
	}
	if (ret == 0) {
		*cascade = next;
	}
	return ret;
}

int gzrl_cascade_reset(struct gzrl_cascade *cascade)
{
	if (cascade == NULL || !cascade->outer.configured || !cascade->inner.configured) {
		return -EINVAL;
	}
	(void)gzrl_pid_reset(&cascade->outer);
	(void)gzrl_pid_reset(&cascade->inner);
	cascade->outer_output = 0.0f;
	cascade->outer_ready = false;
	return 0;
}

int gzrl_cascade_update(struct gzrl_cascade *cascade, float setpoint, float outer_measurement,
			float inner_measurement, bool outer_due, float outer_dt, float inner_dt,
			float *output)
{
	if (cascade == NULL || !cascade->outer.configured || !cascade->inner.configured ||
	    output == NULL) {
		return -EINVAL;
	}
	struct gzrl_cascade next = *cascade;
	float result;
	int ret;

	if (outer_due) {
		ret = gzrl_pid_update(&next.outer, setpoint, outer_measurement, outer_dt,
				      &next.outer_output);
		if (ret != 0) {
			return ret;
		}
		next.outer_ready = true;
	}
	if (!next.outer_ready) {
		return -EAGAIN;
	}
	ret = gzrl_pid_update(&next.inner, next.outer_output, inner_measurement, inner_dt, &result);
	if (ret != 0) {
		return ret;
	}
	*cascade = next;
	*output = result;
	return 0;
}
