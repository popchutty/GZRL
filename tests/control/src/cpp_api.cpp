/*
 * SPDX-FileCopyrightText: 2026 GMaster contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <gzrl/control.h>

extern "C" int control_cpp_probe(void)
{
	float result = 1.0f;
	int ret = gzrl_angle_error(0.0f, 0.0f, &result);

	return ret == 0 && result == 0.0f ? 0 : -1;
}
