/*
 * SPDX-FileCopyrightText: 2026 GMaster contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#include <gzrl/motor/bus.h>

extern "C" uint32_t motor_cpp_capabilities(const struct device *dev)
{
	return gzrl_motor_device_capabilities(dev);
}
