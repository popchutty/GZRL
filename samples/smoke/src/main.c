/*
 * SPDX-FileCopyrightText: 2026 GMaster contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/printk.h>

#ifndef CONFIG_GZRL
#error "GZRL module Kconfig was not imported"
#endif

int main(void)
{
	printk("GZRL module smoke sample\n");

	return 0;
}
