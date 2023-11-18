/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#ifdef CONFIG_FUNCTEST
#include "funcTest.h"
#endif

void main(void)
{
	#ifdef CONFIG_FUNCTEST
		int a = 3;
		int b = 4;
		printk("Hello World!\n\r");
		printk("Total Sum of %d and %d is %d\n\r", a, b, total_sum(a, b));
	#else
		prinkt("FUNCTEST IS NOT ENABLED\n\r");
		return;
	#endif

	while(1) {
		k_msleep(1000);
	}
}