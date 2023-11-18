/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   500

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define LED3_NODE DT_ALIAS(led3)
#define LED3	DT_GPIO_LABEL(LED3_NODE, gpios)
#define PIN_3	DT_GPIO_PIN(LED3_NODE, gpios)
#define FLAGS_3	DT_GPIO_FLAGS(LED3_NODE, gpios)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN	DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0	""
#define PIN	0
#define FLAGS	0
#endif


void main(void)
{
	const struct device *dev;
	const struct device *d_led_3;

	bool led_is_on = true;
	int ret;
	
	dev = device_get_binding(LED0);
	if (dev == NULL) {
		return;
	}

	d_led_3 = device_get_binding(LED3);


	ret = gpio_pin_configure(dev, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return;
	}

	ret = gpio_pin_configure(d_led_3, PIN_3, GPIO_OUTPUT_ACTIVE | FLAGS_3);
	if (ret < 0) {
		return;
	}

	while (1) {
		gpio_pin_set(dev, PIN, (int)led_is_on);
		gpio_pin_set(d_led_3, PIN_3, (int)led_is_on);
		led_is_on = !led_is_on;
		k_msleep(SLEEP_TIME_MS);
	}
}
