/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Minimal bring-up main() for the MCX-N9XX-EVK. This deliberately mirrors
 * the banner style of common/service/main.c without calling into any of
 * OpenBIC's shared sensor/IPMI/FRU pipeline - see CMakeLists.txt and
 * README.md in this directory for why, and for the plan to close that gap.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include "plat_gpio.h"
#include "plat_version.h"

#define HEARTBEAT_PERIOD_MS 500

static const struct gpio_dt_spec heartbeat_led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

int main(void)
{
	printk("Hello, welcome to %s %s %x%x.%x.%x\n", PLATFORM_NAME, PROJECT_NAME,
	       BIC_FW_YEAR_MSB, BIC_FW_YEAR_LSB, BIC_FW_WEEK, BIC_FW_VER);
	printk("Minimal bring-up: no sensor/IPMI/FRU services in this build.\n");

	plat_gpio_init();

	if (!gpio_is_ready_dt(&heartbeat_led)) {
		printk("Heartbeat LED device not ready\n");
		return 0;
	}

	gpio_pin_configure_dt(&heartbeat_led, GPIO_OUTPUT_ACTIVE);

	while (1) {
		gpio_pin_toggle_dt(&heartbeat_led);
		k_msleep(HEARTBEAT_PERIOD_MS);
	}

	return 0;
}
