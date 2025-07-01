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
 */

#include <zephyr.h>
#include <stdio.h>
#include "plat_led.h"
#include "plat_gpio.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(plat_led);

K_WORK_DELAYABLE_DEFINE(heartbeat_led_work, heartbeat_led_handler);

void heartbeat_led_handler(struct k_work *work)
{
	/* Lit heartbeat LED */
	static bool led_status = true;
	led_status = !led_status;
	LOG_DBG("heartbeat LED %s ", led_status ? "OFF" : "ON");
	gpio_set(LED_MMC_HEARTBEAT_R, led_status);

	k_work_schedule(&heartbeat_led_work, K_MSEC(250));
}

void plat_led_init(void)
{
	k_work_schedule(&heartbeat_led_work, K_MSEC(250));
	LOG_INF("Heartbeat LED timer started");
}
