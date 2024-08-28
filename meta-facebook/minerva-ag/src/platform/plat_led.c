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

K_TIMER_DEFINE(heartbeat_led_timer, lit_heartbeat_led, NULL);
K_WORK_DEFINE(lit_heartbeat_led_work, lit_heartbeat_led_handler);

void lit_heartbeat_led_handler(struct k_work *work)
{
	/* Lit heartbeat LED */
	static bool led_status = true;
	led_status = !led_status;
	LOG_DBG("heartbeat LED %s ", led_status ? "OFF" : "ON");
	gpio_set(LED_NPCM_HEARTBEAT_R, led_status);
}

void lit_heartbeat_led(struct k_timer *timer)
{
	k_work_submit(&lit_heartbeat_led_work);
}

void plat_led_init(void)
{
	k_timer_start(&heartbeat_led_timer, K_NO_WAIT, K_MSEC(250));
	LOG_INF("Heartbeat LED timer started");
}
