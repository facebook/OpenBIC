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
#include <device.h>
#include <stdio.h>
#include <drivers/watchdog.h>
#include "hal_wdt.h"

#include <logging/log.h>

LOG_MODULE_REGISTER(hal_wdt);

struct k_thread wdt_thread;
K_KERNEL_STACK_MEMBER(wdt_thread_stack, WDT_THREAD_STACK_SIZE);

const struct device *wdt_dev = NULL;
static bool is_wdt_continue_feed = true;

void set_wdt_continue_feed(bool value)
{
	is_wdt_continue_feed = value;
}

void wdt_handler(void *arug0, void *arug1, void *arug2)
{
	while (1) {
		if (is_wdt_continue_feed) {
			wdt_feed(wdt_dev, 0);
		}
		k_sleep(K_MSEC(WDT_FEED_DELAY_MS));
	}
}

void wdt_init()
{
	int ret = 0;
	struct wdt_timeout_cfg wdt_config;

	wdt_dev = device_get_binding(WDT_DEVICE_NAME);
	if (!wdt_dev) {
		LOG_ERR("Cannot find %s device.", WDT_DEVICE_NAME);
		return;
	}

	wdt_config.window.min = 0U;
	wdt_config.window.max = WDT_TIMEOUT;
	// Pass NULL to use system reset handler
	wdt_config.callback = NULL;
	// Install new timeout: This function must be used before wdt_setup().
	ret = wdt_install_timeout(wdt_dev, &wdt_config);
	if (ret != 0) {
		LOG_ERR("fail to install %s timeout", WDT_DEVICE_NAME);
		return;
	}

	//This function is used for configuring global watchdog settings that affect all timeouts.
	ret = wdt_setup(wdt_dev, WDT_FLAG_RESET_CPU_CORE);
	if (ret != 0) {
		LOG_ERR("fail to setup %s", WDT_DEVICE_NAME);
		return;
	}

	k_thread_create(&wdt_thread, wdt_thread_stack, K_THREAD_STACK_SIZEOF(wdt_thread_stack),
			wdt_handler, NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&wdt_thread, "WDT_thread");
}
