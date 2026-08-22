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
 * Real watchdog subsystem, ported to mainline Zephyr's wdt API from
 * scratch (not common/hal/hal_wdt.c - see the top-level README). Plays
 * the same role wdt_init()/feeding does in common/service/main.c: a
 * real BIC must reset itself if its main loop hangs.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>

#include "plat_wdt.h"

LOG_MODULE_REGISTER(plat_wdt, LOG_LEVEL_INF);

#define WDT_TIMEOUT_MS 3000

static const struct device *const wdt = DEVICE_DT_GET(DT_ALIAS(watchdog0));
static int wdt_channel_id = -1;
static bool feeding_enabled = true;

void plat_wdt_init(void)
{
	int ret;
	struct wdt_timeout_cfg cfg = {
		.window.min = 0,
		.window.max = WDT_TIMEOUT_MS,
		.flags = WDT_FLAG_RESET_SOC,
	};

	if (!device_is_ready(wdt)) {
		LOG_ERR("watchdog0 device not ready");
		return;
	}

	wdt_channel_id = wdt_install_timeout(wdt, &cfg);
	if (wdt_channel_id < 0) {
		LOG_ERR("wdt_install_timeout failed: %d", wdt_channel_id);
		return;
	}

	ret = wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
	if (ret < 0) {
		LOG_ERR("wdt_setup failed: %d", ret);
		wdt_channel_id = -1;
		return;
	}

	LOG_INF("watchdog armed: %dms timeout, resets SoC on starvation", WDT_TIMEOUT_MS);
}

void plat_wdt_feed(void)
{
	if (wdt_channel_id < 0 || !feeding_enabled) {
		return;
	}
	wdt_feed(wdt, wdt_channel_id);
}

void plat_wdt_stop_feeding(void)
{
	feeding_enabled = false;
	LOG_WRN("watchdog feeding stopped on purpose - SoC will reset in <= %dms", WDT_TIMEOUT_MS);
}
