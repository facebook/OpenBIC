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

#include <zephyr/kernel.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/logging/log.h>

#include "plat_hwinfo.h"

LOG_MODULE_REGISTER(plat_hwinfo, LOG_LEVEL_INF);

static uint8_t device_id[16];
static size_t device_id_len;
static uint32_t reset_cause;

static const char *reset_cause_str(uint32_t cause)
{
	if (cause & RESET_PIN) {
		return "pin";
	}
	if (cause & RESET_SOFTWARE) {
		return "software";
	}
	if (cause & RESET_WATCHDOG) {
		return "watchdog";
	}
	if (cause & RESET_POR) {
		return "power-on";
	}
	if (cause == 0) {
		return "unknown/not-supported";
	}
	return "other";
}

void plat_hwinfo_init(void)
{
	ssize_t ret;
	char hex[sizeof(device_id) * 2 + 1] = {0};

	ret = hwinfo_get_device_id(device_id, sizeof(device_id));
	if (ret > 0) {
		device_id_len = (size_t)ret;
		for (size_t i = 0; i < device_id_len; i++) {
			snprintf(&hex[i * 2], 3, "%02x", device_id[i]);
		}
		LOG_INF("device id: %s", hex);
	} else {
		LOG_ERR("hwinfo_get_device_id failed: %d", (int)ret);
	}

	if (hwinfo_get_reset_cause(&reset_cause) == 0) {
		LOG_INF("last reset cause: %s (0x%08x)", reset_cause_str(reset_cause),
			reset_cause);
		hwinfo_clear_reset_cause();
	} else {
		LOG_ERR("hwinfo_get_reset_cause failed");
	}
}
