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
 * Full-board-port main() for the MCX-N9XX-EVK: calls the *real*
 * common/ init pipeline (wdt_init, util_init_timer, util_init_I2C,
 * sensor_init, FRU_init) ported to mainline Zephyr, instead of the
 * bring-up branch's from-scratch stand-ins. util_init_i3c()/ipmi_init()
 * aren't called (see README.md - hal_i3c.c and IPMI transport both
 * need Aspeed-only APIs). A separate real-I3C attempt (plat_i3c.c,
 * i3c_do_daa() against mainline's own I3C API) was tried and reverted:
 * enabling CONFIG_I3C=y alone made the board hang before main() ever
 * runs (no boot banner at all) - a mainline I3C_MCUX driver-level
 * issue for this SoC, not something introduced by that new code. See
 * README.md for the full writeup.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include "fru.h"
#include "hal_i2c.h"
#include "hal_wdt.h"
#include "libutil.h"
#include "plat_gpio.h"
#include "plat_hwinfo.h"
#include "plat_mbox.h"
#include "plat_storage.h"
#include "plat_version.h"
#include "sensor.h"
#include "timer.h"

#define HEARTBEAT_PERIOD_MS 500
#define MBOX_PING_PERIOD_MS 2000

static const struct gpio_dt_spec heartbeat_led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

int main(void)
{
	printk("\n"
	       "  ___                   ____ ___ ____\n"
	       " / _ \\ _ __   ___ _ __ | __ )_ _/ ___|\n"
	       "| | | | '_ \\ / _ \\ '_ \\|  _ \\| | |\n"
	       "| |_| | |_) |  __/ | | | |_) | | |___\n"
	       " \\___/| .__/ \\___|_| |_|____/___\\____|\n"
	       "      |_|\n");
	printk("Hello, welcome to OpenBIC / %s %s %x%x.%x.%x\n", PLATFORM_NAME, PROJECT_NAME,
	       BIC_FW_YEAR_MSB, BIC_FW_YEAR_LSB, BIC_FW_WEEK, BIC_FW_VER);
	printk("Full board port: real common/ wdt/timer/I2C/sensor/FRU init.\n");

	plat_hwinfo_init();
	plat_storage_init();
	plat_gpio_init();
	plat_mbox_init();

	wdt_init();
	util_init_timer();
	util_init_I2C();
	sensor_init();
	FRU_init();

	if (!gpio_is_ready_dt(&heartbeat_led)) {
		printk("Heartbeat LED device not ready\n");
		return 0;
	}

	gpio_pin_configure_dt(&heartbeat_led, GPIO_OUTPUT_ACTIVE);

	int loop_count = 0;

	while (1) {
		gpio_pin_toggle_dt(&heartbeat_led);
		if (++loop_count % (MBOX_PING_PERIOD_MS / HEARTBEAT_PERIOD_MS) == 0) {
			plat_mbox_ping();
		}
		k_msleep(HEARTBEAT_PERIOD_MS);
	}

	return 0;
}
