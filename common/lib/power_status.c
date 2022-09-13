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

#include "power_status.h"

#include <stdio.h>
#include <logging/log.h>

#include "hal_gpio.h"
#include "snoop.h"

LOG_MODULE_REGISTER(power_status);

static bool is_DC_on = false;
static bool is_DC_on_delayed = false;
static bool is_DC_off_delayed = false;
static bool is_CPU_power_good = false;
static bool is_post_complete = false;
static bool vr_monitor_status = true;

void set_DC_status(uint8_t gpio_num)
{
	is_DC_on = (gpio_get(gpio_num) == 1) ? true : false;
	LOG_WRN("DC_STATUS: %s", (is_DC_on) ? "on" : "off");
}

bool get_DC_status()
{
	return is_DC_on;
}

void set_DC_on_delayed_status()
{
	is_DC_on_delayed = is_DC_on;
}

bool get_DC_on_delayed_status()
{
	return is_DC_on_delayed;
}

void set_DC_off_delayed_status()
{
	is_DC_off_delayed = !is_DC_on;
}

bool get_DC_off_delayed_status()
{
	return is_DC_off_delayed;
}

void set_post_status(uint8_t gpio_num)
{
	is_post_complete = (gpio_get(gpio_num) == 1) ? false : true;
	LOG_WRN("POST_COMPLETE: %s", (is_post_complete) ? "yes" : "no");
}

bool get_post_status()
{
	return is_post_complete;
}

void set_CPU_power_status(uint8_t gpio_num)
{
	is_CPU_power_good = gpio_get(gpio_num);
	LOG_WRN("CPU_PWR_GOOD: %s", (is_CPU_power_good) ? "yes" : "no");
}

bool CPU_power_good()
{
	return is_CPU_power_good;
}

void set_post_thread()
{
	if (CPU_power_good() == true) {
#ifdef CONFIG_SNOOP_ASPEED
		init_snoop_thread();
		init_send_postcode_thread();
#endif
	}
}

void set_vr_monitor_status(bool value)
{
	vr_monitor_status = value;
}

bool get_vr_monitor_status()
{
	return vr_monitor_status;
}
