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

#include "util_sys.h"
#include "hal_gpio.h"
#include "plat_gpio.h"
#include "plat_fan.h"
#include <plat_def.h>

void pal_pre_init()
{
	init_fan_mode();
	init_fan_duty();

	// Due to BB CPLD bind HSC device need times
	// wait HSC ready before sensor read
	k_msleep(HSC_DEVICE_READY_DELAY_MS);
}

void pal_set_sys_status()
{
	set_sys_ready_pin(BIC_READY_R);
}

#define DEF_PROJ_GPIO_PRIORITY 61

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);
