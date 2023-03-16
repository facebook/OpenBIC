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
#include "plat_gpio.h"

// Disable BIC internal pull down of GPIO input pin
SCU_CFG scu_cfg[] = {
	//register    value
	{ 0x7e6e2610, 0xAAAA18B0 }, // GPIOA-D
	{ 0x7e6e2614, 0x0D543F00 }, // GPIOE-H
	{ 0x7e6e2618, 0xB8004080 }, // GPIOI-L
	{ 0x7e6e261C, 0x0000002D }, // GPIOM-P
	{ 0x7e6e2630, 0x0000001E }, // GPIOQ-S
};

void pal_pre_init()
{
	scu_init(scu_cfg, ARRAY_SIZE(scu_cfg));
}

void pal_set_sys_status()
{
	set_DC_status(PG_CARD_OK);
}


#define DEF_PROJ_GPIO_PRIORITY 61

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);
