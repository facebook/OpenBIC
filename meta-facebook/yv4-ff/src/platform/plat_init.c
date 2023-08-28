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

#include "hal_gpio.h"

#define DEF_PROJ_GPIO_PRIORITY 78

/*
 * The operating voltage of GPIO input pins are lower than actual voltage because the chip 
 * internal pull-down is enabled.
 * BIC disables the internal GPIO pull-down for all input pins.
 *
 * Disabled GPIO list (Refer to chapter 18 of AST1030's datasheet) :
 * GPIOA3
 * GPIOE0~GPIOE7
 * GPIOH1~GPIOU7
 */

// clang-format off
SCU_CFG scu_cfg[] = {
	//register    value
	{ 0x7e6e2610, 0xFFFFFFF7 },
	{ 0x7e6e2614, 0x01FFFF00 },
	{ 0x7e6e2618, 0x00000000 },
	{ 0x7e6e261c, 0x00000000 },
	{ 0x7e6e2630, 0x00000000 },
	{ 0x7e6e2634, 0x00000000 },
};
// clang-format on

void pal_pre_init()
{
	scu_init(scu_cfg, sizeof(scu_cfg) / sizeof(SCU_CFG));
}

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);
