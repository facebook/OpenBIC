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
#include "power_status.h"
#include "util_sys.h"
#include "plat_gpio.h"

/*
 * The operating voltage of GPIO input pins are lower than actual voltage because the chip 
 * internal pull-down is enabled.
 * BIC disables the internal GPIO pull-down for all input pins.
 *
 * Disabled GPIO list (Refer to chapter 18 of AST1030's datasheet) :
 * GPIOA0 GPIOA1 GPIOA2 GPIOA4
 * GPIOB5 GPIOB6
 * GPIOC0 GPIOC1 GPIOC2 GPIOC4 GPIOC5 GPIOC6 GPIOC7
 * GPIOD0 GPIOD2
 * GPIOF1 GPIOF4
 * GPIOG0 GPIOG2 GPIOG5 GPIOG6 GPIOG7
 * GPIOH1 GPIOH2 GPIOH3
 * GPIOL3 GPIOL4 GPIOL5 GPIOL6 GPIOL7
 * GPIOM1 GPIOM4 GPIOM5
 */
SCU_CFG scu_cfg[] = {
	//register    value
	{ 0x7e6e2610, 0x05F76017 },
	{ 0x7e6e2614, 0x0EE51200 },
	{ 0x7e6e2618, 0xF8000000 },
	{ 0x7e6e261C, 0x00000032 },
};

void pal_pre_init()
{
	scu_init(scu_cfg, ARRAY_SIZE(scu_cfg));
}

void pal_set_sys_status()
{
/*
 *   TODO :
 *   1. Set DC status by GPIO
 *   2. Set Post complete by GPIO
 */

	set_CPU_power_status(PWRGD_CPU_LVC3);
	set_post_thread();
	set_sys_ready_pin(BIC_READY);
}

#define DEF_PROJ_GPIO_PRIORITY 78

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);
