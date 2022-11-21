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
#include "hal_peci.h"
#include "power_status.h"
#include "util_sys.h"
#include "plat_class.h"
#include "plat_gpio.h"
#include "util_worker.h"

SCU_CFG scu_cfg[] = {
	//register    value
	{ 0x7e6e2610, 0xffffffff }, //disable GPIO internal pull down #0
	{ 0x7e6e2614, 0xffffffff }, //disable GPIO internal pull down #1
	{ 0x7e6e2618, 0xF8000000 }, //disable GPIO internal pull down #2
	{ 0x7e6e261c, 0xC0200F3A }, //disable GPIO internal pull down #3
	{ 0x7e789110, 0x0ca60ca2 }, //set kcs data addr:ca2, cmd addr:ca6
};

void pal_pre_init()
{
	init_platform_config();
	disable_PRDY_interrupt();
	scu_init(scu_cfg, sizeof(scu_cfg) / sizeof(SCU_CFG));
	init_plat_worker(CONFIG_MAIN_THREAD_PRIORITY + 1); // work queue for low priority jobs
}

void pal_device_init()
{
	init_me_firmware();
}

void pal_set_sys_status()
{
	set_DC_status(PWRGD_SYS_PWROK);
	set_DC_on_delayed_status();
	set_DC_off_delayed_status();
	set_post_status(FM_BIOS_POST_CMPLT_BMC_N);
	set_CPU_power_status(PWRGD_CPU_LVC3_R);
	set_post_thread();
	set_sys_ready_pin(BIC_READY);
}

#define DEF_PROJ_GPIO_PRIORITY 78

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);
