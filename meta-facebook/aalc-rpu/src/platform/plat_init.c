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

#include <stdio.h>
#include "hal_gpio.h"
#include "plat_pwm.h"
#include <logging/log.h>
#include "plat_class.h"
#include "plat_modbus.h"
#include "plat_util.h"
#include "hal_gpio.h"
#include "plat_threshold.h"
#include "plat_log.h"
#include "plat_gpio.h"

LOG_MODULE_REGISTER(plat_init);

#define DEF_PROJ_GPIO_PRIORITY 78

SCU_CFG scu_cfg[] = {
	//register    value
	{ 0x7e789108, 0x00000500 }, // uart1,2 RS485 DE/nRE
	{ 0x7e6e24b0, 0x00008000 }, // uart2 NRTS2
	{ 0x7e6e24bc, 0x00000001 }, // uart1 NRTS1
	// disable internal PD
	{ 0x7e6e2610, 0x0FFFBFFF },
	{ 0x7e6e2614, 0x018F0100 },
	{ 0x7e6e2618, 0x0F00FF00 },
	{ 0x7e6e261C, 0xFE300005 },
	{ 0x7e6e2630, 0x00000002 },
};

void pal_pre_init()
{
	scu_init(scu_cfg, sizeof(scu_cfg) / sizeof(SCU_CFG));
	init_aalc_config();
	gpio_set(FM_BIC_READY_R_N, 0); //MM4 for bus3 power up
}

void pal_post_init()
{
	init_load_eeprom_log();
	init_pwm_dev();
	init_custom_modbus_server();
	init_modbus_command_table();
	//threshold_poll_init();
	set_rpu_ready();
}

void pal_device_init()
{
	return;
}

void pal_set_sys_status()
{
	return;
}

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);
