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
#include "plat_gpio.h"
#include "expansion_board.h"
#include "plat_isr.h"
#include "plat_power_seq.h"
#include "power_status.h"
#include "cci.h"
#include "plat_mctp.h"
#include "plat_i2c_target.h"
#include "util_worker.h"

SCU_CFG scu_cfg[] = {
	//register    value
	{ 0x7e6e2610, 0x0000D7BF },
	{ 0x7e6e2614, 0x00044300 },
};

void pal_pre_init()
{
	init_platform_config();
	scu_init(scu_cfg, ARRAY_SIZE(scu_cfg));

	/* init i2c target */
	for (int index = 0; index < MAX_TARGET_NUM; index++) {
		if (I2C_TARGET_ENABLE_TABLE[index])
			i2c_target_control(
				index, (struct _i2c_target_config *)&I2C_TARGET_CONFIG_TABLE[index],
				1);
	}
	plat_mctp_init();
	/* Initialize platform work queue for handle CXL power on/off sequence */
	init_plat_worker(CONFIG_MAIN_THREAD_PRIORITY);
}

void pal_set_sys_status()
{
	set_MB_DC_status(FM_POWER_EN);
	set_DC_status(PWRGD_CARD_PWROK);
	set_DC_on_delayed_status();
	set_DC_off_delayed_status();
}

#define DEF_PROJ_GPIO_PRIORITY 61

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);
