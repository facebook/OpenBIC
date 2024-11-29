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

#include "util_worker.h"
#include "ipmi.h"
#include "sensor.h"
#include "power_status.h"
#include "expansion_board.h"

#include "plat_sensor_table.h"
#include "plat_class.h"
#include "plat_hwmon.h"
#include "plat_isr.h"
#include "plat_led.h"
#include "plat_gpio.h"
#include "plat_i2c.h"
#include "plat_i2c_target.h"
#include "plat_mctp.h"
#include "plat_pldm_monitor.h"
#include "plat_hook.h"
#include <logging/log.h>
#include "plat_event.h"
#include "plat_log.h"

LOG_MODULE_REGISTER(plat_init);

void pal_pre_init()
{
	/* init i2c target */
	for (int index = 0; index < MAX_TARGET_NUM; index++) {
		if (I2C_TARGET_ENABLE_TABLE[index])
			i2c_target_control(
				index, (struct _i2c_target_config *)&I2C_TARGET_CONFIG_TABLE[index],
				1);
	}

	init_platform_config();
	plat_led_init();
	vr_mutex_init();
	plat_clock_init();
	plat_eusb_init();
}

void pal_set_sys_status()
{
	//TODO:
}

void pal_post_init()
{
	plat_mctp_init();
	pldm_load_state_effecter_table(MAX_STATE_EFFECTER_IDX);
	pldm_assign_gpio_effecter_id(PLAT_EFFECTER_ID_GPIO_HIGH_BYTE);
	init_load_eeprom_log();
	init_cpld_polling();
}

#define DEF_PROJ_GPIO_PRIORITY 78

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);
