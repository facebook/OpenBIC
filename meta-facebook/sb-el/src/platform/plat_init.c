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
#include <logging/log.h>

#include "expansion_board.h"
#include "plat_led.h"
#include "plat_i2c_target.h"
#include "plat_mctp.h"
#include "plat_hook.h"
#include "plat_fru.h"
#include "plat_adc.h"
#include "flash_shell.h"
#include "plat_class.h"
#include "plat_cpld.h"
#include "plat_log.h"
#include "plat_user_setting.h"
#include "plat_ioexp.h"
#include "plat_thermal.h"
#include "plat_power_capping.h"
#include "plat_kernel_obj.h"
#include "plat_event.h"
#include "plat_hwmon.h"
#include "plat_gpio.h"

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
	init_plat_config();
	plat_led_init();
	vr_mutex_init();
	init_pwm_dev();
	pwr_level_mutex_init();
	// plat_i3c_set_pid();
}

void pal_set_sys_status()
{
	//TODO:
}

void pal_post_init()
{
	plat_mctp_init();
	user_settings_init();
	init_fru_info();
	plat_adc_electra_init();
	plat_power_capping_init();
	init_load_eeprom_log();
	if (get_asic_board_id() == ASIC_BOARD_ID_EVB) {
		// if board id >= EVB EVT1B(FAB2)
		quick_sensor_poll_init();
		init_U200052_IO();
		init_U200053_IO();
		init_U200070_IO();
	}
	plat_set_ac_on_log();
	init_cpld_polling();
	ioexp_init();
	init_thermal_polling();

	// check the thermtrip open-circuit
	if (!gpio_get(FM_ASIC_0_THERMTRIP_R_N))
		plat_asic_thermtrip_error_log(LOG_ASSERT);
}

#define DEF_PROJ_GPIO_PRIORITY 78

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);
