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
#include <string.h>
#include "libutil.h"
#include <logging/log.h>
#include "plat_hook.h"
#include "pmbus.h"
#include "plat_gpio.h"
#include "plat_pldm_sensor.h"
#include "plat_cpld.h"
#include "plat_i2c.h"
#include "plat_fru.h"
#include "plat_user_setting.h"

LOG_MODULE_REGISTER(plat_user_setting);

#define EEPROM_MAX_WRITE_TIME 5 // the BR24G512 eeprom max write time is 3.5 ms

bool set_user_settings_thermaltrip_to_eeprom(void *thermaltrip_user_settings, uint8_t data_length)
{
	CHECK_NULL_ARG_WITH_RETURN(thermaltrip_user_settings, false);

	if (!plat_eeprom_write(THERMALTRIP_USER_SETTINGS_OFFSET, thermaltrip_user_settings,
			       data_length)) {
		LOG_ERR("Failed to write thermaltrip to eeprom");
		return false;
	}

	k_msleep(EEPROM_MAX_WRITE_TIME);

	return true;
}

thermaltrip_user_settings_struct thermaltrip_user_settings = { 0xFF };
bool set_thermaltrip_user_settings(bool thermaltrip_enable, bool is_perm)
{
	uint8_t thermaltrip_enable_value = thermaltrip_enable ? 1 : 0;
	if (!plat_write_cpld(CPLD_THERMALTRIP_SWITCH_ADDR, &thermaltrip_enable_value)) {
		LOG_ERR("Failed to read thermaltrip CPLD");
		return false;
	}

	if (is_perm) {
		thermaltrip_user_settings.thermaltrip_user_setting_value =
			(thermaltrip_enable ? 0x01 : 0x00);

		if (!set_user_settings_thermaltrip_to_eeprom(&thermaltrip_user_settings,
							     sizeof(thermaltrip_user_settings))) {
			LOG_ERR("Failed to write thermaltrip to eeprom error");
			return false;
		}
	}
	return true;
}
