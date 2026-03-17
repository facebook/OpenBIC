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

#ifndef PLAT_USER_SETTING_H
#define PLAT_USER_SETTING_H

#include "plat_cpld.h"
#include "plat_pldm_sensor.h"

#define MMC_SLOT_USER_SETTING_OFFSET 0x0000

#define VR_MUTEX_LOCK_TIMEOUT_MS 1000
#define TEMP_THRESHOLD_USER_SETTINGS_OFFSET 0x8100
#define VR_VOUT_USER_SETTINGS_OFFSET 0x8000
#define ALERT_LEVEL_USER_SETTINGS_OFFSET 0x8200
#define DELAY_PCIE_PERST_USER_SETTINGS_OFFSET 0x8300
#define BOOTSTRAP_USER_SETTINGS_OFFSET 0x8400
#define THERMALTRIP_USER_SETTINGS_OFFSET 0x8500
#define THROTTLE_USER_SETTINGS_OFFSET 0x8600
#define DELAY_ASIC_RST_USER_SETTINGS_OFFSET 0x8700
#define DELAY_MODULE_PG_USER_SETTINGS_OFFSET 0x8800

#define CPLD_THROTTLE_SWITCH_ADDR 0x25
#define CPLD_THERMALTRIP_SWITCH_ADDR 0x3A

enum PLAT_TEMP_INDEX_E {
	TEMP_INDEX_TOP_INLET,
	TEMP_INDEX_BOT_INLET,
	TEMP_INDEX_BOT_OUTLET,
	TEMP_INDEX_ASIC_NUWA0_SENSOR0,
	TEMP_INDEX_ASIC_NUWA0_SENSOR1,
	TEMP_INDEX_ASIC_OWL_W,
	TEMP_INDEX_ASIC_OWL_E,
	TEMP_INDEX_ASIC_NUWA1_SENSOR0,
	TEMP_INDEX_ASIC_NUWA1_SENSOR1,
	TEMP_INDEX_ASIC_HAMSA_CRM,
	TEMP_INDEX_ASIC_HAMSA_LS,
	TEMP_INDEX_MAX,
};

void user_settings_init(void);

#endif