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

#include "sensor.h"
#include "plat_pldm_sensor.h"
#include "plat_cpld.h"

#define VR_MUTEX_LOCK_TIMEOUT_MS 1000
#define TEMP_THRESHOLD_USER_SETTINGS_OFFSET 0x8100
#define VR_VOUT_USER_SETTINGS_OFFSET 0x8000
#define SOC_PCIE_PERST_USER_SETTINGS_OFFSET 0x8300
#define BOOTSTRAP_USER_SETTINGS_OFFSET 0x8400
#define THERMALTRIP_USER_SETTINGS_OFFSET 0x8500
#define THROTTLE_USER_SETTINGS_OFFSET 0x8600

#define CPLD_THERMALTRIP_SWITCH_ADDR 0x3A
enum USER_SETTING_OFFSET_E {
	THERMALTRIP,
};

enum PLAT_TEMP_INDEX_E {
	TEMP_INDEX_TOP_INLET,
	TEMP_INDEX_BOT_INLET,
	TEMP_INDEX_BOT_OUTLET,
	TEMP_INDEX_ASIC_MEDHA0_SENSOR0,
	TEMP_INDEX_ASIC_MEDHA0_SENSOR1,
	TEMP_INDEX_ASIC_OWL_W,
	TEMP_INDEX_ASIC_OWL_E,
	TEMP_INDEX_ASIC_MEDHA1_SENSOR0,
	TEMP_INDEX_ASIC_MEDHA1_SENSOR1,
	TEMP_INDEX_ASIC_HAMSA_CRM,
	TEMP_INDEX_ASIC_HAMSA_LS,
	TEMP_INDEX_MAX,
};
/*
SENSOR_NUM_ASIC_MEDHA0_SENSOR0_TEMP_C 0x04
SENSOR_NUM_ASIC_MEDHA0_SENSOR1_TEMP_C 0x05
SENSOR_NUM_ASIC_OWL_W_TEMP_C 0x06
SENSOR_NUM_ASIC_OWL_E_TEMP_C 0x07
SENSOR_NUM_ASIC_MEDHA1_SENSOR0_TEMP_C 0x08
SENSOR_NUM_ASIC_MEDHA1_SENSOR1_TEMP_C 0x09
SENSOR_NUM_ASIC_HAMSA_CRM_TEMP_C 0x0A
SENSOR_NUM_ASIC_HAMSA_LS_TEMP_C 0x0B
*/


enum PLAT_TEMP_INDEX_THRESHOLD_TYPE_E {
	TOP_INLET_LOW_LIMIT,
	TOP_INLET_HIGH_LIMIT,
	BOT_INLET_LOW_LIMIT,
	BOT_INLET_HIGH_LIMIT,
	BOT_OUTLET_LOW_LIMIT,
	BOT_OUTLET_HIGH_LIMIT,
	ASIC_MEDHA0_SENSOR0_LOW_LIMIT,
	ASIC_MEDHA0_SENSOR0_HIGH_LIMIT,
	ASIC_MEDHA0_SENSOR1_LOW_LIMIT,
	ASIC_MEDHA0_SENSOR1_HIGH_LIMIT,
	ASIC_OWL_W_LOW_LIMIT,
	ASIC_OWL_W_HIGH_LIMIT,
	ASIC_OWL_E_LOW_LIMIT,
	ASIC_OWL_E_HIGH_LIMIT,
	ASIC_MEDHA1_SENSOR0_LOW_LIMIT,
	ASIC_MEDHA1_SENSOR0_HIGH_LIMIT,
	ASIC_MEDHA1_SENSOR1_LOW_LIMIT,
	ASIC_MEDHA1_SENSOR1_HIGH_LIMIT,
	ASIC_HAMSA_CRM_LOW_LIMIT,
	ASIC_HAMSA_CRM_HIGH_LIMIT,
	ASIC_HAMSA_LS_LOW_LIMIT,
	ASIC_HAMSA_LS_HIGH_LIMIT,
	PLAT_TEMP_INDEX_THRESHOLD_TYPE_MAX,
};

typedef struct temp_mapping_sensor {
	uint8_t index;
	uint8_t sensor_id;
	uint8_t *sensor_name;
} temp_mapping_sensor;

typedef struct temp_threshold_mapping_sensor {
	uint8_t temp_index_threshold_type; //PLAT_TEMP_INDEX_THRESHOLD_TYPE_E
	uint8_t temp_threshold_type;
	uint8_t sensor_id;
	uint8_t *temp_threshold_name;
} temp_threshold_mapping_sensor;

typedef struct thermaltrip_user_settings_struct {
	uint8_t thermaltrip_user_setting_value;
} thermaltrip_user_settings_struct;

typedef struct temp_threshold_user_settings_struct {
	uint32_t temperature_reg_val[PLAT_TEMP_INDEX_THRESHOLD_TYPE_MAX];
} temp_threshold_user_settings_struct;


bool set_thermaltrip_user_settings(bool thermaltrip_enable, bool is_perm);
bool get_temp_sensor_rail_name(uint8_t rail, uint8_t **name);
bool get_temp_sensor_rail_enum(uint8_t *name, uint8_t *num);
bool plat_get_temp_status(uint8_t rail, uint8_t *temp_status);
bool get_temp_sensor_rail_name(uint8_t rail, uint8_t **name);
bool get_temp_threshold_type_enum(uint8_t *name, uint8_t *num);
bool get_temp_index_threshold_type_name(uint8_t type, uint8_t **name);
bool get_plat_temp_threshold(uint8_t temp_index_threshold_type, uint32_t *millidegree_celsius);
bool set_plat_temp_threshold(uint8_t temp_index_threshold_type, uint32_t *millidegree_celsius,
			     bool is_default, bool is_perm);
bool plat_clear_temp_status(uint8_t rail);
#endif
