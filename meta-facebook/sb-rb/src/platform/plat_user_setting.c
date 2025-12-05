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
#include "sensor.h"
#include "tmp75.h"
#include "tmp431.h"
#include "plat_class.h"
#include "shell_plat_average_power.h"
#include "shell_plat_throttle_switch.h"
#include "pldm_monitor.h"
#include "sensor.h"
#include "pldm_sensor.h"

LOG_MODULE_REGISTER(plat_user_setting);

#define EEPROM_MAX_WRITE_TIME 5 // the BR24G512 eeprom max write time is 3.5 ms
#define TMP75_ALERT_CPLD_OFFSET 0x2F

int32_t alert_level_mA_default = 180000;
int32_t alert_level_mA_user_setting = 180000;
static bool uart_pwr_event_is_enable = true;
bool alert_level_is_assert = false;
static struct k_mutex pwrlevel_mutex;
static uint8_t power_index[UBC_VR_RAIL_E_MAX] = { 0 };
static uint8_t power_count[UBC_VR_RAIL_E_MAX] = { 0 };

void pwr_level_mutex_init(void)
{
	k_mutex_init(&pwrlevel_mutex);

	return;
}

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
bool get_user_settings_thermaltrip_from_eeprom(void *thermaltrip_user_settings, uint8_t data_length)
{
	CHECK_NULL_ARG_WITH_RETURN(thermaltrip_user_settings, false);

	if (!plat_eeprom_read(THERMALTRIP_USER_SETTINGS_OFFSET, thermaltrip_user_settings,
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
static bool thermaltrip_user_settings_init(void)
{
	uint8_t setting_data = 0xFF;
	if (!get_user_settings_thermaltrip_from_eeprom(&setting_data, sizeof(setting_data))) {
		LOG_ERR("get thermaltrip user settings fail");
		return false;
	}

	if (setting_data != 0xFF) {
		if (!plat_write_cpld(CPLD_THERMALTRIP_SWITCH_ADDR, &setting_data)) {
			LOG_ERR("Can't set thermaltrip=%d by user settings", setting_data);
			return false;
		}
	}

	return true;
}
temp_mapping_sensor temp_index_table[] = {
	{ TEMP_INDEX_TOP_INLET, SENSOR_NUM_TOP_INLET_TEMP_C, "SB_RB_TOP_INLET_TEMP" },
	{ TEMP_INDEX_BOT_INLET, SENSOR_NUM_BOT_INLET_TEMP_C, "SB_RB_BOT_INLET_TEMP" },
	{ TEMP_INDEX_BOT_OUTLET, SENSOR_NUM_BOT_OUTLET_TEMP_C, "SB_RB_BOT_OUTLET_TEMP" },
	{ TEMP_INDEX_ASIC_MEDHA0_SENSOR0, SENSOR_NUM_ASIC_MEDHA0_SENSOR0_TEMP_C,
	  "SB_RB_ASIC_MEDHA0_SENSOR0_TEMP" },
	{ TEMP_INDEX_ASIC_MEDHA0_SENSOR1, SENSOR_NUM_ASIC_MEDHA0_SENSOR1_TEMP_C,
	  "SB_RB_ASIC_MEDHA0_SENSOR1_TEMP" },
	{ TEMP_INDEX_ASIC_OWL_W, SENSOR_NUM_ASIC_OWL_W_TEMP_C, "SB_RB_ASIC_OWL_W_TEMP" },
	{ TEMP_INDEX_ASIC_OWL_E, SENSOR_NUM_ASIC_OWL_E_TEMP_C, "SB_RB_ASIC_OWL_E_TEMP" },
	{ TEMP_INDEX_ASIC_MEDHA1_SENSOR0, SENSOR_NUM_ASIC_MEDHA1_SENSOR0_TEMP_C,
	  "SB_RB_ASIC_MEDHA1_SENSOR0_TEMP" },
	{ TEMP_INDEX_ASIC_MEDHA1_SENSOR1, SENSOR_NUM_ASIC_MEDHA1_SENSOR1_TEMP_C,
	  "SB_RB_ASIC_MEDHA1_SENSOR1_TEMP" },
	{ TEMP_INDEX_ASIC_HAMSA_CRM, SENSOR_NUM_ASIC_HAMSA_CRM_TEMP_C,
	  "SB_RB_ASIC_HAMSA_CRM_TEMP" },
	{ TEMP_INDEX_ASIC_HAMSA_LS, SENSOR_NUM_ASIC_HAMSA_LS_TEMP_C, "SB_RB_ASIC_HAMSA_LS_TEMP" },
};

bool get_temp_sensor_rail_enum(uint8_t *name, uint8_t *num)
{
	CHECK_NULL_ARG_WITH_RETURN(name, false);
	CHECK_NULL_ARG_WITH_RETURN(num, false);

	for (int i = 0; i < TEMP_INDEX_MAX; i++) {
		if (strcmp(name, temp_index_table[i].sensor_name) == 0) {
			*num = temp_index_table[i].index;
			return true;
		}
	}

	LOG_ERR("invalid rail name %s", log_strdup(name));
	return false;
}

bool plat_get_temp_status(uint8_t rail, uint8_t *temp_status)
{
	CHECK_NULL_ARG_WITH_RETURN(temp_status, false);

	bool ret = false;
	uint8_t sensor_id = temp_index_table[rail].sensor_id;
	sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);

	if (cfg == NULL) {
		LOG_ERR("Failed to get sensor config for sensor 0x%x", sensor_id);
		return false;
	}

	switch (cfg->type) {
	case sensor_dev_tmp75: {
		uint8_t data[1] = { 0 };
		if (!plat_read_cpld(TMP75_ALERT_CPLD_OFFSET, data, 1)) {
			LOG_ERR("Failed to read TEMP TMP75 from cpld");
			goto err;
		}

		switch (rail) {
		case TEMP_INDEX_TOP_INLET:
			*temp_status = (data[0] & BIT(7)) >> 7;
			break;
		case TEMP_INDEX_BOT_INLET:
			*temp_status = (data[0] & BIT(6)) >> 6;
			break;
		case TEMP_INDEX_BOT_OUTLET:
			*temp_status = (data[0] & BIT(5)) >> 5;
			break;
		default:
			LOG_ERR("Unsupport TEMP TMP75 alert pin");
			goto err;
		}
	} break;
	case sensor_dev_tmp431:
		if (!tmp432_get_temp_status(cfg, temp_status)) {
			LOG_ERR("The TEMP TMP432 temp status reading failed");
			goto err;
		}
		break;
	default:
		LOG_ERR("Unsupport TEMP type(%x)", cfg->type);
		goto err;
	}

	ret = true;
err:
	return ret;
}

bool get_temp_sensor_rail_name(uint8_t rail, uint8_t **name)
{
	CHECK_NULL_ARG_WITH_RETURN(name, false);

	if (rail >= TEMP_INDEX_MAX) {
		*name = NULL;
		return false;
	}

	*name = (uint8_t *)temp_index_table[rail].sensor_name;
	return true;
}
// to do need check remot or local
temp_threshold_mapping_sensor temp_index_threshold_type_table[] = {
	{ TOP_INLET_LOW_LIMIT, LOCAL_LOW_LIMIT, SENSOR_NUM_TOP_INLET_TEMP_C,
	  "SB_RB_TOP_INLET_TEMP_LOW_LIM" },
	{ TOP_INLET_HIGH_LIMIT, LOCAL_HIGH_LIMIT, SENSOR_NUM_TOP_INLET_TEMP_C,
	  "SB_RB_TOP_INLET_TEMP_HIGH_LIM" },
	{ BOT_INLET_LOW_LIMIT, LOCAL_LOW_LIMIT, SENSOR_NUM_BOT_INLET_TEMP_C,
	  "SB_RB_BOT_INLET_TEMP_LOW_LIM" },
	{ BOT_INLET_HIGH_LIMIT, LOCAL_HIGH_LIMIT, SENSOR_NUM_BOT_INLET_TEMP_C,
	  "SB_RB_BOT_INLET_TEMP_HIGH_LIM" },
	{ BOT_OUTLET_LOW_LIMIT, LOCAL_LOW_LIMIT, SENSOR_NUM_BOT_OUTLET_TEMP_C,
	  "SB_RB_BOT_OUTLET_TEMP_LOW_LIM" },
	{ BOT_OUTLET_HIGH_LIMIT, LOCAL_HIGH_LIMIT, SENSOR_NUM_BOT_OUTLET_TEMP_C,
	  "SB_RB_BOT_OUTLET_TEMP_HIGH_LIM" },
	{ ASIC_MEDHA0_SENSOR0_LOW_LIMIT, REMOTE_1_LOW_LIMIT, SENSOR_NUM_ASIC_MEDHA0_SENSOR0_TEMP_C,
	  "SB_RB_ASIC_MEDHA0_SENSOR0_TEMP_LOW_LIM" },
	{ ASIC_MEDHA0_SENSOR0_HIGH_LIMIT, REMOTE_1_HIGH_LIMIT,
	  SENSOR_NUM_ASIC_MEDHA0_SENSOR0_TEMP_C, "SB_RB_ASIC_MEDHA0_SENSOR0_TEMP_HIGH_LIM" },
	{ ASIC_MEDHA0_SENSOR1_LOW_LIMIT, REMOTE_2_LOW_LIMIT, SENSOR_NUM_ASIC_MEDHA0_SENSOR1_TEMP_C,
	  "SB_RB_ASIC_MEDHA0_SENSOR1_TEMP_LOW_LIM" },
	{ ASIC_MEDHA0_SENSOR1_HIGH_LIMIT, REMOTE_2_HIGH_LIMIT,
	  SENSOR_NUM_ASIC_MEDHA0_SENSOR1_TEMP_C, "SB_RB_ASIC_MEDHA0_SENSOR1_TEMP_HIGH_LIM" },
	{ ASIC_OWL_W_LOW_LIMIT, REMOTE_1_LOW_LIMIT, SENSOR_NUM_ASIC_OWL_W_TEMP_C,
	  "SB_RB_ASIC_OWL_W_TEMP_LOW_LIM" },
	{ ASIC_OWL_W_HIGH_LIMIT, REMOTE_1_HIGH_LIMIT, SENSOR_NUM_ASIC_OWL_W_TEMP_C,
	  "SB_RB_ASIC_OWL_W_TEMP_HIGH_LIM" },
	{ ASIC_OWL_E_LOW_LIMIT, REMOTE_2_LOW_LIMIT, SENSOR_NUM_ASIC_OWL_E_TEMP_C,
	  "SB_RB_ASIC_OWL_E_TEMP_LOW_LIM" },
	{ ASIC_OWL_E_HIGH_LIMIT, REMOTE_2_HIGH_LIMIT, SENSOR_NUM_ASIC_OWL_E_TEMP_C,
	  "SB_RB_ASIC_OWL_E_TEMP_HIGH_LIM" },
	{ ASIC_MEDHA1_SENSOR0_LOW_LIMIT, REMOTE_1_LOW_LIMIT, SENSOR_NUM_ASIC_MEDHA1_SENSOR0_TEMP_C,
	  "SB_RB_ASIC_MEDHA1_SENSOR0_TEMP_LOW_LIM" },
	{ ASIC_MEDHA1_SENSOR0_HIGH_LIMIT, REMOTE_1_HIGH_LIMIT,
	  SENSOR_NUM_ASIC_MEDHA1_SENSOR0_TEMP_C, "SB_RB_ASIC_MEDHA1_SENSOR0_TEMP_HIGH_LIM" },
	{ ASIC_MEDHA1_SENSOR1_LOW_LIMIT, REMOTE_2_LOW_LIMIT, SENSOR_NUM_ASIC_MEDHA1_SENSOR1_TEMP_C,
	  "SB_RB_ASIC_MEDHA1_SENSOR1_TEMP_LOW_LIM" },
	{ ASIC_MEDHA1_SENSOR1_HIGH_LIMIT, REMOTE_2_HIGH_LIMIT,
	  SENSOR_NUM_ASIC_MEDHA1_SENSOR1_TEMP_C, "SB_RB_ASIC_MEDHA1_SENSOR1_TEMP_HIGH_LIM" },
	{ ASIC_HAMSA_CRM_LOW_LIMIT, REMOTE_1_LOW_LIMIT, SENSOR_NUM_ASIC_HAMSA_CRM_TEMP_C,
	  "SB_RB_ASIC_HAMSA_CRM_TEMP_LOW_LIM" },
	{ ASIC_HAMSA_CRM_HIGH_LIMIT, REMOTE_1_HIGH_LIMIT, SENSOR_NUM_ASIC_HAMSA_CRM_TEMP_C,
	  "SB_RB_ASIC_HAMSA_CRM_TEMP_HIGH_LIM" },
	{ ASIC_HAMSA_LS_LOW_LIMIT, REMOTE_2_LOW_LIMIT, SENSOR_NUM_ASIC_HAMSA_LS_TEMP_C,
	  "SB_RB_ASIC_HAMSA_LS_TEMP_LOW_LIM" },
	{ ASIC_HAMSA_LS_HIGH_LIMIT, REMOTE_2_HIGH_LIMIT, SENSOR_NUM_ASIC_HAMSA_LS_TEMP_C,
	  "SB_RB_ASIC_HAMSA_LS_TEMP_HIGH_LIM" },
};

bool set_temp_threshold_user_settings(void *temp_threshold_user_settings)
{
	CHECK_NULL_ARG_WITH_RETURN(temp_threshold_user_settings, false);

	/*write the temp_threshold_user_settings to eeprom */
	if (!plat_eeprom_write(TEMP_THRESHOLD_USER_SETTINGS_OFFSET, temp_threshold_user_settings,
			       sizeof(struct temp_threshold_user_settings_struct))) {
		LOG_ERR("Failed to write temp_threshold_user_settings to eeprom");
		return false;
	}

	k_msleep(EEPROM_MAX_WRITE_TIME);

	return true;
}

temp_threshold_user_settings_struct temp_threshold_user_settings = { 0 };
struct temp_threshold_user_settings_struct temp_threshold_default_settings = { 0 };
bool set_plat_temp_threshold(uint8_t temp_index_threshold_type, uint32_t *millidegree_celsius,
			     bool is_default, bool is_perm)
{
	CHECK_NULL_ARG_WITH_RETURN(millidegree_celsius, false);

	if (temp_index_threshold_type >= PLAT_TEMP_INDEX_THRESHOLD_TYPE_MAX) {
		LOG_ERR("Invalid temp threshold type(%x)", temp_index_threshold_type);
		return false;
	}

	uint8_t temp_threshold_type_tmp =
		temp_index_threshold_type_table[temp_index_threshold_type].temp_threshold_type;

	uint8_t sensor_id = temp_index_threshold_type_table[temp_index_threshold_type].sensor_id;
	sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);
	uint32_t setting_millidegree_celsius = *millidegree_celsius;

	if (cfg == NULL) {
		LOG_ERR("Failed to get sensor config for sensor 0x%x", sensor_id);
		return false;
	}

	if (is_default) {
		*millidegree_celsius = temp_threshold_default_settings
					       .temperature_reg_val[temp_index_threshold_type];
		setting_millidegree_celsius =
			temp_threshold_default_settings
				.temperature_reg_val[temp_index_threshold_type];
	}

	switch (cfg->type) {
	case sensor_dev_tmp431:
		if (!tmp432_set_temp_threshold(cfg, temp_threshold_type_tmp, millidegree_celsius)) {
			LOG_ERR("The TMP431 temp threshold setting failed");
			return false;
		}
		break;
	case sensor_dev_tmp75:
		if (!tmp75_set_temp_threshold(cfg, temp_threshold_type_tmp, millidegree_celsius)) {
			LOG_ERR("The TMP75 temp threshold setting failed");
			return false;
		}
		break;
	default:
		LOG_ERR("Unsupport temp type(%x)", cfg->type);
		return false;
	}

	if (is_perm) {
		temp_threshold_user_settings.temperature_reg_val[temp_index_threshold_type] =
			setting_millidegree_celsius;
		set_temp_threshold_user_settings(&temp_threshold_user_settings);
	}

	return true;
}

bool get_plat_temp_threshold(uint8_t temp_index_threshold_type, int32_t *millidegree_celsius)
{
	CHECK_NULL_ARG_WITH_RETURN(millidegree_celsius, false);

	if (temp_index_threshold_type >= PLAT_TEMP_INDEX_THRESHOLD_TYPE_MAX) {
		LOG_ERR("Invalid temp threshold type(%x)", temp_index_threshold_type);
		return false;
	}

	uint8_t temp_threshold_type_tmp =
		temp_index_threshold_type_table[temp_index_threshold_type].temp_threshold_type;

	uint8_t sensor_id = temp_index_threshold_type_table[temp_index_threshold_type].sensor_id;
	sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);

	if (cfg == NULL) {
		LOG_ERR("Failed to get sensor config for sensor 0x%x", sensor_id);
		return false;
	}

	switch (cfg->type) {
	case sensor_dev_tmp431:
		if (!tmp432_get_temp_threshold(cfg, temp_threshold_type_tmp, millidegree_celsius)) {
			LOG_ERR("The TMP431 temp threshold reading failed");
			return false;
		}
		break;
	case sensor_dev_tmp75:
		if (!tmp75_get_temp_threshold(cfg, temp_threshold_type_tmp, millidegree_celsius)) {
			LOG_ERR("The TMP75 temp threshold reading failed");
			return false;
		}
		break;
	default:
		LOG_ERR("Unsupport temp type(%x)", cfg->type);
		return false;
	}

	return true;
}

bool get_temp_threshold_type_enum(uint8_t *name, uint8_t *num)
{
	CHECK_NULL_ARG_WITH_RETURN(name, false);
	CHECK_NULL_ARG_WITH_RETURN(num, false);

	for (int i = 0; i < PLAT_TEMP_INDEX_THRESHOLD_TYPE_MAX; i++) {
		if (strcmp(name, temp_index_threshold_type_table[i].temp_threshold_name) == 0) {
			*num = i;
			return true;
		}
	}

	LOG_ERR("invalid tmp threshold type name %s", name);
	return false;
}

bool get_temp_index_threshold_type_name(uint8_t type, uint8_t **name)
{
	CHECK_NULL_ARG_WITH_RETURN(name, false);

	if (type >= PLAT_TEMP_INDEX_THRESHOLD_TYPE_MAX) {
		*name = NULL;
		return false;
	}

	*name = (uint8_t *)temp_index_threshold_type_table[type].temp_threshold_name;
	return true;
}

bool plat_clear_temp_status(uint8_t rail)
{
	bool ret = false;
	uint8_t sensor_id = temp_index_table[rail].sensor_id;
	sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);

	if (cfg == NULL) {
		LOG_ERR("Failed to get sensor config for sensor 0x%x", sensor_id);
		return false;
	}

	switch (cfg->type) {
	case sensor_dev_tmp431:
		if (!tmp432_clear_temp_status(cfg)) {
			LOG_ERR("The TEMP TMP432 temp status clear failed");
			goto err;
		}
		break;
	case sensor_dev_tmp75: {
		LOG_DBG("TMP75 temp_status cannot be cleared; its behavior depends on the temp_threshold settings.");
	} break;
	default:
		LOG_ERR("Unsupport TEMP type(%x)", cfg->type);
		goto err;
	}

	ret = true;
err:
	return ret;
}
bool plat_set_temp_threshold(uint8_t temp_index_threshold_type, uint32_t *millidegree_celsius,
			     bool is_default, bool is_perm)
{
	CHECK_NULL_ARG_WITH_RETURN(millidegree_celsius, false);

	if (temp_index_threshold_type >= PLAT_TEMP_INDEX_THRESHOLD_TYPE_MAX) {
		LOG_ERR("Invalid temp threshold type(%x)", temp_index_threshold_type);
		return false;
	}

	uint8_t temp_threshold_type_tmp =
		temp_index_threshold_type_table[temp_index_threshold_type].temp_threshold_type;

	uint8_t sensor_id = temp_index_threshold_type_table[temp_index_threshold_type].sensor_id;
	sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);
	uint32_t setting_millidegree_celsius = *millidegree_celsius;

	if (cfg == NULL) {
		LOG_ERR("Failed to get sensor config for sensor 0x%x", sensor_id);
		return false;
	}

	if (is_default) {
		*millidegree_celsius = temp_threshold_default_settings
					       .temperature_reg_val[temp_index_threshold_type];
		setting_millidegree_celsius =
			temp_threshold_default_settings
				.temperature_reg_val[temp_index_threshold_type];
	}

	switch (cfg->type) {
	case sensor_dev_tmp431:
		if (!tmp432_set_temp_threshold(cfg, temp_threshold_type_tmp, millidegree_celsius)) {
			LOG_ERR("The TMP431 temp threshold setting failed");
			return false;
		}
		break;
	case sensor_dev_tmp75:
		if (!tmp75_set_temp_threshold(cfg, temp_threshold_type_tmp, millidegree_celsius)) {
			LOG_ERR("The TMP75 temp threshold setting failed");
			return false;
		}
		break;
	default:
		LOG_ERR("Unsupport temp type(%x)", cfg->type);
		return false;
	}

	if (is_perm) {
		temp_threshold_user_settings.temperature_reg_val[temp_index_threshold_type] =
			setting_millidegree_celsius;
		set_temp_threshold_user_settings(&temp_threshold_user_settings);
	}

	return true;
}
bool temp_threshold_default_settings_init(void)
{
	for (int i = 0; i < PLAT_TEMP_INDEX_THRESHOLD_TYPE_MAX; i++) {
		uint32_t temp_threshold = 0;
		if (!plat_get_temp_threshold(i, &temp_threshold)) {
			LOG_ERR("Can't find temp_threshold default by type index: %x", i);
			return false;
		}
		temp_threshold_default_settings.temperature_reg_val[i] = temp_threshold;
		uint32_t temperature = 0;
		// these temp_threshold is 100 degree
		if (i == ASIC_MEDHA0_SENSOR0_HIGH_LIMIT || i == ASIC_MEDHA0_SENSOR1_HIGH_LIMIT ||
		    i == ASIC_MEDHA1_SENSOR0_HIGH_LIMIT || i == ASIC_MEDHA1_SENSOR1_HIGH_LIMIT ||
		    i == ASIC_OWL_W_HIGH_LIMIT || i == ASIC_OWL_E_HIGH_LIMIT ||
		    i == ASIC_HAMSA_CRM_HIGH_LIMIT || i == ASIC_HAMSA_LS_HIGH_LIMIT) {
			temperature = 100000;
		}
		// set board temp threshold low to 0 degree
		if (i == TOP_INLET_LOW_LIMIT || i == BOT_INLET_LOW_LIMIT ||
		    i == BOT_OUTLET_LOW_LIMIT) {
			temperature = 75000;
		}
		// set board temp threshold high to 85 degree
		if (i == TOP_INLET_HIGH_LIMIT || i == BOT_INLET_HIGH_LIMIT ||
		    i == BOT_OUTLET_HIGH_LIMIT) {
			temperature = 85000;
		}

		if (!set_plat_temp_threshold(i, &temperature, false, false)) {
			LOG_ERR("Can't set temp threshold index: 0x%x to 100", i);
			return false;
		}
	}

	return true;
}
bool temp_threshold_user_settings_init(void)
{
	if (temp_threshold_user_settings_get(&temp_threshold_user_settings) == false) {
		LOG_ERR("get temp_threshold user settings fail");
		return false;
	}

	for (int i = 0; i < PLAT_TEMP_INDEX_THRESHOLD_TYPE_MAX; i++) {
		if (temp_threshold_user_settings.temperature_reg_val[i] != 0xffffffff) {
			/* TODO: write temp_threshold */
			uint32_t temp_threshold =
				temp_threshold_user_settings.temperature_reg_val[i];
			if (!plat_set_temp_threshold(i, &temp_threshold, false, false)) {
				LOG_ERR("Can't set temp_threshold[%x]=%x by temp_threshold user settings",
					i, temp_threshold);
				return false;
			}
			LOG_INF("set [%x]%s: %d", i,
				temp_index_threshold_type_table[i].temp_threshold_name,
				temp_threshold_user_settings.temperature_reg_val[i]);
		}
	}

	return true;
}
bool plat_get_temp_threshold(uint8_t temp_index_threshold_type, uint32_t *millidegree_celsius)
{
	CHECK_NULL_ARG_WITH_RETURN(millidegree_celsius, false);

	if (temp_index_threshold_type >= PLAT_TEMP_INDEX_THRESHOLD_TYPE_MAX) {
		LOG_ERR("Invalid temp threshold type(%x)", temp_index_threshold_type);
		return false;
	}

	uint8_t temp_threshold_type_tmp =
		temp_index_threshold_type_table[temp_index_threshold_type].temp_threshold_type;

	uint8_t sensor_id = temp_index_threshold_type_table[temp_index_threshold_type].sensor_id;
	sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);

	if (cfg == NULL) {
		LOG_ERR("Failed to get sensor config for sensor 0x%x", sensor_id);
		return false;
	}

	switch (cfg->type) {
	case sensor_dev_tmp431:
		if (!tmp432_get_temp_threshold(cfg, temp_threshold_type_tmp, millidegree_celsius)) {
			LOG_ERR("The TMP431 temp threshold reading failed");
			return false;
		}
		break;
	case sensor_dev_tmp75:
		if (!tmp75_get_temp_threshold(cfg, temp_threshold_type_tmp, millidegree_celsius)) {
			LOG_ERR("The TMP75 temp threshold reading failed");
			return false;
		}
		break;
	default:
		LOG_ERR("Unsupport temp type(%x)", cfg->type);
		return false;
	}

	return true;
}
void set_alert_level_to_default_or_user_setting(bool is_default, int32_t user_setting)
{
	k_mutex_lock(&pwrlevel_mutex, K_MSEC(1000));

	if (is_default == true) {
		alert_level_mA_user_setting = alert_level_mA_default;
	} else {
		alert_level_mA_user_setting = user_setting;
	}

	k_mutex_unlock(&pwrlevel_mutex);

	return;
}

void set_uart_power_event_is_enable(bool is_enable)
{
	if (is_enable == true) {
		uart_pwr_event_is_enable = true;
	} else {
		uart_pwr_event_is_enable = false;
	}

	return;
}

int power_level_send_event(bool is_assert, int ubc1_current, int ubc2_current)
{
	if (is_assert == true) {
		//To Do: need to send assert event to BMC
	} else {
		//To Do: need to send deassert event to BMC
	}

	if (uart_pwr_event_is_enable == true) {
		//print send event to consloe
		LOG_INF("send power level event ubc1_current=%dmA,ubc2_current=%dmA,alert_level_mA_user_setting=%dmA",
			ubc1_current, ubc2_current, alert_level_mA_user_setting);
	}

	return 0;
}

int set_user_settings_alert_level_to_eeprom(void *user_settings, uint8_t data_length)
{
	CHECK_NULL_ARG_WITH_RETURN(user_settings, -1);

	//bool plat_eeprom_write(uint32_t offset, uint8_t *data, uint16_t data_len)
	if (!plat_eeprom_write(ALERT_LEVEL_USER_SETTINGS_OFFSET, user_settings, data_length)) {
		LOG_ERR("Failed to write alert level to eeprom");
		return -1;
	}
	k_msleep(EEPROM_MAX_WRITE_TIME);

	return 0;
}

int get_alert_level_info(bool *is_assert, int32_t *default_value, int32_t *setting_value)
{
	CHECK_NULL_ARG_WITH_RETURN(is_assert, -1);
	CHECK_NULL_ARG_WITH_RETURN(default_value, -1);
	CHECK_NULL_ARG_WITH_RETURN(setting_value, -1);

	k_mutex_lock(&pwrlevel_mutex, K_MSEC(1000));

	*is_assert = alert_level_is_assert;
	*default_value = alert_level_mA_default;
	*setting_value = alert_level_mA_user_setting;

	k_mutex_unlock(&pwrlevel_mutex);

	return 0;
}
bool get_user_settings_throttle_from_eeprom(void *user_settings, uint8_t data_length)
{
	CHECK_NULL_ARG_WITH_RETURN(user_settings, false);

	if (!plat_eeprom_read(THROTTLE_USER_SETTINGS_OFFSET, user_settings, data_length)) {
		LOG_ERR("Failed to write thermaltrip to eeprom");
		return false;
	}

	LOG_HEXDUMP_DBG(user_settings, data_length, "EEPROM data read throttle");

	return true;
}

static bool throttle_user_settings_init(void)
{
	uint8_t setting_data = 0xFF;
	if (!get_user_settings_throttle_from_eeprom(&setting_data, sizeof(setting_data))) {
		LOG_ERR("get throttle user settings fail");
		return false;
	}

	if (setting_data != 0xFF) {
		if (!plat_write_cpld(CPLD_THROTTLE_SWITCH_ADDR, &setting_data)) {
			LOG_ERR("Can't set throttle=%d by user settings", setting_data);
			return false;
		}
		LOG_INF("set throttle=%x by user settings", setting_data);
	}

	return true;
}
int get_user_settings_alert_level_from_eeprom(void *user_settings, uint8_t data_length)
{
	CHECK_NULL_ARG_WITH_RETURN(user_settings, -1);

	if (!plat_eeprom_read(ALERT_LEVEL_USER_SETTINGS_OFFSET, user_settings, data_length)) {
		LOG_ERR("Failed to read alert level from eeprom");
		return -1;
	}

	return 0;
}
static int alert_level_user_settings_init(void)
{
	char setting_data[4] = { 0 };

	if (get_user_settings_alert_level_from_eeprom(setting_data, sizeof(setting_data)) == -1) {
		LOG_ERR("get alert level user settings failed");
		return -1;
	}

	int32_t alert_level_value = ((setting_data[3] << 24) | (setting_data[2] << 16) |
				     (setting_data[1] << 8) | setting_data[0]);

	if (alert_level_value != 0xffffffff) {
		alert_level_mA_user_setting = alert_level_value;
	} else {
		alert_level_mA_user_setting = alert_level_mA_default;
	}

	return 0;
}
static int delay_asic_rst_user_settings_init(void)
{
	uint8_t setting_value = 0;

	if (get_user_settings_delay_asic_rst_from_eeprom(&setting_value, sizeof(setting_value)) ==
	    false) {
		LOG_ERR("get alert level user settings failed");
		return -1;
	}

	if (setting_value != 0xff) {
		if (!plat_write_cpld(CPLD_OFFSET_ASIC_RST_DELAY, &setting_value)) {
			LOG_ERR("plat delay_asic_rst set failed");
			return -1;
		}
	}

	return 0;
}
static int delay_pcie_perst_user_settings_init(void)
{
	uint32_t setting_values[4] = { 0 };

	if (get_user_settings_delay_pcie_perst_from_eeprom(&setting_values,
							   sizeof(setting_values)) == false) {
		LOG_ERR("get delay pcie perst user settings failed");
		return -1;
	}
	uint8_t setting_value = 0;
	if (setting_values[0] != 0xffffffff) {
		setting_value = setting_values[0] & 0xff;
		if (!plat_write_cpld(CPLD_HAMSA_PCIE0_PERST_DELAY_REG, &setting_value)) {
			LOG_ERR("plat delay_pcie_perst PCIE0 set failed");
		}
	}
	if (setting_values[1] != 0xffffffff) {
		setting_value = setting_values[1] & 0xff;
		if (!plat_write_cpld(CPLD_HAMSA_PCIE1_PERST_DELAY_REG, &setting_value)) {
			LOG_ERR("plat delay_pcie_perst PCIE1 set failed");
		}
	}
	if (setting_values[2] != 0xffffffff) {
		setting_value = setting_values[2] & 0xff;
		if (!plat_write_cpld(CPLD_HAMSA_PCIE2_PERST_DELAY_REG, &setting_value)) {
			LOG_ERR("plat delay_pcie_perst PCIE2 set failed");
		}
	}
	if (setting_values[3] != 0xffffffff) {
		setting_value = setting_values[3] & 0xff;
		if (!plat_write_cpld(CPLD_HAMSA_PCIE3_PERST_DELAY_REG, &setting_value)) {
			LOG_ERR("plat delay_pcie_perst PCIE3 set failed");
		}
	}

	return 0;
}
static int delay_module_pg_user_settings_init(void)
{
	uint8_t setting_value = 0;

	if (get_user_settings_delay_module_pg_from_eeprom(&setting_value, sizeof(setting_value)) ==
	    false) {
		LOG_ERR("get alert level user settings failed");
		return -1;
	}

	if (setting_value != 0xff) {
		if (!plat_write_cpld(CPLD_OFFSET_MODULE_PG_DELAY, &setting_value)) {
			LOG_ERR("plat delay_module_pg set failed");
			return -1;
		}
	}

	return 0;
}
bool get_user_settings_delay_pcie_perst_from_eeprom(void *user_settings, uint8_t data_length)
{
	CHECK_NULL_ARG_WITH_RETURN(user_settings, false);

	if (!plat_eeprom_read(DELAY_PCIE_PERST_USER_SETTINGS_OFFSET, user_settings, data_length)) {
		LOG_ERR("Failed to read delay_pcie_perst from eeprom");
		return false;
	}
	return true;
}
bool set_user_settings_delay_pcie_perst_to_eeprom(void *user_settings, uint8_t data_length,
						  uint8_t user_settings_offset)
{
	CHECK_NULL_ARG_WITH_RETURN(user_settings, false);

	if (!plat_eeprom_write(DELAY_PCIE_PERST_USER_SETTINGS_OFFSET + user_settings_offset,
			       user_settings, data_length)) {
		LOG_ERR("delay_pcie_perst Failed to write eeprom");
		return false;
	}
	k_msleep(EEPROM_MAX_WRITE_TIME);

	return true;
}
bool get_user_settings_delay_asic_rst_from_eeprom(void *user_settings, uint8_t data_length)
{
	CHECK_NULL_ARG_WITH_RETURN(user_settings, false);

	if (!plat_eeprom_read(DELAY_ASIC_RST_USER_SETTINGS_OFFSET, user_settings, data_length)) {
		LOG_ERR("Failed to read delay_asic_rst from eeprom");
		return false;
	}
	return true;
}
bool set_user_settings_delay_asic_rst_to_eeprom(void *user_settings, uint8_t data_length)
{
	CHECK_NULL_ARG_WITH_RETURN(user_settings, false);

	if (!plat_eeprom_write(DELAY_ASIC_RST_USER_SETTINGS_OFFSET, user_settings, data_length)) {
		LOG_ERR("delay_asic_rst Failed to write eeprom");
		return false;
	}
	k_msleep(EEPROM_MAX_WRITE_TIME);

	return true;
}
bool get_user_settings_delay_module_pg_from_eeprom(void *user_settings, uint8_t data_length)
{
	CHECK_NULL_ARG_WITH_RETURN(user_settings, false);

	if (!plat_eeprom_read(DELAY_MODULE_PG_USER_SETTINGS_OFFSET, user_settings, data_length)) {
		LOG_ERR("Failed to read delay_module_pg from eeprom");
		return false;
	}
	return true;
}
bool set_user_settings_delay_module_pg_to_eeprom(void *user_settings, uint8_t data_length)
{
	CHECK_NULL_ARG_WITH_RETURN(user_settings, false);

	if (!plat_eeprom_write(DELAY_MODULE_PG_USER_SETTINGS_OFFSET, user_settings, data_length)) {
		LOG_ERR("delay_module_pg Failed to write eeprom");
		return false;
	}
	k_msleep(EEPROM_MAX_WRITE_TIME);

	return true;
}
bool perm_config_clear(void)
{
	/* clear all vout perm parameters */
	memset(user_settings.vout, 0xFF, sizeof(user_settings.vout));
	if (!vr_vout_user_settings_set(&user_settings)) {
		LOG_ERR("The perm_config clear failed");
		return false;
	}

	/* clear all temp_threshold perm parameters */
	memset(temp_threshold_user_settings.temperature_reg_val, 0xFF,
	       sizeof(temp_threshold_user_settings.temperature_reg_val));
	if (!set_temp_threshold_user_settings(&temp_threshold_user_settings)) {
		LOG_ERR("The perm_config clear failed");
		return false;
	}

	int32_t setting_value = 0xffffffff;
	char setting_data[4] = { 0 };
	memcpy(setting_data, &setting_value, sizeof(setting_data));
	if (set_user_settings_alert_level_to_eeprom(setting_data, sizeof(setting_data)) != 0) {
		LOG_ERR("The perm_config clear failed");
		return false;
	}

	/* clear soc_pcie_perst perm parameter */
	uint32_t setting_value_for_delay_pcie_perst[4] = { 0 };
	memset(setting_value_for_delay_pcie_perst, 0xffffffff,
	       sizeof(setting_value_for_delay_pcie_perst));
	if (!set_user_settings_delay_pcie_perst_to_eeprom(
		    &setting_value_for_delay_pcie_perst[0],
		    sizeof(setting_value_for_delay_pcie_perst), 0)) {
		LOG_ERR("The perm_config delay pcie perst clear failed");
		return false;
	}

	/* clear all bootstrap perm parameters */
	memset(bootstrap_user_settings.user_setting_value, 0xFF,
	       sizeof(bootstrap_user_settings.user_setting_value));
	if (!bootstrap_user_settings_set(&bootstrap_user_settings)) {
		LOG_ERR("The perm_config clear failed");
		return false;
	}

	/* clear thermaltrip perm parameter */
	uint8_t setting_value_for_thermaltrip = 0xFF;
	if (!set_user_settings_thermaltrip_to_eeprom(&setting_value_for_thermaltrip,
						     sizeof(setting_value_for_thermaltrip))) {
		LOG_ERR("The perm_config clear failed");
		return false;
	}

	/* clear throttle perm parameter */
	uint8_t setting_value_for_throttle = 0xFF;
	if (!set_user_settings_throttle_to_eeprom(&setting_value_for_throttle,
						  sizeof(setting_value_for_throttle))) {
		LOG_ERR("The perm_config clear failed");
		return false;
	}

	/* TODO TODO wait power capping function add
		// clear power capping perm parameter
		memset(power_capping_user_settings.user_setting_value, 0xFF,
			sizeof(power_capping_user_settings.user_setting_value));
		if (!set_user_settings_power_capping_to_eeprom(&power_capping_user_settings)) {
			LOG_ERR("The perm_config clear failed");
			return false;
		}
	*/

	return true;
}
ubc_vr_power_mapping_sensor ubc_vr_power_table[] = {
	{ UBC_VR_RAIL_E_UBC1, SENSOR_NUM_UBC1_P12V_PWR_W, "UBC1_P12V_PWR_W", { 0 } },
	{ UBC_VR_RAIL_E_UBC2, SENSOR_NUM_UBC2_P12V_PWR_W, "UBC2_P12V_PWR_W", { 0 } },
	{ UBC_VR_RAIL_E_ASIC_P0V85_MEDHA0_VDD,
	  SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_PWR_W,
	  "VR_ASIC_P0V85_MEDHA0_VDD_PWR_W",
	  { 0 } },
	{ UBC_VR_RAIL_E_ASIC_P0V85_MEDHA1_VDD,
	  SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_PWR_W,
	  "VR_ASIC_P0V85_MEDHA1_VDD_PWR_W",
	  { 0 } },
	{ UBC_VR_RAIL_E_ASIC_P0V9_OWL_E_TRVDD,
	  SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_PWR_W,
	  "VR_ASIC_P0V9_OWL_E_TRVDD_PWR_W",
	  { 0 } },
	{ UBC_VR_RAIL_E_ASIC_P0V75_OWL_E_TRVDD,
	  SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_PWR_W,
	  "VR_ASIC_P0V75_OWL_E_TRVDD_PWR_W",
	  { 0 } },
	{ UBC_VR_RAIL_E_ASIC_P0V75_MAX_M_VDD,
	  SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_PWR_W,
	  "VR_ASIC_P0V75_MAX_M_VDD_PWR_W",
	  { 0 } },
	{ UBC_VR_RAIL_E_ASIC_P0V75_VDDPHY_HBM1357,
	  SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_PWR_W,
	  "VR_ASIC_P0V75_VDDPHY_HBM1357_PWR_W",
	  { 0 } },
	{ UBC_VR_RAIL_E_ASIC_P0V75_OWL_E_VDD,
	  SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_PWR_W,
	  "VR_ASIC_P0V75_OWL_E_VDD_PWR_W",
	  { 0 } },
	{ UBC_VR_RAIL_E_ASIC_P0V4_VDDQL_HBM1357,
	  SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_PWR_W,
	  "VR_ASIC_P0V4_VDDQL_HBM1357_PWR_W",
	  { 0 } },
	{ UBC_VR_RAIL_E_ASIC_P1V1_VDDQC_HBM1357,
	  SENSOR_NUM_ASIC_P1V1_VDDQC_HBM1357_PWR_W,
	  "VR_ASIC_P1V1_VDDQC_HBM1357_PWR_W",
	  { 0 } },
	{ UBC_VR_RAIL_E_ASIC_P1V8_VPP_HBM1357,
	  SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_PWR_W,
	  "VR_ASIC_P1V8_VPP_HBM1357_PWR_W",
	  { 0 } },
	{ UBC_VR_RAIL_E_ASIC_P0V75_MAX_N_VDD,
	  SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_PWR_W,
	  "VR_ASIC_P0V75_MAX_N_VDD_PWR_W",
	  { 0 } },
	{ UBC_VR_RAIL_E_ASIC_P0V8_HAMSA_AVDD_PCIE,
	  SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_PWR_W,
	  "VR_ASIC_P0V8_HAMSA_AVDD_PCIE_PWR_W",
	  { 0 } },
	{ UBC_VR_RAIL_E_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE,
	  SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_PWR_W,
	  "VR_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_PWR_W",
	  { 0 } },
	{ UBC_VR_RAIL_E_ASIC_P0V85_HAMSA_VDD,
	  SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_PWR_W,
	  "VR_ASIC_P0V85_HAMSA_VDD_PWR_W",
	  { 0 } },
	{ UBC_VR_RAIL_E_ASIC_P1V1_VDDQC_HBM0246,
	  SENSOR_NUM_ASIC_P1V1_VDDQC_HBM0246_PWR_W,
	  "VR_ASIC_P1V1_VDDQC_HBM0246_PWR_W",
	  { 0 } },
	{ UBC_VR_RAIL_E_ASIC_P1V8_VPP_HBM0246,
	  SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_PWR_W,
	  "VR_ASIC_P1V8_VPP_HBM0246_PWR_W",
	  { 0 } },
	{ UBC_VR_RAIL_E_ASIC_P0V4_VDDQL_HBM0246,
	  SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_PWR_W,
	  "VR_ASIC_P0V4_VDDQL_HBM0246_PWR_W",
	  { 0 } },
	{ UBC_VR_RAIL_E_ASIC_P0V75_VDDPHY_HBM0246,
	  SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_PWR_W,
	  "VR_ASIC_P0V75_VDDPHY_HBM0246_PWR_W",
	  { 0 } },
	{ UBC_VR_RAIL_E_ASIC_P0V75_OWL_W_VDD,
	  SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_PWR_W,
	  "VR_ASIC_P0V75_OWL_W_VDD_PWR_W",
	  { 0 } },
	{ UBC_VR_RAIL_E_ASIC_P0V75_MAX_S_VDD,
	  SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_PWR_W,
	  "VR_ASIC_P0V75_MAX_S_VDD_PWR_W",
	  { 0 } },
	{ UBC_VR_RAIL_E_ASIC_P0V9_OWL_W_TRVDD,
	  SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_PWR_W,
	  "VR_ASIC_P0V9_OWL_W_TRVDD_PWR_W",
	  { 0 } },
	{ UBC_VR_RAIL_E_ASIC_P0V75_OWL_W_TRVDD,
	  SENSOR_NUM_ASIC_P0V75_OWL_W_TRVDD_PWR_W,
	  "VR_ASIC_P0V75_OWL_W_TRVDD_PWR_W",
	  { 0 } },
	{ UBC_VR_RAIL_E_P3V3_OSFP, SENSOR_NUM_P3V3_OSFP_PWR_W, "VR_RAIL_E_P3V3_OSFP", { 0 } },
};

bool ubc_vr_rail_name_get(uint8_t rail, uint8_t **name)
{
	CHECK_NULL_ARG_WITH_RETURN(name, false);

	if (rail >= UBC_VR_RAIL_E_MAX) {
		*name = NULL;
		return false;
	}

	*name = (uint8_t *)ubc_vr_power_table[rail].sensor_name;
	return true;
}
bool ubc_vr_rail_enum_get(uint8_t *name, uint8_t *num)
{
	CHECK_NULL_ARG_WITH_RETURN(name, false);
	CHECK_NULL_ARG_WITH_RETURN(num, false);

	for (int i = 0; i < UBC_VR_RAIL_E_MAX; i++) {
		if (strcmp(name, ubc_vr_power_table[i].sensor_name) == 0) {
			*num = i;
			return true;
		}
	}

	LOG_ERR("invalid rail name %s", name);
	return false;
}

bool post_ubc_read(sensor_cfg *cfg, void *args, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	ARG_UNUSED(args);

	// post reading value update
	uint8_t sensor_number = cfg->num;
	if (!post_sensor_reading_hook_func(sensor_number))
		LOG_ERR("pldm update sensor value failed");

	const sensor_val *reading_val = (sensor_val *)reading;

	int sensor_reading = 0;
	static int ubc1_current_mA;

	if (cfg->num == SENSOR_NUM_UBC1_P12V_CURR_A) {
		if (reading != NULL) {
			if (reading_val->integer >= 0) {
				sensor_reading =
					reading_val->integer * 1000 + reading_val->fraction;
			} else {
				sensor_reading =
					reading_val->integer * 1000 - reading_val->fraction;
			}

			ubc1_current_mA = sensor_reading;
		} else {
			ubc1_current_mA = 0;
		}
	}

	if (cfg->num == SENSOR_NUM_UBC2_P12V_CURR_A) {
		if (reading != NULL) {
			if (reading_val->integer >= 0) {
				sensor_reading =
					reading_val->integer * 1000 + reading_val->fraction;
			} else {
				sensor_reading =
					reading_val->integer * 1000 - reading_val->fraction;
			}
		} else {
			sensor_reading = 0;
		}

		k_mutex_lock(&pwrlevel_mutex, K_MSEC(1000));

		if ((ubc1_current_mA + sensor_reading) > alert_level_mA_user_setting) {
			if (alert_level_is_assert == false) {
				alert_level_is_assert = true;
				power_level_send_event(alert_level_is_assert, ubc1_current_mA,
						       sensor_reading);
			}
		} else {
			if (alert_level_is_assert == true) {
				alert_level_is_assert = false;
				power_level_send_event(alert_level_is_assert, ubc1_current_mA,
						       sensor_reading);
			}
		}

		k_mutex_unlock(&pwrlevel_mutex);
	}

	/* set reading val to 0 if reading val is negative */
	if (reading != NULL) {
		float resolution = 0, offset = 0;
		int cache_reading = 0;
		int8_t unit_modifier = 0;
		uint8_t sensor_operational_state = PLDM_SENSOR_STATUSUNKOWN;
		pldm_sensor_get_info_via_sensor_id(cfg->num, &resolution, &offset, &unit_modifier,
						   &cache_reading, &sensor_operational_state);
		if (resolution == 0)
			LOG_ERR("resolution is 0");

		int16_t integer = *reading & 0xFFFF;
		float fraction = (float)(*reading >> 16) / 1000.0;

		if (integer < 0 && fraction > 0)
			fraction = -fraction;

		float tmp_reading = (float)integer + fraction;

		if (tmp_reading < 0) {
			tmp_reading = 0;
			*reading = 0;
			LOG_DBG("Original sensor reading: integer = %d, fraction = %f", integer,
				fraction);
			LOG_DBG("Negative sensor reading detected. Set reading to 0x%x", *reading);
		}

		int decoded_reading =
			(int)((tmp_reading * power(10, -1 * unit_modifier) - offset) / resolution);

		/* record power history */
		for (int i = 0; i < UBC_VR_RAIL_E_MAX; i++) {
			if (cfg->num == ubc_vr_power_table[i].sensor_id) {
				ubc_vr_power_table[i].power_history[power_index[i]] =
					decoded_reading;
				power_index[i] = (power_index[i] + 1) % POWER_HISTORY_SIZE;
				if (power_count[i] < POWER_HISTORY_SIZE) {
					power_count[i]++;
				}
			}
		}
	}

	return true;
}
bool post_vr_read(sensor_cfg *cfg, void *args, int *const reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);
	for (int i = 0; i < VR_RAIL_E_MAX; i++) {
		if (reading == NULL) {
			break;
		}

		if (((get_asic_board_id() != ASIC_BOARD_ID_EVB)) &&
		    (i == VR_RAIL_E_P3V3_OSFP_VOLT_V))
			continue; // skip osfp p3v3 on AEGIS BD

		if (cfg->num == vr_rail_table[i].sensor_id) {
			if (vr_rail_table[i].peak_value == 0xffffffff) {
				vr_rail_table[i].peak_value = *reading;
			} else {
				if (vr_rail_table[i].peak_value < *reading) {
					vr_rail_table[i].peak_value = *reading;
				}
			}
			break;
		}
	}

	vr_pre_proc_arg *pre_proc_args = (vr_pre_proc_arg *)args;

	/* mutex unlock */
	if (pre_proc_args->mutex) {
		LOG_DBG("%x u %p", cfg->num, pre_proc_args->mutex);
		if (k_mutex_unlock(pre_proc_args->mutex)) {
			LOG_ERR("0x%02x post_vr_read, mutex unlock fail", cfg->num);
			return false;
		}
	}

	/* set reading val to 0 if reading val is negative */
	sensor_val tmp_reading;
	tmp_reading.integer = (int16_t)(*reading & 0xFFFF);
	tmp_reading.fraction = (int16_t)((*reading >> 16) & 0xFFFF);

	/* sensor_value = 1000 times of true value */
	int32_t sensor_value = tmp_reading.integer * 1000 + tmp_reading.fraction;

	if (sensor_value < 0) {
		LOG_DBG("Original sensor reading: integer = %d, fraction = %d (combined value * 1000: %d)",
			tmp_reading.integer, tmp_reading.fraction, sensor_value);
		*reading = 0;
		LOG_DBG("Negative sensor reading detected. Set reading to 0x%x", *reading);
	}
	post_sensor_reading_hook_func(cfg->num);

	if (reading != NULL) {
		float resolution = 0, offset = 0;
		int cache_reading = 0;
		int8_t unit_modifier = 0;
		uint8_t sensor_operational_state = PLDM_SENSOR_STATUSUNKOWN;
		pldm_sensor_get_info_via_sensor_id(cfg->num, &resolution, &offset, &unit_modifier,
						   &cache_reading, &sensor_operational_state);
		if (resolution == 0)
			LOG_ERR("resolution is 0");

		int16_t integer = *reading & 0xFFFF;
		float fraction = (float)(*reading >> 16) / 1000.0;

		if (integer < 0 && fraction > 0)
			fraction = -fraction;

		float tmp_reading = (float)integer + fraction;

		if (tmp_reading < 0) {
			tmp_reading = 0;
			*reading = 0;
			LOG_DBG("Original sensor reading: integer = %d, fraction = %f", integer,
				fraction);
			LOG_DBG("Negative sensor reading detected. Set reading to 0x%x", *reading);
		}

		int decoded_reading =
			(int)((tmp_reading * power(10, -1 * unit_modifier) - offset) / resolution);

		/* record power history */
		for (int i = 0; i < UBC_VR_RAIL_E_MAX; i++) {
			if ((get_asic_board_id() != ASIC_BOARD_ID_EVB) &&
			    (i == UBC_VR_RAIL_E_P3V3_OSFP))
				continue; // skip osfp p3v3
			if (cfg->num == ubc_vr_power_table[i].sensor_id) {
				ubc_vr_power_table[i].power_history[power_index[i]] =
					decoded_reading;
				power_index[i] = (power_index[i] + 1) % POWER_HISTORY_SIZE;
				if (power_count[i] < POWER_HISTORY_SIZE) {
					power_count[i]++;
				}
			}
		}

		/* TO_DO wait power capping add
		if (cfg->num == VR_ASIC_P0V85_PVDD_PWR_W) {
			update_plat_power_capping_table();
			ath_vdd_power = (int)tmp_reading;
			ath_vdd_polling_counter++;
			// LOG_INF("counter:%d/%d", ath_vdd_polling_counter, comparator_counter_max);
			if (ath_vdd_polling_counter >= comparator_counter_max) {
				ath_vdd_polling_counter = 0;
				power_capping_comparator_handler();
			}
		}
		*/
	}

	return true;
}
bool get_average_power(uint8_t rail, uint32_t *milliwatt)
{
	CHECK_NULL_ARG_WITH_RETURN(milliwatt, false);

	if (rail >= UBC_VR_RAIL_E_MAX || power_count[rail] == 0) {
		*milliwatt = 0;
		return false;
	}

	int sum = 0;
	for (int i = 0; i < power_count[rail]; i++) {
		sum += ubc_vr_power_table[rail].power_history[i];
	}

	float avg_sensor_value = sum / (float)power_count[rail];
	if (avg_sensor_value < 0) {
		LOG_ERR("avg_sensor_value is negative: %f", avg_sensor_value);
		*milliwatt = 0;
		return false;
	}

	uint8_t sensor_id = ubc_vr_power_table[rail].sensor_id;
	float resolution = 0, offset = 0;
	int cache_reading = 0;
	int8_t unit_modifier = 0;
	uint8_t sensor_operational_state = PLDM_SENSOR_STATUSUNKOWN;
	pldm_sensor_get_info_via_sensor_id(sensor_id, &resolution, &offset, &unit_modifier,
					   &cache_reading, &sensor_operational_state);
	if (resolution == 0) {
		*milliwatt = 0;
		LOG_ERR("resolution is 0");
		return false;
	}

	float real_power = (avg_sensor_value * resolution + offset) / power(10, -unit_modifier);

	int16_t integer_part = (int16_t)real_power;
	int16_t fraction_part = (int16_t)((real_power - integer_part) * 1000.0);

	if (integer_part < 0 && fraction_part > 0) {
		fraction_part = -fraction_part;
	}

	*milliwatt = ((uint16_t)fraction_part << 16) | (uint16_t)integer_part;

	LOG_DBG("real_power = %f, integer_part = %d, fraction_part = %d, milliwatt = 0x%x",
		real_power, integer_part, fraction_part, *milliwatt);

	return true;
}
void user_settings_init(void)
{
	vr_vout_default_settings_init();
	vr_vout_user_settings_init();
	bootstrap_default_settings_init();
	bootstrap_user_settings_init();
	vr_vout_range_user_settings_init();
	thermaltrip_user_settings_init();
	throttle_user_settings_init();
	alert_level_user_settings_init();
	delay_asic_rst_user_settings_init();
	delay_module_pg_user_settings_init();
	delay_pcie_perst_user_settings_init();
}
