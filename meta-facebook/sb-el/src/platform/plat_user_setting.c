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

#include "plat_hook.h"
#include "plat_fru.h"
#include "sensor.h"
#include "tmp431.h"
#include "emc1413.h"
#include "tmp75.h"
#include "plat_user_setting.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(plat_user_setting);

#define EEPROM_MAX_WRITE_TIME 5

temp_threshold_user_settings_struct temp_threshold_user_settings = { 0 };
struct temp_threshold_user_settings_struct temp_threshold_default_settings = { 0 };

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
	{ ASIC_NUWA0_SENSOR0_LOW_LIMIT, REMOTE_1_LOW_LIMIT, SENSOR_NUM_ASIC_NUWA0_SENSOR0_TEMP_C,
	  "SB_RB_ASIC_NUWA0_SENSOR0_TEMP_LOW_LIM" },
	{ ASIC_NUWA0_SENSOR0_HIGH_LIMIT, REMOTE_1_HIGH_LIMIT,
	  SENSOR_NUM_ASIC_NUWA0_SENSOR0_TEMP_C, "SB_RB_ASIC_NUWA0_SENSOR0_TEMP_HIGH_LIM" },
	{ ASIC_NUWA0_SENSOR1_LOW_LIMIT, REMOTE_2_LOW_LIMIT, SENSOR_NUM_ASIC_NUWA0_SENSOR1_TEMP_C,
	  "SB_RB_ASIC_NUWA0_SENSOR1_TEMP_LOW_LIM" },
	{ ASIC_NUWA0_SENSOR1_HIGH_LIMIT, REMOTE_2_HIGH_LIMIT,
	  SENSOR_NUM_ASIC_NUWA0_SENSOR1_TEMP_C, "SB_RB_ASIC_NUWA0_SENSOR1_TEMP_HIGH_LIM" },
	{ ASIC_OWL_W_LOW_LIMIT, REMOTE_1_LOW_LIMIT, SENSOR_NUM_ASIC_OWL_W_TEMP_C,
	  "SB_RB_ASIC_OWL_W_TEMP_LOW_LIM" },
	{ ASIC_OWL_W_HIGH_LIMIT, REMOTE_1_HIGH_LIMIT, SENSOR_NUM_ASIC_OWL_W_TEMP_C,
	  "SB_RB_ASIC_OWL_W_TEMP_HIGH_LIM" },
	{ ASIC_OWL_E_LOW_LIMIT, REMOTE_2_LOW_LIMIT, SENSOR_NUM_ASIC_OWL_E_TEMP_C,
	  "SB_RB_ASIC_OWL_E_TEMP_LOW_LIM" },
	{ ASIC_OWL_E_HIGH_LIMIT, REMOTE_2_HIGH_LIMIT, SENSOR_NUM_ASIC_OWL_E_TEMP_C,
	  "SB_RB_ASIC_OWL_E_TEMP_HIGH_LIM" },
	{ ASIC_NUWA1_SENSOR0_LOW_LIMIT, REMOTE_1_LOW_LIMIT, SENSOR_NUM_ASIC_NUWA1_SENSOR0_TEMP_C,
	  "SB_RB_ASIC_NUWA1_SENSOR0_TEMP_LOW_LIM" },
	{ ASIC_NUWA1_SENSOR0_HIGH_LIMIT, REMOTE_1_HIGH_LIMIT,
	  SENSOR_NUM_ASIC_NUWA1_SENSOR0_TEMP_C, "SB_RB_ASIC_NUWA1_SENSOR0_TEMP_HIGH_LIM" },
	{ ASIC_NUWA1_SENSOR1_LOW_LIMIT, REMOTE_2_LOW_LIMIT, SENSOR_NUM_ASIC_NUWA1_SENSOR1_TEMP_C,
	  "SB_RB_ASIC_NUWA1_SENSOR1_TEMP_LOW_LIM" },
	{ ASIC_NUWA1_SENSOR1_HIGH_LIMIT, REMOTE_2_HIGH_LIMIT,
	  SENSOR_NUM_ASIC_NUWA1_SENSOR1_TEMP_C, "SB_RB_ASIC_NUWA1_SENSOR1_TEMP_HIGH_LIM" },
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
		if (i == ASIC_NUWA0_SENSOR0_HIGH_LIMIT || i == ASIC_NUWA0_SENSOR1_HIGH_LIMIT ||
		    i == ASIC_NUWA1_SENSOR0_HIGH_LIMIT || i == ASIC_NUWA1_SENSOR1_HIGH_LIMIT ||
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

		if (!plat_set_temp_threshold(i, &temperature, false, false)) {
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
	case sensor_dev_emc1413:
		if (!emc1413_get_temp_threshold(cfg, temp_threshold_type_tmp, millidegree_celsius)) {
			LOG_ERR("The EMC1413 temp threshold reading failed");
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
	case sensor_dev_emc1413:
		if (!emc1413_set_temp_threshold(cfg, temp_threshold_type_tmp, millidegree_celsius)) {
			LOG_ERR("The EMC1413 temp threshold setting failed");
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

bool perm_config_clear(void)
{
	/* clear all temp_threshold perm parameters */
	memset(temp_threshold_user_settings.temperature_reg_val, 0xFF,
	       sizeof(temp_threshold_user_settings.temperature_reg_val));
	if (!set_temp_threshold_user_settings(&temp_threshold_user_settings)) {
		LOG_ERR("The perm_config clear failed");
		return false;
	}

	return true;
}

void user_settings_init(void)
{
	vr_vout_range_user_settings_init();
	bootstrap_default_settings_init();
	bootstrap_user_settings_init();
}