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

LOG_MODULE_REGISTER(plat_user_setting);

#define EEPROM_MAX_WRITE_TIME 5 // the BR24G512 eeprom max write time is 3.5 ms
#define TMP75_ALERT_CPLD_OFFSET 0x2F

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
		if (!plat_write_cpld(CPLD_THERMALTRIP_SWITCH_ADDR, &setting_data)) 
		{
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

bool get_plat_temp_threshold(uint8_t temp_index_threshold_type, uint32_t *millidegree_celsius)
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

void user_settings_init(void)
{
	//alert_level_user_settings_init();
	vr_vout_default_settings_init();
	vr_vout_user_settings_init();
	//soc_pcie_perst_user_settings_init();
	//bootstrap_default_settings_init();
	//bootstrap_user_settings_init();
	vr_vout_range_user_settings_init();
	thermaltrip_user_settings_init();
	//throttle_user_settings_init();
}