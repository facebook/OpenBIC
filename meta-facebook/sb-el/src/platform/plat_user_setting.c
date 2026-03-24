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
#include "plat_i2c.h"
#include "plat_hook.h"
#include "plat_fru.h"
#include "sensor.h"
#include "tmp431.h"
#include "emc1413.h"
#include "tmp75.h"
#include "plat_user_setting.h"
#include "plat_kernel_obj.h"

LOG_MODULE_REGISTER(plat_user_setting);

#define EEPROM_MAX_WRITE_TIME 5 // the BR24G512 eeprom max write time is 3.5 ms

temp_threshold_user_settings_struct temp_threshold_user_settings = { 0 };
struct temp_threshold_user_settings_struct temp_threshold_default_settings = { 0 };
thermaltrip_user_settings_struct thermaltrip_user_settings = { 0xFF };
throttle_user_settings_struct throttle_user_settings = { 0xFF };

// pwrlevel
// bool alert_level_is_assert = false;
static bool uart_pwr_event_is_enable = true;
int32_t alert_level_mA_default = 180000;
int32_t alert_level_mA_user_setting = 180000;

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
	{ ASIC_NUWA0_SENSOR0_HIGH_LIMIT, REMOTE_1_HIGH_LIMIT, SENSOR_NUM_ASIC_NUWA0_SENSOR0_TEMP_C,
	  "SB_RB_ASIC_NUWA0_SENSOR0_TEMP_HIGH_LIM" },
	{ ASIC_NUWA0_SENSOR1_LOW_LIMIT, REMOTE_2_LOW_LIMIT, SENSOR_NUM_ASIC_NUWA0_SENSOR1_TEMP_C,
	  "SB_RB_ASIC_NUWA0_SENSOR1_TEMP_LOW_LIM" },
	{ ASIC_NUWA0_SENSOR1_HIGH_LIMIT, REMOTE_2_HIGH_LIMIT, SENSOR_NUM_ASIC_NUWA0_SENSOR1_TEMP_C,
	  "SB_RB_ASIC_NUWA0_SENSOR1_TEMP_HIGH_LIM" },
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
	{ ASIC_NUWA1_SENSOR0_HIGH_LIMIT, REMOTE_1_HIGH_LIMIT, SENSOR_NUM_ASIC_NUWA1_SENSOR0_TEMP_C,
	  "SB_RB_ASIC_NUWA1_SENSOR0_TEMP_HIGH_LIM" },
	{ ASIC_NUWA1_SENSOR1_LOW_LIMIT, REMOTE_2_LOW_LIMIT, SENSOR_NUM_ASIC_NUWA1_SENSOR1_TEMP_C,
	  "SB_RB_ASIC_NUWA1_SENSOR1_TEMP_LOW_LIM" },
	{ ASIC_NUWA1_SENSOR1_HIGH_LIMIT, REMOTE_2_HIGH_LIMIT, SENSOR_NUM_ASIC_NUWA1_SENSOR1_TEMP_C,
	  "SB_RB_ASIC_NUWA1_SENSOR1_TEMP_HIGH_LIM" },
	{ ASIC_HAMSA_CRM_LOW_LIMIT, REMOTE_1_LOW_LIMIT, SENSOR_NUM_ASIC_HAMSA_CRM_TEMP_C,
	  "SB_RB_ASIC_HAMSA_CRM_TEMP_LOW_LIM" },
	{ ASIC_HAMSA_CRM_HIGH_LIMIT, REMOTE_1_HIGH_LIMIT, SENSOR_NUM_ASIC_HAMSA_CRM_TEMP_C,
	  "SB_RB_ASIC_HAMSA_CRM_TEMP_HIGH_LIM" },
	{ ASIC_HAMSA_LS_LOW_LIMIT, REMOTE_2_LOW_LIMIT, SENSOR_NUM_ASIC_HAMSA_LS_TEMP_C,
	  "SB_RB_ASIC_HAMSA_LS_TEMP_LOW_LIM" },
	{ ASIC_HAMSA_LS_HIGH_LIMIT, REMOTE_2_HIGH_LIMIT, SENSOR_NUM_ASIC_HAMSA_LS_TEMP_C,
	  "SB_RB_ASIC_HAMSA_LS_TEMP_HIGH_LIM" },
};

// temp
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
		if (!emc1413_get_temp_threshold(cfg, temp_threshold_type_tmp,
						millidegree_celsius)) {
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
		if (!emc1413_set_temp_threshold(cfg, temp_threshold_type_tmp,
						millidegree_celsius)) {
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

// pwrlevel
int32_t plat_get_alert_level_mA_user_setting(void)
{
	return alert_level_mA_user_setting;
}

void set_alert_level_to_default_or_user_setting(bool is_default, int32_t user_setting)
{
	if (pwr_level_mutex_lock(K_MSEC(1000)) != 0) {
		LOG_ERR("failed to lock pwrlevel mutex");
		return;
	}

	if (is_default == true) {
		alert_level_mA_user_setting = alert_level_mA_default;
	} else {
		alert_level_mA_user_setting = user_setting;
	}

	pwr_level_mutex_unlock();

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

int get_alert_level_info(bool *is_assert, int32_t *default_value, int32_t *setting_value)
{
	CHECK_NULL_ARG_WITH_RETURN(is_assert, -1);
	CHECK_NULL_ARG_WITH_RETURN(default_value, -1);
	CHECK_NULL_ARG_WITH_RETURN(setting_value, -1);

	if (pwr_level_mutex_lock(K_MSEC(1000)) != 0) {
		LOG_ERR("failed to lock pwrlevel mutex");
		return -1;
	}

	*is_assert = plat_get_alert_level_is_assert();
	*default_value = alert_level_mA_default;
	*setting_value = alert_level_mA_user_setting;

	pwr_level_mutex_unlock();

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

// thermaltrip
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

// throttle
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

bool set_user_settings_throttle_to_eeprom(void *throttle_user_settings, uint8_t data_length)
{
	CHECK_NULL_ARG_WITH_RETURN(throttle_user_settings, false);

	/* write the throttle_user_settings to eeprom */

	if (!plat_eeprom_write(THROTTLE_USER_SETTINGS_OFFSET, (uint8_t *)throttle_user_settings,
			       data_length)) {
		LOG_ERR("throttle user settings failed to write into eeprom");
		return false;
	}

	k_msleep(EEPROM_MAX_WRITE_TIME);

	return true;
}

bool set_throttle_user_settings(uint8_t *throttle_status_reg, bool is_perm)
{
	CHECK_NULL_ARG_WITH_RETURN(throttle_status_reg, false);

	if (!plat_write_cpld(CPLD_THROTTLE_SWITCH_ADDR, throttle_status_reg)) {
		LOG_ERR("Failed to write throttle to cpld error");
		return false;
	}

	if (is_perm) {
		throttle_user_settings.throttle_user_setting_value = *throttle_status_reg;

		if (!set_user_settings_throttle_to_eeprom(&throttle_user_settings,
							  sizeof(throttle_user_settings))) {
			LOG_ERR("Failed to write throttle to eeprom error");
			return false;
		}
	}
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

// delay pcie perst
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
		if (!plat_write_cpld(CPLD_PERST_DELAY_0_REG, &setting_value)) {
			LOG_ERR("plat delay_pcie_perst PCIE0 set failed");
		}
	}
	if (setting_values[1] != 0xffffffff) {
		setting_value = setting_values[1] & 0xff;
		if (!plat_write_cpld(CPLD_PERST_DELAY_1_REG, &setting_value)) {
			LOG_ERR("plat delay_pcie_perst PCIE1 set failed");
		}
	}
	if (setting_values[2] != 0xffffffff) {
		setting_value = setting_values[2] & 0xff;
		if (!plat_write_cpld(CPLD_PERST_DELAY_2_REG, &setting_value)) {
			LOG_ERR("plat delay_pcie_perst PCIE2 set failed");
		}
	}
	if (setting_values[3] != 0xffffffff) {
		setting_value = setting_values[3] & 0xff;
		if (!plat_write_cpld(CPLD_PERST_DELAY_3_REG, &setting_value)) {
			LOG_ERR("plat delay_pcie_perst PCIE3 set failed");
		}
	}

	return 0;
}

// other
bool perm_config_clear(void)
{
	/* clear all temp_threshold perm parameters */
	memset(temp_threshold_user_settings.temperature_reg_val, 0xFF,
	       sizeof(temp_threshold_user_settings.temperature_reg_val));
	if (!set_temp_threshold_user_settings(&temp_threshold_user_settings)) {
		LOG_ERR("The perm_config clear failed");
		return false;
	}

	/* clear pwrlevel perm parameter */
	int32_t setting_value = 0xffffffff;
	char setting_data[4] = { 0 };
	memcpy(setting_data, &setting_value, sizeof(setting_data));
	if (set_user_settings_alert_level_to_eeprom(setting_data, sizeof(setting_data)) != 0) {
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

	return true;
}

void user_settings_init(void)
{
	vr_vout_range_user_settings_init();
	bootstrap_default_settings_init();
	bootstrap_user_settings_init();
	alert_level_user_settings_init();
	thermaltrip_user_settings_init();
	throttle_user_settings_init();
	delay_pcie_perst_user_settings_init();
}