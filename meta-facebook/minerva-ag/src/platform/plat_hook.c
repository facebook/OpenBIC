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
#include "ast_adc.h"
#include "sensor.h"
#include "hal_i2c.h"
#include "plat_i2c.h"
#include "plat_gpio.h"
#include "plat_sensor_table.h"
#include "plat_hook.h"
#include "plat_isr.h"
#include <logging/log.h>
#include "mp2971.h"
#include "isl69259.h"
#include "raa228249.h"
#include "tmp75.h"
#include "tmp431.h"
#include "emc1413.h"
#include "mp29816a.h"
#include "plat_pldm_sensor.h"
#include "plat_class.h"
#include "pmbus.h"

LOG_MODULE_REGISTER(plat_hook);

#define ALERT_LEVEL_USER_SETTINGS_OFFSET 0x8200

static struct k_mutex vr_mutex[VR_MAX_NUM];
static struct k_mutex pwrlevel_mutex;

static bool uart_pwr_event_is_enable = false;

int32_t alert_level_mA_default = 110000;
int32_t alert_level_mA_user_setting = 110000;
bool alert_level_is_assert = false;

int power_level_send_event(bool is_assert, int ubc1_current, int ubc2_current)
{
	if (is_assert == false) {
		//To Do: need to send assert event to BMC
		LOG_INF("UBC1 and UBC2 over current and need to send assert event to BMC");
	} else {
		//To Do: need to send deassert event to BMC
		LOG_INF("UBC1 and UBC2 over current and need to send deassert event to BMC");
	}

	if (uart_pwr_event_is_enable == true) {
		//print send event to consloe
		LOG_INF("send power level event ubc1_current=%dmA,ubc2_current=%dmA,alert_level_mA_user_setting=%dmA",
			ubc1_current, ubc2_current, alert_level_mA_user_setting);
	}

	return 0;
}

bool post_ubc_read(sensor_cfg *cfg, void *args, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	ARG_UNUSED(args);

	sensor_val *reading_val = (sensor_val *)reading;

	int sensor_reading = 0;
	static int ubc1_current_mA;

	if (cfg->num == UBC1_P12V_CURR_A) {
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

	if (cfg->num == UBC2_P12V_CURR_A) {
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
				power_level_send_event(alert_level_is_assert, ubc1_current_mA,
						       sensor_reading);
				alert_level_is_assert = true;
			}
		} else {
			if (alert_level_is_assert == true) {
				power_level_send_event(alert_level_is_assert, ubc1_current_mA,
						       sensor_reading);
				alert_level_is_assert = false;
			}
		}

		k_mutex_unlock(&pwrlevel_mutex);
	}

	return true;
}

#define EEPROM_MAX_WRITE_TIME 5 // the BR24G512 eeprom max write time is 3.5 ms
#define AEGIS_CPLD_ADDR (0x4C >> 1)
#define VR_PRE_READ_ARG(idx)                                                                       \
	{ .mutex = vr_mutex + idx, .vr_page = 0x0 },                                               \
	{                                                                                          \
		.mutex = vr_mutex + idx, .vr_page = 0x1                                            \
	}

vr_pre_proc_arg vr_pre_read_args[] = {
	{ .mutex = vr_mutex + 0, .vr_page = 0x0 },  { .mutex = vr_mutex + 0, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 1, .vr_page = 0x0 },  { .mutex = vr_mutex + 1, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 2, .vr_page = 0x0 },  { .mutex = vr_mutex + 2, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 3, .vr_page = 0x0 },  { .mutex = vr_mutex + 3, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 4, .vr_page = 0x0 },  { .mutex = vr_mutex + 4, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 5, .vr_page = 0x0 },  { .mutex = vr_mutex + 5, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 6, .vr_page = 0x0 },  { .mutex = vr_mutex + 6, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 7, .vr_page = 0x0 },  { .mutex = vr_mutex + 7, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 8, .vr_page = 0x0 },  { .mutex = vr_mutex + 8, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 9, .vr_page = 0x0 },  { .mutex = vr_mutex + 9, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 10, .vr_page = 0x0 }, { .mutex = vr_mutex + 10, .vr_page = 0x1 }
};

mp2971_init_arg mp2971_init_args[] = {
	[0] = { .vout_scale_enable = true },
};

isl69259_init_arg isl69259_init_args[] = {
	[0] = { .vout_scale_enable = true, .vout_scale = (499 / 798.8) },
	[1] = { .vout_scale_enable = true, .vout_scale = (499 / 709.0) },
};

temp_mapping_sensor temp_index_table[] = {
	{ TEMP_INDEX_ON_DIE_1_2, ASIC_DIE_ATH_SENSOR_0_TEMP_C, "CB_ASIC_DIE_ATH_SENSOR_0_TEMP" },
	{ TEMP_INDEX_ON_DIE_3_4, ASIC_DIE_N_OWL_TEMP_C, "CB_ASIC_DIE_N_OWL_TEMP" },
	{ TEMP_INDEX_TOP_INLET, TOP_INLET_TEMP_C, "CB_TOP_INLET_TEMP" },
	{ TEMP_INDEX_BOT_INLET, BOT_INLET_TEMP_C, "CB_BOT_INLET_TEMP" },
	{ TEMP_INDEX_TOP_OUTLET, TOP_OUTLET_TEMP_C, "CB_TOP_OUTLET_TEMP" },
	{ TEMP_INDEX_BOT_OUTLET, BOT_OUTLET_TEMP_C, "CB_BOT_OUTLET_TEMP" },
};

bool temp_sensor_rail_name_get(uint8_t rail, uint8_t **name)
{
	CHECK_NULL_ARG_WITH_RETURN(name, false);

	if (rail >= TEMP_INDEX_MAX) {
		*name = NULL;
		return false;
	}

	*name = (uint8_t *)temp_index_table[rail].sensor_name;
	return true;
}

bool temp_sensor_rail_enum_get(uint8_t *name, uint8_t *num)
{
	CHECK_NULL_ARG_WITH_RETURN(name, false);
	CHECK_NULL_ARG_WITH_RETURN(num, false);

	for (int i = 0; i < TEMP_INDEX_MAX; i++) {
		if (strcmp(name, temp_index_table[i].sensor_name) == 0) {
			*num = i;
			return true;
		}
	}

	LOG_ERR("invalid rail name %s", name);
	return false;
}

void *vr_mutex_get(enum VR_INDEX_E vr_index)
{
	if (vr_index >= VR_INDEX_MAX) {
		LOG_ERR("vr_mutex_get, invalid vr_index %d", vr_index);
		return NULL;
	}

	return vr_mutex + vr_index;
}

bool pre_vr_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	vr_pre_proc_arg *pre_proc_args = (vr_pre_proc_arg *)args;
	uint8_t retry = 5;
	I2C_MSG msg;

	/* mutex lock */
	if (pre_proc_args->mutex) {
		LOG_DBG("%x l %p", cfg->num, pre_proc_args->mutex);
		if (k_mutex_lock(pre_proc_args->mutex, K_MSEC(VR_MUTEX_LOCK_TIMEOUT_MS))) {
			LOG_ERR("pre_vr_read, mutex lock fail");
			return false;
		}
	}

	/* set page */
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 2;
	msg.data[0] = 0x00;
	msg.data[1] = pre_proc_args->vr_page;
	if (i2c_master_write(&msg, retry)) {
		k_mutex_unlock(pre_proc_args->mutex);
		LOG_ERR("pre_vr_read, set page fail");
		return false;
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

		if ((get_board_type() == MINERVA_AEGIS_BD) && (i == 0))
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
			LOG_ERR("post_vr_read, mutex unlock fail");
			return false;
		}
	}

	return true;
}

// clang-format off
temp_threshold_mapping_sensor temp_index_threshold_type_table[] = {
	{ ON_DIE_1_2_LOCAL_HIGH_LIMIT, LOCAL_HIGH_LIMIT, ASIC_DIE_ATH_SENSOR_0_TEMP_C, "CB_ASIC_DIE_ATH_SENSOR_0_TEMP_LOCAL_HIGH_LIM" },
	{ ON_DIE_1_2_LOCAL_LOW_LIMIT, LOCAL_LOW_LIMIT, ASIC_DIE_ATH_SENSOR_0_TEMP_C, "CB_ASIC_DIE_ATH_SENSOR_0_TEMP_LOCAL_LOW_LIM" },
	{ ON_DIE_1_2_REMOTE_1_HIGH_LIMIT, REMOTE_1_HIGH_LIMIT, ASIC_DIE_ATH_SENSOR_0_TEMP_C, "CB_ASIC_DIE_ATH_SENSOR_0_TEMP_REMOTE1_HIGH_LIM" },
	{ ON_DIE_1_2_REMOTE_1_LOW_LIMIT, REMOTE_1_LOW_LIMIT, ASIC_DIE_ATH_SENSOR_0_TEMP_C, "CB_ASIC_DIE_ATH_SENSOR_0_TEMP_REMOTE1_LOW_LIM" },
	{ ON_DIE_1_2_REMOTE_2_HIGH_LIMIT, REMOTE_2_HIGH_LIMIT, ASIC_DIE_ATH_SENSOR_0_TEMP_C, "CB_ASIC_DIE_ATH_SENSOR_0_TEMP_REMOTE2_HIGH_LIM" },
	{ ON_DIE_1_2_REMOTE_2_LOW_LIMIT, REMOTE_2_LOW_LIMIT, ASIC_DIE_ATH_SENSOR_0_TEMP_C, "CB_ASIC_DIE_ATH_SENSOR_0_TEMP_REMOTE2_LOW_LIM" },
	{ ON_DIE_1_2_LOCAL_THERM_LIMIT, LOCAL_THERM_LIMIT, ASIC_DIE_ATH_SENSOR_0_TEMP_C, "CB_ASIC_DIE_ATH_SENSOR_0_TEMP_LOCAL_LIM" },
	{ ON_DIE_1_2_REMOTE_1_THERM_LIMIT, REMOTE_1_THERM_LIMIT, ASIC_DIE_ATH_SENSOR_0_TEMP_C, "CB_ASIC_DIE_ATH_SENSOR_0_TEMP_REMOTE1_LIM" },
	{ ON_DIE_1_2_REMOTE_2_THERM_LIMIT, REMOTE_2_THERM_LIMIT, ASIC_DIE_ATH_SENSOR_0_TEMP_C, "CB_ASIC_DIE_ATH_SENSOR_0_TEMP_REMOTE2_LIM" },


	{ ON_DIE_3_4_LOCAL_HIGH_LIMIT, LOCAL_HIGH_LIMIT, ASIC_DIE_N_OWL_TEMP_C, "CB_ASIC_DIE_N_OWL_TEMP_LOCAL_HIGH_LIM" },
	{ ON_DIE_3_4_LOCAL_LOW_LIMIT, LOCAL_LOW_LIMIT, ASIC_DIE_N_OWL_TEMP_C, "CB_ASIC_DIE_N_OWL_TEMP_LOCAL_LOW_LIM" },
	{ ON_DIE_3_4_REMOTE_1_HIGH_LIMIT, REMOTE_1_HIGH_LIMIT, ASIC_DIE_N_OWL_TEMP_C, "CB_ASIC_DIE_N_OWL_TEMP_REMOTE1_HIGH_LIM" },
	{ ON_DIE_3_4_REMOTE_1_LOW_LIMIT, REMOTE_1_LOW_LIMIT, ASIC_DIE_N_OWL_TEMP_C, "CB_ASIC_DIE_N_OWL_TEMP_REMOTE1_LOW_LIM" },
	{ ON_DIE_3_4_REMOTE_2_HIGH_LIMIT, REMOTE_2_HIGH_LIMIT, ASIC_DIE_N_OWL_TEMP_C, "CB_ASIC_DIE_N_OWL_TEMP_REMOTE2_HIGH_LIM" },
	{ ON_DIE_3_4_REMOTE_2_LOW_LIMIT, REMOTE_2_LOW_LIMIT, ASIC_DIE_N_OWL_TEMP_C, "CB_ASIC_DIE_N_OWL_TEMP_REMOTE2_LOW_LIM" },
	{ ON_DIE_3_4_LOCAL_THERM_LIMIT, LOCAL_THERM_LIMIT, ASIC_DIE_N_OWL_TEMP_C, "CB_ASIC_DIE_N_OWL_TEMP_LOCAL_LIM" },
	{ ON_DIE_3_4_REMOTE_1_THERM_LIMIT, REMOTE_1_THERM_LIMIT, ASIC_DIE_N_OWL_TEMP_C, "CB_ASIC_DIE_N_OWL_TEMP_REMOTE1_LIM" },
	{ ON_DIE_3_4_REMOTE_2_THERM_LIMIT, REMOTE_2_THERM_LIMIT, ASIC_DIE_N_OWL_TEMP_C, "CB_ASIC_DIE_N_OWL_TEMP_REMOTE2_LIM" },
	
	{ TOP_INLET_LOW_LIMIT, LOCAL_LOW_LIMIT, TOP_INLET_TEMP_C, "CB_TOP_INLET_TEMP_LOW_LIM" },
	{ TOP_INLET_HIGH_LIMIT, LOCAL_HIGH_LIMIT, TOP_INLET_TEMP_C, "CB_TOP_INLET_TEMP_HIGH_LIM" },

	{ TOP_OUTLET_LOW_LIMIT, LOCAL_LOW_LIMIT, TOP_OUTLET_TEMP_C, "CB_TOP_OUTLET_TEMP_LOW_LIM" },
	{ TOP_OUTLET_HIGH_LIMIT, LOCAL_HIGH_LIMIT, TOP_OUTLET_TEMP_C, "CB_TOP_OUTLET_TEMP_HIGH_LIM" },

	{ BOT_INLET_LOW_LIMIT, LOCAL_LOW_LIMIT, BOT_INLET_TEMP_C, "CB_BOT_INLET_TEMP_LOW_LIM" },
	{ BOT_INLET_HIGH_LIMIT, LOCAL_HIGH_LIMIT, BOT_INLET_TEMP_C, "CB_BOT_INLET_TEMP_HIGH_LIM" },

	{ BOT_OUTLET_LOW_LIMIT, LOCAL_LOW_LIMIT, BOT_OUTLET_TEMP_C, "CB_BOT_OUTLET_TEMP_LOW_LIM" },
	{ BOT_OUTLET_HIGH_LIMIT, LOCAL_HIGH_LIMIT, BOT_OUTLET_TEMP_C, "CB_BOT_OUTLET_TEMP_HIGH_LIM" },
};
// clang-format on

temp_threshold_user_settings_struct temp_threshold_user_settings = { 0 };
struct temp_threshold_user_settings_struct temp_threshold_default_settings = { 0 };

bool is_mb_dc_on()
{
	/* RST_ATH_PWR_ON_PLD_R1_N is low active,
   * 1 -> power on
   * 0 -> power off
   */
	return gpio_get(RST_ATH_PWR_ON_PLD_R1_N);
}

void vr_mutex_init(void)
{
	for (uint8_t i = 0; i < ARRAY_SIZE(vr_mutex); i++) {
		k_mutex_init(vr_mutex + i);
		LOG_DBG("vr_mutex[%d] %p init", i, vr_mutex + i);
	}

	for (uint8_t i = 0; i < ARRAY_SIZE(vr_pre_read_args); i++) {
		vr_pre_proc_arg *pre_proc_args = vr_pre_read_args + i;
		LOG_DBG("vr_pre_read_args[%d] mutex %p, page %d", i, pre_proc_args->mutex,
			pre_proc_args->vr_page);
	}
}

/* the order is following enum VR_RAIL_E */
vr_mapping_sensor vr_rail_table[] = {
	{ VR_RAIL_E_P3V3, VR_P3V3_VOLT_V, "CB_VR_ASIC_P3V3", 0xffffffff },
	{ VR_RAIL_E_P0V85_PVDD, VR_ASIC_P0V85_PVDD_VOLT_V, "CB_VR_ASIC_P0V85_PVDD", 0xffffffff },
	{ VR_RAIL_E_P0V75_PVDD_CH_N, VR_ASIC_P0V75_PVDD_CH_N_VOLT_V, "CB_VR_ASIC_P0V75_PVDD_CH_N",
	  0xffffffff },
	{ VR_RAIL_E_P0V75_MAX_PHY_N, VR_ASIC_P0V75_MAX_PHY_N_VOLT_V, "CB_VR_ASIC_P0V75_MAX_PHY_N",
	  0xffffffff },
	{ VR_RAIL_E_P0V75_PVDD_CH_S, VR_ASIC_P0V75_PVDD_CH_S_VOLT_V, "CB_VR_ASIC_P0V75_PVDD_CH_S",
	  0xffffffff },
	{ VR_RAIL_E_P0V75_MAX_PHY_S, VR_ASIC_P0V75_MAX_PHY_S_VOLT_V, "CB_VR_ASIC_P0V75_MAX_PHY_S",
	  0xffffffff },
	{ VR_RAIL_E_P0V75_TRVDD_ZONEA, VR_ASIC_P0V75_TRVDD_ZONEA_VOLT_V,
	  "CB_VR_ASIC_P0V75_TRVDD_ZONEA", 0xffffffff },
	{ VR_RAIL_E_P1V8_VPP_HBM0_HBM2_HBM4, VR_ASIC_P1V8_VPP_HBM0_HBM2_HBM4_VOLT_V,
	  "CB_VR_ASIC_P1V8_VPP_HBM0_HBM2_HBM4", 0xffffffff },
	{ VR_RAIL_E_P0V75_TRVDD_ZONEB, VR_ASIC_P0V75_TRVDD_ZONEB_VOLT_V,
	  "CB_VR_ASIC_P0V75_TRVDD_ZONEB", 0xffffffff },
	{ VR_RAIL_E_P0V4_VDDQL_HBM0_HBM2_HBM4, VR_ASIC_P0V4_VDDQL_HBM0_HBM2_HBM4_VOLT_V,
	  "CB_VR_ASIC_P0V4_VDDQL_HBM0_HBM2_HBM4", 0xffffffff },
	{ VR_RAIL_E_P1V1_VDDC_HBM0_HBM2_HBM4, VR_ASIC_P1V1_VDDC_HBM0_HBM2_HBM4_VOLT_V,
	  "CB_VR_ASIC_P1V1_VDDC_HBM0_HBM2_HBM4", 0xffffffff },
	{ VR_RAIL_E_P0V75_VDDPHY_HBM0_HBM2_HBM4, VR_ASIC_P0V75_VDDPHY_HBM0_HBM2_HBM4_VOLT_V,
	  "CB_VR_ASIC_P0V75_VDDPHY_HBM0_HBM2_HBM4", 0xffffffff },
	{ VR_RAIL_E_P0V9_TRVDD_ZONEA, VR_ASIC_P0V9_TRVDD_ZONEA_VOLT_V,
	  "CB_VR_ASIC_P0V9_TRVDD_ZONEA", 0xffffffff },
	{ VR_RAIL_E_P1V8_VPP_HBM1_HBM3_HBM5, VR_ASIC_P1V8_VPP_HBM1_HBM3_HBM5_VOLT_V,
	  "CB_VR_ASIC_P1V8_VPP_HBM1_HBM3_HBM5", 0xffffffff },
	{ VR_RAIL_E_P0V9_TRVDD_ZONEB, VR_ASIC_P0V9_TRVDD_ZONEB_VOLT_V,
	  "CB_VR_ASIC_P0V9_TRVDD_ZONEB", 0xffffffff },
	{ VR_RAIL_E_P0V4_VDDQL_HBM1_HBM3_HBM5, VR_ASIC_P0V4_VDDQL_HBM1_HBM3_HBM5_VOLT_V,
	  "CB_VR_ASIC_P0V4_VDDQL_HBM1_HBM3_HBM5", 0xffffffff },
	{ VR_RAIL_E_P1V1_VDDC_HBM1_HBM3_HBM5, VR_ASIC_P1V1_VDDC_HBM1_HBM3_HBM5_VOLT_V,
	  "CB_VR_ASIC_P1V1_VDDC_HBM1_HBM3_HBM5", 0xffffffff },
	{ VR_RAIL_E_P0V75_VDDPHY_HBM1_HBM3_HBM5, VR_ASIC_P0V75_VDDPHY_HBM1_HBM3_HBM5_VOLT_V,
	  "CB_VR_ASIC_P0V75_VDDPHY_HBM1_HBM3_HBM5", 0xffffffff },
	{ VR_RAIL_E_P0V8_VDDA_PCIE, VR_ASIC_P0V8_VDDA_PCIE_VOLT_V, "CB_VR_ASIC_P0V8_VDDA_PCIE",
	  0xffffffff },
	{ VR_RAIL_E_P1V2_VDDHTX_PCIE, VR_ASIC_P1V2_VDDHTX_PCIE_VOLT_V,
	  "CB_VR_ASIC_P1V2_VDDHTX_PCIE", 0xffffffff },
};

vr_mapping_status vr_status_table[] = {
	{ VR_STAUS_E_STATUS_BYTE, PMBUS_STATUS_BYTE, "STATUS_BYTE" },
	{ VR_STAUS_E_STATUS_WORD, PMBUS_STATUS_WORD, "STATUS_WORD" },
	{ VR_STAUS_E_STATUS_VOUT, PMBUS_STATUS_VOUT, "STATUS_VOUT" },
	{ VR_STAUS_E_STATUS_IOUT, PMBUS_STATUS_IOUT, "STATUS_IOUT" },
	{ VR_STAUS_E_STATUS_INPUT, PMBUS_STATUS_INPUT, "STATUS_INPUT" },
	{ VR_STAUS_E_STATUS_TEMPERATURE, PMBUS_STATUS_TEMPERATURE, "STATUS_TEMPERATURE" },
	{ VR_STAUS_E_STATUS_CML, PMBUS_STATUS_CML, "STATUS_CML_PMBUS" },
};

bool vr_rail_name_get(uint8_t rail, uint8_t **name)
{
	CHECK_NULL_ARG_WITH_RETURN(name, false);

	if (rail >= VR_RAIL_E_MAX) {
		*name = NULL;
		return false;
	}

	*name = (uint8_t *)vr_rail_table[rail].sensor_name;
	return true;
}

bool vr_status_name_get(uint8_t rail, uint8_t **name)
{
	CHECK_NULL_ARG_WITH_RETURN(name, false);

	if (rail >= VR_STAUS_E_MAX) {
		*name = NULL;
		return false;
	}

	*name = (uint8_t *)vr_status_table[rail].vr_status_name;
	return true;
}

#define TEMP_THRESHOLD_USER_SETTINGS_OFFSET 0x8100
#define VR_VOUT_USER_SETTINGS_OFFSET 0x8000
#define SOC_PCIE_PERST_USER_SETTINGS_OFFSET 0x8300

vr_vout_user_settings user_settings = { 0 };
struct vr_vout_user_settings default_settings = { 0 };

bool vr_rail_enum_get(uint8_t *name, uint8_t *num)
{
	CHECK_NULL_ARG_WITH_RETURN(name, false);
	CHECK_NULL_ARG_WITH_RETURN(num, false);

	for (int i = 0; i < VR_RAIL_E_MAX; i++) {
		if (strcmp(name, vr_rail_table[i].sensor_name) == 0) {
			*num = i;
			return true;
		}
	}

	LOG_ERR("invalid rail name %s", name);
	return false;
}

bool vr_status_enum_get(uint8_t *name, uint8_t *num)
{
	CHECK_NULL_ARG_WITH_RETURN(name, false);
	CHECK_NULL_ARG_WITH_RETURN(num, false);

	for (int i = 0; i < VR_STAUS_E_MAX; i++) {
		if (strcmp(name, vr_status_table[i].vr_status_name) == 0) {
			*num = i;
			return true;
		}
	}

	LOG_ERR("invalid vr status name %s", name);
	return false;
}

bool vr_vout_user_settings_get(void *user_settings)
{
	CHECK_NULL_ARG_WITH_RETURN(user_settings, false);

	/* read the user_settings from eeprom */
	I2C_MSG msg = { 0 };
	uint8_t retry = 5;
	msg.bus = I2C_BUS12;
	msg.target_addr = 0xA0 >> 1;
	msg.tx_len = 2;
	msg.data[0] = VR_VOUT_USER_SETTINGS_OFFSET >> 8;
	msg.data[1] = VR_VOUT_USER_SETTINGS_OFFSET & 0xff;
	msg.rx_len = sizeof(struct vr_vout_user_settings);

	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("Failed to read eeprom, bus: %d, addr: 0x%x, reg: 0x%x 0x%x", msg.bus,
			msg.target_addr, msg.data[0], msg.data[1]);
		return false;
	}
	memcpy(user_settings, msg.data, sizeof(struct vr_vout_user_settings));

	return true;
}

bool vr_vout_user_settings_set(void *user_settings)
{
	CHECK_NULL_ARG_WITH_RETURN(user_settings, false);

	/* write the user_settings to eeprom */
	I2C_MSG msg = { 0 };
	uint8_t retry = 5;
	msg.bus = I2C_BUS12;
	msg.target_addr = 0xA0 >> 1;
	msg.tx_len = sizeof(struct vr_vout_user_settings) + 2;
	msg.data[0] = VR_VOUT_USER_SETTINGS_OFFSET >> 8;
	msg.data[1] = VR_VOUT_USER_SETTINGS_OFFSET & 0xff;

	memcpy(&msg.data[2], user_settings, sizeof(struct vr_vout_user_settings));
	LOG_DBG("vout write eeprom, bus: %d, addr: 0x%x, reg: 0x%x 0x%x, tx_len: %d", msg.bus,
		msg.target_addr, msg.data[0], msg.data[1], msg.tx_len);

	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("vout Failed to write eeprom, bus: %d, addr: 0x%x, reg: 0x%x 0x%x, tx_len: %d",
			msg.bus, msg.target_addr, msg.data[0], msg.data[1], msg.tx_len);
		return false;
	}
	k_msleep(EEPROM_MAX_WRITE_TIME);

	return true;
}

bool set_user_settings_soc_pcie_perst_to_eeprom(void *user_settings, uint8_t data_length)
{
	CHECK_NULL_ARG_WITH_RETURN(user_settings, false);

	I2C_MSG msg = { 0 };
	uint8_t retry = 5;
	msg.bus = I2C_BUS12;
	msg.target_addr = 0xA0 >> 1;
	msg.tx_len = data_length + 2;
	msg.data[0] = SOC_PCIE_PERST_USER_SETTINGS_OFFSET >> 8;
	msg.data[1] = SOC_PCIE_PERST_USER_SETTINGS_OFFSET & 0xff;

	memcpy(&msg.data[2], user_settings, data_length);
	LOG_DBG("soc_pcie_perst write eeprom, bus: %d, addr: 0x%x, reg: 0x%x 0x%x, tx_len: %d",
		msg.bus, msg.target_addr, msg.data[0], msg.data[1], msg.tx_len);

	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("soc_pcie_perst Failed to write eeprom, bus: %d, addr: 0x%x, reg: 0x%x 0x%x, tx_len: %d",
			msg.bus, msg.target_addr, msg.data[0], msg.data[1], msg.tx_len);
		return false;
	}
	k_msleep(EEPROM_MAX_WRITE_TIME);

	return true;
}

bool get_user_settings_soc_pcie_perst_from_eeprom(void *user_settings, uint8_t data_length)
{
	CHECK_NULL_ARG_WITH_RETURN(user_settings, false);

	I2C_MSG msg = { 0 };
	uint8_t retry = 5;
	msg.bus = I2C_BUS12;
	msg.target_addr = 0xA0 >> 1;
	msg.tx_len = 2;
	msg.data[0] = SOC_PCIE_PERST_USER_SETTINGS_OFFSET >> 8;
	msg.data[1] = SOC_PCIE_PERST_USER_SETTINGS_OFFSET & 0xff;
	msg.rx_len = data_length;

	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("Failed to read eeprom, bus: %d, addr: 0x%x, reg: 0x%x%x", msg.bus,
			msg.target_addr, msg.data[0], msg.data[1]);
		return false;
	}
	memcpy(user_settings, msg.data, data_length);

	return true;
}

int set_user_settings_alert_level_to_eeprom(void *user_settings, uint8_t data_length)
{
	CHECK_NULL_ARG_WITH_RETURN(user_settings, -1);

	I2C_MSG msg = { 0 };
	uint8_t retry = 5;
	msg.bus = I2C_BUS12;
	msg.target_addr = 0xA0 >> 1;
	msg.tx_len = data_length + 2;
	msg.data[0] = ALERT_LEVEL_USER_SETTINGS_OFFSET >> 8;
	msg.data[1] = ALERT_LEVEL_USER_SETTINGS_OFFSET & 0xff;

	memcpy(&msg.data[2], user_settings, data_length);
	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("Failed to write eeprom, bus: %d, addr: 0x%x, reg: 0x%x%x", msg.bus,
			msg.target_addr, msg.data[0], msg.data[1]);
		return -1;
	}
	k_msleep(EEPROM_MAX_WRITE_TIME);

	return 0;
}

int get_user_settings_alert_level_from_eeprom(void *user_settings, uint8_t data_length)
{
	CHECK_NULL_ARG_WITH_RETURN(user_settings, -1);

	I2C_MSG msg = { 0 };
	uint8_t retry = 5;
	msg.bus = I2C_BUS12;
	msg.target_addr = 0xA0 >> 1;
	msg.tx_len = 2;
	msg.data[0] = ALERT_LEVEL_USER_SETTINGS_OFFSET >> 8;
	msg.data[1] = ALERT_LEVEL_USER_SETTINGS_OFFSET & 0xff;
	msg.rx_len = data_length;

	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("Failed to read eeprom, bus: %d, addr: 0x%x, reg: 0x%x%x", msg.bus,
			msg.target_addr, msg.data[0], msg.data[1]);
		return -1;
	}
	memcpy(user_settings, msg.data, data_length);

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

static bool soc_pcie_perst_user_settings_init(void)
{
	uint8_t setting_data = 0xFF;
	if (!get_user_settings_soc_pcie_perst_from_eeprom(&setting_data, sizeof(setting_data))) {
		LOG_ERR("get soc_pcie_perst user settings fail");
		return false;
	}

	if (setting_data != 0xFF) {
		if (!plat_i2c_write(I2C_BUS5, AEGIS_CPLD_ADDR, 0x43, &setting_data,
				    sizeof(setting_data))) {
			LOG_ERR("Can't set soc_pcie_perst=%d by user settings", setting_data);
			return false;
		}
	}

	return true;
}

static bool vr_vout_user_settings_init(void)
{
	if (vr_vout_user_settings_get(&user_settings) == false) {
		LOG_ERR("get vout user settings fail");
		return false;
	}

	for (int i = 0; i < VR_RAIL_E_MAX; i++) {
		if (user_settings.vout[i] != 0xffff) {
			/* write vout */
			uint16_t millivolt = user_settings.vout[i];
			if (!plat_set_vout_command(i, &millivolt, false, false)) {
				LOG_ERR("Can't set vout[%d]=%x by user settings", i, millivolt);
				return false;
			}
			LOG_INF("set [%x]%s: %dmV", i, vr_rail_table[i].sensor_name,
				user_settings.vout[i]);
		}
	}

	return true;
}

static bool vr_vout_default_settings_init(void)
{
	for (int i = 0; i < VR_RAIL_E_MAX; i++) {
		if ((get_board_type() == MINERVA_AEGIS_BD) && (i == 0)) {
			default_settings.vout[i] = 0xffff;
			continue; // skip osfp p3v3 on AEGIS BD
		}
		uint16_t vout = 0;
		if (!plat_get_vout_command(i, &vout)) {
			LOG_ERR("Can't find vout default by rail index: %d", i);
			return false;
		}
		default_settings.vout[i] = vout;
	}

	return true;
}

bool temp_index_threshold_type_name_get(uint8_t type, uint8_t **name)
{
	CHECK_NULL_ARG_WITH_RETURN(name, false);

	if (type >= PLAT_TEMP_INDEX_THRESHOLD_TYPE_MAX) {
		*name = NULL;
		return false;
	}

	*name = (uint8_t *)temp_index_threshold_type_table[type].temp_threshold_name;
	return true;
}

bool temp_threshold_type_enum_get(uint8_t *name, uint8_t *num)
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

bool temp_threshold_user_settings_get(void *temp_threshold_user_settings)
{
	CHECK_NULL_ARG_WITH_RETURN(temp_threshold_user_settings, false);

	/* TODO: read the temp_threshold_user_settings from eeprom */
	I2C_MSG msg = { 0 };
	uint8_t retry = 5;
	msg.bus = I2C_BUS12;
	msg.target_addr = 0xA0 >> 1;
	msg.tx_len = 2;
	msg.data[0] = TEMP_THRESHOLD_USER_SETTINGS_OFFSET >> 8;
	msg.data[1] = TEMP_THRESHOLD_USER_SETTINGS_OFFSET & 0xff;
	msg.rx_len = sizeof(struct temp_threshold_user_settings_struct);

	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("Failed to read eeprom, bus: %d, addr: 0x%x, reg: 0x%x 0x%x", msg.bus,
			msg.target_addr, msg.data[0], msg.data[1]);
		return false;
	}
	memcpy(temp_threshold_user_settings, msg.data,
	       sizeof(struct temp_threshold_user_settings_struct));

	return true;
}

bool temp_threshold_user_settings_set(void *temp_threshold_user_settings)
{
	CHECK_NULL_ARG_WITH_RETURN(temp_threshold_user_settings, false);

	/* TODO: write the temp_threshold_user_settings to eeprom */
	I2C_MSG msg = { 0 };
	uint8_t retry = 5;
	msg.bus = I2C_BUS12;
	msg.target_addr = 0xA0 >> 1;
	msg.tx_len = sizeof(struct temp_threshold_user_settings_struct) + 2;
	msg.data[0] = TEMP_THRESHOLD_USER_SETTINGS_OFFSET >> 8;
	msg.data[1] = TEMP_THRESHOLD_USER_SETTINGS_OFFSET & 0xff;

	memcpy(&msg.data[2], temp_threshold_user_settings,
	       sizeof(struct temp_threshold_user_settings_struct));
	LOG_DBG("temp write eeprom, bus: %d, addr: 0x%x, reg: 0x%x 0x%x, tx_len: %d", msg.bus,
		msg.target_addr, msg.data[0], msg.data[1], msg.tx_len);

	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("temp Failed to write eeprom, bus: %d, addr: 0x%x, reg: 0x%x 0x%x, tx_len: %d",
			msg.bus, msg.target_addr, msg.data[0], msg.data[1], msg.tx_len);
		return false;
	}
	k_msleep(EEPROM_MAX_WRITE_TIME);
	return true;
}

static bool temp_threshold_user_settings_init(void)
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

static bool temp_threshold_default_settings_init(void)
{
	for (int i = 0; i < PLAT_TEMP_INDEX_THRESHOLD_TYPE_MAX; i++) {
		uint32_t temp_threshold = 0;
		if (!plat_get_temp_threshold(i, &temp_threshold)) {
			LOG_ERR("Can't find temp_threshold default by type index: %x", i);
			return false;
		}
		temp_threshold_default_settings.temperature_reg_val[i] = temp_threshold;
	}

	return true;
}

/* init the user & default settings value by shell command */
void user_settings_init(void)
{
	alert_level_user_settings_init();
	vr_vout_default_settings_init();
	vr_vout_user_settings_init();
	temp_threshold_user_settings_init();
	temp_threshold_default_settings_init();
	soc_pcie_perst_user_settings_init();
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

void pwr_level_mutex_init(void)
{
	k_mutex_init(&pwrlevel_mutex);

	return;
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

bool vr_rail_voltage_peak_get(uint8_t *name, int *peak_value)
{
	CHECK_NULL_ARG_WITH_RETURN(name, false);
	CHECK_NULL_ARG_WITH_RETURN(peak_value, false);

	for (int i = 0; i < VR_RAIL_E_MAX; i++) {
		if (strcmp(name, vr_rail_table[i].sensor_name) == 0) {
			*peak_value = vr_rail_table[i].peak_value;
			return true;
		}
	}

	return false;
}

bool vr_rail_voltage_peak_clear(uint8_t rail_index)
{
	if (rail_index >= VR_RAIL_E_MAX) {
		return false;
	}

	vr_rail_table[rail_index].peak_value = 0xffffffff;

	return true;
}

bool plat_get_vr_status(uint8_t rail, uint8_t vr_status_rail, uint16_t *vr_status)
{
	CHECK_NULL_ARG_WITH_RETURN(vr_status, false);

	bool ret = false;
	uint8_t sensor_id = vr_rail_table[rail].sensor_id;
	sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);

	if (cfg == NULL) {
		LOG_ERR("Failed to get sensor config for sensor 0x%x", sensor_id);
		return false;
	}

	vr_pre_proc_arg *pre_proc_args = vr_pre_read_args + rail;

	if (cfg->pre_sensor_read_hook) {
		if (!cfg->pre_sensor_read_hook(cfg, cfg->pre_sensor_read_args)) {
			LOG_ERR("sensor id: 0x%x pre-read fail", sensor_id);
			goto err;
		}
	}

	uint16_t pmbus_reg_id = vr_status_table[vr_status_rail].pmbus_reg;

	switch (cfg->type) {
	case sensor_dev_isl69259:
		if (!isl69260_get_vr_status(cfg, pre_proc_args->vr_page, pmbus_reg_id, vr_status)) {
			LOG_ERR("The VR ISL69260 vr status reading failed");
			goto err;
		}
		break;
	case sensor_dev_mp2971:
		if (!mp2971_get_vr_status(cfg, pre_proc_args->vr_page, pmbus_reg_id, vr_status)) {
			LOG_ERR("The VR MPS2971 vr status reading failed");
			goto err;
		}
		break;
	case sensor_dev_mp29816a:
		if (!mp29816a_get_vr_status(cfg, pre_proc_args->vr_page, pmbus_reg_id, vr_status)) {
			LOG_ERR("The VR MPS29816a vr status reading failed");
			goto err;
		}
		break;
	case sensor_dev_raa228249:
		if (!raa228249_get_vr_status(cfg, pre_proc_args->vr_page, pmbus_reg_id,
					     vr_status)) {
			LOG_ERR("The VR RAA228249 vr status reading failed");
			goto err;
		}
		break;
	default:
		LOG_ERR("Unsupport VR type(%x)", cfg->type);
		goto err;
	}

	ret = true;
err:
	if (cfg->post_sensor_read_hook) {
		if (cfg->post_sensor_read_hook(cfg, cfg->post_sensor_read_args, NULL) == false) {
			LOG_ERR("sensor id: 0x%x post-read fail", sensor_id);
		}
	}
	return ret;
}

bool plat_clear_vr_status(uint8_t rail)
{
	bool ret = false;
	uint8_t sensor_id = vr_rail_table[rail].sensor_id;
	sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);

	if (cfg == NULL) {
		LOG_ERR("Failed to get sensor config for sensor 0x%x", sensor_id);
		return false;
	}

	vr_pre_proc_arg *pre_proc_args = vr_pre_read_args + rail;

	if (cfg->pre_sensor_read_hook) {
		if (!cfg->pre_sensor_read_hook(cfg, cfg->pre_sensor_read_args)) {
			LOG_ERR("sensor id: 0x%x pre-read fail", sensor_id);
			goto err;
		}
	}

	switch (cfg->type) {
	case sensor_dev_isl69259:
		if (!isl69260_clear_vr_status(cfg, pre_proc_args->vr_page)) {
			LOG_ERR("The VR ISL69260 vr status clear failed");
			goto err;
		}
		break;
	case sensor_dev_mp2971:
		if (!mp2971_clear_vr_status(cfg, pre_proc_args->vr_page)) {
			LOG_ERR("The VR MPS2971 vr status clear failed");
			goto err;
		}
		break;
	case sensor_dev_mp29816a:
		if (!mp29816a_clear_vr_status(cfg, pre_proc_args->vr_page)) {
			LOG_ERR("The VR MPS29816a vr status clear failed");
			goto err;
		}
		break;
	case sensor_dev_raa228249:
		if (!raa228249_clear_vr_status(cfg, pre_proc_args->vr_page)) {
			LOG_ERR("The VR RAA228249 vr status clear failed");
			goto err;
		}
		break;
	default:
		LOG_ERR("Unsupport VR type(%x)", cfg->type);
		goto err;
	}

	ret = true;
err:
	if (cfg->post_sensor_read_hook) {
		if (cfg->post_sensor_read_hook(cfg, cfg->post_sensor_read_args, NULL) == false) {
			LOG_ERR("sensor id: 0x%x post-read fail", sensor_id);
		}
	}
	return ret;
}

/* If any perm parameter are added, remember to update this function accordingly.　*/
bool perm_config_clear(void)
{
	/* clear all vout perm parameters */
	for (int i = 0; i < VR_RAIL_E_MAX; i++) {
		user_settings.vout[i] = 0xffff;
	}
	memset(user_settings.vout, 0xFF, sizeof(user_settings.vout));
	if (!vr_vout_user_settings_set(&user_settings)) {
		LOG_ERR("The perm_config clear failed");
		return false;
	}

	/* clear all temp_threshold perm parameters */
	for (int i = 0; i < PLAT_TEMP_INDEX_THRESHOLD_TYPE_MAX; i++) {
		temp_threshold_user_settings.temperature_reg_val[i] = 0xffffffff;
	}
	memset(temp_threshold_user_settings.temperature_reg_val, 0xFF,
	       sizeof(temp_threshold_user_settings.temperature_reg_val));
	if (!temp_threshold_user_settings_set(&temp_threshold_user_settings)) {
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
	uint8_t setting_value_for_soc_pcie_perst = 0xFF;
	if (!set_user_settings_soc_pcie_perst_to_eeprom(&setting_value_for_soc_pcie_perst,
							sizeof(setting_value_for_soc_pcie_perst))) {
		LOG_ERR("The perm_config clear failed");
		return false;
	}

	return true;
}

bool plat_get_vout_command(uint8_t rail, uint16_t *millivolt)
{
	CHECK_NULL_ARG_WITH_RETURN(millivolt, false);

	bool ret = false;
	uint8_t sensor_id = vr_rail_table[rail].sensor_id;
	sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);

	if (cfg == NULL) {
		LOG_ERR("Failed to get sensor config for sensor 0x%x", sensor_id);
		return false;
	}

	vr_pre_proc_arg *pre_proc_args = vr_pre_read_args + rail;

	if (cfg->pre_sensor_read_hook) {
		if (!cfg->pre_sensor_read_hook(cfg, cfg->pre_sensor_read_args)) {
			LOG_ERR("sensor id: 0x%x pre-read fail", sensor_id);
			goto err;
		}
	}

	switch (cfg->type) {
	case sensor_dev_isl69259:
		if (!isl69260_get_vout_command(cfg, pre_proc_args->vr_page, millivolt)) {
			LOG_ERR("The VR ISL69260 vout reading failed");
			goto err;
		}
		break;
	case sensor_dev_mp2971:
		if (!mp2971_get_vout_command(cfg, pre_proc_args->vr_page, millivolt)) {
			LOG_ERR("The VR MPS2971 vout reading failed");
			goto err;
		}
		break;
	case sensor_dev_mp29816a:
		if (!mp29816a_get_vout_command(cfg, pre_proc_args->vr_page, millivolt)) {
			LOG_ERR("The VR MPS29816a vout reading failed");
			goto err;
		}
		break;
	case sensor_dev_raa228249:
		if (!raa228249_get_vout_command(cfg, pre_proc_args->vr_page, millivolt)) {
			LOG_ERR("The VR RAA228249 vout reading failed");
			goto err;
		}
		break;
	default:
		LOG_ERR("Unsupport VR type(%x)", cfg->type);
		goto err;
	}

	ret = true;
err:
	if (cfg->post_sensor_read_hook) {
		if (cfg->post_sensor_read_hook(cfg, cfg->post_sensor_read_args, NULL) == false) {
			LOG_ERR("sensor id: 0x%x post-read fail", sensor_id);
		}
	}
	return ret;
}

bool plat_set_vout_command(uint8_t rail, uint16_t *millivolt, bool is_default, bool is_perm)
{
	CHECK_NULL_ARG_WITH_RETURN(millivolt, false);

	bool ret = false;
	uint8_t sensor_id = vr_rail_table[rail].sensor_id;
	sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);
	uint16_t setting_millivolt = *millivolt;

	if (cfg == NULL) {
		LOG_ERR("Failed to get sensor config for sensor 0x%x", sensor_id);
		return false;
	}

	vr_pre_proc_arg *pre_proc_args = vr_pre_read_args + rail;

	if (cfg->pre_sensor_read_hook) {
		if (!cfg->pre_sensor_read_hook(cfg, cfg->pre_sensor_read_args)) {
			LOG_ERR("sensor id: 0x%x pre-read fail", sensor_id);
			goto err;
		}
	}

	if (is_default) {
		*millivolt = default_settings.vout[rail];
		setting_millivolt = default_settings.vout[rail];
	}

	switch (cfg->type) {
	case sensor_dev_isl69259:
		if (!isl69260_set_vout_command(cfg, pre_proc_args->vr_page, millivolt)) {
			LOG_ERR("The VR ISL69260 vout setting failed");
			goto err;
		}
		break;
	case sensor_dev_mp2971:
		if (!mp2971_set_vout_command(cfg, pre_proc_args->vr_page, millivolt)) {
			LOG_ERR("The VR MPS2971 vout setting failed");
			goto err;
		}
		break;
	case sensor_dev_mp29816a:
		if (!mp29816a_set_vout_command(cfg, pre_proc_args->vr_page, millivolt)) {
			LOG_ERR("The VR MPS29816a vout setting failed");
			goto err;
		}
		break;
	case sensor_dev_raa228249:
		if (!raa228249_set_vout_command(cfg, pre_proc_args->vr_page, millivolt)) {
			LOG_ERR("The VR RAA228249 vout setting failed");
			goto err;
		}
		break;
	default:
		LOG_ERR("Unsupport VR type(%x)", cfg->type);
		goto err;
	}

	if (is_perm) {
		user_settings.vout[rail] = setting_millivolt;
		vr_vout_user_settings_set(&user_settings);
	}

	ret = true;
err:
	if (cfg->post_sensor_read_hook) {
		if (cfg->post_sensor_read_hook(cfg, cfg->post_sensor_read_args, NULL) == false) {
			LOG_ERR("sensor id: 0x%x post-read fail", sensor_id);
		}
	}
	return ret;
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
		if (!plat_i2c_read(I2C_BUS5, AEGIS_CPLD_ADDR, 0x2B, data, 1)) {
			LOG_ERR("Failed to read TEMP TMP75 from cpld");
			goto err;
		}

		switch (rail) {
		case TEMP_INDEX_TOP_INLET:
			*temp_status = (data[0] & BIT(2)) >> 2;
			break;
		case TEMP_INDEX_TOP_OUTLET:
			*temp_status = (data[0] & BIT(3)) >> 3;
			break;
		case TEMP_INDEX_BOT_INLET:
			*temp_status = (data[0] & BIT(4)) >> 4;
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
	case sensor_dev_emc1413:
		if (!emc1413_get_temp_status(cfg, temp_status)) {
			LOG_ERR("The TMP EMC1413 temp status reading failed");
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
	case sensor_dev_tmp75: {
		uint8_t data[1] = { 0 };
		if (!plat_i2c_read(I2C_BUS5, AEGIS_CPLD_ADDR, 0x2B, data, 1)) {
			LOG_ERR("Failed to read TEMP TMP75 from cpld");
			goto err;
		}

		switch (rail) {
		case TEMP_INDEX_TOP_INLET:
			data[0] &= ~BIT(2);
			break;
		case TEMP_INDEX_TOP_OUTLET:
			data[0] &= ~BIT(3);
			break;
		case TEMP_INDEX_BOT_INLET:
			data[0] &= ~BIT(4);
			break;
		case TEMP_INDEX_BOT_OUTLET:
			data[0] &= ~BIT(5);
			break;
		default:
			LOG_ERR("Unsupport TEMP TMP75 alert pin");
			goto err;
		}

		if (!plat_i2c_write(I2C_BUS5, AEGIS_CPLD_ADDR, 0x2B, data, 1)) {
			LOG_ERR("Failed to clear TEMP TMP75 to cpld");
			goto err;
		}
	} break;
	case sensor_dev_tmp431:
		if (!tmp432_clear_temp_status(cfg)) {
			LOG_ERR("The TEMP TMP432 temp status clear failed");
			goto err;
		}
		break;
	case sensor_dev_emc1413:
		if (!emc1413_clear_temp_status(cfg)) {
			LOG_ERR("The TMP EMC1413 temp status clear failed");
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

#define PLAT_VOUT_GET_SET_HANDLER(get_set, min_max)                                                \
	bool plat_##get_set##_vout_##min_max(uint8_t rail, uint16_t *millivolt)                    \
	{                                                                                          \
		CHECK_NULL_ARG_WITH_RETURN(millivolt, false);                                      \
                                                                                                   \
		bool ret = false;                                                                  \
                                                                                                   \
		uint8_t sensor_id = vr_rail_table[rail].sensor_id;                                 \
                                                                                                   \
		sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);                          \
                                                                                                   \
		if (cfg == NULL) {                                                                 \
			LOG_ERR("Failed to get sensor config for sensor 0x%x", sensor_id);         \
			return false;                                                              \
		}                                                                                  \
                                                                                                   \
		vr_pre_proc_arg *pre_proc_args = vr_pre_read_args + rail;                          \
                                                                                                   \
		if (cfg->pre_sensor_read_hook) {                                                   \
			if (!cfg->pre_sensor_read_hook(cfg, cfg->pre_sensor_read_args)) {          \
				goto err;                                                          \
			}                                                                          \
		}                                                                                  \
                                                                                                   \
		switch (cfg->type) {                                                               \
		case sensor_dev_mp2971:                                                            \
			if (!mp2971_##get_set##_vout_##min_max(cfg, pre_proc_args->vr_page,        \
							       millivolt)) {                       \
				LOG_ERR("The VR MPS2971 vout reading failed");                     \
				goto err;                                                          \
			}                                                                          \
			break;                                                                     \
		case sensor_dev_mp29816a:                                                          \
			if (!mp29816a_##get_set##_vout_##min_max(cfg, pre_proc_args->vr_page,      \
								 millivolt)) {                     \
				LOG_ERR("The VR MPS29816a vout reading failed");                   \
				goto err;                                                          \
			}                                                                          \
			break;                                                                     \
		case sensor_dev_isl69259:                                                          \
			if (!isl69260_##get_set##_vout_##min_max(cfg, pre_proc_args->vr_page,      \
								 millivolt)) {                     \
				LOG_ERR("The VR ISL69260 vout reading failed");                    \
				goto err;                                                          \
			}                                                                          \
			break;                                                                     \
		case sensor_dev_raa228249:                                                         \
			if (!raa228249_##get_set##_vout_##min_max(cfg, pre_proc_args->vr_page,     \
								  millivolt)) {                    \
				LOG_ERR("The VR RAA228249 vout reading failed");                   \
				goto err;                                                          \
			}                                                                          \
			break;                                                                     \
		default:                                                                           \
			LOG_ERR("Unsupport VR type(%x)", cfg->type);                               \
			goto err;                                                                  \
		}                                                                                  \
                                                                                                   \
		ret = true;                                                                        \
                                                                                                   \
	err:                                                                                       \
		if (cfg->post_sensor_read_hook) {                                                  \
			if (cfg->post_sensor_read_hook(cfg, cfg->post_sensor_read_args, NULL) ==   \
			    false) {                                                               \
				LOG_ERR("sensor id: 0x%x post-read fail", sensor_id);              \
			}                                                                          \
		}                                                                                  \
		return ret;                                                                        \
	}

PLAT_VOUT_GET_SET_HANDLER(get, min);
PLAT_VOUT_GET_SET_HANDLER(get, max);
PLAT_VOUT_GET_SET_HANDLER(set, min);
PLAT_VOUT_GET_SET_HANDLER(set, max);

#undef PLAT_VOUT_GET_SET_HANDLER

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
			LOG_ERR("The TMP431 temp threshold reading failed");
			return false;
		}
		break;
	case sensor_dev_emc1413:
		if (!emc1413_set_temp_threshold(cfg, temp_threshold_type_tmp,
						millidegree_celsius)) {
			LOG_ERR("The EMC1413 temp threshold reading failed");
			return false;
		}
		break;
	case sensor_dev_tmp75:
		if (!tmp75_set_temp_threshold(cfg, temp_threshold_type_tmp, millidegree_celsius)) {
			LOG_ERR("The TMP75 temp threshold reading failed");
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
		temp_threshold_user_settings_set(&temp_threshold_user_settings);
	}

	return true;
}
