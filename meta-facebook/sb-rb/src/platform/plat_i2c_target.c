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

/*
  NAME: I2C TARGET INIT
  FILE: plat_i2c_target.c
  DESCRIPTION: Provide i2c target EN/CFG table "I2C_TARGET_EN_TABLE[]/I2C_TARGET_CFG_TABLE[]" for init target config.
  AUTHOR: MouchenHung
  DATE/VERSION: 2021.11.26 - v1.1
  Note: 
    (1) "plat_i2c_target.h" is included by "hal_i2c_target.h"
*/

#include <zephyr.h>
#include <stdio.h>
#include <stdlib.h>
#include "plat_i2c_target.h"
#include <logging/log.h>
#include "libutil.h"
#include "plat_pldm_sensor.h"
#include "pldm_sensor.h"
#include "plat_version.h"
#include "plat_class.h"
#include "plat_cpld.h"
#include "plat_hook.h"
#include "plat_i2c.h"
#include "plat_ioexp.h"

LOG_MODULE_REGISTER(plat_i2c_target);

#define MAX_TARGET_TABLE_NUM 12
#define DATA_TABLE_LENGTH_1 1
#define DATA_TABLE_LENGTH_2 2
#define DATA_TABLE_LENGTH_4 4
#define DEVICE_TYPE 0x01
#define REGISTER_LAYOUT_VERSION 0x01
#define SENSOR_READING_PDR_INDEX_MAX 50
#define SENSOR_INIT_PDR_INDEX_MAX 248
#define PLAT_MASTER_WRITE_STACK_SIZE 1024
#define AEGIS_CARRIER_BOARD_ID 0x0000
#define CPLD_VERSION_GET_REG 0x32
#define CPLD_VERSION_GET_REG_LEN 4
#define STRAP_SET_TYPE 0x44 // 01000100
#define VR_PWR_BUF_SIZE 38
#define I2C_TARGET_BUS_ASIC I2C_BUS7 // asic bus is I2C_BUS7, I2C_BUS6 is only for test

static bool command_reply_data_handle(void *arg);
void set_bootstrap_element_handler();
K_WORK_DEFINE(set_bootstrap_element_work, set_bootstrap_element_handler);

K_THREAD_STACK_DEFINE(plat_master_write_stack, PLAT_MASTER_WRITE_STACK_SIZE);
struct k_thread plat_master_write_thread;
k_tid_t plat_master_write_tid;

struct i2c_target_data *test_for_reading = NULL;

/* I2C target init-enable table */
const bool I2C_TARGET_ENABLE_TABLE[MAX_TARGET_NUM] = {
	TARGET_DISABLE, TARGET_DISABLE, TARGET_DISABLE, TARGET_DISABLE,
	TARGET_DISABLE, TARGET_ENABLE,	TARGET_ENABLE,	TARGET_DISABLE,
	TARGET_DISABLE, TARGET_DISABLE, TARGET_DISABLE, TARGET_DISABLE,
};

void *allocate_table(void **buffer, size_t buffer_size)
{
	if (*buffer) {
		free(*buffer);
		*buffer = NULL;
	}

	*buffer = malloc(buffer_size);
	if (!*buffer) {
		LOG_ERR("Memory allocation failed!");
		return NULL;
	}
	return *buffer;
}

voltage_rail_mapping_sensor voltage_rail_mapping_table[] = {
	{ CONTROL_VOL_VR_ASIC_P0V75_VDDPHY_HBM0246_REG, VR_RAIL_E_ASIC_P0V75_VDDPHY_HBM0246 },
	{ CONTROL_VOL_VR_ASIC_P0V75_VDDPHY_HBM1357_REG, VR_RAIL_E_ASIC_P0V75_VDDPHY_HBM1357 },
	{ CONTROL_VOL_VR_ASIC_P1V1_VDDQC_HBM0246_REG, VR_RAIL_E_ASIC_P1V1_VDDQC_HBM0246 },
	{ CONTROL_VOL_VR_ASIC_P1V1_VDDQC_HBM1357_REG, VR_RAIL_E_ASIC_P1V1_VDDQC_HBM1357 },
	{ CONTROL_VOL_VR_ASIC_P0V4_VDDQL_HBM0246_REG, VR_RAIL_E_ASIC_P0V4_VDDQL_HBM0246 },
	{ CONTROL_VOL_VR_ASIC_P0V4_VDDQL_HBM1357_REG, VR_RAIL_E_ASIC_P0V4_VDDQL_HBM1357 },
	{ CONTROL_VOL_VR_ASIC_P1V8_VPP_HBM0246_REG, VR_RAIL_E_ASIC_P1V8_VPP_HBM0246 },
	{ CONTROL_VOL_VR_ASIC_P1V8_VPP_HBM1357_REG, VR_RAIL_E_ASIC_P1V8_VPP_HBM1357 },
	{ CONTROL_VOL_VR_ASIC_P0V85_MEDHA0_VDD_REG, VR_RAIL_E_ASIC_P0V85_MEDHA0_VDD },
	{ CONTROL_VOL_VR_ASIC_P0V85_MEDHA1_VDD_REG, VR_RAIL_E_ASIC_P0V85_MEDHA1_VDD },
};

uint8_t get_vr_rail_by_control_vol_reg(uint8_t control_vol_reg)
{
	for (int i = 0; i < ARRAY_SIZE(voltage_rail_mapping_table); i++) {
		if (voltage_rail_mapping_table[i].control_vol_reg == control_vol_reg) {
			return voltage_rail_mapping_table[i].vr_rail_e;
		}
	}
	return VR_RAIL_E_MAX;
}
plat_sensor_init_data *sensor_init_data_table[DATA_TABLE_LENGTH_2] = { NULL };
plat_sensor_reading *sensor_reading_table[DATA_TABLE_LENGTH_4] = { NULL };
plat_inventory_ids *inventory_ids_table[DATA_TABLE_LENGTH_1] = { NULL };
plat_strap_capability *strap_capability_table[DATA_TABLE_LENGTH_1] = { NULL };
plat_i2c_bridge_command_status *i2c_bridge_command_status_table[DATA_TABLE_LENGTH_1] = { NULL };
plat_i2c_bridge_command_response_data
	*i2c_bridge_command_response_data_table[DATA_TABLE_LENGTH_1] = { NULL };
bool initialize_sensor_data(telemetry_info *telemetry_info, uint8_t *buffer_size)
{
	CHECK_NULL_ARG_WITH_RETURN(telemetry_info, false);
	int table_index = telemetry_info->telemetry_offset - SENSOR_INIT_DATA_0_REG;

	if (table_index >= DATA_TABLE_LENGTH_2) {
		LOG_ERR("Invalid table index: %d", table_index);
		return false;
	}

	// Calculate num_idx
	int num_idx = (SENSOR_NUM_NUMBERS - 1) - (SENSOR_INIT_PDR_INDEX_MAX * table_index);
	num_idx = (num_idx > 0) ?
			  ((num_idx > SENSOR_INIT_PDR_INDEX_MAX) ? SENSOR_INIT_PDR_INDEX_MAX :
								   num_idx) :
			  0;

	// Calculate the memory size
	size_t table_size = sizeof(plat_sensor_init_data) + num_idx * sizeof(uint8_t);
	plat_sensor_init_data *sensor_data =
		allocate_table((void **)&sensor_init_data_table[table_index], table_size);

	if (!sensor_data) {
		LOG_ERR("sensor_data allocation failed!");
		return false;
	}

	sensor_data->device_type = DEVICE_TYPE;
	sensor_data->register_layout_version = REGISTER_LAYOUT_VERSION;
	sensor_data->num_idx = num_idx;
	sensor_data->reserved_1 = 0xFF;
	sensor_data->sbi = table_index * SENSOR_INIT_PDR_INDEX_MAX;
	sensor_data->max_pdr_idx = (table_index == 0x00) ? SENSOR_NUM_NUMBERS - 2 : 0xFFFF;
	memset(sensor_data->sensor_r_len, 4, num_idx * sizeof(uint8_t));

	*buffer_size = (uint8_t)table_size;
	return true;
}
bool initialize_sensor_reading(telemetry_info *telemetry_info, uint8_t *buffer_size)
{
	CHECK_NULL_ARG_WITH_RETURN(telemetry_info, false);

	int table_index = telemetry_info->telemetry_offset - SENSOR_READING_0_REG;
	if (table_index < 0 || table_index >= DATA_TABLE_LENGTH_4)
		return false;

	int num_idx = (SENSOR_NUM_NUMBERS - 1) - (SENSOR_READING_PDR_INDEX_MAX * table_index);
	num_idx = (num_idx > 0) ?
			  ((num_idx > SENSOR_READING_PDR_INDEX_MAX) ? SENSOR_READING_PDR_INDEX_MAX :
								      num_idx) :
			  0;

	size_t table_size = sizeof(plat_sensor_reading) + num_idx * sizeof(sensor_entry);
	plat_sensor_reading *sensor_data =
		allocate_table((void **)&sensor_reading_table[table_index], table_size);
	if (!sensor_data)
		return false;

	sensor_data->device_type = DEVICE_TYPE;
	sensor_data->register_layout_version = REGISTER_LAYOUT_VERSION;
	sensor_data->sensor_base_index = table_index * SENSOR_READING_PDR_INDEX_MAX;
	sensor_data->max_sbi_off = (num_idx > 0) ? num_idx - 1 : 0;
	LOG_DBG("sensor_base_index: %d, max_sbi_off: %d", sensor_data->sensor_base_index,
		sensor_data->max_sbi_off);
	for (int i = 0; i < num_idx; i++) {
		sensor_data->sensor_entries[i].sensor_index_offset =
			i; // sensor_index_offset range: 0~49
		sensor_data->sensor_entries[i].sensor_value = 0x00000000;
	}

	*buffer_size = (uint8_t)table_size;
	LOG_HEXDUMP_DBG(sensor_data, table_size, "sensor_data");
	return true;
}
void update_sensor_reading_by_sensor_number(uint8_t sensor_number)
{
	// sensor number is 1-base but index is 0-base
	uint8_t sensor_index_offset = sensor_number - 1;
	uint8_t table_index = sensor_index_offset / SENSOR_READING_PDR_INDEX_MAX;
	uint8_t sensor_index = sensor_index_offset % SENSOR_READING_PDR_INDEX_MAX;
	plat_sensor_reading *sensor_data = sensor_reading_table[table_index];

	uint8_t status = SENSOR_UNAVAILABLE;
	int reading = 0;
	uint8_t sensor_operational_state = PLDM_SENSOR_STATUSUNKOWN;

	status = pldm_sensor_get_reading_from_cache(sensor_number, &reading,
						    &sensor_operational_state);

	sensor_data->sensor_entries[sensor_index].sensor_value =
		(status == SENSOR_READ_SUCCESS) ? reading : 0xFFFFFFFF;
}

int get_cached_sensor_reading_by_sensor_number(uint8_t sensor_number)
{
	uint8_t sensor_index_offset = sensor_number - 1;
	uint8_t table_index = sensor_index_offset / SENSOR_READING_PDR_INDEX_MAX;
	uint8_t sensor_index = sensor_index_offset % SENSOR_READING_PDR_INDEX_MAX;
	plat_sensor_reading *sensor_data = sensor_reading_table[table_index];
	return sensor_data->sensor_entries[sensor_index].sensor_value;
}

float get_sensor_reading_cache_as_float(uint8_t sensor_number)
{
	int reading = get_cached_sensor_reading_by_sensor_number(sensor_number);
	const sensor_val *sval = (sensor_val *)&reading;
	return ((sval->integer * 1000 + sval->fraction) / 1000.0);
}

bool initialize_inventory_ids(telemetry_info *telemetry_info, uint8_t *buffer_size)
{
	CHECK_NULL_ARG_WITH_RETURN(telemetry_info, false);

	int table_index = telemetry_info->telemetry_offset - INVENTORY_IDS_REG;
	if (table_index < 0 || table_index >= DATA_TABLE_LENGTH_1)
		return false;

	size_t table_size = sizeof(plat_inventory_ids);
	plat_inventory_ids *sensor_data =
		allocate_table((void **)&inventory_ids_table[table_index], table_size);
	if (!sensor_data)
		return false;

	uint8_t data[4] = { 0 };
	uint32_t bic_version = 0;
	uint32_t cpld_version = 0;
	uint8_t board_id = get_asic_board_id();

	if (!plat_read_cpld(CPLD_VERSION_GET_REG, data, CPLD_VERSION_GET_REG_LEN))
		LOG_ERR("Failed to read cpld version from cpld");

	cpld_version = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
	bic_version =
		(BIC_FW_YEAR_MSB << 24) | (BIC_FW_YEAR_LSB << 16) | (BIC_FW_WEEK << 8) | BIC_FW_VER;

	sensor_data->carrier_board_id = board_id;
	sensor_data->bic_fw_version = bic_version;
	sensor_data->cpld_fw_version = cpld_version;

	*buffer_size = (uint8_t)table_size;
	return true;
}
bool initialize_strap_capability(telemetry_info *telemetry_info, uint8_t *buffer_size)
{
	CHECK_NULL_ARG_WITH_RETURN(telemetry_info, false);

	int table_index = telemetry_info->telemetry_offset - STRAP_CAPABILTITY_REG;
	if (table_index < 0 || table_index >= DATA_TABLE_LENGTH_1)
		return false;

	int num_idx = get_strap_index_max();

	size_t table_size = sizeof(plat_strap_capability) + num_idx * sizeof(strap_entry);
	plat_strap_capability *sensor_data =
		allocate_table((void **)&strap_capability_table[table_index], table_size);
	if (!sensor_data)
		return false;

	sensor_data->strap_data_length = get_strap_index_max() * 3; // strap_entry data length
	for (int i = 0; i < get_strap_index_max(); i++) {
		sensor_data->strap_set_format[i].strap_set_index = i;
		sensor_data->strap_set_format[i].strap_set_type = STRAP_SET_TYPE;
		int drive_level = 0;
		if (!get_bootstrap_change_drive_level(i, &drive_level)) {
			LOG_ERR("Can't get_bootstrap_change_drive_level by index: %x", i);
			continue;
		}
		if (drive_level < 0) {
			LOG_ERR("Invalid drive_level: %x", drive_level);
			continue;
		}
		sensor_data->strap_set_format[i].strap_set_value = drive_level;
	}

	*buffer_size = (uint8_t)table_size;
	return true;
}
void update_strap_capability_table()
{
	if (!strap_capability_table[0])
		return;

	plat_strap_capability *sensor_data = strap_capability_table[0];

	for (int i = 0; i < get_strap_index_max(); i++) {
		int drive_level = 0;
		if (!get_bootstrap_change_drive_level(i, &drive_level)) {
			LOG_ERR("Can't get_bootstrap_change_drive_level by index: %x", i);
			continue;
		}
		if (drive_level < 0) {
			LOG_ERR("Invalid drive_level: %x", drive_level);
			continue;
		}
		sensor_data->strap_set_format[i].strap_set_value = drive_level;
	}
}
void plat_pldm_sensor_poll_post()
{
	update_strap_capability_table();
}
void set_sensor_polling_handler(struct k_work *work_item)
{
	const plat_control_sensor_polling *sensor_data =
		CONTAINER_OF(work_item, plat_control_sensor_polling, work);

	int value = sensor_data->set_value;
	if (!(value == 0 || value == 1)) {
		LOG_ERR("set sensor_polling:%x is out of range", value);
		return;
	}
	set_plat_sensor_polling_enable_flag(value);
}

uint8_t vr_pwr_sensor_table[] = {
	SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_PWR_W,
	SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_PWR_W,
	SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_PWR_W,
	SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_PWR_W,
	SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_PWR_W,
	SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_PWR_W,
	SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_PWR_W,
	SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_PWR_W,
	SENSOR_NUM_ASIC_P1V1_VDDQC_HBM1357_PWR_W,
	SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_PWR_W,
	SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_PWR_W,
	SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_PWR_W,
	SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_PWR_W,
	SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_PWR_W,
	SENSOR_NUM_ASIC_P1V1_VDDQC_HBM0246_PWR_W,
	SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_PWR_W,
	SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_PWR_W,
	SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_PWR_W,
	SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_PWR_W,
	SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_PWR_W,
	SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_PWR_W,
	SENSOR_NUM_ASIC_P0V75_OWL_W_TRVDD_PWR_W,
	SENSOR_NUM_P3V3_OSFP_PWR_W,
};

void vr_power_reading(uint8_t *buffer, size_t buf_size)
{
	/*
	Response Data
	[0:1] - x = All VR power except MEDHA0_VDD and MEDHA1_VDD (Unit: W)
	[2:3] - Chiplet0 power = MEDHA0_VDD + 0.5*x (Unit: W)
	[4:5] - Chiplet1 power = MEDHA1_VDD + 0.5*x (Unit: W)
	[6:7] - P0V85_MEDHA0_VDD (Unit: W)
	[8:9] - P0V85_MEDHA1_VDD (Unit: W)
	[10:11] - P1V1_VDDQC_HBM0246 (Unit: W)
	[12:13] - P0V75_VDDPHY_HBM0246 (Unit: W)
	[14:15] - P1V1_VDDQC_HBM1357 (Unit: W)
	[16:17] - P0V75_VDDPHY_HBM1357 (Unit: W)
	[18:19] - P0V75_MAX_N_VDD (Unit: W)
	[20:21] - P0V75_MAX_S_VDD (Unit: W)
	[22:23] - P0V4_VDDQL_HBM0246 (Unit: W)
	[24:25] - P0V4_VDDQL_HBM1357 (Unit: W)
	[26:27] - P1V8_VPP_HBM0246 (Unit: W)
	[28:29] - P1V8_VPP_HBM1357 (Unit: W)
	[30:31] - P0V75_MAX_M_VDD (Unit: W)
	[32:33] - P0V75_OWL_E_VDD (Unit: W)
	[34:35] - P0V75_OWL_W_VDD (Unit: W)
	[36:37] - PDB1_P52V_ASIC_SENSE_PWR (Unit: W) (Need BMC support)
	each data is 2 bytes
	*/
	float x = 0;
	float chiplet0 = 0;
	float chiplet1 = 0;
	float medha0 = 0;
	float medha1 = 0;

	for (size_t i = 0; i < ARRAY_SIZE(vr_pwr_sensor_table); i++) {
		if (get_asic_board_id() != ASIC_BOARD_ID_EVB &&
		    vr_pwr_sensor_table[i] == SENSOR_NUM_P3V3_OSFP_PWR_W) {
			LOG_WRN("Skip sensor 0x%x", vr_pwr_sensor_table[i]);
			continue;
		}

		uint8_t status = SENSOR_UNAVAILABLE;
		int reading = 0;
		uint8_t sensor_operational_state = PLDM_SENSOR_STATUSUNKOWN;

		status = pldm_sensor_get_reading_from_cache(vr_pwr_sensor_table[i], &reading,
							    &sensor_operational_state);
		// reading value unit is mW need to convert to W
		reading = (reading + 500) / 1000;
		uint16_t val = (uint16_t)reading;
		switch (vr_pwr_sensor_table[i]) {
		case SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_PWR_W:
			memcpy(&buffer[6], &val, 2);
			medha0 = reading;
			break;
		case SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_PWR_W:
			memcpy(&buffer[8], &val, 2);
			medha1 = reading;
			break;
		case SENSOR_NUM_ASIC_P1V1_VDDQC_HBM0246_PWR_W:
			memcpy(&buffer[10], &val, 2);
			break;
		case SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_PWR_W:
			memcpy(&buffer[12], &val, 2);
			break;
		case SENSOR_NUM_ASIC_P1V1_VDDQC_HBM1357_PWR_W:
			memcpy(&buffer[14], &val, 2);
			break;
		case SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_PWR_W:
			memcpy(&buffer[16], &val, 2);
			break;
		case SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_PWR_W:
			memcpy(&buffer[18], &val, 2);
			break;
		case SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_PWR_W:
			memcpy(&buffer[20], &val, 2);
			break;
		case SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_PWR_W:
			memcpy(&buffer[22], &val, 2);
			break;
		case SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_PWR_W:
			memcpy(&buffer[24], &val, 2);
			break;
		case SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_PWR_W:
			memcpy(&buffer[26], &val, 2);
			break;
		case SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_PWR_W:
			memcpy(&buffer[28], &val, 2);
			break;
		case SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_PWR_W:
			memcpy(&buffer[30], &val, 2);
			break;
		case SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_PWR_W:
			memcpy(&buffer[32], &val, 2);
			break;
		case SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_PWR_W:
			memcpy(&buffer[34], &val, 2);
			break;
		default:
			// do nothing
			break;
		}
		LOG_DBG("Sensor 0x%x: status = %d, reading = 0x%x", vr_pwr_sensor_table[i], status,
			reading);

		if (vr_pwr_sensor_table[i] != SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_PWR_W &&
		    vr_pwr_sensor_table[i] != SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_PWR_W) {
			x += reading;
		}
	}
	int reading;
	get_cpld_polling_power_info(&reading);
	reading = (reading + 500) / 1000;
	uint16_t val = (uint16_t)reading;
	memcpy(&buffer[36], &val, 2);
	x += reading;

	chiplet0 = medha0 + 0.5 * x;
	chiplet1 = medha1 + 0.5 * x;
	uint16_t val_x = (uint16_t)x;
	uint16_t val_c0 = (uint16_t)chiplet0;
	uint16_t val_c1 = (uint16_t)chiplet1;
	memcpy(&buffer[0], &val_x, 2);
	memcpy(&buffer[2], &val_c0, 2);
	memcpy(&buffer[4], &val_c1, 2);

	LOG_HEXDUMP_DBG(buffer, buf_size, "VR power sensor data buffer");
}
/*
Request Data
[0] - VR controller
	0 - MEDHA0
	1 - MEDHA1
[1] - alert level
	0 - Level 1
	1 - Level 2
	2 - Level 3
[2] - time window. Unit: us for Level 1, ms for Level 2/3  (LSB, write only)
[3] - time window. Unit: us for Level 1, ms for Level 2/3  (MSB, write only)

*/
vr_controller_t vr_alert_all[VR_PWR_CONTROLLER_MAX] = { 0 };
;

void set_vr_pwr_alert_data(uint8_t controller_id, uint8_t alert_level, uint8_t data_type,
			   uint8_t write_data_lsb, uint8_t write_data_msb)
{
	if (alert_level >= VR_ALERT_MAX) {
		LOG_ERR("Invalid alert level %d, when set ", alert_level);
		return;
	}

	if (data_type >= VR_INFO_TYPE_MAX) {
		LOG_ERR("Invalid data type %d, when set", data_type);
		return;
	}
	switch (data_type) {
	case VR_THRESHOLD:
		vr_alert_all[controller_id].level[alert_level].threshold_lsb = write_data_lsb;
		vr_alert_all[controller_id].level[alert_level].threshold_msb = write_data_msb;
		break;
	case VR_TIME_WINDOW:
		vr_alert_all[controller_id].level[alert_level].time_window_lsb = write_data_lsb;
		vr_alert_all[controller_id].level[alert_level].time_window_msb = write_data_msb;
		break;
	default:
		break;
	}

	LOG_DBG("controller_id %d, alert_level %d, write_data_lsb %d, write_data_msb %d",
		controller_id, alert_level, write_data_lsb, write_data_msb);
	LOG_HEXDUMP_DBG(vr_alert_all, sizeof(vr_alert_all), "vr_alert_all");
};
void get_vr_pwr_alert_data(uint8_t *buffer, size_t buf_size, uint8_t data_type, uint8_t alert_level,
			   uint8_t controller_id)
{
	if (alert_level >= VR_ALERT_MAX) {
		LOG_ERR("Invalid alert level %d", alert_level);
		return;
	}

	if (data_type >= VR_INFO_TYPE_MAX) {
		LOG_ERR("Invalid data type %d", data_type);
		return;
	}

	if (data_type == VR_THRESHOLD) {
		switch (controller_id) {
		/*
			Response Data
			[0] - time window. Unit: us for Level 1, ms for Level 2/3  (LSB, read only)
			[1] - time window. Unit: us for Level 1, ms for Level 2/3  (MSB, read only)
		*/
		case MEDHA0:
			// update read data from write data
			vr_alert_all[MEDHA0].level[alert_level].read_data_msb =
				vr_alert_all[MEDHA0].level[alert_level].threshold_msb;
			vr_alert_all[MEDHA0].level[alert_level].read_data_lsb =
				vr_alert_all[MEDHA0].level[alert_level].threshold_lsb;
			// copy data to buffer
			buffer[0] = vr_alert_all[MEDHA0].level[alert_level].read_data_lsb;
			buffer[1] = vr_alert_all[MEDHA0].level[alert_level].read_data_msb;

			break;
		case MEDHA1:
			// update read data from write data
			vr_alert_all[MEDHA1].level[alert_level].read_data_msb =
				vr_alert_all[MEDHA1].level[alert_level].threshold_msb;
			vr_alert_all[MEDHA1].level[alert_level].read_data_lsb =
				vr_alert_all[MEDHA1].level[alert_level].threshold_lsb;
			// copy data to buffer
			buffer[0] = vr_alert_all[MEDHA1].level[alert_level].read_data_lsb;
			buffer[1] = vr_alert_all[MEDHA1].level[alert_level].read_data_msb;
			break;
		default:
			LOG_ERR("Invalid controller id %d", controller_id);
			break;
		}
	} else if (data_type == VR_TIME_WINDOW) {
		switch (controller_id) {
		case MEDHA0:
			// update read data from write data
			vr_alert_all[MEDHA0].level[alert_level].read_data_msb =
				vr_alert_all[MEDHA0].level[alert_level].time_window_msb;
			vr_alert_all[MEDHA0].level[alert_level].read_data_lsb =
				vr_alert_all[MEDHA0].level[alert_level].time_window_lsb;
			// // copy data to buffer
			buffer[0] = vr_alert_all[MEDHA0].level[alert_level].read_data_lsb;
			buffer[1] = vr_alert_all[MEDHA0].level[alert_level].read_data_msb;
			break;
		case MEDHA1:
			// update read data from write data
			vr_alert_all[MEDHA1].level[alert_level].read_data_msb =
				vr_alert_all[MEDHA1].level[alert_level].time_window_msb;
			vr_alert_all[MEDHA1].level[alert_level].read_data_lsb =
				vr_alert_all[MEDHA1].level[alert_level].time_window_lsb;

			// copy data to buffer
			buffer[0] = vr_alert_all[MEDHA1].level[alert_level].read_data_lsb;
			buffer[1] = vr_alert_all[MEDHA1].level[alert_level].read_data_msb;
			break;
		default:
			LOG_ERR("Invalid controller id %d", controller_id);
			break;
		}
	} else
		LOG_ERR("Invalid data type %d", data_type);
	LOG_DBG("controller_id %d, alert_level %d, data_type %d, read_data_lsb %d, read_data_msb %d",
		controller_id, alert_level, data_type, buffer[0], buffer[1]);
	LOG_HEXDUMP_DBG(vr_alert_all, sizeof(vr_alert_all), "vr_alert_all");
};

telemetry_info telemetry_info_table[] = {
	{ SENSOR_INIT_DATA_0_REG, 0x00, .telemetry_table_init = initialize_sensor_data },
	{ SENSOR_INIT_DATA_1_REG, 0x00, .telemetry_table_init = initialize_sensor_data },
	{ SENSOR_READING_0_REG, 0x00, .telemetry_table_init = initialize_sensor_reading },
	{ SENSOR_READING_1_REG, 0x00, .telemetry_table_init = initialize_sensor_reading },
	{ SENSOR_READING_2_REG, 0x00, .telemetry_table_init = initialize_sensor_reading },
	{ SENSOR_READING_3_REG, 0x00, .telemetry_table_init = initialize_sensor_reading },
	{ INVENTORY_IDS_REG, 0x00, .telemetry_table_init = initialize_inventory_ids },
	{ STRAP_CAPABILTITY_REG, 0x00, .telemetry_table_init = initialize_strap_capability },
	{ WRITE_STRAP_PIN_VALUE_REG },
	{ I2C_BRIDGE_COMMAND_REG },
	{ I2C_BRIDGE_COMMAND_STATUS_REG },
	{ I2C_BRIDGE_COMMAND_RESPONSE_REG },
	{ LEVEL_1_2_3_PWR_ALERT_THRESHOLD_REG },
	{ LEVEL_1_2_3_PWR_ALERT_TIME_WINDOW_REG },
	{ VR_POWER_READING_REG },
};

static bool command_reply_data_handle(void *arg)
{
	struct i2c_target_data *data = (struct i2c_target_data *)arg;
	if (data->wr_buffer_idx >= 1) {
		if (data->wr_buffer_idx == 1) {
			uint8_t reg_offset = data->target_wr_msg.msg[0];
			size_t struct_size = 0;
			for (int i = 0; i < ARRAY_SIZE(telemetry_info_table); i++) {
				if (telemetry_info_table[i].telemetry_offset == reg_offset) {
					struct_size = telemetry_info_table[i].data_size;
					break;
				}
			}
			// Make sure the target buffer is not exceeded when reading
			if (struct_size > sizeof(data->target_rd_msg.msg)) {
				struct_size = sizeof(data->target_rd_msg.msg);
			}
			LOG_DBG("Received reg offset(write 1 data): 0x%02x", reg_offset);
			switch (reg_offset) {
			case SENSOR_INIT_DATA_0_REG:
			case SENSOR_INIT_DATA_1_REG: {
				data->target_rd_msg.msg_length = struct_size;
				memcpy(data->target_rd_msg.msg,
				       sensor_init_data_table[reg_offset - SENSOR_INIT_DATA_0_REG],
				       struct_size);
				LOG_HEXDUMP_DBG(data->target_rd_msg.msg,
						data->target_rd_msg.msg_length, "sensor init data");
			} break;
			case SENSOR_READING_0_REG:
			case SENSOR_READING_1_REG:
			case SENSOR_READING_2_REG:
			case SENSOR_READING_3_REG: {
				data->target_rd_msg.msg_length = struct_size;
				memcpy(data->target_rd_msg.msg,
				       sensor_reading_table[reg_offset - SENSOR_READING_0_REG],
				       struct_size);
				LOG_HEXDUMP_DBG(data->target_rd_msg.msg,
						data->target_rd_msg.msg_length, "sensor reading");
			} break;
			case INVENTORY_IDS_REG: {
				data->target_rd_msg.msg_length = struct_size;
				memcpy(data->target_rd_msg.msg,
				       inventory_ids_table[reg_offset - INVENTORY_IDS_REG],
				       struct_size);
				LOG_HEXDUMP_DBG(data->target_rd_msg.msg,
						data->target_rd_msg.msg_length, "inventory ids");
			} break;
			case STRAP_CAPABILTITY_REG: {
				data->target_rd_msg.msg_length = struct_size;
				memcpy(data->target_rd_msg.msg,
				       strap_capability_table[reg_offset - STRAP_CAPABILTITY_REG],
				       struct_size);
				LOG_HEXDUMP_DBG(data->target_rd_msg.msg,
						data->target_rd_msg.msg_length, "strap capability");
			} break;
			case I2C_BRIDGE_COMMAND_STATUS_REG: {
				data->target_rd_msg.msg_length = 1;
				data->target_rd_msg.msg[0] =
					i2c_bridge_command_status_table[0] ?
						i2c_bridge_command_status_table[0]->data_status :
						I2C_BRIDGE_COMMAND_FAILURE;
			} break;
			case I2C_BRIDGE_COMMAND_RESPONSE_REG: {
				if (i2c_bridge_command_response_data_table[0]) {
					struct_size = i2c_bridge_command_response_data_table[0]
							      ->data_length +
						      1;
					data->target_rd_msg.msg_length = struct_size;
					memcpy(data->target_rd_msg.msg,
					       i2c_bridge_command_response_data_table[0],
					       struct_size);
				} else {
					data->target_rd_msg.msg_length = 1;
					data->target_rd_msg.msg[0] = 0xFF;
				}
				LOG_HEXDUMP_DBG(data->target_rd_msg.msg,
						data->target_rd_msg.msg_length,
						"i2c bridge command response");
			} break;
			case CONTROL_VOL_VR_ASIC_P0V75_VDDPHY_HBM0246_REG:
			case CONTROL_VOL_VR_ASIC_P0V75_VDDPHY_HBM1357_REG:
			case CONTROL_VOL_VR_ASIC_P1V1_VDDQC_HBM0246_REG:
			case CONTROL_VOL_VR_ASIC_P1V1_VDDQC_HBM1357_REG:
			case CONTROL_VOL_VR_ASIC_P0V4_VDDQL_HBM0246_REG:
			case CONTROL_VOL_VR_ASIC_P0V4_VDDQL_HBM1357_REG:
			case CONTROL_VOL_VR_ASIC_P1V8_VPP_HBM0246_REG:
			case CONTROL_VOL_VR_ASIC_P1V8_VPP_HBM1357_REG:
			case CONTROL_VOL_VR_ASIC_P0V85_MEDHA0_VDD_REG:
			case CONTROL_VOL_VR_ASIC_P0V85_MEDHA1_VDD_REG: {
				uint8_t rail = get_vr_rail_by_control_vol_reg(reg_offset);
				uint16_t vout = 0xFFFF;
				if (!voltage_command_setting_get(rail, &vout)) {
					LOG_ERR("Can't voltage_command setting_get by rail: 0x%02x",
						rail);
				}
				memcpy(data->target_rd_msg.msg, &vout, sizeof(vout));
				data->target_rd_msg.msg_length = 2;
			} break;
			case VR_POWER_READING_REG: {
				data->target_rd_msg.msg_length = VR_PWR_BUF_SIZE;
				vr_power_reading(data->target_rd_msg.msg,
						 data->target_rd_msg.msg_length);
				LOG_HEXDUMP_DBG(data->target_rd_msg.msg,
						data->target_rd_msg.msg_length, "vr power reading");
			} break;
			case TRAY_INFO_REG: {
				/* TRAY_INFO_REG:
				 * Byte0: MMC slot
				 * Byte1: tray location
				 */
				uint8_t slot = get_mmc_slot();
				uint8_t tray = get_tray_location();

				data->target_rd_msg.msg[0] = slot;
				data->target_rd_msg.msg[1] = tray;
				data->target_rd_msg.msg_length = 2;

				LOG_DBG("TRAY_INFO_REG: slot=0x%02x, tray=0x%02x", slot, tray);
				LOG_HEXDUMP_DBG(data->target_rd_msg.msg,
						data->target_rd_msg.msg_length, "tray info");
			} break;
			default:
				LOG_ERR("Unknown reg offset: 0x%02x", reg_offset);
				data->target_rd_msg.msg_length = 1;
				data->target_rd_msg.msg[0] = 0xFF;
				break;
			}
		} else if (data->wr_buffer_idx == 2) {
			LOG_DBG("Received reg offset(write 2 data): 0x%02x",
				data->target_wr_msg.msg[0]);
			uint8_t reg_offset = data->target_wr_msg.msg[0];
			switch (reg_offset) {
			case WRITE_STRAP_PIN_VALUE_REG: {
				int rail = data->target_wr_msg.msg[1];
				int drive_level = -1;
				if (!get_bootstrap_change_drive_level(rail, &drive_level)) {
					LOG_ERR("Can't get_bootstrap_change_drive_level by rail index: %x",
						rail);
					data->target_rd_msg.msg[0] = 0xFF;
				} else {
					data->target_rd_msg.msg[0] = drive_level;
				}
				data->target_rd_msg.msg_length = 1;
				LOG_HEXDUMP_DBG(data->target_rd_msg.msg,
						data->target_rd_msg.msg_length, "strap pin value");
			} break;
			default:
				LOG_ERR("Unknown reg offset: 0x%02x", reg_offset);
				data->target_rd_msg.msg_length = 1;
				data->target_rd_msg.msg[0] = 0xFF;
				break;
			}
		} else if (data->wr_buffer_idx == 3) {
			LOG_DBG("Received reg offset(write 3 data): 0x%02x",
				data->target_wr_msg.msg[0]);
			uint8_t reg_offset = data->target_wr_msg.msg[0];
			switch (reg_offset) {
			case LEVEL_1_2_3_PWR_ALERT_THRESHOLD_REG: {
				data->target_rd_msg.msg_length = 2;
				uint8_t vr_controller = data->target_wr_msg.msg[1];
				uint8_t alert_level = data->target_wr_msg.msg[2];
				get_vr_pwr_alert_data(data->target_rd_msg.msg,
						      data->target_rd_msg.msg_length, VR_THRESHOLD,
						      alert_level, vr_controller);
				LOG_DBG("vr controller: %d, alert level: %d", vr_controller,
					alert_level);
				LOG_HEXDUMP_DBG(data->target_rd_msg.msg,
						data->target_rd_msg.msg_length, "vr threshold");
			} break;
			case LEVEL_1_2_3_PWR_ALERT_TIME_WINDOW_REG: {
				data->target_rd_msg.msg_length = 2;
				uint8_t vr_controller = data->target_wr_msg.msg[1];
				uint8_t alert_level = data->target_wr_msg.msg[2];
				get_vr_pwr_alert_data(data->target_rd_msg.msg,
						      data->target_rd_msg.msg_length,
						      VR_TIME_WINDOW, alert_level, vr_controller);
				LOG_DBG("vr controller: %d, alert level: %d", vr_controller,
					alert_level);
				LOG_HEXDUMP_DBG(data->target_rd_msg.msg,
						data->target_rd_msg.msg_length, "vr time window");
			} break;
			default:
				LOG_ERR("Unknown reg offset: 0x%02x", reg_offset);
				data->target_rd_msg.msg_length = 1;
				data->target_rd_msg.msg[0] = 0xFF;
				break;
			}
		} else {
			LOG_ERR("Received data length: 0x%02x", data->wr_buffer_idx);
			data->target_rd_msg.msg_length = 1;
			data->target_rd_msg.msg[0] = 0xFF;
		}
	}
	LOG_DBG("Reply data length: 0x%02x", data->target_rd_msg.msg_length);
	return true;
}

/* I2C target init-config table */
const struct _i2c_target_config I2C_TARGET_CONFIG_TABLE[MAX_TARGET_NUM] = {
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0x42, 0xA },
	{ 0x40, 0xA, command_reply_data_handle },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
};

static uint8_t bootstrap_pin;
static uint8_t user_setting_level;
bool set_bootstrap_element(uint8_t bootstrap_pin, uint8_t user_setting_level)
{
	uint8_t change_setting_value;
	uint8_t drive_index_level = user_setting_level;
	bootstrap_mapping_register bootstrap_item;
	if (!set_bootstrap_table_and_user_settings(bootstrap_pin, &change_setting_value,
						   drive_index_level, false, false)) {
		LOG_ERR("set bootstrap_table[%02x]:%d failed", bootstrap_pin, drive_index_level);
		return false;
	}
	if (!find_bootstrap_by_rail(bootstrap_pin, &bootstrap_item)) {
		LOG_ERR("Can't find bootstrap_item by bootstrap_pin index: 0x%02x", bootstrap_pin);
		return false;
	}
	// LOG_DBG("set bootstrap_table[%2x]=%x, cpld_offsets 0x%02x change_setting_value 0x%02x",
	// 	bootstrap_pin, drive_index_level, bootstrap_item.cpld_offsets,
	// 	change_setting_value);
	if (!set_bootstrap_val_to_device(bootstrap_pin, change_setting_value))
		LOG_ERR("Can't write bootstrap[%2d]=%02x", bootstrap_pin, change_setting_value);

	return true;
}
void set_bootstrap_element_handler()
{
	if (bootstrap_pin >= get_strap_index_max()) {
		LOG_ERR("bootstrap_pin[%02x] is out of range", bootstrap_pin);
		return;
	}
	if (!set_bootstrap_element(bootstrap_pin, user_setting_level)) {
		LOG_ERR("set_bootstrap_element fail");
		return;
	}
}
uint8_t vr_pwr_alert_table[] = {
	SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_PWR_W,	  SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_PWR_W,
	SENSOR_NUM_ASIC_P1V1_VDDQC_HBM0246_PWR_W, SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_PWR_W,
	SENSOR_NUM_ASIC_P1V1_VDDQC_HBM1357_PWR_W, SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_PWR_W,
	SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_PWR_W,	  SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_PWR_W,
};

void i2c_bridge_command_handler(struct k_work *work_item)
{
	const plat_i2c_bridge_command_config *sensor_data_config =
		CONTAINER_OF(work_item, plat_i2c_bridge_command_config, work);

	int response_data_len = sensor_data_config->read_len;

	size_t table_size_41 = sizeof(plat_i2c_bridge_command_status);
	plat_i2c_bridge_command_status *sensor_data_status =
		allocate_table((void **)&i2c_bridge_command_status_table[0], table_size_41);
	if (!sensor_data_status)
		return;

	size_t table_size_42 =
		sizeof(plat_i2c_bridge_command_response_data) + response_data_len * sizeof(uint8_t);
	plat_i2c_bridge_command_response_data *sensor_data_response =
		allocate_table((void **)&i2c_bridge_command_response_data_table[0], table_size_42);
	if (!sensor_data_response)
		return;

	sensor_data_status->data_status = I2C_BRIDGE_COMMAND_IN_PROCESS;
	sensor_data_response->data_length = 0x00;

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = sensor_data_config->bus;
	i2c_msg.target_addr = sensor_data_config->addr;
	i2c_msg.tx_len = sensor_data_config->write_len;
	i2c_msg.rx_len = sensor_data_config->read_len;
	memcpy(&i2c_msg.data, sensor_data_config->data, sensor_data_config->write_len);
	if (response_data_len == 0) {
		if (i2c_master_write(&i2c_msg, retry)) {
			LOG_ERR("Failed to write reg, bus: %d, addr: 0x%x, tx_len: 0x%x",
				i2c_msg.bus, i2c_msg.target_addr, i2c_msg.tx_len);
			sensor_data_status->data_status = I2C_BRIDGE_COMMAND_FAILURE;
			return;
		}
		sensor_data_status->data_status = I2C_BRIDGE_COMMAND_SUCCESS;
		sensor_data_response->data_length = response_data_len;
	} else {
		if (i2c_master_read(&i2c_msg, retry)) {
			LOG_ERR("Failed to read reg, bus: %d, addr: 0x%x, tx_len: 0x%x",
				i2c_msg.bus, i2c_msg.target_addr, i2c_msg.tx_len);
			sensor_data_status->data_status = I2C_BRIDGE_COMMAND_FAILURE;
			return;
		}
		sensor_data_status->data_status = I2C_BRIDGE_COMMAND_SUCCESS;
		sensor_data_response->data_length = response_data_len;
		memcpy(sensor_data_response->response_data, i2c_msg.data, response_data_len);
	}
}
void set_control_voltage_handler(struct k_work *work_item)
{
	const plat_control_voltage *sensor_data =
		CONTAINER_OF(work_item, plat_control_voltage, work);
	uint8_t rail = sensor_data->rail;
	uint16_t millivolt = sensor_data->set_value;
	LOG_DBG("Setting rail %x to %d mV", rail, millivolt);

	plat_set_vout_command(rail, &millivolt, false, false);
}
void plat_master_write_thread_handler()
{
	int rc = 0;
	while (1) {
		uint8_t rdata[MAX_I2C_TARGET_BUFF] = { 0 };
		uint16_t rlen = 0;
		rc = i2c_target_read(I2C_TARGET_BUS_ASIC, rdata, sizeof(rdata), &rlen);
		if (rc) {
			LOG_ERR("i2c_target_read fail, ret %d", rc);
			continue;
		}
		// LOG_DBG("rlen = %d", rlen);
		// LOG_HEXDUMP_DBG(rdata, rlen, "");
		if (rlen < 1) {
			LOG_ERR("Received data too short");
			continue;
		}

		uint8_t reg_offset = rdata[0];
		LOG_DBG("Received reg offset[0]: 0x%02x", rdata[0]);
		LOG_DBG("Received reg offset[1]: 0x%02x", rdata[1]);
		LOG_DBG("Received reg offset[2]: 0x%02x", rdata[2]);
		switch (reg_offset) {
		case WRITE_STRAP_PIN_VALUE_REG: {
			if (rlen != 3) {
				LOG_WRN("WRITE_STRAP_PIN_VALUE_REG Invalid length for offset(write): 0x%02x",
					reg_offset);
				LOG_DBG("Received data length: 0x%02x", rlen);
				break;
			}
			bootstrap_pin = rdata[1];
			user_setting_level = rdata[2];
			k_work_submit(&set_bootstrap_element_work);
		} break;
		case I2C_BRIDGE_COMMAND_REG: {
			if (rlen < 5) {
				LOG_ERR("Invalid length for offset: 0x%02x", reg_offset);
				break;
			}
			size_t payload_len = rlen - 4;
			size_t struct_size = sizeof(plat_i2c_bridge_command_config) + payload_len;
			plat_i2c_bridge_command_config *sensor_data = malloc(struct_size);
			if (!sensor_data) {
				LOG_ERR("Memory allocation failed!");
				break;
			}
			sensor_data->bus = rdata[1];
			sensor_data->addr = rdata[2];
			sensor_data->read_len = rdata[3];
			sensor_data->write_len = payload_len;
			memcpy(sensor_data->data, &rdata[4], payload_len);
			k_work_init(&sensor_data->work, i2c_bridge_command_handler);
			k_work_submit(&sensor_data->work);
			LOG_HEXDUMP_DBG(rdata, rlen, "I2C_BRIDGE_COMMAND_REG");
		} break;
		case CONTROL_VOL_VR_ASIC_P0V75_VDDPHY_HBM0246_REG:
		case CONTROL_VOL_VR_ASIC_P0V75_VDDPHY_HBM1357_REG:
		case CONTROL_VOL_VR_ASIC_P1V1_VDDQC_HBM0246_REG:
		case CONTROL_VOL_VR_ASIC_P1V1_VDDQC_HBM1357_REG:
		case CONTROL_VOL_VR_ASIC_P0V4_VDDQL_HBM0246_REG:
		case CONTROL_VOL_VR_ASIC_P0V4_VDDQL_HBM1357_REG:
		case CONTROL_VOL_VR_ASIC_P1V8_VPP_HBM0246_REG:
		case CONTROL_VOL_VR_ASIC_P1V8_VPP_HBM1357_REG:
		case CONTROL_VOL_VR_ASIC_P0V85_MEDHA0_VDD_REG:
		case CONTROL_VOL_VR_ASIC_P0V85_MEDHA1_VDD_REG: {
			if (rlen != 3) {
				LOG_ERR("Invalid length for offset(write): 0x%02x", reg_offset);
				break;
			}

			plat_control_voltage *sensor_data = malloc(sizeof(plat_control_voltage));
			if (!sensor_data) {
				LOG_ERR("Memory allocation failed!");
				break;
			}
			sensor_data->rail = get_vr_rail_by_control_vol_reg(reg_offset);
			sensor_data->set_value = rdata[1] | (rdata[2] << 8);
			k_work_init(&sensor_data->work, set_control_voltage_handler);
			k_work_submit(&sensor_data->work);
		} break;
		case SET_SENSOR_POLLING_COMMAND_REG: {
			if (rlen != 8) {
				LOG_ERR("Invalid length for offset: 0x%02x", reg_offset);
				LOG_ERR("length: 0x%02x", rlen);
				break;
			}
			uint8_t expected_signature[] = {
				0x4D, 0x54, 0x49, 0x41, 0x56, 0x31
			}; // ascii to hex: "MTIAV1"
			if (memcmp(&rdata[1], expected_signature, sizeof(expected_signature)) !=
			    0) {
				LOG_HEXDUMP_DBG(rdata, rlen, "rdata:");
				LOG_ERR("Wrong command for set sensor_polling");
				break;
			}
			plat_control_sensor_polling *sensor_data =
				malloc(sizeof(plat_control_sensor_polling));
			if (!sensor_data) {
				LOG_ERR("Memory allocation failed!");
				break;
			}
			sensor_data->set_value = rdata[9]; // need to check
			LOG_DBG("set sensor_polling:%x", sensor_data->set_value);
			// transfer rdata[9] type to bool
			sensor_data->set_value = sensor_data->set_value ? true : false;
			k_work_init(&sensor_data->work, set_sensor_polling_handler);
			k_work_submit(&sensor_data->work);
		} break;
		/*
		Request Data
		[0] - VR controller
			0 - MEDHA0
			1 - MEDHA1
		[1] - alert level
			0 - Level 1
			1 - Level 2
			2 - Level 3
		[2] - alert threshold. Unit: W  (LSB, write only)
		[3] - alert threshold. Unit: W  (MSB, write only)
		*/
		case LEVEL_1_2_3_PWR_ALERT_THRESHOLD_REG: {
			if (rlen != 5) {
				LOG_ERR("Invalid length for offset(write threshold): 0x%02x",
					reg_offset);
				break;
			}
			uint8_t vr_controller = rdata[1];
			uint8_t alert_level = rdata[2];
			uint8_t lsb = rdata[3];
			uint8_t msb = rdata[4];
			set_vr_pwr_alert_data(vr_controller, alert_level, VR_THRESHOLD, lsb, msb);
		} break;
		case LEVEL_1_2_3_PWR_ALERT_TIME_WINDOW_REG: {
			if (rlen != 5) {
				LOG_ERR("Invalid length for offset(write time window): 0x%02x",
					reg_offset);
				break;
			}
			uint8_t vr_controller = rdata[1];
			uint8_t alert_level = rdata[2];
			uint8_t lsb = rdata[3];
			uint8_t msb = rdata[4];
			set_vr_pwr_alert_data(vr_controller, alert_level, VR_TIME_WINDOW, lsb, msb);
		} break;
		default:
			LOG_WRN("Unknown reg offset(write): 0x%02x", reg_offset);
			break;
		}
	}
}
void plat_master_write_thread_init()
{
	plat_master_write_tid = k_thread_create(&plat_master_write_thread, plat_master_write_stack,
						K_THREAD_STACK_SIZEOF(plat_master_write_stack),
						plat_master_write_thread_handler, NULL, NULL, NULL,
						CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&plat_master_write_thread, "plat_master_write_thread");
}

void plat_telemetry_table_init(void)
{
	uint8_t buffer_size = 0;
	for (int i = 0; i < ARRAY_SIZE(telemetry_info_table); i++) {
		if (telemetry_info_table[i].telemetry_table_init) {
			bool success = telemetry_info_table[i].telemetry_table_init(
				&telemetry_info_table[i], &buffer_size);
			if (!success) {
				LOG_ERR("initialize sensor data at offset 0x%02X",
					telemetry_info_table[i].telemetry_offset);
			}
			telemetry_info_table[i].data_size = buffer_size;
		}
	}
	LOG_INF("plat_telemetry_table_init done");
	plat_master_write_thread_init();
}
