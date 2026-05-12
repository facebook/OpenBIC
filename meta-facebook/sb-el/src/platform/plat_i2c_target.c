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
#include <logging/log.h>
#include "libutil.h"
#include "pldm_sensor.h"
#include "plat_version.h"
#include "plat_class.h"
#include "plat_cpld.h"
#include "plat_hook.h"
#include "plat_fru.h"
#include "plat_pldm_sensor.h"
#include "plat_i2c_target.h"
#include "plat_power_capping.h"
#include "plat_adc.h"

LOG_MODULE_REGISTER(plat_i2c_target);

/* telemetry */
#define MAX_TARGET_TABLE_NUM 12
#define DATA_TABLE_LENGTH_1 1
#define DATA_TABLE_LENGTH_2 2
#define DATA_TABLE_LENGTH_4 4
#define DATA_TABLE_LENGTH_7 7
#define DATA_TABLE_LENGTH_13 13
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
#define I2C_TARGET_BUS_ASIC I2C_BUS7 // asic HAMSA
#define I2C_TARGET_BUS_ASIC_NUWA0 I2C_BUS4 // asic medha0
#define I2C_TARGET_BUS_ASIC_NUWA1 I2C_BUS5 // asic medha1

plat_sensor_init_data *sensor_init_data_table[DATA_TABLE_LENGTH_2] = { NULL };
plat_sensor_reading *sensor_reading_table[DATA_TABLE_LENGTH_4] = { NULL };
plat_inventory_ids *inventory_ids_table[DATA_TABLE_LENGTH_1] = { NULL };
plat_strap_capability *strap_capability_table[DATA_TABLE_LENGTH_1] = { NULL };
plat_fru_data *fru_board_data_table[DATA_TABLE_LENGTH_13] = { NULL };
plat_fru_data *fru_product_data_table[DATA_TABLE_LENGTH_7] = { NULL };
plat_i2c_bridge_command_status *i2c_bridge_command_status_table[DATA_TABLE_LENGTH_1] = { NULL };
plat_i2c_bridge_command_response_data
	*i2c_bridge_command_response_data_table[DATA_TABLE_LENGTH_1] = { NULL };

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

bool get_fru_info_element(telemetry_info *telemetry_info, char **fru_element,
			  uint8_t *fru_element_size)
{
	CHECK_NULL_ARG_WITH_RETURN(telemetry_info, false);

	FRU_INFO *plat_fru_info = get_fru_info();
	if (!plat_fru_info)
		return false;

	switch (telemetry_info->telemetry_offset) {
	case FRU_BOARD_PART_NUMBER_REG:
		*fru_element = plat_fru_info->board.board_part_number;
		break;
	case FRU_BOARD_SERIAL_NUMBER_REG:
		*fru_element = plat_fru_info->board.board_serial;
		break;
	case FRU_BOARD_PRODUCT_NAME_REG:
		*fru_element = plat_fru_info->board.board_product;
		break;
	case FRU_BOARD_CUSTOM_DATA_1_REG:
		*fru_element = plat_fru_info->board.board_custom_data[0];
		break;
	case FRU_BOARD_CUSTOM_DATA_2_REG:
		*fru_element = plat_fru_info->board.board_custom_data[1];
		break;
	case FRU_BOARD_CUSTOM_DATA_3_REG:
		*fru_element = plat_fru_info->board.board_custom_data[2];
		break;
	case FRU_BOARD_CUSTOM_DATA_4_REG:
		*fru_element = plat_fru_info->board.board_custom_data[3];
		break;
	case FRU_BOARD_CUSTOM_DATA_5_REG:
		*fru_element = plat_fru_info->board.board_custom_data[4];
		break;
	case FRU_BOARD_CUSTOM_DATA_6_REG:
		*fru_element = plat_fru_info->board.board_custom_data[5];
		break;
	case FRU_BOARD_CUSTOM_DATA_7_REG:
		*fru_element = plat_fru_info->board.board_custom_data[6];
		break;
	case FRU_BOARD_CUSTOM_DATA_8_REG:
		*fru_element = plat_fru_info->board.board_custom_data[7];
		break;
	case FRU_BOARD_CUSTOM_DATA_9_REG:
		*fru_element = plat_fru_info->board.board_custom_data[8];
		break;
	case FRU_BOARD_CUSTOM_DATA_10_REG:
		*fru_element = plat_fru_info->board.board_custom_data[9];
		break;
	case FRU_PRODUCT_NAME_REG:
		*fru_element = plat_fru_info->product.product_name;
		break;
	case FRU_PRODUCT_PART_NUMBER_REG:
		*fru_element = plat_fru_info->product.product_part_number;
		break;
	case FRU_PRODUCT_PART_VERSION_REG:
		*fru_element = plat_fru_info->product.product_version;
		break;
	case FRU_PRODUCT_SERIAL_NUMBER_REG:
		*fru_element = plat_fru_info->product.product_serial;
		break;
	case FRU_PRODUCT_ASSET_TAG_REG:
		*fru_element = plat_fru_info->product.product_asset_tag;
		break;
	case FRU_PRODUCT_CUSTOM_DATA_1_REG:
		*fru_element = plat_fru_info->product.product_custom_data[0];
		break;
	case FRU_PRODUCT_CUSTOM_DATA_2_REG:
		*fru_element = plat_fru_info->product.product_custom_data[1];
		break;
	default:
		LOG_ERR("Unknown reg offset: 0x%02x", telemetry_info->telemetry_offset);
		break;
	}
	if (*fru_element) {
		*fru_element_size = (uint8_t)strlen(*fru_element);
	} else {
		*fru_element_size = 0;
	}
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

// telemetry
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

bool initialize_fru_board_data(telemetry_info *telemetry_info, uint8_t *buffer_size)
{
	CHECK_NULL_ARG_WITH_RETURN(telemetry_info, false);

	int table_index = telemetry_info->telemetry_offset - FRU_BOARD_PART_NUMBER_REG;
	if (table_index < 0 || table_index >= DATA_TABLE_LENGTH_13)
		return false;

	char *fru_string = NULL;
	uint8_t fru_length = 0;
	if (!get_fru_info_element(telemetry_info, &fru_string, &fru_length)) {
		LOG_ERR("Failed to retrieve FRU Element");
	}

	size_t table_size = sizeof(plat_fru_data) + fru_length;
	plat_fru_data *sensor_data =
		allocate_table((void **)&fru_board_data_table[table_index], table_size);
	if (!sensor_data)
		return false;

	sensor_data->data_length = fru_length;
	memcpy(sensor_data->fru_data, fru_string, fru_length);

	*buffer_size = (uint8_t)table_size;
	return true;
}

bool initialize_fru_product_data(telemetry_info *telemetry_info, uint8_t *buffer_size)
{
	CHECK_NULL_ARG_WITH_RETURN(telemetry_info, false);

	int table_index = telemetry_info->telemetry_offset - FRU_PRODUCT_NAME_REG;
	if (table_index < 0 || table_index >= DATA_TABLE_LENGTH_7)
		return false;

	char *fru_string = NULL;
	uint8_t fru_length = 0;
	if (!get_fru_info_element(telemetry_info, &fru_string, &fru_length)) {
		LOG_ERR("Failed to retrieve FRU Element");
	}

	size_t table_size = sizeof(plat_fru_data) + fru_length;
	plat_fru_data *sensor_data =
		allocate_table((void **)&fru_product_data_table[table_index], table_size);
	if (!sensor_data)
		return false;

	sensor_data->data_length = fru_length;
	memcpy(sensor_data->fru_data, fru_string, fru_length);

	*buffer_size = (uint8_t)table_size;
	return true;
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
	{ FRU_BOARD_PART_NUMBER_REG, 0x00, .telemetry_table_init = initialize_fru_board_data },
	{ FRU_BOARD_SERIAL_NUMBER_REG, 0x00, .telemetry_table_init = initialize_fru_board_data },
	{ FRU_BOARD_PRODUCT_NAME_REG, 0x00, .telemetry_table_init = initialize_fru_board_data },
	{ FRU_BOARD_CUSTOM_DATA_1_REG, 0x00, .telemetry_table_init = initialize_fru_board_data },
	{ FRU_BOARD_CUSTOM_DATA_2_REG, 0x00, .telemetry_table_init = initialize_fru_board_data },
	{ FRU_BOARD_CUSTOM_DATA_3_REG, 0x00, .telemetry_table_init = initialize_fru_board_data },
	{ FRU_BOARD_CUSTOM_DATA_4_REG, 0x00, .telemetry_table_init = initialize_fru_board_data },
	{ FRU_BOARD_CUSTOM_DATA_5_REG, 0x00, .telemetry_table_init = initialize_fru_board_data },
	{ FRU_BOARD_CUSTOM_DATA_6_REG, 0x00, .telemetry_table_init = initialize_fru_board_data },
	{ FRU_BOARD_CUSTOM_DATA_7_REG, 0x00, .telemetry_table_init = initialize_fru_board_data },
	{ FRU_BOARD_CUSTOM_DATA_8_REG, 0x00, .telemetry_table_init = initialize_fru_board_data },
	{ FRU_BOARD_CUSTOM_DATA_9_REG, 0x00, .telemetry_table_init = initialize_fru_board_data },
	{ FRU_BOARD_CUSTOM_DATA_10_REG, 0x00, .telemetry_table_init = initialize_fru_board_data },
	{ FRU_PRODUCT_NAME_REG, 0x00, .telemetry_table_init = initialize_fru_product_data },
	{ FRU_PRODUCT_PART_NUMBER_REG, 0x00, .telemetry_table_init = initialize_fru_product_data },
	{ FRU_PRODUCT_PART_VERSION_REG, 0x00, .telemetry_table_init = initialize_fru_product_data },
	{ FRU_PRODUCT_SERIAL_NUMBER_REG, 0x00,
	  .telemetry_table_init = initialize_fru_product_data },
	{ FRU_PRODUCT_ASSET_TAG_REG, 0x00, .telemetry_table_init = initialize_fru_product_data },
	{ FRU_PRODUCT_CUSTOM_DATA_1_REG, 0x00,
	  .telemetry_table_init = initialize_fru_product_data },
	{ FRU_PRODUCT_CUSTOM_DATA_2_REG, 0x00,
	  .telemetry_table_init = initialize_fru_product_data },
	{ VR_POWER_READING_REG },
};

voltage_rail_mapping_sensor voltage_rail_mapping_table[] = {
	{ CONTROL_VOL_VR_ASIC_P0V75_VDDPHY_HBM0246_REG, VR_RAIL_E_ASIC_P0V75_VDDPHY_HBM0246 },
	{ CONTROL_VOL_VR_ASIC_P0V75_VDDPHY_HBM1357_REG, VR_RAIL_E_ASIC_P0V75_VDDPHY_HBM1357 },
	{ CONTROL_VOL_VR_ASIC_P1V05_VDDC_HBM0246_REG, VR_RAIL_E_ASIC_P1V05_VDDC_HBM0246 },
	{ CONTROL_VOL_VR_ASIC_P1V05_VDDC_HBM1357_REG, VR_RAIL_E_ASIC_P1V05_VDDC_HBM1357 },
	{ CONTROL_VOL_VR_ASIC_P0V4_VDDQL_HBM0246_REG, VR_RAIL_E_ASIC_P0V4_VDDQL_HBM0246 },
	{ CONTROL_VOL_VR_ASIC_P0V4_VDDQL_HBM1357_REG, VR_RAIL_E_ASIC_P0V4_VDDQL_HBM1357 },
	{ CONTROL_VOL_VR_ASIC_P1V8_VPP_HBM0246_REG, VR_RAIL_E_ASIC_P1V8_VPP_HBM0246 },
	{ CONTROL_VOL_VR_ASIC_P1V8_VPP_HBM1357_REG, VR_RAIL_E_ASIC_P1V8_VPP_HBM1357 },
	{ CONTROL_VOL_VR_ASIC_P0V75_NUWA0_VDD_REG, VR_RAIL_E_ASIC_P0V75_NUWA0_VDD },
	{ CONTROL_VOL_VR_ASIC_P0V75_NUWA1_VDD_REG, VR_RAIL_E_ASIC_P0V75_NUWA1_VDD },
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

uint8_t vr_pwr_sensor_table[] = {
	SENSOR_NUM_ASIC_P0V75_NUWA0_VDD_PWR_W,
	SENSOR_NUM_ASIC_P0V75_NUWA1_VDD_PWR_W,
	SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_PWR_W,
	SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_PWR_W,
	SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_PWR_W,
	SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_PWR_W,
	SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_PWR_W,
	SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_PWR_W,
	SENSOR_NUM_ASIC_P1V05_VDDC_HBM1357_PWR_W,
	SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_PWR_W,
	SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_PWR_W,
	SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_PWR_W,
	SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_PWR_W,
	SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_PWR_W,
	SENSOR_NUM_ASIC_P1V05_VDDC_HBM0246_PWR_W,
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
	[0:1] - x = All VR power except NUWA0_VDD and NUWA1_VDD (Unit: W)
	[2:3] - Chiplet0 power = NUWA0_VDD + 0.5*x (Unit: W)
	[4:5] - Chiplet1 power = NUWA1_VDD + 0.5*x (Unit: W)
	[6:7] - P0V75_NUWA0_VDD (Unit: W)
	[8:9] - P0V75_NUWA1_VDD (Unit: W)
	[10:11] - P1V05_VDDC_HBM0246 (Unit: W)
	[12:13] - P0V75_VDDPHY_HBM0246 (Unit: W)
	[14:15] - P1V05_VDDC_HBM1357 (Unit: W)
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
			continue;
		}

		uint16_t val = 0;
		int milivolt = 0;
		milivolt = get_cached_sensor_reading_by_sensor_number(vr_pwr_sensor_table[i]);
		val = (milivolt + 500) / 1000;

		switch (vr_pwr_sensor_table[i]) {
		case SENSOR_NUM_ASIC_P0V75_NUWA0_VDD_PWR_W: {
			if (get_power_capping_source() == CAPPING_SOURCE_ADC) {
				/* ADC instant power in W */
				float pwr_w = get_adc_nuwa_inst_pwr_w(ADC_EL_IDX_NUWA0);
				uint16_t adc_w = (uint16_t)(pwr_w + 0.5f);
				memcpy(&buffer[6], &adc_w, 2);
				medha0 = (int)(pwr_w * 1000.0f + 0.5f);
			} else {
				memcpy(&buffer[6], &val, 2);
				medha0 = milivolt;
			}
			break;
		}
		case SENSOR_NUM_ASIC_P0V75_NUWA1_VDD_PWR_W: {
			if (get_power_capping_source() == CAPPING_SOURCE_ADC) {
				float pwr_w = get_adc_nuwa_inst_pwr_w(ADC_EL_IDX_NUWA1);

				uint16_t adc_w = (uint16_t)(pwr_w + 0.5f);
				memcpy(&buffer[8], &adc_w, 2);

				medha1 = (int)(pwr_w * 1000.0f + 0.5f);
			} else {
				memcpy(&buffer[8], &val, 2);
				medha1 = milivolt;
			}
			break;
		}
		case SENSOR_NUM_ASIC_P1V05_VDDC_HBM0246_PWR_W:
			memcpy(&buffer[10], &val, 2);
			break;
		case SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_PWR_W:
			memcpy(&buffer[12], &val, 2);
			break;
		case SENSOR_NUM_ASIC_P1V05_VDDC_HBM1357_PWR_W:
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

		if (vr_pwr_sensor_table[i] != SENSOR_NUM_ASIC_P0V75_NUWA0_VDD_PWR_W &&
		    vr_pwr_sensor_table[i] != SENSOR_NUM_ASIC_P0V75_NUWA1_VDD_PWR_W) {
			x += milivolt;
		}
	}
	int reading;
	get_cpld_polling_power_info(&reading);
	uint16_t val = (uint16_t)reading;
	memcpy(&buffer[36], &val, 2);

	chiplet0 = ((medha0 + 0.5 * x) + 500) / 1000;
	chiplet1 = ((medha1 + 0.5 * x) + 500) / 1000;
	uint16_t val_x = (uint16_t)((x + 500) / 1000);
	uint16_t val_c0 = (uint16_t)chiplet0;
	uint16_t val_c1 = (uint16_t)chiplet1;
	memcpy(&buffer[0], &val_x, 2);
	memcpy(&buffer[2], &val_c0, 2);
	memcpy(&buffer[4], &val_c1, 2);

	LOG_HEXDUMP_DBG(buffer, buf_size, "VR power sensor data buffer");
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
	// plat_master_write_thread_init();
}

/* I2C target init-enable table */
const bool I2C_TARGET_ENABLE_TABLE[MAX_TARGET_NUM] = {
	TARGET_DISABLE, TARGET_DISABLE, TARGET_DISABLE, TARGET_ENABLE,
	TARGET_ENABLE,	TARGET_ENABLE,	TARGET_ENABLE,	TARGET_DISABLE,
	TARGET_DISABLE, TARGET_DISABLE, TARGET_DISABLE, TARGET_DISABLE,
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
			case FRU_BOARD_PART_NUMBER_REG:
			case FRU_BOARD_SERIAL_NUMBER_REG:
			case FRU_BOARD_PRODUCT_NAME_REG:
			case FRU_BOARD_CUSTOM_DATA_1_REG:
			case FRU_BOARD_CUSTOM_DATA_2_REG:
			case FRU_BOARD_CUSTOM_DATA_3_REG:
			case FRU_BOARD_CUSTOM_DATA_4_REG:
			case FRU_BOARD_CUSTOM_DATA_5_REG:
			case FRU_BOARD_CUSTOM_DATA_6_REG:
			case FRU_BOARD_CUSTOM_DATA_7_REG:
			case FRU_BOARD_CUSTOM_DATA_8_REG:
			case FRU_BOARD_CUSTOM_DATA_9_REG:
			case FRU_BOARD_CUSTOM_DATA_10_REG: {
				data->target_rd_msg.msg_length = struct_size;
				memcpy(data->target_rd_msg.msg,
				       fru_board_data_table[reg_offset - FRU_BOARD_PART_NUMBER_REG],
				       struct_size);
			} break;
			case FRU_PRODUCT_NAME_REG:
			case FRU_PRODUCT_PART_NUMBER_REG:
			case FRU_PRODUCT_PART_VERSION_REG:
			case FRU_PRODUCT_SERIAL_NUMBER_REG:
			case FRU_PRODUCT_ASSET_TAG_REG:
			case FRU_PRODUCT_CUSTOM_DATA_1_REG:
			case FRU_PRODUCT_CUSTOM_DATA_2_REG: {
				data->target_rd_msg.msg_length = struct_size;
				memcpy(data->target_rd_msg.msg,
				       fru_product_data_table[reg_offset - FRU_PRODUCT_NAME_REG],
				       struct_size);
			} break;
			case CONTROL_VOL_VR_ASIC_P0V75_VDDPHY_HBM0246_REG:
			case CONTROL_VOL_VR_ASIC_P0V75_VDDPHY_HBM1357_REG:
			case CONTROL_VOL_VR_ASIC_P1V05_VDDC_HBM0246_REG:
			case CONTROL_VOL_VR_ASIC_P1V05_VDDC_HBM1357_REG:
			case CONTROL_VOL_VR_ASIC_P0V4_VDDQL_HBM0246_REG:
			case CONTROL_VOL_VR_ASIC_P0V4_VDDQL_HBM1357_REG:
			case CONTROL_VOL_VR_ASIC_P1V8_VPP_HBM0246_REG:
			case CONTROL_VOL_VR_ASIC_P1V8_VPP_HBM1357_REG:
			case CONTROL_VOL_VR_ASIC_P0V75_NUWA0_VDD_REG:
			case CONTROL_VOL_VR_ASIC_P0V75_NUWA1_VDD_REG: {
				uint8_t rail = get_vr_rail_by_control_vol_reg(reg_offset);
				uint16_t vout = 0xFFFF;
				if (!voltage_command_setting_get(rail, &vout)) {
					LOG_ERR("Can't voltage_command setting_get by rail: 0x%02x",
						rail);
				}
				memcpy(data->target_rd_msg.msg, &vout, sizeof(vout));
				data->target_rd_msg.msg_length = 2;
			} break;
			case LEVEL_1_PWR_ALERT_THRESHOLD_TIME_REG:
			case LEVEL_2_PWR_ALERT_THRESHOLD_TIME_REG:
			case LEVEL_3_PWR_ALERT_THRESHOLD_TIME_REG: {
				uint8_t lv = 0;
				uint16_t tmp_value = 0;
				if (reg_offset == LEVEL_1_PWR_ALERT_THRESHOLD_TIME_REG) {
					lv = CAPPING_LV_IDX_LV1;
				} else if (reg_offset == LEVEL_2_PWR_ALERT_THRESHOLD_TIME_REG) {
					lv = CAPPING_LV_IDX_LV2;
				} else if (reg_offset == LEVEL_3_PWR_ALERT_THRESHOLD_TIME_REG) {
					lv = CAPPING_LV_IDX_LV3;
				}
				tmp_value = get_power_capping_threshold(CAPPING_VR_IDX_NUWA0, lv);
				data->target_rd_msg.msg[0] = tmp_value & 0xFF;
				data->target_rd_msg.msg[1] = tmp_value >> 8;
				tmp_value = get_power_capping_time_w(CAPPING_VR_IDX_NUWA0, lv);
				data->target_rd_msg.msg[2] = tmp_value & 0xFF;
				data->target_rd_msg.msg[3] = tmp_value >> 8;
				tmp_value = get_power_capping_threshold(CAPPING_VR_IDX_NUWA1, lv);
				data->target_rd_msg.msg[4] = tmp_value & 0xFF;
				data->target_rd_msg.msg[5] = tmp_value >> 8;
				tmp_value = get_power_capping_time_w(CAPPING_VR_IDX_NUWA1, lv);
				data->target_rd_msg.msg[6] = tmp_value & 0xFF;
				data->target_rd_msg.msg[7] = tmp_value >> 8;
				data->target_rd_msg.msg_length = 8;
			} break;
			case VR_POWER_READING_REG: {
				data->target_rd_msg.msg_length = VR_PWR_BUF_SIZE;
				vr_power_reading(data->target_rd_msg.msg,
						 data->target_rd_msg.msg_length);
				LOG_HEXDUMP_DBG(data->target_rd_msg.msg,
						data->target_rd_msg.msg_length, "vr power reading");
			} break;
			case NUWA_SENSOR_VALUE_REG: {
				data->target_rd_msg.msg_length = 4;
				uint16_t sensor_value = 0;

				if (get_power_capping_source() == CAPPING_SOURCE_ADC) {
					/* ADC instant power in W */
					float pwr_w0 = get_adc_nuwa_inst_pwr_w(ADC_EL_IDX_NUWA0);
					float pwr_w1 = get_adc_nuwa_inst_pwr_w(ADC_EL_IDX_NUWA1);

					sensor_value = (uint16_t)(pwr_w0 + 0.5f);
					memcpy(&data->target_rd_msg.msg[0], &sensor_value,
					       sizeof(sensor_value));

					sensor_value = (uint16_t)(pwr_w1 + 0.5f);
					memcpy(&data->target_rd_msg.msg[2], &sensor_value,
					       sizeof(sensor_value));
				} else {
					/* VR sensor cache power in W */
					sensor_value =
						(get_cached_sensor_reading_by_sensor_number(
							 SENSOR_NUM_ASIC_P0V75_NUWA0_VDD_PWR_W) +
						 500) /
						1000;
					memcpy(&data->target_rd_msg.msg[0], &sensor_value,
					       sizeof(sensor_value));

					sensor_value =
						(get_cached_sensor_reading_by_sensor_number(
							 SENSOR_NUM_ASIC_P0V75_NUWA1_VDD_PWR_W) +
						 500) /
						1000;
					memcpy(&data->target_rd_msg.msg[2], &sensor_value,
					       sizeof(sensor_value));
				}
			} break;
			case POWER_CAPPING_METHOD_REG: {
				data->target_rd_msg.msg[0] = get_power_capping_method();
				data->target_rd_msg.msg_length = 1;
			} break;
			case NUWA_POWER_SOURCE_REG: {
				data->target_rd_msg.msg[0] = get_power_capping_source();
				data->target_rd_msg.msg_length = 1;
			} break;
			case POLLING_RATE_TELEMETRY_REG: {
				uint8_t type_val = get_pwr_capping_polling_rate_type();
				data->target_rd_msg.msg[0] = type_val;
				data->target_rd_msg.msg_length = 1;
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
		} else {
			LOG_ERR("Received data length: 0x%02x", data->wr_buffer_idx);
			data->target_rd_msg.msg_length = 1;
			data->target_rd_msg.msg[0] = 0xFF;
		}
	}
	LOG_DBG("Reply data length: 0x%02x", data->target_rd_msg.msg_length);
	return false;
}

/* I2C target init-config table */
const struct _i2c_target_config I2C_TARGET_CONFIG_TABLE[MAX_TARGET_NUM] = {
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0x40, 0xA, command_reply_data_handle },
	{ 0x40, 0xA, command_reply_data_handle },
	{ 0x42, 0xA },
	{ 0x40, 0xA, command_reply_data_handle },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
};