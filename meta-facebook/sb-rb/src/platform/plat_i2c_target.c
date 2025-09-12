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
	TARGET_DISABLE, TARGET_ENABLE,	TARGET_DISABLE, TARGET_DISABLE,
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

plat_sensor_init_data *sensor_init_data_table[DATA_TABLE_LENGTH_2] = { NULL };
plat_sensor_reading *sensor_reading_table[DATA_TABLE_LENGTH_4] = { NULL };
plat_inventory_ids *inventory_ids_table[DATA_TABLE_LENGTH_1] = { NULL };
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

telemetry_info telemetry_info_table[] = {
	{ SENSOR_INIT_DATA_0_REG, 0x00, .telemetry_table_init = initialize_sensor_data },
	{ SENSOR_INIT_DATA_1_REG, 0x00, .telemetry_table_init = initialize_sensor_data },
	{ SENSOR_READING_0_REG, 0x00, .telemetry_table_init = initialize_sensor_reading },
	{ SENSOR_READING_1_REG, 0x00, .telemetry_table_init = initialize_sensor_reading },
	{ SENSOR_READING_2_REG, 0x00, .telemetry_table_init = initialize_sensor_reading },
	{ SENSOR_READING_3_REG, 0x00, .telemetry_table_init = initialize_sensor_reading },
	{ INVENTORY_IDS_REG, 0x00, .telemetry_table_init = initialize_inventory_ids },
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
			LOG_INF("Received reg offset: 0x%02x", reg_offset);
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
	return true;
}

/* I2C target init-config table */
const struct _i2c_target_config I2C_TARGET_CONFIG_TABLE[MAX_TARGET_NUM] = {
	{ 0xFF, 0xA }, { 0xFF, 0xA }, { 0xFF, 0xA },
	{ 0xFF, 0xA }, { 0xFF, 0xA }, { 0x42, 0xA, command_reply_data_handle },
	{ 0xFF, 0xA }, { 0xFF, 0xA }, { 0xFF, 0xA },
	{ 0xFF, 0xA }, { 0xFF, 0xA }, { 0xFF, 0xA },
};
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
	//plat_master_write_thread_init();
}
