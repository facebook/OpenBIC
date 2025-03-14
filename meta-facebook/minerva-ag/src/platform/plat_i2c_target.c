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
#include <string.h>
#include "libutil.h"
#include "plat_i2c_target.h"
#include "plat_i2c.h"
#include <logging/log.h>
#include "plat_gpio.h"
#include "plat_pldm_sensor.h"
#include "pldm_sensor.h"
#include "plat_version.h"
#include "plat_isr.h"

#define DEVICE_TYPE 0x01
#define REGISTER_LAYOUT_VERSION 0x01
#define AEGIS_CARRIER_BOARD_ID 0x0000
#define AEGIS_CPLD_ADDR (0x4C >> 1)

typedef struct __attribute__((__packed__)) {
	uint8_t device_type;
	uint8_t register_layout_version;
	uint8_t num_idx; //Number of PDR indexes in this register
	uint8_t reserved_1;

	uint16_t sbi; //Sensor base index (SBI= 248*R) used in this register's array
	uint16_t max_pdr_idx; //Max PDR sensor index exported in total (MAX_PDRIDX)

	uint8_t sensor_r_len[]; //sensor[0] reading length ~ sensor[247] reading length
} plat_sensor_init_data_0_1;
// size = sizeof(plat_sensor_init_data_0_1) + num_idx

typedef struct __attribute__((__packed__)) {
	uint8_t sensor_index_offset; // Sensor index offset (e.g. PDR sensor index offset)
	uint32_t sensor_value; // Sensor value (4 bytes)
} sensor_entry;

typedef struct __attribute__((__packed__)) {
	uint8_t device_type; // Device type (Aegis = 0x01, Rainbow = 0x02)
	uint8_t register_layout_version; // Register layout version (e.g. VERSION_1 = 0x01)
	uint16_t sensor_base_index; // Sensor base index (SBI)
	uint8_t max_sbi_off; // Max sensor base index offset in this register (0 <= MAX_SBI_OFF <= 49)
	// The following is a flexible array of sensor entries.
	// The number of entries is (max_sbi_off + 1)
	sensor_entry sensor_entries[];
} plat_sensor_init_data_0_2;
// size = sizeof(plat_sensor_init_data_0_2) + num_sensors * sizeof(SensorEntry);
// num_sensors = max_sbi_off + 1

typedef struct __attribute__((__packed__)) {
	uint16_t carrier_board_id; //  MTIA Gen1 - Aegis=0x00
	uint32_t bic_fw_version;
	uint32_t cpld_fw_version;

	// FRU section
} plat_sensor_init_data_0_6;
// size = sizeof(plat_sensor_init_data_0_6)

LOG_MODULE_REGISTER(plat_i2c_target);
/* I2C target init-enable table */
const bool I2C_TARGET_ENABLE_TABLE[MAX_TARGET_NUM] = {
	TARGET_DISABLE, TARGET_DISABLE, TARGET_DISABLE, TARGET_ENABLE,
	TARGET_DISABLE, TARGET_DISABLE, TARGET_ENABLE,	TARGET_DISABLE,
	TARGET_DISABLE, TARGET_DISABLE, TARGET_DISABLE, TARGET_DISABLE,
};

plat_sensor_init_data_0_1 *sensor_init_data_0_1_table[2] = { NULL };
plat_sensor_init_data_0_2 *sensor_init_data_0_2_table[4] = { NULL };
plat_sensor_init_data_0_6 *sensor_init_data_0_6_table[1] = { NULL };

void init_sensor_data_0_1_table()
{
	for (int reg_offset = 0; reg_offset < 2; reg_offset++) {
		// Calculate num_idx
		int num_idx = (PLAT_SENSOR_NUM_MAX - 1) - (248 * reg_offset);
		num_idx = (num_idx > 0) ? ((num_idx > 248) ? 248 : num_idx) : 0;

		// Calculate the memory size
		size_t sensor_init_data_0_1_size =
			sizeof(plat_sensor_init_data_0_1) + num_idx * sizeof(uint8_t);

		// Release old memory first
		if (sensor_init_data_0_1_table[reg_offset]) {
			free(sensor_init_data_0_1_table[reg_offset]);
			sensor_init_data_0_1_table[reg_offset] = NULL;
		}

		// Allocate memory
		sensor_init_data_0_1_table[reg_offset] =
			(plat_sensor_init_data_0_1 *)malloc(sensor_init_data_0_1_size);

		if (!sensor_init_data_0_1_table[reg_offset]) {
			LOG_ERR("Memory allocation failed!");
			return;
		}

		plat_sensor_init_data_0_1 *sensor_init_data_0_1 =
			sensor_init_data_0_1_table[reg_offset];
		sensor_init_data_0_1->device_type = DEVICE_TYPE;
		sensor_init_data_0_1->register_layout_version = REGISTER_LAYOUT_VERSION;
		sensor_init_data_0_1->num_idx = num_idx;
		sensor_init_data_0_1->reserved_1 = 0xFF;
		sensor_init_data_0_1->sbi = reg_offset * 248;
		sensor_init_data_0_1->max_pdr_idx = (reg_offset == 0x00) ?
							    PLAT_SENSOR_NUM_MAX - 2 :
							    0xFFFF; // PDR indexe is on 0 base
		memset(sensor_init_data_0_1->sensor_r_len, 4, num_idx * sizeof(uint8_t));
	}
}

void init_sensor_data_0_2_table()
{
	for (int reg_offset = 0; reg_offset < 4; reg_offset++) {
		int num_idx = (PLAT_SENSOR_NUM_MAX - 1) - (50 * reg_offset);
		num_idx = (num_idx > 0) ? ((num_idx > 50) ? 50 : num_idx) : 0;

		size_t sensor_init_data_0_2_size =
			sizeof(plat_sensor_init_data_0_2) + num_idx * sizeof(sensor_entry);

		if (sensor_init_data_0_2_table[reg_offset]) {
			free(sensor_init_data_0_2_table[reg_offset]);
			sensor_init_data_0_2_table[reg_offset] = NULL;
		}

		sensor_init_data_0_2_table[reg_offset] =
			(plat_sensor_init_data_0_2 *)malloc(sensor_init_data_0_2_size);

		if (!sensor_init_data_0_2_table[reg_offset]) {
			LOG_ERR("Memory allocation failed!");
			return;
		}

		plat_sensor_init_data_0_2 *sensor_init_data_0_2 =
			sensor_init_data_0_2_table[reg_offset];
		sensor_init_data_0_2->device_type = DEVICE_TYPE;
		sensor_init_data_0_2->register_layout_version = REGISTER_LAYOUT_VERSION;
		sensor_init_data_0_2->sensor_base_index = reg_offset * 50;
		sensor_init_data_0_2->max_sbi_off = (num_idx > 0) ? num_idx - 1 : 0;
		if (num_idx > 0) {
			for (int i = 0; i < num_idx; i++) {
				sensor_init_data_0_2->sensor_entries[i].sensor_index_offset =
					i; // sensor_index_offset range: 0~49
				sensor_init_data_0_2->sensor_entries[i].sensor_value = 0;
			}
		}
	}
}

void update_sensor_data_0_2_table()
{
	for (int reg_offset = 0; reg_offset < 4; reg_offset++) {
		if (!sensor_init_data_0_2_table[reg_offset]) {
			continue; // not to operate on NULL
		}

		int num_idx = (PLAT_SENSOR_NUM_MAX - 1) - (50 * reg_offset);
		num_idx = (num_idx > 0) ? ((num_idx > 50) ? 50 : num_idx) : 0;

		for (int i = 0; i < num_idx; i++) {
			int sensor_number =
				sensor_init_data_0_2_table[reg_offset]->sensor_base_index +
				sensor_init_data_0_2_table[reg_offset]
					->sensor_entries[i]
					.sensor_index_offset +
				1; // sensor number is 1 base

			uint8_t status = SENSOR_UNAVAILABLE;
			int reading = 0;
			uint8_t sensor_operational_state = PLDM_SENSOR_STATUSUNKOWN;

			status = pldm_sensor_get_reading_from_cache(sensor_number, &reading,
								    &sensor_operational_state);

			sensor_init_data_0_2_table[reg_offset]->sensor_entries[i].sensor_value =
				(status == SENSOR_READ_SUCCESS) ? reading : 0xFFFFFFFF;
		}
	}
}

void init_sensor_data_0_6_table()
{
	size_t sensor_init_data_0_6_size = sizeof(plat_sensor_init_data_0_6);

	if (sensor_init_data_0_6_table[0]) {
		free(sensor_init_data_0_6_table[0]);
		sensor_init_data_0_6_table[0] = NULL;
	}

	sensor_init_data_0_6_table[0] =
		(plat_sensor_init_data_0_6 *)malloc(sensor_init_data_0_6_size);

	if (!sensor_init_data_0_6_table[0]) {
		LOG_ERR("Memory allocation failed!");
		return;
	}

	uint8_t data[4] = { 0 };
	uint32_t bic_version = 0;
	uint32_t cpld_version = 0;
	if (!plat_i2c_read(I2C_BUS5, AEGIS_CPLD_ADDR, 0x44, data, 4)) {
		LOG_ERR("Failed to read cpld version from cpld");
		return;
	}
	cpld_version = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
	bic_version =
		(BIC_FW_YEAR_MSB << 24) | (BIC_FW_YEAR_LSB << 16) | (BIC_FW_WEEK << 8) | BIC_FW_VER;

	plat_sensor_init_data_0_6 *sensor_init_data_0_6 = sensor_init_data_0_6_table[0];
	sensor_init_data_0_6->carrier_board_id = AEGIS_CARRIER_BOARD_ID;
	sensor_init_data_0_6->bic_fw_version = bic_version;
	sensor_init_data_0_6->cpld_fw_version = cpld_version;
}

static bool command_reply_data_handle(void *arg)
{
	struct i2c_target_data *data = (struct i2c_target_data *)arg;

	/*TODO: put board telemetry here*/

	/* Only check fisrt byte from received data */
	if (data->wr_buffer_idx == 1) {
		uint8_t reg_offset = data->target_wr_msg.msg[0];
		switch (reg_offset) {
		case 0x00:
		case 0x01: {
			size_t struct_size =
				sizeof(plat_sensor_init_data_0_1) +
				sensor_init_data_0_1_table[reg_offset]->num_idx * sizeof(uint8_t);

			// Make sure the target buffer is not exceeded when reading
			if (struct_size > sizeof(data->target_rd_msg.msg)) {
				struct_size = sizeof(data->target_rd_msg.msg);
			}

			data->target_rd_msg.msg_length = struct_size;
			memcpy(data->target_rd_msg.msg, sensor_init_data_0_1_table[reg_offset],
			       struct_size);
		} break;
		case 0x02:
		case 0x03:
		case 0x04:
		case 0x05: {
			size_t struct_size;
			if (sensor_init_data_0_2_table[reg_offset - 2]->max_sbi_off == 0) {
				struct_size = sizeof(plat_sensor_init_data_0_2);
			} else {
				struct_size =
					sizeof(plat_sensor_init_data_0_2) +
					(sensor_init_data_0_2_table[reg_offset - 2]->max_sbi_off +
					 1) * sizeof(sensor_entry);
			}

			if (struct_size > sizeof(data->target_rd_msg.msg)) {
				struct_size = sizeof(data->target_rd_msg.msg);
			}

			data->target_rd_msg.msg_length = struct_size;
			memcpy(data->target_rd_msg.msg, sensor_init_data_0_2_table[reg_offset - 2],
			       struct_size);

		} break;
		case 0x06: {
			size_t struct_size;
			struct_size = sizeof(plat_sensor_init_data_0_6);

			if (struct_size > sizeof(data->target_rd_msg.msg)) {
				struct_size = sizeof(data->target_rd_msg.msg);
			}

			data->target_rd_msg.msg_length = struct_size;
			memcpy(data->target_rd_msg.msg, sensor_init_data_0_6_table[0], struct_size);

		} break;
		default:
			data->target_rd_msg.msg_length = 20;
			for (int i = 0; i < 20; i++)
				data->target_rd_msg.msg[i] = data->target_wr_msg.msg[0] + i;
			break;
		}
	}
	return false;
}

void sensor_data_table_init(void)
{
	init_sensor_data_0_1_table();
	init_sensor_data_0_2_table();
	init_sensor_data_0_6_table();
}

/* I2C target init-config table */
const struct _i2c_target_config I2C_TARGET_CONFIG_TABLE[MAX_TARGET_NUM] = {
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0x40, 0xA },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0x40, 0xA, command_reply_data_handle },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
};
