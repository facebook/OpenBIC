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

#ifndef PLAT_I2C_SLAVE_H
#define PLAT_I2C_SLAVE_H

#include <drivers/i2c.h>
#include "hal_i2c_target.h"

#define TARGET_ENABLE 1
#define TARGET_DISABLE 0

// i2c target register
#define SENSOR_INIT_DATA_0_REG 0x00
#define SENSOR_INIT_DATA_1_REG 0x01
#define SENSOR_READING_0_REG 0x02
#define SENSOR_READING_1_REG 0x03
#define SENSOR_READING_2_REG 0x04
#define SENSOR_READING_3_REG 0x05
#define INVENTORY_IDS_REG 0x06

typedef struct _i2c_target_command_mapping {
	uint16_t reg; // register address
	uint8_t addr;
	uint8_t (*wr_fn)(struct _i2c_target_command_mapping *);
	uint8_t (*rd_fn)(struct _i2c_target_command_mapping *);
	uint8_t *data;
} i2c_target_command_mapping;
typedef struct __attribute__((__packed__)) {
	uint16_t carrier_board_id; //  MTIA Gen1 - Aegis=0x00
	uint32_t bic_fw_version;
	uint32_t cpld_fw_version;
} plat_inventory_ids;
// size = sizeof(plat_inventory_ids)
typedef struct __attribute__((__packed__)) {
	uint8_t device_type;
	uint8_t register_layout_version;
	uint8_t num_idx; //Number of PDR indexes in this register
	uint8_t reserved_1;

	uint16_t sbi; //Sensor base index (SBI= 248*R) used in this register's array
	uint16_t max_pdr_idx; //Max PDR sensor index exported in total (MAX_PDRIDX)

	uint8_t sensor_r_len[]; //sensor[0] reading length ~ sensor[247] reading length
} plat_sensor_init_data;
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
} plat_sensor_reading;

typedef struct _plat_sensor_reading_data_ {
	uint8_t sensor_number;
	uint8_t data[4];
} plat_sensor_reading_data;

// size = sizeof(plat_sensor_reading) + num_sensors * sizeof(SensorEntry);
// num_sensors = max_sbi_off + 1
typedef struct _telemetry_info_ telemetry_info;
typedef struct _telemetry_info_ {
	uint8_t telemetry_offset;
	uint16_t data_size;
	bool (*telemetry_table_init)(telemetry_info *, uint8_t *);
} telemetry_info;

void plat_telemetry_table_init(void);
void update_sensor_reading_by_sensor_number(uint8_t sensor_number);
#endif
