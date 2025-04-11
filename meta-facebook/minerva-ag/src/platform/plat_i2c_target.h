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

#define TELEMETRY_BUFF_SIZE 255

#define SENSOR_INIT_DATA_0_REG 0x00
#define SENSOR_INIT_DATA_1_REG 0x01
#define SENSOR_READING_0_REG 0x02
#define SENSOR_READING_1_REG 0x03
#define SENSOR_READING_2_REG 0x04
#define SENSOR_READING_3_REG 0x05
#define INVENTORY_IDS_REG 0x06
#define OWL_NIC_MAC_ADDRESSES_REG 0x07 //not used
#define STRAP_CAPABILTITY_REG 0x08
#define WRITE_STRAP_PIN_VALUE_REG 0x09

#define FRU_BOARD_PART_NUMBER_REG 0x60
#define FRU_BOARD_SERIAL_NUMBER_REG 0x61
#define FRU_BOARD_PRODUCT_NAME_REG 0x62
#define FRU_BOARD_CUSTOM_DATA_1_REG 0x63
#define FRU_BOARD_CUSTOM_DATA_2_REG 0x64
#define FRU_BOARD_CUSTOM_DATA_3_REG 0x65
#define FRU_BOARD_CUSTOM_DATA_4_REG 0x66

#define FRU_PRODUCT_NAME_REG 0x70
#define FRU_PRODUCT_PART_NUMBER_REG 0x71
#define FRU_PRODUCT_PART_VERSION_REG 0x72
#define FRU_PRODUCT_SERIAL_NUMBER_REG 0x73
#define FRU_PRODUCT_ASSET_TAG_REG 0x74
#define FRU_PRODUCT_CUSTOM_DATA_1_REG 0x75
#define FRU_PRODUCT_CUSTOM_DATA_2_REG 0x76

void update_sensor_data_2_5_table(void);
void sensor_data_table_init(void);

typedef struct _telemetry_info_ telemetry_info;

typedef struct _telemetry_info_ {
	uint8_t telemetry_offset;
	uint16_t data_size;
	bool (*sensor_data_init)(telemetry_info *, uint8_t *);

} telemetry_info;

#endif
