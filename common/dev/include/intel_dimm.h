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

#ifndef DIMM_H
#define DIMM_H

#include <stdbool.h>
#include <stdint.h>

#define CMD_GET_CPU_MEMORY_TEMP 0x4B
#define CMD_GET_CPU_MEMORY_TEMP_DATA_LEN 12
#define RESPONSE_DIMM_TEMP_INDEX 3

/* Get CPU and memory temperature command request */
#define CHANNEL_0_4_DIMM_NUM_0 BIT(0)
#define CHANNEL_0_4_DIMM_NUM_1 BIT(1)
#define CHANNEL_0_4_DIMM_NUM_2 BIT(2)
#define CHANNEL_0_4_DIMM_NUM_3 BIT(3)
#define CHANNEL_1_5_DIMM_NUM_0 BIT(4)
#define CHANNEL_1_5_DIMM_NUM_1 BIT(5)
#define CHANNEL_1_5_DIMM_NUM_2 BIT(6)
#define CHANNEL_1_5_DIMM_NUM_3 BIT(7)
#define CHANNEL_2_6_DIMM_NUM_0 BIT(8)
#define CHANNEL_2_6_DIMM_NUM_1 BIT(9)
#define CHANNEL_2_6_DIMM_NUM_2 BIT(10)
#define CHANNEL_2_6_DIMM_NUM_3 BIT(11)
#define CHANNEL_3_7_DIMM_NUM_0 BIT(12)
#define CHANNEL_3_7_DIMM_NUM_1 BIT(13)
#define CHANNEL_3_7_DIMM_NUM_2 BIT(14)
#define CHANNEL_3_7_DIMM_NUM_3 BIT(15)

enum DIMM_CHANNEL_NUM {
	DIMM_CHANNEL_NUM_0,
	DIMM_CHANNEL_NUM_1,
	DIMM_CHANNEL_NUM_2,
	DIMM_CHANNEL_NUM_3,
	DIMM_CHANNEL_NUM_4,
	DIMM_CHANNEL_NUM_5,
	DIMM_CHANNEL_NUM_6,
	DIMM_CHANNEL_NUM_7,
};

enum DIMM_NUMBER {
	DIMM_NUMBER_0,
	DIMM_NUMBER_1,
	DIMM_NUMBER_2,
	DIMM_NUMBER_3,
};

enum CPU_USE_RANGE {
	USE_CPU_0_TO_3 = 0x0,
	USE_CPU_4_TO_7 = 0x1,
	USE_CPU_8_TO_11 = 0x2,
};

enum MEMORY_CHANNEL {
	MEMORY_CHANNEL_0_TO_3 = 0x0,
	MEMORY_CHANNEL_4_TO_7 = 0x1,
};

typedef struct _get_cpu_memory_temp_req {
	uint8_t intel_id[3];
	uint8_t set_read_cpu0 : 1;
	uint8_t set_read_cpu1 : 1;
	uint8_t set_read_cpu2 : 1;
	uint8_t set_read_cpu3 : 1;
	uint8_t set_use_cpu : 2;
	uint8_t set_memory_channel : 1;
	uint8_t set_request_format : 1;
	uint16_t cpu0_read_dimm_req;
	uint16_t cpu1_read_dimm_req;
	uint16_t cpu2_read_dimm_req;
	uint16_t cpu3_read_dimm_req;
} get_cpu_memory_temp_req;

enum DIMM_CHANNEL_ADDR {
	ADDR_DIMM_CHANNEL_0_4 = 0x90, // slave address for channel 0 and channel 4
	ADDR_DIMM_CHANNEL_1_5 = 0x94, // slave address for channel 1 and channel 5
	ADDR_DIMM_CHANNEL_2_6 = 0x98, // slave address for channel 2 and channel 6
	ADDR_DIMM_CHANNEL_3_7 = 0x9C, // slave address for channel 3 and channel 7
};

enum BUS_ID {
	BUS_ID_DIMM_CHANNEL_0_TO_3, // bus id for channel 0~3
	BUS_ID_DIMM_CHANNEL_4_TO_7, // bus id for channel 4~7
};

int pal_get_pmic_pwr(uint8_t sensor_num, uint8_t *data);
int pal_get_spd_temp(uint8_t sensor_num, uint8_t *data);

#endif
