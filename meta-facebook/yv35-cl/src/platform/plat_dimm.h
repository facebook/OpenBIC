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

#ifndef PLAT_DIMM_H
#define PLAT_DIMM_H

#include <stdint.h>
#include <stdbool.h>
#include "plat_i3c.h"

#define MAX_COUNT_DIMM 6

#define I3C_MUX_TO_BIC 0x1
#define I3C_MUX_TO_CPU 0x0
#define DIMM_MUX_TO_DIMM_A0A1A3 0x0
#define DIMM_MUX_TO_DIMM_A4A6A7 0x1
#define I3C_DIMM_MUX_MUTEX_TIMEOUT_MS 1000

#define GET_DIMM_INFO_TIME_MS 1000
#define GET_DIMM_INFO_STACK_SIZE 2048

#define MAX_LEN_I3C_GET_PMIC_ERR 7
#define MAX_LEN_I3C_GET_PMIC_PWR 1
#define MAX_LEN_I3C_GET_SPD_TEMP 2

#define DIMM_I3C_MUX_CONTROL_OFFSET 0x0B

enum DIMM_ID {
	DIMM_ID_A0 = 0,
	DIMM_ID_A2,
	DIMM_ID_A3,
	DIMM_ID_A4,
	DIMM_ID_A6,
	DIMM_ID_A7,
	DIMM_ID_UNKNOWN,
};

enum I3C_PMIC_ADDR {
	PMIC_A0_A4_ADDR = 0x48,
	PMIC_A2_A6_ADDR = 0x4c,
	PMIC_A3_A7_ADDR = 0x4e,
};

enum I3C_DIMM_SPD_ADDR {
	DIMM_SPD_A0_A4_ADDR = 0x50,
	DIMM_SPD_A2_A6_ADDR = 0x54,
	DIMM_SPD_A3_A7_ADDR = 0x56,
};

enum DIMM_DEVICE_TYPE {
	DIMM_SPD = 0x00,
	DIMM_SPD_NVM = 0x01,
	DIMM_PMIC = 0x02,
};

typedef struct dimm_info {
	bool is_present;
	uint8_t pmic_error_data[MAX_LEN_I3C_GET_PMIC_ERR];
	uint8_t pmic_pwr_data[MAX_LEN_I3C_GET_PMIC_PWR];
	uint8_t spd_temp_data[MAX_LEN_I3C_GET_SPD_TEMP];
} dimm_info;

extern struct k_mutex i3c_dimm_mux_mutex;
extern uint8_t pmic_i3c_addr_list[MAX_COUNT_DIMM / 2];
extern uint8_t spd_i3c_addr_list[MAX_COUNT_DIMM / 2];

extern dimm_info dimm_data[MAX_COUNT_DIMM];

void start_get_dimm_info_thread();
void get_dimm_info_handler();
void init_i3c_dimm_data();
void clear_unaccessible_dimm_data(uint8_t dimm_id);
int switch_i3c_dimm_mux(uint8_t i3c_mux_position, uint8_t dimm_mux_position);
bool is_i3c_mux_to_bic();
int all_brocast_ccc(I3C_MSG *i3c_msg);
bool is_dimm_present(uint8_t dimm_id);
bool is_dimm_init();
uint8_t sensor_num_map_dimm_id(uint8_t sensor_num);
int get_pmic_error_raw_data(int dimm_index, uint8_t *data);
void get_pmic_power_raw_data(int dimm_index, uint8_t *data);
void get_spd_temp_raw_data(int dimm_index, uint8_t *data);

#endif
