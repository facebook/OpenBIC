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

#include "plat_i3c.h"

#define DIMM_SPD_A_G_ADDR (0xA0 >> 1)
#define DIMM_SPD_B_H_ADDR (0xA2 >> 1)
#define DIMM_SPD_C_I_ADDR (0xA4 >> 1)
#define DIMM_SPD_D_J_ADDR (0xA6 >> 1)
#define DIMM_SPD_E_K_ADDR (0xA8 >> 1)
#define DIMM_SPD_F_L_ADDR (0xAA >> 1)

#define DIMM_PMIC_A_G_ADDR 0x48
#define DIMM_PMIC_B_H_ADDR 0x49
#define DIMM_PMIC_C_I_ADDR 0x4A
#define DIMM_PMIC_D_J_ADDR 0x4B
#define DIMM_PMIC_E_K_ADDR 0x4C
#define DIMM_PMIC_F_L_ADDR 0x4D

#define CPLD_ADDR 0x21

#define MAX_LEN_I3C_GET_PMIC_PWR 1
#define MAX_LEN_I3C_GET_SPD_TEMP 2

#define DIMM_I3C_MUX_CONTROL_OFFSET 0x0C
#define I3C_MUX_BIC_TO_DIMMA_TO_F 0x02
#define I3C_MUX_BIC_TO_DIMMG_TO_L 0x03
#define I3C_MUX_CPU_TO_DIMM 0x00

#define I3C_DIMM_MUTEX_TIMEOUT_MS 1000
#define GET_DIMM_INFO_TIME_MS 1000
#define GET_DIMM_INFO_STACK_SIZE 2304

typedef struct dimm_info {
	uint8_t is_present;
	bool is_ready_monitor;
	uint8_t spd_temp_data[MAX_LEN_I3C_GET_SPD_TEMP];
	uint8_t pmic_pwr_data[MAX_LEN_I3C_GET_PMIC_PWR];
} dimm_info;

enum NUMBER_DIMM_TEMP {
	NUM_DIMM_A_TEMP = 0x0005,
	NUM_DIMM_B_TEMP,
	NUM_DIMM_C_TEMP,
	NUM_DIMM_D_TEMP,
	NUM_DIMM_E_TEMP,
	NUM_DIMM_F_TEMP,
	NUM_DIMM_G_TEMP,
	NUM_DIMM_H_TEMP,
	NUM_DIMM_I_TEMP,
	NUM_DIMM_J_TEMP,
	NUM_DIMM_K_TEMP,
	NUM_DIMM_L_TEMP,
};

enum NUMBER_DIMM_PMIC_PWR {
	NUM_DIMM_A_PMIC_PWR = 0x0056,
	NUM_DIMM_B_PMIC_PWR,
	NUM_DIMM_C_PMIC_PWR,
	NUM_DIMM_D_PMIC_PWR,
	NUM_DIMM_E_PMIC_PWR,
	NUM_DIMM_F_PMIC_PWR,
	NUM_DIMM_G_PMIC_PWR,
	NUM_DIMM_H_PMIC_PWR,
	NUM_DIMM_I_PMIC_PWR,
	NUM_DIMM_J_PMIC_PWR,
	NUM_DIMM_K_PMIC_PWR,
	NUM_DIMM_L_PMIC_PWR,
};

enum DIMM_ID {
	DIMM_ID_A,
	DIMM_ID_B,
	DIMM_ID_C,
	DIMM_ID_D,
	DIMM_ID_E,
	DIMM_ID_F,
	DIMM_ID_G,
	DIMM_ID_H,
	DIMM_ID_I,
	DIMM_ID_J,
	DIMM_ID_K,
	DIMM_ID_L,
	DIMM_ID_MAX,
	DIMM_ID_UNKNOWN = 0xff,
};

enum DIMM_PRSNT_STATUS {
	DIMM_PRSNT,
	DIMM_NOT_PRSNT,
};

void start_get_dimm_info_thread();
void get_dimm_info_handler();
uint8_t sensor_num_map_dimm_id(uint8_t sensor_num);
int pal_get_spd_temp(uint8_t sensor_num, uint8_t *data);
int pal_get_pmic_pwr(uint8_t sensor_num, uint8_t *data);
void clear_unaccessible_dimm_data(uint8_t dimm_id);
int switch_i3c_dimm_mux(uint8_t i3c_ctrl_mux_data);
int all_brocast_ccc(I3C_MSG *i3c_msg);
int init_dimm_prsnt_status();
uint8_t get_dimm_present(uint8_t dimm_id);

#endif
