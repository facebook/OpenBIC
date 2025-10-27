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

#define MAX_LEN_I3C_GET_PMIC_ERR 47
#define MAX_LEN_I3C_GET_PMIC_PWR 1
#define MAX_LEN_I3C_GET_SPD_TEMP 2

#define DIMM_I3C_MUX_CONTROL_OFFSET 0x0C
#define I3C_MUX_BIC_TO_DIMMA_TO_F 0x02
#define I3C_MUX_BIC_TO_DIMMG_TO_L 0x03
#define I3C_MUX_CPU_TO_DIMM 0x00

#define DIMM_I3C_MUX_STATUS_OFFSET 0x0D
#define I3C_MUX_STATUS_ENABLE_FUNCTION_CHECK 7
#define I3C_MUX_STATUS_PD_SPD_1_REMOTE_EN 6
#define I3C_MUX_STATUS_PD_SPD_2_REMOTE_EN 5
#define I3C_MUX_STATUS_SPD_MASK 0x60

#define I3C_DIMM_MUTEX_TIMEOUT_MS 1000
#define GET_DIMM_INFO_TIME_MS 1000
#define GET_DIMM_INFO_STACK_SIZE 2304

#define SPD_MFG_START 0x200
#define SPD_MFG_TOTAL_LEN 128
#define SPD_TEMP_DATA_LEN 2
#define PMIC_PWR_DATA_LEN 1
#define SPD_RAW_LEN 0x50

#define SPD_TYPE 0x00 /* 0x02                              (1)  */
#define SPD_SZ_B4 0x01 /* 0x04 (size_die)                    (1)  */
#define SPD_SZ_B6 0x02 /* 0x06 (device_width)                (1)  */
#define SPD_SPEED_L 0x03 /* 0x14                               (1)  */
#define SPD_SPEED_H 0x04 /* 0x15                               (1)  */
#define SPD_PMICVEN_L 0x05 /* 0xC6                               (1)  */
#define SPD_PMICVEN_H 0x06 /* 0xC7                               (1)  */
#define SPD_REGVEN_L 0x07 /* 0xF0                               (1)  */
#define SPD_REGVEN_H 0x08 /* 0xF1                               (1)  */
#define SPD_SZ_BEA 0x09 /* 0xEA (pkg_rank)                    (1)  */
#define SPD_SZ_BEB 0x0A /* 0xEB (bus_width/ch)                (1)  */
#define SPD_MFG_ID_L 0x0B /* 0x200                              (1)  */
#define SPD_MFG_ID_H 0x0C /* 0x201                              (1)  */
#define SPD_MFG_LOC 0x0D /* 0x202                              (1)  */
#define SPD_MFG_YY 0x0E /* 0x203 (year)                       (1)  */
#define SPD_MFG_WW 0x0F /* 0x204 (week)                       (1)  */
#define SPD_SN_OFF 0x10 /* 0x205..0x208                       (4)  */
#define SPD_SN_LEN 4
#define SPD_PN_OFF 0x14 /* 0x209..0x226                       (30) */
#define SPD_PN_LEN 30
#define SPD_PAYLOAD_LEN (SPD_C_PN_OFF + SPD_C_PN_LEN) /* = 50 bytes */
#ifndef SPD_OEM_PRESENT
#define SPD_OEM_PRESENT 0x4E /* 1 byte: 1=present, 0=not present */
#endif
#ifndef SPD_OEM_STATUS
#define SPD_OEM_STATUS 0x4F /* 1 byte: bit0=SPD compact ready */
#endif
#define SPD_OEM_STATUS_SPD_READY 0x01

typedef struct dimm_info {
	uint8_t is_present;
	bool is_ready_monitor;
	bool is_spd_raw_ready;
	uint8_t spd_raw_data[SPD_RAW_LEN];
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

enum DIMM_DEVICE_TYPE {
	DIMM_SPD = 0x00,
	DIMM_SPD_NVM = 0x01,
	DIMM_PMIC = 0x02,
	DIMM_SPD_CACHE = 0x03,
};
extern struct k_mutex i3c_dimm_mutex;
extern uint8_t spd_i3c_addr_list[];
extern uint8_t pmic_i3c_addr_list[];

void start_get_dimm_info_thread();
void get_dimm_info_handler();
uint8_t sensor_num_map_dimm_id(uint8_t sensor_num);
int pal_get_spd_temp(uint8_t sensor_num, uint8_t *data);
int pal_get_pmic_pwr(uint8_t sensor_num, uint8_t *data);
void clear_unaccessible_dimm_data(uint8_t dimm_id);
int switch_i3c_dimm_mux(uint8_t i3c_ctrl_mux_data);
int check_i3c_dimm_mux(uint8_t *status_data);
int all_brocast_ccc(I3C_MSG *i3c_msg);
int init_dimm_prsnt_status();
uint8_t get_dimm_present(uint8_t dimm_id);
void set_spd_raw_ready(uint8_t dimm_id, bool ready);
bool get_spd_raw_ready(uint8_t dimm_id);
int plat_get_spd_raw(uint8_t dimm_id, uint8_t **buf_out, bool *ready_out);

#endif
