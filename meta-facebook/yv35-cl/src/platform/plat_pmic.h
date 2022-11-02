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

#ifndef PLAT_PMIC_H
#define PLAT_PMIC_H

#include <stdint.h>

#define MONITOR_PMIC_ERROR_STACK_SIZE 4096
#define MONITOR_PMIC_ERROR_TIME_MS (3 * 1000) // 3s

#define MAX_LEN_GET_PMIC_ERROR_INFO 6
#define MAX_LEN_I3C_GET_PMIC_ERROR_INFO 7

#define MAX_COUNT_DIMM 6
#define MAX_COUNT_PMIC_ERROR_TYPE 17

#define I3C_MUX_TO_BIC 0x1
#define I3C_MUX_TO_CPU 0x0
#define DIMM_MUX_TO_DIMM_A0A1A3 0x0
#define DIMM_MUX_TO_DIMM_A4A6A7 0x1

#define CL_CPLD_BMC_CHANNEL_ADDR 0x1E // 8 bits
#define PMIC_FAULT_STATUS_OFFSET 0x0B
#define DIMM_I3C_MUX_CONTROL_OFFSET 0x0B

enum DIMM_ID {
	DIMM_ID_A0 = 0,
	DIMM_ID_A2,
	DIMM_ID_A3,
	DIMM_ID_A4,
	DIMM_ID_A6,
	DIMM_ID_A7,
};

enum READ_PMIC_ERROR_PATH {
	READ_PMIC_ERROR_VIA_ME,
	READ_PMIC_ERROR_VIA_I3C,
};

void start_monitor_pmic_error_thread();
void monitor_pmic_error_handler();
int compare_pmic_error(uint8_t dimm_id, uint8_t *pmic_err_data, uint8_t read_path);
int get_dimm_info(uint8_t dimm_id, uint8_t *bus, uint8_t *addr);
void add_pmic_error_sel(uint8_t dimm_id, uint8_t error_type);
int switch_i3c_dimm_mux(uint8_t i3c_mux_position, uint8_t dimm_mux_position);
void read_pmic_error_via_i3c();
int get_pmic_fault_status();

#endif
