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

#define MONITOR_PMIC_ERROR_STACK_SIZE 1024
#define MONITOR_PMIC_ERROR_TIME_MS (3 * 1000) // 3s

#define MAX_COUNT_PMIC_ERROR_TYPE 17

enum READ_PMIC_ERROR_PATH {
	READ_PMIC_ERROR_VIA_ME,
	READ_PMIC_ERROR_VIA_I3C,
};

void start_monitor_pmic_error_thread();
void monitor_pmic_error_handler();
int compare_pmic_error(uint8_t dimm_id, uint8_t *pmic_err_data, uint8_t pmic_err_data_len,
		       uint8_t read_path);
void add_pmic_error_sel(uint8_t dimm_id, uint8_t error_type);
void read_pmic_error_when_dc_off();

#endif
