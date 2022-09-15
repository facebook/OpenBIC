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
#define MONITOR_PMIC_ERROR_TIME_MS (30 * 1000) // 30s

#define MAX_LEN_GET_PMIC_ERROR_INFO 6 //include R05,  R06,  R08,  R09,  R0A,  R0B

#define MAX_COUNT_DIMM 4
#define MAX_COUNT_PMIC_ERROR_TYPE 17

void start_monitor_pmic_error_thread();
void monitor_pmic_error_handler();
int compare_pmic_error(uint8_t dimm_id, uint8_t *pmic_err_data);
int get_dimm_info(uint8_t dimm_id, uint8_t *bus, uint8_t *addr);
void add_pmic_error_sel(uint8_t dimm_id, uint8_t error_type);

#endif
