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
#define MONITOR_PMIC_ERROR_TIME_MS (3 * 1000) // 30s

#define MAX_LEN_GET_PMIC_ERROR_INFO 7

#define MAX_COUNT_DIMM 8
#define MAX_COUNT_PMIC_ERROR_TYPE 17
#define SYS_CPLD_BMC_CHANNEL_ADDR 0x1E // 8 bits
#define PMIC_FAULT_STATUS_OFFSET 0x0B

void start_monitor_pmic_error_thread();
void pmic_error_check();
void read_pmic_error_when_dc_off();

#endif
