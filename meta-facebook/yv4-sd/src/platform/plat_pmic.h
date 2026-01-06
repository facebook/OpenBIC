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

#define MONITOR_PMIC_ERROR_STACK_SIZE 1536
#define MONITOR_PMIC_ERROR_TIME_MS (3 * 1000) // 3s
#define MONITOR_PMIC_FATAL_ERROR_TIME_MS (3 * 1000) // 3s
#define MONITOR_PMIC_NON_FATAL_ERROR_TIME_MS (9 * 1000) // 9s
#define CLEAR_MTP_DELAY_MS 200
// The PMIC needs a total of 200ms from CAMP signal assertion to complete the write operation
#define READ_PMIC_CRITICAL_ERROR_MS 200

#define CLEAR_MTP_RETRY_MAX 5
#define MAX_COUNT_PMIC_ERROR_OFFSET 7
#define MAX_COUNT_PMIC_ERROR_TYPE 17
#define MAX_LEN_PMIC_CRITICAL_ERROR_INDEX 11
enum {
    PMIC_ERROR_LEVEL_NONE = 0,
    PMIC_ERROR_LEVEL_NON_FATAL,
    PMIC_ERROR_LEVEL_FATAL
};

#define PMIC_ERROR_STATUS_START_OFFSET 0x04
#define PMIC_CLEAR_STATUS_BITS4_OFFSET 0x14
#define PMIC_CLEAR_ERROR_INJECTION_CFG_OFFSET 0x35
#define PMIC_VENDOR_PASSWORD_CONTROL_OFFSET 0x39

void start_monitor_pmic_error_thread();
void monitor_pmic_error_via_i3c_handler();
int compare_pmic_error(uint8_t dimm_id, uint8_t *pmic_err_data, uint8_t pmic_err_data_len,
		       uint8_t level);
int get_pmic_fault_status();
void read_pmic_error_when_dc_off();
void clear_pmic_error(uint8_t dimm_id);
void add_pmic_error_sel(uint8_t dimm_id, uint8_t error_type);
int get_pmic_error_data(uint8_t dimm_id, uint8_t *buffer, uint8_t is_need_check_post_status);
int get_pmic_error_data_one(uint8_t dimm_id, uint8_t *buffer, uint8_t is_need_check_post_status);

void init_pmic_event_work();

#endif
