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

#ifndef PLAT_DEV
#define PLAT_DEV

#include <stdint.h>
#include "sensor.h"
#include "cci.h"

#define CXL_IOEXP_INIT_RETRY_COUNT 5

typedef struct _pm8702_dev_info {
	bool is_init;
	cci_fw_info_resp dev_info;
} pm8702_dev_info;

typedef struct _cxl_vr_fw_info {
	uint8_t checksum[4];
	uint8_t remaining_write;
	uint8_t vendor;
	bool is_init;
} cxl_vr_fw_info;

extern pm8702_dev_info pm8702_table[];
extern cxl_vr_fw_info cxl_vr_info_table[];

bool cxl_single_ioexp_alert_reset(uint8_t ioexp_name, bool is_mutex);
int cxl_ioexp_init(uint8_t cxl_channel);
void cxl_mb_status_init(uint8_t cxl_id);
bool pal_init_pm8702_info(uint8_t cxl_id);
bool pal_pm8702_command_handler(uint8_t cxl_id, uint16_t opcode, uint8_t *data_buf, int data_len,
				uint8_t *response, uint8_t *response_len);
bool pal_get_pm8702_hbo_status(uint8_t cxl_id, uint8_t *resp_buf, uint8_t *resp_len);
bool pal_pm8702_transfer_fw(uint8_t cxl_id, uint8_t *req_buf, int req_len);
bool pal_set_pm8702_active_slot(uint8_t cxl_id, uint8_t *req_buf, int req_len);
void init_cxl_card_ioexp(uint8_t cxl_id);
void clear_cxl_card_cache_value(uint8_t cxl_id);
void init_ssd_power_fault_work();
void clear_ssd_power_fault_flag();

#endif
