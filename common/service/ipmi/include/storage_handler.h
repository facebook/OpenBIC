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

#ifndef STORAGE_HANDLER_H
#define STORAGE_HANDLER_H

#include "ipmi.h"

struct sel_event_record {
	uint16_t record_id;
	uint8_t record_type;
	uint32_t timestamp;
	uint8_t gen_id[2];
	uint8_t evm_rev;
	uint8_t sensor_type;
	uint8_t sensor_num;

	union {
		struct {
			uint8_t event_type : 7;
			uint8_t event_dir : 1;
		};
		uint8_t event_dir_type;
	};
	uint8_t event_data[3];
} __attribute__((__packed__));

struct oem_sel_event_record {
	uint16_t record_id;
	uint8_t record_type;
	uint8_t general_info;
	uint32_t timestamp;
	uint8_t failure_event_type;
	uint8_t rsv_1;
	uint8_t rsv_2;
	uint8_t rsv_3;
	uint8_t failure_event_details;
	uint8_t pxe_http_fail_type;
	uint8_t pxe_http_error_code;
	uint8_t rsv_4;
} __attribute__((__packed__));

struct ipmi_storage_add_sel_req {
	union {
		struct sel_event_record event;
		struct oem_sel_event_record oem_event;
	};
} __attribute__((__packed__));

struct ipmi_storage_add_sel_resp {
	uint8_t record_id[2];
} __attribute__((__packed__));

uint8_t get_add_sel_target_interface();

void STORAGE_GET_FRUID_INFO(ipmi_msg *msg);
void STORAGE_READ_FRUID_DATA(ipmi_msg *msg);
void STORAGE_WRITE_FRUID_DATA(ipmi_msg *msg);
void STORAGE_RSV_SDR(ipmi_msg *msg);
void STORAGE_GET_SDR(ipmi_msg *msg);
void STORAGE_ADD_SEL(ipmi_msg *msg);

void IPMI_Storage_handler(ipmi_msg *msg);

#endif
