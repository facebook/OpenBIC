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

#ifndef PLAT_PLDM_OEM_H
#define PLAT_PLDM_OEM_H

#include "pldm_oem.h"

enum CXL_COREDUMP_CMD_TYPE {
	READ_CXL_FLASH_START = 0x00,
	READ_CXL_FLASH_DATA = 0x01,
	READ_CXL_FLASH_END = 0x02
};

struct _cxl_read_flash_req {
	uint8_t iana[IANA_LEN];
	uint8_t transfer_flag;
	uint8_t cxl_comp_id;
	uint32_t data_offset;
	uint16_t data_len;
} __attribute__((packed));

struct _cxl_read_flash_resp {
	uint8_t completion_code;
	uint8_t iana[IANA_LEN];
	uint16_t data_len;
	uint8_t data[];
} __attribute__((packed));

uint8_t read_flash_data_cmd(void *mctp_inst, uint8_t *req_buf, uint16_t req_len,
			    uint8_t instance_id, uint8_t *resp_buf, uint16_t *resp_len,
			    void *ext_params);

#endif