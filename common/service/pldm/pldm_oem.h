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

#ifndef _PLDM_OEM_H
#define _PLDM_OEM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pldm.h"
#include <stdint.h>

#define IANA_LEN 0x03

/* commands of pldm type 0x3F : PLDM_TYPE_OEM */
#define PLDM_OEM_CMD_ECHO 0x00
#define PLDM_OEM_IPMI_BRIDGE 0x01
#define PLDM_OEM_WRITE_FILE_IO 0x02
#define PLDM_OEM_READ_FILE_IO 0x03

enum cmd_type {
	POST_CODE = 0x00,
};

struct _cmd_echo_req {
	uint8_t iana[IANA_LEN];
	uint8_t first_data;
} __attribute__((packed));

struct _cmd_echo_resp {
	uint8_t completion_code;
	uint8_t iana[IANA_LEN];
	uint8_t first_data;
} __attribute__((packed));

struct _ipmi_cmd_req {
	uint8_t iana[IANA_LEN];
	uint8_t netfn_lun;
	uint8_t cmd;
	uint8_t first_data;
} __attribute__((packed));

struct _ipmi_cmd_resp {
	uint8_t completion_code;
	uint8_t iana[IANA_LEN];
	uint8_t netfn_lun;
	uint8_t cmd;
	uint8_t ipmi_comp_code;
	uint8_t first_data;
} __attribute__((packed));

struct pldm_oem_write_file_io_req {
	uint8_t cmd_code;
	uint32_t data_length;
	uint8_t messages[];
} __attribute__((packed));

struct pldm_oem_write_file_io_resp {
	uint8_t completion_code;
} __attribute__((packed));

uint8_t check_iana(const uint8_t *iana);
uint8_t set_iana(uint8_t *buf, uint8_t buf_len);

uint8_t pldm_oem_handler_query(uint8_t code, void **ret_fn);

#ifdef __cplusplus
}
#endif

#endif /* _PLDM_OEM_H */