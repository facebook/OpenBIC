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

#ifndef _PLDM_BASE_H
#define _PLDM_BASE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pldm.h"
#include <stdint.h>

/* commands of pldm type 0x00 : PLDM_TYPE_CTRL_DISCOV */
#define PLDM_BASE_CMD_CODE_SETTID 0x01
#define PLDM_BASE_CMD_CODE_GETTID 0x02
#define PLDM_BASE_CMD_CODE_GET_PLDM_VER 0x03
#define PLDM_BASE_CMD_CODE_GET_PLDM_TYPE 0x04
#define PLDM_BASE_CMD_CODE_GET_PLDM_CMDS 0x05

#define DEFAULT_TID 0x86

#define GET_PLDM_TYPE_BUF_SIZE 8
#define GET_PLDM_COMMNAD_BUF_SIZE 32

#define INVALID_PLDM_TYPE_IN_REQUEST_DATA 0x83
#define INVALID_PLDM_VERSION_IN_REQUEST_DATA 0x84

enum pldm_completion_codes {
	PLDM_SUCCESS = 0x00,
	PLDM_ERROR = 0x01,
	PLDM_ERROR_INVALID_DATA = 0x02,
	PLDM_ERROR_INVALID_LENGTH = 0x03,
	PLDM_ERROR_NOT_READY = 0x04,
	PLDM_ERROR_UNSUPPORTED_PLDM_CMD = 0x05,
	PLDM_ERROR_INVALID_PLDM_TYPE = 0x20,
	PLDM_INVALID_TRANSFER_OPERATION_FLAG = 0x21,
	/* Use reserved region for oem define */
	PLDM_LATER_RESP = 0x30,
};

enum pldm_transport_protocol_type {
	PLDM_TRANSPORT_PROTOCOL_TYPE_MCTP = 0x00,
	PLDM_TRANSPORT_PROTOCOL_TYPE_NCSI = 0x01,
	PLDM_TRANSPORT_PROTOCOL_TYPE_OEM = 0xFF,
};

struct _set_tid_req {
	uint8_t tid;
} __attribute__((packed));

struct _set_tid_resp {
	uint8_t completion_code;
} __attribute__((packed));

struct _get_tid_resp {
	uint8_t completion_code;
	uint8_t tid;
} __attribute__((packed));

struct _get_pldm_types_resp {
	uint8_t completion_code;
	uint8_t pldm_types[GET_PLDM_TYPE_BUF_SIZE];
} __attribute__((packed));

struct _get_pldm_commands_req {
	uint8_t type;
	uint32_t version;
} __attribute__((packed));

struct _get_pldm_commands_resp {
	uint8_t completion_code;
	uint8_t pldm_commands[GET_PLDM_COMMNAD_BUF_SIZE];
} __attribute__((packed));

uint8_t pldm_base_handler_query(uint8_t code, void **ret_fn);

#ifdef __cplusplus
}
#endif

#endif /* _PLDM_BASE_H */