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

#ifndef SBMR_H
#define SBMR_H

#include "plat_def.h"
#ifdef ENABLE_SBMR

#include <stdint.h>
#include "ipmb.h"
typedef struct __attribute__((packed)) {
	union {
		struct {
			uint8_t type;
			uint16_t rsvd;
			uint8_t severity;
		} __attribute__((packed));
		uint32_t status_code;
	};
	union {
		struct {
			uint16_t operation;
			uint8_t sub_class;
			uint8_t class;
		} __attribute__((packed));
		uint32_t efi_status_code;
	};
	uint8_t inst;
} sbmr_boot_progress_code_t;

#define SBMR_POSTCODE_SIZE sizeof(sbmr_boot_progress_code_t)

struct sbmr_cmd_send_boot_progress_code_req {
	uint8_t group_ext_def_body;
	sbmr_boot_progress_code_t code;
} __attribute__((packed));

struct sbmr_cmd_send_boot_progress_code_resp {
	uint8_t completion_code;
	uint8_t group_ext_def_body;
} __attribute__((packed));

struct sbmr_cmd_get_boot_progress_code_req {
	uint8_t group_ext_def_body;
} __attribute__((packed));

struct sbmr_cmd_get_boot_progress_code_resp {
	uint8_t completion_code;
	uint8_t group_ext_def_body;
	sbmr_boot_progress_code_t code;
} __attribute__((packed));

uint16_t copy_sbmr_read_buffer(uint16_t start, uint16_t length, uint8_t *buffer,
			       uint16_t buffer_len);
void sbmr_postcode_insert(sbmr_boot_progress_code_t boot_progress_code);
void reset_sbmr_postcode_buffer();
bool sbmr_get_9byte_postcode_ok();
void sbmr_reset_9byte_postcode_ok();
bool smbr_cmd_handler(ipmi_msg *msg);

#endif /* ENABLE_SBMR */

#endif /* SBMR_H */
