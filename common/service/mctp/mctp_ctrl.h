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

#ifndef _MCTP_CTRL_H
#define _MCTP_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mctp.h"
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>

typedef uint8_t (*mctp_ctrl_cmd_fn)(void *, uint8_t *, uint16_t, uint8_t *, uint16_t *, void *);

typedef struct _mctp_ctrl_cmd_handler {
	uint8_t cmd_code;
	mctp_ctrl_cmd_fn fn;
} mctp_ctrl_cmd_handler_t;

#define MCTP_BASE_LINE_UNIT 64

#define MCTP_CTRL_CMD_SET_ENDPOINT_ID 0x01
#define MCTP_CTRL_CMD_GET_ENDPOINT_ID 0x02
#define MCTP_CTRL_CMD_GET_ENDPOINT_UUID 0x03

#define MCTP_CTRL_CMD_GET_VERSION_SUPPORT 0x04
#define MCTP_CTRL_CMD_GET_MESSAGE_TYPE_SUPPORT 0x05
#define MCTP_CTRL_CMD_RESOLVE_ENDPOINT_ID 0x07

#define MCTP_CTRL_CMD_GET_ENDPOINT_ID_REQ_LEN 0x00

#define MCTP_CTRL_READ_STATUS_SUCCESS 0x00
#define MCTP_CTRL_READ_STATUS_CC_ERROR 0x01
#define MCTP_CTRL_READ_STATUS_TIMEOUT 0x02

/*
 * MCTP Control Completion Codes
 * See DSP0236 v1.3.0 Table 13.
 */
#define MCTP_CTRL_CC_SUCCESS 0x00
#define MCTP_CTRL_CC_ERROR 0x01
#define MCTP_CTRL_CC_ERROR_INVALID_DATA 0x02
#define MCTP_CTRL_CC_ERROR_INVALID_LENGTH 0x03
#define MCTP_CTRL_CC_ERROR_NOT_READY 0x04
#define MCTP_CTRL_CC_ERROR_UNSUPPORTED_CMD 0x05
/* DSP0236 Table 18 - Get MCTP Version Support response, not the generic
 * completion code table above (this one's command-specific). */
#define MCTP_CTRL_CC_ERROR_MSG_TYPE_NOT_SUPPORTED 0x80

/* DSP0236 Table 18's "Message Type Number" request selector - distinct
 * from (though numerically overlapping) the runtime `enum message_type`
 * above. 0xFF/0x00/0x01 are the ones this board can honestly answer for:
 * the MCTP base spec itself, the MCTP control protocol, and DSP0241
 * (PLDM over MCTP Binding Specification) respectively - see
 * mctp_ctrl_cmd_get_version_support(). Everything else this command
 * defines (0x7E/0x7F vendor-defined, 0x02/0x03 DSP0261) isn't
 * implemented on this board.
 */
#define MCTP_VERSION_SUPPORT_SEL_BASE_SPEC 0xFF
#define MCTP_VERSION_SUPPORT_SEL_CONTROL_PROTOCOL 0x00
#define MCTP_VERSION_SUPPORT_SEL_PLDM_BINDING 0x01

#define SET_EID_REQ_OP_SET_EID 0x00
#define SET_EID_REQ_OP_FORCE_EID 0x01

struct _set_eid_req {
	uint8_t op;
	uint8_t eid;
} __attribute__((packed));

struct _set_eid_resp {
	uint8_t completion_code;
	uint8_t status;
	uint8_t eid;
	uint8_t eid_pool_size;
} __attribute__((packed));

enum endpoint_type {
	SIMPLE_ENDPOINT,
	BRIDGE,
};

enum eid_type {
	DYNAMIC_EID,
	STATIC_EID,
};

/*
Reference from DSP0239_1.3.0 Table 1
*/
enum message_type {
	TYPE_MCTP_CONTROL,
	TYPE_PLDM,
	TYPE_MAX_SIZE,
};

/* Scratch buffer size for mctp_ctrl_cmd_get_message_type_support()'s
 * load_mctp_support_types() call - a board hook can report any real
 * DSP0239 message type (not just the two named above; e.g. this
 * board's Vendor Defined - PCI test type), so this isn't sized off
 * `enum message_type`. Generous headroom over what any real board
 * hook here reports (2-3 types). */
#define MCTP_SUPPORTED_MSG_TYPE_MAX 8

struct _get_message_type_resp {
	uint8_t completion_code;
	uint8_t type_count;
	uint8_t type_number[0];
} __attribute__((packed));

/* DSP0236 Table 18: each 4-byte version entry is
 * [major][minor][update][alpha] in that wire order (matches the
 * spec's worked examples, e.g. version "1.1.0" = 0xF1 0xF1 0xF0 0x00) -
 * written as explicit bytes here rather than a packed uint32_t so the
 * wire order can't get silently flipped by this target's endianness.
 * Single-digit BCD fields are padded with an 0xF nibble (e.g. major=1
 * is encoded 0xF1, not 0x01 - see the spec examples); update=0xFF
 * conventionally means "any/unspecified update version". */
#define MCTP_VERSION_ENTRY_LEN 4

struct _get_version_support_resp {
	uint8_t completion_code;
	uint8_t version_number_entry_count;
	uint8_t version_number_entry[0];
} __attribute__((packed));

#define MCTP_UUID_LEN 16

struct _get_uuid_resp {
	uint8_t completion_code;
	uint8_t uuid[MCTP_UUID_LEN];
} __attribute__((packed));

/* Board-level hook (mirrors plat_get_eid()'s pattern): supplies this
 * device's real, unique-per-device UUID/GUID (RFC4122 byte order, MSB
 * first). Default returns false ("not really available") rather than
 * fabricating one - see mcx-n9xx-evk's plat_mctp.c for a real
 * implementation backed by an actual per-chip hardware ID. */
bool plat_get_endpoint_uuid(uint8_t *uuid_buf);

/* DSP0236 Table 22 - Resolve Endpoint ID. This stack has no routing
 * table (see mctp_ctrl_cmd_resolve_endpoint_id()'s doc comment), so
 * the only EID this can ever honestly resolve is this endpoint's own
 * - there's nothing behind us to bridge to. */
struct _resolve_endpoint_id_req {
	uint8_t target_eid;
} __attribute__((packed));

struct _resolve_endpoint_id_resp {
	uint8_t completion_code;
	/* Matches target_eid when (as here) no bridging is required to
	 * reach it - DSP0236 12.10. */
	uint8_t bridge_eid;
	/* Medium-specific physical address (DSP0236: format defined by
	 * the physical transport binding) - one byte for this board's
	 * SMBus/I2C binding. */
	uint8_t phys_addr;
} __attribute__((packed));

struct _get_eid_resp {
	uint8_t completion_code;
	uint8_t eid;
	uint8_t eid_type : 2;
	uint8_t : 2;
	uint8_t endpoint_type : 2;
	uint8_t : 2;
	uint8_t medium_specific_info;
} __attribute__((packed));

typedef struct __attribute__((packed)) {
	uint8_t msg_type : 7;
	uint8_t ic : 1;

	union {
		struct {
			uint8_t inst_id : 5;
			uint8_t rsvd : 1;
			uint8_t d : 1;
			uint8_t rq : 1;
		};
		uint8_t req_d_id;
	};

	uint8_t cmd;
} mctp_ctrl_hdr;

typedef struct {
	mctp_ctrl_hdr hdr;
	uint8_t *cmd_data;
	uint16_t cmd_data_len;
	mctp_ext_params ext_params;
	void (*recv_resp_cb_fn)(void *, uint8_t *, uint16_t);
	void *recv_resp_cb_args;
	uint16_t timeout_ms;
	void (*timeout_cb_fn)(void *);
	void *timeout_cb_fn_args;
} mctp_ctrl_msg;

typedef struct _mctp_ctrl_resp_arg {
	struct k_msgq *msgq;
	uint8_t *read_buf;
	uint16_t read_len;
	uint16_t return_len;
} mctp_ctrl_resp_arg;

uint8_t mctp_ctrl_cmd_handler(void *mctp_p, uint8_t *buf, uint32_t len, mctp_ext_params ext_params);

uint8_t mctp_ctrl_send_msg(void *mctp_p, mctp_ctrl_msg *msg);
uint8_t mctp_ctrl_read(void *mctp_p, mctp_ctrl_msg *msg, uint8_t *read_buf, uint16_t read_len);

void plat_update_mctp_routing_table(uint8_t eid);

#ifdef __cplusplus
}
#endif

#endif /* _MCTP_CTRL_H */
