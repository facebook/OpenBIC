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
#include <stdint.h>
#include <zephyr.h>

typedef uint8_t (*mctp_ctrl_cmd_fn)(void *, uint8_t *, uint16_t, uint8_t *, uint16_t *, void *);

typedef struct _mctp_ctrl_cmd_handler {
	uint8_t cmd_code;
	mctp_ctrl_cmd_fn fn;
} mctp_ctrl_cmd_handler_t;

#define MCTP_BASE_LINE_UNIT 64

#define MCTP_CTRL_CMD_SET_ENDPOINT_ID 0x01
#define MCTP_CTRL_CMD_GET_ENDPOINT_ID 0x02

#define MCTP_CTRL_CMD_GET_MESSAGE_TYPE_SUPPORT 0x05

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

struct _get_message_type_resp {
	uint8_t completion_code;
	uint8_t type_count;
	uint8_t type_number[0];
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
