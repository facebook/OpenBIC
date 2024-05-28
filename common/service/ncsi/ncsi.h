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

#ifndef _NCSI_H
#define _NCSI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mctp.h"

#define MONITOR_THREAD_STACK_SIZE 1024

#define NCSI_PAYLOAD_LENGTH_MAX 0xFFF
#define NCSI_PAYLOAD_LENGTH_LOW_MASK 0xFF
#define NCSI_PAYLOAD_LENGTH_HIGH_MASK 0xF00
typedef enum {
	NCSI_OEM = 0x50,
} NCSI_CMD;

enum ncsi_response_codes {
	NCSI_COMMAND_COMPLETED = 0x0000,
	NCSI_COMMAND_FAILED = 0x0001,
	NCSI_COMMAND_UNAVAILABLE = 0x0002,
	NCSI_COMMAND_UNSUPPORTED = 0x0003,
	NCSI_DELAYED_RESPONSE = 0x0004,
};

enum ncsi_reason_codes {
	NCSI_NO_ERROR = 0x0000,
	NCSI_INTERFACE_INITIALIZAION_REQUIRED = 0x0001,
	NCSI_PARAMETER_IS_INVALID = 0x0002,
	NCSI_CHANNEL_NOT_READY = 0x0003,
	NCSI_PACKAGE_NOT_READY = 0x0004,
	NCSI_INVALID_PAYLOAD_LENGTH = 0x0005,
	NCSI_INFORMATION_NOT_AVAILABLE = 0x0006,
	NCSI_INTERVENTION_REQUIRED = 0x0007,
	NCSI_LINK_COMMANDS_FAILED_HARDWARE_ACCESS_ERROR = 0x0008,
	NCSI_COMMAND_TIMEOUT = 0x0009,
	NCSI_SECONDARY_DEVICE_NOT_POWERED = 0x000A,
	NCSI_UNSUPPORTED_COMMAND_TYPE = 0x7FFF,
};

enum ncsi_command_codes {
	NCSI_COMMAND_CLEAR_INITIAL_STATE = 0x00,
	NCSI_COMMAND_GET_INFINIBAND_LINK_STATUS = 0x38,
	NCSI_COMMAND_OEM = 0x50,
};

enum ncsi_command_rq {
	NCSI_COMMAND_REQUEST = 0x00,
	NCSI_COMMAND_RESPONSE = 0x01,
};

typedef struct __attribute__((packed)) {
	uint8_t msg_type : 7;
	uint8_t ic : 1;
	uint8_t mc_id;
	uint8_t header_revision;
	uint8_t reserved_1;
	uint8_t inst_id;

	union {
		struct {
			uint8_t command : 7;
			uint8_t rq : 1;
		};
		uint8_t packet_type;
	};

	uint8_t channel_id;
	uint8_t payload_length_high : 4;
	uint8_t reserved_2 : 4;
	uint8_t payload_length_low;
	uint64_t reserved_3;
} ncsi_hdr;

typedef struct _ncsi_msg {
	ncsi_hdr hdr;
	uint8_t *buf;
	uint16_t len;
	mctp_ext_params ext_params;
	void (*recv_resp_cb_fn)(void *, uint8_t *, uint16_t);
	void *recv_resp_cb_args;
	uint16_t timeout_ms;
	void (*timeout_cb_fn)(void *);
	void *timeout_cb_fn_args;
} ncsi_msg;

typedef struct _ncsi {
	/* ncsi message response timeout prcoess resource */
	k_tid_t monitor_task;
	struct k_thread thread_data;
	K_KERNEL_STACK_MEMBER(monitor_thread_stack, MONITOR_THREAD_STACK_SIZE);

	/* store the msg that are not yet to receive the response */
	sys_slist_t wait_recv_resp_list;
	struct k_mutex wait_recv_resp_list_mutex;

	/* store the msg that are not yet to send the response */
	sys_slist_t wait_send_resp_list;
	struct k_mutex wait_send_resp_list_mutex;

	void *interface; /* the ncsi module over which interface, as mctp */
	uint8_t user_idx; /* the alias index for this ncsi instance from application
                       layer */
} ncsi_t;

/* send the ncsi command message through mctp */
uint8_t mctp_ncsi_send_msg(void *mctp_p, ncsi_msg *msg);
uint8_t mctp_ncsi_cmd_handler(void *mctp_p, uint8_t *buf, uint32_t len, mctp_ext_params ext_params);
uint16_t mctp_ncsi_read(void *mctp_p, ncsi_msg *msg, uint8_t *rbuf, uint16_t rbuf_len);

#ifdef __cplusplus
}
#endif

#endif /* _NCSI_H */
