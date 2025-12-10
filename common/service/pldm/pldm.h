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

#ifndef _PLDM_H
#define _PLDM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mctp.h"
#include "pldm_base.h"
#include "pldm_oem.h"
#include "pldm_monitor.h"
#include "pldm_firmware_update.h"
#include "pldm_state_set.h"
#include "pldm_smbios.h"
#include "ipmb.h"

#define MONITOR_THREAD_STACK_SIZE 1024

#define PLDM_MAX_DATA_SIZE_DEFAULT 512

#ifndef PLDM_MAX_DATA_SIZE
#define PLDM_MAX_DATA_SIZE PLDM_MAX_DATA_SIZE_DEFAULT
#endif
typedef uint8_t (*pldm_cmd_proc_fn)(void *, uint8_t *, uint16_t, uint8_t, uint8_t *, uint16_t *,
				    void *);

typedef enum {
	PLDM_TYPE_BASE = 0x00,
	PLDM_TYPE_SMBIOS = 0x01,
	PLDM_TYPE_PLAT_MON_CTRL,
	PLDM_TYPE_BIOS_CTRL_CONF,
	PLDM_TYPE_FW_UPDATE = 0x05,
	PLDM_TYPE_OEM = 0x3F
} PLDM_TYPE;

enum PLDM_MESSAGE_TYPE {
	PLDM_RESPONSE,
	PLDM_REQUEST,
};

typedef struct _pldm_cmd_handler {
	uint8_t cmd_code;
	pldm_cmd_proc_fn fn;
} pldm_cmd_handler;

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

	uint8_t pldm_type : 6;
	uint8_t ver : 2;
	uint8_t cmd;
} pldm_hdr;

typedef struct __attribute__((packed)) {
	pldm_hdr common_hdr;
	uint8_t resp_comp_code;
} pldm_resp_hdr;

typedef struct _pldm_msg {
	pldm_hdr hdr;
	uint8_t *buf;
	uint16_t len;
	mctp_ext_params ext_params;
	void (*recv_resp_cb_fn)(void *, uint8_t *, uint16_t);
	void *recv_resp_cb_args;
	uint16_t timeout_ms;
	void (*timeout_cb_fn)(void *);
	void *timeout_cb_fn_args;
} pldm_msg;

typedef struct _pldm {
	/* pldm message response timeout prcoess resource */
	k_tid_t monitor_task;
	struct k_thread thread_data;
	K_KERNEL_STACK_MEMBER(monitor_thread_stack, MONITOR_THREAD_STACK_SIZE);

	/* store the msg that are not yet to receive the response */
	sys_slist_t wait_recv_resp_list;
	struct k_mutex wait_recv_resp_list_mutex;

	/* store the msg that are not yet to send the response */
	sys_slist_t wait_send_resp_list;
	struct k_mutex wait_send_resp_list_mutex;

	void *interface; /* the pldm module over which interface, as mctp */
	uint8_t user_idx; /* the alias index for this pldm instance from application
                       layer */
} pldm_t;

struct pldm_variable_field {
	uint8_t *ptr;
	size_t length;
};

struct _pldm_ipmi_cmd_resp {
	uint8_t completion_code;
	uint8_t netfn_lun;
	uint8_t cmd;
	uint8_t ipmi_comp_code;
	uint8_t first_data;
} __attribute__((packed));

/* the pldm command handler */
uint8_t mctp_pldm_cmd_handler(void *mctp_p, uint8_t *buf, uint32_t len, mctp_ext_params ext_params);

/* send the pldm command message through mctp */
uint8_t mctp_pldm_send_msg(void *mctp_p, pldm_msg *msg);
int pldm_send_ipmi_response(uint8_t interface, ipmi_msg *msg);
int pldm_send_ipmi_request(ipmi_msg *msg);

uint16_t mctp_pldm_read(void *mctp_p, pldm_msg *msg, uint8_t *rbuf, uint16_t rbuf_len);

pldm_t *pldm_init(void *interface, uint8_t user_idx);

uint8_t get_supported_pldm_type(uint8_t *buf, uint8_t buf_size);

uint8_t get_supported_pldm_commands(PLDM_TYPE type, uint8_t *buf, uint8_t buf_size);

#ifdef __cplusplus
}
#endif

#endif /* _PLDM_H */
