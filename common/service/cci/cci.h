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

#ifndef _CCI_H
#define _CCI_H
#include "mctp.h"

typedef enum {
	CCI_GET_HEALTH_INFO = 0x4200,
	CCI_GET_FW_INFO = 0x0200,
} CCI_CMD;

/*CCI Request paypload length */
#define HEALTH_INFO_REQ_PL_LEN 0 /*Size Bytes*/
#define GET_FW_INFO_REQ_PL_LEN 0

/*CCI Response paypload length */
#define HEALTH_INFO_RESP_PL_LEN 18 /*Size Bytes*/
#define GET_FW_INFO_RESP_PL_LEN 80

struct _cci_handler_query_entry {
	CCI_CMD type;
	void (*handler_query)(uint8_t *, uint16_t);
};

typedef struct _cci_recv_resp_arg {
	struct k_msgq *msgq;
	uint8_t *rbuf;
	uint16_t rbuf_len;
	uint16_t return_len;
} cci_recv_resp_arg;

typedef struct __attribute__((packed)) {
	uint8_t msg_type : 7;
	uint8_t ic : 1;
	uint8_t cci_msg_req_resp; /* 0h = Request, 1h = Response*/
	uint8_t msg_tag;
	uint8_t cci_rsv;
	uint16_t op;
	int pl_len : 21;
	uint8_t rsv : 2;
	uint8_t BO : 1;
	uint16_t ret;
	uint16_t stat;
} mctp_cci_hdr;

typedef struct {
	mctp_cci_hdr hdr;
	uint8_t *pl_data;
	mctp_ext_params ext_params;
	void (*recv_resp_cb_fn)(void *, uint8_t *, uint16_t, uint16_t);
	void *recv_resp_cb_args;
	uint16_t timeout_ms;
	void (*timeout_cb_fn)(void *);
	void *timeout_cb_fn_args;
} mctp_cci_msg;

typedef uint8_t (*mctp_cci_cmd_fn)(void *, uint8_t *, uint16_t, uint8_t *, uint16_t *, void *);
typedef uint8_t (*cci_cmd_proc_fn)(void *, uint8_t *, uint16_t, uint8_t *, uint16_t *, void *);

typedef struct _mctp_cci_cmd_handler {
	uint8_t cmd_code;
	mctp_cci_cmd_fn fn;
} mctp_cci_cmd_handler_t;

#define DEV_TEMP_OFFSET 16
#define CCI_CC_INVALID_INPUT 0x0002

/*
 * CCI Return Codes
 */
#define CCI_CC_SUCCESS 0x0000
#define CCI_CC_INVALID_INPUT 0x0002
#define CCI_CC_PAYLOAD_INVALID_LEN 0x0016

/*
 * CCI Return Codes
 */
#define CCI_SUCCESS 0x0000
#define CCI_ERROR 0x0001
#define CCI_INVALID_TYPE 0x0002

typedef struct _wait_msg {
	sys_snode_t node;
	mctp *mctp_inst;
	int64_t exp_to_ms;
	mctp_cci_msg msg;
} wait_msg;

/*CCI command handler */
uint8_t mctp_cci_cmd_handler(void *mctp_p, uint8_t *buf, uint32_t len, mctp_ext_params ext_params);
void cci_read_resp_handler(void *args, uint8_t *rbuf, uint16_t rlen, uint16_t ret_code);
bool cci_get_chip_temp(void *mctp_p, mctp_ext_params ext_params, int16_t *chip_temp);

/* send CCI command message through mctp */
uint8_t mctp_cci_send_msg(void *mctp_p, mctp_cci_msg *msg);

uint16_t mctp_cci_read(void *mctp_p, mctp_cci_msg *msg, uint8_t *rbuf, uint16_t rbuf_len);

typedef struct __attribute__((__packed__)) {
	uint8_t health_status;
	uint8_t media_status;
	uint8_t additional_status;
	uint8_t life_used;
	int16_t dev_temp;
	uint32_t shutdown_cnt;
	uint32_t volatile_mem_err_cnt;
	uint32_t persistent_mem_err_cnt;
} cci_health_info_resp;

#endif /* _CCI_H */