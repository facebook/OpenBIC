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

#include "pldm.h"
#include "mctp.h"
#include <logging/log.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/printk.h>
#include <sys/slist.h>
#include <zephyr.h>
#include "libutil.h"
#include "ipmi.h"
#include "plat_def.h"

LOG_MODULE_REGISTER(pldm);

#define PLDM_HDR_INST_ID_MASK 0x1F
#define PLDM_MAX_INSTID_COUNT (PLDM_HDR_INST_ID_MASK + 1)
#define PLDM_MSG_CHECK_PER_MS 1000
#define PLDM_MSG_TIMEOUT_MS 5000
#define PLDM_RESP_MSG_PROC_MUTEX_TIMEOUT_MS 500
#define PLDM_TASK_NAME_MAX_SIZE 32
#define PLDM_MSG_MAX_RETRY 3

#define PLDM_READ_EVENT_SUCCESS BIT(0)
#define PLDM_READ_EVENT_TIMEOUT BIT(1)

typedef struct _wait_msg {
	sys_snode_t node;
	mctp *mctp_inst;
	int64_t exp_to_ms;
	pldm_msg msg;
} wait_msg;

struct _pldm_handler_query_entry {
	PLDM_TYPE type;
	uint8_t (*handler_query)(uint8_t, void **);
};

typedef struct _pldm_recv_resp_arg {
	struct k_msgq *msgq;
	uint8_t *rbuf;
	uint16_t rbuf_len;
	uint16_t return_len;
} pldm_recv_resp_arg;

static struct _pldm_handler_query_entry query_tbl[] = {
	{ PLDM_TYPE_BASE, pldm_base_handler_query },
	{ PLDM_TYPE_SMBIOS, pldm_smbios_handler_query },
	{ PLDM_TYPE_PLAT_MON_CTRL, pldm_monitor_handler_query },
	{ PLDM_TYPE_FW_UPDATE, pldm_fw_update_handler_query },
	{ PLDM_TYPE_OEM, pldm_oem_handler_query },
};

static K_MUTEX_DEFINE(wait_recv_resp_mutex);

static sys_slist_t wait_recv_resp_list = SYS_SLIST_STATIC_INIT(&wait_recv_resp_list);

static bool unregister_instid(void *mctp_p, uint8_t inst_num)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, false);

	if (inst_num >= PLDM_MAX_INSTID_COUNT) {
		LOG_ERR("Invalid instance number %d", inst_num);
		return false;
	}

	mctp *mctp_inst = (mctp *)mctp_p;
	if (!mctp_inst->pldm_inst_table) {
		LOG_ERR("Instance table not init!");
		return false;
	}
	if (!(mctp_inst->pldm_inst_table & BIT(inst_num))) {
		LOG_ERR("Inatant id %d not register yet!", inst_num);
		return false;
	}
	WRITE_BIT(mctp_inst->pldm_inst_table, inst_num, 0);

	return true;
}

static bool register_instid(void *mctp_p, uint8_t *inst_num)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, false);
	CHECK_NULL_ARG_WITH_RETURN(inst_num, false);

	mctp *mctp_inst = (mctp *)mctp_p;

	static uint8_t cur_inst_num = 0;

	int retry = 0;
	for (retry = 0; retry < PLDM_MAX_INSTID_COUNT; retry++) {
		if (!(mctp_inst->pldm_inst_table & BIT(cur_inst_num))) {
			break;
		}
		cur_inst_num = (cur_inst_num + 1) & PLDM_HDR_INST_ID_MASK;
	}

	if (retry == PLDM_MAX_INSTID_COUNT) {
		LOG_DBG("Inatant id %d not available!", cur_inst_num);
		return false;
	}

	WRITE_BIT(mctp_inst->pldm_inst_table, cur_inst_num, 1);
	*inst_num = cur_inst_num;

	cur_inst_num = (cur_inst_num + 1) & PLDM_HDR_INST_ID_MASK;

	return true;
}

void pldm_read_resp_handler(void *args, uint8_t *rbuf, uint16_t rlen)
{
	CHECK_NULL_ARG(args);
	CHECK_NULL_ARG(rbuf);

	if (!rlen)
		return;

	pldm_recv_resp_arg *recv_arg = (pldm_recv_resp_arg *)args;

	if (rlen > recv_arg->rbuf_len) {
		LOG_WRN("Response length(%d) is greater than buffer length(%d)!", rlen,
			recv_arg->rbuf_len);
		recv_arg->return_len = recv_arg->rbuf_len;
	} else {
		recv_arg->return_len = rlen;
	}
	memcpy(recv_arg->rbuf, rbuf, recv_arg->return_len);
	uint8_t status = PLDM_READ_EVENT_SUCCESS;
	k_msgq_put(recv_arg->msgq, &status, K_NO_WAIT);
}

static void pldm_read_timeout_handler(void *args)
{
	CHECK_NULL_ARG(args);

	struct k_msgq *msgq = (struct k_msgq *)args;
	uint8_t status = PLDM_READ_EVENT_TIMEOUT;
	k_msgq_put(msgq, &status, K_NO_WAIT);
}

/*
 * The return value is the read length from PLDM device
 */
uint16_t mctp_pldm_read(void *mctp_p, pldm_msg *msg, uint8_t *rbuf, uint16_t rbuf_len)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, 0);
	CHECK_NULL_ARG_WITH_RETURN(msg, 0);
	CHECK_NULL_ARG_WITH_RETURN(rbuf, 0);

	if (!rbuf_len)
		return 0;

	uint8_t event_msgq_buffer[1];
	struct k_msgq *event_msgq_p = (struct k_msgq *)malloc(sizeof(struct k_msgq));
	if (!event_msgq_p) {
		LOG_WRN("Failed to allocate event_msgq_p");
		return 0;
	}
	uint16_t ret_len = 0;

	k_msgq_init(event_msgq_p, event_msgq_buffer, sizeof(uint8_t), 1);

	pldm_recv_resp_arg *recv_arg_p = (pldm_recv_resp_arg *)malloc(sizeof(pldm_recv_resp_arg));
	if (!recv_arg_p) {
		SAFE_FREE(event_msgq_p);
		LOG_WRN("Failed to allocate recv_arg_p");
		return 0;
	}
	recv_arg_p->msgq = event_msgq_p;
	recv_arg_p->rbuf = rbuf;
	recv_arg_p->rbuf_len = rbuf_len;
	recv_arg_p->return_len = 0;

	msg->recv_resp_cb_fn = pldm_read_resp_handler;
	msg->recv_resp_cb_args = (void *)recv_arg_p;
	msg->timeout_cb_fn = pldm_read_timeout_handler;
	msg->timeout_cb_fn_args = (void *)event_msgq_p;
	msg->timeout_ms = PLDM_MSG_TIMEOUT_MS;

	for (uint8_t retry_count = 0; retry_count < PLDM_MSG_MAX_RETRY; retry_count++) {
		uint8_t event = 0;
		if (mctp_pldm_send_msg(mctp_p, msg) == PLDM_ERROR) {
#ifdef PLDM_SEND_FAIL_DELAY_MS
			k_msleep(PLDM_SEND_FAIL_DELAY_MS);
#endif
			LOG_WRN("Send msg failed!");
			continue;
		}
		if (k_msgq_get(event_msgq_p, &event, K_FOREVER)) {
			LOG_WRN("Failed to get status from msgq!");
			continue;
		}
		if (event == PLDM_READ_EVENT_SUCCESS) {
			ret_len = recv_arg_p->return_len;
			SAFE_FREE(recv_arg_p);
			SAFE_FREE(event_msgq_p);
			return ret_len;
		}
	}
	SAFE_FREE(event_msgq_p);
	SAFE_FREE(recv_arg_p);
	LOG_WRN("Retry reach max!");
	return 0;
}

static uint8_t pldm_msg_timeout_check(sys_slist_t *list, struct k_mutex *mutex)
{
	CHECK_NULL_ARG_WITH_RETURN(list, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(mutex, MCTP_ERROR);

	if (k_mutex_lock(mutex, K_MSEC(PLDM_RESP_MSG_PROC_MUTEX_TIMEOUT_MS))) {
		LOG_WRN("pldm mutex is locked over %d ms!!", PLDM_RESP_MSG_PROC_MUTEX_TIMEOUT_MS);
		return MCTP_ERROR;
	}

	sys_snode_t *node;
	sys_snode_t *s_node;
	sys_snode_t *pre_node = NULL;
	int64_t cur_uptime = k_uptime_get();

	SYS_SLIST_FOR_EACH_NODE_SAFE (list, node, s_node) {
		wait_msg *p = (wait_msg *)node;

		if ((p->exp_to_ms <= cur_uptime)) {
			printk("pldm msg timeout!!\n");
			printk("cmd %x, inst_id %x\n", p->msg.hdr.cmd, p->msg.hdr.inst_id);
			sys_slist_remove(list, pre_node, node);

			if (unregister_instid(p->mctp_inst, p->msg.hdr.inst_id) == false) {
				LOG_ERR("Unregister failed!");
				return PLDM_ERROR;
			}

			if (p->msg.timeout_cb_fn)
				p->msg.timeout_cb_fn(p->msg.timeout_cb_fn_args);

			free(p);
		} else {
			pre_node = node;
		}
	}

	k_mutex_unlock(mutex);
	return MCTP_SUCCESS;
}

static void pldm_msg_timeout_monitor(void *dummy0, void *dummy1, void *dummy2)
{
	ARG_UNUSED(dummy0);
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);

	while (1) {
		k_msleep(PLDM_MSG_CHECK_PER_MS);

		pldm_msg_timeout_check(&wait_recv_resp_list, &wait_recv_resp_mutex);
	}
}

static uint8_t pldm_resp_msg_process(mctp *const mctp_inst, uint8_t *buf, uint32_t len,
				     mctp_ext_params ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);

	if (!len)
		return PLDM_ERROR;

	pldm_hdr *hdr = (pldm_hdr *)buf;
	sys_snode_t *node;
	sys_snode_t *s_node;
	sys_snode_t *pre_node = NULL;
	sys_snode_t *found_node = NULL;

	if (k_mutex_lock(&wait_recv_resp_mutex, K_MSEC(PLDM_RESP_MSG_PROC_MUTEX_TIMEOUT_MS))) {
		LOG_WRN("pldm mutex is locked over %d ms!!", PLDM_RESP_MSG_PROC_MUTEX_TIMEOUT_MS);
		return PLDM_ERROR;
	}

	SYS_SLIST_FOR_EACH_NODE_SAFE (&wait_recv_resp_list, node, s_node) {
		wait_msg *p = (wait_msg *)node;

		/* found the proper handler */
		if ((p->msg.hdr.inst_id == hdr->inst_id) &&
		    (p->msg.hdr.pldm_type == hdr->pldm_type) && (p->msg.hdr.cmd == hdr->cmd) &&
		    (p->mctp_inst == mctp_inst)) {
			found_node = node;
			sys_slist_remove(&wait_recv_resp_list, pre_node, node);

			if (unregister_instid(mctp_inst, p->msg.hdr.inst_id) == false) {
				LOG_ERR("Unregister failed!");
				k_mutex_unlock(&wait_recv_resp_mutex);
				return PLDM_ERROR;
			}
			break;
		} else {
			pre_node = node;
		}
	}
	k_mutex_unlock(&wait_recv_resp_mutex);

	if (found_node) {
		/* invoke resp handler */
		wait_msg *p = (wait_msg *)found_node;
		if (p->msg.recv_resp_cb_fn)
			/* remove pldm header for handler */
			p->msg.recv_resp_cb_fn(p->msg.recv_resp_cb_args, buf + sizeof(p->msg.hdr),
					       len - sizeof(p->msg.hdr));
		free(p);
	}

	return PLDM_SUCCESS;
}

uint8_t mctp_pldm_cmd_handler(void *mctp_p, uint8_t *buf, uint32_t len, mctp_ext_params ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);

	if (!len)
		return PLDM_ERROR;

	mctp *mctp_inst = (mctp *)mctp_p;
	pldm_hdr *hdr = (pldm_hdr *)buf;
	LOG_DBG("msg_type = %d", hdr->msg_type);
	LOG_DBG("req_d_id = 0x%x", hdr->req_d_id);
	LOG_DBG("pldm_type = 0x%x", hdr->pldm_type);
	LOG_DBG("cmd = 0x%x", hdr->cmd);

	/*
* The message is a response, check if any callback function should be
* invoked
*/

	if (!hdr->rq) {
		return pldm_resp_msg_process(mctp_inst, buf, len, ext_params);
	}

	/* the message is a request, find the proper handler to handle it */

	/* initial response data */
	uint8_t resp_buf[PLDM_MAX_DATA_SIZE] = { 0 };
	/*
* Default without header length, the header length will be added before
* sending.
*/
	uint16_t resp_len = 1;

	/* make response header */
	hdr->rq = 0;
	memcpy(resp_buf, hdr, sizeof(*hdr));

	/* default one byte response data - completion code */
	uint8_t *comp = resp_buf + sizeof(*hdr);

	void *handler = NULL;
	uint8_t (*handler_query)(uint8_t, void **) = NULL;

	uint8_t i;
	for (i = 0; i < ARRAY_SIZE(query_tbl); i++) {
		if (hdr->pldm_type == query_tbl[i].type) {
			handler_query = query_tbl[i].handler_query;
			break;
		}
	}

	if (!handler_query) {
		*comp = PLDM_ERROR_UNSUPPORTED_PLDM_CMD;
		goto send_msg;
	}

	uint8_t rc = PLDM_ERROR;
	/* found the proper cmd handler in the pldm_type_cmd table */
	rc = handler_query(hdr->cmd, &handler);
	if (rc == PLDM_ERROR || !handler) {
		*comp = PLDM_ERROR_UNSUPPORTED_PLDM_CMD;
		goto send_msg;
	}

	rc = ((pldm_cmd_proc_fn)handler)(mctp_inst, buf + sizeof(*hdr), len - sizeof(*hdr),
					 (hdr->req_d_id) & 0x1F, resp_buf + sizeof(*hdr), &resp_len,
					 &ext_params);
	if (rc == PLDM_LATER_RESP)
		return PLDM_SUCCESS;

send_msg:
	/* send the pldm response data */
	resp_len = sizeof(*hdr) + resp_len;
	return mctp_send_msg(mctp_inst, resp_buf, resp_len, ext_params);
}

uint8_t mctp_pldm_send_msg(void *mctp_p, pldm_msg *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(msg, PLDM_ERROR);

	mctp *mctp_inst = (mctp *)mctp_p;
	uint8_t get_inst_id = 0xff;
	wait_msg *p = NULL;

	/*
	* The request should be set inst_id/msg_type/mctp_tag_owner in the
	* header
	*/

	if (msg->hdr.rq) {
		if (register_instid(mctp_p, &get_inst_id) == false) {
			LOG_ERR("Register failed!");
			return PLDM_ERROR;
		}

		/* set pldm header */
		msg->hdr.inst_id = get_inst_id;
		msg->hdr.msg_type = MCTP_MSG_TYPE_PLDM;

		/* set mctp extra parameters */
		msg->ext_params.tag_owner = 1;
	}

	uint16_t len = sizeof(msg->hdr) + msg->len;
	uint8_t buf[len];

	LOG_HEXDUMP_DBG(buf, len, __func__);

	memcpy(buf, &msg->hdr, sizeof(msg->hdr));
	memcpy(buf + sizeof(msg->hdr), msg->buf, msg->len);

	if (msg->hdr.rq) {
		p = (wait_msg *)malloc(sizeof(*p));
		if (!p) {
			LOG_WRN("wait_msg alloc failed!");
			goto error;
		}

		memset(p, 0, sizeof(*p));
		p->mctp_inst = mctp_inst;
		p->msg = *msg;
		p->exp_to_ms =
			k_uptime_get() + (msg->timeout_ms ? msg->timeout_ms : PLDM_MSG_TIMEOUT_MS);

		k_mutex_lock(&wait_recv_resp_mutex, K_FOREVER);
		sys_slist_append(&wait_recv_resp_list, &p->node);
		k_mutex_unlock(&wait_recv_resp_mutex);
	}

	uint8_t rc = mctp_send_msg(mctp_inst, buf, len, msg->ext_params);
	if (rc == MCTP_ERROR) {
		LOG_ERR("mctp_send_msg error!!");

		if (p != NULL) {
			sys_slist_find_and_remove(&wait_recv_resp_list, &p->node);
			SAFE_FREE(p);
		}

		goto error;
	}

	return PLDM_SUCCESS;

error:
	if (msg->hdr.rq) {
		if (unregister_instid(mctp_p, get_inst_id) == false) {
			LOG_ERR("Unregister failed!");
		}
	}

	return PLDM_ERROR;
}

/**
 * @brief Get the supported PLDM types.
 *
 * @param buf Pointer to the buffer, which should have 8 bytes
 * @param buf_size Number of buf's bytes
 *
 * @retval 0 if successful
 */
uint8_t get_supported_pldm_type(uint8_t *buf, uint8_t buf_size)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);

	if (buf_size != GET_PLDM_TYPE_BUF_SIZE)
		return PLDM_ERROR_INVALID_LENGTH;

	memset(buf, 0, buf_size);

	for (uint8_t i = 0; i < ARRAY_SIZE(query_tbl); i++) {
		const uint8_t buf_index = query_tbl[i].type / 8;
		const uint8_t bit_index = query_tbl[i].type % 8;

		if (buf_index >= buf_size) {
			LOG_WRN("The PLDM type(0x%x) is out of range!", query_tbl[i].type);
			continue;
		}

		buf[buf_index] |= BIT(bit_index);
	}

	return PLDM_SUCCESS;
}

uint8_t get_supported_pldm_commands(PLDM_TYPE type, uint8_t *buf, uint8_t buf_size)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);

	if (buf_size != GET_PLDM_COMMAND_BUF_SIZE)
		return PLDM_ERROR_INVALID_LENGTH;

	memset(buf, 0, buf_size);

	uint8_t (*handler_query)(uint8_t, void **) = NULL;

	for (uint8_t i = 0; i < ARRAY_SIZE(query_tbl); i++) {
		if (type == query_tbl[i].type) {
			handler_query = query_tbl[i].handler_query;
			break;
		}
	}

	if (!handler_query) {
		LOG_WRN("Invalid PLDM type, (0x%x)", type);
		return PLDM_ERROR_INVALID_PLDM_TYPE;
	}

	for (uint16_t cmd = 0; cmd < (GET_PLDM_COMMAND_BUF_SIZE * 8); cmd++) {
		void *handler = NULL;

		uint8_t rc = handler_query(cmd, &handler);
		if ((rc == PLDM_SUCCESS) && handler)
			buf[cmd / 8] |= BIT(cmd % 8);
	}

	return PLDM_SUCCESS;
}

// Send IPMI response to MCTP/PLDM thread
int pldm_send_ipmi_response(uint8_t interface, ipmi_msg *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, -1);

	pldm_msg pmsg = { 0 };
	uint8_t resp_buf[PLDM_MAX_DATA_SIZE] = { 0 };
	memset(&pmsg, 0, sizeof(pmsg));
	memset(&resp_buf, 0, sizeof(resp_buf));

	mctp_port *p = pal_find_mctp_port_by_channel_target(interface);
	CHECK_NULL_ARG_WITH_RETURN(p, -1);

	int medium_type = p->medium_type;
	if (medium_type < 0) {
		return -1;
	}
	int target = pal_find_bus_in_mctp_port(p);
	if (target < 0) {
		return -1;
	}

	// Set PLDM header
	pmsg.ext_params.type = medium_type;
	pmsg.ext_params.i3c_ext_params.addr = target;

	pmsg.hdr.msg_type = MCTP_MSG_TYPE_PLDM;
	pmsg.hdr.pldm_type = PLDM_TYPE_OEM;
	pmsg.hdr.cmd = PLDM_OEM_IPMI_BRIDGE;
	pmsg.hdr.rq = PLDM_RESPONSE;
	pmsg.hdr.inst_id = msg->pldm_inst_id;

	pmsg.buf = resp_buf;

	struct _ipmi_cmd_resp *cmd_resp = (struct _ipmi_cmd_resp *)pmsg.buf;
	set_iana(cmd_resp->iana, sizeof(cmd_resp->iana));
	cmd_resp->completion_code = PLDM_SUCCESS;
	cmd_resp->netfn_lun = (msg->netfn | 0x01) << 2;
	cmd_resp->cmd = msg->cmd;
	cmd_resp->ipmi_comp_code = msg->completion_code;
	memcpy(&cmd_resp->first_data, msg->data, msg->data_len);

	// Total data len = IANA + PLDM CC + ipmi CC + ipmi netfn + ipmi cmd + ipmi response data len
	pmsg.len = sizeof(struct _ipmi_cmd_resp) - 1 + msg->data_len;

	LOG_HEXDUMP_DBG(pmsg.buf, pmsg.len, "pmsg.buf");

	CHECK_NULL_ARG_WITH_RETURN(p->mctp_inst, -1);
	// Send response to PLDM/MCTP thread
	mctp_pldm_send_msg(p->mctp_inst, &pmsg);
	return 0;
}

// Send IPMI request to MCTP/PLDM thread and get response
int pldm_send_ipmi_request(ipmi_msg *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, -1);

	pldm_msg pmsg = { 0 };
	uint8_t req_buf[PLDM_MAX_DATA_SIZE] = { 0 };
	memset(&pmsg, 0, sizeof(pmsg));
	memset(req_buf, 0, sizeof(req_buf));

	mctp_port *p = pal_find_mctp_port_by_channel_target(msg->InF_target);
	CHECK_NULL_ARG_WITH_RETURN(p, -1);

	int target = pal_find_bus_in_mctp_port(p);
	if (target < 0) {
		return -1;
	}

	// Set PLDM header
	pmsg.ext_params.type = p->mctp_inst->medium_type;
	pmsg.ext_params.i3c_ext_params.addr = target;

	pmsg.hdr.msg_type = MCTP_MSG_TYPE_PLDM;
	pmsg.hdr.pldm_type = PLDM_TYPE_OEM;
	pmsg.hdr.cmd = PLDM_OEM_IPMI_BRIDGE;
	pmsg.hdr.rq = PLDM_REQUEST;

	pmsg.buf = req_buf;

	struct _ipmi_cmd_req *cmd_req = (struct _ipmi_cmd_req *)pmsg.buf;
	set_iana(cmd_req->iana, sizeof(cmd_req->iana));
	cmd_req->netfn_lun = msg->netfn << 2;
	cmd_req->cmd = msg->cmd;
	memcpy(&cmd_req->first_data, msg->data, msg->data_len);

	// Total data len = IANA + ipmi netfn + ipmi cmd + ipmi request data len
	pmsg.len = sizeof(struct _ipmi_cmd_req) - 1 + msg->data_len;

	uint8_t rbuf[PLDM_MAX_DATA_SIZE];

	LOG_HEXDUMP_DBG(pmsg.buf, pmsg.len, "pmsg.buf");
	// Send request to PLDM/MCTP thread and get response
	uint8_t res_len = mctp_pldm_read(p->mctp_inst, &pmsg, rbuf, sizeof(rbuf));

	if (!res_len) {
		LOG_ERR("mctp_pldm_read fail");
		return false;
	}

	struct _pldm_ipmi_cmd_resp *resp = (struct _pldm_ipmi_cmd_resp *)rbuf;

	if ((resp->completion_code != MCTP_SUCCESS)) {
		resp->ipmi_comp_code = CC_UNSPECIFIED_ERROR;
	}

	msg->completion_code = resp->ipmi_comp_code;
	msg->netfn = resp->netfn_lun >> 2;
	msg->cmd = resp->cmd;
	// MCTP CC, Netfn, cmd, ipmi CC
	if (res_len > 4) {
		msg->data_len = res_len - 4;
		memcpy(msg->data, &rbuf[4], msg->data_len);
	} else {
		msg->data_len = 0;
	}

	return 0;
}

K_THREAD_DEFINE(pldm_wait_resp_to, 1024, pldm_msg_timeout_monitor, NULL, NULL, NULL, 7, 0, 0);
