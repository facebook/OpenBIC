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

#include "mctp.h"
#include "mctp_ctrl.h"
#include <logging/log.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/printk.h>
#include <zephyr.h>
#include "libutil.h"

LOG_MODULE_DECLARE(mctp);

#define DEFAULT_WAIT_TO_MS 3000
#define RESP_MSG_PROC_MUTEX_WAIT_TO_MS 1000
#define TO_CHK_INTERVAL_MS 1000

#define MCTP_CTRL_INST_ID_MASK 0x3F

typedef struct _wait_msg {
	sys_snode_t node;
	mctp *mctp_inst;
	int64_t exp_to_ms;
	mctp_ctrl_msg msg;
} wait_msg;

static K_MUTEX_DEFINE(wait_recv_resp_mutex);

static sys_slist_t wait_recv_resp_list = SYS_SLIST_STATIC_INIT(&wait_recv_resp_list);

__weak int load_mctp_support_types(uint8_t *type_len, uint8_t *types)
{
	return -1;
}

__weak void plat_update_mctp_routing_table(uint8_t eid)
{
	return;
}

uint8_t mctp_ctrl_cmd_set_endpoint_id(void *mctp_inst, uint8_t *buf, uint16_t len, uint8_t *resp,
				      uint16_t *resp_len, void *ext_params)
{
	ARG_UNUSED(ext_params);
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, MCTP_ERROR);

	struct _set_eid_req *req = (struct _set_eid_req *)buf;
	struct _set_eid_resp *p = (struct _set_eid_resp *)resp;

	uint8_t plat_mctp_port_count = plat_get_mctp_port_count();
	if (plat_mctp_port_count != 0) {
		for (uint8_t i = 0; i < plat_mctp_port_count; i++) {
			mctp_port *port = plat_get_mctp_port(i);
			if (port != NULL) {
				port->mctp_inst->endpoint = req->eid;
			} else {
				LOG_ERR("plat_get_mctp_port not implemented");
				p->completion_code = MCTP_CTRL_CC_ERROR;
				*resp_len = 1;
				return MCTP_SUCCESS;
			}
		}
		p->completion_code = MCTP_CTRL_CC_SUCCESS;
	} else {
		LOG_ERR("plat_get_mctp_port not implemented");
		p->completion_code = MCTP_CTRL_CC_ERROR;
	}

	plat_update_mctp_routing_table(req->eid);

	p->status = 0; // Assignment accepted. Device does not use an EID pool.
	p->eid = req->eid;
	p->eid_pool_size = 0;

	*resp_len = (p->completion_code == MCTP_CTRL_CC_SUCCESS) ? sizeof(*p) : 1;

	return MCTP_SUCCESS;
}

uint8_t mctp_ctrl_cmd_get_endpoint_id(void *mctp_inst, uint8_t *buf, uint16_t len, uint8_t *resp,
				      uint16_t *resp_len, void *ext_params)
{
	ARG_UNUSED(ext_params);
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, MCTP_ERROR);

	struct _get_eid_resp *p = (struct _get_eid_resp *)resp;

	p->eid = plat_get_eid();
	p->eid_type = STATIC_EID;
	p->endpoint_type = BRIDGE;
	/* Not support fairness arbitration */
	p->medium_specific_info = 0x00;

	p->completion_code = (len != 0) ? MCTP_CTRL_CC_ERROR_INVALID_LENGTH : MCTP_CTRL_CC_SUCCESS;

	*resp_len = (p->completion_code == MCTP_CTRL_CC_SUCCESS) ? sizeof(*p) : 1;

	return MCTP_SUCCESS;
}

uint8_t mctp_ctrl_cmd_get_message_type_support(void *mctp_inst, uint8_t *buf, uint16_t len,
					       uint8_t *resp, uint16_t *resp_len, void *ext_params)
{
	ARG_UNUSED(ext_params);
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, MCTP_ERROR);

	struct _get_message_type_resp *p = (struct _get_message_type_resp *)resp;

	uint8_t type_len = 0;
	uint8_t *types = malloc(sizeof(TYPE_MAX_SIZE));
	int ret = 0;

	ret = load_mctp_support_types(&type_len, types);

	if (ret < 0) {
		LOG_ERR("Command not supported, %d", ret);
		p->completion_code = MCTP_ERROR;
	} else {
		p->completion_code = MCTP_CTRL_CC_SUCCESS;
		p->type_count = type_len;
		memcpy(p->type_number, types, type_len);
	}

	free(types);
	*resp_len = (p->completion_code == MCTP_CTRL_CC_SUCCESS) ? (sizeof(*p) + type_len) : 1;

	return MCTP_SUCCESS;
}

static uint8_t mctp_ctrl_msg_timeout_check(sys_slist_t *list, struct k_mutex *mutex)
{
	CHECK_NULL_ARG_WITH_RETURN(list, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(mutex, MCTP_ERROR);

	if (k_mutex_lock(mutex, K_MSEC(RESP_MSG_PROC_MUTEX_WAIT_TO_MS))) {
		LOG_WRN("pldm mutex is locked over %d ms!!", RESP_MSG_PROC_MUTEX_WAIT_TO_MS);
		return MCTP_ERROR;
	}

	sys_snode_t *node;
	sys_snode_t *s_node;
	sys_snode_t *pre_node = NULL;
	int64_t cur_uptime = k_uptime_get();

	SYS_SLIST_FOR_EACH_NODE_SAFE (list, node, s_node) {
		wait_msg *p = (wait_msg *)node;

		if ((p->exp_to_ms <= cur_uptime)) {
			printk("mctp ctrl msg timeout!!\n");
			printk("cmd %x, inst_id %x\n", p->msg.hdr.cmd, p->msg.hdr.inst_id);
			sys_slist_remove(list, pre_node, node);

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

static void mctp_ctrl_msg_timeout_monitor(void *dummy0, void *dummy1, void *dummy2)
{
	ARG_UNUSED(dummy0);
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);

	while (1) {
		k_msleep(TO_CHK_INTERVAL_MS);

		mctp_ctrl_msg_timeout_check(&wait_recv_resp_list, &wait_recv_resp_mutex);
	}
}

static uint8_t mctp_ctrl_cmd_resp_process(mctp *mctp_inst, uint8_t *buf, uint32_t len,
					  mctp_ext_params ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, MCTP_ERROR);

	if (!len)
		return MCTP_ERROR;

	if (k_mutex_lock(&wait_recv_resp_mutex, K_MSEC(RESP_MSG_PROC_MUTEX_WAIT_TO_MS))) {
		LOG_WRN("mutex is locked over %d ms!", RESP_MSG_PROC_MUTEX_WAIT_TO_MS);
		return MCTP_ERROR;
	}

	mctp_ctrl_hdr *hdr = (mctp_ctrl_hdr *)buf;
	sys_snode_t *node;
	sys_snode_t *s_node;
	sys_snode_t *pre_node = NULL;
	sys_snode_t *found_node = NULL;

	SYS_SLIST_FOR_EACH_NODE_SAFE (&wait_recv_resp_list, node, s_node) {
		wait_msg *p = (wait_msg *)node;
		/* found the proper handler */
		if ((p->msg.hdr.inst_id == hdr->inst_id) && (p->mctp_inst == mctp_inst) &&
		    (p->msg.hdr.cmd == hdr->cmd)) {
			found_node = node;
			sys_slist_remove(&wait_recv_resp_list, pre_node, node);
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
			p->msg.recv_resp_cb_fn(
				p->msg.recv_resp_cb_args, buf + sizeof(p->msg.hdr),
				len - sizeof(p->msg.hdr)); /* remove mctp ctrl header for handler */
		free(p);
	}

	return MCTP_SUCCESS;
}

static mctp_ctrl_cmd_handler_t mctp_ctrl_cmd_tbl[] = {
	{ MCTP_CTRL_CMD_SET_ENDPOINT_ID, mctp_ctrl_cmd_set_endpoint_id },
	{ MCTP_CTRL_CMD_GET_ENDPOINT_ID, mctp_ctrl_cmd_get_endpoint_id },
	{ MCTP_CTRL_CMD_GET_MESSAGE_TYPE_SUPPORT, mctp_ctrl_cmd_get_message_type_support }
};

uint8_t mctp_ctrl_cmd_handler(void *mctp_p, uint8_t *buf, uint32_t len, mctp_ext_params ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, MCTP_ERROR);

	if (!len)
		return MCTP_ERROR;

	mctp *mctp_inst = (mctp *)mctp_p;
	mctp_ctrl_hdr *hdr = (mctp_ctrl_hdr *)buf;

	/*
* The message is a response, check if any callback function should be
* invoked
*/
	if (!hdr->rq)
		return mctp_ctrl_cmd_resp_process(mctp_inst, buf, len, ext_params);

	/* The message is a request, find the proper handler to handle it */

	/* Initial response data */
	uint8_t resp_buf[MCTP_BASE_LINE_UNIT] = { 0 };

	/*
* Default without header length, the header length will be added before
* sending
*/
	uint16_t resp_len = 1;

	/* Make response header */
	hdr->rq = 0;
	memcpy(resp_buf, hdr, sizeof(*hdr));

	/* Default one byte response data - completion code */
	uint8_t *comp_code = resp_buf + sizeof(*hdr);

	mctp_ctrl_cmd_fn cmd_func = NULL;

	for (uint8_t i = 0; i < ARRAY_SIZE(mctp_ctrl_cmd_tbl); i++) {
		if (hdr->cmd == mctp_ctrl_cmd_tbl[i].cmd_code) {
			cmd_func = mctp_ctrl_cmd_tbl[i].fn;
			break;
		}
	}

	if (!cmd_func) {
		*comp_code = MCTP_CTRL_CC_ERROR_UNSUPPORTED_CMD;
		goto send_msg;
	}

	uint8_t rc = MCTP_ERROR;
	rc = cmd_func(mctp_inst, buf + sizeof(*hdr), len - sizeof(*hdr), resp_buf + sizeof(*hdr),
		      &resp_len, &ext_params);

	/*
* If return MCTP_ERROR means it has an error in using the function internally
* by code, like using a NULL pointer or zero response length by argument.
*/
	if (rc == MCTP_ERROR)
		*comp_code = MCTP_CTRL_CC_ERROR;

send_msg:
	/* Send the mctp control response data */
	resp_len = sizeof(*hdr) + resp_len;
	return mctp_send_msg(mctp_inst, resp_buf, resp_len, ext_params);
}

void mctp_ctrl_read_resp_handler(void *args, uint8_t *read_buf, uint16_t read_len)
{
	CHECK_NULL_ARG(args);
	CHECK_NULL_ARG(read_buf);

	uint8_t status = 0;
	mctp_ctrl_resp_arg *resp_arg = (mctp_ctrl_resp_arg *)args;

	if (read_len > resp_arg->read_len) {
		LOG_WRN("Response length(%d) is greater than buffer length(%d)", read_len,
			resp_arg->read_len);
		resp_arg->return_len = resp_arg->read_len;
	} else {
		resp_arg->return_len = read_len;
	}

	/* Return first data is completion code */
	if (read_buf[0] != MCTP_CTRL_CC_SUCCESS) {
		LOG_ERR("Return code status(0x%x)", read_buf[0]);
		status = MCTP_CTRL_READ_STATUS_CC_ERROR;
	} else {
		memcpy(resp_arg->read_buf, read_buf, resp_arg->return_len);
		status = MCTP_CTRL_READ_STATUS_SUCCESS;
	}

	k_msgq_put(resp_arg->msgq, &status, K_NO_WAIT);
}

void mctp_ctrl_read_timeout_handler(void *args)
{
	CHECK_NULL_ARG(args);

	struct k_msgq *msgq = (struct k_msgq *)args;
	uint8_t status = MCTP_CTRL_READ_STATUS_TIMEOUT;

	k_msgq_put(msgq, &status, K_NO_WAIT);
}

uint8_t mctp_ctrl_send_msg(void *mctp_p, mctp_ctrl_msg *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(msg, MCTP_ERROR);

	if (msg->cmd_data_len != 0) {
		CHECK_NULL_ARG_WITH_RETURN(msg->cmd_data, MCTP_ERROR);
	}

	mctp *mctp_inst = (mctp *)mctp_p;
	wait_msg *p = NULL;

	if (msg->hdr.rq) {
		static uint8_t inst_id;

		msg->hdr.inst_id = (inst_id++) & MCTP_CTRL_INST_ID_MASK;
		msg->hdr.msg_type = MCTP_MSG_TYPE_CTRL;

		msg->ext_params.tag_owner = 1;
	}

	uint16_t len = sizeof(msg->hdr) + msg->cmd_data_len;
	uint8_t buf[len];

	memcpy(buf, &msg->hdr, sizeof(msg->hdr));
	memcpy(buf + sizeof(msg->hdr), msg->cmd_data, msg->cmd_data_len);

	LOG_HEXDUMP_DBG(buf, len, __func__);

	if (msg->hdr.rq) {
		p = (wait_msg *)malloc(sizeof(*p));
		if (!p) {
			LOG_WRN("wait_msg alloc failed!");
			return MCTP_ERROR;
		}

		memset(p, 0, sizeof(*p));
		p->mctp_inst = mctp_inst;
		p->msg = *msg;
		p->exp_to_ms =
			k_uptime_get() + (msg->timeout_ms ? msg->timeout_ms : DEFAULT_WAIT_TO_MS);

		k_mutex_lock(&wait_recv_resp_mutex, K_FOREVER);
		sys_slist_append(&wait_recv_resp_list, &p->node);
		k_mutex_unlock(&wait_recv_resp_mutex);
	}

	uint8_t rc = mctp_send_msg(mctp_inst, buf, len, msg->ext_params);
	if (rc == MCTP_ERROR) {
		LOG_WRN("mctp_send_msg error!!");

		if (p != NULL) {
			sys_slist_find_and_remove(&wait_recv_resp_list, &p->node);
			SAFE_FREE(p);
		}
		return MCTP_ERROR;
	}

	return MCTP_SUCCESS;
}

uint8_t mctp_ctrl_read(void *mctp_p, mctp_ctrl_msg *msg, uint8_t *read_buf, uint16_t read_len)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(msg, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(read_buf, MCTP_ERROR);

	if (msg->cmd_data_len != 0) {
		CHECK_NULL_ARG_WITH_RETURN(msg->cmd_data, MCTP_ERROR);
	}

	uint8_t ret = MCTP_ERROR;
	uint8_t status = 0;
	uint8_t status_msgq_buf[1];
	struct k_msgq *status_msgq = (struct k_msgq *)malloc(sizeof(struct k_msgq));
	if (status_msgq == NULL) {
		LOG_ERR("Fail to allocate status_msgq");
		return ret;
	}

	k_msgq_init(status_msgq, status_msgq_buf, sizeof(uint8_t), 1);

	mctp_ctrl_resp_arg *resp_arg = (mctp_ctrl_resp_arg *)malloc(sizeof(mctp_ctrl_resp_arg));
	if (resp_arg == NULL) {
		SAFE_FREE(status_msgq);
		LOG_ERR("Fail to allocate resp_arg");
		return ret;
	}

	resp_arg->msgq = status_msgq;
	resp_arg->read_buf = read_buf;
	resp_arg->read_len = read_len;
	resp_arg->return_len = 0;

	msg->recv_resp_cb_fn = mctp_ctrl_read_resp_handler;
	msg->recv_resp_cb_args = (void *)resp_arg;
	msg->timeout_cb_fn = mctp_ctrl_read_timeout_handler;
	msg->timeout_cb_fn_args = (void *)status_msgq;
	msg->timeout_ms = DEFAULT_WAIT_TO_MS;

	if (mctp_ctrl_send_msg(mctp_p, msg) == MCTP_ERROR) {
		LOG_ERR("Fail to send ctrl msg");
		goto exit;
	}

	if (k_msgq_get(status_msgq, &status, K_FOREVER)) {
		LOG_ERR("Fail to get status from msgq");
		goto exit;
	}

	if (status == MCTP_CTRL_READ_STATUS_SUCCESS) {
		ret = MCTP_SUCCESS;
	} else {
		LOG_ERR("MCTP ctrl status: 0x%x", status);
	}

exit:
	SAFE_FREE(resp_arg);
	SAFE_FREE(status_msgq);
	return ret;
}

K_THREAD_DEFINE(monitor_tid, 1024, mctp_ctrl_msg_timeout_monitor, NULL, NULL, NULL, K_PRIO_PREEMPT(1), 0, 0);
