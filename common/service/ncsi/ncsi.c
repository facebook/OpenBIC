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

#include "ncsi.h"
#include "mctp.h"
#include <logging/log.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/printk.h>
#include <sys/slist.h>
#include <zephyr.h>
#include "libutil.h"
#include "plat_def.h"

#ifdef ENABLE_NCSI

LOG_MODULE_REGISTER(ncsi);

#define NCSI_MAX_INSTID_COUNT 256
#define NCSI_MSG_CHECK_PER_MS 1000
#define NCSI_MSG_TIMEOUT_MS 6000
#define NCSI_RESP_MSG_PROC_MUTEX_TIMEOUT_MS 500
#define NCSI_MSG_MAX_RETRY 3
#define NCSI_MANAGE_CONTROLLER_ID 0x00
#define NCSI_HEADER_REVISION 0x01

#define NCSI_READ_EVENT_SUCCESS BIT(0)
#define NCSI_READ_EVENT_TIMEOUT BIT(1)

typedef struct _wait_msg {
	sys_snode_t node;
	mctp *mctp_inst;
	int64_t exp_to_ms;
	ncsi_msg msg;
} wait_msg;

typedef struct _ncsi_recv_resp_arg {
	struct k_msgq *msgq;
	uint8_t *rbuf;
	uint16_t rbuf_len;
	uint16_t return_len;
} ncsi_recv_resp_arg;

static K_MUTEX_DEFINE(wait_recv_resp_mutex);

static sys_slist_t wait_recv_resp_list = SYS_SLIST_STATIC_INIT(&wait_recv_resp_list);

static bool unregister_instid(void *mctp_p, uint8_t inst_num)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, false);

	if (inst_num >= NCSI_MAX_INSTID_COUNT) {
		LOG_ERR("Invalid instance number %d", inst_num);
		return false;
	}

	mctp *mctp_inst = (mctp *)mctp_p;

	uint8_t cur_inst_table = inst_num / 32; //bit field of ncsi_inst_table is 32 bits
	uint8_t cur_inst_num = inst_num % 32;

	LOG_DBG("cur_inst_table = %d, cur_inst_num = %d", cur_inst_table, cur_inst_num);

	if (!(mctp_inst->ncsi_inst_table[cur_inst_table] & BIT(cur_inst_num))) {
		LOG_ERR("Inatant id %d not register yet!", inst_num);
		return false;
	}
	WRITE_BIT(mctp_inst->ncsi_inst_table[cur_inst_table], cur_inst_num, 0);

	return true;
}

static bool register_instid(void *mctp_p, uint8_t *inst_num)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, false);
	CHECK_NULL_ARG_WITH_RETURN(inst_num, false);

	mctp *mctp_inst = (mctp *)mctp_p;

	static uint8_t cur_inst_num = 0;
	static uint8_t cur_inst_table = 0;

	int retry = 0;
	for (retry = 0; retry < NCSI_MAX_INSTID_COUNT; retry++) {
		cur_inst_table = cur_inst_num / 32; //bit field of ncsi_inst_table is 32 bits
		if (!(mctp_inst->ncsi_inst_table[cur_inst_table] & BIT(cur_inst_num % 32))) {
			break;
		}
		cur_inst_num = (cur_inst_num + 1);
	}

	if (retry == NCSI_MAX_INSTID_COUNT) {
		LOG_DBG("Inatant id %d not available!", cur_inst_num);
		return false;
	}

	WRITE_BIT(mctp_inst->ncsi_inst_table[cur_inst_table], cur_inst_num % 32, 1);
	*inst_num = cur_inst_num;

	cur_inst_num = (cur_inst_num + 1);

	return true;
}

void ncsi_read_resp_handler(void *args, uint8_t *rbuf, uint16_t rlen)
{
	CHECK_NULL_ARG(args);
	CHECK_NULL_ARG(rbuf);

	LOG_DBG("Response length(%d)", rlen);

	if (!rlen)
		return;

	ncsi_recv_resp_arg *recv_arg = (ncsi_recv_resp_arg *)args;

	if (rlen > recv_arg->rbuf_len) {
		LOG_WRN("Response length(%d) is greater than buffer length(%d)!", rlen,
			recv_arg->rbuf_len);
		recv_arg->return_len = recv_arg->rbuf_len;
	} else {
		recv_arg->return_len = rlen;
	}
	memcpy(recv_arg->rbuf, rbuf, recv_arg->return_len);
	uint8_t status = NCSI_READ_EVENT_SUCCESS;
	k_msgq_put(recv_arg->msgq, &status, K_NO_WAIT);
}

static void ncsi_read_timeout_handler(void *args)
{
	CHECK_NULL_ARG(args);

	struct k_msgq *msgq = (struct k_msgq *)args;
	uint8_t status = NCSI_READ_EVENT_TIMEOUT;
	k_msgq_put(msgq, &status, K_NO_WAIT);
}

static uint8_t ncsi_resp_msg_process(mctp *const mctp_inst, uint8_t *buf, uint32_t len,
				     mctp_ext_params ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, NCSI_COMMAND_FAILED);
	CHECK_NULL_ARG_WITH_RETURN(buf, NCSI_COMMAND_FAILED);

	if (!len)
		return NCSI_COMMAND_FAILED;

	ncsi_hdr *hdr = (ncsi_hdr *)buf;
	sys_snode_t *node;
	sys_snode_t *s_node;
	sys_snode_t *pre_node = NULL;
	sys_snode_t *found_node = NULL;

	if (k_mutex_lock(&wait_recv_resp_mutex, K_MSEC(NCSI_RESP_MSG_PROC_MUTEX_TIMEOUT_MS))) {
		LOG_WRN("ncsi mutex is locked over %d ms!!", NCSI_RESP_MSG_PROC_MUTEX_TIMEOUT_MS);
		return NCSI_COMMAND_FAILED;
	}

	SYS_SLIST_FOR_EACH_NODE_SAFE (&wait_recv_resp_list, node, s_node) {
		wait_msg *p = (wait_msg *)node;

		/* found the proper handler */
		if ((p->msg.hdr.inst_id == hdr->inst_id) && (p->msg.hdr.command == hdr->command) &&
		    (p->mctp_inst == mctp_inst)) {
			found_node = node;
			sys_slist_remove(&wait_recv_resp_list, pre_node, node);

			if (unregister_instid(mctp_inst, p->msg.hdr.inst_id) == false) {
				LOG_ERR("Unregister failed!");
				k_mutex_unlock(&wait_recv_resp_mutex);
				return NCSI_COMMAND_FAILED;
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
			/* remove ncsi header for handler */
			p->msg.recv_resp_cb_fn(p->msg.recv_resp_cb_args, buf + sizeof(p->msg.hdr),
					       len - sizeof(p->msg.hdr));
		free(p);
	}

	return NCSI_COMMAND_COMPLETED;
}

/*
 * The return value is the read length from NCSI device
 */
uint16_t mctp_ncsi_read(void *mctp_p, ncsi_msg *msg, uint8_t *rbuf, uint16_t rbuf_len)
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

	ncsi_recv_resp_arg *recv_arg_p = (ncsi_recv_resp_arg *)malloc(sizeof(ncsi_recv_resp_arg));
	if (!recv_arg_p) {
		SAFE_FREE(event_msgq_p);
		LOG_WRN("Failed to allocate recv_arg_p");
		return 0;
	}
	recv_arg_p->msgq = event_msgq_p;
	recv_arg_p->rbuf = rbuf;
	recv_arg_p->rbuf_len = rbuf_len;
	recv_arg_p->return_len = 0;

	msg->recv_resp_cb_fn = ncsi_read_resp_handler;
	msg->recv_resp_cb_args = (void *)recv_arg_p;
	msg->timeout_cb_fn = ncsi_read_timeout_handler;
	msg->timeout_cb_fn_args = (void *)event_msgq_p;
	msg->timeout_ms = NCSI_MSG_TIMEOUT_MS;

	for (uint8_t retry_count = 0; retry_count < NCSI_MSG_MAX_RETRY; retry_count++) {
		uint8_t event = 0;
		if (mctp_ncsi_send_msg(mctp_p, msg) == NCSI_COMMAND_FAILED) {
			LOG_WRN("Send msg failed!");
			continue;
		}
		if (k_msgq_get(event_msgq_p, &event, K_FOREVER)) {
			LOG_WRN("Failed to get status from msgq!");
			continue;
		}
		if (event == NCSI_READ_EVENT_SUCCESS) {
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

static uint8_t ncsi_msg_timeout_check(sys_slist_t *list, struct k_mutex *mutex)
{
	CHECK_NULL_ARG_WITH_RETURN(list, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(mutex, MCTP_ERROR);

	if (k_mutex_lock(mutex, K_MSEC(NCSI_RESP_MSG_PROC_MUTEX_TIMEOUT_MS))) {
		LOG_WRN("ncsi mutex is locked over %d ms!!", NCSI_RESP_MSG_PROC_MUTEX_TIMEOUT_MS);
		return MCTP_ERROR;
	}

	sys_snode_t *node;
	sys_snode_t *s_node;
	sys_snode_t *pre_node = NULL;
	int64_t cur_uptime = k_uptime_get();

	SYS_SLIST_FOR_EACH_NODE_SAFE (list, node, s_node) {
		wait_msg *p = (wait_msg *)node;

		if ((p->exp_to_ms <= cur_uptime)) {
			printk("ncsi msg timeout!!\n");
			printk("cmd %x, inst_id %x\n", p->msg.hdr.packet_type, p->msg.hdr.inst_id);
			sys_slist_remove(list, pre_node, node);

			if (unregister_instid(p->mctp_inst, p->msg.hdr.inst_id) == false) {
				LOG_ERR("Unregister failed!");
				return NCSI_COMMAND_FAILED;
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

static void ncsi_msg_timeout_monitor(void *dummy0, void *dummy1, void *dummy2)
{
	ARG_UNUSED(dummy0);
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);

	while (1) {
		k_msleep(NCSI_MSG_CHECK_PER_MS);

		ncsi_msg_timeout_check(&wait_recv_resp_list, &wait_recv_resp_mutex);
	}
}

uint8_t mctp_ncsi_cmd_handler(void *mctp_p, uint8_t *buf, uint32_t len, mctp_ext_params ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, NCSI_COMMAND_FAILED);
	CHECK_NULL_ARG_WITH_RETURN(buf, NCSI_COMMAND_FAILED);

	if (!len)
		return NCSI_COMMAND_FAILED;

	mctp *mctp_inst = (mctp *)mctp_p;
	ncsi_hdr *hdr = (ncsi_hdr *)buf;

	LOG_DBG("ncsi inst_id %x", hdr->inst_id);
	LOG_DBG("ncsi packet_type %x", hdr->packet_type);

	/*
* The message is a response, check if any callback function should be
* invoked
*/
	if (hdr->rq == NCSI_COMMAND_RESPONSE)
		return ncsi_resp_msg_process(mctp_inst, buf, len, ext_params);

	/* The message is a request, not support to be a responser yet */
	LOG_INF("NC-SI message received, not support to be a responser, packet_type %x",
		hdr->packet_type);
	return NCSI_COMMAND_FAILED;
}

uint8_t mctp_ncsi_send_msg(void *mctp_p, ncsi_msg *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, NCSI_COMMAND_FAILED);
	CHECK_NULL_ARG_WITH_RETURN(msg, NCSI_COMMAND_FAILED);

	mctp *mctp_inst = (mctp *)mctp_p;
	uint8_t get_inst_id = 0xff;
	wait_msg *p = NULL;

	/*
	* The request should be set inst_id/msg_type/mctp_tag_owner in the
	* header
	*/

	if (msg->hdr.rq == NCSI_COMMAND_REQUEST) {
		if (register_instid(mctp_p, &get_inst_id) == false) {
			LOG_ERR("Register failed!");
			return NCSI_COMMAND_FAILED;
		}

		/* set ncsi header */
		msg->hdr.msg_type = MCTP_MSG_TYPE_NCSI;
		msg->hdr.mc_id = NCSI_MANAGE_CONTROLLER_ID;
		msg->hdr.header_revision = NCSI_HEADER_REVISION;
		msg->hdr.inst_id = get_inst_id;

		/* set mctp extra parameters */
		msg->ext_params.tag_owner = 1;
	}

	uint16_t payload_length = (msg->hdr.payload_length_high << 8) | msg->hdr.payload_length_low;

	uint16_t len = sizeof(msg->hdr) + payload_length;
	uint8_t buf[len];

	memcpy(buf, &msg->hdr, sizeof(msg->hdr));
	memcpy(buf + sizeof(msg->hdr), msg->buf, payload_length);

	LOG_HEXDUMP_DBG(buf, len, __func__);

	if (msg->hdr.rq == NCSI_COMMAND_REQUEST) {
		p = (wait_msg *)malloc(sizeof(*p));
		if (!p) {
			LOG_WRN("wait_msg alloc failed!");
			goto error;
		}

		memset(p, 0, sizeof(*p));
		p->mctp_inst = mctp_inst;
		p->msg = *msg;
		p->exp_to_ms =
			k_uptime_get() + (msg->timeout_ms ? msg->timeout_ms : NCSI_MSG_TIMEOUT_MS);

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

	return NCSI_COMMAND_COMPLETED;

error:
	if (msg->hdr.rq == NCSI_COMMAND_REQUEST) {
		if (unregister_instid(mctp_p, get_inst_id) == false) {
			LOG_ERR("Unregister failed!, msg command %x", msg->hdr.command);
		}
	}

	return NCSI_COMMAND_FAILED;
}

K_THREAD_DEFINE(ncsi_wait_resp_to, 1024, ncsi_msg_timeout_monitor, NULL, NULL, NULL, 7, 0, 0);

#endif //ENABLE_NCSI
