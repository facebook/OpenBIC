#include "pldm.h"
#include "mctp.h"
#include <logging/log.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/printk.h>
#include <sys/slist.h>
#include <zephyr.h>

LOG_MODULE_REGISTER(pldm);

#define PLDM_HDR_INST_ID_MASK 0x1F
#define PLDM_MSG_CHECK_PER_MS 1000
#define PLDM_MSG_TIMEOUT_MS 5000
#define PLDM_RESP_MSG_PROC_MUTEX_TIMEOUT_MS 500
#define PLDM_TASK_NAME_MAX_SIZE 32

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

static struct _pldm_handler_query_entry query_tbl[] = { { PLDM_TYPE_BASE, pldm_base_handler_query },
							{ PLDM_TYPE_OEM, pldm_oem_handler_query } };

static K_MUTEX_DEFINE(wait_recv_resp_mutex);

static sys_slist_t wait_recv_resp_list = SYS_SLIST_STATIC_INIT(&wait_recv_resp_list);

static uint8_t pldm_msg_timeout_check(sys_slist_t *list, struct k_mutex *mutex)
{
	if (!list || !mutex)
		return MCTP_ERROR;

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

static uint8_t pldm_resp_msg_process(mctp *mctp_inst, uint8_t *buf, uint32_t len,
				     mctp_ext_params ext_params)
{
	if (!mctp_inst || !buf || !len)
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
	if (!mctp_p || !buf || !len)
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
	if (!hdr->rq)
		return pldm_resp_msg_process(mctp_inst, buf, len, ext_params);

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

	pldm_cmd_proc_fn handler = NULL;
	uint8_t (*handler_query)(uint8_t, void **) = NULL;

	uint8_t i;
	for (i = 0; i < ARRAY_SIZE(query_tbl); i++) {
		if (hdr->pldm_type == query_tbl[i].type) {
			handler_query = query_tbl[i].handler_query;
			break;
		}
	}

	if (!handler_query) {
		*comp = PLDM_BASE_CODES_ERROR_UNSUPPORT_PLDM_TYPE;
		goto send_msg;
	}

	uint8_t rc = PLDM_ERROR;
	/* found the proper cmd handler in the pldm_type_cmd table */
	rc = handler_query(hdr->cmd, (void **)&handler);
	if (rc == PLDM_ERROR || !handler) {
		*comp = PLDM_BASE_CODES_ERROR_UNSUPPORT_PLDM_CMD;
		goto send_msg;
	}

	/* invoke the cmd handler to process */
	rc = handler(mctp_inst, buf + sizeof(*hdr), len - sizeof(*hdr), resp_buf + sizeof(*hdr),
		     &resp_len, &ext_params);
	if (rc == PLDM_LATER_RESP)
		return PLDM_SUCCESS;

send_msg:
	/* send the pldm response data */
	resp_len = sizeof(*hdr) + resp_len;
	return mctp_send_msg(mctp_inst, resp_buf, resp_len, ext_params);
}

uint8_t mctp_pldm_send_msg(void *mctp_p, pldm_msg *msg)
{
	if (!mctp_p || !msg)
		return PLDM_ERROR;

	mctp *mctp_inst = (mctp *)mctp_p;

	/*
* The request should be set inst_id/msg_type/mctp_tag_owner in the
* header
*/
	if (msg->hdr.rq) {
		/* set pldm header */
		msg->hdr.inst_id = (mctp_inst->pldm_inst_id++) & PLDM_HDR_INST_ID_MASK;
		msg->hdr.msg_type = MCTP_MSG_TYPE_PLDM;

		/* set mctp extra parameters */
		msg->ext_params.tag_owner = 1;
	}

	uint16_t len = sizeof(msg->hdr) + msg->len;
	uint8_t buf[len];

	memcpy(buf, &msg->hdr, sizeof(msg->hdr));
	memcpy(buf + sizeof(msg->hdr), msg->buf, msg->len);

	LOG_HEXDUMP_DBG(buf, len, __func__);

	uint8_t rc = mctp_send_msg(mctp_inst, buf, len, msg->ext_params);

	if (rc == MCTP_ERROR) {
		LOG_WRN("mctp_send_msg error!!");
		return PLDM_ERROR;
	}

	if (msg->hdr.rq) {
		wait_msg *p = (wait_msg *)malloc(sizeof(*p));
		if (!p) {
			LOG_WRN("wait_msg alloc failed!");
			return MCTP_ERROR;
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

	return PLDM_SUCCESS;
}

K_THREAD_DEFINE(pldm_wait_resp_to, 1024, pldm_msg_timeout_monitor, NULL, NULL, NULL, 7, 0, 0);