#include "cci.h"
#include "mctp.h"
#include <logging/log.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/printk.h>
#include <sys/slist.h>
#include <zephyr.h>
#include "libutil.h"
#include "sensor.h"
#include "plat_def.h"

#ifdef ENABLE_CCI

LOG_MODULE_REGISTER(cci);

#define DEFAULT_WAIT_TO_MS 3000
#define RESP_MSG_PROC_MUTEX_WAIT_TO_MS 1000
#define TO_CHK_INTERVAL_MS 1000
#define CCI_MSG_MAX_RETRY 3
#define CCI_MSG_TIMEOUT_MS 3000
#define CCI_READ_EVENT_SUCCESS BIT(0)
#define CCI_READ_EVENT_TIMEOUT BIT(1)

static K_MUTEX_DEFINE(wait_recv_resp_mutex);

static sys_slist_t wait_recv_resp_list = SYS_SLIST_STATIC_INIT(&wait_recv_resp_list);

static uint8_t mctp_cci_msg_timeout_check(sys_slist_t *list, struct k_mutex *mutex)
{
	CHECK_NULL_ARG_WITH_RETURN(list, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(mutex, MCTP_ERROR);

	if (k_mutex_lock(mutex, K_MSEC(RESP_MSG_PROC_MUTEX_WAIT_TO_MS))) {
		LOG_WRN("cci mutex is locked over %d ms!!", RESP_MSG_PROC_MUTEX_WAIT_TO_MS);
		return MCTP_ERROR;
	}

	sys_snode_t *node;
	sys_snode_t *s_node;
	sys_snode_t *pre_node = NULL;
	int64_t cur_uptime = k_uptime_get();

	SYS_SLIST_FOR_EACH_NODE_SAFE (list, node, s_node) {
		wait_msg *p = (wait_msg *)node;

		if ((p->exp_to_ms <= cur_uptime)) {
			printk("mctp cci msg timeout!!\n");
			sys_slist_remove(list, pre_node, node);

			if (p->msg.timeout_cb_fn)
				p->msg.timeout_cb_fn(p->msg.timeout_cb_fn_args);

			SAFE_FREE(p);
		} else {
			pre_node = node;
		}
	}

	k_mutex_unlock(mutex);
	return MCTP_SUCCESS;
}

static void mctp_cci_msg_timeout_monitor(void *dummy0, void *dummy1, void *dummy2)
{
	ARG_UNUSED(dummy0);
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);

	while (1) {
		k_msleep(TO_CHK_INTERVAL_MS);

		mctp_cci_msg_timeout_check(&wait_recv_resp_list, &wait_recv_resp_mutex);
	}
}

static void cci_read_timeout_handler(void *args)
{
	CHECK_NULL_ARG(args);
	struct k_msgq *msgq = (struct k_msgq *)args;
	uint8_t status = CCI_READ_EVENT_TIMEOUT;
	k_msgq_put(msgq, &status, K_NO_WAIT);
}

static uint8_t mctp_cci_cmd_resp_process(mctp *mctp_inst, uint8_t *buf, uint32_t len,
					 mctp_ext_params ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, MCTP_ERROR);

	mctp_cci_hdr *cci_hdr = (mctp_cci_hdr *)buf;
	sys_snode_t *node;
	sys_snode_t *s_node;
	sys_snode_t *pre_node = NULL;
	sys_snode_t *found_node = NULL;

	if (k_mutex_lock(&wait_recv_resp_mutex, K_MSEC(RESP_MSG_PROC_MUTEX_WAIT_TO_MS))) {
		LOG_WRN("mutex is locked over %d ms!", RESP_MSG_PROC_MUTEX_WAIT_TO_MS);
		return MCTP_ERROR;
	}
	SYS_SLIST_FOR_EACH_NODE_SAFE (&wait_recv_resp_list, node, s_node) {
		wait_msg *p = (wait_msg *)node;
		/* found the proper handler */
		if ((p->mctp_inst == mctp_inst) && (p->msg.hdr.msg_tag == cci_hdr->msg_tag) &&
		    (p->msg.hdr.op == cci_hdr->op)) {
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
				len - sizeof(p->msg.hdr),
				cci_hdr->ret); /* remove mctp cci header for handler */
		SAFE_FREE(p);
	}

	return MCTP_SUCCESS;
}

uint8_t mctp_cci_send_msg(void *mctp_p, mctp_cci_msg *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, CCI_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(msg, CCI_ERROR);

	mctp *mctp_inst = (mctp *)mctp_p;

	if (!msg->hdr.cci_msg_req_resp) {
		msg->hdr.msg_tag = mctp_inst->cci_msg_tag++;
		msg->hdr.msg_type = MCTP_MSG_TYPE_CCI;
		msg->ext_params.tag_owner = 1;
	}

	uint16_t len = sizeof(msg->hdr) + msg->hdr.pl_len;
	uint8_t buf[len];

	memcpy(buf, &msg->hdr, sizeof(msg->hdr));
	if (msg->hdr.pl_len) {
		memcpy(buf + sizeof(msg->hdr), msg->pl_data, msg->hdr.pl_len);
	}
	LOG_HEXDUMP_DBG(buf, len, __func__);

	uint8_t rc = mctp_send_msg(mctp_inst, buf, len, msg->ext_params);

	if (rc == CCI_ERROR) {
		LOG_WRN("mctp_send_msg error!!");
		return CCI_ERROR;
	}
	if (!msg->hdr.cci_msg_req_resp) {
		wait_msg *p = (wait_msg *)malloc(sizeof(*p));
		if (!p) {
			LOG_WRN("wait_msg alloc failed!");
			return CCI_ERROR;
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

	return CCI_SUCCESS;
}

void cci_read_resp_handler(void *args, uint8_t *rbuf, uint16_t rlen, uint16_t ret_code)
{
	CHECK_NULL_ARG(args);
	CHECK_NULL_ARG(rbuf);
	uint8_t status = 0;
	cci_recv_resp_arg *recv_arg = (cci_recv_resp_arg *)args;

	if (rlen > recv_arg->rbuf_len) {
		LOG_WRN("Response length(%d) is greater than buffer length(%d)!", rlen,
			recv_arg->rbuf_len);
		recv_arg->return_len = recv_arg->rbuf_len;
	} else {
		recv_arg->return_len = rlen;
	}
	memcpy(recv_arg->rbuf, rbuf, recv_arg->return_len);
	if (ret_code == CCI_CC_SUCCESS) {
		status = CCI_READ_EVENT_SUCCESS;
	} else {
		LOG_ERR("Return code status(0x%04x)!", ret_code);
	}
	k_msgq_put(recv_arg->msgq, &status, K_NO_WAIT);
}

uint16_t mctp_cci_read(void *mctp_p, mctp_cci_msg *msg, uint8_t *rbuf, uint16_t rbuf_len)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, 0);
	CHECK_NULL_ARG_WITH_RETURN(msg, 0);
	CHECK_NULL_ARG_WITH_RETURN(rbuf, 0);

	uint8_t event_msgq_buffer[1];
	struct k_msgq *event_msgq_p = (struct k_msgq *)malloc(sizeof(struct k_msgq));
	if (!event_msgq_p) {
		LOG_WRN("event_msgq_p alloc failed!");
		return CCI_ERROR;
	}
	uint16_t ret_len = 0;

	k_msgq_init(event_msgq_p, event_msgq_buffer, sizeof(uint8_t), 1);

	cci_recv_resp_arg *recv_arg_p = (cci_recv_resp_arg *)malloc(sizeof(cci_recv_resp_arg));
	if (!recv_arg_p) {
		SAFE_FREE(event_msgq_p);
		LOG_WRN("recv_arg_p alloc failed!");
		return CCI_ERROR;
	}
	recv_arg_p->msgq = event_msgq_p;
	recv_arg_p->rbuf = rbuf;
	recv_arg_p->rbuf_len = rbuf_len;
	recv_arg_p->return_len = 0;

	msg->recv_resp_cb_fn = cci_read_resp_handler;
	msg->recv_resp_cb_args = (void *)recv_arg_p;
	msg->timeout_cb_fn = cci_read_timeout_handler;
	msg->timeout_cb_fn_args = (void *)event_msgq_p;
	msg->timeout_ms = CCI_MSG_TIMEOUT_MS;

	for (uint8_t retry_count = 0; retry_count < CCI_MSG_MAX_RETRY; retry_count++) {
		uint8_t event = 0;
		if (mctp_cci_send_msg(mctp_p, msg) == CCI_ERROR) {
			LOG_WRN("send msg failed!");
			continue;
		}
		if (k_msgq_get(event_msgq_p, &event, K_MSEC(CCI_MSG_TIMEOUT_MS + 1000))) {
			LOG_WRN("Failed to get status from msgq!");
			continue;
		}
		if (event == CCI_READ_EVENT_SUCCESS) {
			ret_len = recv_arg_p->return_len;
			SAFE_FREE(recv_arg_p);
			SAFE_FREE(event_msgq_p);
			return ret_len;
		}
	}
	SAFE_FREE(recv_arg_p);
	SAFE_FREE(event_msgq_p);
	LOG_WRN("Retry reach max!");
	return 0;
}

uint8_t mctp_cci_cmd_handler(void *mctp_p, uint8_t *buf, uint32_t len, mctp_ext_params ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, MCTP_ERROR);

	mctp *mctp_inst = (mctp *)mctp_p;
	mctp_cci_hdr *cci_hdr = (mctp_cci_hdr *)buf;

	uint8_t mctp_msg_type = cci_hdr->msg_type;
	uint8_t cci_msg_resp = cci_hdr->cci_msg_req_resp;

	if (mctp_msg_type != MCTP_MSG_TYPE_CCI) {
		return CCI_INVALID_TYPE;
	}

	if (cci_msg_resp) {
		mctp_cci_cmd_resp_process(mctp_inst, buf, len, ext_params);
		return CCI_CC_SUCCESS;
	}

	/*TODO : request handler*/

	return CCI_CC_SUCCESS;
}

bool cci_get_chip_temp(void *mctp_p, mctp_ext_params ext_params, int16_t *chip_temp)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, false);
	CHECK_NULL_ARG_WITH_RETURN(chip_temp, false);

	mctp_cci_msg msg = { 0 };
	memcpy(&msg.ext_params, &ext_params, sizeof(msg.ext_params));

	msg.hdr.op = CCI_GET_HEALTH_INFO;
	msg.hdr.pl_len = HEALTH_INFO_REQ_PL_LEN;

	int resp_len = sizeof(cci_health_info_resp);
	uint8_t rbuf[resp_len];

	if (mctp_cci_read(mctp_p, &msg, rbuf, resp_len) != resp_len) {
		LOG_ERR("mctp_cci_read fail");
		return false;
	}

	cci_health_info_resp *resp_p = (cci_health_info_resp *)rbuf;
	*chip_temp = resp_p->dev_temp;

	return true;
}

bool cci_get_chip_fw_version(void *mctp_p, mctp_ext_params ext_params, uint8_t *fw_version,
			     uint8_t *return_len)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, false);
	CHECK_NULL_ARG_WITH_RETURN(fw_version, false);
	CHECK_NULL_ARG_WITH_RETURN(return_len, false);

	mctp_cci_msg msg = { 0 };
	memcpy(&msg.ext_params, &ext_params, sizeof(msg.ext_params));

	msg.hdr.op = CCI_GET_FW_INFO;
	msg.hdr.pl_len = GET_FW_INFO_REQ_PL_LEN;

	int resp_len = sizeof(cci_fw_info_resp);
	uint8_t active_slot = 0;
	uint8_t resp_buf[resp_len];
	uint8_t *fw_version_ptr = NULL;

	memset(resp_buf, 0, sizeof(cci_fw_info_resp));

	if (mctp_cci_read(mctp_p, &msg, resp_buf, resp_len) != resp_len) {
		LOG_ERR("Get chip fw version fail");
		return false;
	}

	cci_fw_info_resp *resp = (cci_fw_info_resp *)resp_buf;
	active_slot = resp->fw_slot_info.fields.ACTIVE_FW_SLOT;

	switch (active_slot) {
	case SLOT1_FW_ACTIVE:
		fw_version_ptr = resp->slot1_fw_revision;
		break;
	case SLOT2_FW_ACTIVE:
		fw_version_ptr = resp->slot2_fw_revision;
		break;
	case SLOT3_FW_ACTIVE:
		fw_version_ptr = resp->slot3_fw_revision;
		break;
	case SLOT4_FW_ACTIVE:
		fw_version_ptr = resp->slot4_fw_revision;
		break;
	default:
		LOG_ERR("Active slot: %d is invalid", active_slot);
		return false;
	};

	CHECK_NULL_ARG_WITH_RETURN(fw_version_ptr, false);
	memcpy(fw_version, fw_version_ptr, sizeof(uint8_t) * GET_FW_INFO_REVISION_LEN);
	*return_len = GET_FW_INFO_REVISION_LEN;

	return true;
}

K_THREAD_DEFINE(monitor_cci_msg, 1024, mctp_cci_msg_timeout_monitor, NULL, NULL, NULL, 7, 0, 0);

#endif
