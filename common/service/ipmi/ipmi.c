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

#include <zephyr.h>
#include <kernel.h>
#include <stdio.h>
#include <logging/log.h>
#include "cmsis_os2.h"
#include "ipmi.h"

#ifdef CONFIG_IPMI_KCS_ASPEED
#include "kcs.h"
#endif

#include "usb.h"
#include <string.h>
#include <stdlib.h>
#include "libutil.h"
#include "app_handler.h"
#include "chassis_handler.h"
#include "oem_handler.h"
#include "oem_1s_handler.h"
#include "sensor_handler.h"
#include "storage_handler.h"
#include "mctp.h"
#include "pldm.h"
#include "plat_ipmb.h"

LOG_MODULE_REGISTER(ipmi);

#define IPMI_QUEUE_SIZE 5

struct k_thread IPMI_thread;
K_KERNEL_STACK_MEMBER(IPMI_thread_stack, IPMI_THREAD_STACK_SIZE);

char __aligned(4) ipmi_msgq_buffer[IPMI_BUF_LEN * sizeof(struct ipmi_msg_cfg)];
struct k_msgq ipmi_msgq;
char __aligned(4) self_ipmi_msgq_buffer[1 * sizeof(struct ipmi_msg_cfg)];
struct k_msgq self_ipmi_msgq;

__weak uint32_t get_iana(uint8_t *iana_buf)
{
	CHECK_NULL_ARG_WITH_RETURN(iana_buf, 0);

	uint32_t recieved_iana = iana_buf[2];
	recieved_iana = (recieved_iana << 8) | (uint32_t)iana_buf[1];
	recieved_iana = (recieved_iana << 8) | (uint32_t)iana_buf[0];

	if (recieved_iana == IANA_ID) {
		return IANA_ID;
	} else {
		return 0;
	}
}

static uint8_t send_msg_by_pldm(ipmi_msg_cfg *msg_cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg_cfg, 0);

	/* get the mctp/pldm for sending response from buffer */
	uint16_t pldm_hdr_ofs = sizeof(msg_cfg->buffer.data) - sizeof(pldm_hdr);
	uint16_t mctp_ext_params_ofs = pldm_hdr_ofs - sizeof(mctp_ext_params);
	uint16_t mctp_inst_ofs = mctp_ext_params_ofs - 4;

	/* get the mctp_inst */
	mctp *mctp_inst;
	memcpy(&mctp_inst, msg_cfg->buffer.data + mctp_inst_ofs, 4);
	LOG_DBG("mctp_inst = %p", mctp_inst);

	LOG_HEXDUMP_DBG(msg_cfg->buffer.data + mctp_ext_params_ofs, sizeof(mctp_ext_params),
			"mctp ext param");

	/* get the pldm hdr for response */
	pldm_hdr *hdr = (pldm_hdr *)(msg_cfg->buffer.data + pldm_hdr_ofs);
	LOG_HEXDUMP_DBG(msg_cfg->buffer.data + pldm_hdr_ofs, sizeof(pldm_hdr), "pldm header");

	/* make response data */
	uint8_t resp_buf[PLDM_MAX_DATA_SIZE] = { 0 };
	pldm_msg resp;
	memset(&resp, 0, sizeof(resp));

	/* pldm header */
	resp.buf = resp_buf;
	resp.hdr = *hdr;
	resp.hdr.rq = 0;

	LOG_DBG("msg_cfg->buffer.data_len = %d", msg_cfg->buffer.data_len);
	LOG_DBG("msg_cfg->buffer.completion_code = %x", msg_cfg->buffer.completion_code);

	/* setup ipmi response data of pldm */
	struct _ipmi_cmd_resp *cmd_resp = (struct _ipmi_cmd_resp *)resp.buf;
	set_iana(cmd_resp->iana, sizeof(cmd_resp->iana));
	cmd_resp->completion_code = PLDM_SUCCESS;
	cmd_resp->netfn_lun = (msg_cfg->buffer.netfn | 0x01) << 2;
	cmd_resp->cmd = msg_cfg->buffer.cmd;
	cmd_resp->ipmi_comp_code = msg_cfg->buffer.completion_code;
	memcpy(&cmd_resp->first_data, msg_cfg->buffer.data, msg_cfg->buffer.data_len);

	resp.len = sizeof(*cmd_resp) - 1 + msg_cfg->buffer.data_len;
	LOG_HEXDUMP_DBG(&resp, sizeof(resp.hdr) + resp.len, "pldm resp data");

	memcpy(&resp.ext_params, msg_cfg->buffer.data + mctp_ext_params_ofs,
	       sizeof(resp.ext_params));
	mctp_pldm_send_msg(mctp_inst, &resp);

	return 1;
}

__weak bool pal_request_msg_to_BIC_from_KCS(uint8_t netfn, uint8_t cmd)
{
	if (netfn == NETFN_OEM_1S_REQ) {
		if ((cmd == CMD_OEM_1S_FW_UPDATE) || (cmd == CMD_OEM_1S_RESET_BMC) ||
		    (cmd == CMD_OEM_1S_GET_BIC_STATUS) || (cmd == CMD_OEM_1S_RESET_BIC) ||
		    (cmd == CMD_OEM_1S_GET_BIC_FW_INFO))
			return true;
	}

	return false;
}

__weak bool pal_immediate_respond_from_KCS(uint8_t netfn, uint8_t cmd)
{
	if ((netfn == NETFN_STORAGE_REQ && cmd == CMD_STORAGE_ADD_SEL) ||
	    ((netfn == NETFN_SENSOR_REQ) && (cmd == CMD_SENSOR_PLATFORM_EVENT))) {
		return true;
	}

	return false;
}

__weak int pal_record_bios_fw_version(uint8_t *buf, uint8_t size)
{
	return -2;
}

__weak bool pal_request_msg_to_BIC_from_ME(uint8_t netfn, uint8_t cmd)
{
	if ((netfn == NETFN_OEM_REQ) && (cmd == CMD_OEM_NM_SENSOR_READ)) {
		return true;
	}

	return false;
}

__weak bool pal_is_not_return_cmd(uint8_t netfn, uint8_t cmd)
{
	if (netfn == NETFN_OEM_1S_REQ) {
		if ((cmd == CMD_OEM_1S_MSG_OUT) || (cmd == CMD_OEM_1S_MSG_IN)) {
			return true;
		}
	}

	// Reserve for future commands

	return false;
}

bool common_add_sel_evt_record(common_addsel_msg_t *sel_msg)
{
	CHECK_NULL_ARG_WITH_RETURN(sel_msg, false);

	ipmb_error status;
	static uint16_t record_id = 0x1;
	uint8_t system_event_record = 0x02;
	uint8_t evt_msg_version = 0x04;
	ipmi_msg *msg = (ipmi_msg *)malloc(sizeof(ipmi_msg));
	if (msg == NULL) {
		LOG_ERR("Add sel msg malloc fail");
		return false;
	}
	memset(msg, 0, sizeof(ipmi_msg));

	msg->data_len = 16;
	msg->InF_source = SELF;
	msg->InF_target = sel_msg->InF_target;
	msg->netfn = NETFN_STORAGE_REQ;
	msg->cmd = CMD_STORAGE_ADD_SEL;
	msg->data[0] = (record_id & 0xFF); // Record id byte 0, lsb
	msg->data[1] = ((record_id >> 8) & 0xFF); // Record id byte 1
	msg->data[2] = system_event_record; // Record type
	msg->data[3] = 0x00; // Timestamp, bmc would fill up for bic
	msg->data[4] = 0x00; // Timestamp, bmc would fill up for bic
	msg->data[5] = 0x00; // Timestamp, bmc would fill up for bic
	msg->data[6] = 0x00; // Timestamp, bmc would fill up for bic
	msg->data[7] = (SELF_I2C_ADDRESS << 1); // Generator id
	msg->data[8] = 0x00; // Generator id
	msg->data[9] = evt_msg_version; // Event message format version
	memcpy(&msg->data[10], &sel_msg->sensor_type,
	       sizeof(common_addsel_msg_t) - sizeof(uint8_t));
	record_id++;

	LOG_DBG("BIC add sel to target(%xh) with gen_id[%xh %xh] sensor[type %xh #%xh] event[type %xh] data[%xh %xh %xh]",
		msg->InF_target, msg->data[7], msg->data[8], msg->data[10], msg->data[11],
		msg->data[12], msg->data[13], msg->data[14], msg->data[15]);

	bool ipmb_flag = true;
	status = ipmb_read(msg, IPMB_inf_index_map[msg->InF_target]);
	switch (status) {
	case IPMB_ERROR_FAILURE:
		ipmb_flag = false;
		LOG_ERR("Fail to post msg to InF_target 0x%x txqueue for addsel", msg->InF_target);
		break;
	case IPMB_ERROR_GET_MESSAGE_QUEUE:
		ipmb_flag = false;
		LOG_ERR("No response from InF_target 0x%x for addsel", msg->InF_target);
		break;
	default:
		break;
	}

	SAFE_FREE(msg);
	return ipmb_flag;
}

void IPMI_handler(void *arug0, void *arug1, void *arug2)
{
	ipmi_msg_cfg msg_cfg;

	while (1) {
		uint32_t iana = 0;
		memset(&msg_cfg, 0, sizeof(ipmi_msg_cfg));
		k_msgq_get(&ipmi_msgq, &msg_cfg, K_FOREVER);

		LOG_DBG("IPMI_handler[%d]: netfn: %x", msg_cfg.buffer.data_len,
			msg_cfg.buffer.netfn);
		LOG_HEXDUMP_DBG(msg_cfg.buffer.data, msg_cfg.buffer.data_len, "");

		msg_cfg.buffer.completion_code = CC_INVALID_CMD;
		switch (msg_cfg.buffer.netfn) {
		case NETFN_CHASSIS_REQ:
			IPMI_CHASSIS_handler(&msg_cfg.buffer);
			break;
		case NETFN_BRIDGE_REQ:
			// IPMI_BRIDGE_handler();
			break;
		case NETFN_SENSOR_REQ:
			IPMI_SENSOR_handler(&msg_cfg.buffer);
			break;
		case NETFN_APP_REQ:
			IPMI_APP_handler(&msg_cfg.buffer);
			break;
		case NETFN_FIRMWARE_REQ:
			break;
		case NETFN_STORAGE_REQ:
			IPMI_Storage_handler(&msg_cfg.buffer);
			break;
		case NETFN_TRANSPORT_REQ:
			break;
		case NETFN_DCMI_REQ:
			break;
		case NETFN_NM_REQ:
			break;
		case NETFN_OEM_REQ:
			IPMI_OEM_handler(&msg_cfg.buffer);
			break;
		case NETFN_OEM_1S_REQ:
			iana = get_iana(msg_cfg.buffer.data);
			if ((msg_cfg.buffer.data_len >= 3) && (iana != 0)) {
				msg_cfg.buffer.data_len -= 3;
				memcpy(&msg_cfg.buffer.data[0], &msg_cfg.buffer.data[3],
				       msg_cfg.buffer.data_len);
				IPMI_OEM_1S_handler(&msg_cfg.buffer);
				break;
			} else if (pal_is_not_return_cmd(msg_cfg.buffer.netfn,
							 msg_cfg.buffer.cmd)) {
				// Due to command not returning to bridge command source,
				// enter command handler and return with other invalid CC
				msg_cfg.buffer.completion_code = CC_INVALID_IANA;
				IPMI_OEM_1S_handler(&msg_cfg.buffer);
				break;
			} else {
				msg_cfg.buffer.completion_code = CC_INVALID_IANA;
				msg_cfg.buffer.data_len = 0;
				break;
			}
		default: // invalid net function
			LOG_ERR("invalid msg netfn: %x, cmd: %x", msg_cfg.buffer.netfn,
				msg_cfg.buffer.cmd);
			msg_cfg.buffer.data_len = 0;
			break;
		}

		if (pal_is_not_return_cmd(msg_cfg.buffer.netfn, msg_cfg.buffer.cmd)) {
			continue;
		}

		if (msg_cfg.buffer.completion_code != CC_SUCCESS) {
			msg_cfg.buffer.data_len = 0;
		} else if (msg_cfg.buffer.netfn == NETFN_OEM_1S_REQ) {
			uint8_t copy_data[msg_cfg.buffer.data_len];
			memcpy(&copy_data[0], &msg_cfg.buffer.data[0], msg_cfg.buffer.data_len);
			memcpy(&msg_cfg.buffer.data[3], &copy_data[0], msg_cfg.buffer.data_len);
			msg_cfg.buffer.data_len += 3;
			msg_cfg.buffer.data[0] = iana & 0xFF;
			msg_cfg.buffer.data[1] = (iana >> 8) & 0xFF;
			msg_cfg.buffer.data[2] = (iana >> 16) & 0xFF;
		}

		switch (msg_cfg.buffer.InF_source) {
#ifdef CONFIG_USB
		case BMC_USB:
			usb_write_by_ipmi(&msg_cfg.buffer);
			break;
#endif
#ifdef CONFIG_IPMI_KCS_ASPEED
		case HOST_KCS: {
			uint8_t *kcs_buff;
			kcs_buff = malloc(KCS_BUFF_SIZE * sizeof(uint8_t));
			if (kcs_buff == NULL) { // allocate fail, retry allocate
				k_msleep(10);
				kcs_buff = malloc(KCS_BUFF_SIZE * sizeof(uint8_t));
				if (kcs_buff == NULL) {
					LOG_ERR("IPMI_handler: Fail to malloc for kcs_buff");
					continue;
				}
			}
			kcs_buff[0] = (msg_cfg.buffer.netfn + 1)
				      << 2; // ipmi netfn response package
			kcs_buff[1] = msg_cfg.buffer.cmd;
			kcs_buff[2] = msg_cfg.buffer.completion_code;
			if (msg_cfg.buffer.data_len) {
				if (msg_cfg.buffer.data_len <= (KCS_BUFF_SIZE - 3))
					memcpy(&kcs_buff[3], msg_cfg.buffer.data,
					       msg_cfg.buffer.data_len);
				else
					memcpy(&kcs_buff[3], msg_cfg.buffer.data,
					       (KCS_BUFF_SIZE - 3));
			}

			LOG_DBG("kcs from ipmi netfn %x, cmd %x, length %d, cc %x", kcs_buff[0],
				kcs_buff[1], msg_cfg.buffer.data_len, kcs_buff[2]);

			kcs_write(kcs_buff, msg_cfg.buffer.data_len + 3);
			SAFE_FREE(kcs_buff);
			break;
		}
#endif
		case PLDM:
			/* the message should be passed to source by pldm format */
			send_msg_by_pldm(&msg_cfg);
			break;
		case SELF:
			/* for bic self test */
			if (k_msgq_put(&self_ipmi_msgq, &msg_cfg, K_NO_WAIT)) {
				k_msgq_purge(&self_ipmi_msgq);
				LOG_ERR("Failed to put msg into self ipmi msgq");
			}
			break;
		default: {
#if MAX_IPMB_IDX
			ipmb_error status;
			status = ipmb_send_response(&msg_cfg.buffer,
						    IPMB_inf_index_map[msg_cfg.buffer.InF_source]);
			if (status != IPMB_ERROR_SUCCESS) {
				LOG_ERR("IPMI_handler send IPMB resp fail status: %x", status);
			}
#endif
			break;
		}
		}
	}
}

void ipmi_init(void)
{
	LOG_DBG("ipmi_init");
	k_msgq_init(&ipmi_msgq, ipmi_msgq_buffer, sizeof(struct ipmi_msg_cfg), IPMI_BUF_LEN);
	k_msgq_init(&self_ipmi_msgq, self_ipmi_msgq_buffer, sizeof(struct ipmi_msg_cfg), 1);

	k_thread_create(&IPMI_thread, IPMI_thread_stack, K_THREAD_STACK_SIZEOF(IPMI_thread_stack),
			IPMI_handler, NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&IPMI_thread, "IPMI_thread");

#if MAX_IPMB_IDX
	ipmb_init();
#endif
}
