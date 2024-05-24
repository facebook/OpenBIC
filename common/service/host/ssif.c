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

#include "plat_def.h"
#ifdef ENABLE_SSIF

#include <zephyr.h>
#include <string.h>
#include <stdio.h>
#include <device.h>
#include <stdlib.h>
#include <logging/log.h>
#include <drivers/i2c.h>
#include <sys/crc.h>
#include "ipmi.h"
#include "ssif.h"
#include "pldm.h"
#include "libutil.h"
#include "plat_i2c.h"
#include "hal_i2c_target.h"
#include "hal_gpio.h"
#include "util_worker.h"

#ifdef ENABLE_SBMR
#include "sbmr.h"
#endif

LOG_MODULE_REGISTER(ssif);

#define SSIF_TARGET_MSGQ_SIZE 0x0A

#define SSIF_STATUS_CHECK_PER_MS 100
#define SSIF_TIMEOUT_MS 5000 // i2c bus drop off maximum time

ssif_dev *ssif;
static uint8_t ssif_channel_cnt = 0;

static bool proc_ssif_ok = false;

bool ssif_set_data(uint8_t channel, ipmi_msg_cfg *msg_cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg_cfg, false);

	if (channel >= ssif_channel_cnt) {
		LOG_WRN("Invalid SSIF channel %d", channel);
		return false;
	}

	CHECK_MUTEX_INIT_WITH_RETURN(&ssif[channel].rsp_buff_mutex, false);

	if (k_mutex_lock(&ssif[channel].rsp_buff_mutex, K_MSEC(1000))) {
		LOG_ERR("SSIF[%d] mutex lock failed", channel);
		ssif_error_record(ssif[channel].index, SSIF_STATUS_MUTEX_ERR);
		return false;
	}

	ssif[channel].rsp_buff[0] = msg_cfg->buffer.netfn; // Should modify outside by user
	ssif[channel].rsp_buff[1] = msg_cfg->buffer.cmd;
	ssif[channel].rsp_buff[2] = msg_cfg->buffer.completion_code;
	if (msg_cfg->buffer.data_len) {
		if (msg_cfg->buffer.data_len <= (IPMI_MSG_MAX_LENGTH - 3))
			memcpy(&ssif[channel].rsp_buff[3], msg_cfg->buffer.data,
			       msg_cfg->buffer.data_len);
		else
			memcpy(&ssif[channel].rsp_buff[3], msg_cfg->buffer.data,
			       (IPMI_MSG_MAX_LENGTH - 3));
	}

	ssif[channel].rsp_buf_len = msg_cfg->buffer.data_len + 3;

	LOG_DBG("ssif from ipmi netfn %x, cmd %x, data length %d, cc %x", ssif[channel].rsp_buff[0],
		ssif[channel].rsp_buff[1], msg_cfg->buffer.data_len, ssif[channel].rsp_buff[2]);

	k_sem_give(&ssif[channel].rsp_buff_sem);

	if (k_mutex_unlock(&ssif[channel].rsp_buff_mutex))
		LOG_ERR("SSIF[%d] mutex unlock failed", channel);

	return true;
}

bool get_ssif_ok()
{
	return proc_ssif_ok;
}

void reset_ssif_ok()
{
	proc_ssif_ok = false;
}

ssif_dev *ssif_inst_get_by_bus(uint8_t bus)
{
	for (int i = 0; i < ssif_channel_cnt; i++) {
		if (ssif[i].i2c_bus == bus) {
			return &ssif[i];
		}
	}

	return NULL;
}

void ssif_error_record(uint8_t channel, ssif_err_status_t errcode)
{
	if (channel >= ssif_channel_cnt) {
		LOG_WRN("Invalid SSIF channel %d", channel);
		return;
	}

	if (ssif[channel].err_idx == SSIF_ERR_RCD_SIZE) {
		LOG_WRN("Error record over wirte!");
		ssif[channel].err_idx = 0;
	}

	ssif[channel].err_status = errcode;
	if (errcode != SSIF_STATUS_NO_ERR) {
		ssif[channel].err_status_lst[ssif[channel].err_idx] = errcode;
		ssif[channel].err_idx++;
	}
}

__weak void pal_ssif_alert_trigger(uint8_t status)
{
	return;
}

__weak void pal_bios_post_complete()
{
	return;
}

static bool ssif_lock_ctl(ssif_dev *ssif_inst, bool lck_flag)
{
	CHECK_NULL_ARG_WITH_RETURN(ssif_inst, false);

	uint8_t addr = ssif_inst->addr << 1;

	if (lck_flag == true) {
		ssif_inst->addr_lock = true;
		ssif_inst->exp_to_ms = k_uptime_get() + SSIF_TIMEOUT_MS;
		addr = 0;
	} else {
		ssif_inst->addr_lock = false;
	}

	if (i2c_addr_set(ssif_inst->i2c_bus, addr))
		return false;

	return true;
}

/**
 * @brief SSIF pec check function
 *
 * Check CRC 8 from received message with pec.
 *
 * @param addr Target address(BIC itself)
 * @param data Data to be calculted, should include pec at last byte
 *             and exclude target address
 * @param len Length of the data in bytes
 * @param is_pec_exist Whether pec exist
 *
 * @return true, if match 
 */
static bool ssif_pec_check(uint8_t addr, uint8_t *data, uint16_t len, uint8_t *is_pec_exist)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);
	CHECK_NULL_ARG_WITH_RETURN(is_pec_exist, false);

	if (len < 2) {
		LOG_ERR("SSIF request data should at last contain smb_cmd and length");
		return false;
	}

	/* skip if pec not exist */
	if ((len - data[1] - 2) == 0) {
		*is_pec_exist = 0;
		return true;
	} else {
		*is_pec_exist = 1;
	}

	uint8_t pec_buf[len];
	pec_buf[0] = (addr << 1); // wr:0
	memcpy(pec_buf + 1, data, len - 1);

	/** pec byte use 7-degree polynomial with 0 init value and false reverse **/
	uint8_t cal_pec = crc8(pec_buf, sizeof(pec_buf), 0x07, 0x00, false);

	if (data[len - 1] != cal_pec)
		return false;

	return true;
}

/**
 * @brief SSIF pec get function
 *
 * Compute CRC 8 of send out message by passing address, smbus command, data.
 *
 * @param addr Target address(BIC itself)
 * @param smb_cmd SMBus command previously received
 * @param data Data to be calculted, should exclude target address
 * @param len Length of the data in bytes
 *
 * @return pec(crc8)
 */
static uint8_t ssif_pec_get(uint8_t addr, uint8_t smb_cmd, uint8_t *data, uint16_t len)
{
	CHECK_NULL_ARG_WITH_RETURN(data, 0);

	uint8_t pec_buf[len + 3]; // addr + smb_cmd + addr + data
	pec_buf[0] = (addr << 1); // wr:0
	pec_buf[1] = smb_cmd;
	pec_buf[2] = (addr << 1) + 1; // rd:1
	memcpy(pec_buf + 3, data, len);

	/** pec byte use 7-degree polynomial with 0 init value and false reverse **/
	return crc8(pec_buf, sizeof(pec_buf), 0x07, 0x00, false);
}

static bool ssif_state_machine(ssif_dev *ssif_inst, ssif_status_t status)
{
	CHECK_NULL_ARG_WITH_RETURN(ssif_inst, false);

	LOG_DBG("* state machine 0x%x --> 0x%x", ssif_inst->cur_status, status);
	ssif_inst->cur_status = status;
	return true;
}

static void ssif_reset(ssif_dev *ssif_inst)
{
	CHECK_NULL_ARG(ssif_inst);

	ssif_state_machine(ssif_inst, SSIF_STATUS_WAIT_FOR_WR_START);
	memset(&ssif_inst->current_ipmi_msg, 0, sizeof(ssif_inst->current_ipmi_msg));
}

static bool ssif_status_check(ssif_dev *ssif_inst, uint8_t smb_cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(ssif_inst, false);

	switch (smb_cmd) {
	case SSIF_WR_SINGLE:
	case SSIF_WR_MULTI_START:
		if (ssif_inst->cur_status != SSIF_STATUS_WAIT_FOR_WR_START &&
		    ssif_inst->cur_status != SSIF_STATUS_WAIT_FOR_RD_START)
			goto error;
		if (smb_cmd == SSIF_WR_SINGLE)
			ssif_state_machine(ssif_inst, SSIF_STATUS_WR_SINGLE_START);
		else if (smb_cmd == SSIF_WR_MULTI_START)
			ssif_state_machine(ssif_inst, SSIF_STATUS_WR_MULTI_START);
		else
			ssif_state_machine(ssif_inst, SSIF_STATUS_RD_START);
		break;

	case SSIF_WR_MULTI_MIDDLE:
	case SSIF_WR_MULTI_END:
		if (ssif_inst->cur_status != SSIF_STATUS_WAIT_FOR_WR_NEXT)
			goto error;
		if (smb_cmd == SSIF_WR_MULTI_MIDDLE)
			ssif_state_machine(ssif_inst, SSIF_STATUS_WR_MIDDLE);
		else
			ssif_state_machine(ssif_inst, SSIF_STATUS_WR_END);
		break;

	case SSIF_RD_START:
		if (ssif_inst->cur_status != SSIF_STATUS_WAIT_FOR_RD_START)
			goto error;
		ssif_state_machine(ssif_inst, SSIF_STATUS_RD_START);
		break;

	case SSIF_RD_NEXT:
		if (ssif_inst->cur_status != SSIF_STATUS_WAIT_FOR_RD_NEXT) {
			goto error;
		}
		ssif_state_machine(ssif_inst, SSIF_STATUS_RD_MIDDLE);
		break;

	case SSIF_RD_RETRY:
		if (ssif_inst->cur_status != SSIF_STATUS_WAIT_FOR_RD_NEXT) {
			goto error;
		}
		ssif_state_machine(ssif_inst, SSIF_STATUS_RD_RETRY);
		break;

	default:
		LOG_ERR("SSIF[%d] received invalid SMB command 0x%x in first package",
			ssif_inst->index, smb_cmd);
		ssif_error_record(ssif_inst->index, SSIF_STATUS_INVALID_CMD);
		return false;
	}

	return true;

error:
	LOG_ERR("Current status 0x%x not expect smb command 0x%x", ssif_inst->cur_status, smb_cmd);
	ssif_error_record(ssif_inst->index, SSIF_STATUS_INVALID_CMD_IN_CUR_STATUS);

	return false;
}

static void send_cmd_work_handler(void *arg1, uint32_t arg2)
{
	CHECK_NULL_ARG(arg1);
	ARG_UNUSED(arg2);

	ssif_dev *ssif_inst = (ssif_dev *)arg1;
	int ret = 0;

	do {
		uint8_t seq_source = 0xFF;
		ipmi_msg msg = { 0 };
		msg = construct_ipmi_message(seq_source, ssif_inst->current_ipmi_msg.buffer.netfn,
					     ssif_inst->current_ipmi_msg.buffer.cmd, SELF, BMC_IPMB,
					     ssif_inst->current_ipmi_msg.buffer.data_len,
					     ssif_inst->current_ipmi_msg.buffer.data);

#if MAX_IPMB_IDX
		// Check BMC communication interface if use IPMB or not
		if (!pal_is_interface_use_ipmb(IPMB_inf_index_map[BMC_IPMB])) {
			msg.InF_target = PLDM;
			// Send request to MCTP/PLDM thread to ask BMC
			ret = pldm_send_ipmi_request(&msg);
			if (ret < 0) {
				LOG_ERR("SSIF[%d] Failed to send SSIF msg to BMC via PLDM with ret: 0x%x",
					ssif_inst->index, ret);
				break;
			}
		} else {
			ipmb_error ipmb_ret = ipmb_read(&msg, IPMB_inf_index_map[msg.InF_target]);
			if (ipmb_ret != IPMB_ERROR_SUCCESS) {
				LOG_ERR("SSIF[%d] Failed to send SSIF msg to BMC via IPMB with ret: 0x%x",
					ssif_inst->index, ipmb_ret);
				break;
			}
		}
#else
		msg.InF_target = PLDM;
		// Send request to MCTP/PLDM thread to ask BMC
		ret = pldm_send_ipmi_request(&msg);
		if (ret < 0) {
			LOG_ERR("SSIF[%d] Failed to send SSIF msg to BMC via PLDM with ret: 0x%x",
				ssif_inst->index, ret);
			break;
		}
#endif
	} while (0);
}

static bool ssif_data_handle(ssif_dev *ssif_inst, ssif_action_t action, uint8_t smb_cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(ssif_inst, false);

	switch (action) {
	case SSIF_SEND_IPMI:
		/* Message to BIC */
		if (pal_request_msg_to_BIC_from_HOST(ssif_inst->current_ipmi_msg.buffer.netfn,
						     ssif_inst->current_ipmi_msg.buffer.cmd)) {
			bool skip_ipmi_handle = false;
#ifdef ENABLE_SBMR
			if (ssif_inst->current_ipmi_msg.buffer.netfn == NETFN_DCMI_REQ) {
				if (smbr_cmd_handler(&ssif_inst->current_ipmi_msg.buffer) == true) {
					if (ssif_set_data(ssif_inst->index,
							  &ssif_inst->current_ipmi_msg) == false) {
						LOG_ERR("Failed to write ssif response data");
					}
				}
				skip_ipmi_handle = true;
			}
#endif
			if (skip_ipmi_handle)
				goto exit;

			while (k_msgq_put(&ipmi_msgq, &ssif_inst->current_ipmi_msg, K_NO_WAIT) !=
			       0) {
				k_msgq_purge(&ipmi_msgq);
				LOG_WRN("SSIF[%d] retrying put ipmi msgq", ssif_inst->index);
			}
			/* Message to BMC */
		} else {
			int ret = 0;
			if (pal_immediate_respond_from_HOST(
				    ssif_inst->current_ipmi_msg.buffer.netfn,
				    ssif_inst->current_ipmi_msg.buffer.cmd)) {
				ipmi_msg_cfg ipmi_req = { 0 };
				ipmi_req = ssif_inst->current_ipmi_msg;

				ipmi_req.buffer.data_len = 0;
				ipmi_req.buffer.completion_code = CC_SUCCESS;
				if (((ssif_inst->current_ipmi_msg.buffer.netfn ==
				      NETFN_STORAGE_REQ) &&
				     (ssif_inst->current_ipmi_msg.buffer.cmd ==
				      CMD_STORAGE_ADD_SEL))) {
					ipmi_req.buffer.data_len = 2;
					ipmi_req.buffer.data[0] = 0x00;
					ipmi_req.buffer.data[1] = 0x00;
				}

				ipmi_req.buffer.netfn =
					(ssif_inst->current_ipmi_msg.buffer.netfn + 1) << 2;

				if (ssif_set_data(ssif_inst->index, &ipmi_req) == false) {
					LOG_ERR("Failed to write ssif response data");
				}

				worker_job job = { 0 };
				job.delay_ms = 0;
				job.fn = send_cmd_work_handler;
				job.ptr_arg = ssif_inst;
				add_work(&job);

				goto exit;
			}

			if ((ssif_inst->current_ipmi_msg.buffer.netfn == NETFN_APP_REQ) &&
			    (ssif_inst->current_ipmi_msg.buffer.cmd ==
			     CMD_APP_SET_SYS_INFO_PARAMS) &&
			    (ssif_inst->current_ipmi_msg.buffer.data[0] ==
			     CMD_SYS_INFO_FW_VERSION)) {
				uint8_t ipmi_buff[IPMI_MSG_MAX_LENGTH] = { 0 };
				ipmi_buff[0] = ssif_inst->current_ipmi_msg.buffer.netfn;
				ipmi_buff[1] = ssif_inst->current_ipmi_msg.buffer.cmd;
				memcpy(ipmi_buff + 2, ssif_inst->current_ipmi_msg.buffer.data,
				       ssif_inst->current_ipmi_msg.buffer.data_len);
				ret = pal_record_bios_fw_version(
					ipmi_buff, ssif_inst->current_ipmi_msg.buffer.data_len + 2);
				if (ret == -1) {
					LOG_ERR("Record bios fw version fail");
				}
			}

			if ((ssif_inst->current_ipmi_msg.buffer.netfn == NETFN_OEM_REQ) &&
			    (ssif_inst->current_ipmi_msg.buffer.cmd == CMD_OEM_POST_END)) {
				pal_bios_post_complete();
			}

			do {
				uint8_t seq_source = 0xFF;
				ipmi_msg msg;
				msg = construct_ipmi_message(
					seq_source, ssif_inst->current_ipmi_msg.buffer.netfn,
					ssif_inst->current_ipmi_msg.buffer.cmd, SELF, BMC_IPMB,
					ssif_inst->current_ipmi_msg.buffer.data_len,
					ssif_inst->current_ipmi_msg.buffer.data);

#if MAX_IPMB_IDX
				// Check BMC communication interface if use IPMB or not
				if (!pal_is_interface_use_ipmb(IPMB_inf_index_map[BMC_IPMB])) {
					msg.InF_target = PLDM;
					// Send request to MCTP/PLDM thread to ask BMC
					ret = pldm_send_ipmi_request(&msg);
					if (ret < 0) {
						LOG_ERR("SSIF[%d] Failed to send SSIF msg to BMC via PLDM with ret: 0x%x",
							ssif_inst->index, ret);
						break;
					}
				} else {
					ipmb_error ipmb_ret =
						ipmb_read(&msg, IPMB_inf_index_map[msg.InF_target]);
					if (ipmb_ret != IPMB_ERROR_SUCCESS) {
						LOG_ERR("SSIF[%d] Failed to send SSIF msg to BMC via IPMB with ret: 0x%x",
							ssif_inst->index, ipmb_ret);
						break;
					}
				}
#else
				msg.InF_target = PLDM;
				// Send request to MCTP/PLDM thread to ask BMC
				ret = pldm_send_ipmi_request(&msg);
				if (ret < 0) {
					LOG_ERR("SSIF[%d] Failed to send SSIF msg to BMC via PLDM with ret: 0x%x",
						ssif_inst->index, ret);
					break;
				}
#endif

				ipmi_msg_cfg ssif_rsp = { 0 };
				ssif_rsp.buffer = msg;
				ssif_rsp.buffer.netfn = ssif_rsp.buffer.netfn << 2;
				if (ssif_set_data(ssif_inst->index, &ssif_rsp) == false) {
					LOG_ERR("SSIF[%d] failed to write ssif response data",
						ssif_inst->index);
				}
			} while (0);
		}

	exit:
		if (k_sem_take(&ssif_inst->rsp_buff_sem, K_MSEC(1500)) != 0) {
			LOG_ERR("SSIF[%d] Get ipmi response message timeout!", ssif_inst->index);
			ssif_error_record(ssif_inst->index, SSIF_STATUS_RSP_MSG_TIMEOUT);
			return false;
		}

		LOG_DBG("SSIF[%d] ipmi rsp netfn 0x%x, cmd 0x%x, cc 0x%x, data length %d:",
			ssif_inst->index, ssif_inst->rsp_buff[0], ssif_inst->rsp_buff[1],
			ssif_inst->rsp_buff[2], ssif_inst->rsp_buf_len - 3);
		LOG_HEXDUMP_DBG(ssif_inst->rsp_buff + 3, ssif_inst->rsp_buf_len - 3, "");

		/* unlock i2c bus address */
		if (ssif_lock_ctl(ssif_inst, false) == false) {
			LOG_ERR("SSIF[%d] can't unlock address after sending message",
				ssif_inst->index);
			ssif_error_record(ssif_inst->index, SSIF_STATUS_ADDR_LOCK_ERR);
			return false;
		}

		/* Let HOST know data ready by i2c alert pin */
		pal_ssif_alert_trigger(GPIO_LOW);
		break;

	case SSIF_COLLECT_DATA: {
		ssif_status_t next_status = SSIF_STATUS_WAIT_FOR_WR_START;
		uint16_t wdata_len = 0;
		uint8_t wdata[SSIF_BUFF_SIZE];
		memset(wdata, 0, sizeof(wdata));

		if (ssif_inst->rsp_buf_len) {
			switch (smb_cmd) {
			case SSIF_RD_START: {
				ssif_inst->remain_data_len = ssif_inst->rsp_buf_len;
				ssif_inst->cur_rd_blck = 0;
				if (ssif_inst->remain_data_len > SSIF_MAX_IPMI_DATA_SIZE) {
					wdata[1] = (SSIF_MULTI_RD_KEY >> 8) & 0xFF;
					wdata[2] = SSIF_MULTI_RD_KEY & 0xFF;
					memcpy(wdata + 3, ssif_inst->rsp_buff,
					       SSIF_MAX_IPMI_DATA_SIZE - 2);
					wdata_len = SSIF_MAX_IPMI_DATA_SIZE;
					next_status = SSIF_STATUS_WAIT_FOR_RD_NEXT;
					ssif_inst->remain_data_len -= (SSIF_MAX_IPMI_DATA_SIZE - 2);
				} else {
					memcpy(wdata + 1, ssif_inst->rsp_buff,
					       ssif_inst->rsp_buf_len);
					wdata_len = ssif_inst->rsp_buf_len;
					next_status = SSIF_STATUS_WAIT_FOR_WR_START;
					ssif_inst->remain_data_len = 0;
				}
				break;
			}

			case SSIF_RD_NEXT: {
				if (ssif_inst->remain_data_len > (SSIF_MAX_IPMI_DATA_SIZE - 1)) {
					wdata[1] = ssif_inst->cur_rd_blck;
					wdata_len = SSIF_MAX_IPMI_DATA_SIZE;
					next_status = SSIF_STATUS_WAIT_FOR_RD_NEXT;
				} else {
					wdata[1] = 0xFF;
					wdata_len = ssif_inst->remain_data_len + 1;
					ssif_state_machine(ssif_inst, SSIF_STATUS_RD_END);
					next_status = SSIF_STATUS_WAIT_FOR_WR_START;
				}
				memcpy(wdata + 2,
				       ssif_inst->rsp_buff + (ssif_inst->rsp_buf_len -
							      ssif_inst->remain_data_len),
				       wdata_len - 1);
				ssif_inst->remain_data_len -= (wdata_len - 1);
				ssif_inst->cur_rd_blck++;
				break;
			}

			default:
				LOG_WRN("SSIF[%d] get invalid cmd %d", ssif_inst->index, smb_cmd);
				ssif_error_record(ssif_inst->index, SSIF_STATUS_INVALID_CMD);
				return false;
			}

			wdata[0] = wdata_len;
			wdata_len++;

#ifdef ENABLE_SSIF_RSP_PEC
			wdata[wdata_len] = ssif_pec_get(ssif_inst->addr, smb_cmd, wdata, wdata_len);
			wdata_len++;
#endif

			LOG_DBG("SSIF[%d] write RSP data:", ssif_inst->index);
			LOG_HEXDUMP_DBG(wdata, wdata_len, "");

			uint8_t rc = i2c_target_write(ssif_inst->i2c_bus, wdata, wdata_len);
			if (rc) {
				LOG_ERR("SSIF[%d] i2c_target_write fail, ret %d\n",
					ssif_inst->index, rc);
				ssif_error_record(ssif_inst->index, SSIF_STATUS_TARGET_WR_RD_ERROR);
				return false;
			}

			ssif_state_machine(ssif_inst, next_status);
		} else {
			LOG_WRN("SSIF[%d] data not ready", ssif_inst->index);
			ssif_error_record(ssif_inst->index, SSIF_STATUS_RSP_NOT_READY);
			return false;
		}
		break;
	}

	default:
		LOG_WRN("SSIF[%d] action %d invalid", ssif_inst->index, action);
		ssif_error_record(ssif_inst->index, SSIF_STATUS_UNKNOWN_ERR);
		return false;
	}

	return true;
}

/* Return false if need to skip data collect while target stop */
static bool ssif_collect_data(uint8_t smb_cmd, uint8_t bus)
{
	if ((smb_cmd != SSIF_RD_START) && (smb_cmd != SSIF_RD_NEXT) && (smb_cmd != SSIF_RD_RETRY)) {
		return true;
	}

	/* Deassert after master read back */
	pal_ssif_alert_trigger(GPIO_HIGH);

	ssif_dev *ssif_inst = ssif_inst_get_by_bus(bus);
	if (!ssif_inst) {
		LOG_ERR("Could not find ssif inst by i2c bus %d", bus);
		goto skip_target_read;
	}

	switch (smb_cmd) {
	case SSIF_RD_START:
	case SSIF_RD_NEXT: {
		if (ssif_status_check(ssif_inst, smb_cmd) == false)
			goto skip_target_read;

		if (ssif_data_handle(ssif_inst, SSIF_COLLECT_DATA, smb_cmd) == false)
			goto skip_target_read;

		if (ssif_inst->cur_status == SSIF_STATUS_WAIT_FOR_RD_NEXT)
			return false;

		if (ssif_inst->cur_status == SSIF_STATUS_WAIT_FOR_WR_START) {
			ssif_error_record(ssif_inst->index, SSIF_STATUS_NO_ERR);
			goto skip_target_read;
		}

		break;
	}
	case SSIF_RD_RETRY:
		LOG_WRN("RETRY cmd not ready yet!");
		goto skip_target_read;

	default:
		break;
	}

	return true;

skip_target_read:
	ssif_reset(ssif_inst);
	return false;
}

static bool ssif_write_data(void *arg)
{
	CHECK_NULL_ARG_WITH_RETURN(arg, false);

	struct i2c_target_data *data = (struct i2c_target_data *)arg;

	if (data->wr_buffer_idx == 1) {
		if (ssif_collect_data(data->target_wr_msg.msg[0], data->i2c_bus) == false)
			return false;
	}

	return true;
}

static void ssif_bus_drop(void *arg)
{
	CHECK_NULL_ARG(arg);

	struct i2c_target_data *data = (struct i2c_target_data *)arg;

	/* Only check fisrt byte from received data */
	if (data->wr_buffer_idx == 1) {
		uint8_t smb_cmd = data->target_wr_msg.msg[0];
		/* Drop bus if write last message and wait for next read */
		if ((smb_cmd == SSIF_WR_SINGLE) || (smb_cmd == SSIF_WR_MULTI_END)) {
			ssif_dev *ssif_inst = ssif_inst_get_by_bus(data->i2c_bus);
			if (!ssif_inst) {
				LOG_ERR("Could not find ssif inst by i2c bus %d", data->i2c_bus);
				return;
			}

			//pal_ssif_alert_trigger(GPIO_LOW);
			if (ssif_lock_ctl(ssif_inst, true) == false) {
				LOG_ERR("Could not lock address");
				ssif_error_record(ssif_inst->index, SSIF_STATUS_ADDR_LOCK_ERR);
				return;
			}
			//pal_ssif_alert_trigger(GPIO_HIGH);
		}
	}
}

static void ssif_timeout_monitor(void *dummy0, void *dummy1, void *dummy2)
{
	ARG_UNUSED(dummy0);
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);

	while (1) {
		k_msleep(SSIF_STATUS_CHECK_PER_MS);

		for (int idx = 0; idx < ssif_channel_cnt; idx++) {
			if (ssif[idx].addr_lock == false)
				continue;

			int64_t cur_uptime = k_uptime_get();
			if ((ssif[idx].exp_to_ms <= cur_uptime)) {
				LOG_WRN("SSIF[%d] msg timeout, ssif unlock!!", idx);
				ssif_error_record(ssif[idx].index, SSIF_STATUS_ADDR_LCK_TIMEOUT);
				if (ssif_lock_ctl(&ssif[idx], false) == false) {
					LOG_ERR("SSIF[%d] unlock failed", idx);
					ssif_error_record(ssif[idx].index,
							  SSIF_STATUS_ADDR_LOCK_ERR);
				}
				ssif_reset(&ssif[idx]);
			}
		}
	}
}

static bool ssif_data_pre_handle(ssif_dev *ssif_inst, uint8_t smb_cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(ssif_inst, false);

	/* Should not received READ command, cause already been handle in lower level */
	if ((smb_cmd == SSIF_RD_START) || (smb_cmd == SSIF_RD_NEXT) || (smb_cmd == SSIF_RD_RETRY)) {
		LOG_ERR("SSIF[%d] not expect READ commands in this thread", ssif_inst->index);
		ssif_error_record(ssif_inst->index, SSIF_STATUS_UNKNOWN_ERR);
		return false;
	}

	if (ssif_status_check(ssif_inst, smb_cmd) == false) {
		LOG_ERR("SSIF[%d] status check failed", ssif_inst->index);
		return false;
	}

	return true;
}

static void ssif_read_task(void *arvg0, void *arvg1, void *arvg2)
{
	ARG_UNUSED(arvg1);
	ARG_UNUSED(arvg2);

	ssif_dev *ssif_inst = (ssif_dev *)arvg0;
	CHECK_NULL_ARG(ssif_inst);

	int rc = 0;
	uint8_t cur_smb_cmd = 0;

	memset(&ssif_inst->current_ipmi_msg, 0, sizeof(ssif_inst->current_ipmi_msg));

	while (1) {
		uint8_t rdata[SSIF_BUFF_SIZE] = { 0 };
		uint16_t rlen = 0;
		rc = i2c_target_read(ssif_inst->i2c_bus, rdata, sizeof(rdata), &rlen);
		if (rc) {
			LOG_ERR("SSIF[%d] i2c_target_read failed, ret %d\n", ssif_inst->index, rc);
			ssif_error_record(ssif_inst->index, SSIF_STATUS_TARGET_WR_RD_ERROR);
			goto cold_reset;
		}

		proc_ssif_ok = true;

		if (rlen == 0) {
			LOG_ERR("SSIF[%d] received invalid length of message", ssif_inst->index);
			ssif_error_record(ssif_inst->index, SSIF_STATUS_INVALID_LEN);
			goto cold_reset;
		}

		LOG_DBG("SSIF[%d] read REQ data:", ssif_inst->index);
		LOG_HEXDUMP_DBG(rdata, rlen, "");

		cur_smb_cmd = rdata[0];

		if (ssif_data_pre_handle(ssif_inst, cur_smb_cmd) == false) {
			goto cold_reset;
		}

		uint8_t is_pec_exist = 0;
		if (ssif_pec_check(ssif_inst->addr, rdata, rlen, &is_pec_exist) == false) {
			LOG_ERR("SSIF[%d] pec check failed", ssif_inst->index);
			ssif_error_record(ssif_inst->index, SSIF_STATUS_INVALID_PEC);
			goto cold_reset;
		}

		/* This part should only handle WRITE command */
		switch (cur_smb_cmd) {
		case SSIF_WR_SINGLE:
		case SSIF_WR_MULTI_START: {
			if (rlen - 1 - is_pec_exist >
			    sizeof(struct ssif_wr_start)) { // exclude smb_cmd, pec
				LOG_WRN("SSIF[%d] received invalid message length for smb command %d",
					ssif_inst->index, cur_smb_cmd);
				ssif_error_record(ssif_inst->index, SSIF_STATUS_INVALID_LEN);
				goto cold_reset;
			}

			struct ssif_wr_start *wr_start_msg = (struct ssif_wr_start *)(rdata + 1);
			if (wr_start_msg->len !=
			    (rlen - 2 - is_pec_exist)) { // exclude smb_cmd, len, pec
				LOG_WRN("SSIF[%d] received invalid length byte for smb command %d",
					ssif_inst->index, cur_smb_cmd);
				ssif_error_record(ssif_inst->index, SSIF_STATUS_INVALID_LEN);
				goto cold_reset;
			}

			ssif_inst->current_ipmi_msg.buffer.InF_source =
				HOST_SSIF_1 + ssif_inst->index;
			ssif_inst->current_ipmi_msg.buffer.netfn = wr_start_msg->netfn >> 2;
			ssif_inst->current_ipmi_msg.buffer.cmd = wr_start_msg->cmd;
			ssif_inst->current_ipmi_msg.buffer.data_len =
				wr_start_msg->len - 2; // exclude netfn, cmd
			if (ssif_inst->current_ipmi_msg.buffer.data_len != 0) {
				memcpy(ssif_inst->current_ipmi_msg.buffer.data, wr_start_msg->data,
				       ssif_inst->current_ipmi_msg.buffer.data_len);
			}

			if (cur_smb_cmd == SSIF_WR_MULTI_START) {
				ssif_error_record(ssif_inst->index, SSIF_STATUS_NO_ERR);
				ssif_state_machine(ssif_inst, SSIF_STATUS_WAIT_FOR_WR_NEXT);
				continue;
			}

			break;
		}

		case SSIF_WR_MULTI_MIDDLE:
		case SSIF_WR_MULTI_END: {
			if (rlen - 1 - is_pec_exist >
			    sizeof(struct ssif_wr_middle)) { // exclude smb_cmd, pec
				LOG_WRN("SSIF[%d] received invalid message length for smb command %d",
					ssif_inst->index, cur_smb_cmd);
				ssif_error_record(ssif_inst->index, SSIF_STATUS_INVALID_LEN);
				goto cold_reset;
			}

			struct ssif_wr_middle *wr_middle_msg = (struct ssif_wr_middle *)(rdata + 1);
			if (wr_middle_msg->len !=
			    (rlen - 2 - is_pec_exist)) { // exclude smb_cmd, len, pec
				LOG_WRN("SSIF[%d] received invalid length byte for smb command %d",
					ssif_inst->index, cur_smb_cmd);
				ssif_error_record(ssif_inst->index, SSIF_STATUS_INVALID_LEN);
				goto cold_reset;
			}

			if ((cur_smb_cmd == SSIF_WR_MULTI_MIDDLE) &&
			    (wr_middle_msg->len != SSIF_MAX_IPMI_DATA_SIZE)) {
				LOG_WRN("SSIF[%d] received invalid length for multi middle read",
					ssif_inst->index);
				ssif_error_record(ssif_inst->index, SSIF_STATUS_INVALID_LEN);
				goto cold_reset;
			}

			if (ssif_inst->current_ipmi_msg.buffer.data_len == 0) {
				LOG_WRN("SSIF[%d] lost first multi read message", ssif_inst->index);
				ssif_error_record(ssif_inst->index,
						  SSIF_STATUS_INVALID_CMD_IN_CUR_STATUS);
				goto cold_reset;
			}

			memcpy(ssif_inst->current_ipmi_msg.buffer.data +
				       ssif_inst->current_ipmi_msg.buffer.data_len,
			       wr_middle_msg->data, wr_middle_msg->len);
			ssif_inst->current_ipmi_msg.buffer.data_len += wr_middle_msg->len;

			if (cur_smb_cmd == SSIF_WR_MULTI_MIDDLE) {
				ssif_error_record(ssif_inst->index, SSIF_STATUS_NO_ERR);
				ssif_state_machine(ssif_inst, SSIF_STATUS_WAIT_FOR_WR_NEXT);
				continue;
			}

			break;
		}

		default:
			LOG_ERR("SSIF[%d] get invalid smb cmd %d", ssif_inst->index, cur_smb_cmd);
			goto cold_reset;
		}

		LOG_DBG("SSIF[%d] ipmi req netfn 0x%x, cmd 0x%x, data length %d:", ssif_inst->index,
			ssif_inst->current_ipmi_msg.buffer.netfn,
			ssif_inst->current_ipmi_msg.buffer.cmd,
			ssif_inst->current_ipmi_msg.buffer.data_len);
		LOG_HEXDUMP_DBG(ssif_inst->current_ipmi_msg.buffer.data,
				ssif_inst->current_ipmi_msg.buffer.data_len, "");

		if (ssif_data_handle(ssif_inst, SSIF_SEND_IPMI, cur_smb_cmd) == false)
			goto warm_reset;

		ssif_error_record(ssif_inst->index, SSIF_STATUS_NO_ERR);
		ssif_state_machine(ssif_inst, SSIF_STATUS_WAIT_FOR_RD_START);
		continue;

	cold_reset:
		/* Bus resume */
		if (ssif_inst->addr_lock == true) {
			if (ssif_lock_ctl(ssif_inst, false) == false) {
				LOG_ERR("Could not unlock address");
				ssif_error_record(ssif_inst->index, SSIF_STATUS_ADDR_LOCK_ERR);
			}
		}

	warm_reset:
		ssif_reset(ssif_inst);
	}
}

void ssif_device_init(struct ssif_init_cfg *config, uint8_t size)
{
	CHECK_NULL_ARG(config);

	SAFE_FREE(ssif);
	ssif = (ssif_dev *)malloc(size * sizeof(*ssif));
	if (!ssif)
		return;
	memset(ssif, 0, size * sizeof(*ssif));

	ssif_channel_cnt = size;

	for (int i = 0; i < size; i++) {
		if (config[i].i2c_bus >= I2C_BUS_MAX_NUM) {
			LOG_ERR("Given i2c bus index %d over limit", config[i].i2c_bus);
			continue;
		}

		struct _i2c_target_config cfg;
		memset(&cfg, 0, sizeof(cfg));
		cfg.address = config[i].addr;
		cfg.i2c_msg_count = config[i].target_msgq_cnt;
		cfg.rd_data_collect_func = ssif_write_data;
		cfg.post_wr_rcv_func = ssif_bus_drop;

		if (i2c_target_control(config[i].i2c_bus, &cfg, I2C_CONTROL_REGISTER) !=
		    I2C_TARGET_API_NO_ERR) {
			LOG_ERR("SSIF[%d] register target failed", i);
			continue;
		}

		if (k_mutex_init(&ssif->rsp_buff_mutex)) {
			LOG_ERR("SSIF[%d] rd mutex initial failed", i);
			continue;
		}

		if (k_sem_init(&ssif->rsp_buff_sem, 0, 1)) {
			LOG_ERR("SSIF[%d] rd semaphore initial failed", i);
			continue;
		}

		ssif[i].i2c_bus = config[i].i2c_bus;
		ssif[i].addr = config[i].addr >> 1;
		ssif[i].addr_lock = false;
		ssif[i].cur_status = SSIF_STATUS_WAIT_FOR_WR_START;
		ssif[i].index = i;
		ssif_error_record(ssif[i].index, SSIF_STATUS_NO_ERR);

		snprintf(ssif[i].task_name, sizeof(ssif[i].task_name), "ssif%d_polling",
			 config[i].i2c_bus);

		ssif[i].ssif_task_tid =
			k_thread_create(&ssif[i].task_thread, ssif[i].ssif_task_stack,
					K_THREAD_STACK_SIZEOF(ssif[i].ssif_task_stack),
					ssif_read_task, (void *)&ssif[i], NULL, NULL,
					CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
		k_thread_name_set(ssif[i].ssif_task_tid, ssif[i].task_name);

		LOG_INF("SSIF[%d] created", i);
	}

	return;
}

K_THREAD_DEFINE(ssif_addr_lck_check, 1024, ssif_timeout_monitor, NULL, NULL, NULL, 7, 0, 0);

#endif /* ENABLE_SSIF */
