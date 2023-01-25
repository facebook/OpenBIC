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

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <logging/log.h>
#include "util_spi.h"
#include "util_sys.h"
#include "libutil.h"
#include "pldm_firmware_update.h"
#include "xdpe12284c.h"
#include "isl69259.h"
#include "mp2971.h"
#include "lattice.h"

LOG_MODULE_DECLARE(pldm);

#define PLDM_FW_UPDATE_STACK_SIZE 4096
#define UPDATE_THREAD_DELAY_SECOND 1
#define MIN_FW_UPDATE_BASELINE_TRANS_SIZE 32

pldm_fw_update_info_t *comp_config = NULL;
uint8_t comp_config_count = 0;

k_tid_t fw_update_tid;
struct k_thread pldm_fw_update_thread;
K_KERNEL_STACK_MEMBER(pldm_fw_update_stack, PLDM_FW_UPDATE_STACK_SIZE);

struct pldm_fw_update_cfg fw_update_cfg = { .image_size = 0,
					    .max_buff_size = MIN_FW_UPDATE_BASELINE_TRANS_SIZE };

static enum pldm_firmware_update_aux_state cur_aux_state = STATE_AUX_NOT_IN_UPDATE;
static enum pldm_firmware_update_state current_state = STATE_IDLE;
static enum pldm_firmware_update_state previous_state = STATE_IDLE;
static uint16_t cur_update_comp_cnt = 0;
static uint16_t rcv_comp_cnt = 0;
static uint16_t cur_update_comp_id = -1;
static char cur_update_comp_str[100] = "unknown";

static bool keep_update_flag = false;

__weak void load_pldmupdate_comp_config(void)
{
	LOG_ERR("Failed to load pldm update table config");
}

uint8_t pldm_bic_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, 1);

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;

	CHECK_NULL_ARG_WITH_RETURN(p->data, 1);

	uint8_t update_flag = 0;

	/* prepare next data offset and length */
	p->next_ofs = p->data_ofs + p->data_len;
	p->next_len = fw_update_cfg.max_buff_size;

	if (p->next_ofs < fw_update_cfg.image_size) {
		if (p->next_ofs + p->next_len > fw_update_cfg.image_size)
			p->next_len = fw_update_cfg.image_size - p->next_ofs;

		if (((p->next_ofs % SECTOR_SZ_64K) + p->next_len) > SECTOR_SZ_64K)
			p->next_len = SECTOR_SZ_64K - (p->next_ofs % SECTOR_SZ_64K);
	} else {
		/* current data is the last packet
		 * set the next data length to 0 to inform the update completely
		 */
		p->next_len = 0;
		update_flag = (SECTOR_END_FLAG | NO_RESET_FLAG);
	}

	uint8_t ret = fw_update(p->data_ofs, p->data_len, p->data, update_flag, DEVSPI_FMC_CS0);

	if (ret) {
		LOG_ERR("Firmware update failed, offset(0x%x), length(0x%x), status(%d)",
			p->data_ofs, p->data_len, ret);
		return 1;
	}

	return 0;
}

uint8_t pldm_vr_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, 1);

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;

	CHECK_NULL_ARG_WITH_RETURN(p->data, 1);

	uint8_t ret = 1;

	static uint8_t *hex_buff = NULL;
	if (p->data_ofs == 0) {
		if (hex_buff) {
			LOG_ERR("previous hex_buff doesn't clean up!");
			goto exit;
		}
		hex_buff = malloc(fw_update_cfg.image_size);
		if (!hex_buff) {
			LOG_ERR("Failed to malloc hex_buff");
			return 1;
		}
	}

	if (!hex_buff) {
		LOG_ERR("First package(offset=0) has missed");
		return 1;
	}

	memcpy(hex_buff + p->data_ofs, p->data, p->data_len);

	p->next_ofs = p->data_ofs + p->data_len;
	p->next_len = fw_update_cfg.max_buff_size;

	if (p->next_ofs < fw_update_cfg.image_size) {
		if (p->next_ofs + p->next_len > fw_update_cfg.image_size)
			p->next_len = fw_update_cfg.image_size - p->next_ofs;
		return 0;
	} else {
		p->next_len = 0;
	}

	if (!strncmp(p->comp_version_str, KEYWORD_VR_ISL69259,
		     ARRAY_SIZE(KEYWORD_VR_ISL69259) - 1)) {
		if (isl69259_fwupdate(p->bus, p->addr, hex_buff) == false)
			goto exit;
	} else if (!strncmp(p->comp_version_str, KEYWORD_VR_XDPE12284C,
			    ARRAY_SIZE(KEYWORD_VR_XDPE12284C) - 1)) {
		if (xdpe12284c_fwupdate(p->bus, p->addr, hex_buff) == false)
			goto exit;
	} else if (!strncmp(p->comp_version_str, KEYWORD_VR_MP2971,
			    ARRAY_SIZE(KEYWORD_VR_MP2971) - 1)) {
		if (mp2971_fwupdate(p->bus, p->addr, hex_buff) == false)
			goto exit;
	} else {
		LOG_ERR("Non-support VR detected with component string %s!", p->comp_version_str);
		goto exit;
	}

	ret = 0;
exit:
	SAFE_FREE(hex_buff);
	return ret;
}

uint8_t pldm_cpld_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, 1);

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;

	CHECK_NULL_ARG_WITH_RETURN(p->data, 1);

	if (!strncmp(p->comp_version_str, KEYWORD_CPLD_LATTICE, strlen(KEYWORD_CPLD_LATTICE))) {
		lattice_update_config_t cpld_update_cfg;
		cpld_update_cfg.interface = p->inf;
		cpld_update_cfg.type = find_type_by_str(p->comp_version_str);
		cpld_update_cfg.bus = p->bus;
		cpld_update_cfg.addr = p->addr;
		cpld_update_cfg.data = p->data;
		cpld_update_cfg.data_len = p->data_len;
		cpld_update_cfg.data_ofs = p->data_ofs;

		if (lattice_fwupdate(&cpld_update_cfg) == false) {
			return 1;
		}

		p->next_len = cpld_update_cfg.next_len;
		p->next_ofs = cpld_update_cfg.next_ofs;
	} else {
		LOG_ERR("Component version string %s not contains support device's keyword",
			p->comp_version_str);
		return 1;
	}

	return 0;
}

K_WORK_DELAYABLE_DEFINE(submit_warm_reset_work, submit_bic_warm_reset);
uint8_t pldm_bic_activate(void *arg)
{
	ARG_UNUSED(arg);

	k_work_schedule(&submit_warm_reset_work, K_SECONDS(1));

	return 0;
}

static uint8_t verify_comp(uint16_t comp_class, uint16_t comp_id, uint8_t comp_class_idx)
{
	if (comp_config_count == 0) {
		LOG_ERR("comp_config not loaded yet");
		return COMP_RSP_UNKNOWN_ERR;
	}

	int tab_idx;
	for (tab_idx = 0; tab_idx < comp_config_count; tab_idx++) {
		if (comp_config[tab_idx].enable == DISABLE)
			continue;
		if (comp_class == comp_config[tab_idx].comp_classification &&
		    comp_id == comp_config[tab_idx].comp_identifier &&
		    comp_class_idx == comp_config[tab_idx].comp_classification_index)
			break;
	}

	if (tab_idx == comp_config_count) {
		return COMP_RSP_FD_NOT_SUPPORT;
	}

	return COMP_RSP_CAN_UPDATE;
}

static uint8_t do_self_activate(uint16_t comp_id)
{
	for (int idx = 0; idx < comp_config_count; idx++) {
		if (comp_config[idx].comp_identifier != comp_id)
			continue;
		if (comp_config[idx].activate_method == COMP_ACT_SELF &&
		    comp_config[idx].self_act_func) {
			if (comp_config[idx].self_act_func(NULL)) {
				return 1;
			}
			break;
		}
	}

	return 0;
}

static void state_update(uint8_t state)
{
	if (state == 0xff)
		return;

	previous_state = current_state;
	current_state = state;
}

static void pldm_status_reset()
{
	state_update(STATE_IDLE);
	cur_aux_state = STATE_AUX_NOT_IN_UPDATE;
	cur_update_comp_id = -1;
	cur_update_comp_cnt = 0;
	rcv_comp_cnt = 0;
	memcpy(cur_update_comp_str, "unknown", 8);
	keep_update_flag = false;
}

uint16_t pldm_fw_update_read(void *mctp_p, enum pldm_firmware_update_commands cmd, uint8_t *req,
			     uint16_t req_len, uint8_t *rbuf, uint16_t rbuf_len, void *ext_params)
{
	/* Return read length zero means read fail */
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, 0);
	CHECK_NULL_ARG_WITH_RETURN(req, 0);
	CHECK_NULL_ARG_WITH_RETURN(rbuf, 0);
	CHECK_NULL_ARG_WITH_RETURN(ext_params, 0);

	pldm_msg msg = { 0 };
	mctp_ext_params *extra_data = (mctp_ext_params *)ext_params;

	msg.ext_params = *extra_data;

	msg.hdr.pldm_type = PLDM_TYPE_FW_UPDATE;
	msg.hdr.cmd = cmd;
	msg.hdr.rq = 1;

	msg.buf = req;
	msg.len = req_len;

	return mctp_pldm_read(mctp_p, &msg, rbuf, rbuf_len);
}

static uint8_t report_tranfer(void *mctp_p, void *ext_params, uint8_t result_code)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, 1);
	CHECK_NULL_ARG_WITH_RETURN(ext_params, 1);

	uint16_t read_len = 0;
	uint8_t rbuf[10] = { 0 };

	switch (current_state) {
	case STATE_DOWNLOAD: {
		struct pldm_transfer_complete_req tran_comp_req = { 0 };
		tran_comp_req.transferResult = result_code;

		read_len = pldm_fw_update_read(mctp_p, PLDM_FW_UPDATE_CMD_CODE_TRANSFER_COMPLETE,
					       (uint8_t *)&tran_comp_req,
					       sizeof(struct pldm_transfer_complete_req), rbuf,
					       ARRAY_SIZE(rbuf), ext_params);

		if ((read_len != 1) || (rbuf[0] != PLDM_SUCCESS)) {
			LOG_ERR("Send transfer complete command wrong response length or received bad cc 0x%x",
				rbuf[0]);
			return 1;
		}
		break;
	}
	case STATE_VERIFY: {
		struct pldm_verify_complete_req verify_comp_req = { 0 };
		verify_comp_req.verifyResult = result_code;

		read_len = pldm_fw_update_read(mctp_p, PLDM_FW_UPDATE_CMD_CODE_VERIFY_COMPLETE,
					       (uint8_t *)&verify_comp_req,
					       sizeof(struct pldm_verify_complete_req), rbuf,
					       ARRAY_SIZE(rbuf), ext_params);

		if ((read_len != 1) || (rbuf[0] != PLDM_SUCCESS)) {
			LOG_ERR("Send verify complete command wrong response length or received bad cc 0x%x",
				rbuf[0]);
			return 1;
		}
		break;
	}
	case STATE_APPLY: {
		struct pldm_apply_complete_req apply_comp_req = { 0 };
		apply_comp_req.applyResult = result_code;
		apply_comp_req.compActivationMethodsModification = 0x0000;

		read_len = pldm_fw_update_read(mctp_p, PLDM_FW_UPDATE_CMD_CODE_APPLY_COMPLETE,
					       (uint8_t *)&apply_comp_req,
					       sizeof(struct pldm_apply_complete_req), rbuf,
					       ARRAY_SIZE(rbuf), ext_params);

		if ((read_len != 1) || (rbuf[0] != PLDM_SUCCESS)) {
			LOG_ERR("Send apply complete command wrong response length or received bad cc 0x%x",
				rbuf[0]);
			return 1;
		}
		break;
	}
	default:
		break;
	}

	return 0;
}

static pldm_fw_update_info_t *found_fw_update_func(uint16_t comp_id)
{
	if (!comp_config_count) {
		LOG_ERR("comp_config not loaded yet");
		return NULL;
	}

	for (uint8_t idx = 0; idx < comp_config_count; idx++) {
		if (comp_config[idx].comp_identifier == comp_id) {
			return &comp_config[idx];
		}
	}

	return NULL;
}

void req_fw_update_handler(void *mctp_p, void *ext_params, void *arg)
{
	ARG_UNUSED(arg);
	if (!mctp_p || !ext_params) {
		LOG_ERR("Pass argument is NULL");
		pldm_status_reset();
		SAFE_FREE(ext_params);
		return;
	}

	LOG_INF("Component %d start update process...", cur_update_comp_id);

	pldm_fw_update_info_t *fw_info = found_fw_update_func(cur_update_comp_id);
	if (!fw_info->update_func) {
		LOG_WRN("Cannot find comp %x update function", cur_update_comp_id);
		pldm_status_reset();
		SAFE_FREE(ext_params);
		return;
	}

	pldm_fw_update_param_t update_param = { 0 };
	update_param.comp_id = cur_update_comp_id;
	update_param.comp_version_str = cur_update_comp_str;
	if (cur_update_comp_id < comp_config_count)
		update_param.inf = comp_config[cur_update_comp_id].inf;
	else {
		LOG_ERR("Given component id %d doesn't exist in config table", cur_update_comp_id);
		return;
	}

	/* do pre-update */
	if (fw_info->pre_update_func) {
		if (fw_info->pre_update_func(&update_param)) {
			LOG_ERR("pre-update failed!");
			goto exit;
		}
	}

	/* the request length is max_buf_size at first request */
	struct pldm_request_firmware_data_req req = { .offset = 0,
						      .length = fw_update_cfg.max_buff_size };

	cur_aux_state = STATE_AUX_INPROGRESS;

	do {
		if (keep_update_flag == false) {
			LOG_WRN("Update has been canceled by UA(Update Agent)");
			cur_aux_state = STATE_AUX_FAILED;
			goto exit;
		}

		uint8_t resp_buf[req.length + 1];
		memset(resp_buf, 0, req.length + 1);

		uint16_t read_len =
			pldm_fw_update_read(mctp_p, PLDM_FW_UPDATE_CMD_CODE_REQUEST_FIRMWARE_DATA,
					    (uint8_t *)&req,
					    sizeof(struct pldm_request_firmware_data_req), resp_buf,
					    ARRAY_SIZE(resp_buf), ext_params);

		/* read_len = request length + completion code */
		if (read_len != req.length + 1) {
			LOG_ERR("Request firmware update failed, offset(0x%x), length(0x%x), read length(%d)",
				req.offset, req.length, read_len);
			cur_aux_state = STATE_AUX_FAILED;
			goto exit;
		}

		update_param.data = resp_buf + 1;
		update_param.data_len = read_len - 1;
		update_param.data_ofs = req.offset;

		uint8_t percent = ((update_param.data_ofs + update_param.data_len) * 100) /
				  fw_update_cfg.image_size;

		static uint8_t previous_percent = 0;
		if (previous_percent != percent)
			LOG_INF("package loaded: %d%%", percent);
		previous_percent = percent;

		if (fw_info->update_func(&update_param)) {
			LOG_ERR("Component %d update failed!", cur_update_comp_id);
			report_tranfer(mctp_p, ext_params, PLDM_FW_UPDATE_GENERIC_ERROR);
			cur_aux_state = STATE_AUX_FAILED;
			goto exit;
		}

		if (!update_param.next_len)
			break;

		req.offset = update_param.next_ofs;
		req.length = update_param.next_len;

	} while (1);

	LOG_INF("Component %d update success!", cur_update_comp_id);
	cur_aux_state = STATE_AUX_SUCCESS;

	LOG_INF("Transfer complete");
	if (report_tranfer(mctp_p, ext_params, PLDM_FW_UPDATE_TRANSFER_SUCCESS)) {
		report_tranfer(mctp_p, ext_params, PLDM_FW_UPDATE_GENERIC_ERROR);
		cur_aux_state = STATE_AUX_FAILED;
		goto exit;
	}
	state_update(STATE_VERIFY);

	LOG_INF("Verify complete");
	if (report_tranfer(mctp_p, ext_params, PLDM_FW_UPDATE_VERIFY_SUCCESS)) {
		report_tranfer(mctp_p, ext_params, PLDM_FW_UPDATE_GENERIC_ERROR);
		cur_aux_state = STATE_AUX_FAILED;
		goto exit;
	}
	state_update(STATE_APPLY);

	LOG_INF("Apply complete");
	if (report_tranfer(mctp_p, ext_params, PLDM_FW_UPDATE_APPLY_SUCCESS)) {
		report_tranfer(mctp_p, ext_params, PLDM_FW_UPDATE_GENERIC_ERROR);
		cur_aux_state = STATE_AUX_FAILED;
		goto exit;
	}
	state_update(STATE_RDY_XFER);

	cur_aux_state = STATE_AUX_SUCCESS;

exit:
	/* do post-update */
	if (fw_info->pos_update_func) {
		if (fw_info->pos_update_func(&update_param)) {
			LOG_ERR("post-update failed!");
		}
	}
	fw_update_cfg.image_size = 0;
	if (fw_update_tid) {
		fw_update_tid = NULL;
	}
	SAFE_FREE(ext_params);
	return;
}

static uint8_t request_update(const void *mctp_inst, const uint8_t *buf, uint16_t len,
			      uint8_t *resp, uint16_t *resp_len, const void *ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(ext_params, PLDM_ERROR);

	struct pldm_request_update_req *req_p = (struct pldm_request_update_req *)buf;
	struct pldm_request_update_resp *resp_p = (struct pldm_request_update_resp *)resp;

	*resp_len = 1;

	if (len != (sizeof(struct pldm_request_update_req) + req_p->comp_image_set_ver_str_len)) {
		resp_p->completion_code = PLDM_ERROR_INVALID_LENGTH;
		goto exit;
	}

	if (current_state != STATE_IDLE) {
		LOG_ERR("Firmware update failed because current state %d is not %d", current_state,
			STATE_IDLE);
		resp_p->completion_code = PLDM_FW_UPDATE_CC_ALREADY_IN_UPDATE_MODE;
		goto exit;
	}

	fw_update_cfg.max_buff_size = req_p->max_transfer_size;
	if (fw_update_cfg.max_buff_size > MAX_FWUPDATE_RSP_BUF_SIZE) {
		LOG_WRN("Maximum transfer size 0x%x over mctp response buffer size limit 0x%x, set to default.",
			fw_update_cfg.max_buff_size, MAX_FWUPDATE_RSP_BUF_SIZE);
		fw_update_cfg.max_buff_size = MAX_FWUPDATE_RSP_BUF_SIZE;
	}

	resp_p->fd_meta_data_len = 0x0000;

	if (req_p->pkg_data_len) {
		resp_p->fd_will_send_pkg_data = 0x01;
	} else {
		resp_p->fd_will_send_pkg_data = 0x00;
	}

	cur_update_comp_cnt = req_p->num_of_comp;

	LOG_INF("max transfer size(%u), number of component(%d), max_outstanding_transfer_req(%d)",
		req_p->max_transfer_size, req_p->num_of_comp, req_p->max_outstanding_transfer_req);
	LOG_INF("packet data length(%d), component sting type(%d) length(%d)", req_p->pkg_data_len,
		req_p->comp_image_set_ver_str_type, req_p->comp_image_set_ver_str_len);
	LOG_HEXDUMP_INF(buf + sizeof(struct pldm_request_update_req),
			req_p->comp_image_set_ver_str_len, "Component image version: ");

	*resp_len = sizeof(struct pldm_request_update_resp);
	resp_p->completion_code = PLDM_SUCCESS;

	state_update(STATE_LEARN_COMP);
	keep_update_flag = true;

exit:
	return PLDM_SUCCESS;
}

static uint8_t pass_component_table(const void *mctp_inst, const uint8_t *buf, uint16_t len,
				    uint8_t *resp, uint16_t *resp_len, const void *ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(ext_params, PLDM_ERROR);

	struct pldm_pass_component_table_req *req_p = (struct pldm_pass_component_table_req *)buf;
	struct pldm_pass_component_table_resp *resp_p =
		(struct pldm_pass_component_table_resp *)resp;

	*resp_len = 1;

	if (len != (sizeof(struct pldm_pass_component_table_req) + req_p->comp_ver_str_len)) {
		resp_p->completion_code = PLDM_ERROR_INVALID_LENGTH;
		goto exit;
	}

	LOG_INF("Received component class: %xh id: %d with version:", req_p->comp_classification,
		req_p->comp_identifier);
	LOG_HEXDUMP_INF(buf + sizeof(struct pldm_pass_component_table_req), req_p->comp_ver_str_len,
			"");

	if (current_state != STATE_LEARN_COMP) {
		LOG_ERR("Firmware update failed because current state %d is not %d", current_state,
			STATE_LEARN_COMP);
		resp_p->completion_code = PLDM_FW_UPDATE_CC_INVALID_STATE_FOR_COMMAND;
		goto exit;
	}

	/* only support one currently */
	if (req_p->transfer_flag != PLDM_START_AND_END) {
		resp_p->completion_code = PLDM_ERROR_INVALID_DATA;
		goto exit;
	}

	/* The classification should be downstream device */
	if (req_p->comp_classification == PLDM_COMP_DOWNSTREAM_DEVICE) {
		/**
     * The identifier should be range 0x0000-0x0FFF when classification is the
     * downstream device.
     * TBD: Can use this field to identify update other components
     */
		if (req_p->comp_identifier > 0x0FFF) {
			LOG_ERR("Invalid component identifier id");
			resp_p->completion_code = PLDM_ERROR_INVALID_DATA;
			goto exit;
		}

		if (req_p->comp_classification_index != 0x00) {
			LOG_ERR("Only support update one device currently");
			resp_p->completion_code = PLDM_ERROR_INVALID_DATA;
			goto exit;
		}
	} else {
		LOG_ERR("Not support non-oem component classification currently");
		resp_p->completion_code = PLDM_ERROR_INVALID_DATA;
		goto exit;
	}

	resp_p->comp_resp = 0x00;
	resp_p->comp_resp_code = verify_comp(req_p->comp_classification, req_p->comp_identifier,
					     req_p->comp_classification_index);
	if (resp_p->comp_resp_code)
		resp_p->comp_resp = 0x01;

	/* Currently not support error condition, so only response zero */
	resp_p->completion_code = PLDM_SUCCESS;

	*resp_len = sizeof(struct pldm_pass_component_table_resp);

	rcv_comp_cnt++;
	if (rcv_comp_cnt == cur_update_comp_cnt) {
		state_update(STATE_RDY_XFER);
		rcv_comp_cnt = 0;
	}

exit:
	return PLDM_SUCCESS;
}

static uint8_t update_component(const void *mctp_inst, const uint8_t *buf, uint16_t len,
				uint8_t *resp, uint16_t *resp_len, const void *ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(ext_params, PLDM_ERROR);

	struct pldm_update_component_req *req_p = (struct pldm_update_component_req *)buf;
	struct pldm_update_component_resp *resp_p = (struct pldm_update_component_resp *)resp;

	*resp_len = 1;

	if (len != (sizeof(struct pldm_update_component_req) + req_p->comp_ver_str_len)) {
		resp_p->completion_code = PLDM_ERROR_INVALID_LENGTH;
		goto exit;
	}

	if (current_state != STATE_RDY_XFER) {
		LOG_ERR("Firmware update failed because current state %d is not %d", current_state,
			STATE_RDY_XFER);
		resp_p->completion_code = PLDM_FW_UPDATE_CC_NOT_IN_UPDATE_MODE;
		goto exit;
	}

	/* The classification should be downstream device */
	if (req_p->comp_classification == PLDM_COMP_DOWNSTREAM_DEVICE) {
		/**
     * The identifier should be range 0x0000-0x0FFF when classification is the
     * downstream device.
     * TBD: Can use this field to identify update other components
     */
		if (req_p->comp_identifier > 0x0FFF) {
			LOG_ERR("Invalid component identifier id");
			resp_p->completion_code = PLDM_ERROR_INVALID_DATA;
			goto exit;
		}

		if (req_p->comp_classification_index != 0x00) {
			LOG_ERR("Only support update one device currently");
			resp_p->completion_code = PLDM_ERROR_INVALID_DATA;
			goto exit;
		}
	} else {
		LOG_ERR("Not support non-oem component classification currently");
		resp_p->completion_code = PLDM_ERROR_INVALID_DATA;
		goto exit;
	}

	resp_p->comp_compatability_resp = 0x00;
	resp_p->comp_compatability_resp_code =
		verify_comp(req_p->comp_classification, req_p->comp_identifier,
			    req_p->comp_classification_index);
	if (resp_p->comp_compatability_resp_code)
		resp_p->comp_compatability_resp = 0x01;

	fw_update_cfg.image_size = req_p->comp_image_size;

	LOG_INF("Update component class 0x%x id: %d image_size: 0x%x version: ",
		req_p->comp_classification, req_p->comp_identifier, fw_update_cfg.image_size);
	LOG_HEXDUMP_INF(buf + sizeof(struct pldm_update_component_req), req_p->comp_ver_str_len,
			"");

	resp_p->completion_code = PLDM_SUCCESS;

	/* Return update option flags by request message */
	resp_p->update_option_flags_enabled = req_p->update_option_flags;
	/* Delay 1 second to start update thread */
	resp_p->time_before_req_fw_data = UPDATE_THREAD_DELAY_SECOND;
	*resp_len = sizeof(struct pldm_update_component_resp);

	/* should not do update if component not support */
	if (resp_p->comp_compatability_resp)
		goto exit;

	cur_update_comp_id = req_p->comp_identifier;
	memcpy(cur_update_comp_str, buf + sizeof(struct pldm_update_component_req),
	       req_p->comp_ver_str_len);

	mctp_ext_params *extra_data = (mctp_ext_params *)malloc(sizeof(mctp_ext_params));

	if (!extra_data) {
		LOG_ERR("Allocate memory failed");
		resp_p->completion_code = PLDM_ERROR;
		goto exit;
	}

	memcpy(extra_data, ext_params, sizeof(mctp_ext_params));

	fw_update_tid =
		k_thread_create(&pldm_fw_update_thread, pldm_fw_update_stack,
				K_THREAD_STACK_SIZEOF(pldm_fw_update_stack), req_fw_update_handler,
				(void *)mctp_inst, extra_data, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0,
				K_SECONDS(UPDATE_THREAD_DELAY_SECOND));
	k_thread_name_set(&pldm_fw_update_thread, "pldm_fw_update_thread");

	state_update(STATE_DOWNLOAD);

exit:
	return PLDM_SUCCESS;
}

static uint8_t activate_firmware(const void *mctp_inst, const uint8_t *buf, uint16_t len,
				 uint8_t *resp, uint16_t *resp_len, const void *ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(ext_params, PLDM_ERROR);

	struct pldm_activate_firmware_req *req_p = (struct pldm_activate_firmware_req *)buf;
	struct pldm_activate_firmware_resp *resp_p = (struct pldm_activate_firmware_resp *)resp;

	*resp_len = 1;

	if (len != sizeof(struct pldm_activate_firmware_req)) {
		resp_p->completion_code = PLDM_ERROR_INVALID_LENGTH;
		goto exit;
	}

	if (current_state != STATE_RDY_XFER) {
		LOG_ERR("Firmware update failed because current state %d is not %d", current_state,
			STATE_RDY_XFER);
		resp_p->completion_code = PLDM_FW_UPDATE_CC_INVALID_STATE_FOR_COMMAND;
		goto exit;
	}

	if (req_p->selfContainedActivationRequest != PLDM_NOT_ACTIVATE_SELF_CONTAINED_COMPONENTS) {
		resp_p->completion_code = PLDM_ERROR_INVALID_DATA;
		goto exit;
	}

	state_update(STATE_ACTIVATE);

	if (do_self_activate(cur_update_comp_id)) {
		resp_p->completion_code = PLDM_FW_UPDATE_CC_SELF_CONTAINED_ACTIVATION_NOT_PERMITTED;
		cur_aux_state = STATE_AUX_FAILED;
		goto exit;
	}

	cur_aux_state = STATE_AUX_SUCCESS;

	LOG_INF("Activate firmware");
	resp_p->completion_code = PLDM_SUCCESS;
	resp_p->estimated = 0;
	*resp_len = sizeof(struct pldm_activate_firmware_resp);

	pldm_status_reset();

exit:
	return PLDM_SUCCESS;
}

static uint8_t get_status(const void *mctp_inst, const uint8_t *buf, uint16_t len, uint8_t *resp,
			  uint16_t *resp_len, const void *ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(ext_params, PLDM_ERROR);

	struct pldm_get_status_resp *resp_p = (struct pldm_get_status_resp *)resp;

	*resp_len = 1;

	if (len != 0) {
		resp_p->completion_code = PLDM_ERROR_INVALID_LENGTH;
		goto exit;
	}

	LOG_INF("Get status");
	resp_p->completion_code = PLDM_SUCCESS;
	resp_p->cur_state = current_state;
	resp_p->pre_state = previous_state;
	resp_p->aux_state = cur_aux_state;
	if (cur_aux_state == STATE_AUX_FAILED)
		resp_p->aux_state_status = 0x0A; //generic error
	else
		resp_p->aux_state_status = 0;
	resp_p->reason_code = 0; //not support
	resp_p->prog_percent = 0x65; //not support
	resp_p->update_op_flag_en = 0;
	*resp_len = sizeof(struct pldm_get_status_resp);

exit:
	return PLDM_SUCCESS;
}

static uint8_t cancel_update(const void *mctp_inst, const uint8_t *buf, uint16_t len, uint8_t *resp,
			     uint16_t *resp_len, const void *ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(ext_params, PLDM_ERROR);

	struct pldm_cancel_update_resp *resp_p = (struct pldm_cancel_update_resp *)resp;

	*resp_len = 1;

	if (len != 0) {
		resp_p->completion_code = PLDM_ERROR_INVALID_LENGTH;
		goto exit;
	}

	if (current_state == STATE_IDLE) {
		LOG_WRN("Failed to cancel update cause current state not in update mode");
		resp_p->completion_code = PLDM_FW_UPDATE_CC_NOT_IN_UPDATE_MODE;
		goto exit;
	}

	pldm_status_reset();

	LOG_INF("Update canceled");
	resp_p->completion_code = PLDM_SUCCESS;
	resp_p->non_func_comp_ind = 0;
	resp_p->non_func_comp_bitmap = 0;
	*resp_len = sizeof(struct pldm_cancel_update_resp);

exit:
	return PLDM_SUCCESS;
}

static pldm_cmd_handler pldm_fw_update_cmd_tbl[] = {
	{ PLDM_FW_UPDATE_CMD_CODE_REQUEST_UPDATE, request_update },
	{ PLDM_FW_UPDATE_CMD_CODE_PASS_COMPONENT_TABLE, pass_component_table },
	{ PLDM_FW_UPDATE_CMD_CODE_UPDATE_COMPONENT, update_component },
	{ PLDM_FW_UPDATE_CMD_CODE_ACTIVE_FIRMWARE, activate_firmware },
	{ PLDM_FW_UPDATE_CMD_CODE_GET_STATUS, get_status },
	{ PLDM_FW_UPDATE_CMD_CODE_CANCEL_UPDATE, cancel_update },
};

uint8_t pldm_fw_update_handler_query(uint8_t code, void **ret_fn)
{
	CHECK_NULL_ARG_WITH_RETURN(ret_fn, PLDM_ERROR);

	/* load platform component table */
	static bool is_loaded = false;
	if (is_loaded == false) {
		load_pldmupdate_comp_config();
		is_loaded = true;
	}

	pldm_cmd_proc_fn fn = NULL;

	for (int i = 0; i < ARRAY_SIZE(pldm_fw_update_cmd_tbl); i++) {
		if (pldm_fw_update_cmd_tbl[i].cmd_code == code) {
			fn = pldm_fw_update_cmd_tbl[i].fn;
			break;
		}
	}

	*ret_fn = (void *)fn;
	return fn ? PLDM_SUCCESS : PLDM_ERROR;
}
