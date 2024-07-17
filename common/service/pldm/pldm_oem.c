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
#include "ipmi.h"
#include "ipmb.h"
#include "libutil.h"
#include "plat_mctp.h"
#include "util_sys.h"
#include <logging/log.h>
#include <stdlib.h>
#include <string.h>
#include <sys/printk.h>
#include <sys/slist.h>
#include <sys/util.h>
#include <zephyr.h>

LOG_MODULE_DECLARE(pldm, LOG_LEVEL_DBG);

static uint8_t bmc_interface = 0;

uint8_t check_iana(const uint8_t *iana)
{
	CHECK_NULL_ARG_WITH_RETURN(iana, PLDM_ERROR);

	for (uint8_t i = 0; i < IANA_LEN; i++) {
		if (iana[i] != ((IANA_ID >> (i * 8)) & 0xFF))
			return PLDM_ERROR;
	}

	return PLDM_SUCCESS;
}

uint8_t set_iana(uint8_t *buf, uint8_t buf_len)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);
	CHECK_ARG_WITH_RETURN(buf_len < IANA_LEN, PLDM_ERROR);

	for (uint8_t i = 0; i < IANA_LEN; i++)
		buf[i] = (IANA_ID >> (i * 8)) & 0xFF;

	return PLDM_SUCCESS;
}

static uint8_t cmd_echo(void *mctp_inst, uint8_t *buf, uint16_t len, uint8_t instance_id,
			uint8_t *resp, uint16_t *resp_len, void *ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(ext_params, PLDM_ERROR);

	struct _cmd_echo_req *req_p = (struct _cmd_echo_req *)buf;
	struct _cmd_echo_resp *resp_p = (struct _cmd_echo_resp *)resp;

	if (check_iana(req_p->iana) == PLDM_ERROR) {
		resp_p->completion_code = PLDM_ERROR_INVALID_DATA;
		return PLDM_SUCCESS;
	}

	set_iana(resp_p->iana, sizeof(resp_p->iana));
	resp_p->completion_code = PLDM_SUCCESS;
	memcpy(&resp_p->first_data, &req_p->first_data, len);
	*resp_len = len + 1;
	return PLDM_SUCCESS;
}

static uint8_t ipmi_cmd(void *mctp_inst, uint8_t *buf, uint16_t len, uint8_t instance_id,
			uint8_t *resp, uint16_t *resp_len, void *ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(ext_params, PLDM_ERROR);

	struct _ipmi_cmd_req *req_p = (struct _ipmi_cmd_req *)buf;

	if (len < (sizeof(*req_p) - 1)) {
		LOG_WRN("request len %d is invalid", len);
		struct _ipmi_cmd_resp *resp_p = (struct _ipmi_cmd_resp *)resp;
		resp_p->completion_code = PLDM_ERROR_INVALID_LENGTH;
		set_iana(resp_p->iana, sizeof(resp_p->iana));

		*resp_len += sizeof(resp_p->iana);
		LOG_WRN("*resp_len = %d...", *resp_len);
		return PLDM_ERROR;
	}

	if (check_iana(req_p->iana) == PLDM_ERROR) {
		LOG_WRN("IANA %08x incorrect", (uint32_t)req_p->iana);
		struct _ipmi_cmd_resp *resp_p = (struct _ipmi_cmd_resp *)resp;
		resp_p->completion_code = PLDM_ERROR_INVALID_DATA;
		set_iana(resp_p->iana, sizeof(resp_p->iana));
		return PLDM_SUCCESS;
	}

	LOG_DBG("ipmi over pldm, len = %d", len);
	LOG_DBG("netfn %x, cmd %x", req_p->netfn_lun, req_p->cmd);
	LOG_HEXDUMP_DBG(buf, len, "ipmi cmd data");

	ipmi_msg_cfg msg = { 0 };

	/* Setup ipmi data */
	msg.buffer.netfn = req_p->netfn_lun >> 2;
	msg.buffer.cmd = req_p->cmd;
	msg.buffer.data_len = len - sizeof(*req_p) + 1;
	msg.buffer.pldm_inst_id = instance_id;
	memcpy(msg.buffer.data, &req_p->first_data, msg.buffer.data_len);

	/* For ipmi/ipmb service to know the source is pldm */
	msg.buffer.InF_source = PLDM;

	/*
* Store the mctp_inst/pldm header/mctp_ext_params in the last of buffer data
* those will use for the ipmi/ipmb service that is done the request.
*/
	uint16_t pldm_hdr_ofs = sizeof(msg.buffer.data) - sizeof(pldm_hdr);
	uint16_t mctp_ext_params_ofs = pldm_hdr_ofs - sizeof(mctp_ext_params);
	uint16_t mctp_inst_ofs = mctp_ext_params_ofs - sizeof(mctp_inst);

	/* store the address of mctp_inst in the buffer */
	memcpy(msg.buffer.data + mctp_inst_ofs, &mctp_inst, 4);

	/* store the ext_params in the buffer */
	memcpy(msg.buffer.data + mctp_ext_params_ofs, ext_params, sizeof(mctp_ext_params));

	/* store the pldm header in the buffer */
	memcpy(msg.buffer.data + pldm_hdr_ofs, buf - sizeof(pldm_hdr), sizeof(pldm_hdr));

	notify_ipmi_client(&msg);

	return PLDM_LATER_RESP;
}

uint8_t send_event_log_to_bmc(uint8_t event_type, uint8_t assertion)
{
	pldm_msg msg = { 0 };
	uint8_t bmc_bus = I2C_BUS_BMC;

	if (bmc_interface == BMC_INTERFACE_I3C) {
		bmc_bus = I3C_BUS_BMC;
		msg.ext_params.type = MCTP_MEDIUM_TYPE_TARGET_I3C;
		msg.ext_params.i3c_ext_params.addr = I3C_STATIC_ADDR_BMC;
		msg.ext_params.ep = MCTP_EID_BMC;
	} else {
		bmc_bus = I2C_BUS_BMC;
		msg.ext_params.type = MCTP_MEDIUM_TYPE_SMBUS;
		msg.ext_params.smbus_ext_params.addr = I2C_ADDR_BMC;
		msg.ext_params.ep = MCTP_EID_BMC;
	}

	msg.hdr.pldm_type = PLDM_TYPE_OEM;
	msg.hdr.cmd = PLDM_OEM_WRITE_FILE_IO;
	msg.hdr.rq = 1;

	struct pldm_oem_write_file_io_req *ptr = (struct pldm_oem_write_file_io_req *)malloc(
		sizeof(struct pldm_oem_write_file_io_req) + (sizeof(uint8_t) * OEM_EVENT_LEN));

	if (!ptr) {
		LOG_ERR("Failed to allocate memory.");
		return PLDM_ERROR;
	}

	ptr->cmd_code = EVENT_LOG;
	ptr->data_length = OEM_EVENT_LEN;
	uint8_t msg_data[2] = {event_type, assertion};
	memcpy(ptr->messages, msg_data, OEM_EVENT_LEN);

	msg.buf = (uint8_t *)ptr;
	msg.len = sizeof(struct pldm_oem_write_file_io_req) + OEM_EVENT_LEN;

	uint8_t resp_len = sizeof(struct pldm_oem_write_file_io_resp);
	uint8_t rbuf[resp_len];

	if (!mctp_pldm_read(find_mctp_by_bus(bmc_bus), &msg, rbuf, resp_len)) {
		SAFE_FREE(ptr);
		LOG_ERR("mctp_pldm_read fail");
		return PLDM_ERROR;
	}

	struct pldm_oem_write_file_io_resp *resp = (struct pldm_oem_write_file_io_resp *)rbuf;
	if (resp->completion_code != PLDM_SUCCESS) {
		SAFE_FREE(ptr);
		LOG_ERR("Check reponse completion code fail %x", resp->completion_code);
		return resp->completion_code;
	}

	SAFE_FREE(ptr);
	return PLDM_SUCCESS;
}

static pldm_cmd_handler pldm_oem_cmd_tbl[] = { { PLDM_OEM_CMD_ECHO, cmd_echo },
					       { PLDM_OEM_IPMI_BRIDGE, ipmi_cmd } };

uint8_t pldm_oem_handler_query(uint8_t code, void **ret_fn)
{
	if (!ret_fn)
		return PLDM_ERROR;

	pldm_cmd_proc_fn fn = NULL;
	uint8_t i;

	for (i = 0; i < ARRAY_SIZE(pldm_oem_cmd_tbl); i++) {
		if (pldm_oem_cmd_tbl[i].cmd_code == code) {
			fn = pldm_oem_cmd_tbl[i].fn;
			break;
		}
	}

	*ret_fn = (void *)fn;
	return fn ? PLDM_SUCCESS : PLDM_ERROR;
}
