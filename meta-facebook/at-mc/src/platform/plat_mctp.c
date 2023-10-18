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

/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "plat_mctp.h"

#include <zephyr.h>
#include <logging/log.h>
#include "libutil.h"
#include "mctp.h"
#include "mctp_ctrl.h"
#include "pldm.h"
#include "cci.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_mctp);

typedef struct _cxl_mctp_info {
	uint8_t cxl_card_id;
	bool is_set_eid;
} cxl_mctp_info;

static mctp_port smbus_port[] = {
	{ .conf.smbus_conf.addr = I2C_ADDR_BIC, .conf.smbus_conf.bus = I2C_BUS_BMC },
	{ .conf.smbus_conf.addr = I2C_ADDR_BIC, .conf.smbus_conf.bus = I2C_BUS_CXL },
};

static mctp_route_entry mctp_route_tbl[] = {
	{ .endpoint = MCTP_EID_BMC, .bus = I2C_BUS_BMC, .addr = I2C_ADDR_BMC },
	{ .endpoint = MCTP_EID_CXL, .bus = I2C_BUS_CXL, .addr = I2C_ADDR_CXL },
};

static cxl_mctp_info cxl_info[] = {
	{ .cxl_card_id = CXL_CARD_1, .is_set_eid = false },
	{ .cxl_card_id = CXL_CARD_2, .is_set_eid = false },
	{ .cxl_card_id = CXL_CARD_3, .is_set_eid = false },
	{ .cxl_card_id = CXL_CARD_4, .is_set_eid = false },
	{ .cxl_card_id = CXL_CARD_5, .is_set_eid = false },
	{ .cxl_card_id = CXL_CARD_6, .is_set_eid = false },
	{ .cxl_card_id = CXL_CARD_7, .is_set_eid = false },
	{ .cxl_card_id = CXL_CARD_8, .is_set_eid = false },
};

bool get_cxl_eid_flag(uint8_t cxl_card_id)
{
	uint8_t index = 0;
	for (index = 0; index < ARRAY_SIZE(cxl_info); ++index) {
		if (cxl_card_id == cxl_info[index].cxl_card_id) {
			return cxl_info[index].is_set_eid;
		}
	}

	LOG_ERR("Input cxl card id is invalid, card id: 0x%x", cxl_card_id);
	return false;
}

void set_cxl_eid_flag(uint8_t cxl_card_id, bool value)
{
	uint8_t index = 0;
	for (index = 0; index < ARRAY_SIZE(cxl_info); ++index) {
		if (cxl_card_id == cxl_info[index].cxl_card_id) {
			cxl_info[index].is_set_eid = value;
			return;
		}
	}

	LOG_ERR("Input cxl card id is invalid, card id: 0x%x", cxl_card_id);
}

mctp *find_mctp_by_smbus(uint8_t bus)
{
	uint8_t i;
	for (i = 0; i < ARRAY_SIZE(smbus_port); i++) {
		mctp_port *p = smbus_port + i;
		if (!p) {
			return NULL;
		}
		if (bus == p->conf.smbus_conf.bus) {
			return p->mctp_inst;
		}
	}
	return NULL;
}

static uint8_t mctp_msg_recv(void *mctp_p, uint8_t *buf, uint32_t len, mctp_ext_params ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, MCTP_ERROR);

	/** first byte is message type and ic **/
	uint8_t msg_type = (buf[0] & MCTP_MSG_TYPE_MASK) >> MCTP_MSG_TYPE_SHIFT;
	uint8_t ic = (buf[0] & MCTP_IC_MASK) >> MCTP_IC_SHIFT;
	(void)ic;

	switch (msg_type) {
	case MCTP_MSG_TYPE_CTRL:
		mctp_ctrl_cmd_handler(mctp_p, buf, len, ext_params);
		break;

	case MCTP_MSG_TYPE_PLDM:
		mctp_pldm_cmd_handler(mctp_p, buf, len, ext_params);
		break;

	case MCTP_MSG_TYPE_CCI:
		mctp_cci_cmd_handler(mctp_p, buf, len, ext_params);
		break;

	default:
		LOG_WRN("unable to find message receive function");
		return MCTP_ERROR;
	}

	return MCTP_SUCCESS;
}

uint8_t get_mctp_info(uint8_t dest_endpoint, mctp **mctp_inst, mctp_ext_params *ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(ext_params, MCTP_ERROR);

	uint8_t ret = MCTP_ERROR;
	uint32_t index = 0;

	for (index = 0; index < ARRAY_SIZE(mctp_route_tbl); index++) {
		mctp_route_entry *port = mctp_route_tbl + index;
		CHECK_NULL_ARG_WITH_RETURN(port, MCTP_ERROR);

		if (port->endpoint == dest_endpoint) {
			*mctp_inst = find_mctp_by_smbus(port->bus);
			CHECK_NULL_ARG_WITH_RETURN(mctp_inst, MCTP_ERROR);

			ext_params->ep = port->endpoint;
			ext_params->type = MCTP_MEDIUM_TYPE_SMBUS;
			ext_params->smbus_ext_params.addr = port->addr;
			ret = MCTP_SUCCESS;
			break;
		}
	}
	return ret;
}

bool get_set_cxl_endpoint(uint8_t cxl_card_id, uint8_t eid)
{
	uint8_t ret = 0;
	mctp *mctp_inst = NULL;
	mctp_ctrl_msg msg = { 0 };

	ret = get_mctp_info(eid, &mctp_inst, &msg.ext_params);
	if (ret != MCTP_SUCCESS) {
		LOG_ERR("Get mctp route info fail");
		return false;
	}

	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, false);

	/** Get eid from CXL **/
	msg.hdr.cmd = MCTP_CTRL_CMD_GET_ENDPOINT_ID;
	msg.hdr.rq = MCTP_REQUEST;
	msg.cmd_data_len = 0;

	struct _get_eid_resp get_eid_resp = { 0 };
	ret = mctp_ctrl_read(mctp_inst, &msg, (uint8_t *)&get_eid_resp, sizeof(get_eid_resp));
	if (ret != MCTP_SUCCESS) {
		LOG_ERR("Fail to get eid, cxl id: 0x%x", cxl_card_id);
		return false;
	}

	/** Set eid if the getting eid is not match with stored eid **/
	if (get_eid_resp.eid != eid) {
		memset(&msg, 0, sizeof(mctp_ctrl_msg));

		struct _set_eid_req set_eid_req = { 0 };
		struct _set_eid_resp set_eid_resp = { 0 };
		set_eid_req.op = SET_EID_REQ_OP_SET_EID;
		set_eid_req.eid = eid;

		msg.hdr.cmd = MCTP_CTRL_CMD_SET_ENDPOINT_ID;
		msg.hdr.rq = MCTP_REQUEST;
		msg.cmd_data = (uint8_t *)&set_eid_req;
		msg.cmd_data_len = sizeof(set_eid_req);

		ret = mctp_ctrl_read(mctp_inst, &msg, (uint8_t *)&set_eid_resp,
				     sizeof(set_eid_resp));
		if (ret != MCTP_SUCCESS) {
			LOG_ERR("Fail to set eid, card id: 0x%x", cxl_card_id);
			return false;
		}
	}

	set_cxl_eid_flag(cxl_card_id, SET_EID_FLAG);
	return true;
}

int pal_pldm_send_ipmi_request(ipmi_msg *msg, uint8_t eid)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, -1);

	uint8_t ret = MCTP_ERROR;
	uint8_t resp_len = 0;
	pldm_msg pmsg = { 0 };
	uint8_t req_buf[PLDM_MAX_DATA_SIZE] = { 0 };
	uint8_t resp_buf[PLDM_MAX_DATA_SIZE] = { 0 };
	mctp *mctp_inst = NULL;

	ret = get_mctp_info(eid, &mctp_inst, &pmsg.ext_params);
	if (ret != MCTP_SUCCESS) {
		LOG_ERR("Invalid EID: 0x%x, unable to get route information", eid);
		return -1;
	}

	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, -1);

	// Set PLDM header
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

	// Send request to PLDM/MCTP thread and get response
	resp_len = mctp_pldm_read(mctp_inst, &pmsg, resp_buf, sizeof(resp_buf));
	if (resp_len == 0) {
		LOG_ERR("mctp_pldm_read fail");
		return -1;
	}

	struct _pldm_ipmi_cmd_resp *resp = (struct _pldm_ipmi_cmd_resp *)resp_buf;
	if ((resp->completion_code != MCTP_SUCCESS)) {
		resp->ipmi_comp_code = CC_UNSPECIFIED_ERROR;
	}

	msg->completion_code = resp->ipmi_comp_code;
	msg->netfn = resp->netfn_lun >> 2;
	msg->cmd = resp->cmd;
	if (resp_len > MCTP_RESP_HEADER_COUNT) {
		msg->data_len = resp_len - MCTP_RESP_HEADER_COUNT;
		memcpy(msg->data, &resp_buf[MCTP_RESP_DATA_INDEX], msg->data_len);
	} else {
		msg->data_len = 0;
	}

	return 0;
}

void plat_mctp_init(void)
{
	LOG_INF("plat_mctp_init");

	uint8_t ret = 0;
	uint8_t index = 0;

	for (index = 0; index < ARRAY_SIZE(smbus_port); index++) {
		mctp_port *port = smbus_port + index;
		CHECK_NULL_ARG(port);

		if (port->conf.smbus_conf.bus == I2C_BUS_CXL) {
			/** BIC will not initialize mctp instance if CXL is not present **/
			if (is_cxl_present() == false) {
				continue;
			}
		}

		/** init mctp for bmc bus **/
		port->mctp_inst = mctp_init();
		CHECK_NULL_ARG(port->mctp_inst);

		ret = mctp_set_medium_configure(port->mctp_inst, MCTP_MEDIUM_TYPE_SMBUS,
						port->conf);
		if (ret != MCTP_SUCCESS) {
			LOG_INF("[%s] mctp set medium configure failed", __func__);
		}

		mctp_reg_msg_rx_func(port->mctp_inst, mctp_msg_recv);
		mctp_start(port->mctp_inst);
	}

	LOG_DBG("mctp_start");
}
