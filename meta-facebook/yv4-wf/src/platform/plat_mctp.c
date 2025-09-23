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
 *
 *
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "plat_mctp.h"

#include <zephyr.h>
#include <sys/printk.h>
#include <logging/log.h>
#include <logging/log_ctrl.h>
#include <stdlib.h>
#include <stdio.h>
#include "mctp.h"
#include "mctp_ctrl.h"
#include "pldm.h"
#include "ipmi.h"
#include "sensor.h"
#include "cci.h"
#include "plat_ipmb.h"
#include "plat_power_seq.h"
#include "plat_pldm_sensor.h"

#include "hal_i2c.h"

LOG_MODULE_REGISTER(plat_mctp);

uint8_t plat_eid = MCTP_DEFAULT_ENDPOINT;

K_THREAD_STACK_DEFINE(set_dev_endpoint_stack, SET_DEV_ENDPOINT_STACK_SIZE);
struct k_thread set_dev_endpoint_thread_data;
static k_tid_t set_dev_endpoint_tid = NULL;

static mctp_port plat_mctp_port[] = {
	{ .conf.smbus_conf.addr = I3C_ADDR_SD_BIC,
	  .conf.i3c_conf.bus = I3C_BUS_SD_BIC,
	  .medium_type = MCTP_MEDIUM_TYPE_TARGET_I3C },
	{ .conf.smbus_conf.addr = I2C_ADDR_BIC,
	  .conf.smbus_conf.bus = I2C_BUS_CXL1,
	  .medium_type = MCTP_MEDIUM_TYPE_SMBUS },
	{ .conf.smbus_conf.addr = I2C_ADDR_BIC,
	  .conf.smbus_conf.bus = I2C_BUS_CXL2,
	  .medium_type = MCTP_MEDIUM_TYPE_SMBUS },
};

mctp_route_entry plat_mctp_route_tbl[] = {
	{ MCTP_EID_BMC, I3C_BUS_SD_BIC, I3C_ADDR_SD_BIC, .set_endpoint = false },
	{ MCTP_EID_SD_BIC, I3C_BUS_SD_BIC, I3C_ADDR_SD_BIC, .set_endpoint = false },
	{ MCTP_EID_CXL1, I2C_BUS_CXL1, I2C_ADDR_CXL1, .set_endpoint = true },
	{ MCTP_EID_CXL2, I2C_BUS_CXL2, I2C_ADDR_CXL2, .set_endpoint = true },
};

uint8_t MCTP_SUPPORTED_MESSAGES_TYPES[] = {
	TYPE_MCTP_CONTROL,
	TYPE_PLDM,
};

mctp *find_mctp_by_bus(uint8_t bus)
{
	uint8_t i;
	for (i = 0; i < ARRAY_SIZE(plat_mctp_port); i++) {
		mctp_port *p = plat_mctp_port + i;

		if (p->medium_type == MCTP_MEDIUM_TYPE_SMBUS) {
			if (bus == p->conf.smbus_conf.bus) {
				return p->mctp_inst;
			}
		} else if (p->medium_type == MCTP_MEDIUM_TYPE_TARGET_I3C) {
			if (bus == p->conf.i3c_conf.bus) {
				return p->mctp_inst;
			}
		} else {
			LOG_ERR("Unknown medium type");
			return NULL;
		}
	}

	return NULL;
}

static void set_endpoint_resp_handler(void *args, uint8_t *buf, uint16_t len)
{
	ARG_UNUSED(args);
	CHECK_NULL_ARG(buf);

	LOG_HEXDUMP_DBG(buf, len, __func__);
}

static void set_endpoint_resp_timeout(void *args)
{
	CHECK_NULL_ARG(args);

	mctp_route_entry *p = (mctp_route_entry *)args;
	LOG_DBG("Endpoint 0x%x set endpoint failed on bus %d", p->endpoint, p->bus);
}

bool check_cxl_eid(uint8_t cxl_id)
{
	uint8_t eid = plat_get_cxl_eid(cxl_id);
	mctp *mctp_inst = NULL;
	mctp_ext_params ext_params = { 0 };
	if (get_mctp_info(eid, &mctp_inst, &ext_params) != MCTP_SUCCESS)
		return false;

	mctp_ctrl_msg msg = { 0 };
	msg.hdr.rq = 1;
	msg.hdr.cmd = MCTP_CTRL_CMD_GET_ENDPOINT_ID;
	msg.ext_params = ext_params;

	uint8_t resp_buf[8] = { 0 };
	if (mctp_ctrl_read(mctp_inst, &msg, resp_buf, sizeof(resp_buf)) != MCTP_SUCCESS)
		return false;

	// Response[0] = completion code, must be 0x00
	if (resp_buf[0] != 0x00)
		return false;

	return true;
}

bool set_cxl_eid(uint8_t cxl_id)
{
	if (cxl_id != CXL_ID_1 && cxl_id != CXL_ID_2) {
		LOG_ERR("set_cxl_eid: invalid cxl_id=%d", cxl_id);
		return false;
	}

	for (uint8_t i = 0; i < ARRAY_SIZE(plat_mctp_route_tbl); i++) {
		const mctp_route_entry *p = plat_mctp_route_tbl + i;
		if (!p->set_endpoint)
			continue;

		if ((cxl_id == CXL_ID_1 && p->bus != I2C_BUS_CXL1) ||
		    (cxl_id == CXL_ID_2 && p->bus != I2C_BUS_CXL2))
			continue;

		struct _set_eid_req req = { 0 };
		req.op = SET_EID_REQ_OP_SET_EID;
		req.eid = p->endpoint;

		mctp_ctrl_msg msg;
		memset(&msg, 0, sizeof(msg));
		msg.ext_params.type = MCTP_MEDIUM_TYPE_SMBUS;
		msg.ext_params.smbus_ext_params.addr = p->addr;

		msg.hdr.cmd = MCTP_CTRL_CMD_SET_ENDPOINT_ID;
		msg.hdr.rq = 1;

		msg.cmd_data = (uint8_t *)&req;
		msg.cmd_data_len = sizeof(req);

		uint8_t rc = mctp_ctrl_send_msg(find_mctp_by_bus(p->bus), &msg);

		if (!rc) {
			LOG_INF("Send set EID command to CXL%d success", cxl_id + 1);
			return true;
		} else {
			LOG_ERR("Fail to set EID %d for CXL%d", p->endpoint, cxl_id + 1);
			return false;
		}
	}
	LOG_ERR("No matching entry to set EID for CXL%d", cxl_id + 1);
	return false;
}

static void set_dev_endpoint(void)
{
	bool set_eid[MAX_CXL_ID] = { false, false };
	// The CXL FW is unstable and its booting up time is random now.
	// Temporary add retry mechanism for it.
	for (int attempt = 0; attempt < 60; attempt++) {
		// We only need to set CXL EID.
		for (uint8_t i = 0; i < ARRAY_SIZE(plat_mctp_route_tbl); i++) {
			const mctp_route_entry *p = plat_mctp_route_tbl + i;
			if (!p->set_endpoint)
				continue;

			// Check CXLs ready status before setting EID
			if (p->bus == I2C_BUS_CXL1 && !get_cxl_ready_status(CXL_ID_1))
				continue;

			if (p->bus == I2C_BUS_CXL2 && !get_cxl_ready_status(CXL_ID_2))
				continue;

			for (uint8_t j = 0; j < ARRAY_SIZE(plat_mctp_port); j++) {
				if (p->bus != plat_mctp_port[j].conf.smbus_conf.bus)
					continue;

				struct _set_eid_req req = { 0 };
				req.op = SET_EID_REQ_OP_SET_EID;
				req.eid = p->endpoint;

				mctp_ctrl_msg msg;
				memset(&msg, 0, sizeof(msg));
				msg.ext_params.type = MCTP_MEDIUM_TYPE_SMBUS;
				msg.ext_params.smbus_ext_params.addr = p->addr;

				msg.hdr.cmd = MCTP_CTRL_CMD_SET_ENDPOINT_ID;
				msg.hdr.rq = 1;

				msg.cmd_data = (uint8_t *)&req;
				msg.cmd_data_len = sizeof(req);

				msg.recv_resp_cb_fn = set_endpoint_resp_handler;
				msg.timeout_cb_fn = set_endpoint_resp_timeout;
				msg.timeout_cb_fn_args = (void *)p;

				uint8_t rc = mctp_ctrl_send_msg(find_mctp_by_bus(p->bus), &msg);
				if (!rc) {
					switch (p->bus) {
					case I2C_BUS_CXL1: {
						LOG_INF("Send set EID command to CXL1");
						set_eid[CXL_ID_1] = true;
						break;
					}
					case I2C_BUS_CXL2: {
						LOG_INF("Send set EID command to CXL2");
						set_eid[CXL_ID_2] = true;
						break;
					}
					}
				} else {
					LOG_ERR("Fail to set EID %d", p->endpoint);
				}
			}
		}
		// break if set both CXL EID success
		if (set_eid[CXL_ID_1] == true && set_eid[CXL_ID_2] == true)
			break;
		// Delay for 10 seconds before the next attempt
		k_sleep(K_SECONDS(10));
	}
}

static uint8_t mctp_msg_recv(void *mctp_p, uint8_t *buf, uint32_t len, mctp_ext_params ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, MCTP_ERROR);
	/* first byte is message type */
	uint8_t msg_type = (buf[0] & MCTP_MSG_TYPE_MASK) >> MCTP_MSG_TYPE_SHIFT;

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
		LOG_WRN("Cannot find message receive function!!");
		return MCTP_ERROR;
	}

	return MCTP_SUCCESS;
}

static uint8_t get_mctp_route_info(uint8_t dest_endpoint, void **mctp_inst,
				   mctp_ext_params *ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(ext_params, MCTP_ERROR);

	uint8_t rc = MCTP_ERROR;
	uint32_t i;

	for (i = 0; i < ARRAY_SIZE(plat_mctp_route_tbl); i++) {
		const mctp_route_entry *p = plat_mctp_route_tbl + i;
		if (p->endpoint == dest_endpoint) {
			*mctp_inst = find_mctp_by_bus(p->bus);
			if (p->bus != I3C_BUS_SD_BIC) {
				ext_params->type = MCTP_MEDIUM_TYPE_SMBUS;
				ext_params->smbus_ext_params.addr = p->addr;
			} else {
				ext_params->type = MCTP_MEDIUM_TYPE_TARGET_I3C;
				ext_params->i3c_ext_params.addr = p->addr;
			}
			rc = MCTP_SUCCESS;
			break;
		}
	}

	return rc;
}

uint8_t get_mctp_info(uint8_t dest_endpoint, mctp **mctp_inst, mctp_ext_params *ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(ext_params, MCTP_ERROR);

	uint8_t rc = MCTP_ERROR;
	uint32_t i;

	for (i = 0; i < ARRAY_SIZE(plat_mctp_route_tbl); i++) {
		const mctp_route_entry *p = plat_mctp_route_tbl + i;
		if (p->endpoint == dest_endpoint) {
			*mctp_inst = find_mctp_by_bus(p->bus);
			if (p->bus != I3C_BUS_SD_BIC) {
				ext_params->type = MCTP_MEDIUM_TYPE_SMBUS;
				ext_params->smbus_ext_params.addr = p->addr;
			} else {
				ext_params->type = MCTP_MEDIUM_TYPE_TARGET_I3C;
				ext_params->i3c_ext_params.addr = p->addr;
			}
			ext_params->ep = p->endpoint;
			rc = MCTP_SUCCESS;
			break;
		}
	}

	return rc;
}

void set_dev_endpoint_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	/* init the device endpoint */
	set_dev_endpoint();
}

void create_set_dev_endpoint_thread()
{
	if ((set_dev_endpoint_tid != NULL) &&
	    ((strcmp(k_thread_state_str(set_dev_endpoint_tid), "dead") != 0) &&
	     (strcmp(k_thread_state_str(set_dev_endpoint_tid), "unknown") != 0))) {
		LOG_ERR("Delete exited thread");
		k_thread_abort(set_dev_endpoint_tid);
	}

	set_dev_endpoint_tid =
		k_thread_create(&set_dev_endpoint_thread_data, set_dev_endpoint_stack,
				K_THREAD_STACK_SIZEOF(set_dev_endpoint_stack),
				set_dev_endpoint_thread, NULL, NULL, NULL,
				CONFIG_MAIN_THREAD_PRIORITY + 1, 0, K_SECONDS(10));

	k_thread_name_set(set_dev_endpoint_tid, "set_dev_endpoint_thread");
	k_thread_start(set_dev_endpoint_tid);
}

int pal_get_medium_type(uint8_t interface)
{
	int medium_type = -1;

	switch (interface) {
	case BMC_IPMB:
	case MCTP:
	case PLDM:
		medium_type = MCTP_MEDIUM_TYPE_SMBUS;
		break;
	default:
		break;
	}

	return medium_type;
}

mctp *pal_get_mctp(uint8_t mctp_medium_type, uint8_t bus)
{
	switch (mctp_medium_type) {
	case MCTP_MEDIUM_TYPE_SMBUS:
	case MCTP_MEDIUM_TYPE_TARGET_I3C:
	case MCTP_MEDIUM_TYPE_CONTROLLER_I3C:
		return find_mctp_by_bus(bus);
	default:
		return NULL;
	}
}

void plat_mctp_init(void)
{
	int ret = 0;

	/* init the mctp/pldm instance */
	for (uint8_t i = 0; i < ARRAY_SIZE(plat_mctp_port); i++) {
		mctp_port *p = plat_mctp_port + i;

		p->mctp_inst = mctp_init();
		if (!p->mctp_inst) {
			LOG_ERR("mctp_init failed!!");
			continue;
		}

		uint8_t rc = mctp_set_medium_configure(p->mctp_inst, p->medium_type, p->conf);
		if (rc != MCTP_SUCCESS) {
			LOG_ERR("mctp set medium configure failed");
		}

		mctp_reg_endpoint_resolve_func(p->mctp_inst, get_mctp_route_info);

		mctp_reg_msg_rx_func(p->mctp_inst, mctp_msg_recv);

		ret = mctp_start(p->mctp_inst);
	}
}

uint8_t plat_get_mctp_port_count()
{
	return ARRAY_SIZE(plat_mctp_port);
}

mctp_port *plat_get_mctp_port(uint8_t index)
{
	return plat_mctp_port + index;
}

void plat_update_mctp_routing_table(uint8_t eid)
{
	// Set platform eid
	plat_eid = eid;
	LOG_INF("plat_eid= %d ", plat_eid);

	// update sd bic eid
	mctp_route_entry *p = plat_mctp_route_tbl + 1;
	p->endpoint = eid - 2;

	// update cxl1 eid
	p = plat_mctp_route_tbl + 2;
	p->endpoint = eid + 2;

	// update cxl2 eid
	p = plat_mctp_route_tbl + 3;
	p->endpoint = eid + 3;

	update_entity_name_with_eid(eid);

	// send set eid to cxl
	create_set_dev_endpoint_thread();

	return;
}

int load_mctp_support_types(uint8_t *type_len, uint8_t *types)
{
	*type_len = sizeof(MCTP_SUPPORTED_MESSAGES_TYPES);
	memcpy(types, MCTP_SUPPORTED_MESSAGES_TYPES, sizeof(MCTP_SUPPORTED_MESSAGES_TYPES));
	return MCTP_SUCCESS;
}

uint8_t plat_get_eid()
{
	return plat_eid;
}

uint8_t plat_get_cxl_eid(uint8_t cxl_id)
{
	switch (cxl_id) {
	case CXL_ID_1:
		return (plat_eid + 2);
	case CXL_ID_2:
		return (plat_eid + 3);
	default:
		return UNKNOWN_CXL_EID;
	}
}

int pal_get_cci_timeout_ms()
{
	// 5 seconds
	return 5000;
}

bool pal_is_need_mctp_interval(mctp *mctp_inst)
{
	switch (mctp_inst->medium_type) {
	case MCTP_MEDIUM_TYPE_SMBUS: {
		const mctp_smbus_conf *smbus_conf =
			(const mctp_smbus_conf *)&mctp_inst->medium_conf;
		if ((smbus_conf->bus == I2C_BUS_CXL1) || (smbus_conf->bus == I2C_BUS_CXL2)) {
			return true;
		}
		break;
	}
	case MCTP_MEDIUM_TYPE_CONTROLLER_I3C:
	case MCTP_MEDIUM_TYPE_TARGET_I3C:
	default:
		break;
	}

	return false;
}

int pal_get_mctp_interval_ms(mctp *mctp_inst)
{
	switch (mctp_inst->medium_type) {
	case MCTP_MEDIUM_TYPE_SMBUS: {
		const mctp_smbus_conf *smbus_conf =
			(const mctp_smbus_conf *)&mctp_inst->medium_conf;
		if ((smbus_conf->bus == I2C_BUS_CXL1) || (smbus_conf->bus == I2C_BUS_CXL2)) {
			// 40ms
			return 40;
		}
		break;
	}
	case MCTP_MEDIUM_TYPE_CONTROLLER_I3C:
	case MCTP_MEDIUM_TYPE_TARGET_I3C:
	default:
		break;
	}

	return 0;
}
