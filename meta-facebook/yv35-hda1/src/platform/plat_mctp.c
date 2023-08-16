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

#include <zephyr.h>
#include <sys/printk.h>
#include <logging/log.h>
#include <logging/log_ctrl.h>
#include <stdlib.h>
#include "mctp.h"
#include "mctp_ctrl.h"
#include "pldm.h"
#include "ipmi.h"
#include "sensor.h"
#include "plat_power_status.h"
#include "plat_mctp.h"
#include "plat_pldm.h"
#include "plat_gpio.h"
#include "plat_i2c.h"
#include "hal_i2c_target.h"

LOG_MODULE_REGISTER(plat_mctp);

#define MCTP_MSG_TYPE_SHIFT 0
#define MCTP_MSG_TYPE_MASK 0x7F
#define MCTP_IC_SHIFT 7
#define MCTP_IC_MASK 0x80

#define MAX_PLDM_EVENT_RECV_BUFF_SIZE 0xC0 //192 Bytes

K_WORK_DEFINE(send_cmd_work, send_cmd_to_dev_handler);

typedef struct _mctp_smbus_port {
	mctp *mctp_inst;
	mctp_medium_conf conf;
	uint8_t user_idx;
} mctp_smbus_port;

/* mctp route entry struct */
typedef struct _mctp_route_entry {
	uint8_t endpoint;
	uint8_t bus; /* TODO: only consider smbus/i3c */
	uint8_t addr; /* TODO: only consider smbus/i3c */
	uint8_t dev_present_pin;
} mctp_route_entry;

typedef struct _mctp_msg_handler {
	MCTP_MSG_TYPE type;
	mctp_fn_cb msg_handler_cb;
} mctp_msg_handler;

static mctp_smbus_port smbus_port[] = {
	{ .conf.smbus_conf.addr = I2C_ADDR_BIC, .conf.smbus_conf.bus = I2C_BUS_MPRO },
};

mctp_route_entry mctp_route_tbl[] = {
	{ MCTP_EID_MPRO, I2C_BUS_MPRO, I2C_ADDR_MPRO },
};

static mctp *find_mctp_by_smbus(uint8_t bus)
{
	uint8_t i;
	for (i = 0; i < ARRAY_SIZE(smbus_port); i++) {
		mctp_smbus_port *p = smbus_port + i;

		if (bus == p->conf.smbus_conf.bus)
			return p->mctp_inst;
	}

	return NULL;
}

static uint8_t get_mctp_route_info(uint8_t dest_endpoint, void **mctp_inst,
				   mctp_ext_params *ext_params)
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

uint8_t get_mctp_info(uint8_t dest_endpoint, mctp **mctp_inst, mctp_ext_params *ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(ext_params, MCTP_ERROR);

	uint8_t rc = MCTP_ERROR;
	uint32_t i;

	for (i = 0; i < ARRAY_SIZE(mctp_route_tbl); i++) {
		mctp_route_entry *p = mctp_route_tbl + i;
		if (!p) {
			return MCTP_ERROR;
		}
		if (p->endpoint == dest_endpoint) {
			*mctp_inst = find_mctp_by_smbus(p->bus);
			ext_params->type = MCTP_MEDIUM_TYPE_SMBUS;
			ext_params->smbus_ext_params.addr = p->addr;
			rc = MCTP_SUCCESS;
			break;
		}
	}
	return rc;
}

static void set_endpoint_resp_handler(void *args, uint8_t *buf, uint16_t len)
{
	if (!buf || !len)
		return;
	LOG_HEXDUMP_INF(buf, len, "Set device eid:");
}

static void set_endpoint_resp_timeout(void *args)
{
	mctp_route_entry *p = (mctp_route_entry *)args;
	LOG_ERR("Endpoint 0x%x set endpoint failed on bus %d", p->endpoint, p->bus);
}

static void set_dev_endpoint(void)
{
	mctp_route_entry *p = mctp_route_tbl;

	for (uint8_t j = 0; j < ARRAY_SIZE(smbus_port); j++) {
		if (p->bus != smbus_port[j].conf.smbus_conf.bus)
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
		msg.timeout_cb_fn_args = p;

		mctp_ctrl_send_msg(find_mctp_by_smbus(p->bus), &msg);
	}
}

static void set_tid(void)
{
	uint8_t ret = MCTP_ERROR;
	uint8_t resp_len = 0;
	pldm_msg pmsg = { 0 };
	uint8_t resp_buf[PLDM_MAX_DATA_SIZE] = { 0 };
	mctp *mctp_inst = NULL;

	ret = get_mctp_route_info(MCTP_EID_MPRO, (void **)&mctp_inst, &pmsg.ext_params);
	if (ret != MCTP_SUCCESS) {
		LOG_ERR("Invalid EID: 0x%x, unable to get route information", MCTP_EID_MPRO);
	}

	pmsg.hdr.msg_type = MCTP_MSG_TYPE_PLDM;
	pmsg.hdr.pldm_type = PLDM_TYPE_BASE;
	pmsg.hdr.cmd = PLDM_BASE_CMD_CODE_SETTID;
	pmsg.hdr.rq = PLDM_REQUEST;

	struct _set_tid_req req = { 0 };
	req.tid = PLDM_TID_MPRO;

	pmsg.buf = (uint8_t *)&req;
	pmsg.len = sizeof(req);

	resp_len = mctp_pldm_read(mctp_inst, &pmsg, resp_buf, sizeof(resp_buf));
	if (resp_len == 0) {
		LOG_ERR("Set tid receiver FAILED!");
		return;
	}

	struct _set_tid_resp *resp = (struct _set_tid_resp *)resp_buf;
	if (resp->completion_code == PLDM_SUCCESS)
		LOG_INF("Set tid receiver SUCCESS!");
	else
		LOG_ERR("Set tid receiver response = 0x%x", resp->completion_code);
}

static void set_event_receiver(void)
{
	uint8_t ret = MCTP_ERROR;
	uint8_t resp_len = 0;
	pldm_msg pmsg = { 0 };
	uint8_t resp_buf[PLDM_MAX_DATA_SIZE] = { 0 };
	mctp *mctp_inst = NULL;

	ret = get_mctp_route_info(MCTP_EID_MPRO, (void **)&mctp_inst, &pmsg.ext_params);
	if (ret != MCTP_SUCCESS) {
		LOG_ERR("Invalid EID: 0x%x, unable to get route information", MCTP_EID_MPRO);
	}

	pmsg.hdr.msg_type = MCTP_MSG_TYPE_PLDM;
	pmsg.hdr.pldm_type = PLDM_TYPE_PLAT_MON_CTRL;
	pmsg.hdr.cmd = PLDM_MONITOR_CMD_CODE_SET_EVENT_RECEIVER;
	pmsg.hdr.rq = PLDM_REQUEST;

	struct pldm_set_event_receiver_req req = { 0 };
	req.event_message_global_enable = 0x02; //Victor test
	req.transport_protocol_type = 0x00;
	req.event_receiver_address_info = 0x0A;
	req.heartbeat_timer = 0x0000;

	pmsg.buf = (uint8_t *)&req;
	pmsg.len = sizeof(req);

	resp_len = mctp_pldm_read(mctp_inst, &pmsg, resp_buf, sizeof(resp_buf));
	if (resp_len == 0) {
		LOG_ERR("Set event receiver FAILED!");
		return;
	}

	struct pldm_set_event_receiver_resp *resp = (struct pldm_set_event_receiver_resp *)resp_buf;
	if (resp->completion_code == PLDM_SUCCESS)
		LOG_INF("Set event receiver SUCCESS!");
	else
		LOG_ERR("Set event receiver response = 0x%x", resp->completion_code);
}

static void event_message_buffer_size(void)
{
	uint8_t ret = MCTP_ERROR;
	uint8_t resp_len = 0;
	pldm_msg pmsg = { 0 };
	uint8_t resp_buf[PLDM_MAX_DATA_SIZE] = { 0 };
	mctp *mctp_inst = NULL;

	ret = get_mctp_route_info(MCTP_EID_MPRO, (void **)&mctp_inst, &pmsg.ext_params);
	if (ret != MCTP_SUCCESS) {
		LOG_ERR("Invalid EID: 0x%x, unable to get route information", MCTP_EID_MPRO);
	}

	pmsg.hdr.msg_type = MCTP_MSG_TYPE_PLDM;
	pmsg.hdr.pldm_type = PLDM_TYPE_PLAT_MON_CTRL;
	pmsg.hdr.cmd = PLDM_MONITOR_CMD_CODE_EVENT_MESSAGE_BUFF_SIZE;
	pmsg.hdr.rq = PLDM_REQUEST;

	struct pldm_event_message_buffer_size_req req = { 0 };
	req.event_receiver_max_buffer_size = MAX_PLDM_EVENT_RECV_BUFF_SIZE;

	pmsg.buf = (uint8_t *)&req;
	pmsg.len = sizeof(req);

	resp_len = mctp_pldm_read(mctp_inst, &pmsg, resp_buf, sizeof(resp_buf));
	if (resp_len == 0) {
		LOG_ERR("Event message buffer size set FAILED!");
		return;
	}

	struct pldm_event_message_buffer_size_resp *resp =
		(struct pldm_event_message_buffer_size_resp *)resp_buf;
	if (resp->completion_code == PLDM_SUCCESS) {
		LOG_INF("Event message buffer size set %d SUCCESS!", resp->term_max_buff_size);
	} else
		LOG_ERR("Event message buffer size response = 0x%x", resp->completion_code);
}

static uint8_t mctp_msg_recv(void *mctp_p, uint8_t *buf, uint32_t len, mctp_ext_params ext_params)
{
	if (!mctp_p || !buf || !len)
		return MCTP_ERROR;

	/* first byte is message type and ic */
	uint8_t msg_type = (buf[0] & MCTP_MSG_TYPE_MASK) >> MCTP_MSG_TYPE_SHIFT;
	uint8_t ic = (buf[0] & MCTP_IC_MASK) >> MCTP_IC_SHIFT;
	(void)ic;

	switch (msg_type) {
	case MCTP_MSG_TYPE_CTRL:
		mctp_ctrl_cmd_handler(mctp_p, buf, len, ext_params);
		break;

	case MCTP_MSG_TYPE_PLDM:
		if (pldm_request_msg_need_bypass(buf, len) == true) {
			ipmi_msg bridge_msg = { 0 };
			bridge_msg.seq_source = 0xff; // No seq for MPRO
			bridge_msg.InF_source = MPRO_PLDM;
			bridge_msg.InF_target =
				BMC_IPMB; // default bypassing IPMI standard command to BMC
			bridge_msg.netfn = NETFN_OEM_1S_REQ;
			bridge_msg.cmd = CMD_OEM_1S_MSG_IN;

			bridge_msg.data_len =
				len - 1 + 3 + 1; // exclude msg_type and include IANA, channel
			bridge_msg.data[0] = IANA_ID & 0xFF;
			bridge_msg.data[1] = (IANA_ID >> 8) & 0xFF;
			bridge_msg.data[2] = (IANA_ID >> 16) & 0xFF;
			bridge_msg.data[3] = MPRO_PLDM;

			memcpy(&bridge_msg.data[4], &buf[1], len - 1);

			LOG_HEXDUMP_INF(&bridge_msg.data[0], bridge_msg.data_len,
					"Bridging pldm req msg:");

			ipmb_error status =
				ipmb_send_request(&bridge_msg, IPMB_inf_index_map[BMC_IPMB]);
			if (status != IPMB_ERROR_SUCCESS) {
				LOG_ERR("Fail to send pldm bridge ipmb request msg with ret 0x%x",
					status);
				return MCTP_ERROR;
			}

			/* Record mctp relative info */
			pldm_hdr *hdr = (pldm_hdr *)buf;
			if (pldm_save_mctp_inst_from_ipmb_req(mctp_p, hdr->inst_id, ext_params) ==
			    false) {
				LOG_ERR("Failed to save bridge info via inst id %d", hdr->inst_id);
				return MCTP_ERROR;
			}

			return MCTP_SUCCESS;
		}

		mctp_pldm_cmd_handler(mctp_p, buf, len, ext_params);
		break;

	default:
		LOG_WRN("Cannot find message receive function!!");
		return MCTP_ERROR;
	}

	return MCTP_SUCCESS;
}

void send_cmd_to_dev_handler(struct k_work *work)
{
	/* mctp - base */
	set_dev_endpoint();
	/* pldm - base */
	set_tid();
	/* pldm - monitor */
	set_event_receiver();
	event_message_buffer_size();

	set_mpro_status();
}

void send_cmd_to_dev(struct k_timer *timer)
{
	k_work_submit(&send_cmd_work);
}

void plat_mctp_init(void)
{
	LOG_INF("plat_mctp_init");

	/* init the mctp/pldm instance */
	for (uint8_t i = 0; i < ARRAY_SIZE(smbus_port); i++) {
		mctp_smbus_port *p = smbus_port + i;
		LOG_DBG("smbus port %d", i);
		LOG_DBG("bus = %x, addr = %x", p->conf.smbus_conf.bus, p->conf.smbus_conf.addr);

		struct _i2c_target_config cfg;
		memset(&cfg, 0, sizeof(cfg));
		cfg.address = p->conf.smbus_conf.addr;
		cfg.i2c_msg_count = 0x0A;

		if (i2c_target_control(p->conf.smbus_conf.bus, &cfg, I2C_CONTROL_REGISTER) !=
		    I2C_TARGET_API_NO_ERR) {
			LOG_ERR("i2c %d register target failed", p->conf.smbus_conf.bus);
			continue;
		}

		p->mctp_inst = mctp_init();
		if (!p->mctp_inst) {
			LOG_ERR("mctp_init failed!!");
			continue;
		}

		LOG_DBG("mctp_inst = %p", p->mctp_inst);
		uint8_t rc =
			mctp_set_medium_configure(p->mctp_inst, MCTP_MEDIUM_TYPE_SMBUS, p->conf);
		LOG_DBG("mctp_set_medium_configure %s",
			(rc == MCTP_SUCCESS) ? "success" : "failed");

		mctp_reg_endpoint_resolve_func(p->mctp_inst, get_mctp_route_info);
		mctp_reg_msg_rx_func(p->mctp_inst, mctp_msg_recv);

		mctp_start(p->mctp_inst);
	}
}
