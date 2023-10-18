/*
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
#include "plat_ipmb.h"
#include "plat_class.h"

#include "hal_i2c.h"

LOG_MODULE_REGISTER(plat_mctp);

#define MCTP_MSG_TYPE_SHIFT 0
#define MCTP_MSG_TYPE_MASK 0x7F
#define MCTP_IC_SHIFT 7
#define MCTP_IC_MASK 0x80

/* i2c 8 bit address */
#define I2C_ADDR_BIC 0x40
#define I2C_ADDR_BMC 0x20

/* i2c dev bus*/
#define I2C_BUS_BMC 0x02

/* mctp endpoint */
#define MCTP_EID_BMC 0x08
#define MCTP_EID_SELF 0x0A

K_TIMER_DEFINE(send_cmd_timer, send_cmd_to_dev, NULL);
K_WORK_DEFINE(send_cmd_work, send_cmd_to_dev_handler);

static mctp_port plat_mctp_port[] = {
	{ .conf.smbus_conf.addr = I2C_ADDR_BIC,
	  .conf.smbus_conf.bus = I2C_BUS_BMC,
	  .medium_type = MCTP_MEDIUM_TYPE_SMBUS },
};

mctp_route_entry mctp_route_tbl[] = {
	{ MCTP_EID_BMC, I2C_BUS_BMC, I2C_ADDR_BMC },
};

static mctp *find_mctp_by_smbus(uint8_t bus)
{
	uint8_t i;
	for (i = 0; i < ARRAY_SIZE(plat_mctp_port); i++) {
		mctp_port *p = plat_mctp_port + i;

		if (bus == p->conf.smbus_conf.bus) {
			return p->mctp_inst;
		}
	}

	return NULL;
}

static void set_endpoint_resp_handler(void *args, uint8_t *buf, uint16_t len)
{
	ARG_UNUSED(args);
	CHECK_NULL_ARG(buf);
	//TODO: support set device endpoint
	LOG_HEXDUMP_DBG(buf, len, __func__);
}

static void set_endpoint_resp_timeout(void *args)
{
	CHECK_NULL_ARG(args);
	//TODO: support set device endpoint
	mctp_route_entry *p = (mctp_route_entry *)args;
	LOG_DBG("Endpoint 0x%x set endpoint failed on bus %d", p->endpoint, p->bus);
}

static void set_dev_endpoint(void)
{
	for (uint8_t i = 0; i < ARRAY_SIZE(mctp_route_tbl); i++) {
		mctp_route_entry *p = mctp_route_tbl + i;

		/* skip BMC */
		if (p->bus == I2C_BUS_BMC && p->addr == I2C_ADDR_BMC)
			continue;

		for (uint8_t j = 0; j < ARRAY_SIZE(plat_mctp_port); j++) {
			if (p->bus != plat_mctp_port[j].conf.smbus_conf.bus)
				continue;

			struct _set_eid_req req = { 0 };
			req.op = SET_EID_REQ_OP_SET_EID;
			req.eid = p->endpoint;

			mctp_ctrl_msg msg;
			memset(&msg, 0, sizeof(msg));
			msg.ext_params.type = plat_mctp_port[j].medium_type;
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

	for (i = 0; i < ARRAY_SIZE(mctp_route_tbl); i++) {
		mctp_route_entry *p = mctp_route_tbl + i;
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

void send_cmd_to_dev_handler(struct k_work *work)
{
	/* init the device endpoint */
	set_dev_endpoint();
}

void send_cmd_to_dev(struct k_timer *timer)
{
	k_work_submit(&send_cmd_work);
}

bool mctp_add_sel_to_ipmi(common_addsel_msg_t *sel_msg)
{
	CHECK_NULL_ARG_WITH_RETURN(sel_msg, false);

	uint8_t system_event_record = 0x02;
	uint8_t evt_msg_version = 0x04;

	pldm_msg msg = { 0 };
	struct mctp_to_ipmi_sel_req req = { 0 };

	msg.ext_params.type = MCTP_MEDIUM_TYPE_SMBUS;
	msg.ext_params.smbus_ext_params.addr = I2C_ADDR_BMC;

	msg.hdr.pldm_type = PLDM_TYPE_OEM;
	msg.hdr.cmd = PLDM_OEM_IPMI_BRIDGE;
	msg.hdr.rq = 1;

	msg.buf = (uint8_t *)&req;
	msg.len = sizeof(struct mctp_to_ipmi_sel_req);

	if (set_iana(req.header.iana, sizeof(req.header.iana))) {
		LOG_ERR("Set IANA fail");
		return false;
	}

	req.header.netfn_lun = (NETFN_STORAGE_REQ << 2);
	req.header.ipmi_cmd = CMD_STORAGE_ADD_SEL;
	req.req_data.event.record_type = system_event_record;
	req.req_data.event.gen_id[0] = (I2C_ADDR_BIC << 1);
	req.req_data.event.evm_rev = evt_msg_version;

	memcpy(&req.req_data.event.sensor_type, &sel_msg->sensor_type,
	       sizeof(common_addsel_msg_t) - sizeof(uint8_t));

	uint8_t resp_len = sizeof(struct mctp_to_ipmi_sel_resp);
	uint8_t rbuf[resp_len];

	if (!mctp_pldm_read(find_mctp_by_smbus(I2C_BUS_BMC), &msg, rbuf, resp_len)) {
		LOG_ERR("mctp_pldm_read fail");
		return false;
	}

	struct mctp_to_ipmi_sel_resp *resp = (struct mctp_to_ipmi_sel_resp *)rbuf;

	if ((resp->header.completion_code != MCTP_SUCCESS) ||
	    (resp->header.ipmi_comp_code != CC_SUCCESS)) {
		LOG_ERR("Check reponse completion code fail %x %x", resp->header.completion_code,
			resp->header.ipmi_comp_code);
		return false;
	}

	return true;
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

int pal_get_target(uint8_t interface)
{
	int target = -1;

	switch (interface) {
	case BMC_IPMB:
	case MCTP:
	case PLDM:
		target = I2C_BUS_BMC;
		break;
	default:
		break;
	}

	return target;
}

mctp *pal_get_mctp(uint8_t medium_type, uint8_t bus)
{
	switch (medium_type) {
	case MCTP_MEDIUM_TYPE_SMBUS:
		return find_mctp_by_smbus(bus);
	default:
		return NULL;
	}
}

uint8_t plat_eid = MCTP_DEFAULT_ENDPOINT;
void plat_set_eid_by_slot()
{
	uint8_t slot_eid = get_slot_eid();
	plat_eid = slot_eid;
}

uint8_t plat_get_eid()
{
	return plat_eid;
}

void plat_mctp_init(void)
{
	int ret = 0;
	plat_set_eid_by_slot();
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
