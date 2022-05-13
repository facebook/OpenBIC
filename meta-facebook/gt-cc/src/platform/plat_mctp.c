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
#include "plat_mctp.h"
#include "power_status.h"

LOG_MODULE_REGISTER(plat_mctp);

#define MCTP_MSG_TYPE_SHIFT 0
#define MCTP_MSG_TYPE_MASK 0x7F
#define MCTP_IC_SHIFT 7
#define MCTP_IC_MASK 0x80

/* i2c 8 bit address */
#define I2C_ADDR_BIC 0x40
#define I2C_ADDR_BMC 0x20
#define I2C_ADDR_NIC 0x64

/* i2c dev bus */
#define I2C_BUS_BMC 0x06
#define I2C_BUS_NIC_0 0x00
#define I2C_BUS_NIC_1 0x01
#define I2C_BUS_NIC_2 0x02
#define I2C_BUS_NIC_3 0x03
#define I2C_BUS_NIC_4 0x0A
#define I2C_BUS_NIC_5 0x0B
#define I2C_BUS_NIC_6 0x0C
#define I2C_BUS_NIC_7 0x0D
/* mctp endpoint */
#define MCTP_EID_BMC 0x08
#define MCTP_EID_NIC_0 0x10
#define MCTP_EID_NIC_1 0x11
#define MCTP_EID_NIC_2 0x12
#define MCTP_EID_NIC_3 0x13
#define MCTP_EID_NIC_4 0x14
#define MCTP_EID_NIC_5 0x15
#define MCTP_EID_NIC_6 0x16
#define MCTP_EID_NIC_7 0x17

K_TIMER_DEFINE(send_cmd_timer, send_cmd_to_dev, NULL);
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
} mctp_route_entry;

typedef struct _mctp_msg_handler {
	MCTP_MSG_TYPE type;
	mctp_fn_cb msg_handler_cb;
} mctp_msg_handler;

static mctp_smbus_port smbus_port[] = {
	{ .conf.smbus_conf.addr = I2C_ADDR_BIC, .conf.smbus_conf.bus = I2C_BUS_BMC },
	{ .conf.smbus_conf.addr = I2C_ADDR_BIC, .conf.smbus_conf.bus = I2C_BUS_NIC_0 },
	{ .conf.smbus_conf.addr = I2C_ADDR_BIC, .conf.smbus_conf.bus = I2C_BUS_NIC_1 },
	{ .conf.smbus_conf.addr = I2C_ADDR_BIC, .conf.smbus_conf.bus = I2C_BUS_NIC_2 },
	{ .conf.smbus_conf.addr = I2C_ADDR_BIC, .conf.smbus_conf.bus = I2C_BUS_NIC_3 },
	{ .conf.smbus_conf.addr = I2C_ADDR_BIC, .conf.smbus_conf.bus = I2C_BUS_NIC_4 },
	{ .conf.smbus_conf.addr = I2C_ADDR_BIC, .conf.smbus_conf.bus = I2C_BUS_NIC_5 },
	{ .conf.smbus_conf.addr = I2C_ADDR_BIC, .conf.smbus_conf.bus = I2C_BUS_NIC_6 },
	{ .conf.smbus_conf.addr = I2C_ADDR_BIC, .conf.smbus_conf.bus = I2C_BUS_NIC_7 },
};

mctp_route_entry mctp_route_tbl[] = {
	{ MCTP_EID_BMC, I2C_BUS_BMC, I2C_ADDR_BMC },
	{ MCTP_EID_NIC_0, I2C_BUS_NIC_0, I2C_ADDR_NIC },
	{ MCTP_EID_NIC_1, I2C_BUS_NIC_1, I2C_ADDR_NIC },
	{ MCTP_EID_NIC_2, I2C_BUS_NIC_2, I2C_ADDR_NIC },
	{ MCTP_EID_NIC_3, I2C_BUS_NIC_3, I2C_ADDR_NIC },
	{ MCTP_EID_NIC_4, I2C_BUS_NIC_4, I2C_ADDR_NIC },
	{ MCTP_EID_NIC_5, I2C_BUS_NIC_5, I2C_ADDR_NIC },
	{ MCTP_EID_NIC_6, I2C_BUS_NIC_6, I2C_ADDR_NIC },
	{ MCTP_EID_NIC_7, I2C_BUS_NIC_7, I2C_ADDR_NIC },
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

static void set_endpoint_resp_handler(void *args, uint8_t *buf, uint16_t len)
{
	if (!buf || !len)
		return;
	LOG_HEXDUMP_WRN(buf, len, __func__);
}

static void set_endpoint_resp_timeout(void *args)
{
	mctp_route_entry *p = (mctp_route_entry *)args;
	printk("[%s] Endpoint 0x%x set endpoint failed on bus %d \n", __func__, p->endpoint,
	       p->bus);
}

static void set_dev_endpoint(void)
{
	for (uint8_t i = 0; i < ARRAY_SIZE(mctp_route_tbl); i++) {
		mctp_route_entry *p = mctp_route_tbl + i;

		/* skip BMC */
		if (p->bus == I2C_BUS_BMC && p->addr == I2C_ADDR_BMC)
			continue;

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
}

static void get_dev_firmware_resp_timeout(void *args)
{
	mctp_route_entry *p = (mctp_route_entry *)args;
	printk("[%s] Endpoint 0x%x get parameter failed on bus %d \n", __func__, p->endpoint,
	       p->bus);
}

struct pldm_variable_field nic_vesion[8];

static void get_dev_firmware_resp_handler(void *args, uint8_t *buf, uint16_t len)
{
	if (!buf || !len)
		return;

	mctp_route_entry *p = (mctp_route_entry *)args;
	struct pldm_get_firmware_parameters_resp *response =
		(struct pldm_get_firmware_parameters_resp *)buf;

	uint8_t nic_index = p->endpoint - MCTP_EID_NIC_0;

	nic_vesion[nic_index].ptr =
		(uint8_t *)malloc(sizeof(uint8_t) * response->active_comp_image_set_ver_str_len);
	nic_vesion[nic_index].length = response->active_comp_image_set_ver_str_len;
	memcpy(nic_vesion[nic_index].ptr, buf + sizeof(struct pldm_get_firmware_parameters_resp),
	       nic_vesion[nic_index].length);
}

static void get_dev_firmware_parameters(void)
{
	for (uint8_t i = 0; i < ARRAY_SIZE(mctp_route_tbl); i++) {
		mctp_route_entry *p = mctp_route_tbl + i;

		if (p->addr != I2C_ADDR_NIC)
			continue;

		for (uint8_t j = 0; j < ARRAY_SIZE(smbus_port); j++) {
			if (p->bus != smbus_port[j].conf.smbus_conf.bus)
				continue;
		}
		pldm_msg msg = { 0 };

		msg.ext_params.type = MCTP_MEDIUM_TYPE_SMBUS;
		msg.ext_params.smbus_ext_params.addr = p->addr;

		msg.hdr.pldm_type = PLDM_TYPE_FW_UPDATE;
		msg.hdr.cmd = 0x02;
		msg.hdr.rq = 1;
		msg.len = 0;

		msg.recv_resp_cb_fn = get_dev_firmware_resp_handler;
		msg.recv_resp_cb_args = p;
		msg.timeout_cb_fn = get_dev_firmware_resp_timeout;
		msg.timeout_cb_fn_args = p;

		mctp_pldm_send_msg(find_mctp_by_smbus(p->bus), &msg);
	}
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
	if (!mctp_inst || !ext_params)
		return MCTP_ERROR;

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
	/* get device parameters */
	get_dev_firmware_parameters();
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

	/* Only send command to device when DC on */
	if (get_DC_status())
		k_timer_start(&send_cmd_timer, K_MSEC(3000), K_NO_WAIT);
}