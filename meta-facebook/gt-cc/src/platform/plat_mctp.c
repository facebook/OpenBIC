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
#include "ncsi.h"
#include "ipmi.h"
#include "sensor.h"
#include "plat_hook.h"
#include "plat_mctp.h"
#include "plat_gpio.h"
#include "plat_ncsi.h"
#include "plat_fru.h"
#include "plat_sensor_table.h"

LOG_MODULE_REGISTER(plat_mctp);

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

#define NVIDIA_NIC_MANUFACTURER "Nvidia"
#define BROADCOM_NIC_MANUFACTURER "Broadcom"
#define AMD_NIC_MANUFACTURER "AMD"

static uint8_t nic_config = NIC_CONFIG_UNKNOWN;
static bool is_nic_config_set = false;

K_TIMER_DEFINE(send_cmd_timer, send_cmd_to_dev, NULL);
K_WORK_DEFINE(send_cmd_work, send_cmd_to_dev_handler);

static mctp_port smbus_port[] = {
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
	{ MCTP_EID_NIC_0, I2C_BUS_NIC_0, I2C_ADDR_NIC, PRSNT_NIC0_R_N },
	{ MCTP_EID_NIC_1, I2C_BUS_NIC_1, I2C_ADDR_NIC, PRSNT_NIC1_R_N },
	{ MCTP_EID_NIC_2, I2C_BUS_NIC_2, I2C_ADDR_NIC, PRSNT_NIC2_R_N },
	{ MCTP_EID_NIC_3, I2C_BUS_NIC_3, I2C_ADDR_NIC, PRSNT_NIC3_R_N },
	{ MCTP_EID_NIC_4, I2C_BUS_NIC_4, I2C_ADDR_NIC, PRSNT_NIC4_R_N },
	{ MCTP_EID_NIC_5, I2C_BUS_NIC_5, I2C_ADDR_NIC, PRSNT_NIC5_R_N },
	{ MCTP_EID_NIC_6, I2C_BUS_NIC_6, I2C_ADDR_NIC, PRSNT_NIC6_R_N },
	{ MCTP_EID_NIC_7, I2C_BUS_NIC_7, I2C_ADDR_NIC, PRSNT_NIC7_R_N },
};

static mctp *find_mctp_by_smbus(uint8_t bus)
{
	uint8_t i;
	for (i = 0; i < ARRAY_SIZE(smbus_port); i++) {
		mctp_port *p = smbus_port + i;

		if (bus == p->conf.smbus_conf.bus)
			return p->mctp_inst;
	}

	return NULL;
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
			ext_params->ep = p->endpoint;
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

		if (gpio_get(p->dev_present_pin))
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

void plat_set_dev_endpoint(void)
{
	set_dev_endpoint();
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
	CHECK_NULL_ARG(args);
	CHECK_NULL_ARG(buf);

	if (!len)
		return;

	mctp_route_entry *p = (mctp_route_entry *)args;
	struct pldm_get_firmware_parameters_resp *response =
		(struct pldm_get_firmware_parameters_resp *)buf;

	uint8_t nic_index = p->endpoint - MCTP_EID_NIC_0;

	SAFE_FREE(nic_vesion[nic_index].ptr);
	nic_vesion[nic_index].ptr =
		(uint8_t *)malloc(sizeof(uint8_t) * response->active_comp_image_set_ver_str_len);

	if (!nic_vesion[nic_index].ptr) {
		LOG_ERR("The buffer of NIC%d version memory allocate failed", nic_index);
		return;
	}

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

		if (gpio_get(p->dev_present_pin))
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

	req.header.netfn_lun = NETFN_STORAGE_REQ;
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
		LOG_ERR("Check reponse completion code fail");
		return false;
	}

	return true;
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

	case MCTP_MSG_TYPE_NCSI:
		mctp_ncsi_cmd_handler(mctp_p, buf, len, ext_params);
		break;

	default:
		LOG_WRN("Cannot find message receive function!! msg_type = %d", msg_type);
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
			if ((p->addr == I2C_ADDR_NIC) && gpio_get(p->dev_present_pin))
				return MCTP_ERROR;
			*mctp_inst = find_mctp_by_smbus(p->bus);
			ext_params->type = MCTP_MEDIUM_TYPE_SMBUS;
			ext_params->smbus_ext_params.addr = p->addr;
			rc = MCTP_SUCCESS;
			break;
		}
	}

	return rc;
}

static void mellanox_cx7_ncsi_init(void)
{
	for (uint8_t i = 0; i < ARRAY_SIZE(mctp_route_tbl); i++) {
		mctp_route_entry *p = mctp_route_tbl + i;

		/* skip BMC */
		if (p->bus == I2C_BUS_BMC && p->addr == I2C_ADDR_BMC)
			continue;

		if (gpio_get(p->dev_present_pin))
			continue;

		if (mellanox_cx7_set_self_recovery_setting(p->endpoint) == true) {
			LOG_INF("Set mellanox cx7 self recovery setting success eid = 0x%x",
				p->endpoint);
		} else {
			LOG_ERR("Set mellanox cx7 self recovery setting fail eid = 0x%x",
				p->endpoint);
		}
	}
}

uint8_t mellanox_cx7_ncsi_get_link_type(void)
{
	uint8_t config = NIC_CONFIG_UNKNOWN;

	for (uint8_t i = 0; i < ARRAY_SIZE(mctp_route_tbl); i++) {
		mctp_route_entry *p = mctp_route_tbl + i;

		/* skip BMC */
		if (p->bus == I2C_BUS_BMC && p->addr == I2C_ADDR_BMC)
			continue;

		if (gpio_get(p->dev_present_pin))
			continue;

		if (mellanox_cx7_clear_initial_state(p->endpoint) == true) {
			LOG_INF("Clear initial state success eid = 0x%x", p->endpoint);
		} else {
			LOG_ERR("Clear initial state fail eid = 0x%x", p->endpoint);
		}

		uint8_t link_type = NCSI_IB_LINK_TYPE_UNKNOWN;
		uint16_t response_code = 0xffff;
		uint16_t reason_code = 0xffff;
		if (mellanox_cx7_get_infiniband_link_status(p->endpoint, &link_type, &response_code,
							    &reason_code) == true) {
			LOG_INF("get link status success eid = 0x%x , link_type = 0x%x, response_code = 0x%x, reason_code = 0x%x",
				p->endpoint, link_type, response_code, reason_code);
		} else {
			LOG_ERR("get mellanox cx7 infiniband link status fail eid = 0x%x",
				p->endpoint);
		}

		/* Only IB_CX7 NIC supports the "Get InfiniBand Link Status" command.
		 * Set NIC configuration to IB_CX7 if there is any InfiniBand NIC.
		 */
		if ((link_type == NCSI_IB_LINK_TYPE_INFINIBAND) ||
		    (link_type == NCSI_IB_LINK_TYPE_ETHERNET)) {
			config = NIC_CONFIG_IB_CX7;
			LOG_INF("eid 0x%x NIC is IB_CX7 NIC", p->endpoint);
		}

		/* CX7 NIC does not support the "Get InfiniBand Link Status" command */
		if (response_code == NCSI_COMMAND_UNSUPPORTED) {
			LOG_INF("eid 0x%x NIC is CX7 NIC", p->endpoint);
			if (config != NIC_CONFIG_IB_CX7) {
				config = NIC_CONFIG_CX7;
			}
		}
	}

	return config;
}

bool is_broadcom_thor2_nic_type()
{
	// check nic_type by accessing sensor

	bool is_broadcom_thor2_nic = false;

	for (uint8_t i = 0; i < ARRAY_SIZE(mctp_route_tbl); i++) {
		mctp_route_entry *p = mctp_route_tbl + i;

		/* skip BMC */
		if (p->bus == I2C_BUS_BMC && p->addr == I2C_ADDR_BMC)
			continue;

		if (gpio_get(p->dev_present_pin))
			continue;

		uint8_t mctp_dest_eid = p->endpoint;

		mctp *mctp_inst = NULL;
		mctp_ext_params ext_params = { 0 };
		if (get_mctp_info_by_eid(mctp_dest_eid, &mctp_inst, &ext_params) == false) {
			LOG_ERR("Failed to get mctp info by eid 0x%x", mctp_dest_eid);
			continue;
		}

		uint8_t resp_buf[10] = { 0 };
		uint8_t req_len = sizeof(struct pldm_get_sensor_reading_req);
		struct pldm_get_sensor_reading_req req = { 0 };
		req.sensor_id = 0x01F4;
		req.rearm_event_state = 0;

		uint16_t resp_len = pldm_platform_monitor_read(
			mctp_inst, ext_params, PLDM_MONITOR_CMD_CODE_GET_SENSOR_READING,
			(uint8_t *)&req, req_len, resp_buf, sizeof(resp_buf));

		if (resp_len == 0) {
			LOG_ERR("Failed to get sensor #%d reading", req.sensor_id);
			continue;
		}

		struct pldm_get_sensor_reading_resp *res =
			(struct pldm_get_sensor_reading_resp *)resp_buf;

		if ((res->completion_code != PLDM_SUCCESS)) {
			LOG_ERR("Failed to get get sensor reading, completion_code = 0x%x",
				res->completion_code);
			continue;
		}

		if (res->sensor_operational_state == PLDM_SENSOR_ENABLED) {
			is_broadcom_thor2_nic = true;
		} else {
			LOG_ERR("CX7 Failed to get get sensor reading, sensor operational state=0x%x",
				res->sensor_operational_state);
			continue;
		}
	}

	return is_broadcom_thor2_nic;
}

void check_nic_config(void)
{
	char nic_manufacturer[32];

	bool found = get_first_nic_manufacturer(nic_manufacturer, sizeof(nic_manufacturer));

	if (found) {
		LOG_INF("First NIC manufacturer found: %s", nic_manufacturer);
	} else {
		LOG_INF("No NIC manufacturer found");
	}

	uint8_t config = NIC_CONFIG_UNKNOWN;
	if (strncmp(nic_manufacturer, NVIDIA_NIC_MANUFACTURER,
		    sizeof(NVIDIA_NIC_MANUFACTURER) - 1) == 0) {
		LOG_INF("NVIDIA NIC detected");
		config = mellanox_cx7_ncsi_get_link_type();
	} else if (strncmp(nic_manufacturer, BROADCOM_NIC_MANUFACTURER,
			   sizeof(BROADCOM_NIC_MANUFACTURER) - 1) == 0) {
		LOG_INF("Broadcom NIC detected");
		set_cx7_init_arg_to_thor2();
		config = NIC_CONFIG_THOR2;
	} else if (strncmp(nic_manufacturer, AMD_NIC_MANUFACTURER,
			   sizeof(AMD_NIC_MANUFACTURER) - 1) == 0) {
		LOG_INF("AMD NIC detected");
		config = NIC_CONFIG_POLLARA;
	} else {
		LOG_INF("Unknown NIC manufacturer: %s", nic_manufacturer);
	}

	nic_config = config;
	LOG_INF("NIC config is %d, 0: UNKNOWN, 1: CX7, 2: IB_CX7, 3: THOR2, 4: POLLARA",
		nic_config);

	is_nic_config_set = true;
}

uint8_t get_nic_config(void)
{
	return nic_config;
}

bool get_is_nic_config_set(void)
{
	return is_nic_config_set;
}

void send_cmd_to_dev_handler(struct k_work *work)
{
	/* init the device endpoint */
	set_dev_endpoint();

	/* check nic config by ncsi */
	check_nic_config();
	// Update NIC sensor configurations if POLLARA is detected
	update_nic_sensor_config_for_pollara();
	nic_drive_reinit_for_pollara();
	nic_optics_drive_reinit_for_pollara();

	/* init mellanox cx7 ncsi */
	if ((nic_config == NIC_CONFIG_IB_CX7) || (nic_config == NIC_CONFIG_CX7))
		mellanox_cx7_ncsi_init();

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
		mctp_port *p = smbus_port + i;
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

	/* The NIC can be ready for access within 1050ms. Setting the timer to 3000ms to ensure reliability. */
	k_timer_start(&send_cmd_timer, K_MSEC(3000), K_NO_WAIT);
}
