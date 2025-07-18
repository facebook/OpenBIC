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
#include "plat_i2c.h"
#include "plat_hook.h"
#include "plat_mctp.h"
#include "plat_gpio.h"
#include "hal_i3c.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_mctp);

/* i2c 8 bit address */
#define I2C_ADDR_BIC 0x40
#define I2C_ADDR_BMC 0x20

/* i2c dev bus */
#define I2C_BUS_BMC I2C_BUS6

/* i3c dev bus */
#define I3C_BUS_BMC I2C_BUS6
/* i3c vendor-def-id */
#define DEFAULT_VENDOR_DEF_ID 0x567

/* mctp endpoint */
#define MCTP_EID_BMC 0x08

uint8_t plat_eid = MCTP_DEFAULT_ENDPOINT;

/*
static mctp_port i3c_port[] = {
	{
		.conf.i3c_conf.addr = I2C_ADDR_BMC,
		.conf.i3c_conf.bus = I3C_BUS_BMC,
	},
};*/

static mctp_port smbus_port[] = {
	{ .conf.smbus_conf.addr = I2C_ADDR_BIC, .conf.smbus_conf.bus = I2C_BUS_BMC },
};

mctp_port plat_mctp_port[] = {
	{ .channel_target = PLDM,
	  .medium_type = MCTP_MEDIUM_TYPE_TARGET_I3C,
	  .conf.i3c_conf.bus = I3C_BUS_BMC,
	  .conf.i3c_conf.addr = I2C_ADDR_BMC },
};

mctp_route_entry mctp_route_tbl[] = {
	{ MCTP_EID_BMC, I3C_BUS_BMC, I2C_ADDR_BMC },
};

uint8_t MCTP_SUPPORTED_MESSAGES_TYPES[] = {
	TYPE_MCTP_CONTROL,
	TYPE_PLDM,
};

static mctp *find_mctp_by_bus(uint8_t bus)
{
	uint8_t i;
	for (i = 0; i < ARRAY_SIZE(plat_mctp_port); i++) {
		mctp_port *p = plat_mctp_port + i;

		if (bus == p->conf.i3c_conf.bus)
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
		const mctp_route_entry *p = mctp_route_tbl + i;
		if (!p) {
			return MCTP_ERROR;
		}
		if (p->endpoint == dest_endpoint) {
			*mctp_inst = find_mctp_by_bus(p->bus);
			ext_params->type = MCTP_MEDIUM_TYPE_TARGET_I3C;
			ext_params->smbus_ext_params.addr = p->addr;
			rc = MCTP_SUCCESS;
			break;
		}
	}
	return rc;
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

void plat_mctp_init(void)
{
	/* init the mctp/pldm instance */
	LOG_INF("plat_mctp_i3c_init");

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

		uint8_t rc;
		if (p->conf.i3c_conf.bus == I3C_BUS_BMC) {
			// Slave (i3c5)More actions
			rc = mctp_set_medium_configure(p->mctp_inst, MCTP_MEDIUM_TYPE_SMBUS,
						       p->conf);
			LOG_DBG("MCTP medium type for slave: %s",
				(rc == MCTP_SUCCESS) ? "success" : "failed");
		} else {
			LOG_ERR("Unknown I3C Bus number!");
			continue;
		}

		// mctp_reg_endpoint_resolve_func(p->mctp_inst, get_mctp_route_info);
		mctp_reg_msg_rx_func(p->mctp_inst, mctp_msg_recv);
		mctp_start(p->mctp_inst);
	}
}

int load_mctp_support_types(uint8_t *type_len, uint8_t *types)
{
	*type_len = sizeof(MCTP_SUPPORTED_MESSAGES_TYPES);
	memcpy(types, MCTP_SUPPORTED_MESSAGES_TYPES, sizeof(MCTP_SUPPORTED_MESSAGES_TYPES));
	return MCTP_SUCCESS;
}

void plat_i3c_set_pid(void)
{
	I3C_MSG i3c_msg;
	i3c_msg.bus = I3C_BUS_BMC;
	i3c_set_pid(&i3c_msg, DEFAULT_VENDOR_DEF_ID);
}

uint8_t plat_get_eid()
{
	// mmc slot 1-4 * 0x0A
	return ((get_mmc_slot() + 1) * MCTP_DEFAULT_ENDPOINT);
}