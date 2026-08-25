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
 * Minimal MCTP bring-up for this board: one local endpoint on
 * flexcomm3_lpi2c3, responding to MCTP Control and PLDM messages
 * addressed to it. Unlike a real product board (see
 * meta-facebook/gt-cc/src/platform/plat_mctp.c for what that looks
 * like with real downstream NICs to route to and detect), this EVK has
 * nothing else on its MCTP bus - so there's no route table, no
 * endpoint presence detection, and no outbound request-sending logic
 * here yet, just enough to stand up the local responder side.
 */

#include "mctp.h"
#include "mctp_ctrl.h"
#include "pldm.h"
#include "hal_i2c.h"
#include "plat_i2c.h"
#include "plat_mctp.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(plat_mctp);

/* mctp.c/mctp_ctrl.c call plat_get_eid()/plat_get_mctp_port_count()/
 * plat_get_mctp_port() as __weak-overridable board hooks - without
 * these, MCTP Control's Set Endpoint ID always fails (needs the port
 * table to know which mctp_port to update) and Get Endpoint ID always
 * reports a fixed default instead of whatever was actually assigned.
 */
static mctp_port bic_mctp_port;
static bool bic_mctp_port_ready;

uint8_t plat_get_eid(void)
{
	/* mctp_init() calls this before bic_mctp_port.mctp_inst exists, to
	 * pick the instance's starting EID - fall back to the static
	 * default then. Once up, report the live (possibly since
	 * reassigned by Set Endpoint ID) value instead. */
	if (!bic_mctp_port_ready) {
		return PLAT_MCTP_EID;
	}
	return bic_mctp_port.mctp_inst->endpoint;
}

uint8_t plat_get_mctp_port_count(void)
{
	return bic_mctp_port_ready ? 1 : 0;
}

mctp_port *plat_get_mctp_port(uint8_t index)
{
	if (!bic_mctp_port_ready || index != 0) {
		return NULL;
	}
	return &bic_mctp_port;
}

/* mctp_ctrl.c's Get Message Type Support handler needs this board-level
 * hook (__weak in mctp_ctrl.c, defaults to returning -1 - an honest
 * "not implemented" completion code, not a crash, but not a real answer
 * either). Both types this board actually handles in
 * plat_mctp_msg_recv() below are genuinely supported. */
int load_mctp_support_types(uint8_t *type_len, uint8_t *types)
{
	if (!type_len || !types) {
		return -1;
	}

	types[0] = TYPE_MCTP_CONTROL;
	types[1] = TYPE_PLDM;
	*type_len = 2;
	return 0;
}

static uint8_t plat_mctp_resolve_endpoint(uint8_t dest_endpoint, void **mctp_inst,
					   mctp_ext_params *ext_params)
{
	ARG_UNUSED(dest_endpoint);
	ARG_UNUSED(mctp_inst);
	ARG_UNUSED(ext_params);

	/* Nothing downstream to route an outbound message to yet - this
	 * board only responds to requests addressed to it, doesn't
	 * originate its own MCTP requests to other endpoints. */
	return MCTP_ERROR;
}

static uint8_t plat_mctp_msg_recv(void *mctp_p, uint8_t *buf, uint32_t len,
				   mctp_ext_params ext_params)
{
	if (!mctp_p || !buf || !len) {
		return MCTP_ERROR;
	}

	uint8_t msg_type = (buf[0] & MCTP_MSG_TYPE_MASK) >> MCTP_MSG_TYPE_SHIFT;

	switch (msg_type) {
	case MCTP_MSG_TYPE_CTRL:
		mctp_ctrl_cmd_handler(mctp_p, buf, len, ext_params);
		break;
	case MCTP_MSG_TYPE_PLDM:
		mctp_pldm_cmd_handler(mctp_p, buf, len, ext_params);
		break;
	default:
		LOG_WRN("unhandled mctp msg_type 0x%x", msg_type);
		return MCTP_ERROR;
	}

	return MCTP_SUCCESS;
}

void plat_mctp_init(void)
{
	int ret = mctp_i2c_target_register(I2C_BUS_ARDUINO_HEADER, PLAT_MCTP_I2C_TARGET_ADDR);

	if (ret) {
		LOG_ERR("mctp_i2c_target_register failed, ret %d", ret);
		return;
	}

	mctp *mctp_inst = mctp_init();

	if (!mctp_inst) {
		LOG_ERR("mctp_init failed");
		return;
	}

	mctp_medium_conf conf = {
		.smbus_conf = {
			.bus = I2C_BUS_ARDUINO_HEADER,
			.addr = PLAT_MCTP_I2C_TARGET_ADDR << 1,
		},
	};

	if (mctp_set_medium_configure(mctp_inst, MCTP_MEDIUM_TYPE_SMBUS, conf) != MCTP_SUCCESS) {
		LOG_ERR("mctp_set_medium_configure failed");
		return;
	}

	mctp_reg_endpoint_resolve_func(mctp_inst, plat_mctp_resolve_endpoint);
	mctp_reg_msg_rx_func(mctp_inst, plat_mctp_msg_recv);

	if (mctp_start(mctp_inst) != MCTP_SUCCESS) {
		LOG_ERR("mctp_start failed");
		return;
	}

	bic_mctp_port.mctp_inst = mctp_inst;
	bic_mctp_port.medium_type = MCTP_MEDIUM_TYPE_SMBUS;
	bic_mctp_port.conf = conf;
	/* Publish the port table (plat_get_mctp_port_count()/port() above)
	 * only now that mctp_inst is fully live - Set Endpoint ID can
	 * reach it from this point on. */
	bic_mctp_port_ready = true;

	LOG_INF("MCTP endpoint up on flexcomm3_lpi2c3, addr 0x%x, eid 0x%x",
		PLAT_MCTP_I2C_TARGET_ADDR, mctp_inst->endpoint);
}
