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
 * flexcomm2_lpi2c2 - the same physical bus that carries IPMB, as a
 * second I2C target address (0x10 vs IPMB's 0x20) - responding to
 * MCTP Control and PLDM messages
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
#include "plat_hwinfo.h"
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
	types[2] = MCTP_MSG_TYPE_VEN_DEF_PCI;
	*type_len = 3;
	return 0;
}

/* MCTP Control's Get Endpoint UUID hook - a real, genuinely unique
 * per-chip hardware ID (read once at boot by plat_hwinfo_init(), the
 * same value logged as this board's "device id"), not a fabricated
 * placeholder. */
bool plat_get_endpoint_uuid(uint8_t *uuid_buf)
{
	return plat_get_device_id(uuid_buf, MCTP_UUID_LEN);
}

/* MCTP Control's Get Vendor Defined Message Support hook. This board
 * advertises MCTP message type 0x7E (Vendor Defined - PCI) in
 * load_mctp_support_types(); the one vendor ID behind it is the test
 * VDM echo command (see plat_mctp_vdm_handler / plat_mctp.h). Exactly
 * one entry, selector 0. */
int plat_mctp_get_vdm_support(uint8_t selector, uint8_t *vendor_id_format, uint16_t *pci_vendor_id,
			     uint16_t *cmd_set_version, uint8_t *next_selector)
{
	if (selector != 0) {
		return -1;
	}

	*vendor_id_format = MCTP_VENDOR_ID_FORMAT_PCI;
	*pci_vendor_id = PLAT_MCTP_TEST_VENDOR_ID;
	*cmd_set_version = 0;
	*next_selector = MCTP_VENDOR_ID_SELECTOR_NONE;
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

/* Test-only VDM echo handler - see plat_mctp.h for the wire format.
 * Not a real vendor protocol: exists purely so a peer can request an
 * N-byte reply and confirm our outbound multi-packet fragmentation
 * (mctp_tx_task()'s chunking at MCTP_DEFAULT_MSG_MAX_SIZE) works,
 * mirroring the inbound direction already exercised by real commands
 * like Get Endpoint ID. */
static void plat_mctp_vdm_handler(void *mctp_p, uint8_t *buf, uint32_t len,
				   mctp_ext_params ext_params)
{
	mctp *mctp_inst = (mctp *)mctp_p;

	if (len < 6) {
		LOG_WRN("vdm test msg too short (%d)", len);
		return;
	}

	uint16_t vendor_id = ((uint16_t)buf[1] << 8) | buf[2];
	uint8_t cmd = buf[3];

	if (vendor_id != PLAT_MCTP_TEST_VENDOR_ID || cmd != PLAT_MCTP_TEST_CMD_ECHO) {
		LOG_WRN("unhandled vdm test vendor_id 0x%x cmd 0x%x", vendor_id, cmd);
		return;
	}

	uint16_t req_len = ((uint16_t)buf[4] << 8) | buf[5];
	uint8_t status = PLAT_MCTP_TEST_STATUS_SUCCESS;

	if (req_len > PLAT_MCTP_TEST_MAX_ECHO_LEN) {
		req_len = PLAT_MCTP_TEST_MAX_ECHO_LEN;
		status = PLAT_MCTP_TEST_STATUS_ERROR;
	}

	uint8_t resp_buf[5 + PLAT_MCTP_TEST_MAX_ECHO_LEN];
	resp_buf[0] = buf[0];
	resp_buf[1] = buf[1];
	resp_buf[2] = buf[2];
	resp_buf[3] = cmd;
	resp_buf[4] = status;
	for (uint16_t i = 0; i < req_len; i++) {
		resp_buf[5 + i] = (uint8_t)(i & 0xFF);
	}

	if (mctp_send_msg(mctp_inst, resp_buf, 5 + req_len, ext_params) != MCTP_SUCCESS) {
		LOG_ERR("vdm test echo reply send failed");
	}
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
	case MCTP_MSG_TYPE_VEN_DEF_PCI:
		plat_mctp_vdm_handler(mctp_p, buf, len, ext_params);
		break;
	default:
		LOG_WRN("unhandled mctp msg_type 0x%x", msg_type);
		return MCTP_ERROR;
	}

	return MCTP_SUCCESS;
}

void plat_mctp_init(void)
{
	int ret = mctp_i2c_target_register(I2C_BUS_SIDEBAND, PLAT_MCTP_I2C_TARGET_ADDR);

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
			.bus = I2C_BUS_SIDEBAND,
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

	LOG_INF("MCTP endpoint up on flexcomm2_lpi2c2 (shared with IPMB), addr 0x%x, eid 0x%x",
		PLAT_MCTP_I2C_TARGET_ADDR, mctp_inst->endpoint);
}
