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

#include <stdio.h>
#include <stdlib.h>
#include <logging/log.h>
#include "ipmi.h"
#include "plat_ipmi.h"
#include "plat_ipmb.h"
#include "plat_class.h"
#include "libutil.h"
#include "plat_fru.h"
#include "util_spi.h"
#include "plat_sensor_table.h"
#include "plat_power_status.h"
#include "ipmb.h"
#include "mctp.h"
#include "pldm.h"
#include "plat_mctp.h"
#include "sensor.h"
#include "pmbus.h"

LOG_MODULE_REGISTER(plat_ipmi);

enum HDA1_FIRMWARE_COMPONENT {
	HDA1_COMPNT_CPLD = 1,
	HDA1_COMPNT_BIC,
	HDA1_COMPNT_BIOS,
	HDA1_COMPNT_BSD = 63,
};

bool pal_request_msg_to_BIC_from_HOST(uint8_t netfn, uint8_t cmd)
{
	if (netfn == NETFN_OEM_1S_REQ) {
		if ((cmd == CMD_OEM_1S_FW_UPDATE) || (cmd == CMD_OEM_1S_RESET_BMC) ||
		    (cmd == CMD_OEM_1S_GET_BIC_STATUS) || (cmd == CMD_OEM_1S_RESET_BIC) ||
		    (cmd == CMD_OEM_1S_GET_BIC_FW_INFO) ||
		    (cmd == CMD_OEM_1S_GET_DIMM_I3C_MUX_SELECTION))
			return true;
	}

	return false;
}

void OEM_1S_GET_CARD_TYPE(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg)

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	CARD_STATUS _1ou_status = get_1ou_status();
	CARD_STATUS _2ou_status = get_2ou_status();
	switch (msg->data[0]) {
	case GET_1OU_CARD_TYPE:
		msg->data_len = 2;
		msg->completion_code = CC_SUCCESS;
		msg->data[0] = GET_1OU_CARD_TYPE;
		if (_1ou_status.present) {
			msg->data[1] = _1ou_status.card_type;
		} else {
			msg->data[1] = TYPE_1OU_ABSENT;
		}
		break;
	case GET_2OU_CARD_TYPE:
		msg->data_len = 2;
		msg->completion_code = CC_SUCCESS;
		msg->data[0] = GET_2OU_CARD_TYPE;
		if (_2ou_status.present) {
			msg->data[1] = _2ou_status.card_type;
		} else {
			msg->data[1] = TYPE_2OU_ABSENT;
		}
		break;
	default:
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_DATA_FIELD;
		break;
	}

	return;
}

int pal_record_bios_fw_version(uint8_t *buf, uint8_t size)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, -1);

	int ret = -1;
	EEPROM_ENTRY set_bios_ver = { 0 };
	EEPROM_ENTRY get_bios_ver = { 0 };

	const uint8_t block_index = buf[3];
	if (block_index >= BIOS_FW_VERSION_BLOCK_NUM) {
		LOG_ERR("BIOS version block index is out of range");
		return -1;
	}

	ret = get_bios_version(&get_bios_ver, block_index);
	if (ret == -1) {
		LOG_ERR("Get BIOS version failed");
		return -1;
	}

	set_bios_ver.data_len = size - 3; // skip netfn, cmd and command code
	memcpy(&set_bios_ver.data[0], &buf[3], set_bios_ver.data_len);

	// Check the written BIOS version is the same with the stored
	ret = memcmp(&get_bios_ver.data[0], &set_bios_ver.data[0],
		     BIOS_FW_VERSION_BLOCK_MAX_SIZE * sizeof(uint8_t));
	if (ret == 0) {
		LOG_DBG("The Written BIOS version is the same as the stored BIOS version in EEPROM");
	} else {
		LOG_DBG("BIOS version set successfully");

		ret = set_bios_version(&set_bios_ver, block_index);
		if (ret == -1) {
			LOG_ERR("Set BIOS version failed");
			return -1;
		}
	}

	return 0;
}

void OEM_1S_GET_BIOS_VERSION(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	msg->data_len = 0;

	for (uint8_t block_index = 0; block_index < BIOS_FW_VERSION_BLOCK_NUM; block_index++) {
		EEPROM_ENTRY get_bios_ver = { 0 };
		int ret = get_bios_version(&get_bios_ver, block_index);
		if (ret == -1) {
			LOG_ERR("Get BIOS version failed");
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}

		memcpy(msg->data + msg->data_len, get_bios_ver.data, get_bios_ver.data_len);
		msg->data_len += get_bios_ver.data_len;
	}

	msg->completion_code = CC_SUCCESS;
	return;
}

void OEM_1S_GET_FW_VERSION(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t component;
	component = msg->data[0];

	switch (component) {
	case HDA1_COMPNT_BIC:
		msg->data[0] = BIC_FW_YEAR_MSB;
		msg->data[1] = BIC_FW_YEAR_LSB;
		msg->data[2] = BIC_FW_WEEK;
		msg->data[3] = BIC_FW_VER;
		msg->data[4] = BIC_FW_platform_0;
		msg->data[5] = BIC_FW_platform_1;
		msg->data[6] = BIC_FW_platform_2;
		msg->data_len = 7;
		msg->completion_code = CC_SUCCESS;
		break;

	case HDA1_COMPNT_BSD: {
		write_bsd_version();
		k_msleep(100);
		uint32_t bsd_version = read_bsd_version();
		if (bsd_version == 0) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			break;
		}

		msg->data_len = sizeof(bsd_version);
		for (int i = 0; i < msg->data_len; i++)
			msg->data[i] = (bsd_version >> (8 * ((msg->data_len - 1) - i))) & 0xFF;

		msg->completion_code = CC_SUCCESS;
		break;
	}

	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}
	return;
}

#ifdef CONFIG_I2C_IPMB_SLAVE

static void mpro_resp_handler(void *args, uint8_t *buf, uint16_t len)
{
	CHECK_NULL_ARG(args);
	CHECK_NULL_ARG(buf);

	LOG_HEXDUMP_DBG(buf, len, "mpro rsp:");

	ipmi_msg *msg = (ipmi_msg *)args;

	if (!len)
		return;

	msg->InF_source = BMC_IPMB;
	msg->netfn = NETFN_OEM_1S_REQ;
	msg->cmd = CMD_OEM_1S_MSG_OUT;
	msg->completion_code = CC_SUCCESS;
	msg->data_len = len + 3;
	msg->data[0] = IANA_ID & 0xFF;
	msg->data[1] = (IANA_ID >> 8) & 0xFF;
	msg->data[2] = (IANA_ID >> 16) & 0xFF;

	memcpy(&msg->data[3], buf, len);

	ipmb_error status = ipmb_send_response(msg, IPMB_inf_index_map[msg->InF_source]);
	if (status != IPMB_ERROR_SUCCESS) {
		LOG_ERR("OEM_MSG_OUT send IPMB resp fail status: %x", status);
	}
}

void OEM_1S_MSG_OUT(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);
#if MAX_IPMB_IDX
	uint8_t target_IF;
	ipmb_error status;
	ipmi_msg *bridge_msg = NULL;

	// If command is valid the default cc is CC_INVALID_CMD, set cc to CC_SUCCESS before we execute the command.
	// If the default cc is CC_INVALID_IANA, call ipmb_send_response for this invalid command.
	if (msg->completion_code != CC_INVALID_IANA) {
		msg->completion_code = CC_SUCCESS;
	}

	// Should input target, netfn, cmd
	if (msg->data_len <= 2) {
		msg->completion_code = CC_INVALID_LENGTH;
	}

	CARD_STATUS _1ou_status = get_1ou_status();
	target_IF = msg->data[0];

	switch (target_IF) {
	case PEER_BMC_IPMB:
		switch (msg->InF_source) {
		case SLOT1_BIC:
			target_IF = msg->data[0] = SLOT3_BIC;
			break;
		case SLOT3_BIC:
			target_IF = msg->data[0] = SLOT1_BIC;
			break;
		default:
			msg->completion_code = CC_INVALID_DATA_FIELD;
			break;
		}

		if (msg->data[1] == NETFN_STORAGE_REQ) {
			msg->data[1] = msg->data[1] << 2;
		}
		break;
	case EXP2_IPMB:
		if (_1ou_status.card_type == TYPE_1OU_EXP_WITH_E1S) {
			target_IF = EXP1_IPMB;
		}
		break;
	case EXP4_IPMB:
		target_IF = EXP3_IPMB;
		break;
	case MPRO_PLDM:
		if (msg->data_len < 3) {
			msg->completion_code = CC_INVALID_LENGTH;
			goto error;
		}

		if (get_mpro_status() == false) {
			msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
			goto error;
		}

		pldm_msg pmsg = { 0 };
		pmsg.hdr.msg_type = MCTP_MSG_TYPE_PLDM;
		pmsg.hdr.rq = PLDM_REQUEST;
		pmsg.hdr.pldm_type = msg->data[1];
		pmsg.hdr.cmd = msg->data[2];
		pmsg.len = msg->data_len - 3;
		if (pmsg.len)
			pmsg.buf = &msg->data[3];

		LOG_DBG("*Bridge pldm command 0x%x 0x%x", pmsg.hdr.pldm_type, pmsg.hdr.cmd);
		LOG_HEXDUMP_DBG(pmsg.buf, pmsg.len, "mpro req:");

		ipmi_msg save = { 0 };
		memcpy(&save, msg, sizeof(ipmi_msg));

		pmsg.recv_resp_cb_fn = mpro_resp_handler;
		pmsg.recv_resp_cb_args = &save;
		pmsg.timeout_cb_fn = NULL;
		pmsg.timeout_cb_fn_args = NULL;

		mctp *mctp_inst = NULL;
		if (get_mctp_info_by_eid(MCTP_EID_MPRO, &mctp_inst, &pmsg.ext_params) == false) {
			LOG_ERR("Failed to get mctp info by eid 0x%x", MCTP_EID_MPRO);
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			goto error;
		}

		if (mctp_pldm_send_msg(mctp_inst, &pmsg) != PLDM_SUCCESS) {
			LOG_ERR("Failed to send mctp-pldm request command to mpro");
			msg->completion_code = CC_BRIDGE_MSG_ERR;
			goto error;
		}

		return;

	default:
		break;
	}

	// Bridge to invalid or disabled interface
	if ((IPMB_inf_index_map[target_IF] == RESERVED) ||
	    (IPMB_config_table[IPMB_inf_index_map[target_IF]].interface == RESERVED_IF) ||
	    (IPMB_config_table[IPMB_inf_index_map[target_IF]].enable_status == DISABLE)) {
		LOG_ERR("OEM_MSG_OUT: Invalid bridge interface: %x", target_IF);
		msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
	}

	// only send to target while msg is valid
	if (msg->completion_code == CC_SUCCESS) {
		bridge_msg = (ipmi_msg *)malloc(sizeof(ipmi_msg));
		if (bridge_msg == NULL) {
			msg->completion_code = CC_OUT_OF_SPACE;
			return;
		} else {
			memset(bridge_msg, 0, sizeof(ipmi_msg));

			LOG_DBG("bridge targetIf %x, len %d, netfn %x, cmd %x", target_IF,
				msg->data_len, msg->data[1] >> 2, msg->data[2]);

			if ((_1ou_status.card_type == TYPE_1OU_EXP_WITH_E1S) &&
			    ((msg->data[0] == EXP2_IPMB) || (msg->data[0] == EXP4_IPMB))) {
				bridge_msg->seq_source = msg->seq_source;
				bridge_msg->InF_target = msg->data[0];
				bridge_msg->InF_source = msg->InF_source;
				bridge_msg->netfn = NETFN_OEM_1S_REQ;
				bridge_msg->cmd = CMD_OEM_1S_MSG_OUT;
				bridge_msg->data[0] = IANA_ID & 0xFF;
				bridge_msg->data[1] = (IANA_ID >> 8) & 0xFF;
				bridge_msg->data[2] = (IANA_ID >> 16) & 0xFF;

				if (msg->data_len != 0) {
					memcpy(&bridge_msg->data[3], &msg->data[0],
					       msg->data_len * sizeof(msg->data[0]));
				}

				bridge_msg->data_len = msg->data_len + 3;
			} else {
				bridge_msg->data_len = msg->data_len - 3;
				bridge_msg->seq_source = msg->seq_source;
				bridge_msg->InF_target = msg->data[0];
				bridge_msg->InF_source = msg->InF_source;
				bridge_msg->netfn = msg->data[1] >> 2;
				bridge_msg->cmd = msg->data[2];

				if (bridge_msg->data_len != 0) {
					memcpy(&bridge_msg->data[0], &msg->data[3],
					       bridge_msg->data_len * sizeof(msg->data[0]));
				}
			}

			status = ipmb_send_request(bridge_msg, IPMB_inf_index_map[target_IF]);

			if (status != IPMB_ERROR_SUCCESS) {
				LOG_ERR("OEM_MSG_OUT send IPMB req fail status: %x", status);
				msg->completion_code = CC_BRIDGE_MSG_ERR;
			}
			SAFE_FREE(bridge_msg);
		}
	}

error:
	// Return to source while data is invalid or sending req to Tx task fail
	if (msg->completion_code != CC_SUCCESS) {
		msg->data_len = 0;
		status = ipmb_send_response(msg, IPMB_inf_index_map[msg->InF_source]);
		if (status != IPMB_ERROR_SUCCESS) {
			LOG_ERR("OEM_MSG_OUT send IPMB resp fail status: %x", status);
		}
	}
#else
	msg->completion_code = CC_UNSPECIFIED_ERROR;
#endif
	return;
}
#endif

void OEM_1S_GET_HSC_STATUS(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t hsc_type = get_hsc_module();
	I2C_MSG i2c_msg;
	uint8_t retry = 5;

	i2c_msg.bus = I2C_BUS2;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;

	switch (hsc_type) {
	case HSC_MODULE_ADM1278:
		i2c_msg.target_addr = ADM1278_ADDR;
		i2c_msg.data[0] = PMBUS_STATUS_BYTE;
		break;
	case HSC_MODULE_LTC4282:
		i2c_msg.target_addr = LTC4282_ADDR;
		i2c_msg.data[0] = LTC4282_STATUS_OFFSET_BYTE1;
		break;
	case HSC_MODULE_MP5990:
		i2c_msg.target_addr = MP5990_ADDR;
		i2c_msg.data[0] = PMBUS_STATUS_BYTE;
		break;
	default:
		LOG_WRN("Unknown hsc module %d", hsc_type);
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_WRN("Failed to read hsc status.");
		msg->completion_code = CC_TIMEOUT;
		return;
	}

	switch (hsc_type) {
	case HSC_MODULE_ADM1278:
	case HSC_MODULE_MP5990:
		msg->data[1] = i2c_msg.data[0] & 0x1C;
		break;
	case HSC_MODULE_LTC4282:
		msg->data[1] = ((i2c_msg.data[0] & BIT(0)) ? BIT(5) : 0) |
			       ((i2c_msg.data[0] & BIT(1)) ? BIT(3) : 0) |
			       ((i2c_msg.data[0] & BIT(2)) ? BIT(4) : 0);
		break;
	default:
		LOG_WRN("Unknown hsc module %d", hsc_type);
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	msg->data_len = 2;
	msg->data[0] = hsc_type;
	msg->completion_code = CC_SUCCESS;
	return;
}
