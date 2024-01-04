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

#include "libutil.h"
#include "ipmi.h"
#include "plat_class.h"
#include "plat_ipmb.h"
#include <logging/log.h>
#include "plat_dimm.h"
#include "plat_i3c.h"
#include "power_status.h"
#include "rg3mxxb12.h"
#include "p3h284x.h"
#include "plat_ipmi.h"
#include "plat_ipmb.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_ipmi);

bool pal_set_dimm_presence_status(uint8_t *buf)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, -1);

	uint8_t dimm_index = buf[DIMM_INDEX_BYTE];
	uint8_t status = buf[DIMM_STATUS_BYTE];

	if (dimm_index < DIMM_INDEX_MIN || dimm_index > DIMM_INDEX_MAX) {
		LOG_ERR("DIMM index is out of range");
		return false;
	}

	set_dimm_presence_status(dimm_index, status);

	return true;
}

void OEM_1S_GET_CARD_TYPE(ipmi_msg *msg)
{
	if (msg == NULL) {
		LOG_ERR("Failed due to parameter *msg is NULL");
		return;
	}

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	CARD_STATUS _1ou_status = get_1ou_status();
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
	default:
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_DATA_FIELD;
		break;
	}

	return;
}

/*
Byte 0 - DIMM location
  00h - A
  01h - B
  02h - C
  03h - D
  04h - E
  05h - F
  06h - G
  07h - H
Byte 1: Device type
  00h - SPD
  01h - SPD NVM
  02h - PMIC
Byte 2: Read/write data length
Byte 3:4: 2byte offset
Byte 5:~ write data
*/
void OEM_1S_WRITE_READ_DIMM(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	// At least include DIMM location, device type, write/read len, offset
	if (msg->data_len < 4) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	int ret = 0;
	uint8_t dimm_id = msg->data[0];
	uint8_t device_type = msg->data[1];

	// If host is DC on but POST not complete, BIC can't read DIMM information via I3C
	// Return failed
	if (get_DC_status() && (!get_post_status())) {
		msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
		return;
	}

	I3C_MSG i3c_msg = { 0 };
	i3c_msg.bus = I3C_BUS4;
	i3c_msg.tx_len = msg->data_len - 3;
	i3c_msg.rx_len = msg->data[2];
	uint16_t i3c_hub_type = I3C_HUB_TYPE_UNKNOWN;
	i3c_hub_type = get_i3c_hub_type();

	// Check offset byte count: SPD_NVM has 2 bytes offset
	if (device_type == DIMM_SPD_NVM) {
		if (i3c_msg.tx_len < 2) {
			msg->completion_code = CC_INVALID_DATA_FIELD;
			return;
		}
	} else {
		// One byte offset
		if (i3c_msg.tx_len < 1) {
			msg->completion_code = CC_INVALID_DATA_FIELD;
			return;
		}
	}

	memcpy(&i3c_msg.data[0], &msg->data[3], i3c_msg.tx_len);
	msg->data_len = i3c_msg.rx_len;

	if (k_mutex_lock(&i3c_dimm_mutex, K_MSEC(I3C_DIMM_MUTEX_TIMEOUT_MS))) {
		LOG_ERR("Failed to lock I3C dimm MUX");
		msg->completion_code = CC_NODE_BUSY;
		return;
	}

	ret = switch_i3c_dimm_mux(I3C_MUX_TO_BIC);
	if (ret < 0) {
		LOG_ERR("Failed to switch mux to BIC");
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		goto exit;
	}

	uint8_t slave_port_setting =
		(dimm_id / (MAX_COUNT_DIMM / 2)) ? I3C_HUB_TO_DIMMEFGH : I3C_HUB_TO_DIMMABCD;

	if (i3c_hub_type == RG3M87B12_DEVICE_INFO) {
		if (!rg3mxxb12_set_slave_port(I3C_BUS4, RG3MXXB12_DEFAULT_STATIC_ADDRESS,
					      slave_port_setting)) {
			LOG_ERR("Failed to set slave port to slave port: 0x%x", slave_port_setting);
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			goto exit;
		}
	} else {
		if (!p3h284x_set_slave_port(I3C_BUS4, P3H284X_DEFAULT_STATIC_ADDRESS,
					    slave_port_setting)) {
			LOG_ERR("Failed to set slave port to slave port: 0x%x", slave_port_setting);
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			goto exit;
		}
	}

	switch (device_type) {
	case DIMM_SPD:
	case DIMM_SPD_NVM:
		i3c_msg.target_addr = spd_i3c_addr_list[dimm_id % (MAX_COUNT_DIMM / 2)];
		i3c_attach(&i3c_msg);

		// I3C_CCC_RSTDAA: Reset dynamic address assignment
		// I3C_CCC_SETAASA: Set all addresses to static address
		ret = all_brocast_ccc(&i3c_msg);
		if (ret != 0) {
			LOG_ERR("Failed to brocast CCC, ret%d bus%d", ret, i3c_msg.bus);
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			goto exit;
		}

		if (device_type == DIMM_SPD_NVM) {
			ret = i3c_spd_reg_read(&i3c_msg, true);
		} else {
			ret = i3c_spd_reg_read(&i3c_msg, false);
		}

		if (ret != 0) {
			LOG_ERR("Failed to read SPD addr0x%x offset0x%x, ret%d",
				i3c_msg.target_addr, i3c_msg.data[0], ret);
			msg->completion_code = CC_UNSPECIFIED_ERROR;
		} else {
			memcpy(&msg->data[0], &i3c_msg.data, i3c_msg.rx_len);
			msg->data_len = i3c_msg.rx_len;
			msg->completion_code = CC_SUCCESS;
		}

		i3c_detach(&i3c_msg);
		break;

	case DIMM_PMIC:
		i3c_msg.target_addr = pmic_i3c_addr_list[dimm_id % (MAX_COUNT_DIMM / 2)];

		// I3C_CCC_RSTDAA: Reset dynamic address assignment
		// I3C_CCC_SETAASA: Set all addresses to static address
		ret = all_brocast_ccc(&i3c_msg);
		if (ret != 0) {
			LOG_ERR("Failed to brocast CCC, ret%d bus%d", ret, i3c_msg.bus);
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			goto exit;
		}

		ret = i3c_transfer(&i3c_msg);
		if (ret != 0) {
			LOG_ERR("Failed to read PMIC addr0x%x offset0x%x, ret%d bus%d",
				i3c_msg.target_addr, i3c_msg.data[0], ret, i3c_msg.bus);
			msg->completion_code = CC_UNSPECIFIED_ERROR;
		} else {
			memcpy(&msg->data[0], &i3c_msg.data, i3c_msg.rx_len);
			msg->data_len = i3c_msg.rx_len;
			msg->completion_code = CC_SUCCESS;
		}
		break;

	default:
		msg->completion_code = CC_INVALID_DATA_FIELD;
		break;
	}

exit:
	if (k_mutex_unlock(&i3c_dimm_mutex)) {
		LOG_ERR("Failed to lock I3C dimm MUX");
	}
}

#ifdef CONFIG_I2C_IPMB_SLAVE
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
		if (_1ou_status.card_type == TYPE_1OU_OLMSTED_POINT) {
			target_IF = EXP1_IPMB;
		}
		break;
	case EXP4_IPMB:
		target_IF = EXP3_IPMB;
		break;
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

			if ((_1ou_status.card_type == TYPE_1OU_OLMSTED_POINT) &&
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
