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

#include "plat_ipmi.h"

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

	if (!rg3mxxb12_set_slave_port(I3C_BUS4, RG3MXXB12_DEFAULT_STATIC_ADDRESS,
				      slave_port_setting)) {
		LOG_ERR("Failed to set slave port to slave port: 0x%x", slave_port_setting);
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		goto exit;
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
