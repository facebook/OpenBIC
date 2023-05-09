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
#include "plat_spi.h"
#include "plat_sensor_table.h"
#include "sensor.h"
#include "pmbus.h"

LOG_MODULE_REGISTER(plat_ipmi);

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
		if (_1ou_status.card_type == TYPE_1OU_EXP_WITH_E1S) {
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

void OEM_1S_COPY_FLASH_IMAGE(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 13) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t copy_type = msg->data[0];
	uint32_t src_offset =
		msg->data[1] | (msg->data[2] << 8) | (msg->data[3] << 16) | (msg->data[4] << 24);
	uint32_t dest_offset =
		msg->data[5] | (msg->data[6] << 8) | (msg->data[7] << 16) | (msg->data[8] << 24);
	uint32_t length =
		msg->data[9] | (msg->data[10] << 8) | (msg->data[11] << 16) | (msg->data[12] << 24);

	if (((src_offset % SECTOR_SZ_4K) != 0) || ((dest_offset % SECTOR_SZ_4K) != 0) ||
	    ((length % SECTOR_SZ_4K) != 0)) {
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	if (start_flash_copy(copy_type, src_offset, dest_offset, length)) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	msg->data_len = 0;
	msg->completion_code = CC_SUCCESS;
	return;
}

void GET_COPY_FLASH_STATUS(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	FLASH_COPY_INFO current_info = { 0 };

	get_flash_copy_info(&current_info);

	msg->data[0] = current_info.status;
	msg->data[1] = current_info.completion_code;
	if ((current_info.total_length == 0)) {
		msg->data[2] = 0;
	} else {
		msg->data[2] = 100 * current_info.current_len / current_info.total_length;
	}
	msg->data_len = 3;
	msg->completion_code = CC_SUCCESS;
	return;
}

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

	i2c_msg.bus = I2C_BUS5;
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
