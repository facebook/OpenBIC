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
#include <logging/log.h>
#include "libutil.h"
#include "ipmi.h"
#include "fru.h"
#include "eeprom.h"
#include "power_status.h"
#include "sensor.h"
#include "plat_fru.h"
#include "plat_class.h"
#include "plat_hook.h"
#include "plat_ipmb.h"
#include "plat_dimm.h"
#include "plat_sensor_table.h"
#include "pmbus.h"
#include "oem_1s_handler.h"

#include "pldm.h"
#include "plat_mctp.h"

#include "pldm.h"
#include "plat_mctp.h"

LOG_MODULE_REGISTER(plat_ipmi);

bool pal_request_msg_to_BIC_from_HOST(uint8_t netfn, uint8_t cmd)
{
	if (netfn == NETFN_OEM_1S_REQ) {
		if ((cmd == CMD_OEM_1S_FW_UPDATE) || (cmd == CMD_OEM_1S_RESET_BMC) ||
		    (cmd == CMD_OEM_1S_GET_BIC_STATUS) || (cmd == CMD_OEM_1S_RESET_BIC) ||
		    (cmd == CMD_OEM_1S_GET_BIC_FW_INFO))
			return true;
	} else if (netfn == NETFN_APP_REQ) {
		if (cmd == CMD_APP_GET_SYSTEM_GUID) {
			return true;
		}
	}

	return false;
}

int pal_record_bios_fw_version(uint8_t *buf, uint8_t size)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, -1);

	int ret = -1;
	EEPROM_ENTRY set_bios_ver = { 0 };
	EEPROM_ENTRY get_bios_ver = { 0 };

	const uint8_t block_index = buf[3];
	if (block_index >= BIOS_FW_VERSION_BLOCK_NUM) {
		LOG_ERR("bios version block index is out of range");
		return -1;
	}

	ret = get_bios_version(&get_bios_ver, block_index);
	if (ret == -1) {
		LOG_ERR("Get version fail");
		return -1;
	}

	set_bios_ver.data_len = size - 3; // skip netfn, cmd and command code
	memcpy(&set_bios_ver.data[0], &buf[3], set_bios_ver.data_len);

	// Check the written BIOS version is the same with the stored
	ret = memcmp(&get_bios_ver.data[0], &set_bios_ver.data[0],
		     BIOS_FW_VERSION_BLOCK_MAX_SIZE * sizeof(uint8_t));
	if (ret == 0) {
		LOG_DBG("The Written bios version is the same with the stored bios version in EEPROM");
	} else {
		LOG_DBG("Set bios version");

		ret = set_bios_version(&set_bios_ver, block_index);
		if (ret == -1) {
			LOG_ERR("Set version fail");
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
			LOG_ERR("Get version fail");
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}

		memcpy(msg->data + msg->data_len, get_bios_ver.data, get_bios_ver.data_len);
		msg->data_len += get_bios_ver.data_len;
	}

	msg->completion_code = CC_SUCCESS;

	return;
}

void OEM_1S_GET_CARD_TYPE(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

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
		i2c_msg.target_addr = ADI_ADM1278_ADDR;
		i2c_msg.data[0] = PMBUS_STATUS_BYTE;
		break;
	case HSC_MODULE_LTC4282:
		i2c_msg.target_addr = ADI_LTC4282_ADDR;
		i2c_msg.data[0] = LTC4282_STATUS_OFFSET_BYTE1;
		break;
	case HSC_MODULE_MP5990:
		i2c_msg.target_addr = MPS_MP5990_ADDR;
		i2c_msg.data[0] = PMBUS_STATUS_BYTE;
		break;
	case HSC_MODULE_LTC4286:
		i2c_msg.target_addr = ADI_LTC4286_ADDR;
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
	case HSC_MODULE_LTC4286:
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

	msg->data[0] = hsc_type;
	msg->data_len = 2;
	msg->completion_code = CC_SUCCESS;
	return;
}

/*
Byte 0 - DIMM location
  00h - A0
  01h - A2
  02h - A3
  03h - A4
  04h - A6
  05h - A7
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

	// If host is DC on, BIC can't read DIMM information via I3C
	// Return failed and BMC asks ME
	if (get_DC_status()) {
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

	if (k_mutex_lock(&i3c_dimm_mux_mutex, K_MSEC(I3C_DIMM_MUX_MUTEX_TIMEOUT_MS))) {
		LOG_ERR("Failed to lock I3C dimm MUX");
		msg->completion_code = CC_NODE_BUSY;
		return;
	}

	ret = switch_i3c_dimm_mux(I3C_MUX_TO_BIC, dimm_id / (MAX_COUNT_DIMM / 2));
	if (ret < 0) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		goto exit;
	}

	// I3C_CCC_RSTDAA: Reset dynamic address assignment
	// I3C_CCC_SETAASA: Set all addresses to static address
	ret = all_brocast_ccc(&i3c_msg);
	if (ret != 0) {
		LOG_ERR("Failed to brocast CCC, ret%d bus%d", ret, i3c_msg.bus);
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		goto exit;
	}

	switch (device_type) {
	case DIMM_SPD:
	case DIMM_SPD_NVM:
		i3c_msg.target_addr = spd_i3c_addr_list[dimm_id % (MAX_COUNT_DIMM / 2)];

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
		break;

	case DIMM_PMIC:
		i3c_msg.target_addr = pmic_i3c_addr_list[dimm_id % (MAX_COUNT_DIMM / 2)];

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
	// Switch I3C MUX to CPU after read finish
	switch_i3c_dimm_mux(I3C_MUX_TO_CPU, DIMM_MUX_TO_DIMM_A0A1A3);

	if (k_mutex_unlock(&i3c_dimm_mux_mutex)) {
		LOG_ERR("Failed to lock I3C dimm MUX");
	}
}

void OEM_1S_GET_DIMM_I3C_MUX_SELECTION(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	I2C_MSG i2c_msg = { 0 };
	int ret = 0, retry = 3;

	i2c_msg.bus = I2C_BUS1;
	i2c_msg.target_addr = CPLD_ADDR;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = DIMM_I3C_MUX_CONTROL_OFFSET;

	ret = i2c_master_read(&i2c_msg, retry);
	if (ret != 0) {
		LOG_ERR("Failed to read I3C MUX status, ret=%d", ret);
		return;
	}

	if (GETBIT(i2c_msg.data[0], 0) == I3C_MUX_TO_CPU) {
		msg->data[0] = I3C_MUX_TO_CPU;
	} else if (GETBIT(i2c_msg.data[0], 0) == I3C_MUX_TO_BIC) {
		msg->data[0] = I3C_MUX_TO_BIC;
	} else {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	msg->completion_code = CC_SUCCESS;
	msg->data_len = 1;
	return;
}

void OEM_1S_GET_FW_VERSION(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);
	uint8_t index = 0;

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t component = msg->data[0];
	ipmb_error status;
	ipmi_msg *bridge_msg;

	switch (component) {
	case COMPNT_CPLD:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	case COMPNT_BIC:
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
	case COMPNT_ME:
		if ((IPMB_inf_index_map[ME_IPMB] == RESERVED) ||
		    (IPMB_config_table[IPMB_inf_index_map[ME_IPMB]].enable_status == DISABLE)) {
			LOG_ERR("IPMB ME interface not enabled.");
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}

		bridge_msg = (ipmi_msg *)malloc(sizeof(ipmi_msg));
		if (bridge_msg == NULL) {
			msg->completion_code = CC_OUT_OF_SPACE;
			return;
		}
		bridge_msg->data_len = 0;
		bridge_msg->seq_source = 0xff;
		bridge_msg->InF_source = SELF;
		bridge_msg->InF_target = ME_IPMB;
		bridge_msg->netfn = NETFN_APP_REQ;
		bridge_msg->cmd = CMD_APP_GET_DEVICE_ID;

		status = ipmb_read(bridge_msg, IPMB_inf_index_map[bridge_msg->InF_target]);
		if (status != IPMB_ERROR_SUCCESS) {
			LOG_ERR("ipmb read fail status: %x", status);
			SAFE_FREE(bridge_msg);
			msg->completion_code = CC_BRIDGE_MSG_ERR;
			return;
		} else {
			msg->data[0] = bridge_msg->data[2] & 0x7F;
			msg->data[1] = bridge_msg->data[3] >> 4;
			msg->data[2] = bridge_msg->data[3] & 0x0F;
			msg->data[3] = bridge_msg->data[12];
			msg->data[4] = bridge_msg->data[13] >> 4;
			msg->data_len = 5;
			msg->completion_code = CC_SUCCESS;
			SAFE_FREE(bridge_msg);
		}
		break;
	case COMPNT_PVCCIN:
	case COMPNT_PVCCFA_EHV_FIVRA:
	case COMPNT_PVCCD_HV:
	case COMPNT_PVCCINFAON:
	case COMPNT_PVCCFA_EHV:
		/* Only for Infineon xdpe15284 */
		index = component - COMPNT_PVCCIN;
		if (ifx_vr_fw_info_table[index].is_init) {
			memcpy(&msg->data[0], ifx_vr_fw_info_table[index].checksum, 4);
			msg->data[4] = ifx_vr_fw_info_table[index].remaining_write;
			msg->data[5] = ifx_vr_fw_info_table[index].vendor;
			msg->data_len = 6;
			msg->completion_code = CC_SUCCESS;
		} else {
			msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
		}
		break;
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}
	return;
}
