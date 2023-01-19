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
#include "util_spi.h"
#include "fru.h"
#include "plat_fru.h"
#include "hal_i2c.h"
#include "plat_sensor_table.h"
#include "common_i2c_mux.h"
#include "plat_sensor_table.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_ipmi);

int pal_write_read_cxl_fru(uint8_t optional, uint8_t fru_id, EEPROM_ENTRY *fru_entry,
			   uint8_t *status)
{
	CHECK_NULL_ARG_WITH_RETURN(fru_entry, -1);
	CHECK_NULL_ARG_WITH_RETURN(status, -1);

	bool ret = 0;

	if (optional != CXL_FRU_WRITE && optional != CXL_FRU_READ) {
		LOG_ERR("CXL fru optional is invalid, optional: %d", optional);
		return -1;
	}

	/* Switch mux channel */
	mux_config cxl_mux = { 0 };
	cxl_mux.bus = I2C_BUS2;
	cxl_mux.target_addr = CXL_FRU_MUX0_ADDR;
	cxl_mux.channel = pal_cxl_map_mux0_channel(fru_id);
	if (cxl_mux.channel < 0) {
		LOG_ERR("Get cxl mux0 channel fail");
		return -1;
	}

	struct k_mutex *mutex = get_i2c_mux_mutex(cxl_mux.bus);
	int mutex_status = k_mutex_lock(mutex, K_MSEC(MUTEX_LOCK_INTERVAL_MS));
	if (mutex_status != 0) {
		LOG_ERR("Mutex lock fail, status: %d", mutex_status);
		return -1;
	}

	ret = set_mux_channel(cxl_mux);
	if (ret == false) {
		LOG_ERR("Switch mux channel fail");
		k_mutex_unlock(mutex);
		return -1;
	}

	if (optional == CXL_FRU_WRITE) {
		*status = FRU_write(fru_entry);
	} else {
		*status = FRU_read(fru_entry);
	}

	/* Disable mux channel */
	cxl_mux.channel = 0;

	ret = set_mux_channel(cxl_mux);
	if (ret == false) {
		LOG_ERR("Disable mux channel fail");
	}

	mutex_status = k_mutex_unlock(mutex);
	if (mutex_status != 0) {
		LOG_ERR("Mutex unlock fail, status: %d", mutex_status);
	}

	return 0;
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

	if (component >= MC_COMPNT_MAX) {
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		return;
	}
	/*
	* Return data format:
	* data[0] = component id
	* data[1] = data length
	* data[2] - data[data length + 1] = firmware version
	*/
	switch (component) {
	case MC_COMPNT_BIC:
		msg->data[0] = MC_COMPNT_BIC;
		msg->data[1] = BIC_FW_DATA_LENGTH;
		msg->data[2] = BIC_FW_YEAR_MSB;
		msg->data[3] = BIC_FW_YEAR_LSB;
		msg->data[4] = BIC_FW_WEEK;
		msg->data[5] = BIC_FW_VER;
		msg->data[6] = BIC_FW_platform_0;
		msg->data[7] = BIC_FW_platform_1;
		msg->data[8] = BIC_FW_platform_2;
		msg->data_len = 9;
		msg->completion_code = CC_SUCCESS;
		break;
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}
	return;
}

void STORAGE_READ_FRUID_DATA(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	int ret = -1;
	uint8_t status = 0;
	EEPROM_ENTRY fru_entry;

	if (msg->data_len != 4) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	fru_entry.config.dev_id = msg->data[0];
	fru_entry.offset = (msg->data[2] << 8) | msg->data[1];
	fru_entry.data_len = msg->data[3];

	// According to IPMI, messages are limited to 32 bytes
	if (fru_entry.data_len > 32) {
		msg->completion_code = CC_LENGTH_EXCEEDED;
		return;
	}

	if (fru_entry.config.dev_id != MC_FRU_ID) {
		ret = pal_write_read_cxl_fru(CXL_FRU_READ, fru_entry.config.dev_id, &fru_entry,
					     &status);
		if (ret < 0) {
			msg->completion_code = CC_INVALID_PARAM;
			return;
		}
		goto exit;
	}

	status = FRU_read(&fru_entry);

exit:
	msg->data_len = fru_entry.data_len + 1;
	msg->data[0] = fru_entry.data_len;
	memcpy(&msg->data[1], &fru_entry.data[0], fru_entry.data_len);

	switch (status) {
	case FRU_READ_SUCCESS:
		msg->completion_code = CC_SUCCESS;
		break;
	case FRU_INVALID_ID:
		msg->completion_code = CC_INVALID_PARAM;
		break;
	case FRU_OUT_OF_RANGE:
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		break;
	case FRU_FAIL_TO_ACCESS:
		msg->completion_code = CC_FRU_DEV_BUSY;
		break;
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}

	return;
}

void STORAGE_WRITE_FRUID_DATA(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	int ret = -1;
	uint8_t status;
	EEPROM_ENTRY fru_entry;

	if (msg->data_len < 4) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	fru_entry.config.dev_id = msg->data[0];
	fru_entry.offset = (msg->data[2] << 8) | msg->data[1];
	fru_entry.data_len = msg->data_len - 3; // skip id and offset
	if (fru_entry.data_len > 32) { // According to IPMI, messages are limited to 32 bytes
		msg->completion_code = CC_LENGTH_EXCEEDED;
		return;
	}
	memcpy(&fru_entry.data[0], &msg->data[3], fru_entry.data_len);

	msg->data[0] = msg->data_len - 3;
	msg->data_len = 1;

	if (fru_entry.config.dev_id != MC_FRU_ID) {
		ret = pal_write_read_cxl_fru(CXL_FRU_WRITE, fru_entry.config.dev_id, &fru_entry,
					     &status);
		if (ret < 0) {
			msg->completion_code = CC_INVALID_PARAM;
			return;
		}
		goto exit;
	}

	status = FRU_write(&fru_entry);

exit:
	switch (status) {
	case FRU_WRITE_SUCCESS:
		msg->completion_code = CC_SUCCESS;
		break;
	case FRU_INVALID_ID:
		msg->completion_code = CC_INVALID_PARAM;
		break;
	case FRU_OUT_OF_RANGE:
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		break;
	case FRU_FAIL_TO_ACCESS:
		msg->completion_code = CC_FRU_DEV_BUSY;
		break;
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}

	return;
}

void OEM_1S_GET_PCIE_CARD_STATUS(ipmi_msg *msg)
{
	/* IPMI command format
	*  Request:
	*    Byte 0: FRU id
	*    Byte 1: PCIE device id
	*  Response:
	*    Byte 0: PCIE card presence status (0: not present, 1: present)
	*    Byte 1: PCIE card type (Return unknown type if device not present) */

	CHECK_NULL_ARG(msg);

	if (msg->data_len != 2) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	/* BMC would check pcie card type via fru id and device id */
	int ret = -1;
	uint8_t fru_id = msg->data[0];
	uint8_t pcie_card_id = fru_id - PCIE_CARD_ID_OFFSET;
	uint8_t pcie_device_id = msg->data[1];
	uint8_t card_type = 0;
	uint8_t presence_status = PCIE_CARD_PRESENT;

	ret = get_pcie_device_type(pcie_card_id, pcie_device_id, &card_type);
	if (ret < 0) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	if (card_type == CARD_NOT_PRESENT) {
		presence_status = PCIE_CARD_NOT_PRESENT;
		card_type = UNKNOWN_CARD;
	}

	msg->data[0] = presence_status;
	msg->data[1] = card_type;
	msg->data_len = 2;
	msg->completion_code = CC_SUCCESS;
	return;
}
