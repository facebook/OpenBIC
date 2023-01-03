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
#include "plat_hook.h"
#include "plat_dev.h"

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

int pal_get_pcie_card_sensor_reading(uint8_t read_type, uint8_t sensor_num, uint8_t pcie_card_id,
				     uint8_t *card_status, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(card_status, -1);
	CHECK_NULL_ARG_WITH_RETURN(reading, -1);

	bool ret = 0;
	int retry = 0;
	uint8_t index = 0;
	uint8_t sensor_status = 0;
	bool (*pre_switch_mux_func)(uint8_t, uint8_t) = NULL;
	bool (*post_switch_mux_func)(uint8_t, uint8_t) = NULL;
	sensor_cfg *cfg = NULL;

	switch (read_type) {
	case PCIE_CARD_E1S:
		pre_switch_mux_func = pre_e1s_switch_mux;
		post_switch_mux_func = post_e1s_switch_mux;
		if (sensor_num <= SENSOR_NUM_TEMP_JCN_E1S_1) {
			index = sensor_num - 1;
			if (pcie_card_id <= CARD_12_INDEX) {
				cfg = &plat_e1s_1_12_sensor_config[index];
			} else {
				cfg = &plat_e1s_13_14_sensor_config[index];
			}
		} else {
			LOG_ERR("Invalid e1s sensor num: 0x%x", sensor_num);
			return -1;
		}
		break;
	case PCIE_CARD_CXL:
		pre_switch_mux_func = pre_cxl_switch_mux;
		post_switch_mux_func = post_cxl_switch_mux;

		ret = get_cxl_sensor_config_index(sensor_num, &index);
		if (ret != true) {
			LOG_ERR("Invalid cxl sensor num: 0x%x", sensor_num);
			return -1;
		}
		cfg = &plat_cxl_sensor_config[index];
		break;
	default:
		LOG_ERR("Invalid read_type: %d", read_type);
		return -1;
	}

	for (retry = 0; retry < 3; ++retry) {
		*reading = 0;

		if (cfg->access_checker(sensor_num) != true) {
			*card_status |= PCIE_CARD_NOT_ACCESSIABLE_BIT;
			return 0;
		}

		ret = pre_switch_mux_func(sensor_num, pcie_card_id);
		if (ret != true) {
			LOG_ERR("Pre switch mux fail, sensor num: 0x%x, card id: 0x%x", sensor_num,
				pcie_card_id);
			return -1;
		}

		if (cfg->pre_sensor_read_hook) {
			if (cfg->pre_sensor_read_hook(sensor_num, cfg->pre_sensor_read_args) ==
			    false) {
				LOG_ERR("Pre sensor read function, sensor number: 0x%x",
					sensor_num);
				return -1;
			}
		}

		ret = pal_sensor_drive_read(cfg, reading, &sensor_status);
		if (ret != true) {
			LOG_ERR("sensor: 0x%x read fail", sensor_num);
		}

		if (cfg->post_sensor_read_hook) {
			if (cfg->post_sensor_read_hook(sensor_num, cfg->post_sensor_read_args,
						       reading) == false) {
				LOG_ERR("Post sensor read function, sensor number: 0x%x",
					sensor_num);
			}
		}

		ret = post_switch_mux_func(sensor_num, pcie_card_id);
		if (ret != true) {
			LOG_ERR("Post switch mux fail, sensor num: 0x%x, card id: 0x%x", sensor_num,
				pcie_card_id);
		}

		if ((sensor_status == SENSOR_READ_SUCCESS) ||
		    (sensor_status == SENSOR_READ_ACUR_SUCCESS)) {
			break;
		}
	}

	switch (sensor_status) {
	case SENSOR_READ_SUCCESS:
	case SENSOR_READ_ACUR_SUCCESS:
		break;
	case SENSOR_INIT_STATUS:
	case SENSOR_NOT_ACCESSIBLE:
		*card_status |= PCIE_CARD_DEVICE_NOT_READY_BIT;
		*reading = 0;
		break;
	default:
		*reading = 0;
		return -1;
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

void OEM_1S_GET_PCIE_CARD_SENSOR_READING(ipmi_msg *msg)
{
	/* IPMI command format
  *  Request:
  *    Byte 0: FRU id
  *    Byte 1: Sensor number
  *  Response:
  *    Byte 0:   Device status (bit 0: presence status, bit 1: power status, bit 2: device status)
  *    Byte 1~2: Integer bytes
  *    Byte 3~4: Fraction bytes */

	CHECK_NULL_ARG(msg);

	if (msg->data_len != 2) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	int ret = -1;
	int reading = 0;
	uint8_t card_type = 0;
	uint8_t device_status = 0;
	uint8_t fru_id = msg->data[0];
	uint8_t sensor_num = msg->data[1];
	uint8_t pcie_card_id = fru_id - PCIE_CARD_ID_OFFSET;

	ret = get_pcie_card_type(pcie_card_id, &card_type);
	if (ret < 0) {
		msg->completion_code = CC_INVALID_PARAM;
		return;
	}

	switch (card_type) {
	case E1S_0_CARD:
	case E1S_1_CARD:
	case E1S_0_1_CARD:

		ret = pal_get_pcie_card_sensor_reading(PCIE_CARD_E1S, sensor_num, pcie_card_id,
						       &device_status, &reading);

		if (ret < 0) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}
		break;
	case CXL_CARD:

		ret = pal_get_pcie_card_sensor_reading(PCIE_CARD_CXL, sensor_num, pcie_card_id,
						       &device_status, &reading);

		if (ret < 0) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}
		break;
	case CARD_NOT_PRESENT:
		device_status |= PCIE_CARD_NOT_PRESENT_BIT;
		break;
	default:
		msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
		return;
	}

	msg->data_len = 5;
	msg->data[0] = device_status;
	memcpy(&msg->data[1], &reading, sizeof(reading));
	msg->completion_code = CC_SUCCESS;
	return;
}
