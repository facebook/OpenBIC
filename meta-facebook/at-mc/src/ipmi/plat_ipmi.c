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
#include "plat_isr.h"
#include "cci.h"
#include "pm8702.h"
#include "power_status.h"
#include "plat_ipmb.h"
#include "app_handler.h"

LOG_MODULE_REGISTER(plat_ipmi);

void pal_construct_ipmi_add_sel_msg(ipmi_msg *msg, common_addsel_msg_t *sel_msg)
{
	CHECK_NULL_ARG(msg);
	CHECK_NULL_ARG(sel_msg);

	static uint16_t record_id = 0x1;
	uint8_t system_event_record = 0x02;
	uint8_t evt_msg_version = 0x04;

	msg->data_len = 16;
	msg->InF_source = SELF;
	msg->InF_target = sel_msg->InF_target;
	msg->netfn = NETFN_STORAGE_REQ;
	msg->cmd = CMD_STORAGE_ADD_SEL;
	msg->data[0] = (record_id & 0xFF); // Record id byte 0, lsb
	msg->data[1] = ((record_id >> 8) & 0xFF); // Record id byte 1
	msg->data[2] = system_event_record; // Record type
	msg->data[3] = 0x00; // Timestamp, bmc would fill up for bic
	msg->data[4] = 0x00; // Timestamp, bmc would fill up for bic
	msg->data[5] = 0x00; // Timestamp, bmc would fill up for bic
	msg->data[6] = 0x00; // Timestamp, bmc would fill up for bic
	msg->data[7] = (SELF_I2C_ADDRESS << 1); // Generator id
	msg->data[8] = 0x00; // Generator id
	msg->data[9] = evt_msg_version; // Event message format version
	memcpy(&msg->data[10], &sel_msg->sensor_type,
	       sizeof(common_addsel_msg_t) - sizeof(uint8_t));
	record_id++;

	return;
}

int pal_cxl_component_id_map_cxl_id(uint8_t component_id, uint8_t *cxl_id)
{
	CHECK_NULL_ARG_WITH_RETURN(cxl_id, -1);

	switch (component_id) {
	case MC_COMPNT_CXL1:
	case MC_COMPNT_CXL2:
	case MC_COMPNT_CXL3:
	case MC_COMPNT_CXL4:
	case MC_COMPNT_CXL5:
	case MC_COMPNT_CXL6:
	case MC_COMPNT_CXL7:
	case MC_COMPNT_CXL8:
		*cxl_id = component_id - MC_COMPNT_CXL1;
		break;
	default:
		return -1;
	}

	return 0;
}

uint8_t fw_update_pm8702(uint8_t cxl_id, uint8_t next_active_slot, uint32_t offset,
			 uint16_t msg_len, uint8_t *msg_buf, bool sector_end)
{
	CHECK_NULL_ARG_WITH_RETURN(msg_buf, FWUPDATE_UPDATE_FAIL);

	/* PM8702 image size maximum 1.3M */
	if (offset > PM8702_UPDATE_MAX_OFFSET) {
		LOG_ERR("Offset: 0x%x is over PM8702 image size maximum", offset);
		return FWUPDATE_ERROR_OFFSET;
	}

	if (msg_len > PM8702_TRANSFER_FW_DATA_LEN) {
		LOG_ERR("Transfer data len over maximum, msg_len: 0x%x", msg_len);
		return FWUPDATE_OVER_LENGTH;
	}

	int req_len = 0;
	uint8_t resp_len = 0;
	pm8702_hbo_status_resp hbo_status = { 0 };

	k_msleep(PM8702_TRANSFER_DELAY_MS);
	if (pal_get_pm8702_hbo_status(cxl_id, (uint8_t *)&hbo_status, &resp_len) != true) {
		LOG_ERR("Fail to get HBO status");
		return FWUPDATE_UPDATE_FAIL;
	}

	if (hbo_status.bo_run != PM8702_NO_HBO_RUN_VAL) {
		LOG_ERR("PM8702 HBO bo_run");
		return FWUPDATE_UPDATE_FAIL;
	}

	if (hbo_status.return_code != PM8702_RETURN_SUCCESS) {
		LOG_ERR("PM8702 HBO return code: 0x%x, vendor status: 0x%x", hbo_status.return_code,
			hbo_status.vendor_status);
		return FWUPDATE_UPDATE_FAIL;
	}

	cci_transfer_fw_req update_fw_req = { 0 };
	req_len = PM8702_TRANSFER_FW_HEADER_LEN + msg_len;

	if (offset == PM8702_INITIATE_FW_OFFSET) {
		update_fw_req.action = INITIATE_FW_TRANSFER;
	} else if (sector_end == true) {
		update_fw_req.action = END_TRANSFER;
	} else {
		update_fw_req.action = CONTINUE_FW_TRANSFER;
	}

	update_fw_req.slot = next_active_slot;
	update_fw_req.offset = offset / PM8702_TRANSFER_FW_DATA_LEN;
	memcpy(update_fw_req.data, msg_buf, sizeof(uint8_t) * msg_len);

	k_msleep(PM8702_TRANSFER_DELAY_MS);

	if (pal_pm8702_transfer_fw(cxl_id, (uint8_t *)&update_fw_req, req_len) != true) {
		LOG_ERR("Fail to transfer PM8702 firmware");
		return FWUPDATE_UPDATE_FAIL;
	}

	return FWUPDATE_SUCCESS;
}

int pal_write_read_cxl_fru(uint8_t optional, uint8_t fru_id, EEPROM_ENTRY *fru_entry,
			   uint8_t *status)
{
	CHECK_NULL_ARG_WITH_RETURN(fru_entry, -1);
	CHECK_NULL_ARG_WITH_RETURN(status, -1);

	bool ret = 0;
	uint8_t retry = 0;
	int mutex_status = 0;

	if (optional != CXL_FRU_WRITE && optional != CXL_FRU_READ) {
		LOG_ERR("CXL fru optional is invalid, optional: %d", optional);
		return -1;
	}

	/* Switch mux channel */
	mux_config cxl_mux = { 0 };
	cxl_mux.bus = I2C_BUS2;
	cxl_mux.target_addr = CXL_FRU_MUX0_ADDR;
	cxl_mux.channel = pal_cxl_map_mux0_channel(fru_id);
	if (cxl_mux.channel == 0) {
		LOG_ERR("Get cxl mux0 channel fail");
		return -1;
	}

	struct k_mutex *mutex = get_i2c_mux_mutex(cxl_mux.bus);
	for (retry = 0; retry < 5; ++retry) {
		mutex_status = k_mutex_lock(mutex, K_MSEC(MUTEX_LOCK_INTERVAL_MS));
		if (mutex_status == 0) {
			break;
		}
	}

	if (retry >= 5) {
		LOG_ERR("Mutex lock fail, status: %d, fru id: 0x%x", mutex_status, fru_id);
		return -1;
	}

	ret = set_mux_channel(cxl_mux, MUTEX_LOCK_ENABLE);
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

	ret = set_mux_channel(cxl_mux, MUTEX_LOCK_ENABLE);
	if (ret == false) {
		LOG_ERR("Disable mux channel fail");
	}

	mutex_status = k_mutex_unlock(mutex);
	if (mutex_status != 0) {
		LOG_ERR("Mutex unlock fail, status: %d", mutex_status);
	}

	return 0;
}

int pal_get_pcie_card_sensor_reading(uint8_t sensor_num, uint8_t cxl_id, uint8_t *card_status,
				     int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(card_status, -1);
	CHECK_NULL_ARG_WITH_RETURN(reading, -1);

	uint8_t sensor_status = 0;
	uint8_t cfg_count = 0;

	sensor_cfg *cfg = NULL;

	cfg = get_cxl_sensor_cfg_info(cxl_id, &cfg_count);
	if (cfg == NULL) {
		LOG_ERR("Fail to find CXL sensor config via cxl id: 0x%x", cxl_id);
		return -1;
	}

	sensor_status = get_sensor_reading(cfg, cfg_count, sensor_num, reading, GET_FROM_CACHE);

	switch (sensor_status) {
	case SENSOR_READ_SUCCESS:
	case SENSOR_READ_ACUR_SUCCESS:
	case SENSOR_READ_4BYTE_ACUR_SUCCESS:
		break;
	case SENSOR_INIT_STATUS:
		*card_status |= PCIE_CARD_DEVICE_NOT_READY_BIT;
		*reading = 0;
		break;
	case SENSOR_NOT_ACCESSIBLE:
	case SENSOR_POLLING_DISABLE:
		*card_status |= PCIE_CARD_NOT_ACCESSIABLE_BIT;
		*reading = 0;
		break;
	case SENSOR_NOT_PRESENT:
		*card_status |= PCIE_CARD_NOT_PRESENT_BIT;
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

	bool ret = false;
	uint8_t cxl_id = 0;
	uint8_t index = 0;
	uint8_t component = msg->data[0];

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
	case MC_COMPNT_CXL1:
	case MC_COMPNT_CXL2:
	case MC_COMPNT_CXL3:
	case MC_COMPNT_CXL4:
	case MC_COMPNT_CXL5:
	case MC_COMPNT_CXL6:
	case MC_COMPNT_CXL7:
	case MC_COMPNT_CXL8:
		if (pal_cxl_component_id_map_cxl_id(component, &cxl_id) != 0) {
			LOG_ERR("Invalid cxl component id: 0x%x", component);
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}

		if (is_cxl_access(cxl_id) != true) {
			msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
			return;
		}

		if (pm8702_table[cxl_id].is_init != true) {
			ret = pal_init_pm8702_info(cxl_id);
			if (ret == false) {
				LOG_ERR("Initial cxl id: 0x%x info fail", cxl_id);
				msg->completion_code = CC_UNSPECIFIED_ERROR;
				return;
			}
		}

		uint8_t *fw_ptr = NULL;
		uint8_t active_slot =
			pm8702_table[cxl_id].dev_info.fw_slot_info.fields.ACTIVE_FW_SLOT;

		switch (active_slot) {
		case SLOT1_FW_ACTIVE:
			fw_ptr = pm8702_table[cxl_id].dev_info.slot1_fw_revision;
			break;
		case SLOT2_FW_ACTIVE:
			fw_ptr = pm8702_table[cxl_id].dev_info.slot2_fw_revision;
			break;
		case SLOT3_FW_ACTIVE:
			fw_ptr = pm8702_table[cxl_id].dev_info.slot3_fw_revision;
			break;
		case SLOT4_FW_ACTIVE:
			fw_ptr = pm8702_table[cxl_id].dev_info.slot4_fw_revision;
			break;
		default:
			LOG_ERR("Active fw slot: 0x%x is invalid", active_slot);
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}

		msg->data[0] = component;
		msg->data[1] = GET_FW_INFO_REVISION_LEN;
		memcpy(&msg->data[2], fw_ptr, sizeof(uint8_t) * GET_FW_INFO_REVISION_LEN);
		msg->data_len = GET_FW_INFO_REVISION_LEN + 2;
		msg->completion_code = CC_SUCCESS;
		break;
	case MC_COMPNT_CXL1_VR_P0V89A:
	case MC_COMPNT_CXL1_VR_P0V8D_PVDDQ_AB:
	case MC_COMPNT_CXL1_VR_VR_PVDDQ_CD:
	case MC_COMPNT_CXL2_VR_P0V89A:
	case MC_COMPNT_CXL2_VR_P0V8D_PVDDQ_AB:
	case MC_COMPNT_CXL2_VR_VR_PVDDQ_CD:
	case MC_COMPNT_CXL3_VR_P0V89A:
	case MC_COMPNT_CXL3_VR_P0V8D_PVDDQ_AB:
	case MC_COMPNT_CXL3_VR_VR_PVDDQ_CD:
	case MC_COMPNT_CXL4_VR_P0V89A:
	case MC_COMPNT_CXL4_VR_P0V8D_PVDDQ_AB:
	case MC_COMPNT_CXL4_VR_VR_PVDDQ_CD:
	case MC_COMPNT_CXL5_VR_P0V89A:
	case MC_COMPNT_CXL5_VR_P0V8D_PVDDQ_AB:
	case MC_COMPNT_CXL5_VR_VR_PVDDQ_CD:
	case MC_COMPNT_CXL6_VR_P0V89A:
	case MC_COMPNT_CXL6_VR_P0V8D_PVDDQ_AB:
	case MC_COMPNT_CXL6_VR_VR_PVDDQ_CD:
	case MC_COMPNT_CXL7_VR_P0V89A:
	case MC_COMPNT_CXL7_VR_P0V8D_PVDDQ_AB:
	case MC_COMPNT_CXL7_VR_VR_PVDDQ_CD:
	case MC_COMPNT_CXL8_VR_P0V89A:
	case MC_COMPNT_CXL8_VR_P0V8D_PVDDQ_AB:
	case MC_COMPNT_CXL8_VR_VR_PVDDQ_CD:
		index = component - MC_COMPNT_CXL1_VR_P0V89A;
		if (cxl_vr_info_table[index].is_init) {
			memcpy(&msg->data[0], cxl_vr_info_table[index].checksum, 4);
			msg->data[4] = cxl_vr_info_table[index].remaining_write;
			msg->data[5] = cxl_vr_info_table[index].vendor;
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

void STORAGE_READ_FRUID_DATA(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	int ret = -1;
	int power_status = 0;
	uint8_t status = 0;
	uint8_t pcie_card_id = msg->data[0] - PCIE_CARD_ID_OFFSET;
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
		power_status = get_pcie_card_power_status(pcie_card_id);
		if (power_status < 0) {
			LOG_ERR("Fail to get PCIE card power status, pcie card id: 0x%x",
				pcie_card_id);
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}

		if ((power_status & PCIE_CARD_POWER_GOOD_BIT) == 0) {
			msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
			return;
		}

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
	int power_status = 0;
	uint8_t status;
	uint8_t pcie_card_id = msg->data[0] - PCIE_CARD_ID_OFFSET;
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
		power_status = get_pcie_card_power_status(pcie_card_id);
		if (power_status < 0) {
			LOG_ERR("Fail to get PCIE card power status, pcie card id: 0x%x",
				pcie_card_id);
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}

		if ((power_status & PCIE_CARD_POWER_GOOD_BIT) == 0) {
			msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
			return;
		}

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
	uint8_t cxl_id = 0;
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
	case CXL_CARD:
		ret = pcie_card_id_to_cxl_id(pcie_card_id, &cxl_id);
		if (ret != 0) {
			LOG_ERR("Fail to get CXL id through pcie card id: 0x%x", pcie_card_id);
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}

		ret = pal_get_pcie_card_sensor_reading(sensor_num, cxl_id, &device_status,
						       &reading);
		if (ret != 0) {
			LOG_ERR("Fail to get CXL sensor reading, cxl id: 0x%x, sensor num: 0x%x",
				cxl_id, sensor_num);
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

void OEM_1S_FW_UPDATE(ipmi_msg *msg)
{
	/*
	* Request:
	* byte 0:   component, bit7 is the indicator sector end
	* byte 1-4: offset, lsb first
	* byte 5-6: length, lsb first
	* byte 7-N: data
	*/

	CHECK_NULL_ARG(msg);

	if (msg->data_len < 8) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t cxl_id = 0;
	uint8_t component = msg->data[0];
	uint8_t status = -1;
	uint32_t offset =
		((msg->data[4] << 24) | (msg->data[3] << 16) | (msg->data[2] << 8) | msg->data[1]);
	uint16_t length = ((msg->data[6] << 8) | msg->data[5]);

	if ((length == 0) || (length != msg->data_len - 7)) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	switch (component) {
	case MC_COMPNT_BIC:
	case (MC_COMPNT_BIC | IS_SECTOR_END_MASK):
		if (offset == 0) {
			// Set default fw update retry count at first package
			set_default_retry_count(FW_UPDATE_RETRY_MAX_COUNT);
		}

		status = fw_update(offset, length, &msg->data[7], (component & IS_SECTOR_END_MASK),
				   DEVSPI_FMC_CS0);
		break;
	case MC_COMPNT_CXL1:
	case (MC_COMPNT_CXL1 | IS_SECTOR_END_MASK):
	case MC_COMPNT_CXL2:
	case (MC_COMPNT_CXL2 | IS_SECTOR_END_MASK):
	case MC_COMPNT_CXL3:
	case (MC_COMPNT_CXL3 | IS_SECTOR_END_MASK):
	case MC_COMPNT_CXL4:
	case (MC_COMPNT_CXL4 | IS_SECTOR_END_MASK):
	case MC_COMPNT_CXL5:
	case (MC_COMPNT_CXL5 | IS_SECTOR_END_MASK):
	case MC_COMPNT_CXL6:
	case (MC_COMPNT_CXL6 | IS_SECTOR_END_MASK):
	case MC_COMPNT_CXL7:
	case (MC_COMPNT_CXL7 | IS_SECTOR_END_MASK):
	case MC_COMPNT_CXL8:
	case (MC_COMPNT_CXL8 | IS_SECTOR_END_MASK):
		if (pal_cxl_component_id_map_cxl_id((component & WITHOUT_SENCTOR_END_MASK),
						    &cxl_id) != 0) {
			LOG_ERR("Invalid cxl component id: 0x%x", component);
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}

		if (is_cxl_access(cxl_id) != true) {
			msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
			return;
		}

		if (pm8702_table[cxl_id].is_init != true) {
			bool ret = pal_init_pm8702_info(cxl_id);
			if (ret == false) {
				LOG_ERR("Initial cxl id: 0x%x info fail", cxl_id);
				msg->completion_code = CC_UNSPECIFIED_ERROR;
				return;
			}
		}

		uint8_t next_active_slot =
			pm8702_table[cxl_id].dev_info.fw_slot_info.fields.NEXT_ACTIVE_FW_SLOT;
		if (next_active_slot == PM8702_DEFAULT_NEXT_ACTIVE_SLOT) {
			uint8_t index = 0;
			uint8_t support_fw_slot = pm8702_table[cxl_id].dev_info.fw_slot_supported;

			for (index = 1; index <= support_fw_slot; ++index) {
				if (index != pm8702_table[cxl_id]
						     .dev_info.fw_slot_info.fields.ACTIVE_FW_SLOT) {
					next_active_slot = index;
				}
			}

			if (next_active_slot == PM8702_DEFAULT_NEXT_ACTIVE_SLOT) {
				LOG_ERR("Fail to find next active slot");
				msg->completion_code = CC_UNSPECIFIED_ERROR;
				return;
			}

			pm8702_table[cxl_id].dev_info.fw_slot_info.fields.NEXT_ACTIVE_FW_SLOT =
				next_active_slot;
		}

		status = fw_update_pm8702(cxl_id, next_active_slot, offset, length, &msg->data[7],
					  (component & IS_SECTOR_END_MASK));
		break;
	default:
		LOG_ERR("target: 0x%x is invalid", component);
		msg->completion_code = CC_INVALID_PARAM;
		return;
	}

	msg->data_len = 0;
	switch (status) {
	case FWUPDATE_SUCCESS:
		msg->completion_code = CC_SUCCESS;
		break;
	case FWUPDATE_OUT_OF_HEAP:
		msg->completion_code = CC_LENGTH_EXCEEDED;
		break;
	case FWUPDATE_OVER_LENGTH:
		msg->completion_code = CC_OUT_OF_SPACE;
		break;
	case FWUPDATE_REPEATED_UPDATED:
		msg->completion_code = CC_INVALID_DATA_FIELD;
		break;
	case FWUPDATE_UPDATE_FAIL:
		msg->completion_code = CC_TIMEOUT;
		break;
	case FWUPDATE_ERROR_OFFSET:
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		break;
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}
	if (status != FWUPDATE_SUCCESS) {
		LOG_ERR("firmware (0x%02X) update failed cc: %x", component, msg->completion_code);
	}

	return;
}

void OEM_1S_SET_DEVICE_ACTIVE(ipmi_msg *msg)
{
	/*
	* Request:
	* byte 0:   component id
	*/

	CHECK_NULL_ARG(msg);

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	int req_len = 0;
	uint8_t cxl_id = 0;
	uint8_t component = msg->data[0];
	uint8_t next_active_slot = 0;

	switch (component) {
	case (MC_COMPNT_CXL1 | IS_SECTOR_END_MASK):
	case (MC_COMPNT_CXL2 | IS_SECTOR_END_MASK):
	case (MC_COMPNT_CXL3 | IS_SECTOR_END_MASK):
	case (MC_COMPNT_CXL4 | IS_SECTOR_END_MASK):
	case (MC_COMPNT_CXL5 | IS_SECTOR_END_MASK):
	case (MC_COMPNT_CXL6 | IS_SECTOR_END_MASK):
	case (MC_COMPNT_CXL7 | IS_SECTOR_END_MASK):
	case (MC_COMPNT_CXL8 | IS_SECTOR_END_MASK):
		if (pal_cxl_component_id_map_cxl_id((component & WITHOUT_SENCTOR_END_MASK),
						    &cxl_id) != 0) {
			LOG_ERR("Invalid cxl component id: 0x%x", component);
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}

		if (is_cxl_access(cxl_id) != true) {
			msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
			return;
		}

		next_active_slot =
			pm8702_table[cxl_id].dev_info.fw_slot_info.fields.NEXT_ACTIVE_FW_SLOT;
		if (next_active_slot == PM8702_DEFAULT_NEXT_ACTIVE_SLOT) {
			LOG_ERR("Invalid active slot, cxl id: 0x%x", cxl_id);
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}

		cci_activate_fw_req activate_req = { 0 };
		req_len = sizeof(cci_activate_fw_req);
		activate_req.action = NEXT_COLD_RESET_ACTIVE_FW;
		activate_req.slot = next_active_slot;

		if (pal_set_pm8702_active_slot(cxl_id, (uint8_t *)&activate_req, req_len) != true) {
			LOG_ERR("Fail to set next active slot");
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}
		break;

	default:
		LOG_ERR("Invalid component id: 0x%x", component);
		msg->completion_code = CC_INVALID_PARAM;
		return;
	}

	msg->data_len = 0;
	msg->completion_code = CC_SUCCESS;
	return;
}

void OEM_1S_BRIDGE_I2C_MSG_BY_COMPNT(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len < 2) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t fru_id = msg->data[0];
	uint8_t sensor_num = 0;
	ipmi_msg bridge_msg = { 0 };

	switch (fru_id) {
	case MC_E1S_1_FRU_ID:
		sensor_num = SENSOR_NUM_TEMP_E1S_1;
		break;
	case MC_E1S_2_FRU_ID:
		sensor_num = SENSOR_NUM_TEMP_E1S_2;
		break;
	case MC_E1S_3_FRU_ID:
		sensor_num = SENSOR_NUM_TEMP_E1S_3;
		break;
	case MC_E1S_4_FRU_ID:
		sensor_num = SENSOR_NUM_TEMP_E1S_4;
		break;
	default:
		msg->completion_code = CC_INVALID_PARAM;
		return;
	}

	sensor_cfg *cfg = get_common_sensor_cfg_info(sensor_num);
	if (cfg == NULL) {
		LOG_ERR("Fail to find sensor cfg via sensor num: 0x%x, fru id: 0x%x", sensor_num,
			fru_id);
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	if (cfg->access_checker(sensor_num) != true) {
		msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
		return;
	}

	if (cfg->pre_sensor_read_hook) {
		if (cfg->pre_sensor_read_hook(cfg, cfg->pre_sensor_read_args) != true) {
			LOG_ERR("Failed to do pre sensor read function, sensor num: 0x%x",
				cfg->num);
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}
	}

	bridge_msg.data[0] = cfg->port << 1;
	bridge_msg.data[1] = cfg->target_addr << 1;
	memcpy(&bridge_msg.data[2], &msg->data[1], msg->data_len - 1);
	bridge_msg.data_len = msg->data_len + 1;

	APP_MASTER_WRITE_READ(&bridge_msg);

	if (cfg->post_sensor_read_hook) {
		if (cfg->post_sensor_read_hook(cfg, cfg->post_sensor_read_args, NULL) != true) {
			LOG_ERR("Failed to do post sensor read function, sensor num: 0x%x",
				cfg->num);
		}
	}

	msg->completion_code = bridge_msg.completion_code;
	msg->data_len = bridge_msg.data_len;
	memcpy(&msg->data[0], &bridge_msg.data[0], bridge_msg.data_len);
	return;
}
