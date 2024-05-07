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
#include "fru.h"
#include "ipmi.h"
#include "util_spi.h"
#include "hal_gpio.h"
#include "plat_fru.h"
#include "plat_gpio.h"
#include "plat_class.h"
#include "plat_sensor_table.h"
#include "pex89000.h"
#include "xdpe15284.h"
#include "hal_gpio.h"
#include "plat_gpio.h"
#include "plat_hook.h"
#include "plat_dev.h"
#include "fru.h"
#include "app_handler.h"
#include "plat_ipmb.h"
#include "mp2985.h"

LOG_MODULE_REGISTER(plat_ipmi);

struct SWITCH_MUX_INFO pcie_switch_mux_info[PEX_MAX_NUMBER] = {
	[0] = { .device = DEVSPI_SPI1_CS0,
		.control_gpio = SPI_ROM0_SEL,
		.sw_to_flash_value = GPIO_HIGH,
		.bic_to_flash_value = GPIO_LOW },
	[1] = { .device = DEVSPI_SPI2_CS0,
		.control_gpio = SPI_ROM1_SEL,
		.sw_to_flash_value = GPIO_HIGH,
		.bic_to_flash_value = GPIO_LOW },
};

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

void OEM_1S_GET_FW_VERSION(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t component;
	component = msg->data[0];

	if (component >= CB_COMPNT_MAX) {
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
	case CB_COMPNT_BIC:
		msg->data[0] = CB_COMPNT_BIC;
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
	case CB_COMPNT_PCIE_SWITCH0:
	case CB_COMPNT_PCIE_SWITCH1:
		/* Only can be read when DC is on */
		if (is_acb_power_good() == false) {
			msg->completion_code = CC_PEX_NOT_POWER_ON;
			return;
		}
		const uint8_t pex_sensor_num_table[PEX_MAX_NUMBER] = { SENSOR_NUM_TEMP_PEX_0,
								       SENSOR_NUM_TEMP_PEX_1 };
		int reading;

		uint8_t pex_sensor_num = pex_sensor_num_table[component - CB_COMPNT_PCIE_SWITCH0];
		sensor_cfg *cfg = &sensor_config[sensor_config_index_map[pex_sensor_num]];
		pex89000_unit *p = (pex89000_unit *)cfg->priv_data;

		if (cfg->pre_sensor_read_hook) {
			if (cfg->pre_sensor_read_hook(cfg, cfg->pre_sensor_read_args) == false) {
				LOG_ERR("PEX%d pre-read failed!",
					component - CB_COMPNT_PCIE_SWITCH0);
				msg->completion_code = CC_PEX_PRE_READING_FAIL;
				return;
			}
		}

		if (pex_access_engine(cfg->port, cfg->target_addr, p->idx, pex_access_sbr_ver,
				      &reading)) {
			if (cfg->post_sensor_read_hook) {
				if (cfg->post_sensor_read_hook(cfg, cfg->post_sensor_read_args,
							       NULL) == false) {
					LOG_ERR("PEX%d post-read failed!",
						component - CB_COMPNT_PCIE_SWITCH0);
				}
			}
			msg->completion_code = CC_PEX_ACCESS_FAIL;
			return;
		}

		if (cfg->post_sensor_read_hook) {
			if (cfg->post_sensor_read_hook(cfg, cfg->post_sensor_read_args, NULL) ==
			    false) {
				LOG_ERR("PEX%d post-read failed!",
					component - CB_COMPNT_PCIE_SWITCH0);
			}
		}

		memcpy(&msg->data[2], &reading, sizeof(reading));

		msg->data[0] = component;
		msg->data[1] = sizeof(reading);
		msg->data_len = sizeof(reading) + 2;
		msg->completion_code = CC_SUCCESS;
		break;
	case CB_COMPNT_VR_XDPE15284:
		if (is_acb_power_good() == false) {
			msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
			return;
		}

		switch (get_vr_module()) {
		case VR_XDPE15284D:
			if (xdpe15284_get_checksum(I2C_BUS1, XDPE15284D_ADDR, &msg->data[2]) ==
			    false) {
				msg->completion_code = CC_UNSPECIFIED_ERROR;
				return;
			}
			break;
		case VR_MP2985H: {
			sensor_cfg *vr_cfg =
				&sensor_config[sensor_config_index_map[SENSOR_NUM_TEMP_P0V8_VDD_1]];
			if (vr_cfg->pre_sensor_read_hook) {
				if (vr_cfg->pre_sensor_read_hook(
					    vr_cfg, vr_cfg->pre_sensor_read_args) == false) {
					LOG_ERR("VR MP2985H pre-read fail");
					msg->completion_code = CC_UNSPECIFIED_ERROR;
					return;
				}
			}
			if (mp2985_get_checksum(I2C_BUS1, XDPE15284D_ADDR, &msg->data[2]) ==
			    false) {
				msg->completion_code = CC_UNSPECIFIED_ERROR;
				if (vr_cfg->post_sensor_read_hook) {
					vr_cfg->post_sensor_read_hook(
						vr_cfg, vr_cfg->post_sensor_read_args, NULL);
				}
				return;
			}
			if (vr_cfg->post_sensor_read_hook) {
				if (vr_cfg->post_sensor_read_hook(
					    vr_cfg, vr_cfg->post_sensor_read_args, NULL) == false) {
					LOG_ERR("VR MP2985H post-read fail");
				}
			}
			break;
		}
		default:
			LOG_ERR("Unknown VR module: 0x%x, Fail to get VR version", get_vr_module());
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}

		msg->data[0] = component;
		msg->data[1] = VR_FW_VERSION_LEN;
		msg->data_len = VR_FW_VERSION_LEN + 2;
		msg->completion_code = CC_SUCCESS;
		break;
	case CB_COMPNT_ACCL1_CH1_FREYA:
	case CB_COMPNT_ACCL1_CH2_FREYA:
	case CB_COMPNT_ACCL2_CH1_FREYA:
	case CB_COMPNT_ACCL2_CH2_FREYA:
	case CB_COMPNT_ACCL3_CH1_FREYA:
	case CB_COMPNT_ACCL3_CH2_FREYA:
	case CB_COMPNT_ACCL4_CH1_FREYA:
	case CB_COMPNT_ACCL4_CH2_FREYA:
	case CB_COMPNT_ACCL5_CH1_FREYA:
	case CB_COMPNT_ACCL5_CH2_FREYA:
	case CB_COMPNT_ACCL6_CH1_FREYA:
	case CB_COMPNT_ACCL6_CH2_FREYA:
	case CB_COMPNT_ACCL7_CH1_FREYA:
	case CB_COMPNT_ACCL7_CH2_FREYA:
	case CB_COMPNT_ACCL8_CH1_FREYA:
	case CB_COMPNT_ACCL8_CH2_FREYA:
	case CB_COMPNT_ACCL9_CH1_FREYA:
	case CB_COMPNT_ACCL9_CH2_FREYA:
	case CB_COMPNT_ACCL10_CH1_FREYA:
	case CB_COMPNT_ACCL10_CH2_FREYA:
	case CB_COMPNT_ACCL11_CH1_FREYA:
	case CB_COMPNT_ACCL11_CH2_FREYA:
	case CB_COMPNT_ACCL12_CH1_FREYA:
	case CB_COMPNT_ACCL12_CH2_FREYA:
		if (is_acb_power_good() != true) {
			msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
			return;
		}

		uint8_t card_id = (component - CB_COMPNT_ACCL1_CH1_FREYA) / 2;
		uint8_t dev_id = (component - CB_COMPNT_ACCL1_CH1_FREYA) % 2;

		if (dev_id == FREYA_ID1) {
			if (asic_card_info[card_id].asic_1_status != ASIC_CARD_DEVICE_PRESENT) {
				msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
				return;
			}

			memcpy(&msg->data[2], &accl_freya_info[card_id].freya1_fw_info,
			       FREYA_FW_VERSION_LENGTH);
		} else {
			if (asic_card_info[card_id].asic_2_status != ASIC_CARD_DEVICE_PRESENT) {
				msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
				return;
			}

			memcpy(&msg->data[2], &accl_freya_info[card_id].freya2_fw_info,
			       FREYA_FW_VERSION_LENGTH);
		}

		msg->data[0] = component;
		msg->data[1] = FREYA_FW_VERSION_LENGTH;
		msg->data_len = FREYA_FW_VERSION_LENGTH + 2;
		msg->completion_code = CC_SUCCESS;
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
	*    Byte 1: Card type (0: freya, 1: Artemis module) */

	CHECK_NULL_ARG(msg);

	if (msg->data_len != 2) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	/* BMC would check pcie card type via fru id and device id */
	uint8_t fru_id = msg->data[0];
	uint8_t pcie_card_id = 0;
	uint8_t pcie_device_id = msg->data[1];

	switch (fru_id) {
	case FIO_FRU_ID:
		switch (pcie_device_id) {
		case PCIE_DEVICE_ID1:
			if (gpio_get(PRSNT_FIO_N) == LOW_ACTIVE) {
				msg->data[0] = FIO_PRESENT;
			} else {
				msg->data[0] = FIO_NOT_PRESENT;
			}
			msg->data[1] = RESERVE_DEFAULT_VALUE;
			msg->completion_code = CC_SUCCESS;
			break;
		default:
			msg->completion_code = CC_PARAM_OUT_OF_RANGE;
			return;
		}
		break;
	case ACCL_1_FRU_ID:
	case ACCL_2_FRU_ID:
	case ACCL_3_FRU_ID:
	case ACCL_4_FRU_ID:
	case ACCL_5_FRU_ID:
	case ACCL_6_FRU_ID:
	case ACCL_7_FRU_ID:
	case ACCL_8_FRU_ID:
	case ACCL_9_FRU_ID:
	case ACCL_10_FRU_ID:
	case ACCL_11_FRU_ID:
	case ACCL_12_FRU_ID:
		pcie_card_id = fru_id - PCIE_CARD_ID_OFFSET;

		switch (pcie_device_id) {
		case PCIE_DEVICE_ID1:
			msg->data[0] = asic_card_info[pcie_card_id].card_status;
			break;
		case PCIE_DEVICE_ID2:
			msg->data[0] = asic_card_info[pcie_card_id].asic_1_status;
			break;
		case PCIE_DEVICE_ID3:
			msg->data[0] = asic_card_info[pcie_card_id].asic_2_status;
			break;
		default:
			msg->completion_code = CC_PARAM_OUT_OF_RANGE;
			return;
		}
		msg->data[1] = asic_card_info[pcie_card_id].card_type;
		msg->completion_code = CC_SUCCESS;
		break;
	default:
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		break;
	}

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

	uint8_t target = msg->data[0];
	uint8_t status = -1;
	uint8_t pcie_switch_id = 0;
	uint32_t offset =
		((msg->data[4] << 24) | (msg->data[3] << 16) | (msg->data[2] << 8) | msg->data[1]);
	uint16_t length = ((msg->data[6] << 8) | msg->data[5]);

	if ((length == 0) || (length != msg->data_len - 7)) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	switch (target) {
	case CB_COMPNT_BIC:
	case (CB_COMPNT_BIC | IS_SECTOR_END_MASK):
		if (offset == 0) {
			// Set default fw update retry count at first package
			set_default_retry_count(FW_UPDATE_RETRY_MAX_COUNT);
		}

		status = fw_update(offset, length, &msg->data[7], (target & IS_SECTOR_END_MASK),
				   DEVSPI_FMC_CS0);
		break;
	case CB_COMPNT_PCIE_SWITCH0:
	case CB_COMPNT_PCIE_SWITCH1:
	case (CB_COMPNT_PCIE_SWITCH0 | IS_SECTOR_END_MASK):
	case (CB_COMPNT_PCIE_SWITCH1 | IS_SECTOR_END_MASK):
		/* Only can be update when DC is on */
		if (is_acb_power_good() == false) {
			msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
			return;
		}

		pcie_switch_id = (target & WITHOUT_SECTOR_END_MASK) - CB_COMPNT_PCIE_SWITCH0;
		gpio_set(pcie_switch_mux_info[pcie_switch_id].control_gpio,
			 pcie_switch_mux_info[pcie_switch_id].bic_to_flash_value);

		status = fw_update(offset, length, &msg->data[7], (target & IS_SECTOR_END_MASK),
				   pcie_switch_mux_info[pcie_switch_id].device);

		gpio_set(pcie_switch_mux_info[pcie_switch_id].control_gpio,
			 pcie_switch_mux_info[pcie_switch_id].sw_to_flash_value);
		break;
	default:
		LOG_ERR("target: 0x%x is invalid", target);
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
		LOG_ERR("firmware (0x%02X) update failed cc: %x", target, msg->completion_code);
	}

	return;
}

int pal_get_pcie_card_sensor_reading(uint8_t card_id, uint8_t sensor_num, uint8_t *card_status,
				     int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(card_status, -1);
	CHECK_NULL_ARG_WITH_RETURN(reading, -1);

	uint8_t sensor_status = 0;
	uint8_t cfg_count = 0;

	sensor_cfg *cfg = NULL;

	cfg = get_accl_sensor_cfg_info(card_id, &cfg_count);
	if (cfg == NULL) {
		LOG_ERR("Fail to find ACCL sensor config via card id: 0x%x", card_id);
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
		*card_status |= PCIE_CARD_NOT_ACCESSIBLE_BIT;
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
	uint8_t device_status = 0;
	uint8_t fru_id = msg->data[0];
	uint8_t sensor_num = msg->data[1];
	uint8_t card_id = fru_id - ACCL_1_FRU_ID;

	ret = pal_get_pcie_card_sensor_reading(card_id, sensor_num, &device_status, &reading);
	if (ret != 0) {
		LOG_ERR("Get pcie card sensor reading fail, card id: 0x%x, sensor num: 0x%x",
			card_id, sensor_num);
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	msg->data_len = 5;
	msg->data[0] = device_status;
	memcpy(&msg->data[1], &reading, sizeof(reading));
	msg->completion_code = CC_SUCCESS;
	return;
}

int pal_write_read_accl_fru(uint8_t fru_id, uint8_t option, EEPROM_ENTRY *fru_entry,
			    uint8_t *status)
{
	CHECK_NULL_ARG_WITH_RETURN(fru_entry, -1);
	CHECK_NULL_ARG_WITH_RETURN(status, -1);

	bool ret = 0;
	uint8_t retry = 0;
	uint8_t dev_id = 0;
	uint8_t card_id = 0;
	int mutex_status = 0;
	mux_config card_mux = { 0 };

	if (option != ACCL_FRU_READ && option != ACCL_FRU_WRITE) {
		LOG_ERR("Invalid accl fru option: 0x%x", option);
		return -1;
	}

	ret = pal_accl_fru_id_map_accl_id_dev_id(fru_id, &card_id, &dev_id);
	if (ret != true) {
		LOG_ERR("Invalid fru id: 0x%x to card id and dev id", fru_id);
		return -1;
	}

	ret = get_accl_mux_config(card_id, &card_mux);
	if (ret != true) {
		LOG_ERR("Invalid card id: 0x%x to get accl mux config", card_id);
		return -1;
	}

	struct k_mutex *mutex = get_i2c_mux_mutex(card_mux.bus);
	for (retry = 0; retry < 3; ++retry) {
		mutex_status = k_mutex_lock(mutex, K_MSEC(MUTEX_LOCK_INTERVAL_MS));
		if (mutex_status == 0) {
			break;
		}
	}

	if (retry >= 3) {
		LOG_ERR("Mutex lock fail, status: %d, fru id: 0x%x", mutex_status, fru_id);
		return -1;
	}

	ret = set_mux_channel(card_mux, MUTEX_LOCK_ENABLE);
	if (ret == false) {
		LOG_ERR("Switch card mux channel fail");
		k_mutex_unlock(mutex);
		return -1;
	}

	if (option == ACCL_FRU_READ) {
		*status = FRU_read(fru_entry);
	} else {
		*status = FRU_write(fru_entry);
	}

	mutex_status = k_mutex_unlock(mutex);
	if (mutex_status != 0) {
		LOG_ERR("Mutex unlock fail, status: %d", mutex_status);
	}

	return 0;
}

void STORAGE_READ_FRUID_DATA(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	int ret = 0;
	uint8_t status = 0;
	uint8_t fru_id = msg->data[0];
	uint8_t data_len = msg->data[3];
	EEPROM_ENTRY fru_entry = { 0 };

	if (msg->data_len != 4) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	// According to IPMI, messages are limited to 32 bytes
	if (data_len > 32) {
		msg->completion_code = CC_LENGTH_EXCEEDED;
		return;
	}

	fru_entry.config.dev_id = fru_id;
	fru_entry.offset = (msg->data[2] << 8) | msg->data[1];
	fru_entry.data_len = data_len;

	if (fru_id == CB_FRU_ID || fru_id == FIO_FRU_ID) {
		status = FRU_read(&fru_entry);
	} else {
		ret = pal_write_read_accl_fru(fru_id, ACCL_FRU_READ, &fru_entry, &status);
		if (ret != 0) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}
	}

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

	int ret = 0;
	uint8_t status = 0;
	uint8_t fru_id = msg->data[0];
	uint8_t data_len = msg->data_len - 3; // skip id and offset
	EEPROM_ENTRY fru_entry = { 0 };

	if (fru_id >= ACCL_1_CH1_FREYA_FRU_ID) {
		LOG_ERR("Not support ACCL freya fru write, fru id: 0x%x", fru_id);
		msg->completion_code = CC_INVALID_PARAM;
		return;
	}

	if (msg->data_len < 4) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	// According to IPMI, messages are limited to 32 bytes
	if (data_len > 32) {
		msg->completion_code = CC_LENGTH_EXCEEDED;
		return;
	}

	fru_entry.config.dev_id = fru_id;
	fru_entry.offset = (msg->data[2] << 8) | msg->data[1];
	fru_entry.data_len = data_len;
	memcpy(&fru_entry.data[0], &msg->data[3], fru_entry.data_len);

	if (fru_id == CB_FRU_ID || fru_id == FIO_FRU_ID) {
		status = FRU_write(&fru_entry);
	} else {
		ret = pal_write_read_accl_fru(fru_id, ACCL_FRU_WRITE, &fru_entry, &status);
		if (ret != 0) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}
	}

	msg->data[0] = data_len;
	msg->data_len = 1;

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

void OEM_1S_BRIDGE_I2C_MSG_BY_COMPNT(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len < 2) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	bool ret = false;
	uint8_t retry = 0;
	uint8_t fru_id = msg->data[0];
	uint8_t card_id = 0;
	uint8_t dev_id = 0;
	uint8_t address = 0;
	ipmi_msg bridge_msg = { 0 };
	mux_config card_mux = { 0 };

	ret = pal_accl_fru_id_map_accl_id_dev_id(fru_id, &card_id, &dev_id);
	if (ret != true) {
		LOG_ERR("Invalid fru id: 0x%x to card id and dev id", fru_id);
		msg->completion_code = CC_INVALID_PARAM;
		return;
	}

	switch (dev_id) {
	case PCIE_DEVICE_ID2:
	case PCIE_DEVICE_ID3:
		address = ACCL_ARTEMIS_MODULE_2_ADDR;
		break;
	default:
		LOG_ERR("Invalid device id: 0x%x", dev_id);
		msg->completion_code = CC_INVALID_PARAM;
		return;
	}

	ret = get_accl_mux_config(card_id, &card_mux);
	if (ret != true) {
		LOG_ERR("Invalid card id: 0x%x to get accl mux config", card_id);
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	int mutex_status = 0;
	struct k_mutex *mutex = get_i2c_mux_mutex(card_mux.bus);

	for (retry = 0; retry < 3; ++retry) {
		mutex_status = k_mutex_lock(mutex, K_MSEC(MUTEX_LOCK_INTERVAL_MS));
		if (mutex_status == 0) {
			break;
		}
	}

	if (retry >= 3) {
		LOG_ERR("Mutex lock fail, status: %d", mutex_status);
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	ret = set_mux_channel(card_mux, MUTEX_LOCK_ENABLE);
	if (ret == false) {
		LOG_ERR("ACCL switch mux fail, fru id: 0x%x", fru_id);
		k_mutex_unlock(mutex);
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	bridge_msg.data[0] = card_mux.bus << 1;
	bridge_msg.data[1] = address << 1;
	memcpy(&bridge_msg.data[2], &msg->data[1], msg->data_len - 1);
	bridge_msg.data_len = msg->data_len + 1;

	APP_MASTER_WRITE_READ(&bridge_msg);

	msg->completion_code = bridge_msg.completion_code;
	msg->data_len = bridge_msg.data_len;
	memcpy(&msg->data[0], &bridge_msg.data[0], bridge_msg.data_len);

	k_mutex_unlock(mutex);
	return;
}
