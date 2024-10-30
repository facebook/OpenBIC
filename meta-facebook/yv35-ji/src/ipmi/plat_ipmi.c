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
#include <drivers/flash.h>
#include "libutil.h"
#include "ipmi.h"
#include "eeprom.h"
#include "hal_gpio.h"
#include "oem_1s_handler.h"
#include "plat_fru.h"
#include "plat_class.h"
#include "plat_spi.h"
#include "util_spi.h"

LOG_MODULE_REGISTER(plat_ipmi);

#define WORK_AROUND_BIOS_DEBUG_PIN_IDX 62
#define WORK_AROUND_BIOS_DEBUG_PIN_IDX_REAL 23

#define VIRTUAL_E1S_PRSNT_PIN_IDX 63
#define VIRTUAL_E1S_PRSNT_PIN_IDX_REAL 55

#define VIRTUAL_RETIMER_PRSNT_PIN_IDX 64
#define VIRTUAL_RETIMER_PRSNT_PIN_IDX_REAL 58

#define VIRTUAL_SATMC_READY_PIN_IDX 65
#define VIRTUAL_SATMC_READY_PIN_IDX_REAL 59

#define CMET_INFO_OFFSET_1 0xc0000
#define CMET_INFO_OFFSET_2 0xd0000
#define CLEAR_BYTE_START_OFFSET_NUM 2
#define CLEAR_BYTE_TOTAL_NUM 4

bool pal_request_msg_to_BIC_from_HOST(uint8_t netfn, uint8_t cmd)
{
	if (netfn == NETFN_OEM_1S_REQ) {
		if ((cmd == CMD_OEM_1S_FW_UPDATE) || (cmd == CMD_OEM_1S_RESET_BMC) ||
		    (cmd == CMD_OEM_1S_GET_BIC_STATUS) || (cmd == CMD_OEM_1S_RESET_BIC) ||
		    (cmd == CMD_OEM_1S_GET_BIC_FW_INFO || cmd == CMD_OEM_1S_SEND_MCTP_PLDM_COMMAND))
			return true;
	} else if (netfn == NETFN_APP_REQ) {
		if (cmd == CMD_APP_GET_SYSTEM_GUID)
			return true;
	} else if (netfn == NETFN_DCMI_REQ) {
		if ((cmd == CMD_DCMI_SEND_BOOT_PROGRESS_CODE) ||
		    (cmd == CMD_DCMI_GET_BOOT_PROGRESS_CODE))
			return true;
	}

	return false;
}

bool pal_immediate_respond_from_HOST(uint8_t netfn, uint8_t cmd)
{
	if (netfn == NETFN_STORAGE_REQ) {
		if (cmd == CMD_STORAGE_ADD_SEL)
			return true;
	} else if (netfn == NETFN_SENSOR_REQ) {
		if (cmd == CMD_SENSOR_PLATFORM_EVENT)
			return true;
	} else if (netfn == NETFN_OEM_REQ) {
		if (cmd == CMD_OEM_POST_END)
			return true;
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

/* work around - Modify gpio index */
void OEM_1S_GET_GPIO(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	// only input enable status
	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t eight_bit_value = 0, gpio_value, gpio_cnt, data_len;
	// Bump up the gpio_ind_to_num_table_cnt to multiple of 8.
	gpio_cnt = gpio_ind_to_num_table_cnt + (8 - (gpio_ind_to_num_table_cnt % 8));
	data_len = gpio_cnt / 8;
	msg->data_len = data_len;

	int tmp_gpio_idx = 0;
	for (uint8_t i = 0; i < gpio_cnt; i++) {
		if (i == VIRTUAL_RETIMER_PRSNT_PIN_IDX)
			tmp_gpio_idx = VIRTUAL_RETIMER_PRSNT_PIN_IDX_REAL;
		else if (i == VIRTUAL_E1S_PRSNT_PIN_IDX)
			tmp_gpio_idx = VIRTUAL_E1S_PRSNT_PIN_IDX_REAL;
		else if (i == WORK_AROUND_BIOS_DEBUG_PIN_IDX)
			tmp_gpio_idx = WORK_AROUND_BIOS_DEBUG_PIN_IDX_REAL;
		else if (i == VIRTUAL_SATMC_READY_PIN_IDX)
			tmp_gpio_idx = VIRTUAL_SATMC_READY_PIN_IDX_REAL;
		else if (i >= WORK_AROUND_BIOS_DEBUG_PIN_IDX_REAL &&
			 i < (VIRTUAL_E1S_PRSNT_PIN_IDX_REAL - 1))
			tmp_gpio_idx = i + 1;
		else if (i >= (VIRTUAL_E1S_PRSNT_PIN_IDX_REAL - 1) &&
			 i < (VIRTUAL_RETIMER_PRSNT_PIN_IDX_REAL - 2))
			tmp_gpio_idx = i + 2;
		else if (i >= (VIRTUAL_RETIMER_PRSNT_PIN_IDX_REAL - 2) &&
			 i < WORK_AROUND_BIOS_DEBUG_PIN_IDX)
			tmp_gpio_idx = i + 4;
		else
			tmp_gpio_idx = i;

		gpio_value = (tmp_gpio_idx >= gpio_ind_to_num_table_cnt) ?
				     0 :
				     gpio_get(gpio_ind_to_num_table[tmp_gpio_idx]);

		// clear temporary variable to avoid return wrong GPIO value
		if (i % 8 == 0) {
			eight_bit_value = 0;
		}
		eight_bit_value = eight_bit_value | (gpio_value << (i % 8));
		msg->data[i / 8] = eight_bit_value;
	}
	msg->completion_code = CC_SUCCESS;

	return;
}

/* work around - Modify gpio index */
uint8_t gpio_idx_exchange(ipmi_msg *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, 1);

	if (msg->data_len < 2)
		return 1;

	int need_change = 1;
	switch (msg->data[0]) {
	case GET_GPIO_STATUS:
	case GET_GPIO_DIRECTION_STATUS:
		if (msg->data_len == 3) {
			if (msg->data[2] == GLOBAL_GPIO_IDX_KEY) {
				need_change = 0;
			}
			// Ignore last data byte if provided.
			msg->data_len--;
		}
		break;
	case SET_GPIO_OUTPUT_STATUS:
	case SET_GPIO_DIRECTION_STATUS:
		if (msg->data_len == 4) {
			if (msg->data[3] == GLOBAL_GPIO_IDX_KEY) {
				need_change = 0;
			}
			// Ignore last data byte if provided.
			msg->data_len--;
		}
		break;
	default:
		break;
	}

	if (need_change) {
		if (msg->data[1] == VIRTUAL_RETIMER_PRSNT_PIN_IDX)
			msg->data[1] = VIRTUAL_RETIMER_PRSNT_PIN_IDX_REAL;
		else if (msg->data[1] == VIRTUAL_E1S_PRSNT_PIN_IDX)
			msg->data[1] = VIRTUAL_E1S_PRSNT_PIN_IDX_REAL;
		else if (msg->data[1] == WORK_AROUND_BIOS_DEBUG_PIN_IDX)
			msg->data[1] = WORK_AROUND_BIOS_DEBUG_PIN_IDX_REAL;
		else if (msg->data[1] == VIRTUAL_SATMC_READY_PIN_IDX)
			msg->data[1] = VIRTUAL_SATMC_READY_PIN_IDX_REAL;
		else if (msg->data[1] >= WORK_AROUND_BIOS_DEBUG_PIN_IDX_REAL &&
			 msg->data[1] < (VIRTUAL_E1S_PRSNT_PIN_IDX_REAL - 1))
			msg->data[1]++;
		else if (msg->data[1] >= (VIRTUAL_E1S_PRSNT_PIN_IDX_REAL - 1) &&
			 msg->data[1] < (VIRTUAL_RETIMER_PRSNT_PIN_IDX_REAL - 2))
			msg->data[1] += 2;
		else if (msg->data[1] >= (VIRTUAL_RETIMER_PRSNT_PIN_IDX_REAL - 2) &&
			 msg->data[1] < WORK_AROUND_BIOS_DEBUG_PIN_IDX)
			msg->data[1] += 4;

		msg->data[1] = gpio_ind_to_num_table[msg->data[1]];
	}

	return 0;
}

void OEM_1S_CLEAR_CMET(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	const struct device *flash_dev;
	uint8_t loop_index = 0, loop_index2 = 0;
	uint8_t *op_buf = NULL, *read_back_buf = NULL;
	int32_t ret = 0;
	uint32_t start_offset = 0;
	msg->completion_code = CC_UNSPECIFIED_ERROR;

	//Need to switch mux first(Bios->read write flash)
	ret = switch_bios_read_write_flash_mux(SWITCH_MUX_TO_READ_WRITE_FLASH);
	if (ret != 0) {
		LOG_ERR("Failed to switch mux, ret %d.", ret);
		return;
	}

	//Need to reinit flash if flash not init before
	flash_dev = device_get_binding("spi1_cs0");
	if (flash_dev == NULL) {
		LOG_ERR("Failed to get device.");
		goto end;
	}
	ret = ckeck_flash_device_isinit(flash_dev, DEVSPI_SPI1_CS0);
	if (ret != 0) {
		LOG_ERR("Failed to re-init flash, ret %d.", ret);
		goto end;
	}

	//Clear CMET offset C0000/D0000 first 4 bytes to 0xff
	uint32_t flash_sz = flash_get_flash_size(flash_dev);
	uint32_t sector_sz = flash_get_write_block_size(flash_dev);

	op_buf = (uint8_t *)malloc(sector_sz);
	if (op_buf == NULL) {
		LOG_ERR("Failed to allocate op_buf.");
		goto end;
	}

	read_back_buf = (uint8_t *)malloc(sector_sz);
	if (read_back_buf == NULL) {
		LOG_ERR("Failed to allocate read_back_buf.");
		goto end;
	}

	for (loop_index = 0; loop_index < CLEAR_BYTE_START_OFFSET_NUM; loop_index++) {
		if (loop_index == 0) {
			start_offset = CMET_INFO_OFFSET_1;
		} else if (loop_index == 1) {
			start_offset = CMET_INFO_OFFSET_2;
		}

		if (flash_sz < start_offset + sector_sz) {
			LOG_ERR("Update boundary exceeds flash size. (%u, %u, %u)", flash_sz,
				start_offset, sector_sz);
			goto end;
		}

		//Read data back from flash
		ret = flash_read(flash_dev, start_offset, read_back_buf, sector_sz);
		if (ret != 0) {
			LOG_ERR("Failed to read %u.", start_offset);
			goto end;
		}

		//Set first 4 byte to 0xff
		for (loop_index2 = 0; loop_index2 < CLEAR_BYTE_TOTAL_NUM; loop_index2++) {
			read_back_buf[loop_index2] = 0xff;
		}

		memcpy(op_buf, read_back_buf, sector_sz);
		memset(read_back_buf, 0, sector_sz);

		//do erase write and verify
		ret = flash_erase(flash_dev, start_offset, sector_sz);
		if (ret != 0) {
			LOG_ERR("Failed to erase %u.", start_offset);
			goto end;
		}

		ret = flash_write(flash_dev, start_offset, op_buf, sector_sz);
		if (ret != 0) {
			LOG_ERR("Failed to write %u.", start_offset);
			goto end;
		}

		ret = flash_read(flash_dev, start_offset, read_back_buf, sector_sz);
		if (ret != 0) {
			LOG_ERR("Failed to read %u.", start_offset);
			goto end;
		}

		if (memcmp(op_buf, read_back_buf, sector_sz) != 0) {
			LOG_ERR("Failed to write flash at 0x%x.", start_offset);
			LOG_HEXDUMP_ERR(op_buf, sector_sz, "to be written:");
			LOG_HEXDUMP_ERR(read_back_buf, sector_sz, "readback:");
			goto end;
		}

		memset(op_buf, 0, sector_sz);
		memset(read_back_buf, 0, sector_sz);
	}

	msg->completion_code = CC_SUCCESS;

end:
	SAFE_FREE(op_buf);
	SAFE_FREE(read_back_buf);
	ret = switch_bios_read_write_flash_mux(SWITCH_MUX_TO_BIOS);
	if (ret != 0) {
		LOG_ERR("Failed to switch mux, ret %d.", ret);
	}

	return;
}
