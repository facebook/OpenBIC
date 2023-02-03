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

#include "oem_1s_handler.h"
#include <stdlib.h>
#include <drivers/peci.h>
#include "libutil.h"
#include "ipmb.h"
#include "sensor.h"
#include "snoop.h"
#include "pmic.h"
#include "hal_gpio.h"
#include "hal_i2c.h"
#include "hal_jtag.h"
#include "hal_peci.h"
#include "plat_def.h"
#include "plat_gpio.h"
#include "plat_i2c.h"
#include "plat_ipmi.h"
#include "plat_sensor_table.h"
#include "plat_sys.h"
#ifdef ENABLE_FAN
#include "plat_fan.h"
#endif
#include "plat_ipmb.h"
#include "power_status.h"
#include "pmbus.h"
#include "altera.h"
#include "util_spi.h"
#include "util_sys.h"
#include <logging/log.h>
#ifdef ENABLE_APML
#include "plat_apml.h"
#endif
#include "pcc.h"
#include "hal_wdt.h"

#define BIOS_UPDATE_MAX_OFFSET 0x4000000
#define BIC_UPDATE_MAX_OFFSET 0x50000

#define _4BYTE_ACCURACY_SENSOR_READING_RES_LEN 5
#define MAX_MULTI_ACCURACY_SENSOR_READING_QUERY_NUM 32
#define MAX_CONTROL_SENSOR_POLLING_COUNT 10
#define FOUR_BYTE_POST_CODE_PAGE_SIZE 60

LOG_MODULE_DECLARE(ipmi);

__weak int pal_extend_msg_out_interface_handler(ipmi_msg *msg)
{
	return -1;
}

__weak void OEM_1S_MSG_OUT(ipmi_msg *msg)
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

	target_IF = msg->data[0];

	if (target_IF == PEER_BMC_IPMB) {
		if (msg->InF_source == SLOT1_BIC) {
			target_IF = msg->data[0] = SLOT3_BIC;
		} else if (msg->InF_source == SLOT3_BIC) {
			target_IF = msg->data[0] = SLOT1_BIC;
		} else {
			msg->completion_code = CC_INVALID_DATA_FIELD;
		}

		if (msg->data[1] == NETFN_STORAGE_REQ) {
			msg->data[1] = msg->data[1] << 2;
		}
	}

	// Bridge to invalid or disabled interface
	if ((IPMB_inf_index_map[target_IF] == RESERVED) ||
	    (IPMB_config_table[IPMB_inf_index_map[target_IF]].interface == RESERVED_IF) ||
	    (IPMB_config_table[IPMB_inf_index_map[target_IF]].enable_status == DISABLE)) {
		// check the platform extend message out handler
		if (!pal_extend_msg_out_interface_handler(msg)) {
			// the platform has the extended handler
			// the ipmb response should be handled by itself
			return;
		}
		LOG_ERR("OEM_MSG_OUT: Invalid bridge interface: %x", target_IF);
		msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
	}

	// only send to target while msg is valid
	if (msg->completion_code == CC_SUCCESS) {
		bridge_msg = (ipmi_msg *)malloc(sizeof(ipmi_msg));
		if (bridge_msg == NULL) {
			msg->completion_code = CC_OUT_OF_SPACE;
		} else {
			memset(bridge_msg, 0, sizeof(ipmi_msg));

			LOG_DBG("bridge targetIf %x, len %d, netfn %x, cmd %x", target_IF,
				msg->data_len, msg->data[1] >> 2, msg->data[2]);

			bridge_msg->data_len = msg->data_len - 3;
			bridge_msg->seq_source = msg->seq_source;
			bridge_msg->InF_target = msg->data[0];
			bridge_msg->InF_source = msg->InF_source;
			bridge_msg->netfn = msg->data[1] >> 2;
			bridge_msg->cmd = msg->data[2];
			bridge_msg->pldm_inst_id = msg->pldm_inst_id;

			if (bridge_msg->data_len != 0) {
				memcpy(&bridge_msg->data[0], &msg->data[3],
				       bridge_msg->data_len * sizeof(msg->data[0]));
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

__weak void OEM_1S_GET_GPIO(ipmi_msg *msg)
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
	for (uint8_t i = 0; i < gpio_cnt; i++) {
		gpio_value =
			(i >= gpio_ind_to_num_table_cnt) ? 0 : gpio_get(gpio_ind_to_num_table[i]);

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

__weak void OEM_1S_GET_GPIO_CONFIG(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	msg->data_len = 0;
	msg->completion_code = CC_INVALID_CMD;
	return;
}

__weak void OEM_1S_SET_GPIO_CONFIG(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	msg->data_len = 0;
	msg->completion_code = CC_INVALID_CMD;
	return;
}

__weak void OEM_1S_FW_UPDATE(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	/*********************************
	* Request Data
	*
	* Byte   0: [6:0] fw update target, [7] indicate last packet
	* Byte 1-4: offset, lsb first
	* Byte 5-6: length, lsb first
	* Byte 7-N: data
	***********************************/
	if (msg->data_len < 8) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t target = msg->data[0];
	uint8_t status = -1;
	uint32_t offset =
		((msg->data[4] << 24) | (msg->data[3] << 16) | (msg->data[2] << 8) | msg->data[1]);
	uint16_t length = ((msg->data[6] << 8) | msg->data[5]);

	if ((length == 0) || (length != msg->data_len - 7)) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	if (target == BIOS_UPDATE || (target == (BIOS_UPDATE | IS_SECTOR_END_MASK))) {
		// BIOS size maximum 64M bytes
		if (offset > BIOS_UPDATE_MAX_OFFSET) {
			msg->completion_code = CC_PARAM_OUT_OF_RANGE;
			return;
		}
		int pos = pal_get_bios_flash_position();
		if (pos == -1) {
			msg->completion_code = CC_INVALID_PARAM;
			return;
		}

		// Switch GPIO(BIOS SPI Selection Pin) to BIC
		bool ret = pal_switch_bios_spi_mux(GPIO_HIGH);
		if (!ret) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}

		status = fw_update(offset, length, &msg->data[7], (target & IS_SECTOR_END_MASK),
				   pos);

		// Switch GPIO(BIOS SPI Selection Pin) to PCH
		ret = pal_switch_bios_spi_mux(GPIO_LOW);
		if (!ret) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}

	} else if ((target == BIC_UPDATE) || (target == (BIC_UPDATE | IS_SECTOR_END_MASK))) {
		// Expect BIC firmware size not bigger than 320k
		if (offset > BIC_UPDATE_MAX_OFFSET) {
			msg->completion_code = CC_PARAM_OUT_OF_RANGE;
			return;
		}
		status = fw_update(offset, length, &msg->data[7], (target & IS_SECTOR_END_MASK),
				   DEVSPI_FMC_CS0);

	} else if ((target == CPLD_UPDATE) || (target == (CPLD_UPDATE | IS_SECTOR_END_MASK))) {
		status = cpld_altera_max10_fw_update(offset, length, &msg->data[7]);

	} else if (target == CXL_UPDATE || (target == (CXL_UPDATE | IS_SECTOR_END_MASK))) {
		status =
			fw_update_cxl(offset, length, &msg->data[7], (target & IS_SECTOR_END_MASK));

	} else if (target == PRoT_FLASH_UPDATE ||
		   (target == (PRoT_FLASH_UPDATE | IS_SECTOR_END_MASK))) {
		if (offset > BIOS_UPDATE_MAX_OFFSET) {
			msg->completion_code = CC_PARAM_OUT_OF_RANGE;
			return;
		}

		int pos = pal_get_prot_flash_position();
		if (pos == -1) {
			msg->completion_code = CC_INVALID_PARAM;
			return;
		}

		status = fw_update(offset, length, &msg->data[7], (target & IS_SECTOR_END_MASK),
				   pos);

	} else {
		msg->completion_code = CC_INVALID_DATA_FIELD;
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
	case FWUPDATE_NOT_SUPPORT:
		msg->completion_code = CC_INVALID_PARAM;
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

__weak void OEM_1S_GET_BIC_FW_INFO(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	msg->data_len = 0;

	uint8_t component;
	component = msg->data[0];
	switch (component) {
	case BIC_PLAT_NAME:
		msg->data_len = strlen(PLATFORM_NAME);
		memcpy(&msg->data[0], PLATFORM_NAME, msg->data_len);
		break;

	case BIC_PLAT_BOARD_ID:
		msg->data_len = 1;
		msg->data[0] = BOARD_ID;
		break;

	case BIC_PROJ_NAME:
		msg->data_len = strlen(PROJECT_NAME);
		memcpy(&msg->data[0], PROJECT_NAME, msg->data_len);
		break;

	case BIC_PROJ_STAGE:
		msg->data_len = 1;
		msg->data[0] = PROJECT_STAGE;
		break;

	default:
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	msg->completion_code = CC_SUCCESS;
	return;
}

__weak void OEM_1S_GET_FW_VERSION(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t component;
	component = msg->data[0];
#if MAX_IPMB_IDX
	ipmb_error status;
	ipmi_msg *bridge_msg;
#endif

#ifdef ENABLE_ISL69260
	I2C_MSG i2c_msg;
	uint8_t retry = 3;
#endif

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
#if MAX_IPMB_IDX
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
#endif
#ifdef ENABLE_ISL69260
	case COMPNT_PVCCIN:
	case COMPNT_PVCCFA_EHV_FIVRA:
	case COMPNT_PVCCD_HV:
	case COMPNT_PVCCINFAON:
	case COMPNT_PVCCFA_EHV:
		if ((component == COMPNT_PVCCIN) || (component == COMPNT_PVCCFA_EHV_FIVRA)) {
			i2c_msg.target_addr = PVCCIN_ADDR;
		}
		if (component == COMPNT_PVCCD_HV) {
			i2c_msg.target_addr = PVCCD_HV_ADDR;
		}
		if ((component == COMPNT_PVCCINFAON) || (component == COMPNT_PVCCFA_EHV)) {
			i2c_msg.target_addr = PVCCFA_EHV_ADDR;
		}
		i2c_msg.tx_len = 1;
		i2c_msg.rx_len = 7;
		i2c_msg.bus = I2C_BUS5;
		i2c_msg.data[0] = PMBUS_IC_DEVICE_ID;

		if (i2c_master_read(&i2c_msg, retry)) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}

		if (i2c_msg.data[0] == 0x04 && i2c_msg.data[1] == 0x00 && i2c_msg.data[2] == 0x81 &&
		    i2c_msg.data[3] == 0xD2 && i2c_msg.data[4] == 0x49) {
			/* Renesas isl69259 */
			i2c_msg.tx_len = 3;
			i2c_msg.data[0] = 0xC7;
			i2c_msg.data[1] = 0x94;
			i2c_msg.data[2] = 0x00;

			if (i2c_master_write(&i2c_msg, retry)) {
				msg->completion_code = CC_UNSPECIFIED_ERROR;
				return;
			}

			i2c_msg.tx_len = 1;
			i2c_msg.rx_len = 4;
			i2c_msg.data[0] = 0xC5;

			if (i2c_master_read(&i2c_msg, retry)) {
				msg->completion_code = CC_UNSPECIFIED_ERROR;
				return;
			}

			msg->data[0] = i2c_msg.data[3];
			msg->data[1] = i2c_msg.data[2];
			msg->data[2] = i2c_msg.data[1];
			msg->data[3] = i2c_msg.data[0];
			msg->data_len = 4;
			msg->completion_code = CC_SUCCESS;

		} else if (i2c_msg.data[0] == 0x06 && i2c_msg.data[1] == 0x54 &&
			   i2c_msg.data[2] == 0x49 && i2c_msg.data[3] == 0x53 &&
			   i2c_msg.data[4] == 0x68 && i2c_msg.data[5] == 0x90 &&
			   i2c_msg.data[6] == 0x00) {
			/* TI tps53689 */
			i2c_msg.tx_len = 1;
			i2c_msg.rx_len = 2;
			i2c_msg.data[0] = 0xF4;

			if (i2c_master_read(&i2c_msg, retry)) {
				msg->completion_code = CC_UNSPECIFIED_ERROR;
				return;
			}

			msg->data[0] = i2c_msg.data[1];
			msg->data[1] = i2c_msg.data[0];
			msg->data_len = 2;
			msg->completion_code = CC_SUCCESS;

		} else if (i2c_msg.data[0] == 0x02 && i2c_msg.data[2] == 0x8A) {
			/* Infineon xdpe15284 */
			i2c_msg.tx_len = 6;
			i2c_msg.data[0] = 0xFD;
			i2c_msg.data[1] = 0x04;
			i2c_msg.data[2] = 0x00;
			i2c_msg.data[3] = 0x00;
			i2c_msg.data[4] = 0x00;
			i2c_msg.data[5] = 0x00;

			if (i2c_master_write(&i2c_msg, retry)) {
				msg->completion_code = CC_UNSPECIFIED_ERROR;
				return;
			}

			i2c_msg.tx_len = 2;
			i2c_msg.data[0] = 0xFE;
			i2c_msg.data[1] = 0x2D;

			if (i2c_master_write(&i2c_msg, retry)) {
				msg->completion_code = CC_UNSPECIFIED_ERROR;
				return;
			}

			i2c_msg.tx_len = 1;
			i2c_msg.rx_len = 5;
			i2c_msg.data[0] = 0xFD;

			if (i2c_master_read(&i2c_msg, retry)) {
				msg->completion_code = CC_UNSPECIFIED_ERROR;
				return;
			}

			msg->data[0] = i2c_msg.data[4];
			msg->data[1] = i2c_msg.data[3];
			msg->data[2] = i2c_msg.data[2];
			msg->data[3] = i2c_msg.data[1];
			msg->data_len = 4;
			msg->completion_code = CC_SUCCESS;
		} else {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
		}
		break;
#endif
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}
	return;
}

__weak void OEM_1S_RESET_BMC(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	int ret = pal_submit_bmc_cold_reset();
	if (ret == -1) {
		msg->completion_code = CC_INVALID_CMD;
	} else {
		msg->completion_code = CC_SUCCESS;
	}

	msg->data_len = 0;
	return;
}

__weak void OEM_1S_READ_FW_IMAGE(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 6) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	if (get_DC_status()) {
		msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
		return;
	}

	uint8_t target = msg->data[0];
	uint32_t offset =
		((msg->data[4] << 24) | (msg->data[3] << 16) | (msg->data[2] << 8) | msg->data[1]);
	uint8_t length = msg->data[5];

	if (target == BIOS_UPDATE) {
		if (!pal_switch_bios_spi_mux(GPIO_HIGH)) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}

		int pos = pal_get_bios_flash_position();
		if (pos == -1) {
			msg->completion_code = CC_INVALID_PARAM;
		} else {
			if (read_fw_image(offset, length, msg->data, pos)) {
				msg->completion_code = CC_UNSPECIFIED_ERROR;
			} else {
				msg->data_len = length;
				msg->completion_code = CC_SUCCESS;
			}
		}

		if (!pal_switch_bios_spi_mux(GPIO_LOW)) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}
	} else if (target == PRoT_FLASH_UPDATE) {
		int pos = pal_get_prot_flash_position();

		if (pos == -1) {
			msg->completion_code = CC_INVALID_PARAM;
		} else {
			if (read_fw_image(offset, length, msg->data, pos)) {
				msg->completion_code = CC_UNSPECIFIED_ERROR;
			} else {
				msg->data_len = length;
				msg->completion_code = CC_SUCCESS;
			}
		}
	} else {
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}
	return;
}

__weak void OEM_1S_SET_WDT_FEED(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	if ((msg->data[0] != 0) && (msg->data[0] != 1)) {
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	set_wdt_continue_feed(msg->data[0]);
	msg->data_len = 0;
	msg->completion_code = CC_SUCCESS;
	return;
}

#ifdef CONFIG_SNOOP_ASPEED
__weak void OEM_1S_GET_POST_CODE(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	int postcode_num = snoop_read_num;
	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	if (postcode_num) {
		uint8_t offset = 0;
		if (snoop_read_num > SNOOP_MAX_LEN) {
			postcode_num = SNOOP_MAX_LEN;
			offset = snoop_read_num % SNOOP_MAX_LEN;
		}
		copy_snoop_read_buffer(offset, postcode_num, msg->data, COPY_ALL_POSTCODE);
	}

	for (int i = 0; i < (postcode_num / 2); ++i) {
		uint8_t tmp = msg->data[i];
		msg->data[i] = msg->data[postcode_num - i - 1];
		msg->data[postcode_num - i - 1] = tmp;
	}

	msg->data_len = postcode_num;
	msg->completion_code = CC_SUCCESS;
	return;
}
#endif

#ifdef CONFIG_PCC_ASPEED
__weak void OEM_1S_GET_4BYTE_POST_CODE(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t page = msg->data[0];
	if ((page > 17)) {
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}
	uint16_t read_len, start_idx;
	start_idx = page * FOUR_BYTE_POST_CODE_PAGE_SIZE;
	read_len = copy_pcc_read_buffer(start_idx, FOUR_BYTE_POST_CODE_PAGE_SIZE, msg->data,
					IPMI_MSG_MAX_LENGTH);

	msg->data_len = read_len & 0xFF;
	msg->completion_code = CC_SUCCESS;
	return;
}
#endif

#ifdef CONFIG_PECI
__weak void OEM_1S_PECI_ACCESS(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	uint8_t addr, cmd, *writeBuf, *readBuf;
	uint8_t writeLen, readLen;
	int ret;

	if (msg->data_len < 3) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	addr = msg->data[0];
	writeLen = msg->data[1];
	readLen = msg->data[2];
	cmd = msg->data[3];

	// PECI driver would add 1 byte to check that data writing to host is correct
	// so input data len would one less then input writelen in write command.
	if ((msg->data_len - 3 != writeLen) && (msg->data_len - 3 != writeLen - 1)) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	if (msg->data_len == 3) {
		if ((writeLen == 0) && (readLen == 0)) {
			ret = peci_ping(addr);
			msg->data[0] = ret;
			msg->data_len = 1;
			msg->completion_code = CC_SUCCESS;
			return;
		} else {
			msg->completion_code = CC_INVALID_LENGTH;
			return;
		}
	} else {
		if ((cmd == PECI_GET_DIB_CMD) || (cmd == PECI_GET_TEMP0_CMD)) {
			readBuf = (uint8_t *)malloc(sizeof(uint8_t) * (readLen + 1));
		} else {
			readBuf = (uint8_t *)malloc(sizeof(uint8_t) * readLen);
		}
		writeBuf = (uint8_t *)malloc(sizeof(uint8_t) * writeLen);
		if ((readBuf == NULL) || (writeBuf == NULL)) {
			LOG_ERR("PECI access util buffer alloc fail");
			SAFE_FREE(writeBuf);
			SAFE_FREE(readBuf);
			msg->completion_code = CC_OUT_OF_SPACE;
			return;
		}
		memcpy(&writeBuf[0], &msg->data[4], writeLen);

		ret = peci_write(cmd, addr, readLen, readBuf, writeLen, writeBuf);
		if (ret) {
			SAFE_FREE(writeBuf);
			SAFE_FREE(readBuf);
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}
		memcpy(&msg->data[0], &readBuf[0], readLen);

		SAFE_FREE(writeBuf);
		SAFE_FREE(readBuf);
		msg->data_len = readLen;
		msg->completion_code = CC_SUCCESS;
		return;
	}
}
#endif

#ifdef CONFIG_JTAG
__weak void OEM_1S_SET_JTAG_TAP_STA(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 2) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}
	uint8_t tapbitlen, tapdata;

	tapbitlen = msg->data[0];
	tapdata = msg->data[1];
	jtag_set_tap(tapdata, tapbitlen);

	msg->data_len = 0;
	msg->completion_code = CC_SUCCESS;
	return;
}

__weak void OEM_1S_JTAG_DATA_SHIFT(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	uint8_t lastidx;
	uint16_t writebitlen, readbitlen, readbyte, databyte;

	writebitlen = (msg->data[1] << 8) | msg->data[0];
	databyte = (writebitlen + 7) >> 3;
	readbitlen = (msg->data[3 + databyte] << 8) | msg->data[2 + databyte];
	readbyte = (readbitlen + 7) >> 3;
	lastidx = msg->data[4 + databyte];

	if (msg->data_len != (5 + databyte)) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t shiftdata[databyte], receivedata[readbyte];
	memset(shiftdata, 0, databyte);
	memset(receivedata, 0, readbyte);
	memcpy(shiftdata, &msg->data[2], databyte);
	jtag_shift_data(writebitlen, shiftdata, readbitlen, receivedata, lastidx);

	memcpy(&msg->data[0], &receivedata[0], readbyte);
	msg->data_len = readbyte;
	msg->completion_code = CC_SUCCESS;
	return;
}

#ifdef ENABLE_ASD
__weak void OEM_1S_ASD_INIT(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	if (msg->data[0] == 0x01) {
		enable_PRDY_interrupt();
	} else if (msg->data[0] == 0xff) {
		disable_PRDY_interrupt();
	} else {
		disable_PRDY_interrupt();
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	msg->data_len = 0;
	msg->completion_code = CC_SUCCESS;
	return;
}
#endif
#endif

__weak void OEM_1S_SENSOR_POLL_EN(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	if (msg->data[0] == 1) {
		enable_sensor_poll();
	} else if (msg->data[0] == 0) {
		disable_sensor_poll();
	} else {
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	msg->data_len = 0;
	msg->completion_code = CC_SUCCESS;
	return;
}

__weak void OEM_1S_ACCURACY_SENSOR_READING(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	ACCURACY_SENSOR_READING_REQ *req = (ACCURACY_SENSOR_READING_REQ *)msg->data;
	ACCURACY_SENSOR_READING_RES *res = (ACCURACY_SENSOR_READING_RES *)msg->data;
	uint8_t status = -1, sensor_report_status;
	int reading;
	if (msg->data_len != 2) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	// following IPMI sensor status response
	if (enable_sensor_poll_thread) {
		sensor_report_status = SENSOR_EVENT_MESSAGES_ENABLE | SENSOR_SCANNING_ENABLE;
	} else {
		sensor_report_status = SENSOR_EVENT_MESSAGES_ENABLE;
	}

	if (req->read_option == GET_FROM_CACHE) {
		if (enable_sensor_poll_thread) {
			status = get_sensor_reading(req->sensor_num, &reading, GET_FROM_CACHE);
		} else {
			status = SENSOR_POLLING_DISABLE;
		}
	} else if (req->read_option == GET_FROM_SENSOR) {
		status = get_sensor_reading(req->sensor_num, &reading, GET_FROM_SENSOR);
	} else {
		LOG_ERR("Error: read_option was not either GET_FROM_CACHE or GET_FROM_SENSOR.");
		status = SENSOR_UNSPECIFIED_ERROR;
	}
	switch (status) {
	case SENSOR_READ_SUCCESS:
	case SENSOR_READ_ACUR_SUCCESS:
		res->decimal = (int16_t)reading;
		if (reading < 0) {
			res->fraction = (uint16_t)((res->decimal - reading + 0.0005) * 1000);
		} else {
			res->fraction = (uint16_t)((reading - res->decimal + 0.0005) * 1000);
		}
		msg->data[3] = sensor_report_status;
		msg->data_len = 4;
		msg->completion_code = CC_SUCCESS;
		break;
	case SENSOR_READ_4BYTE_ACUR_SUCCESS:
		memcpy(msg->data, &reading, sizeof(reading));
		msg->data[4] = sensor_report_status;
		msg->data_len = 5;
		msg->completion_code = CC_SUCCESS;
		break;
	case SENSOR_NOT_ACCESSIBLE:
	case SENSOR_INIT_STATUS:
	case SENSOR_NOT_PRESENT:
		res->decimal = 0;
		res->fraction = 0;
		// notice BMC about sensor temporary in not accessible status
		msg->data[4] = (sensor_report_status | SENSOR_READING_STATE_UNAVAILABLE);
		msg->data_len = 5;
		msg->completion_code = CC_SUCCESS;
		break;
	case SENSOR_POLLING_DISABLE:
		// getting sensor cache while sensor polling disable
		msg->completion_code = CC_SENSOR_NOT_PRESENT;
		break;
	case SENSOR_FAIL_TO_ACCESS:
		// transaction error
		msg->completion_code = CC_NODE_BUSY;
		break;
	case SENSOR_NOT_FOUND:
		// request sensor number not found
		msg->completion_code = CC_INVALID_DATA_FIELD;
		break;
	case SENSOR_UNSPECIFIED_ERROR:
	default:
		// unknown error
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}
	return;
}

/* For command Get Set GPIO NetFn:0x38 Cmd:0x41 */
__weak uint8_t gpio_idx_exchange(ipmi_msg *msg)
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

	if (need_change)
		msg->data[1] = gpio_ind_to_num_table[msg->data[1]];
	return 0;
}

__weak void OEM_1S_GET_SET_GPIO(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	// whether need to change gpio index type
	if (gpio_idx_exchange(msg)) {
		msg->data_len = 0;
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		return;
	}

	uint8_t gpio_num = msg->data[1];
	if (gpio_num >= TOTAL_GPIO_NUM) {
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	if (gpio_cfg[gpio_num].is_init == DISABLE) {
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	uint8_t completion_code = CC_INVALID_LENGTH;
	switch (msg->data[0]) {
	case GET_GPIO_STATUS:
		if (msg->data_len == 2) {
			msg->data[0] = gpio_num;
			msg->data[1] = gpio_get(gpio_num);
			completion_code = CC_SUCCESS;
		}
		break;
	case SET_GPIO_OUTPUT_STATUS:
		if (msg->data_len == 3) {
			msg->data[0] = gpio_num;
			gpio_set(gpio_num, msg->data[2]);
			msg->data[1] = gpio_get(gpio_num);
			completion_code = CC_SUCCESS;
		}
		break;
	case GET_GPIO_DIRECTION_STATUS:
		if (msg->data_len == 2) {
			msg->data[0] = gpio_num;
			msg->data[1] = gpio_get_direction(gpio_num);
			completion_code = CC_SUCCESS;
		}
		break;
	case SET_GPIO_DIRECTION_STATUS:
		if (msg->data_len == 3) {
			if (msg->data[2]) {
				gpio_conf(gpio_num, GPIO_OUTPUT);
			} else {
				gpio_conf(gpio_num, GPIO_INPUT);
			}
			msg->data[0] = gpio_num;
			msg->data[1] = msg->data[2];
			completion_code = CC_SUCCESS;
		}
		break;
	default:
		LOG_ERR("Unknown options(0x%x)", msg->data[0]);
		return;
	}

	if (completion_code != CC_SUCCESS) {
		msg->data_len = 0;
	} else {
		msg->data_len = 2; // Return GPIO number, status
	}
	msg->completion_code = completion_code;
	return;
}

__weak void OEM_1S_CONTROL_SENSOR_POLLING(ipmi_msg *msg)
{
	/***************************************************
	Request:
	Data 0 - total sensor count to be polling controlled
	Data 1 - control operation [ 0 is disable, 1 is enable ]
	Data 2:N - sensor number list to be polling controlled
	Note: Sensor count should be less than or equal to 10

	Response:
	Data 0:N - [sensor numer] [status]
	***************************************************/

	CHECK_NULL_ARG(msg);

	uint8_t total_sensor_count = msg->data[0];
	uint8_t operation = msg->data[1];
	if (total_sensor_count > MAX_CONTROL_SENSOR_POLLING_COUNT) {
		LOG_ERR("Total sensor count over maximum value, total sensor count: %d, maximum: %d",
			total_sensor_count, MAX_CONTROL_SENSOR_POLLING_COUNT);
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		return;
	}

	if ((operation != DISABLE_SENSOR_POLLING) && (operation != ENABLE_SENSOR_POLLING)) {
		LOG_ERR("Input operation is invalid, operation: %d", operation);
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	// Data length subtract two bytes (one byte is total sensor count, one byte is operation) should be equal to the input number of sensor number list
	if ((msg->data_len - 2) != total_sensor_count) {
		LOG_ERR("Input data length is invalid, total sensor count: %d, number of sensor list: %d",
			total_sensor_count, msg->data_len - 2);
		msg->completion_code = CC_LENGTH_EXCEEDED;
		return;
	}

	uint8_t index = 0, control_sensor_index = 0, return_data_index = 0;
	uint8_t sensor_number[total_sensor_count];
	memcpy(&sensor_number[0], &msg->data[2], total_sensor_count);
	for (index = 0; index < total_sensor_count; ++index) {
		// Response data is a set of two
		return_data_index = 2 * index;

		control_sensor_index = sensor_config_index_map[sensor_number[index]];
		msg->data[return_data_index] = sensor_number[index];
		if (control_sensor_index != SENSOR_FAIL) {
			// Enable or Disable sensor polling
			sensor_config[control_sensor_index].is_enable_polling =
				((operation == DISABLE_SENSOR_POLLING) ? DISABLE_SENSOR_POLLING :
									       ENABLE_SENSOR_POLLING);
			msg->data[return_data_index + 1] =
				sensor_config[control_sensor_index].is_enable_polling;
		} else {
			msg->data[return_data_index + 1] = SENSOR_FAIL;
		}
	}

	msg->data_len = 2 * total_sensor_count;
	msg->completion_code = CC_SUCCESS;
	return;
}

#ifdef CONFIG_CRYPTO_ASPEED
__weak void OEM_1S_GET_FW_SHA256(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	uint8_t status = -1;
	uint32_t target = msg->data[0];
	uint32_t offset =
		(msg->data[1] | (msg->data[2] << 8) | (msg->data[3] << 16) | (msg->data[4] << 24));
	uint32_t length =
		(msg->data[5] | (msg->data[6] << 8) | (msg->data[7] << 16) | (msg->data[8] << 24));

	if (msg->data_len != 9) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}
	if (target == BIOS_UPDATE) {
		int pos = pal_get_bios_flash_position();
		if (pos == -1) {
			msg->completion_code = CC_INVALID_PARAM;
			return;
		}

		// Switch GPIO(BIOS SPI Selection Pin) to BIC
		bool ret = pal_switch_bios_spi_mux(GPIO_HIGH);
		if (!ret) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}

		status = get_fw_sha256(&msg->data[0], offset, length, pos);

		// Switch GPIO(BIOS SPI Selection Pin) to PCH
		ret = pal_switch_bios_spi_mux(GPIO_LOW);
		if (!ret) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}
	} else {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	msg->completion_code = status;
	msg->data_len = SHA256_DIGEST_SIZE;

	return;
}
#endif

__weak void OEM_1S_I2C_DEV_SCAN(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 1) { // only input scan bus
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t bus = msg->data[0];

	i2c_scan(bus, &msg->data[0], (uint8_t *)&msg->data_len);

	msg->completion_code = CC_SUCCESS;
	return;
}

__weak void OEM_1S_GET_BIC_STATUS(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	msg->data[0] = FIRMWARE_REVISION_1;
	msg->data[1] = FIRMWARE_REVISION_2;

	msg->data_len = 2;
	msg->completion_code = CC_SUCCESS;
	return;
}

__weak void OEM_1S_SET_VR_MONITOR_STATUS(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	if (msg->data[0] == 0) {
		set_vr_monitor_status(false);
	} else if (msg->data[0] == 1) {
		set_vr_monitor_status(true);
	} else {
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	msg->data_len = 0;
	msg->completion_code = CC_SUCCESS;
	return;
}

__weak void OEM_1S_GET_VR_MONITOR_STATUS(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	msg->data[0] = (uint8_t)get_vr_monitor_status();
	msg->data_len = 1;
	msg->completion_code = CC_SUCCESS;
	return;
}

__weak void OEM_1S_RESET_BIC(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	submit_bic_warm_reset();

	msg->data_len = 0;
	msg->completion_code = CC_SUCCESS;
	return;
}

__weak void OEM_1S_GET_SET_M2(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	msg->data_len = 0;
	msg->completion_code = CC_INVALID_CMD;
	return;
}

__weak void OEM_1S_SET_SSD_LED(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	msg->data_len = 0;
	msg->completion_code = CC_INVALID_CMD;
	return;
}

__weak void OEM_1S_GET_SSD_STATUS(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	msg->data_len = 0;
	msg->completion_code = CC_INVALID_CMD;
	return;
}

__weak void OEM_1S_12V_CYCLE_SLOT(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	int ret = pal_submit_12v_cycle_slot(msg);
	switch (ret) {
	case SUCCESS_12V_CYCLE_SLOT:
		msg->completion_code = CC_SUCCESS;
		break;
	case NOT_SUPPORT_12V_CYCLE_SLOT:
		msg->completion_code = CC_INVALID_CMD;
		break;
	case SLOT_OFF_FAILED:
	case SLOT_ON_FAILED:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}

	return;
}

__weak void OEM_1S_READ_BIC_REGISTER(ipmi_msg *msg)
{
	/*********************************
	* data 0~3: start of register address to read, LSB first
	* data 4  : bytes to read
	***********************************/
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 5) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}
	uint32_t *addr = (uint32_t *)(msg->data[0] | (msg->data[1] << 8) | (msg->data[2] << 16) |
				      (msg->data[3] << 24));
	uint8_t read_len = msg->data[4];
	memcpy(&msg->data[0], (uint8_t *)addr, read_len);

	msg->data_len = read_len;
	msg->completion_code = CC_SUCCESS;
	return;
}

__weak void OEM_1S_WRITE_BIC_REGISTER(ipmi_msg *msg)
{
	/*********************************
 	* data 0~3: start of register address to write, LSB first
 	* data 4  : bytes to write, <= 4 bytes
	* data 5~N: date to write, LSB first
	*
	* NOTE: The register address must be a multiple of 4
	***********************************/
	CHECK_NULL_ARG(msg);

	if (msg->data[4] < 1 || msg->data[4] > 4 || (msg->data_len != 5 + msg->data[4])) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint32_t addr =
		msg->data[0] | (msg->data[1] << 8) | (msg->data[2] << 16) | (msg->data[3] << 24);
	if (addr % 4) {
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	uint8_t write_len = msg->data[4];
	uint32_t reg_data = *(uint32_t *)addr;
	/* replace write_len bytes */
	reg_data &= ~BIT_MASK(write_len * 8);
	for (uint8_t i = 0; i < write_len; i++) {
		reg_data |= msg->data[i + 5] << (i * 8);
	}
	*(uint32_t *)addr = reg_data;

	msg->data_len = 0;
	msg->completion_code = CC_SUCCESS;
	return;
}

#ifdef ENABLE_FAN
__weak void OEM_1S_SET_FAN_DUTY_AUTO(ipmi_msg *msg)
{
	/*********************************
	Request -
	data 0: Fan pwm index
	data 1: Duty value
	Response -
	data 0: Completion code
	***********************************/
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 2) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t pwm_id = msg->data[0];
	uint8_t duty = msg->data[1];
	uint8_t current_fan_mode = FAN_AUTO_MODE, slot_index = 0;
	int ret = 0;

	msg->data_len = 0;
	msg->completion_code = CC_SUCCESS;

	if (msg->InF_source == SLOT1_BIC) {
		slot_index = INDEX_SLOT1;
	} else if (msg->InF_source == SLOT3_BIC) {
		slot_index = INDEX_SLOT3;
	} else {
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		return;
	}

	if (pwm_id >= MAX_FAN_PWM_INDEX_COUNT) {
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		return;
	}

	ret = pal_get_fan_ctrl_mode(&current_fan_mode);
	if (ret < 0) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	if (current_fan_mode != FAN_AUTO_MODE) {
		LOG_WRN("Set Fan was called when it's not at auto mode");
		return;
	}

	if (duty > MAX_FAN_DUTY_VALUE) {
		duty = MAX_FAN_DUTY_VALUE;
	}

	ret = pal_set_fan_duty(pwm_id, duty, slot_index);
	if (ret < 0) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
	}

	return;
}

__weak void OEM_1S_GET_FAN_DUTY(ipmi_msg *msg)
{
	/*********************************
	Request -
	data 0: Fan pwm index
	Response -
	data 0: Completion code
	data 1: current fan duty
	***********************************/
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t pwm_id = msg->data[0];
	uint8_t duty = 0, slot_index = 0;
	int ret = 0;

	msg->data_len = 0;
	if (msg->InF_source == SLOT1_BIC) {
		slot_index = INDEX_SLOT1;
	} else if (msg->InF_source == SLOT3_BIC) {
		slot_index = INDEX_SLOT3;
	} else {
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		return;
	}

	if (pwm_id >= MAX_FAN_PWM_INDEX_COUNT) {
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		return;
	}

	ret = pal_get_fan_duty(pwm_id, &duty, slot_index);
	if (ret < 0) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
	} else {
		msg->data[0] = duty;
		msg->data_len = 1;
		msg->completion_code = CC_SUCCESS;
	}

	return;
}

__weak void OEM_1S_GET_FAN_RPM(ipmi_msg *msg)
{
	/*********************************
	Request -
	data 0: Fan index
	Response -
	data 0: Completion code
	data 1: Current fan rpm
	***********************************/
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t fan_id = msg->data[0];
	uint16_t data = 0;
	int ret = 0;

	ret = pal_get_fan_rpm(fan_id, &data);
	if (ret < 0) {
		msg->data_len = 0;
		msg->completion_code = CC_UNSPECIFIED_ERROR;

	} else {
		msg->data[0] = (data >> 8) & 0xFF;
		msg->data[1] = data & 0xFF;
		msg->data_len = 2;
		msg->completion_code = CC_SUCCESS;
	}

	return;
}
#endif

__weak void OEM_1S_COPY_FLASH_IMAGE(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	msg->data_len = 0;
	msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
	return;
}

__weak void GET_COPY_FLASH_STATUS(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	msg->data_len = 0;
	msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
	return;
}

__weak void OEM_1S_INFORM_PEER_SLED_CYCLE(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	msg->data_len = 0;
	msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
	return;
}

__weak void OEM_1S_PEX_FLASH_READ(ipmi_msg *msg)
{
	return;
}

__weak void OEM_1S_GET_FPGA_USER_CODE(ipmi_msg *msg)
{
	return;
}

__weak void OEM_1S_GET_BOARD_ID(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 0) {
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	msg->data_len = 0;
	msg->completion_code = CC_INVALID_CMD;
	return;
}

__weak void OEM_1S_GET_CARD_TYPE(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	msg->data_len = 0;
	msg->completion_code = CC_INVALID_CMD;
	return;
}

__weak void OEM_1S_MULTI_ACCURACY_SENSOR_READING(ipmi_msg *msg)
{
	/*********************************
	Request -
	data 0 ~ N: multiple sensor numbers
	Response -
	data 0: Completion code
	***********************************/
	CHECK_NULL_ARG(msg);

	if (!msg->data_len) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t query_sensor_number = msg->data_len;

	if (query_sensor_number > MAX_MULTI_ACCURACY_SENSOR_READING_QUERY_NUM) {
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		return;
	}

	uint8_t query_sensor[query_sensor_number];
	memcpy(query_sensor, msg->data, query_sensor_number);

	uint16_t ofs = 0;
	/* get accuracy sensor reading by sensor number */
	for (uint8_t i = 0; i < query_sensor_number; i++) {
		ipmi_msg temp_msg = { 0 };

		ACCURACY_SENSOR_READING_REQ *req = (ACCURACY_SENSOR_READING_REQ *)temp_msg.data;
		req->sensor_num = query_sensor[i];
		req->read_option = GET_FROM_CACHE;
		temp_msg.data_len = 2;
		OEM_1S_ACCURACY_SENSOR_READING(&temp_msg);

		ACCURACY_SENSOR_READING_RES resp;
		memset(&resp, 0xFF, sizeof(resp));

		msg->data[ofs++] = query_sensor[i];

		if (temp_msg.completion_code == CC_SUCCESS) {
			if (temp_msg.data_len ==
			    _4BYTE_ACCURACY_SENSOR_READING_RES_LEN) { /* only support 4 bytes accuracy reading */
				memcpy(&resp, temp_msg.data, temp_msg.data_len);
			}
		}

		memcpy(msg->data + ofs, &resp, _4BYTE_ACCURACY_SENSOR_READING_RES_LEN);
		ofs += _4BYTE_ACCURACY_SENSOR_READING_RES_LEN;
	}

	msg->data_len = ofs;
	msg->completion_code = CC_SUCCESS;
}

__weak void OEM_1S_CLEAR_CMOS(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	int ret = pal_clear_cmos();

	if (ret < 0) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	msg->data_len = 0;
	msg->completion_code = CC_SUCCESS;
	return;
}

__weak void OEM_1S_BRIDGE_I2C_MSG_BY_COMPNT(ipmi_msg *msg)
{
	return;
}

#ifdef ENABLE_APML
__weak void OEM_1S_APML_READ(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 2) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t read_data;
	switch (msg->data[0]) {
	case 0x00: /* RMI */
		if (apml_read_byte(APML_BUS, SB_RMI_ADDR, msg->data[1], &read_data)) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
		} else {
			msg->data[0] = read_data;
			msg->data_len = 1;
			msg->completion_code = CC_SUCCESS;
		}
		break;
	case 0x01: /* TSI */
		if (apml_read_byte(APML_BUS, SB_TSI_ADDR, msg->data[1], &read_data)) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
		} else {
			msg->data[0] = read_data;
			msg->data_len = 1;
			msg->completion_code = CC_SUCCESS;
		}
		break;
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}
	return;
}

__weak void OEM_1S_APML_WRITE(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 3) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	switch (msg->data[0]) {
	case 0x00: /* RMI */
		if (apml_write_byte(APML_BUS, SB_RMI_ADDR, msg->data[1], msg->data[2])) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
		} else {
			msg->completion_code = CC_SUCCESS;
		}
		break;
	case 0x01: /* TSI */
		if (apml_write_byte(APML_BUS, SB_TSI_ADDR, msg->data[1], msg->data[2])) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
		} else {
			msg->completion_code = CC_SUCCESS;
		}
		break;
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}
	msg->data_len = 0;
	return;
}

__weak void OEM_1S_SEND_APML_REQUEST(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len < 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	static uint8_t index = 0;
	apml_msg apml_data;

	switch (msg->data[0]) {
	case APML_MSG_TYPE_MAILBOX: /* Mailbox */
		if (msg->data_len != 1 + sizeof(mailbox_WrData)) {
			msg->completion_code = CC_INVALID_LENGTH;
			return;
		}
		memcpy(apml_data.WrData, &msg->data[1], sizeof(mailbox_WrData));
		break;
	case APML_MSG_TYPE_CPUID: /* CPUID */
		if (msg->data_len != 1 + sizeof(cpuid_WrData)) {
			msg->completion_code = CC_INVALID_LENGTH;
			return;
		}
		memcpy(apml_data.WrData, &msg->data[1], sizeof(cpuid_WrData));
		break;
	case APML_MSG_TYPE_MCA: /* MCA */
		if (msg->data_len != 1 + sizeof(mca_WrData)) {
			msg->completion_code = CC_INVALID_LENGTH;
			return;
		}
		memcpy(apml_data.WrData, &msg->data[1], sizeof(mca_WrData));
		break;
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}
	apml_data.msg_type = msg->data[0];
	apml_data.bus = APML_BUS;
	apml_data.target_addr = SB_RMI_ADDR;
	apml_data.cb_fn = apml_request_callback;
	apml_data.ui32_arg = index;
	if (apml_read(&apml_data)) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	msg->data[0] = index;
	msg->data_len = 1;
	msg->completion_code = CC_SUCCESS;
	index++;
	return;
}

__weak void OEM_1S_GET_APML_RESPONSE(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	apml_msg apml_data;
	if (get_apml_response_by_index(&apml_data, msg->data[0])) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	switch (apml_data.msg_type) {
	case APML_MSG_TYPE_MAILBOX: /* Mailbox */
		memcpy(&msg->data[0], apml_data.RdData, sizeof(mailbox_RdData));
		msg->data_len = sizeof(mailbox_RdData);
		break;
	case APML_MSG_TYPE_CPUID: /* CPUID */
		memcpy(&msg->data[0], apml_data.RdData, sizeof(cpuid_RdData));
		msg->data_len = sizeof(cpuid_RdData);
		break;
	case APML_MSG_TYPE_MCA: /* MCA */
		memcpy(&msg->data[0], apml_data.RdData, sizeof(mca_RdData));
		msg->data_len = sizeof(mca_RdData);
		break;
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	msg->completion_code = CC_SUCCESS;
	return;
}
#endif

__weak void OEM_1S_NOTIFY_PMIC_ERROR(ipmi_msg *msg)
{
	int ret = 0;

	CHECK_NULL_ARG(msg);

	if (msg->data_len != 2) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	ret = pal_set_pmic_error_flag(msg->data[0], msg->data[1]);

	switch (ret) {
	case NOT_SUPPORT:
		msg->completion_code = CC_INVALID_CMD;
		break;
	case INVALID_ERROR_TYPE:
	case INVALID_DIMM_ID:
		msg->completion_code = CC_INVALID_DATA_FIELD;
		break;
	case SUCCESS:
		msg->completion_code = CC_SUCCESS;
		break;
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}
	msg->data_len = 0;

	return;
}

__weak void OEM_1S_GET_SDR(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	uint16_t next_record_ID = 0;
	uint16_t rsv_ID = 0, record_ID = 0;
	uint8_t offset = 0, req_len = 0;
	uint8_t *table_ptr = NULL;
	uint8_t rsv_table_index = RSV_TABLE_INDEX_0;

	// Config D: slot1 and slot3 need to use different reservation id
	if (msg->InF_source == SLOT3_BIC) {
		rsv_table_index = RSV_TABLE_INDEX_1;
	}

	rsv_ID = (msg->data[1] << 8) | msg->data[0];
	record_ID = (msg->data[3] << 8) | msg->data[2];
	offset = msg->data[4];
	req_len = msg->data[5];

	if (msg->data_len != 6) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	// If SDR_RSV_ID_check gets false is represent reservation id is incorrect
	if (SDR_RSV_ID_check(rsv_ID, rsv_table_index) == false) {
		msg->completion_code = CC_INVALID_RESERVATION;
		return;
	}

	if (!SDR_check_record_ID(record_ID)) {
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	// request length + next record ID (2 bytes) should not over IPMB data limit
	if ((req_len + 2) > IPMI_DATA_MAX_LENGTH) {
		msg->completion_code = CC_LENGTH_EXCEEDED;
		return;
	}

	if ((offset + req_len) > IPMI_SDR_HEADER_LEN + full_sdr_table[record_ID].record_len) {
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		return;
	}

	next_record_ID = SDR_get_record_ID(record_ID);
	msg->data[0] = next_record_ID & 0xFF;
	msg->data[1] = (next_record_ID >> 8) & 0xFF;

	table_ptr = (uint8_t *)&full_sdr_table[record_ID];
	memcpy(&msg->data[2], (table_ptr + offset), req_len);

	// return next record ID (2 bytes) + sdr data
	msg->data_len = req_len + 2;
	msg->completion_code = CC_SUCCESS;

	return;
}

__weak void OEM_1S_BMC_IPMB_ACCESS(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

#if MAX_IPMB_IDX
	if (msg->data_len < 2) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	if ((IPMB_inf_index_map[BMC_IPMB] == RESERVED) ||
	    (IPMB_config_table[IPMB_inf_index_map[BMC_IPMB]].interface == RESERVED_IF) ||
	    (IPMB_config_table[IPMB_inf_index_map[BMC_IPMB]].enable_status == DISABLE)) {
		LOG_ERR("BMC IPMB interface not supported on this platform");
		msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
		return;
	}

	ipmi_msg msgtobmc;
	memset(&msgtobmc, 0, sizeof(ipmi_msg));

	msgtobmc.InF_source = SELF;
	msgtobmc.InF_target = BMC_IPMB;
	msgtobmc.netfn = msg->data[0];
	msgtobmc.cmd = msg->data[1];
	msgtobmc.data_len = msg->data_len - 2;
	memcpy(msgtobmc.data, &msg->data[2], msgtobmc.data_len);

	ipmb_error status = ipmb_read(&msgtobmc, IPMB_inf_index_map[msgtobmc.InF_target]);
	if (status != IPMB_ERROR_SUCCESS) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	msg->data_len = 1 + msgtobmc.data_len;
	msg->data[0] = msgtobmc.completion_code;
	memcpy(&msg->data[1], msgtobmc.data, msgtobmc.data_len);
	msg->completion_code = CC_SUCCESS;

#else
	msg->completion_code = CC_UNSPECIFIED_ERROR;

#endif
}

__weak void OEM_1S_GET_BIOS_VERSION(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	msg->data_len = 0;
	msg->completion_code = CC_INVALID_CMD;
	return;
}

__weak void OEM_1S_GET_PCIE_CARD_STATUS(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	msg->data_len = 0;
	msg->completion_code = CC_INVALID_CMD;
	return;
}

__weak void OEM_1S_GET_PCIE_CARD_SENSOR_READING(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	msg->data_len = 0;
	msg->completion_code = CC_INVALID_CMD;
	return;
}

#ifdef CONFIG_I3C_ASPEED
__weak void OEM_1S_WRITE_READ_DIMM(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	msg->data_len = 0;
	msg->completion_code = CC_INVALID_CMD;
}
#endif

__weak void OEM_1S_GET_DIMM_I3C_MUX_SELECTION(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	msg->data_len = 0;
	msg->completion_code = CC_INVALID_CMD;
	return;
}

void IPMI_OEM_1S_handler(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);
	switch (msg->cmd) {
	case CMD_OEM_1S_MSG_IN:
		LOG_DBG("Received 1S Message In command");
		break;
	case CMD_OEM_1S_MSG_OUT:
		LOG_DBG("Received 1S Message Out command");
		OEM_1S_MSG_OUT(msg);
		break;
	case CMD_OEM_1S_GET_GPIO:
		LOG_DBG("Received 1S Get GPIO command");
		OEM_1S_GET_GPIO(msg);
		break;
	case CMD_OEM_1S_SET_GPIO:
		LOG_DBG("Received 1S Set GPIO command");
		break;
	case CMD_OEM_1S_GET_GPIO_CONFIG:
		LOG_DBG("Received 1S Get GPIO Config command");
		OEM_1S_GET_GPIO_CONFIG(msg);
		break;
	case CMD_OEM_1S_SET_GPIO_CONFIG:
		LOG_DBG("Received 1S Get GPIO Config command");
		OEM_1S_SET_GPIO_CONFIG(msg);
		break;
	case CMD_OEM_1S_FW_UPDATE:
		LOG_DBG("Received 1S Firmware Update command");
		OEM_1S_FW_UPDATE(msg);
		break;
	case CMD_OEM_1S_GET_BIC_FW_INFO:
		LOG_DBG("Received 1S Get BIC fw info command");
		OEM_1S_GET_BIC_FW_INFO(msg);
		break;
	case CMD_OEM_1S_GET_FW_VERSION:
		LOG_DBG("Received 1S Get Firmware Version command");
		OEM_1S_GET_FW_VERSION(msg);
		break;
	case CMD_OEM_1S_RESET_BMC:
		LOG_DBG("Received 1S BMC Reset command");
		OEM_1S_RESET_BMC(msg);
		break;
	case CMD_OEM_1S_READ_FW_IMAGE:
		LOG_DBG("Received 1S read fw image command");
		OEM_1S_READ_FW_IMAGE(msg);
		break;
	case CMD_OEM_1S_SET_WDT_FEED:
		LOG_DBG("Received 1S Set WatchDogTimer Feed command");
		OEM_1S_SET_WDT_FEED(msg);
		break;
	case CMD_OEM_1S_SENSOR_POLL_EN: // debug command
		LOG_DBG("Received 1S Sensor Poll Enable (Debug) command");
		OEM_1S_SENSOR_POLL_EN(msg);
		break;
	case CMD_OEM_1S_ACCURACY_SENSOR_READING:
		LOG_DBG("Received 1S Sesnor Reading (Accuracy) command");
		OEM_1S_ACCURACY_SENSOR_READING(msg);
		break;
	case CMD_OEM_1S_GET_SET_GPIO:
		LOG_DBG("Received 1S Get/Set GPIO command");
		OEM_1S_GET_SET_GPIO(msg);
		break;
	case CMD_OEM_1S_CONTROL_SENSOR_POLLING:
		LOG_DBG("Received 1S Sensor Polling Control command");
		OEM_1S_CONTROL_SENSOR_POLLING(msg);
		break;
#ifdef CONFIG_CRYPTO_ASPEED
	case CMD_OEM_1S_GET_FW_SHA256:
		LOG_DBG("Received 1S Get Firmware SHA256 command");
		OEM_1S_GET_FW_SHA256(msg);
		break;
#endif
	case CMD_OEM_1S_I2C_DEV_SCAN: // debug command
		LOG_DBG("Received 1S I2C Device Scan (Debug) command");
		OEM_1S_I2C_DEV_SCAN(msg);
		break;
	case CMD_OEM_1S_GET_BIC_STATUS:
		LOG_DBG("Received 1S Get BIC Status command");
		OEM_1S_GET_BIC_STATUS(msg);
		break;
	case CMD_OEM_1S_SET_VR_MONITOR_STATUS:
		LOG_DBG("Received 1S Set VR Monitor Status command");
		OEM_1S_SET_VR_MONITOR_STATUS(msg);
		break;
	case CMD_OEM_1S_GET_VR_MONITOR_STATUS:
		LOG_DBG("Received 1S Get VR Monitor Status command");
		OEM_1S_GET_VR_MONITOR_STATUS(msg);
		break;
	case CMD_OEM_1S_RESET_BIC:
		LOG_DBG("Received 1S Reset BIC command");
		OEM_1S_RESET_BIC(msg);
		break;
	case CMD_OEM_1S_GET_SET_M2:
		LOG_DBG("Received 1S Get/Set M2 command");
		OEM_1S_GET_SET_M2(msg);
		break;
	case CMD_OEM_1S_SET_SSD_LED:
		LOG_DBG("Received 1S Set SSD LED command");
		OEM_1S_SET_SSD_LED(msg);
		break;
	case CMD_OEM_1S_GET_SSD_STATUS:
		LOG_DBG("Received 1S Get SSD Status command");
		OEM_1S_GET_SSD_STATUS(msg);
		break;
	case CMD_OEM_1S_12V_CYCLE_SLOT:
		LOG_DBG("Received 1S 12v-Cycle Slot command");
		OEM_1S_12V_CYCLE_SLOT(msg);
		break;
	case CMD_OEM_1S_READ_BIC_REGISTER:
		LOG_DBG("Received 1S Read BIC Register command");
		OEM_1S_READ_BIC_REGISTER(msg);
		break;
	case CMD_OEM_1S_WRITE_BIC_REGISTER:
		LOG_DBG("Received 1S Write BIC Register command");
		OEM_1S_WRITE_BIC_REGISTER(msg);
		break;
	case CMD_OEM_1S_CLEAR_CMOS:
		LOG_DBG("Received 1S Clear CMOS command");
		OEM_1S_CLEAR_CMOS(msg);
		break;
#ifdef CONFIG_SNOOP_ASPEED
	case CMD_OEM_1S_GET_POST_CODE:
		LOG_DBG("Received 1S Get Post Code command");
		OEM_1S_GET_POST_CODE(msg);
		break;
#endif
#ifdef CONFIG_PCC_ASPEED
	case CMD_OEM_1S_GET_4BYTE_POST_CODE:
		LOG_DBG("Received 1S Get Post Code (4-Byte) command");
		OEM_1S_GET_4BYTE_POST_CODE(msg);
		break;
#endif
#ifdef CONFIG_PECI
	case CMD_OEM_1S_PECI_ACCESS:
		LOG_DBG("Received 1S Access PECI command");
		OEM_1S_PECI_ACCESS(msg);
		break;
#endif
#ifdef ENABLE_APML
	case CMD_OEM_1S_APML_READ:
		LOG_DBG("Received 1S APML Read command");
		OEM_1S_APML_READ(msg);
		break;
	case CMD_OEM_1S_APML_WRITE:
		LOG_DBG("Received 1S APML Write command");
		OEM_1S_APML_WRITE(msg);
		break;
	case CMD_OEM_1S_SEND_APML_REQUEST:
		LOG_DBG("Received 1S Send APML Request command");
		OEM_1S_SEND_APML_REQUEST(msg);
		break;
	case CMD_OEM_1S_GET_APML_RESPONSE:
		LOG_DBG("Received 1S Get APML Response command");
		OEM_1S_GET_APML_RESPONSE(msg);
		break;
#endif
#ifdef CONFIG_JTAG
	case CMD_OEM_1S_SET_JTAG_TAP_STA:
		LOG_DBG("Received 1S Set JTAG Tap Status command");
		OEM_1S_SET_JTAG_TAP_STA(msg);
		break;
	case CMD_OEM_1S_JTAG_DATA_SHIFT:
		LOG_DBG("Received 1S JTAG Data Shift command");
		OEM_1S_JTAG_DATA_SHIFT(msg);
		break;
#ifdef ENABLE_ASD
	case CMD_OEM_1S_ASD_INIT:
		LOG_DBG("Received 1S ASD Initialize command");
		OEM_1S_ASD_INIT(msg);
		break;
#endif
#endif
#ifdef ENABLE_FAN
	case CMD_OEM_1S_SET_FAN_DUTY_AUTO:
		LOG_DBG("Received 1S Set Fan Duty (Auto) command");
		OEM_1S_SET_FAN_DUTY_AUTO(msg);
		break;
	case CMD_OEM_1S_GET_FAN_DUTY:
		LOG_DBG("Received 1S Get Fan Duty command");
		OEM_1S_GET_FAN_DUTY(msg);
		break;
	case CMD_OEM_1S_GET_FAN_RPM:
		LOG_DBG("Received 1S Get Fan RPM command");
		OEM_1S_GET_FAN_RPM(msg);
		break;
#endif
	case CMD_OEM_1S_COPY_FLASH_IMAGE:
		LOG_DBG("Received 1S Copy Flash Image command");
		OEM_1S_COPY_FLASH_IMAGE(msg);
		break;
	case CMD_GET_COPY_FLASH_STATUS:
		LOG_DBG("Received 1S Get Copy Flash Status command");
		GET_COPY_FLASH_STATUS(msg);
		break;
	case CMD_OEM_1S_INFORM_PEER_SLED_CYCLE:
		LOG_DBG("Received 1S Inform Peer Sled-cycle command");
		OEM_1S_INFORM_PEER_SLED_CYCLE(msg);
		break;
	case CMD_OEM_1S_PEX_FLASH_READ:
		LOG_DBG("Received 1S Read PEX Flash command");
		OEM_1S_PEX_FLASH_READ(msg);
		break;
	case CMD_OEM_1S_GET_FPGA_USER_CODE:
		LOG_DBG("Received 1S Get FPGA User Code command");
		OEM_1S_GET_FPGA_USER_CODE(msg);
		break;
	case CMD_OEM_1S_GET_BOARD_ID:
		LOG_DBG("Received 1S Get Board ID command");
		OEM_1S_GET_BOARD_ID(msg);
		break;
	case CMD_OEM_1S_GET_CARD_TYPE:
		LOG_DBG("Received 1S Get Card Type command");
		OEM_1S_GET_CARD_TYPE(msg);
		break;
	case CMD_OEM_1S_MULTI_ACCURACY_SENSOR_READING:
		LOG_DBG("Received 1S Multi-accuracy Sensor Read command");
		OEM_1S_MULTI_ACCURACY_SENSOR_READING(msg);
		break;
	case CMD_OEM_1S_BRIDGE_I2C_MSG_BY_COMPNT:
		LOG_DBG("Received 1S Bridge I2C Message by Component command");
		OEM_1S_BRIDGE_I2C_MSG_BY_COMPNT(msg);
		break;
	case CMD_OEM_1S_NOTIFY_PMIC_ERROR:
		LOG_DBG("Received 1S Notify PMIC Error command");
		OEM_1S_NOTIFY_PMIC_ERROR(msg);
		break;
	case CMD_OEM_1S_GET_SDR:
		LOG_DBG("Received 1S Get SDR command");
		OEM_1S_GET_SDR(msg);
		break;
	case CMD_OEM_1S_BMC_IPMB_ACCESS:
		LOG_DBG("Received 1S BMC IPMB Access command");
		OEM_1S_BMC_IPMB_ACCESS(msg);
		break;
	case CMD_OEM_1S_GET_BIOS_VERSION:
		LOG_DBG("Received 1S Get BIOS version command");
		OEM_1S_GET_BIOS_VERSION(msg);
		break;
	case CMD_OEM_1S_GET_PCIE_CARD_STATUS:
		LOG_DBG("Received 1S Get PCIE card status command");
		OEM_1S_GET_PCIE_CARD_STATUS(msg);
		break;
	case CMD_OEM_1S_GET_PCIE_CARD_SENSOR_READING:
		LOG_DBG("Received 1S Get PCIE card sensor reading command");
		OEM_1S_GET_PCIE_CARD_SENSOR_READING(msg);
		break;
#ifdef CONFIG_I3C_ASPEED
	case CMD_OEM_1S_WRITE_READ_DIMM:
		LOG_DBG("Received 1S write read dimm information command");
		OEM_1S_WRITE_READ_DIMM(msg);
		break;
#endif
	case CMD_OEM_1S_GET_DIMM_I3C_MUX_SELECTION:
		LOG_DBG("Received 1S Get DIMM I3C MUX selection command");
		OEM_1S_GET_DIMM_I3C_MUX_SELECTION(msg);
		break;
	default:
		LOG_ERR("Invalid OEM message, netfn(0x%x) cmd(0x%x)", msg->netfn, msg->cmd);
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_CMD;
		break;
	}
	return;
}
