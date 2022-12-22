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
#include "plat_sensor_table.h"
#include "pex89000.h"
#include "hal_gpio.h"
#include "plat_fru.h"
#include "plat_gpio.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_ipmi);

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
		if (is_mb_dc_on() == false) {
			msg->completion_code = CC_PEX_NOT_POWER_ON;
			return;
		}
		uint8_t pex_sensor_num_table[PEX_MAX_NUMBER] = { SENSOR_NUM_TEMP_PEX_0,
								 SENSOR_NUM_TEMP_PEX_1 };
		int reading;

		uint8_t pex_sensor_num = pex_sensor_num_table[component - CB_COMPNT_PCIE_SWITCH0];
		sensor_cfg *cfg = &sensor_config[sensor_config_index_map[pex_sensor_num]];
		pex89000_unit *p = (pex89000_unit *)cfg->priv_data;

		if (cfg->pre_sensor_read_hook) {
			if (cfg->pre_sensor_read_hook(cfg->num, cfg->pre_sensor_read_args) ==
			    false) {
				LOG_ERR("PEX%d pre-read failed!",
					component - CB_COMPNT_PCIE_SWITCH0);
				msg->completion_code = CC_PEX_PRE_READING_FAIL;
				return;
			}
		}

		if (pex_access_engine(cfg->port, cfg->target_addr, p->idx, pex_access_sbr_ver,
				      &reading)) {
			if (cfg->post_sensor_read_hook(cfg->num, cfg->post_sensor_read_args,
						       NULL) == false) {
				LOG_ERR("PEX%d post-read failed!",
					component - CB_COMPNT_PCIE_SWITCH0);
			}
			msg->completion_code = CC_PEX_ACCESS_FAIL;
			return;
		}

		if (cfg->post_sensor_read_hook) {
			if (cfg->post_sensor_read_hook(cfg->num, cfg->post_sensor_read_args,
						       NULL) == false) {
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
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}
	return;
}

void OEM_1S_GET_ASIC_CARD_STATUS(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t asic_card_id = msg->data[0];
	uint8_t offset = 0;

	switch (asic_card_id) {
	case FIO_FRU_ID:
		if (gpio_get(PRSNT_FIO_N) == LOW_ACTIVE) {
			msg->data[0] = FIO_PRESENT;
		} else {
			msg->data[0] = FIO_NOT_PRESENT;
		}
		msg->completion_code = CC_SUCCESS;
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
		offset = asic_card_id - ACCL_1_FRU_ID;
		msg->data[0] = asic_card_info[offset].card_presence_status;
		msg->completion_code = CC_SUCCESS;
		break;
	default:
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		break;
	}

	return;
}
