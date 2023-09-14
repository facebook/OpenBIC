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

#ifndef PLAT_IPMI_H
#define PLAT_IPMI_H

#include <stdint.h>
#include "ipmi.h"
#include "plat_sensor_table.h"

#define BIC_FW_DATA_LENGTH 7
#define VR_FW_VERSION_LEN 4
#define IS_SECTOR_END_MASK 0x80
#define WITHOUT_SECTOR_END_MASK 0x7F
#define BIC_UPDATE_MAX_OFFSET 0x50000

#define CC_PEX_NOT_POWER_ON 0xB0
#define CC_PEX_PRE_READING_FAIL 0xB1
#define CC_PEX_ACCESS_FAIL 0xB2

#define PCIE_CARD_ID_OFFSET 18
#define RESERVE_DEFAULT_VALUE 0

#define PCIE_CARD_NOT_PRESENT_BIT BIT(0)
#define PCIE_CARD_NOT_ACCESSIBLE_BIT BIT(1)
#define PCIE_CARD_DEVICE_NOT_READY_BIT BIT(2)

#define FREYA_FW_VERSION_LENGTH sizeof(freya_fw_info)
#define FREYA_READY 0
#define FREYA_NOT_READY 1
#define FREYA_SUPPORT_MODULE_IDENTIFIER 0
#define FREYA_NOT_SUPPORT_MODULE_IDENTIFIER 1

/* switch mux selection */
struct SWITCH_MUX_INFO {
	uint8_t device;
	uint8_t control_gpio;
	uint8_t sw_to_flash_value;
	uint8_t bic_to_flash_value;
};

/** enum number follow GT for now since bmc hasn't ready **/
enum CB_FIRMWARE_COMPONENT {
	CB_COMPNT_VR_XDPE15284 = 0,
	CB_COMPNT_BIC = 2,
	CB_COMPNT_PCIE_SWITCH0 = 3,
	CB_COMPNT_PCIE_SWITCH1 = 4,
	CB_COMPNT_CPLD = 7,
	CB_COMPNT_ACCL1_CH1_FREYA,
	CB_COMPNT_ACCL1_CH2_FREYA,
	CB_COMPNT_ACCL2_CH1_FREYA,
	CB_COMPNT_ACCL2_CH2_FREYA,
	CB_COMPNT_ACCL3_CH1_FREYA,
	CB_COMPNT_ACCL3_CH2_FREYA,
	CB_COMPNT_ACCL4_CH1_FREYA,
	CB_COMPNT_ACCL4_CH2_FREYA,
	CB_COMPNT_ACCL5_CH1_FREYA,
	CB_COMPNT_ACCL5_CH2_FREYA,
	CB_COMPNT_ACCL6_CH1_FREYA,
	CB_COMPNT_ACCL6_CH2_FREYA,
	CB_COMPNT_ACCL7_CH1_FREYA,
	CB_COMPNT_ACCL7_CH2_FREYA,
	CB_COMPNT_ACCL8_CH1_FREYA,
	CB_COMPNT_ACCL8_CH2_FREYA,
	CB_COMPNT_ACCL9_CH1_FREYA,
	CB_COMPNT_ACCL9_CH2_FREYA,
	CB_COMPNT_ACCL10_CH1_FREYA,
	CB_COMPNT_ACCL10_CH2_FREYA,
	CB_COMPNT_ACCL11_CH1_FREYA,
	CB_COMPNT_ACCL11_CH2_FREYA,
	CB_COMPNT_ACCL12_CH1_FREYA,
	CB_COMPNT_ACCL12_CH2_FREYA,
	CB_COMPNT_MAX,
};

void pal_construct_ipmi_add_sel_msg(ipmi_msg *msg, common_addsel_msg_t *sel_msg);
extern struct SWITCH_MUX_INFO pcie_switch_mux_info[PEX_MAX_NUMBER];

#endif
