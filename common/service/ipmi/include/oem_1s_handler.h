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

#ifndef OEM_1S_HANDLER_H
#define OEM_1S_HANDLER_H

#include "ipmi.h"

enum FIRMWARE_INFO {
	BIC_PLAT_NAME = 1,
	BIC_PLAT_BOARD_ID,
	BIC_PROJ_NAME,
	BIC_PROJ_STAGE,
};

enum FIRMWARE_COMPONENT {
	COMPNT_CPLD = 1,
	COMPNT_BIC,
	COMPNT_ME,
	COMPNT_BIOS,
	COMPNT_PVCCIN,
	COMPNT_PVCCFA_EHV_FIVRA,
	COMPNT_PVCCD_HV,
	COMPNT_PVCCINFAON,
	COMPNT_PVCCFA_EHV
};

enum ERROR_CODE_SET_PMIC_ERROR_FLAG {
	SUCCESS = 0,
	NOT_SUPPORT = -1,
	INVALID_ERROR_TYPE = -2,
	INVALID_DIMM_ID = -3,
};

#define IS_SECTOR_END_MASK 0x80
enum FIRWARE_UPDATE_TARGET {
	BIOS_UPDATE = 0,
	CPLD_UPDATE,
	BIC_UPDATE,
	CXL_UPDATE,
	PRoT_FLASH_UPDATE,
};

#define GLOBAL_GPIO_IDX_KEY 0xFF
enum GET_SET_GPIO_OPTIONS {
	GET_GPIO_STATUS = 0,
	SET_GPIO_OUTPUT_STATUS,
	GET_GPIO_DIRECTION_STATUS,
	SET_GPIO_DIRECTION_STATUS,
};

enum GET_SET_VGPIO_OPTIONS {
	GET_VGPIO_DIRECTION_AND_STATUS = 0,
	SET_VGPIO_STATUS,
};

typedef struct _ACCURACY_SENSOR_READING_REQ {
	uint8_t sensor_num;
	uint8_t read_option;
} ACCURACY_SENSOR_READING_REQ;

typedef struct _ACCURACY_SENSOR_READING_RES {
	uint16_t decimal;
	uint16_t fraction;
	uint8_t status;
} ACCURACY_SENSOR_READING_RES;

uint8_t gpio_idx_exchange(ipmi_msg *msg);

void OEM_1S_MSG_OUT(ipmi_msg *msg);
void OEM_1S_GET_GPIO(ipmi_msg *msg);
void OEM_1S_GET_GPIO_CONFIG(ipmi_msg *msg);
void OEM_1S_SET_GPIO_CONFIG(ipmi_msg *msg);
void OEM_1S_FW_UPDATE(ipmi_msg *msg);
void OEM_1S_GET_BIC_FW_INFO(ipmi_msg *msg);
void OEM_1S_GET_FW_VERSION(ipmi_msg *msg);
void OEM_1S_SET_VR_MONITOR_STATUS(ipmi_msg *msg);
void OEM_1S_GET_VR_MONITOR_STATUS(ipmi_msg *msg);
void OEM_1S_RESET_BMC(ipmi_msg *msg);
void OEM_1S_READ_FW_IMAGE(ipmi_msg *msg);
void OEM_1S_SENSOR_POLL_EN(ipmi_msg *msg);
void OEM_1S_ACCURACY_SENSOR_READING(ipmi_msg *msg);
void OEM_1S_GET_SET_GPIO(ipmi_msg *msg);
void OEM_1S_GET_SET_BIC_VGPIO(ipmi_msg *msg);
void OEM_1S_GET_FW_SHA256(ipmi_msg *msg);
void OEM_1S_I2C_DEV_SCAN(ipmi_msg *msg);
void OEM_1S_GET_BIC_STATUS(ipmi_msg *msg);
void OEM_1S_RESET_BIC(ipmi_msg *msg);
void OEM_1S_12V_CYCLE_SLOT(ipmi_msg *msg);
void OEM_1S_READ_BIC_REGISTER(ipmi_msg *msg);
void OEM_1S_WRITE_BIC_REGISTER(ipmi_msg *msg);
void OEM_1S_INFORM_PEER_SLED_CYCLE(ipmi_msg *msg);
void OEM_1S_PEX_FLASH_READ(ipmi_msg *msg);
void OEM_1S_GET_FPGA_USER_CODE(ipmi_msg *msg);
void OEM_1S_GET_CARD_TYPE(ipmi_msg *msg);
void OEM_1S_CLEAR_CMOS(ipmi_msg *msg);
void OEM_1S_NOTIFY_PMIC_ERROR(ipmi_msg *msg);
void OEM_1S_GET_SDR(ipmi_msg *msg);
void OEM_1S_BMC_IPMB_ACCESS(ipmi_msg *msg);
void OEM_1S_GET_HSC_STATUS(ipmi_msg *msg);
void OEM_1S_GET_BIOS_VERSION(ipmi_msg *msg);
void OEM_1S_GET_PCIE_CARD_STATUS(ipmi_msg *msg);
void OEM_1S_GET_PCIE_CARD_SENSOR_READING(ipmi_msg *msg);
void OEM_1S_GET_DIMM_I3C_MUX_SELECTION(ipmi_msg *msg);

#ifdef CONFIG_SNOOP_ASPEED
void OEM_1S_GET_POST_CODE(ipmi_msg *msg);
#endif

#ifdef CONFIG_PCC_ASPEED
void OEM_1S_GET_4BYTE_POST_CODE(ipmi_msg *msg);
#endif

#ifdef CONFIG_PECI
void OEM_1S_PECI_ACCESS(ipmi_msg *msg);
#endif

#ifdef ENABLE_APML
void OEM_1S_APML_READ(ipmi_msg *msg);
void OEM_1S_APML_WRITE(ipmi_msg *msg);
void OEM_1S_SEND_APML_REQUEST(ipmi_msg *msg);
void OEM_1S_GET_APML_RESPONSE(ipmi_msg *msg);
#endif

#ifdef CONFIG_JTAG
void OEM_1S_SET_JTAG_TAP_STA(ipmi_msg *msg);
void OEM_1S_JTAG_DATA_SHIFT(ipmi_msg *msg);

#ifdef ENABLE_ASD
void OEM_1S_ASD_INIT(ipmi_msg *msg);
#endif
#endif

#ifdef ENABLE_FAN
void OEM_1S_SET_FAN_DUTY_AUTO(ipmi_msg *msg);
void OEM_1S_GET_FAN_DUTY(ipmi_msg *msg);
void OEM_1S_GET_FAN_RPM(ipmi_msg *msg);
#endif

#ifdef CONFIG_I3C_ASPEED
void OEM_1S_WRITE_READ_DIMM(ipmi_msg *msg);
#endif

void IPMI_OEM_1S_handler(ipmi_msg *msg);

#endif
