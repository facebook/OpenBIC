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

#ifndef IPMI_H
#define IPMI_H

#include <string.h>
#include "ipmb.h"
#include "plat_version.h"

#define IPMI_THREAD_STACK_SIZE 4096
#define IPMI_BUF_LEN 10
#ifndef IANA_ID
#define IANA_ID 0x00A015 // Meta's IANA
#endif

#define SENSOR_EVENT_MESSAGES_ENABLE (1 << 7)
#define SENSOR_SCANNING_ENABLE (1 << 6)
#define SENSOR_READING_STATE_UNAVAILABLE (1 << 5)

extern uint8_t IPMB_inf_index_map[];
extern uint8_t isPwOn;
extern struct k_msgq ipmi_msgq;
extern struct k_msgq self_ipmi_msgq;

struct ipmi_request {
	uint8_t netfn;
	uint8_t cmd;
	uint8_t data[0];
};

struct ipmi_response {
	uint8_t netfn;
	uint8_t cmd;
	uint8_t cmplt_code;
	uint8_t data[0];
};

typedef struct common_addsel_msg_t {
	uint8_t InF_target;
	uint8_t sensor_type;
	uint8_t sensor_number;
	uint8_t event_type;
	uint8_t event_data1;
	uint8_t event_data2;
	uint8_t event_data3;
} common_addsel_msg_t;

static inline void pack_ipmi_resp(struct ipmi_response *resp, ipmi_msg *ipmi_resp)
{
	resp->netfn = (ipmi_resp->netfn + 1) << 2; // ipmi netfn response package
	resp->cmd = ipmi_resp->cmd;
	resp->cmplt_code = ipmi_resp->completion_code;
	if (ipmi_resp->data_len != 0) {
		memcpy(resp->data, ipmi_resp->data, ipmi_resp->data_len);
	}
}

// If command is from KCS, we need to check whether BIC support this command.
bool pal_request_msg_to_BIC_from_KCS(uint8_t netfn, uint8_t cmd);
// If command is from KCS, we need to check whether BIC responds immediately.
bool pal_immediate_respond_from_KCS(uint8_t netfn, uint8_t cmd);
// If command is from KCS, we need to check whether system information is set via BIC.
int pal_record_bios_fw_version(uint8_t *buf, uint8_t size);
// If command is from ME, we need to check whether BIC support this command.
bool pal_request_msg_to_BIC_from_ME(uint8_t netfn, uint8_t cmd);
// For the command that BIC only bridges it, BIC doesn't return the command directly
// For this kind of commands we return through IPMB that receiving the responses from the other devices.
bool pal_is_not_return_cmd(uint8_t netfn, uint8_t cmd);
bool common_add_sel_evt_record(common_addsel_msg_t *sel_msg);
void ipmi_init(void);
void IPMI_handler(void *arug0, void *arug1, void *arug2);

enum {
	/* generic completion codes 00h, C0h-FFh */
	CC_SUCCESS = 0x00,
	CC_NODE_BUSY = 0xC0,
	CC_INVALID_CMD = 0xC1,
	CC_INVALID_LUN = 0xC2,
	CC_TIMEOUT = 0xC3,
	CC_OUT_OF_SPACE = 0xC4,
	CC_INVALID_RESERVATION = 0xC5,
	CC_DATA_TRUNCATED = 0xC6,
	CC_INVALID_LENGTH = 0xC7,
	CC_LENGTH_EXCEEDED = 0xC8,
	CC_PARAM_OUT_OF_RANGE = 0xC9,
	CC_SENSOR_NOT_PRESENT = 0xCB,
	CC_INVALID_DATA_FIELD = 0xCC,
	CC_CAN_NOT_RESPOND = 0xCE,
	CC_NOT_SUPP_IN_CURR_STATE = 0xD5,
	CC_UNSPECIFIED_ERROR = 0xFF,

	/* device-specific (OEM) codes 01h-7Eh */

	/* command-specific codes 80h-BEh */
	CC_INVALID_PARAM = 0x80,
	CC_FRU_DEV_BUSY = 0x81,
	CC_BRIDGE_MSG_ERR = 0x82,
	CC_I2C_BUS_ERROR = 0x83,
	CC_INVALID_IANA = 0x84,
};

// Network Function Codes (IPMI/Section 5.1)
enum {
	NETFN_CHASSIS_REQ = 0x00,
	NETFN_CHASSIS_RES,
	NETFN_BRIDGE_REQ,
	NETFN_BRIDGE_RES,
	NETFN_SENSOR_REQ,
	NETFN_SENSOR_RES,
	NETFN_APP_REQ,
	NETFN_APP_RES,
	NETFN_FIRMWARE_REQ,
	NETFN_FIRMWARE_RES,
	NETFN_STORAGE_REQ,
	NETFN_STORAGE_RES,
	NETFN_TRANSPORT_REQ,
	NETFN_TRANSPORT_RES,
	NETFN_DCMI_REQ = 0x2C,
	NETFN_DCMI_RES = 0x2D,
	NETFN_NM_REQ = 0x2E,
	NETFN_NM_RES = 0x2F,
	NETFN_OEM_REQ = 0x30,
	NETFN_OEM_RES = 0x31,
	NETFN_OEM_STORAGE_REQ = 0x32,
	NETFN_OEM_STORAGE_RES = 0x33,
	NETFN_OEM_Q_REQ = 0x36,
	NETFN_OEM_Q_RES = 0x37,
	NETFN_OEM_1S_REQ = 0x38,
	NETFN_OEM_1S_RES = 0x39,
	NETFN_OEM_ZION_REQ = 0x3A,
	NETFN_OEM_ZION_RES = 0x3B,
	NETFN_OEM_USB_DBG_REQ = 0x3C,
	NETFN_OEM_USB_DBG_RES = 0x3D,
};

// Application Command Codes (0x06)
enum {
	CMD_APP_GET_DEVICE_ID = 0x01,
	CMD_APP_COLD_RESET = 0x02,
	CMD_APP_WARM_RESET = 0x03,
	CMD_APP_GET_SELFTEST_RESULTS = 0x04,
	CMD_APP_GET_SYSTEM_GUID = 0x37,
	CMD_APP_MASTER_WRITE_READ = 0x52,
	CMD_APP_SET_SYS_INFO_PARAMS = 0x58,
};

// Chassis Command Codes (0x00)
enum {
	CMD_CHASSIS_GET_CHASSIS_STATUS = 0x01,
};

// Sensor Command Codes (0x04)
enum {
	CMD_SENSOR_PLATFORM_EVENT = 0x02,
	CMD_SENSOR_GET_SENSOR_READING = 0x2D,
};

// Storage Command Codes (0x0A)
enum {
	CMD_STORAGE_GET_FRUID_INFO = 0x10,
	CMD_STORAGE_READ_FRUID_DATA = 0x11,
	CMD_STORAGE_WRITE_FRUID_DATA = 0x12,
	CMD_STORAGE_RSV_SDR = 0x22,
	CMD_STORAGE_GET_SDR = 0x23,
	CMD_STORAGE_GET_SEL_INFO = 0x40,
	CMD_STORAGE_RSV_SEL = 0x42,
	CMD_STORAGE_GET_SEL = 0x43,
	CMD_STORAGE_ADD_SEL = 0x44,
};

// OEM NM Command Codes (0x2E)
enum {
	CMD_OEM_NM_FORCE_ME_RECOVERY = 0xDF,
};

// OEM Command Codes (0x30)
enum {
	CMD_OEM_GET_BOARD_ID = 0x37,
	CMD_OEM_CABLE_DETECTION = 0xCB,
	CMD_OEM_NM_SENSOR_READ = 0xE2,
	CMD_OEM_SET_SYSTEM_GUID = 0xEF,
	CMD_OEM_GET_MB_INDEX = 0xF0,
	CMD_OEM_SET_FAN_DUTY_MANUAL = 0xF1,
	CMD_OEM_GET_SET_FAN_CTRL_MODE = 0xF2,
};

// OEM 1S Command Codes (0x38)
enum {
	CMD_OEM_1S_MSG_IN = 0x1,
	CMD_OEM_1S_MSG_OUT = 0x2,
	CMD_OEM_1S_GET_GPIO = 0x3,
	CMD_OEM_1S_SET_GPIO = 0x4,
	CMD_OEM_1S_GET_GPIO_CONFIG = 0x5,
	CMD_OEM_1S_SET_GPIO_CONFIG = 0x6,
	CMD_OEM_1S_SEND_INTERRUPT_TO_BMC = 0x7,
	CMD_OEM_1S_SEND_POST_CODE_TO_BMC = 0x8,
	CMD_OEM_1S_FW_UPDATE = 0x9,
	CMD_OEM_1S_GET_BIC_FW_INFO = 0xA,
	CMD_OEM_1S_GET_FW_VERSION = 0xB,
	CMD_OEM_1S_SEND_HOST_POWER_STATE_TO_BMC = 0xC,
	CMD_OEM_1S_GET_POST_CODE = 0x12,
	CMD_OEM_1S_SET_VR_MONITOR_STATUS = 0x14,
	CMD_OEM_1S_GET_VR_MONITOR_STATUS = 0x15,
	CMD_OEM_1S_RESET_BMC = 0x16,
	CMD_OEM_1S_READ_FW_IMAGE = 0x18,
	CMD_OEM_1S_SET_WDT_FEED = 0x1E,
	CMD_OEM_1S_SET_JTAG_TAP_STA = 0x21,
	CMD_OEM_1S_JTAG_DATA_SHIFT = 0x22,
	CMD_OEM_1S_ACCURACY_SENSOR_READING = 0x23,
	CMD_OEM_1S_CLEAR_CMOS = 0x25,
	CMD_OEM_1S_ASD_INIT = 0x28,
	CMD_OEM_1S_PECI_ACCESS = 0x29,
	CMD_OEM_1S_GET_4BYTE_POST_CODE = 0x2A,
	CMD_OEM_1S_APML_READ = 0x2C,
	CMD_OEM_1S_APML_WRITE = 0x2D,
	CMD_OEM_1S_SEND_APML_REQUEST = 0x2E,
	CMD_OEM_1S_GET_APML_RESPONSE = 0x2F,

	CMD_OEM_1S_SENSOR_POLL_EN = 0x30,
	CMD_OEM_1S_GET_BIC_STATUS = 0x31,
	CMD_OEM_1S_RESET_BIC = 0x32,
	CMD_OEM_1S_SEND_4BYTE_POST_CODE_TO_BMC = 0x33,
	CMD_OEM_1S_GET_SET_M2 = 0x34,
	CMD_OEM_1S_SET_SSD_LED = 0x39,
	CMD_OEM_1S_GET_SSD_STATUS = 0x3A,
	CMD_OEM_1S_GET_SET_GPIO = 0x41,
	CMD_OEM_1S_GET_SET_BIC_VGPIO = 0x42,
	CMD_OEM_1S_GET_FW_SHA256 = 0x43,
	CMD_OEM_1S_CONTROL_SENSOR_POLLING = 0x45,
	CMD_OEM_1S_SET_FAN_DUTY_AUTO = 0x50,
	CMD_OEM_1S_GET_FAN_DUTY = 0x51,
	CMD_OEM_1S_GET_FAN_RPM = 0x52,
	CMD_OEM_1S_COPY_FLASH_IMAGE = 0x53,
	CMD_GET_COPY_FLASH_STATUS = 0x54,
	// Debug command
	CMD_OEM_1S_I2C_DEV_SCAN = 0x60,

	CMD_OEM_1S_12V_CYCLE_SLOT = 0x64,
	CMD_OEM_1S_INFORM_PEER_SLED_CYCLE = 0x66,
	CMD_OEM_1S_READ_BIC_REGISTER = 0x68,
	CMD_OEM_1S_WRITE_BIC_REGISTER = 0x69,

	CMD_OEM_1S_BMC_IPMB_ACCESS = 0x6A,
	CMD_OEM_1S_GET_HSC_STATUS = 0x6C,

	CMD_OEM_1S_PEX_FLASH_READ = 0x72,
	CMD_OEM_1S_GET_FPGA_USER_CODE = 0x73,
	CMD_OEM_1S_BRIDGE_I2C_MSG_BY_COMPNT = 0x74,
	CMD_OEM_1S_GET_PCIE_CARD_STATUS = 0x76,
	CMD_OEM_1S_GET_PCIE_CARD_SENSOR_READING = 0x77,

	CMD_OEM_1S_MULTI_ACCURACY_SENSOR_READING = 0x88,
	CMD_OEM_1S_GET_BOARD_ID = 0xA0,
	CMD_OEM_1S_GET_CARD_TYPE = 0xA1,
	CMD_OEM_1S_GET_BIOS_VERSION = 0xA2,
	CMD_OEM_1S_NOTIFY_PMIC_ERROR = 0xB0,
	CMD_OEM_1S_WRITE_READ_DIMM = 0xB1,
	CMD_OEM_1S_GET_SDR = 0xC0,
	CMD_OEM_1S_SEND_APML_ALERT_TO_BMC = 0xD0,
	CMD_OEM_1S_GET_DIMM_I3C_MUX_SELECTION = 0xB2,
};

enum {
	INDEX_SLOT1 = 0x01,
	INDEX_SLOT3 = 0x03,
};

#endif
