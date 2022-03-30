#ifndef IPMI_H
#define IPMI_H

#include <string.h>
#include "ipmb.h"
#include "plat_version.h"

#define IPMI_THREAD_STACK_SIZE 4000
#define IPMI_BUF_LEN 10
#define DEBUG_IPMI 0

extern uint8_t IPMB_inf_index_map[];
extern uint8_t isPwOn;
extern struct k_msgq ipmi_msgq;

typedef enum ipmi_error {
	ipmi_error_success = 0, /**< Generic no-error flag    */
	ipmi_error_mutex_timeout, /**< Fail to get mutex in time*/
} ipmi_error;

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
// If command is from ME, we need to check whether BIC support this command.
bool pal_request_msg_to_BIC_from_ME(uint8_t netfn, uint8_t cmd);
// For the command that BIC only bridges it, BIC doesn't return the command directly
// For this kind of commands we return through IPMB that receiving the responses from the other devices.
bool pal_is_not_return_cmd(uint8_t netfn, uint8_t cmd);

void ipmi_init(void);
ipmi_error IPMI_handler(void *arug0, void *arug1, void *arug2);

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
};

// Chassis Command Codes (0x00)
enum {
	CMD_CHASSIS_GET_CHASSIS_STATUS = 0x01,
};

// Sensor Command Codes (0x04)
enum {
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

// OEM Command Codes (0x30)
enum {
	CMD_OEM_NM_SENSOR_READ = 0xE2,
	CMD_OEM_SET_SYSTEM_GUID = 0xEF,
	CMD_OEM_GET_MB_INDEX = 0xF0,
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
	CMD_OEM_1S_GET_FW_VERSION = 0xB,
	CMD_OEM_1S_GET_POST_CODE = 0x12,
	CMD_OEM_1S_RESET_BMC = 0x16,
	CMD_OEM_1S_SET_JTAG_TAP_STA = 0x21,
	CMD_OEM_1S_JTAG_DATA_SHIFT = 0x22,
	CMD_OEM_1S_ACCURACY_SENSOR = 0x23,
	CMD_OEM_1S_ASD_INIT = 0x28,
	CMD_OEM_1S_PECI_ACCESS = 0x29,
	CMD_OEM_1S_SENSOR_POLL_EN = 0x30,
	CMD_OEM_1S_GET_BIC_STATUS = 0x31,
	CMD_OEM_1S_RESET_BIC = 0x32,
	CMD_OEM_1S_GET_SET_GPIO = 0x41,
	// Debug command
	CMD_OEM_1S_I2C_DEV_SCAN = 0x60,

	CMD_OEM_1S_12V_CYCLE_SLOT = 0x64,
};

#endif
