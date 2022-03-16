#ifndef IPMI_H
#define IPMI_H

#include <string.h>
#include "ipmb.h"

#define WW_IANA_ID 0x009c9c
#define IPMI_THREAD_STACK_SIZE 4000
#define ipmi_buf_len 10

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

void ipmi_init(void);
void IPMI_handler(void *arug0, void *arug1, void *arug2);

// IPMI CHASSIS
void pal_CHASSIS_GET_CHASSIS_STATUS(ipmi_msg *msg);

// IPMI SENSOR
void pal_SENSOR_GET_SENSOR_READING(ipmi_msg *msg);

// IPMI APP
void pal_APP_GET_DEVICE_ID(ipmi_msg *msg);
void pal_APP_COLD_RESET(ipmi_msg *msg);
void pal_APP_WARM_RESET(ipmi_msg *msg);
void pal_APP_GET_SELFTEST_RESULTS(ipmi_msg *msg);
void pal_APP_GET_SYSTEM_GUID(ipmi_msg *msg);
void pal_APP_MASTER_WRITE_READ(ipmi_msg *msg);

// IPMI STORAGE
void pal_STORAGE_GET_FRUID_INFO(ipmi_msg *msg);
void pal_STORAGE_READ_FRUID_DATA(ipmi_msg *msg);
void pal_STORAGE_WRITE_FRUID_DATA(ipmi_msg *msg);
void pal_STORAGE_RSV_SDR(ipmi_msg *msg);
void pal_STORAGE_GET_SDR(ipmi_msg *msg);

// IPMI OEM
void pal_OEM_SENSOR_READ(ipmi_msg *msg);
void pal_OEM_SET_SYSTEM_GUID(ipmi_msg *msg);
void pal_OEM_GET_MB_INDEX(ipmi_msg *msg);

// IPMI OEM 1S
void pal_OEM_1S_MSG_OUT(ipmi_msg *msg);
void pal_OEM_1S_GET_GPIO(ipmi_msg *msg);
void pal_OEM_1S_SET_GPIO(ipmi_msg *msg);
void pal_OEM_1S_SEND_INTERRUPT_TO_BMC(ipmi_msg *msg);
void pal_OEM_1S_SENSOR_POLL_EN(ipmi_msg *msg);
void pal_OEM_1S_FW_UPDATE(ipmi_msg *msg);
void pal_OEM_1S_GET_FW_VERSION(ipmi_msg *msg);
void pal_OEM_1S_GET_POST_CODE(ipmi_msg *msg);
void pal_OEM_1S_SET_VR_MONITOR_STATUS(ipmi_msg *msg);
void pal_OEM_1S_GET_VR_MONITOR_STATUS(ipmi_msg *msg);
void pal_OEM_1S_RESET_BMC(ipmi_msg *msg);
void pal_OEM_1S_PECIaccess(ipmi_msg *msg);
void pal_OEM_1S_ASD_INIT(ipmi_msg *msg);
void pal_OEM_1S_GET_SET_GPIO(ipmi_msg *msg);
void pal_OEM_1S_ACCURACY_SENSOR_READING(ipmi_msg *msg);
void pal_OEM_1S_I2C_DEV_SCAN(ipmi_msg *msg);
void pal_OEM_1S_SET_JTAG_TAP_STA(ipmi_msg *msg);
void pal_OEM_1S_JTAG_DATA_SHIFT(ipmi_msg *msg);
void pal_OEM_1S_GET_BIC_STATUS(ipmi_msg *msg);
void pal_OEM_1S_RESET_BIC(ipmi_msg *msg);
void pal_OEM_1S_12V_CYCLE_SLOT(ipmi_msg *msg);
void pal_OEM_1S_READ_BIC_REGISTER(ipmi_msg *msg);
void pal_OEM_1S_WRITE_BIC_REGISTER(ipmi_msg *msg);

enum { CC_SUCCESS = 0x00,

       CC_INVALID_PARAM = 0x80,
       CC_FRU_DEV_BUSY = 0x81,
       CC_BRIDGE_MSG_ERR = 0x82,
       CC_I2C_BUS_ERROR = 0x83,
       CC_INVALID_IANA = 0x84,

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
};

// Network Function Codes (IPMI/Section 5.1)
enum { NETFN_CHASSIS_REQ = 0x00,
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

// Application Command Codes
enum { CMD_APP_GET_DEVICE_ID = 0x01,
       CMD_APP_COLD_RESET = 0x02,
       CMD_APP_WARM_RESET = 0x03,
       CMD_APP_GET_SELFTEST_RESULTS = 0x04,
       CMD_APP_GET_SYSTEM_GUID = 0x37,
       CMD_APP_MASTER_WRITE_READ = 0x52,
};

// Chassis Command Codes
enum { CMD_CHASSIS_GET_CHASSIS_STATUS = 0x01,
};

// Sensor Command Codes
enum { CMD_SENSOR_GET_SENSOR_READING = 0x2D,
};

// Storage Command Codes
enum { CMD_STORAGE_GET_FRUID_INFO = 0x10,
       CMD_STORAGE_READ_FRUID_DATA = 0x11,
       CMD_STORAGE_WRITE_FRUID_DATA = 0x12,
       CMD_STORAGE_RSV_SDR = 0x22,
       CMD_STORAGE_GET_SDR = 0x23,
       CMD_STORAGE_GET_SEL_INFO = 0x40,
       CMD_STORAGE_RSV_SEL = 0x42,
       CMD_STORAGE_GET_SEL = 0x43,
       CMD_STORAGE_ADD_SEL = 0x44,
};

// OEM Command Codes
enum { CMD_OEM_SENSOR_READ = 0xE2,
       CMD_OEM_SET_SYSTEM_GUID = 0xEF,
       CMD_OEM_GET_MB_INDEX = 0xF0,
};

// OEM 1S Command Codes
enum { CMD_OEM_1S_MSG_IN = 0x1,
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
       CMD_OEM_1S_SET_VR_MONITOR_STATUS = 0x14,
       CMD_OEM_1S_GET_VR_MONITOR_STATUS = 0x15,
       CMD_OEM_1S_RESET_BMC = 0x16,
       CMD_OEM_1S_SET_JTAG_TAP_STA = 0x21,
       CMD_OEM_1S_JTAG_DATA_SHIFT = 0x22,
       CMD_OEM_1S_ACCURACY_SENSOR_READING = 0x23,
       CMD_OEM_1S_ASD_INIT = 0x28,
       CMD_OEM_1S_PECIaccess = 0x29,
       CMD_OEM_1S_SENSOR_POLL_EN = 0x30,
       CMD_OEM_1S_GET_BIC_STATUS = 0x31,
       CMD_OEM_1S_RESET_BIC = 0x32,
       CMD_OEM_1S_GET_SET_GPIO = 0x41,
       // Debug command
       CMD_OEM_1S_I2C_DEV_SCAN = 0x60,

       CMD_OEM_1S_12V_CYCLE_SLOT = 0x64,
       CMD_OEM_1S_READ_BIC_REGISTER = 0x68,
       CMD_OEM_1S_WRITE_BIC_REGISTER = 0x69,
};

#endif
