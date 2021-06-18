/*
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef IPMI_H
#define IPMI_H

#include "ipmb.h"
#include "plat_ipmi.h"
#include "plat_ipmb.h"

extern uint8_t IPMB_inf_index_map[];

typedef enum ipmi_error {
  ipmi_error_success = 0,             /**< Generic no-error flag    */
  ipmi_error_mutex_timeout,           /**< Fail to get mutex in time*/
} ipmi_error;


void ipmi_init(void);
ipmi_error IPMI_handler (ipmi_msg_cfg *msg_cfg);

// IPMI SENSOR
void pal_SENSOR_GET_SENSOR_READING(ipmi_msg *msg);

// IPMI APP
void pal_APP_GET_DEVICE_ID(ipmi_msg *msg);
void pal_APP_WARM_RESET(ipmi_msg *msg);
void pal_APP_GET_SELFTEST_RESULTS(ipmi_msg *msg);
void pal_APP_MASTER_WRITE_READ(ipmi_msg *msg);

// IPMI STORAGE
void pal_STORAGE_GET_FRUID_INFO(ipmi_msg *msg);
void pal_STORAGE_READ_FRUID_DATA(ipmi_msg *msg);
void pal_STORAGE_WRITE_FRUID_DATA(ipmi_msg *msg);
void pal_STORAGE_RSV_SDR(ipmi_msg *msg);
void pal_STORAGE_GET_SDR(ipmi_msg *msg);

// IPMI OEM
void pal_OEM_MSG_OUT(ipmi_msg *msg);
void pal_OEM_GET_GPIO(ipmi_msg *msg);
void pal_OEM_SET_GPIO(ipmi_msg *msg);
void pal_OEM_SENSOR_POLL_EN(ipmi_msg *msg);
void pal_OEM_FW_UPDATE(ipmi_msg *msg);
void pal_OEM_GET_SET_GPIO(ipmi_msg *msg);
void pal_OEM_I2C_DEV_SCAN(ipmi_msg *msg);


enum {
  CC_SUCCESS = 0x00,
  
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

// Application Command Codes 
enum {
  CMD_APP_GET_DEVICE_ID = 0x01,
  CMD_APP_COLD_RESET = 0x02,
  CMD_APP_WARM_RESET = 0x03,
  CMD_APP_GET_SELFTEST_RESULTS = 0x04,
  CMD_APP_MASTER_WRITE_READ = 0x52,
};

// Sensor Command Codes 
enum {
  CMD_SENSOR_GET_SENSOR_READING = 0x2D,
};

// Storage Command Codes 
enum {
  CMD_STORAGE_GET_FRUID_INFO = 0x10,
  CMD_STORAGE_READ_FRUID_DATA = 0x11,
  CMD_STORAGE_WRITE_FRUID_DATA = 0x12,
  CMD_STORAGE_RSV_SDR = 0x22,
  CMD_STORAGE_GET_SDR = 0x23,
};



#endif
