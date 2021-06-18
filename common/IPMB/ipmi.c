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

#include <stdio.h>
#include "cmsis_os2.h"
#include "board_device.h"
#include "objects.h"
#include "ipmi.h"
#include <string.h>
#include "hal_i2c.h"
#include "sensor.h"
#include "fru.h"
#include "ipmi_def.h"
#define IPMI_QUEUE_SIZE 5


extern uint8_t IPMB_inf_index_map[MAX_I2C_BUS_NUM];
extern SDR_Full_sensor full_sensor_table[];

osMutexId_t IPMI_mutex;
const osMutexAttr_t IPMI_Mutex_attr = {
        "IPMIQueueMutex",                         // human readable mutex name
        osMutexRecursive | osMutexPrioInherit,    // attr_bits
        NULL,                                     // memory for control block
        0U,                                       // size for control block
};

__WEAK bool pal_is_not_return_cmd(uint8_t netfn, uint8_t cmd) {
  return 0;
}

void IPMI_SENSOR_handler(ipmi_msg *msg) {
  switch (msg->cmd) {
    case CMD_SENSOR_GET_SENSOR_READING:
      pal_SENSOR_GET_SENSOR_READING(msg);
      break;
    default:
      printf("invalid sensor msg netfn: %x, cmd: %x\n",msg->netfn,msg->cmd);
      msg->data_len = 0;
      break;
  }
  return;
}

void IPMI_APP_handler(ipmi_msg *msg) {
  switch (msg->cmd) {
    case CMD_APP_GET_DEVICE_ID:
      pal_APP_GET_DEVICE_ID(msg);
      break;
    case CMD_APP_COLD_RESET:
      break;
    case CMD_APP_WARM_RESET:
      pal_APP_WARM_RESET(msg);
      break;
    case CMD_APP_GET_SELFTEST_RESULTS:
      pal_APP_GET_SELFTEST_RESULTS(msg);
      break;
    case CMD_APP_MASTER_WRITE_READ:
      pal_APP_MASTER_WRITE_READ(msg);
      break;
    default:
      printf("invalid APP msg netfn: %x, cmd: %x\n",msg->netfn,msg->cmd);
      msg->data_len = 0;
      break;
  }
  
  return;
}

void IPMI_Storage_handler(ipmi_msg *msg) {
  switch (msg->cmd) {
    case CMD_STORAGE_GET_FRUID_INFO:
      pal_STORAGE_GET_FRUID_INFO(msg);
      break;
    case CMD_STORAGE_READ_FRUID_DATA:
      pal_STORAGE_READ_FRUID_DATA(msg);
      break;
    case CMD_STORAGE_WRITE_FRUID_DATA:
      pal_STORAGE_WRITE_FRUID_DATA(msg);
      break;
    case CMD_STORAGE_RSV_SDR:
      pal_STORAGE_RSV_SDR(msg);
      break;
    case CMD_STORAGE_GET_SDR:
      pal_STORAGE_GET_SDR(msg);
      break;
    default:
      printf("invalid Storage msg netfn: %x, cmd: %x\n",msg->netfn, msg->cmd);
      msg->data_len = 0;
      break;
  }
  return;
}

void IPMI_OEM_1S_handler(ipmi_msg *msg) {
  switch (msg->cmd) {
    case CMD_OEM_MSG_IN:
      break;
    case CMD_OEM_MSG_OUT:
      pal_OEM_MSG_OUT(msg);
      break;
    case CMD_OEM_GET_GPIO:
      pal_OEM_GET_GPIO(msg);
      break;
    case CMD_OEM_SET_GPIO:
      pal_OEM_SET_GPIO(msg);
      break;
    case CMD_OEM_GET_GPIO_CONFIG:
      break;
    case CMD_OEM_SET_GPIO_CONFIG:
      break;
    case CMD_OEM_SENSOR_POLL_EN:
      pal_OEM_SENSOR_POLL_EN(msg);
      break;
    case CMD_OEM_FW_UPDATE:
      pal_OEM_FW_UPDATE(msg);
      break;
    case CMD_OEM_GET_SET_GPIO:
      pal_OEM_GET_SET_GPIO(msg);
      break;
    case CMD_OEM_I2C_DEV_SCAN: // debug command
      pal_OEM_I2C_DEV_SCAN(msg);
      break;
    default:
      printf("invalid OEM msg netfn: %x, cmd: %x\n",msg->netfn, msg->cmd);
      msg->data_len = 0;
      break;

  }
  return;
}

ipmi_error IPMI_handler(ipmi_msg_cfg *msg_cfg) {
  uint8_t i;  

  osMutexAcquire(IPMI_mutex, osWaitForever);

  if (DEBUG_IPMI) {
    printf("IPMI_handler[%d]: netfn: %x\n",msg_cfg->buffer.data_len, msg_cfg->buffer.netfn);
    for (i = 0; i < msg_cfg->buffer.data_len; i++) {
      printf(" 0x%2x",msg_cfg->buffer.data[i]);
    }
    printf("\n");
  }

  msg_cfg->buffer.completion_code = CC_INVALID_CMD;
  switch (msg_cfg->buffer.netfn) {
    case NETFN_CHASSIS_REQ:
      //IPMI_CHASSIS_handler;
      break;
    case NETFN_BRIDGE_REQ:
      //IPMI_BRIDGE_handler();
      break;
    case NETFN_SENSOR_REQ:
      IPMI_SENSOR_handler(&msg_cfg->buffer);
      break;
    case NETFN_APP_REQ:
      IPMI_APP_handler(&msg_cfg->buffer); 
      break;
    case NETFN_FIRMWARE_REQ:
      break;
    case NETFN_STORAGE_REQ:
      IPMI_Storage_handler(&msg_cfg->buffer);
      break;
    case NETFN_TRANSPORT_REQ:  
      break;
    case NETFN_DCMI_REQ:
      break;
    case NETFN_NM_REQ:
      break;
    case NETFN_OEM_REQ:
      break;
    case NETFN_OEM_1S_REQ:
      if ( (msg_cfg->buffer.data[0] | (msg_cfg->buffer.data[1] << 8) | (msg_cfg->buffer.data[2] << 16) ) == WW_IANA_ID ) {
        memcpy(&msg_cfg->buffer.data[0], &msg_cfg->buffer.data[3], msg_cfg->buffer.data_len);
        msg_cfg->buffer.data_len -= 3;
        IPMI_OEM_1S_handler(&msg_cfg->buffer);
        break;
      } else if ( pal_is_not_return_cmd(msg_cfg->buffer.netfn, msg_cfg->buffer.cmd) ) {
        msg_cfg->buffer.completion_code = CC_INVALID_IANA;
        IPMI_OEM_1S_handler(&msg_cfg->buffer); // Due to command not returning, enter command handler and return with other invalid CC
        break;
      } else {
        msg_cfg->buffer.completion_code = CC_INVALID_IANA;
        msg_cfg->buffer.data_len = 0;
        break;
      }
    default:  // invalid net function
      printf("invalid msg netfn: %x, cmd: %x\n",msg_cfg->buffer.netfn, msg_cfg->buffer.cmd);
      msg_cfg->buffer.data_len = 0;
      break;
  }

  if ( pal_is_not_return_cmd(msg_cfg->buffer.netfn, msg_cfg->buffer.cmd) ) {
    vPortFree(msg_cfg);
  } else {
    ipmb_error status;

    if (msg_cfg->buffer.completion_code != CC_SUCCESS) {
      msg_cfg->buffer.data_len = 0;
    } else if(msg_cfg->buffer.netfn == NETFN_OEM_1S_REQ) {
      uint8_t copy_data[msg_cfg->buffer.data_len];
      memcpy(&copy_data[0], &msg_cfg->buffer.data[0], msg_cfg->buffer.data_len);
      memcpy(&msg_cfg->buffer.data[3], &copy_data[0], msg_cfg->buffer.data_len);
      msg_cfg->buffer.data_len += 3;
      msg_cfg->buffer.data[0] = WW_IANA_ID & 0xFF;
      msg_cfg->buffer.data[1] = (WW_IANA_ID >> 8) & 0xFF;
      msg_cfg->buffer.data[2] = (WW_IANA_ID >> 16) & 0xFF;
    }

    if (msg_cfg->buffer.InF_source == BMC_USB_IFs ) {
      ;
    } else {
      status = ipmb_send_response(&msg_cfg->buffer, IPMB_inf_index_map[msg_cfg->buffer.InF_source]);
      if (status != ipmb_error_success) {
        printf("IPMI_handler send IPMB resp fail status: %x",status);
      }
    }
    vPortFree(msg_cfg);
  }

  osMutexRelease(IPMI_mutex);
  return ipmi_error_success;

}

void ipmi_init(void) {
  IPMI_mutex = osMutexNew(&IPMI_Mutex_attr);
  if (IPMI_mutex == NULL) {
    printf("IPMI_mutex create fail\n");
  }

  ipmb_init();
}

