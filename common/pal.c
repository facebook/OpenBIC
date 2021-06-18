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
#include <stdbool.h>
#include "cmsis_os2.h"
#include "board_device.h"
#include "objects.h"
#include "cmsis_compiler.h"
#include "ipmi.h"
#include "pal.h"

/***********************************************************
 *
 * Create weak function here
 * All weak functions should be define in project for usage
 *
 * *********************************************************/

__WEAK bool pal_is_not_return_cmd(uint8_t netfn, uint8_t cmd) {
    return 0;
}

// USB
__WEAK void pal_usb_handler(uint8_t *rx_buff,int rx_len) {
  return;
}

// IPMI SENSOR 
__WEAK void pal_SENSOR_GET_SENSOR_READING(ipmi_msg *msg) {
  msg->completion_code = CC_UNSPECIFIED_ERROR;
  return;
}

// IPMI APP 
__WEAK void pal_APP_GET_DEVICE_ID(ipmi_msg *msg) {
  msg->completion_code = CC_UNSPECIFIED_ERROR;
  return;
}

__WEAK void pal_APP_WARM_RESET(ipmi_msg *msg) {
  msg->completion_code = CC_UNSPECIFIED_ERROR;
  return;
}

__WEAK void pal_APP_GET_SELFTEST_RESULTS(ipmi_msg *msg) {
  msg->completion_code = CC_UNSPECIFIED_ERROR;
  return;
}

__WEAK void pal_APP_MASTER_WRITE_READ(ipmi_msg *msg) {
  msg->completion_code = CC_UNSPECIFIED_ERROR;
  return;
}

// IPMI STORAGE
__WEAK void pal_STORAGE_GET_FRUID_INFO(ipmi_msg *msg) {
  msg->completion_code = CC_UNSPECIFIED_ERROR;
  return;
}

__WEAK void pal_STORAGE_READ_FRUID_DATA(ipmi_msg *msg) {
  msg->completion_code = CC_UNSPECIFIED_ERROR;
  return;
}

__WEAK void pal_STORAGE_WRITE_FRUID_DATA(ipmi_msg *msg) {
  msg->completion_code = CC_UNSPECIFIED_ERROR;
  return;
}

__WEAK void pal_STORAGE_RSV_SDR(ipmi_msg *msg) {
  msg->completion_code = CC_UNSPECIFIED_ERROR;
  return;
}

__WEAK void pal_STORAGE_GET_SDR(ipmi_msg *msg) {
  msg->completion_code = CC_UNSPECIFIED_ERROR;
  return;
}

// IPMI OEM 
__WEAK void pal_OEM_MSG_OUT(ipmi_msg *msg) {
  msg->completion_code = CC_UNSPECIFIED_ERROR;
  return;
}

__WEAK void OEM_GET_GPIO(ipmi_msg *msg) {
  msg->completion_code = CC_UNSPECIFIED_ERROR;
  return;
}

__WEAK void OEM_SET_GPIO(ipmi_msg *msg) {
  msg->completion_code = CC_UNSPECIFIED_ERROR;
  return;
}

__WEAK void pal_OEM_SENSOR_POLL_EN(ipmi_msg *msg) {
  msg->completion_code = CC_UNSPECIFIED_ERROR;
  return;
}

__WEAK void pal_OEM_FW_UPDATE(ipmi_msg *msg) {
  msg->completion_code = CC_UNSPECIFIED_ERROR;
  return;

}

__WEAK void pal_OEM_GET_SET_GPIO(ipmi_msg *msg) {
  msg->completion_code = CC_UNSPECIFIED_ERROR;
  return;
}

__WEAK void pal_OEM_I2C_DEV_SCAN(ipmi_msg *msg) {
  msg->completion_code = CC_UNSPECIFIED_ERROR;
  return;
}

// init
__WEAK void pal_I2C_init(void) {
  return;
}

__WEAK void pal_BIC_init(void) {
  return;
}

// sensor accessible
__WEAK bool pal_stby_access(uint8_t snr_num) {
  return 1;
}

__WEAK bool pal_DC_access(uint8_t snr_num) {
  return 0;
}

