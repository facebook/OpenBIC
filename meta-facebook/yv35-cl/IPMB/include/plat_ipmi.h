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

#ifndef PLAT_IPMI_H
#define PLAT_IPMI_H

// OEM Command Codes 
enum {
  CMD_OEM_MSG_IN = 0x1,
  CMD_OEM_MSG_OUT = 0x2,
  CMD_OEM_GET_GPIO = 0x3,
  CMD_OEM_SET_GPIO = 0x4,
  CMD_OEM_GET_GPIO_CONFIG = 0x5,
  CMD_OEM_SET_GPIO_CONFIG = 0x6,
  CMD_OEM_SENSOR_POLL_EN = 0x7,
  CMD_OEM_FW_UPDATE = 0x9,
  CMD_OEM_GET_SET_GPIO = 0x41,
// Debug command
  CMD_OEM_I2C_DEV_SCAN = 0x60,
};


#endif
