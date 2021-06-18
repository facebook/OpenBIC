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
#include <string.h>
#include "board_device.h"
#include "objects.h"
#include "ipmi.h"

#define DEBUG_USB 0

void pal_usb_handler(uint8_t *rx_buff,int rx_len) {
  ipmi_msg_cfg *current_msg_rx;
  current_msg_rx = pvPortMalloc(sizeof(ipmi_msg_cfg));

  if (DEBUG_USB) {
    printf("USB: len %d, req: %x %x ID: %x %x %x target: %x offset: %x %x %x %x len: %x %x\n", rx_len, rx_buff[0], rx_buff[1], rx_buff[2], rx_buff[3], rx_buff[4], rx_buff[5], rx_buff[6], rx_buff[7], rx_buff[8], rx_buff[9], rx_buff[10], rx_buff[11]);
  }

  current_msg_rx->buffer.netfn = rx_buff[0] >> 2;
  current_msg_rx->buffer.cmd = rx_buff[1];
  current_msg_rx->buffer.InF_source = BMC_USB_IFs;
  current_msg_rx->buffer.data_len = rx_len - 2; // skip netfn, cmd
  memcpy( &current_msg_rx->buffer.data[0], &rx_buff[2], current_msg_rx->buffer.data_len);

  IPMI_handler(current_msg_rx);

  rx_buff[0] = (current_msg_rx->buffer.netfn + 1) << 2;
  rx_buff[1] = current_msg_rx->buffer.cmd;
  rx_buff[2] = current_msg_rx->buffer.completion_code;
  rx_len = current_msg_rx->buffer.data_len + 3; // netfn + cmd + complt code + data
  memcpy( &rx_buff[3], &current_msg_rx->buffer.data[0], current_msg_rx->buffer.data_len);

  if (DEBUG_USB) {
    printf("USB: len %d, buf: %x %x %x \n", rx_len, rx_buff[0], rx_buff[1], rx_buff[2]);
  }
  
  return;
}
