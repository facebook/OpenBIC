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

#ifndef USB_H
#define USB_H

#ifdef CONFIG_USB

#define DEBUG_USB 0
#define USB_HANDLER_STACK_SIZE 2048
#define RX_BUFF_SIZE 64
#define RING_BUF_SIZE 576

#define FWUPDATE_HEADER_SIZE 12
#define SIZE_NETFN_CMD 2

#include "ipmb.h"

void usb_targetdev_init(void);
void usb_write_by_ipmi(ipmi_msg *ipmi_resp);
void usb_dev_init(void);

#endif

#endif
