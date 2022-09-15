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

#ifndef OEM_HANDLER_H
#define OEM_HANDLER_H

#include "ipmi.h"

uint8_t get_hsc_pwr_reading(int *reading);

void OEM_NM_SENSOR_READ(ipmi_msg *msg);

#ifdef CONFIG_ESPI
void OEM_SET_SYSTEM_GUID(ipmi_msg *msg);
#endif

#ifdef ENABLE_FAN
void OEM_SET_FAN_DUTY_MANUAL(ipmi_msg *msg);
void OEM_GET_SET_FAN_CTRL_MODE(ipmi_msg *msg);
#endif

void OEM_GET_MB_INDEX(ipmi_msg *msg);
void OEM_CABLE_DETECTION(ipmi_msg *msg);
void IPMI_OEM_handler(ipmi_msg *msg);

#endif
