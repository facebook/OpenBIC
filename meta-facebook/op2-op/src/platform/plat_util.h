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

#ifndef PLAT_UTIL_H
#define PLAT_UTIL_H

#include <stdint.h>

// Platform Event
#define IPMI_EVENT_OFFSET_STS_E1S_PRESENT 0x80
#define IPMI_EVENT_OFFSET_SYS_E1S_P12V_FAULT 0x83
#define IPMI_EVENT_OFFSET_SYS_E1S_P3V3_FAULT 0x84
#define IPMI_EVENT_OFFSET_SYS_INA233_ALERT 0x86

void send_system_status_event(uint8_t event_type, uint8_t error_type, uint8_t device_index);

#endif
