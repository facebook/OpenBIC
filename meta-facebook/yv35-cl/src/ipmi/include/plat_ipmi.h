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

#ifndef PLAT_IPMI_H
#define PLAT_IPMI_H

#include <stdbool.h>
#include <stdint.h>

// IPMI OEM event types for VR power fault
#define IPMI_EVENT_VR_PWR_FAULT_PVCCD_HV 0xA9
#define IPMI_EVENT_VR_PWR_FAULT_PVCCFA_EHV 0xAD
#define IPMI_EVENT_VR_PWR_FAULT_PVCCFA_EHV_FIVRA 0xAE
#define IPMI_EVENT_VR_PWR_FAULT_PVCCINFAON 0xAF
#define IPMI_EVENT_VR_PWR_FAULT_VCCIN 0xB0

enum REQ_GET_CARD_TYPE {
	GET_1OU_CARD_TYPE = 0x0,
	GET_2OU_CARD_TYPE,
};

#endif
