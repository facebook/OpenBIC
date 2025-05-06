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
#define IPMI_EVENT_OFFSET_SYS_EXPA_CLOCK_BUFFER 0x88

// NE = Not Enable (for exp bic power on handler)
#define IPMI_EVENT_OFFSET_SYS_NE_FM_EXP_MAIN_PWR_EN 0x89
#define IPMI_EVENT_OFFSET_SYS_NE_PWRGD_P12V_MAIN 0x90
#define IPMI_EVENT_OFFSET_SYS_NE_OPB_BIC_MAIN_PWR_EN_R 0x91
#define IPMI_EVENT_OFFSET_SYS_NE_OPA_PWRGD_P1V8_VR 0x92
#define IPMI_EVENT_OFFSET_SYS_NE_OPA_PWRGD_P0V9_VR 0x93
#define IPMI_EVENT_OFFSET_SYS_NE_OPA_PWRGD_EXP_PWR 0x94
#define IPMI_EVENT_OFFSET_SYS_NE_OPA_CLKBUF_RTM_OE_N 0x95
#define IPMI_EVENT_OFFSET_SYS_NE_OPA_RESET_BIC_RTM_N 0x96
#define IPMI_EVENT_OFFSET_SYS_NE_OPA_PERST_BIC_RTM_N 0x97
#define IPMI_EVENT_OFFSET_SYS_NE_E1S_PRESENT 0x98
#define IPMI_EVENT_OFFSET_SYS_NE_E1S_P12V_EFUSE_PWRG 0x99
#define IPMI_EVENT_OFFSET_SYS_NE_E1S_P3V3_EFUSE_PWRG 0x9A
#define IPMI_EVENT_OFFSET_SYS_NE_E1S_CLKBUF_OE_EN 0x9B
#define IPMI_EVENT_OFFSET_SYS_NE_E1S_PCIE_RESET 0x9C
// ND = Not Disable (for exp bic power off handler)
#define IPMI_EVENT_OFFSET_SYS_ND_E1S_PWR_OFF 0x9D
#define IPMI_EVENT_OFFSET_SYS_ND_OPA_PERST_BIC_RTM_N 0x9E
#define IPMI_EVENT_OFFSET_SYS_ND_OPA_RESET_BIC_RTM_N 0x9F
#define IPMI_EVENT_OFFSET_SYS_ND_OPA_CLKBUF_RTM_OE_N 0xA0
#define IPMI_EVENT_OFFSET_SYS_ND_OPA_PWRGD_EXP_PWR 0xA1
#define IPMI_EVENT_OFFSET_SYS_ND_OPA_PWRGD_P0V9_VR 0xA2
#define IPMI_EVENT_OFFSET_SYS_ND_OPB_BIC_MAIN_PWR_EN_R 0xA3
#define IPMI_EVENT_OFFSET_SYS_ND_E1S_PCIE_RESET 0xA4
#define IPMI_EVENT_OFFSET_SYS_ND_CLKBUF_OE_EN 0xA5
#define IPMI_EVENT_OFFSET_SYS_ND_E1S_P12V_EFUSE_PWRG 0xA6
#define IPMI_EVENT_OFFSET_SYS_ND_E1S_P3V3_EFUSE_PWRG 0xA7

void send_system_status_event(uint8_t event_type, uint8_t error_type, uint8_t device_index);

#endif
