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

#ifndef PLAT_FUNC_H
#define PLAT_FUNC_H

#include <stdint.h>
#include "ipmi.h"
//#include "hal_i2c.h"

#define DETECT_SMI_DELAY_90S 90

#define VR_PWR_FAULT_DELAY_MS 10
#define CPLD_VR_FAULT_REG 0x01
// VR Address
// Page 0: PVCCIN / Page 1: FIVRA
#define PVCCIN_FIVRA_ADDR (0xC0 >> 1)
// Page 0: PVCCD
#define PVCCD_ADDR (0xC4 >> 1)
// Page 0: FAON / Page 1: EHV
#define FAON_EHV_ADDR (0xEC >> 1)

enum GET_SET_M2_OPTION {
	DEVICE_SET_POWER_OFF = 0x00,
	DEVICE_SET_POWER_ON = 0x01,
	DEVICE_GET_POWER_STATUS = 0x03,
};

void send_gpio_interrupt(uint8_t gpio_num);
void init_vr_pwr_fault_work();
void ISR_PLTRST();
void ISR_SLP3();
void ISR_DC_ON();
void ISR_BMC_PRDY();
void ISR_PWRGD_CPU();
void ISR_CATERR();
void ISR_DBP_PRSNT();
void ISR_POST_COMPLETE();
void ISR_SOC_THMALTRIP();
void ISR_SYS_THROTTLE();
void ISR_PCH_THMALTRIP();
void ISR_HSC_OC();
void ISR_CPU_MEMHOT();
void ISR_CPUVR_HOT();
void ISR_PCH_PWRGD();
void ISR_MB_THROTTLE();
void ISR_HSC_THROTTLE();
void ISR_FM_THROTTLE();
void ISR_RMCA();
void ISR_CPU_VPP_INT();
void ISR_NMI();
void ISR_RST_PLTRST_PLD();
void ISR_SMI();
void ISR_VR_PWR_FAULT();

#endif
