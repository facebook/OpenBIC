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
#include <zephyr.h>
#include "ipmi.h"

#define DETECT_SMI_DELAY_90S 90
#define CPLD_OFFSET_10 0x10
#define CPLD_BIT_E1S_0_12V_POWER_R_EN 5
#define CPLD_BIT_E1S_0_3V3_POWER_R_EN 6

enum GET_SET_M2_OPTION {
	DEVICE_SET_POWER_OFF = 0x00,
	DEVICE_SET_POWER_ON = 0x01,
	DEVICE_GET_POWER_STATUS = 0x03,
};

typedef enum { DEASSERT = 0, ASSERT } event_state_t;

enum cpld_reg_info_idx { CPLD_REG_INFO_IDX_0 = 0, CPLD_REG_INFO_IDX_1, CPLD_REG_INFO_IDX_MAX };

typedef enum {
	PVCCIN_CPU0 = 0,
	PVCCFA_EHV_FIVRA_CPU0 = 1,
	PVCCINFAON_CPU0 = 2,
	PVCCFA_EHV_CPU0 = 3,
	PVCCD_HV_CPU = 4,
	P1V05_PCH_STB = 5,
	P1V8_STBY = 6,
	P5V_STBY = 7,
	P3V3_STBY = 8,
	VR_P12V_E1S_0 = 9,
	VR_P3V3_E1S_0 = 10,
} vr_source_id_t;

typedef struct _vr_fault_info {
	vr_source_id_t vr_source_id;
	uint8_t cpld_reg_data_idx;
	uint8_t cpld_reg_bit;
	bool is_pmbus_vr;
	uint8_t vr_i2c_bus;
	uint8_t vr_addr;
	uint8_t vr_page;
} vr_fault_info;

typedef struct _cpld_reg_info {
	uint8_t cpld_reg_i2c_bus;
	uint8_t cpld_reg_addr;
	uint8_t cpld_reg_offset;
} cpld_reg_info;

typedef struct _add_vr_sel_info {
	bool is_init;
	uint8_t gpio_num;
	struct k_work_delayable add_sel_work;
} add_vr_sel_info;

void send_gpio_interrupt(uint8_t gpio_num);
int get_set_1ou_m2_power(ipmi_msg *msg, uint8_t device_id, uint8_t option);
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
void ISR_SMI();
void ISR_E1S_0_INA233_ALRT();
void ISR_SMB_SENSOR_LVC3_ALERT();
void ISR_VR_PWR_FAULT();
void init_vr_event_work();
#endif
