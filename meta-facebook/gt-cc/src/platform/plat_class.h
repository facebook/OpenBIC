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

#ifndef PLAT_CLASS_H
#define PLAT_CLASS_H

#include <stdbool.h>
#include <stdint.h>

/* ADC channel number */
enum GT_COMPONENT_TYPE_ADC_CHANNEL {
	HSC_TYPE_ADC_CHANNEL = 11,
	VR_TYPE_ADC_CHANNEL = 12,
	POWER_IC_TYPE_ADC_CHANNEL = 13,
};

typedef enum {
	GT_STAGE_EVT = 1,
	GT_STAGE_EVT2 = 2,
	GT_STAGE_DVT = 3,
	GT_STAGE_PVT = 4,
} GT_STAGE_REVISION_ID;

typedef enum {
	VR_UNKNOWN,
	VR_RNS_ISL69259,
	VR_INF_XDPE12284,
	VR_MPS_MPS2971,
} gt_vr_type_t;

typedef enum {
	POWER_IC_UNKNOWN,
	POWER_IC_ISL28022,
	POWER_IC_INA230,
} gt_power_monitor_ic_type_t;

typedef enum {
	HSC_UNKNOWN,
	HSC_MP5990,
	HSC_LTC4282,
	HSC_LTC4286,
} gt_hsc_type_t;

enum GT_FIRMWARE_COMPONENT {
	GT_COMPNT_VR0,
	GT_COMPNT_VR1,
	GT_COMPNT_BIC,
	GT_COMPNT_PEX0,
	GT_COMPNT_PEX1,
	GT_COMPNT_PEX2,
	GT_COMPNT_PEX3,
	GT_COMPNT_CPLD,
	GT_COMPNT_NIC0,
	GT_COMPNT_NIC1,
	GT_COMPNT_NIC2,
	GT_COMPNT_NIC3,
	GT_COMPNT_NIC4,
	GT_COMPNT_NIC5,
	GT_COMPNT_NIC6,
	GT_COMPNT_NIC7,
	GT_COMPNT_MAX,
};

bool get_adc_voltage(int channel, float *voltage);
GT_STAGE_REVISION_ID get_stage_by_rev_id();
uint8_t get_hsc_type();
uint8_t get_vr_type();
uint8_t get_power_moniter_ic_type();
void init_platform_config();

#endif