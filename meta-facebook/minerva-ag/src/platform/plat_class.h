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

typedef enum {
	DELTA_UBC_AND_MPS_VR,
	DELTA_UBC_AND_RNS_VR,
	MPS_UBC_AND_MPS_VR,
	MPS_UBC_AND_RNS_VR,
	FLEX_UBC_AND_MPS_VR,
	FLEX_UBC_AND_RNS_VR,
	VR_VENDOR_UNKNOWN,
} ag_vr_vendor_type_t;

typedef enum {
	VR_MPS_MP2971_MP2891,
	VR_RNS_ISL69260_RAA228238,
	VR_UNKNOWN,
} ag_vr_type_t;

typedef enum {
	UBC_DELTA_U50SU4P180PMDAFC,
	UBC_FLEX_BMR313,
	UBC_MPS_MPC12109,
	UBC_UNKNOWN,
} ag_ubc_type_t;

void init_platform_config();
uint8_t get_vr_type();
uint8_t get_ubc_type();

#endif
