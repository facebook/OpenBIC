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

typedef enum {
	DELTA_UBC_AND_MPS_VR,
	DELTA_UBC_AND_RNS_VR,
	MPS_UBC_AND_MPS_VR,
	MPS_UBC_AND_RNS_VR,
	FLEX_BMR313_UBC_AND_MPS_VR,
	FLEX_BMR313_UBC_AND_RNS_VR,
	FLEX_BMR316_UBC_AND_MPS_VR,
	FLEX_BMR316_UBC_AND_RNS_VR,
	LUXSHURE_UBC_AND_MPS_VR,
	LUXSHURE_UBC_AND_RNS_VR,
	DELTA_S54SS4P180PMDCF_UBC_AND_MPS_VR,
	DELTA_S54SS4P180PMDCF_UBC_AND_RNS_VR,
	VR_VENDOR_UNKNOWN,
} ag_vr_vendor_type_t;

typedef enum {
	VR_MPS_MP2971_MP2891,
	VR_MPS_MP2971_MP29816A,
	VR_RNS_ISL69260_RAA228238,
	VR_RNS_ISL69260_RAA228249,
	VR_UNKNOWN,
} ag_vr_type_t;

typedef enum {
	UBC_DELTA_U50SU4P180PMDAFC,
	UBC_FLEX_BMR313,
	UBC_MPS_MPC12109,
	UBC_FLEX_BMR316,
	UBC_LUXSHURE_LX6301,
	UBC_DELTA_S54SS4P180PMDCF,
	UBC_UNKNOWN,
} ag_ubc_type_t;

typedef enum {
	TMP_TMP432,
	TMP_EMC1413,
	TMP_TYPE_UNKNOWN,
} ag_tmp_type_t;

typedef enum {
	FAB1_EVT,
	FAB2_DVT,
	FAB3_DVT2,
	FAB4_DVT2,
	FAB4_PVT,
	TBD_MP,
	BOARD_STAGE_UNKNOWN,
} ag_board_stage_t;

typedef enum {
	MINERVA_EVB_BD,
	MINERVA_AEGIS_BD,
	BOARD_TYPE_UNKNOWN,
} ag_board_type_t;

typedef enum {
	MB_NOT_PRESENT,
	MB_PRESENT,
	MB_TYPE_UNKNOWN,
} ag_mb_type_t;

void init_platform_config();
uint8_t get_vr_type();
uint8_t get_ubc_type();
uint8_t get_board_stage();
uint8_t get_board_type();
uint8_t get_tmp_type();
uint8_t get_mb_type();
bool plat_read_cpld(uint8_t offset, uint8_t *data);

#endif
