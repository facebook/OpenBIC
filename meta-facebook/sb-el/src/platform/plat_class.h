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

#include "stdint.h"

enum EL_VR_VENDER_MODULE {
	DELTA_UBC_AND_MPS_VR,
	DELTA_UBC_AND_RNS_VR,
	MPS_UBC_AND_MPS_VR,
	MPS_UBC_AND_RNS_VR,
	LUXSHURE_UBC_AND_MPS_VR,
	LUXSHURE_UBC_AND_RNS_VR,
	VENDOR_TYPE_UNKNOWN,
};

enum VR_MODULE {
	VR_MODULE_MPS,
	VR_MODULE_RNS,
	VR_MODULE_UNKNOWN,
};

enum UBC_MODULE {
	UBC_MODULE_DELTA,
	UBC_MODULE_MPS,
	UBC_MODULE_FLEX,
	UBC_MODULE_LUXSHARE,
	UBC_MODULE_UNKNOWN,
};

enum TMP_MODULE {
	TMP_TMP432,
	TMP_TYPE_UNKNOWN,
};

enum ASIC_BOARD_ID {
	ASIC_BOARD_ID_RSVD1,
	ASIC_BOARD_ID_RSVD2,
	ASIC_BOARD_ID_ELECTRA,
	ASIC_BOARD_ID_EVB,
	ASIC_BOARD_ID_UNKNOWN,
};

enum REV_ID {
	REV_ID_EVT1A,
	REV_ID_EVT1B,
	REV_ID_EVT2,
	REV_ID_DVT,
	REV_ID_PVT,
	REV_ID_MP,
	REV_ID_RSVD1,
	REV_ID_RSVD2,
	MAX_REV_ID,
};

void init_plat_config();
uint8_t get_vr_module();
uint8_t get_ubc_module();
uint8_t get_mmc_slot();
uint8_t get_asic_board_id();
uint8_t get_board_rev_id();
uint8_t get_tray_location();
bool plat_cpld_eerprom_read(uint8_t *data, uint16_t offset, uint8_t len);
#endif
