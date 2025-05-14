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

#ifndef _PLAT_FWUPDATE_H_
#define _PLAT_FWUPDATE_H_

#include <stdbool.h>
#include <stdint.h>
#include "pldm_firmware_update.h"

enum FIRMWARE_COMPONENT {
	RB_COMPNT_BIC,
	RB_COMPNT_MEDHA0_VDD,
	RB_COMPNT_MEDHA1_VDD,
	RB_COMPNT_P075_OWL_VDD,
	RB_COMPNT_VDDQC_HBM1_HBM3_HBM5_HBM7,
	RB_COMPNT_VDDPHY_HBM1_HBM3_HBM5_HBM7,
	RB_COMPNT_P0V75_TRVDD,
	RB_COMPNT_HAMSA_VDD,
	RB_COMPNT_VPP_HBM0_HBM2_HBM4_HBM6,
	RB_COMPNT_P0V85_VDDA_PCIE,
	RB_COMPNT_VDDQC_HBM0_HBM2_HBM4_HBM6,
	RB_COMPNT_VDDPHY_HBM0_HBM2_HBM4_HBM6,
};

void load_pldmupdate_comp_config(void);
void clear_pending_version(uint8_t activate_method);
bool find_sensor_id_and_name_by_firmware_comp_id(uint8_t comp_identifier, uint8_t *sensor_id,
						 char *sensor_name);
int get_aegis_vr_compnt_mapping_sensor_table_count(void);

#endif /* _PLAT_FWUPDATE_H_ */
