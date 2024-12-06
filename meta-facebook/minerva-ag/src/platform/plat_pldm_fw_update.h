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
	AG_COMPNT_BIC,
	AG_COMPNT_OSFP_P3V3,
	AG_COMPNT_CPU_P0V85_PVDD,
	AG_COMPNT_CPU_P0V75_PVDD_CH_N,
	AG_COMPNT_CPU_P0V75_PVDD_CH_S,
	AG_COMPNT_CPU_P0V75_TRVDD_ZONEA,
	AG_COMPNT_CPU_P0V75_TRVDD_ZONEB,
	AG_COMPNT_CPU_P1V1_VDDC_HBM0_2_4,
	AG_COMPNT_CPU_P0V9_TRVDD_ZONEA,
	AG_COMPNT_CPU_P0V9_TRVDD_ZONEB,
	AG_COMPNT_CPU_P1V1_VDDC_HBM1_3_5,
	AG_COMPNT_CPU_P0V8_VDDA_PCIE,
};

void load_pldmupdate_comp_config(void);
void clear_pending_version(uint8_t activate_method);
bool find_sensor_id_and_name_by_firmware_comp_id(uint8_t comp_identifier, uint8_t *sensor_id,
						 char *sensor_name);
int get_aegis_vr_compnt_mapping_sensor_table_count(void);

#endif /* _PLAT_FWUPDATE_H_ */
