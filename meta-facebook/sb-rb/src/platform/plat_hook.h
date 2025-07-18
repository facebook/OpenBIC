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

#ifndef PLAT_HOOK_H
#define PLAT_HOOK_H

#include "sensor.h"

#define VR_MAX_NUM 12
#define VR_MUTEX_LOCK_TIMEOUT_MS 1000

#include "plat_pldm_sensor.h"

enum VR_INDEX_E {
	VR_INDEX_E_1 = 0,
	VR_INDEX_E_2,
	VR_INDEX_E_3,
	VR_INDEX_E_4,
	VR_INDEX_E_5,
	VR_INDEX_E_6,
	VR_INDEX_E_7,
	VR_INDEX_E_8,
	VR_INDEX_E_9,
	VR_INDEX_E_10,
	VR_INDEX_E_11,
	VR_INDEX_E_12,
	VR_INDEX_MAX,
};

enum VR_RAIL_E {
	VR_RAIL_E_ASIC_P0V85_MEDHA0_VDD = 0,
	VR_RAIL_E_ASIC_P0V85_MEDHA1_VDD,
	VR_RAIL_E_ASIC_P0V9_OWL_E_TRVDD,
	VR_RAIL_E_ASIC_P0V75_OWL_E_TRVDD,
	VR_RAIL_E_ASIC_P0V75_MAX_M_VDD,
	VR_RAIL_E_ASIC_P0V75_VDDPHY_HBM1357,
	VR_RAIL_E_ASIC_P0V75_OWL_E_VDD,
	VR_RAIL_E_ASIC_P0V4_VDDQL_HBM1357,
	VR_RAIL_E_ASIC_P1V1_VDDQC_HBM1357,
	VR_RAIL_E_ASIC_P1V8_VPP_HBM1357,
	VR_RAIL_E_ASIC_P0V75_MAX_N_VDD,
	VR_RAIL_E_ASIC_P0V8_HAMSA_AVDD_PCIE,
	VR_RAIL_E_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE,
	VR_RAIL_E_ASIC_P0V85_HAMSA_VDD,
	VR_RAIL_E_ASIC_P1V1_VDDQC_HBM0246,
	VR_RAIL_E_ASIC_P1V8_VPP_HBM0246,
	VR_RAIL_E_ASIC_P0V4_VDDQL_HBM0246,
	VR_RAIL_EASIC_P0V75_VDDPHY_HBM0246,
	VR_RAIL_E_ASIC_P0V75_OWL_W_VDD,
	VR_RAIL_E_ASIC_P0V75_MAX_S_VDD,
	VR_RAIL_E_ASIC_P0V9_OWL_W_TRVDD,
	VR_RAIL_E_ASIC_P0V75_OWL_W_TRVDD,
	VR_RAIL_E_MAX,
};

enum VR_STAUS_E {
	VR_STAUS_E_STATUS_BYTE = 0,
	VR_STAUS_E_STATUS_WORD,
	VR_STAUS_E_STATUS_VOUT,
	VR_STAUS_E_STATUS_IOUT,
	VR_STAUS_E_STATUS_INPUT,
	VR_STAUS_E_STATUS_TEMPERATURE,
	VR_STAUS_E_STATUS_CML,
	VR_STAUS_E_MAX,
};

typedef struct vr_mapping_status {
	uint8_t index;
	uint16_t pmbus_reg;
	uint8_t *vr_status_name;
} vr_mapping_status;

typedef struct _vr_pre_proc_arg {
	void *mutex;
	uint8_t vr_page;
} vr_pre_proc_arg;

typedef struct vr_mapping_sensor {
	uint8_t index;
	uint8_t sensor_id;
	uint8_t *sensor_name;
	int peak_value;
} vr_mapping_sensor;

bool pre_vr_read(sensor_cfg *cfg, void *args);
bool post_vr_read(sensor_cfg *cfg, void *args, int *const reading);
bool is_mb_dc_on();
void *vr_mutex_get(enum VR_INDEX_E vr_index);
void vr_mutex_init(void);
bool vr_rail_name_get(uint8_t rail, uint8_t **name);
bool vr_status_name_get(uint8_t rail, uint8_t **name);
bool vr_rail_enum_get(uint8_t *name, uint8_t *num);
bool vr_status_enum_get(uint8_t *name, uint8_t *num);
bool plat_get_vr_status(uint8_t rail, uint8_t vr_status_rail, uint16_t *vr_status);
bool plat_clear_vr_status(uint8_t rail);
#endif
