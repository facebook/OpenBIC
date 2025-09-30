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

#define VR_MAX_NUM 2
#define VR_MUTEX_LOCK_TIMEOUT_MS 1000

#include "plat_pldm_sensor.h"

enum VR_INDEX_E {
	VR_INDEX_E_P0V895,
	VR_INDEX_E_P0V825,
	VR_INDEX_MAX,
};

enum VR_RAIL_E {
	VR_RAIL_E_P0V895_PEX = 0,
	VR_RAIL_E_P0V825_A0,
	VR_RAIL_E_P0V825_A1,
	VR_RAIL_E_P0V825_A2,
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

enum PLAT_TEMP_INDEX_E {
	TEMP_INDEX_THERMAL_SENSOR_1_TEMP_C,
	TEMP_INDEX_THERMAL_SENSOR_2_TEMP_C,
	TEMP_INDEX_MAX,
};

typedef struct vr_mapping_sensor {
	uint8_t index;
	uint8_t sensor_id;
	uint8_t *sensor_name;
	int peak_value;
} vr_mapping_sensor;

typedef struct vr_vout_user_settings {
	uint16_t vout[VR_RAIL_E_MAX];
} vr_vout_user_settings;

extern vr_vout_user_settings user_settings;

typedef struct vr_mapping_status {
	uint8_t index;
	uint16_t pmbus_reg;
	uint8_t *vr_status_name;
} vr_mapping_status;

extern vr_mapping_sensor vr_rail_table[];

typedef struct _vr_pre_proc_arg {
	void *mutex;
	uint8_t vr_page;
} vr_pre_proc_arg;

typedef struct temp_mapping_sensor {
	uint8_t index;
	uint8_t sensor_id;
	uint8_t *sensor_name;
} temp_mapping_sensor;

extern vr_pre_proc_arg vr_pre_read_args[];

typedef struct temp_threshold_mapping_sensor {
	uint8_t temp_index_threshold_type; //PLAT_TEMP_INDEX_THRESHOLD_TYPE_E
	uint8_t temp_threshold_type;
	uint8_t sensor_id;
	uint8_t *temp_threshold_name;
} temp_threshold_mapping_sensor;

extern temp_threshold_mapping_sensor temp_threshold_table[];
extern pex90144_init_arg pex_sensor_init_args[];

bool temp_sensor_rail_name_get(uint8_t rail, uint8_t **name);
bool temp_sensor_rail_enum_get(uint8_t *name, uint8_t *num);
bool pre_vr_read(sensor_cfg *cfg, void *args);
bool post_vr_read(sensor_cfg *cfg, void *args, int *const reading);
void *vr_mutex_get(enum VR_INDEX_E vr_index);
void vr_mutex_init(void);
bool vr_rail_name_get(uint8_t rail, uint8_t **name);
bool vr_status_name_get(uint8_t rail, uint8_t **name);
bool vr_rail_enum_get(uint8_t *name, uint8_t *num);
bool vr_status_enum_get(uint8_t *name, uint8_t *num);
bool plat_get_vr_status(uint8_t rail, uint8_t vr_status_rail, uint16_t *vr_status);
bool plat_clear_vr_status(uint8_t rail);

#endif
