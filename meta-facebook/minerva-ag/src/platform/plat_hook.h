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

#define VR_MAX_NUM 11
#define VR_MUTEX_LOCK_TIMEOUT_MS 1000

#include "plat_pldm_sensor.h"

enum VR_INDEX_E {
	VR_INDEX_E_OSFP_P3V3 = 0,
	VR_INDEX_E_P0V85,
	VR_INDEX_E_P0V75_CH_N,
	VR_INDEX_E_P0V75_CH_S,
	VR_INDEX_E_P0V75_TRVDD_ZONEA,
	VR_INDEX_E_P0V75_TRVDD_ZONEB,
	VR_INDEX_E_P1V1_VDDC_HBM0_HBM2_HBM4,
	VR_INDEX_E_P0V9_TRVDD_ZONEA,
	VR_INDEX_E_P0V9_TRVDD_ZONEB,
	VR_INDEX_E_P1V1_VDDC_HBM1_HBM3_HBM5,
	VR_INDEX_E_VDDA_PCIE,
	VR_INDEX_MAX,
};

enum VR_RAIL_E {
	VR_RAIL_E_OSFP_P3V3 = 0,
	VR_RAIL_E_P0V85,
	VR_RAIL_E_P0V75_PVDD_CH_N,
	VR_RAIL_E_P0V75_MAX_PHY_N,
	VR_RAIL_E_P0V75_PVDD_CH_S,
	VR_RAIL_E_P0V75_MAX_PHY_S,
	VR_RAIL_E_P0V75_TRVDD_ZONEA,
	VR_RAIL_E_P1V8_VPP_HBM0_2_4,
	VR_RAIL_E_P0V75_TRVDD_ZONEB,
	VR_RAIL_E_P0V4_VDDQL_HBM0_2_4,
	VR_RAIL_E_P1V1_VDDC_HBM0_2_4,
	VR_RAIL_E_P0V75_VDDPHY_HBM0_2_4,
	VR_RAIL_E_P0V9_TRVDD_ZONEA,
	VR_RAIL_E_P1V8_VPP_HBM1_3_5,
	VR_RAIL_E_P0V9_TRVDD_ZONEB,
	VR_RAIL_E_P0V4_VDDQL_HBM1_3_5,
	VR_RAIL_E_P1V1_VDDC_HBM1_3_5,
	VR_RAIL_E_P0V75_VDDPHY_HBM1_3_5,
	VR_RAIL_E_P0V8_VDDA_PCIE,
	VR_RAIL_E_P1V2_VDDHTX_PCIE,
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

typedef struct vr_mapping_sensor {
	uint8_t index;
	uint8_t sensor_id;
	uint8_t *sensor_name;
	int peak_value;
} vr_mapping_sensor;

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

extern vr_pre_proc_arg vr_pre_read_args[];
extern mp2971_init_arg mp2971_init_args[];
extern isl69259_init_arg isl69259_init_args[];

bool plat_get_vout_min(uint8_t rail, uint16_t *millivolt);
bool plat_get_vout_max(uint8_t rail, uint16_t *millivolt);
bool plat_set_vout_min(uint8_t rail, uint16_t *millivolt);
bool plat_set_vout_max(uint8_t rail, uint16_t *millivolt);
bool pre_vr_read(sensor_cfg *cfg, void *args);
bool post_vr_read(sensor_cfg *cfg, void *args, int *const reading);
bool is_mb_dc_on();
void *vr_mutex_get(enum VR_INDEX_E vr_index);
void vr_mutex_init(void);
bool vr_rail_name_get(uint8_t rail, uint8_t **name);
bool vr_rail_enum_get(uint8_t *name, uint8_t *num);
int power_level_send_event(int ubc1_current, int ubc2_current);
bool post_ubc_read(sensor_cfg *cfg, void *args, int *reading);
void set_uart_power_event_is_enable(bool is_enable);
void pwr_level_mutex_init(void);
void set_alert_level_to_default_or_user_setting(bool is_default, int32_t user_setting);
int set_user_settings_alert_level_to_eeprom(void *user_settings, uint8_t data_length);
int get_alert_level_info(bool *is_assert, int32_t *default_value, int32_t *setting_value);
bool vr_rail_voltage_peak_get(uint8_t *name, int *peak_value);
bool vr_rail_voltage_peak_clear(uint8_t rail_index);
bool vr_vout_user_settings_get(void *user_settings);
void user_settings_init(void);
bool plat_get_vout_command(uint8_t rail, uint16_t *millivolt);
bool plat_set_vout_command(uint8_t rail, uint16_t *millivolt, bool is_default, bool is_perm);
bool plat_get_vr_status(uint8_t rail, uint8_t vr_status_rail, uint16_t *vr_status);
bool plat_clear_vr_status(uint8_t rail);
bool vr_status_name_get(uint8_t rail, uint8_t **name);
bool vr_status_enum_get(uint8_t *name, uint8_t *num);

#endif
