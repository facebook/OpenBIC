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
#include "plat_pldm_sensor.h"

#define VR_MUTEX_LOCK_TIMEOUT_MS 1000

extern mp2971_init_arg mp2971_init_args[];

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
	VR_INDEX_E_13, // P3V3 OSFP
	VR_INDEX_MAX,
};

enum VR_RAIL_E {
	ASIC_P0V4_VDDQL_HBM0246_VOLT_V = 0,
	ASIC_P0V4_VDDQL_HBM1357_VOLT_V,
	ASIC_P0V75_MAX_M_VDD_VOLT_V,
	ASIC_P0V75_MAX_N_VDD_VOLT_V,
	ASIC_P0V75_MAX_S_VDD_VOLT_V,
	ASIC_P0V75_OWL_E_TRVDD_VOLT_V,
	ASIC_P0V75_OWL_E_VDD_VOLT_V,
	ASIC_P0V75_OWL_W_TRVDD_VOLT_V,
	ASIC_P0V75_OWL_W_VDD_VOLT_V,
	ASIC_P0V75_VDDPHY_HBM0246_VOLT_V,
	ASIC_P0V75_VDDPHY_HBM1357_VOLT_V,
	ASIC_P0V85_HAMSA_VDD_VOLT_V,
	ASIC_P0V85_MEDHA0_VDD_VOLT_V,
	ASIC_P0V85_MEDHA1_VDD_VOLT_V,
	ASIC_P0V8_HAMSA_AVDD_PCIE_VOLT_V,
	ASIC_P0V9_OWL_E_TRVDD_VOLT_V,
	ASIC_P0V9_OWL_W_TRVDD_VOLT_V,
	ASIC_P1V1_VDDQC_HBM0246_VOLT_V,
	ASIC_P1V1_VDDQC_HBM1357_VOLT_V,
	ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_VOLT_V,
	ASIC_P1V8_VPP_HBM0246_VOLT_V,
	ASIC_P1V8_VPP_HBM1357_VOLT_V,
	P3V3_OSFP_VOLT_V,
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

enum PLAT_STRAP_INDEX_E {
	STRAP_INDEX_SOC_JTAG_MUX_SEL_0_3,
	STRAP_INDEX_SOC_DFT_TAP_EN_L,
	STRAP_INDEX_SOC_ATPG_MODE_L,
	STRAP_INDEX_SOC_PAD_TRI_N,
	STRAP_INDEX_SOC_CORE_TAP_CTRL_L,
	STRAP_INDEX_SOC_BOOT_SOURCE_0_4,
	STRAP_INDEX_SOC_BOOT_SOURCE_5_6,
	STRAP_INDEX_SOC_BOOT_SOURCE_7,
	STRAP_INDEX_SOC_GPIO2,
	STRAP_INDEX_S_OWL_BOOT_SOURCE_0_7,
	STRAP_INDEX_N_OWL_BOOT_SOURCE_0_7,
	STRAP_INDEX_S_OWL_PAD_TRI_N,
	STRAP_INDEX_S_OWL_ATPG_MODE_L,
	STRAP_INDEX_S_OWL_DFT_TAP_EN_L,
	STRAP_INDEX_S_OWL_CORE_TAP_CTRL_L,
	STRAP_INDEX_N_OWL_PAD_TRI_N,
	STRAP_INDEX_N_OWL_ATPG_MODE_L,
	STRAP_INDEX_N_OWL_DFT_TAP_EN_L,
	STRAP_INDEX_N_OWL_CORE_TAP_CTRL_L,
	STRAP_INDEX_S_OWL_JTAG_MUX_SEL_0_3,
	STRAP_INDEX_N_OWL_JTAG_MUX_SEL_0_3,
	STRAP_INDEX_S_OWL_UART_MUX_SEL_0_2,
	STRAP_INDEX_N_OWL_UART_MUX_SEL_0_2,
	STRAP_INDEX_MAX,
};
typedef struct vr_vout_range_user_settings_struct {
	uint16_t default_vout_max[STRAP_INDEX_MAX];
	uint16_t default_vout_min[STRAP_INDEX_MAX];
	uint16_t change_vout_max[STRAP_INDEX_MAX];
	uint16_t change_vout_min[STRAP_INDEX_MAX];
} vr_vout_range_user_settings_struct;

extern vr_vout_range_user_settings_struct vout_range_user_settings;
extern vr_mapping_sensor vr_rail_table[];
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
bool plat_get_vout_command(uint8_t rail, uint16_t *millivolt);
bool plat_set_vout_command(uint8_t rail, uint16_t *millivolt, bool is_default, bool is_perm);
bool vr_rail_voltage_peak_get(uint8_t *name, int *peak_value);
bool vr_rail_voltage_peak_clear(uint8_t rail_index);
bool plat_set_vout_range_min(uint8_t rail, uint16_t *millivolt);
bool plat_set_vout_range_max(uint8_t rail, uint16_t *millivolt);
bool vr_vout_user_settings_get(void *user_settings);
void user_settings_init(void);
bool vr_vout_range_user_settings_init(void);
bool vr_vout_default_settings_init(void);
bool vr_vout_user_settings_init(void);
#endif
