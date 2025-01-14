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

#ifndef PLAT_PLDM_SENSOR_H
#define PLAT_PLDM_SENSOR_H

#include "pdr.h"
#include "sensor.h"
#define ADDR_UNKNOWN (0xFF >> 1)

/* Define sensors address(7 bit) */
#define DC_BRICK_1_ADDR (0x28 >> 1)
#define DC_BRICK_2_ADDR (0x34 >> 1)

#define TOP_INLET_TEMP_ADDR (0x92 >> 1)
#define TOP_OUTLET_TEMP_ADDR (0x9E >> 1)
#define BOT_INLET_TEMP_ADDR (0x94 >> 1)
#define BOT_OUTLET_TEMP_ADDR (0X96 >> 1)

#define ON_DIE_1_TEMP_TMP432_ADDR (0X98 >> 1)
#define ON_DIE_2_TEMP_TMP432_ADDR (0X98 >> 1)
#define ON_DIE_3_TEMP_TMP432_ADDR (0X9A >> 1)
#define ON_DIE_4_TEMP_TMP432_ADDR (0X9A >> 1)

#define ON_DIE_1_TEMP_EMC1413_ADDR (0XB8 >> 1)
#define ON_DIE_2_TEMP_EMC1413_ADDR (0XB8 >> 1)
#define ON_DIE_3_TEMP_EMC1413_ADDR (0X38 >> 1)
#define ON_DIE_4_TEMP_EMC1413_ADDR (0X38 >> 1)

#define P3V3_MP2971_ADDR (0xF6 >> 1)
#define P3V3_MP2971_FAB3_ADDR (0xD6 >> 1) // Change to 0xD6 at FAB3 board
#define P3V3_ISL69260_ADDR (0xC0 >> 1)

#define P0V85_PVDD_MP2891_ADDR (0x4C >> 1)
#define P0V85_PVDD_RAA228238_ADDR (0xE4 >> 1)

#define P0V75_PVDD_CH_N_MP2971_ADDR (0xE0 >> 1)
#define P0V75_PVDD_CH_N_ISL69260_ADDR (0xC0 >> 1)

#define P0V75_MAX_PHY_N_MP2971_ADDR (0xE0 >> 1)
#define P0V75_MAX_PHY_N_ISL69260_ADDR (0xC0 >> 1)

#define P0V75_PVDD_CH_S_MP2971_ADDR (0xE2 >> 1)
#define P0V75_PVDD_CH_S_ISL69260_ADDR (0xC2 >> 1)

#define P0V75_MAX_PHY_S_MP2971_ADDR (0xE2 >> 1)
#define P0V75_MAX_PHY_S_ISL69260_ADDR (0xC2 >> 1)

#define P0V75_TRVDD_ZONEA_MP2971_ADDR (0xE6 >> 1)
#define P0V75_TRVDD_ZONEA_ISL69260_ADDR (0xC4 >> 1)

#define P1V8_VPP_HBM0_HBM2_HBM4_MP2971_ADDR (0xE6 >> 1)
#define P1V8_VPP_HBM0_HBM2_HBM4_ISL69260_ADDR (0xC4 >> 1)

#define P0V75_TRVDD_ZONEB_MP2971_ADDR (0xEC >> 1)
#define P0V75_TRVDD_ZONEB_ISL69260_ADDR (0xC6 >> 1)

#define P0V4_VDDQL_HBM0_HBM2_HBM4_MP2971_ADDR (0xEC >> 1)
#define P0V4_VDDQL_HBM0_HBM2_HBM4_ISL69260_ADDR (0xC6 >> 1)

#define P1V1_VDDC_HBM0_HBM2_HBM4_MP2971_ADDR (0xEA >> 1)
#define P1V1_VDDC_HBM0_HBM2_HBM4_ISL69260_ADDR (0xE8 >> 1)

#define P0V75_VDDPHY_HBM0_HBM2_HBM4_MP2971_ADDR (0xEA >> 1)
#define P0V75_VDDPHY_HBM0_HBM2_HBM4_ISL69260_ADDR (0xE8 >> 1)

#define P0V9_TRVDD_ZONEA_MP2971_ADDR (0xE4 >> 1)
#define P0V9_TRVDD_ZONEA_ISL69260_ADDR (0xC0 >> 1)

#define P1V8_VPP_HBM1_HBM3_HBM5_MP2971_ADDR (0xE4 >> 1)
#define P1V8_VPP_HBM1_HBM3_HBM5_ISL69260_ADDR (0xC0 >> 1)

#define P0V9_TRVDD_ZONEB_MP2971_ADDR (0xE8 >> 1)
#define P0V9_TRVDD_ZONEB_ISL69260_ADDR (0xC2 >> 1)

#define P0V4_VDDQL_HBM1_HBM3_HBM5_MP2971_ADDR (0xE8 >> 1)
#define P0V4_VDDQL_HBM1_HBM3_HBM5_ISL69260_ADDR (0xC2 >> 1)

#define P1V1_VDDC_HBM1_HBM3_HBM5_MP2971_ADDR (0xEE >> 1)
#define P1V1_VDDC_HBM1_HBM3_HBM5_ISL69260_ADDR (0xC4 >> 1)

#define P0V75_VDDPHY_HBM1_HBM3_HBM5_MP2971_ADDR (0xEE >> 1)
#define P0V75_VDDPHY_HBM1_HBM3_HBM5_ISL69260_ADDR (0xC4 >> 1)

#define P0V8_VDDA_PCIE_MP2971_ADDR (0xF2 >> 1)
#define P0V8_VDDA_PCIE_MP2971_FAB3_ADDR (0xD2 >> 1) // Change to 0xD2 at FAB3 board
#define P0V8_VDDA_PCIE_ISL69260_ADDR (0xC6 >> 1)

#define P1V2_VDDHTX_PCIE_MP2971_ADDR (0xF2 >> 1)
#define P1V2_VDDHTX_PCIE_MP2971_FAB3_ADDR (0xD2 >> 1) // Change to 0xD2 at FAB3 board
#define P1V2_VDDHTX_PCIE_ISL69260_ADDR (0xC6 >> 1)

/* Define the sensor numbers used in this platform */
#define SENSOR_NUM_UBC_1_TEMP_C 0x01
#define SENSOR_NUM_UBC_1_P50V_VIN_VOLT_V 0x02
#define SENSOR_NUM_UBC_1_P12V_VOUT_VOLT_V 0x03
#define SENSOR_NUM_UBC_1_P12V_CURR_A 0x04
#define SENSOR_NUM_UBC_1_P12V_PWR_W 0x05

#define SENSOR_NUM_UBC_2_TEMP_C 0x06
#define SENSOR_NUM_UBC_2_P50V_VIN_VOLT_V 0x07
#define SENSOR_NUM_UBC_2_P12V_VOUT_VOLT_V 0x08
#define SENSOR_NUM_UBC_2_P12V_CURR_A 0x09
#define SENSOR_NUM_UBC_2_P12V_PWR_W 0x0A

#define SENSOR_NUM_TOP_INLET_TEMP_C 0x0B
#define SENSOR_NUM_TOP_OUTLET_TEMP_C 0x0C
#define SENSOR_NUM_BOT_INLET_TEMP_C 0x0D
#define SENSOR_NUM_BOT_OUTLET_TEMP_C 0x0E
#define SENSOR_NUM_ON_DIE_1_TEMP_C 0x0F
#define SENSOR_NUM_ON_DIE_2_TEMP_C 0x10
#define SENSOR_NUM_ON_DIE_3_TEMP_C 0x11
#define SENSOR_NUM_ON_DIE_4_TEMP_C 0x12

#define SENSOR_NUM_OSFP_P3V3_TEMP_C 0x13
#define SENSOR_NUM_OSFP_P3V3_VOLT_V 0x14
#define SENSOR_NUM_OSFP_P3V3_CURR_A 0x15
#define SENSOR_NUM_OSFP_P3V3_PWR_W 0x16

#define SENSOR_NUM_CPU_P0V85_PVDD_TEMP_C 0x17
#define SENSOR_NUM_CPU_P0V85_PVDD_VOLT_V 0x18
#define SENSOR_NUM_CPU_P0V85_PVDD_CURR_A 0x19
#define SENSOR_NUM_CPU_P0V85_PVDD_PWR_W 0x1A

#define SENSOR_NUM_CPU_P0V75_PVDD_CH_N_TEMP_C 0x1B
#define SENSOR_NUM_CPU_P0V75_PVDD_CH_N_VOLT_V 0x1C
#define SENSOR_NUM_CPU_P0V75_PVDD_CH_N_CURR_A 0x1D
#define SENSOR_NUM_CPU_P0V75_PVDD_CH_N_PWR_W 0x1E
#define SENSOR_NUM_CPU_P0V75_MAX_PHY_N_TEMP_C 0x1F
#define SENSOR_NUM_CPU_P0V75_MAX_PHY_N_VOLT_V 0x20
#define SENSOR_NUM_CPU_P0V75_MAX_PHY_N_CURR_A 0x21
#define SENSOR_NUM_CPU_P0V75_MAX_PHY_N_PWR_W 0x22

#define SENSOR_NUM_CPU_P0V75_PVDD_CH_S_TEMP_C 0x23
#define SENSOR_NUM_CPU_P0V75_PVDD_CH_S_VOLT_V 0x24
#define SENSOR_NUM_CPU_P0V75_PVDD_CH_S_CURR_A 0x25
#define SENSOR_NUM_CPU_P0V75_PVDD_CH_S_PWR_W 0x26
#define SENSOR_NUM_CPU_P0V75_MAX_PHY_S_TEMP_C 0x27
#define SENSOR_NUM_CPU_P0V75_MAX_PHY_S_VOLT_V 0x28
#define SENSOR_NUM_CPU_P0V75_MAX_PHY_S_CURR_A 0x29
#define SENSOR_NUM_CPU_P0V75_MAX_PHY_S_PWR_W 0x2A

#define SENSOR_NUM_CPU_P0V75_TRVDD_ZONEA_TEMP_C 0x2B
#define SENSOR_NUM_CPU_P0V75_TRVDD_ZONEA_VOLT_V 0x2C
#define SENSOR_NUM_CPU_P0V75_TRVDD_ZONEA_CURR_A 0x2D
#define SENSOR_NUM_CPU_P0V75_TRVDD_ZONEA_PWR_W 0x2E
#define SENSOR_NUM_CPU_P1V8_VPP_HBM0_2_4_TEMP_C 0x2F
#define SENSOR_NUM_CPU_P1V8_VPP_HBM0_2_4_VOLT_V 0x30
#define SENSOR_NUM_CPU_P1V8_VPP_HBM0_2_4_CURR_A 0x31
#define SENSOR_NUM_CPU_P1V8_VPP_HBM0_2_4_PWR_W 0x32

#define SENSOR_NUM_CPU_P0V75_TRVDD_ZONEB_TEMP_C 0x33
#define SENSOR_NUM_CPU_P0V75_TRVDD_ZONEB_VOLT_V 0x34
#define SENSOR_NUM_CPU_P0V75_TRVDD_ZONEB_CURR_A 0x35
#define SENSOR_NUM_CPU_P0V75_TRVDD_ZONEB_PWR_W 0x36
#define SENSOR_NUM_CPU_P0V4_VDDQL_HBM0_2_4_TEMP_C 0x37
#define SENSOR_NUM_CPU_P0V4_VDDQL_HBM0_2_4_VOLT_V 0x38
#define SENSOR_NUM_CPU_P0V4_VDDQL_HBM0_2_4_CURR_A 0x39
#define SENSOR_NUM_CPU_P0V4_VDDQL_HBM0_2_4_PWR_W 0x3A

#define SENSOR_NUM_CPU_P1V1_VDDC_HBM0_2_4_TEMP_C 0x3B
#define SENSOR_NUM_CPU_P1V1_VDDC_HBM0_2_4_VOLT_V 0x3C
#define SENSOR_NUM_CPU_P1V1_VDDC_HBM0_2_4_CURR_A 0x3D
#define SENSOR_NUM_CPU_P1V1_VDDC_HBM0_2_4_PWR_W 0x3E
#define SENSOR_NUM_CPU_P0V75_VDDPHY_HBM0_2_4_TEMP_C 0x3F
#define SENSOR_NUM_CPU_P0V75_VDDPHY_HBM0_2_4_VOLT_V 0x40
#define SENSOR_NUM_CPU_P0V75_VDDPHY_HBM0_2_4_CURR_A 0x41
#define SENSOR_NUM_CPU_P0V75_VDDPHY_HBM0_2_4_PWR_W 0x42

#define SENSOR_NUM_CPU_P0V9_TRVDD_ZONEA_TEMP_C 0x43
#define SENSOR_NUM_CPU_P0V9_TRVDD_ZONEA_VOLT_V 0x44
#define SENSOR_NUM_CPU_P0V9_TRVDD_ZONEA_CURR_A 0x45
#define SENSOR_NUM_CPU_P0V9_TRVDD_ZONEA_PWR_W 0x46
#define SENSOR_NUM_CPU_P1V8_VPP_HBM1_3_5_TEMP_C 0x47
#define SENSOR_NUM_CPU_P1V8_VPP_HBM1_3_5_VOLT_V 0x48
#define SENSOR_NUM_CPU_P1V8_VPP_HBM1_3_5_CURR_A 0x49
#define SENSOR_NUM_CPU_P1V8_VPP_HBM1_3_5_PWR_W 0x4A

#define SENSOR_NUM_CPU_P0V9_TRVDD_ZONEB_TEMP_C 0x4B
#define SENSOR_NUM_CPU_P0V9_TRVDD_ZONEB_VOLT_V 0x4C
#define SENSOR_NUM_CPU_P0V9_TRVDD_ZONEB_CURR_A 0x4D
#define SENSOR_NUM_CPU_P0V9_TRVDD_ZONEB_PWR_W 0x4E
#define SENSOR_NUM_CPU_P0V4_VDDQL_HBM1_3_5_TEMP_C 0x4F
#define SENSOR_NUM_CPU_P0V4_VDDQL_HBM1_3_5_VOLT_V 0x50
#define SENSOR_NUM_CPU_P0V4_VDDQL_HBM1_3_5_CURR_A 0x51
#define SENSOR_NUM_CPU_P0V4_VDDQL_HBM1_3_5_PWR_W 0x52

#define SENSOR_NUM_CPU_P1V1_VDDC_HBM1_3_5_TEMP_C 0x53
#define SENSOR_NUM_CPU_P1V1_VDDC_HBM1_3_5_VOLT_V 0x54
#define SENSOR_NUM_CPU_P1V1_VDDC_HBM1_3_5_CURR_A 0x55
#define SENSOR_NUM_CPU_P1V1_VDDC_HBM1_3_5_PWR_W 0x56
#define SENSOR_NUM_CPU_P0V75_VDDPHY_HBM1_3_5_TEMP_C 0x57
#define SENSOR_NUM_CPU_P0V75_VDDPHY_HBM1_3_5_VOLT_V 0x58
#define SENSOR_NUM_CPU_P0V75_VDDPHY_HBM1_3_5_CURR_A 0x59
#define SENSOR_NUM_CPU_P0V75_VDDPHY_HBM1_3_5_PWR_W 0x5A

#define SENSOR_NUM_CPU_P0V8_VDDA_PCIE_TEMP_C 0x5B
#define SENSOR_NUM_CPU_P0V8_VDDA_PCIE_VOLT_V 0x5C
#define SENSOR_NUM_CPU_P0V8_VDDA_PCIE_CURR_A 0x5D
#define SENSOR_NUM_CPU_P0V8_VDDA_PCIE_PWR_W 0x5E
#define SENSOR_NUM_CPU_P1V2_VDDHTX_PCIE_TEMP_C 0x5F
#define SENSOR_NUM_CPU_P1V2_VDDHTX_PCIE_VOLT_V 0x60
#define SENSOR_NUM_CPU_P1V2_VDDHTX_PCIE_CURR_A 0x61
#define SENSOR_NUM_CPU_P1V2_VDDHTX_PCIE_PWR_W 0x62

#define TMP75_TEMP_OFFSET 0x00
#define UPDATE_INTERVAL_1S 1
#define UPDATE_INTERVAL_5S 5
#define UPDATE_INTERVAL_60S 60

enum SENSOR_THREAD_LIST {
	UBC_SENSOR_THREAD_ID = 0,
	VR_SENSOR_THREAD_ID,
	TEMP_SENSOR_THREAD_ID,
	MAX_SENSOR_THREAD_ID,
};

enum GET_VR_DEV_STATUS {
	GET_VR_DEV_SUCCESS = 0,
	GET_VR_DEV_FAILED,
};

int plat_pldm_sensor_get_sensor_count(int thread_id);
sensor_cfg *get_sensor_cfg_by_sensor_id(uint8_t sensor_id);
void plat_pldm_sensor_get_pdr_numeric_sensor(int thread_id, int sensor_num,
					     PDR_numeric_sensor *numeric_sensor_table);
uint8_t plat_pldm_sensor_get_vr_dev(uint8_t *vr_dev);
void plat_pldm_sensor_change_vr_dev();
void plat_pldm_sensor_change_cpu_bus();
void plat_pldm_sensor_change_retimer_dev();
bool is_dc_access(uint8_t sensor_num);
void set_plat_sensor_polling_enable_flag(bool value);
void set_plat_sensor_ubc_polling_enable_flag(bool value);
void set_plat_sensor_temp_polling_enable_flag(bool value);
void set_plat_sensor_vr_polling_enable_flag(bool value);
bool get_plat_sensor_polling_enable_flag();
bool get_plat_sensor_ubc_polling_enable_flag();
bool get_plat_sensor_temp_polling_enable_flag();
bool get_plat_sensor_vr_polling_enable_flag();
bool is_ubc_access(uint8_t sensor_num);
bool is_temp_access(uint8_t cfg_idx);
bool is_vr_access(uint8_t sensor_num);
bool get_sensor_info_by_sensor_id(uint8_t sensor_id, uint8_t *vr_bus, uint8_t *vr_addr,
				  uint8_t *sensor_dev);
bool is_osfp_3v3_access(uint8_t sensor_num);
size_t char16_strlen(const char16_t *str);
char16_t *char16_strcpy(char16_t *dest, const char16_t *src);
char16_t *char16_strcat_char(char16_t *dest);

#endif
