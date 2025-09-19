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
#define THERMAL_SENSOR_1_ADDR (0x6A >> 1)
#define THERMAL_SENSOR_2_ADDR (0x98 >> 1)
#define PCIE_SWITCH_ADDR (0xD4 >> 1)

#define VR_ASIC_P0V895_PEX_MP2971_ADDR (0x84 >> 1)
#define VR_ASIC_P0V895_PEX_ISL69260_ADDR (0xC0 >> 1)

#define VR_ASIC_P0V825_A0_MP2971_ADDR (0x84 >> 1)
#define VR_ASIC_P0V825_A0_ISL69260_ADDR (0xC0 >> 1)

#define VR_ASIC_P0V825_A1_MP2971_ADDR (0x40 >> 1)
#define VR_ASIC_P0V825_A1_ISL69260_ADDR (0xC2 >> 1)

#define VR_ASIC_P0V825_A2_MP2971_ADDR (0x40 >> 1)
#define VR_ASIC_P0V825_A2_ISL69260_ADDR (0xC2 >> 1)

#define ADS7830_I2C_ADDR (0x90 >> 1)

/* Define the sensor numbers used in this platform */
enum SENSOR_NUM_LIST {
	SENSOR_NUM_THERMAL_SENSOR_1_TEMP_C = 0x01,
	SENSOR_NUM_THERMAL_SENSOR_2_TEMP_C,
	SENSOR_NUM_PCIE_SWITCH_TEMP_C,
	SENSOR_NUM_VR_ASIC_P0V895_PEX_TEMP_C,
	SENSOR_NUM_VR_ASIC_P0V895_PEX_VOLT_V,
	SENSOR_NUM_VR_ASIC_P0V895_PEX_CURR_A,
	SENSOR_NUM_VR_ASIC_P0V895_PEX_PWR_W,
	SENSOR_NUM_VR_ASIC_P0V825_A0_TEMP_C,
	SENSOR_NUM_VR_ASIC_P0V825_A0_VOLT_V,
	SENSOR_NUM_VR_ASIC_P0V825_A0_CURR_A,
	SENSOR_NUM_VR_ASIC_P0V825_A0_PWR_W,
	SENSOR_NUM_VR_ASIC_P0V825_A1_TEMP_C,
	SENSOR_NUM_VR_ASIC_P0V825_A1_VOLT_V,
	SENSOR_NUM_VR_ASIC_P0V825_A1_CURR_A,
	SENSOR_NUM_VR_ASIC_P0V825_A1_PWR_W,
	SENSOR_NUM_VR_ASIC_P0V825_A2_TEMP_C,
	SENSOR_NUM_VR_ASIC_P0V825_A2_VOLT_V,
	SENSOR_NUM_VR_ASIC_P0V825_A2_CURR_A,
	SENSOR_NUM_VR_ASIC_P0V825_A2_PWR_W,
	SENSOR_NUM_ADC_P12V_SCALED_VOLT_V,
	SENSOR_NUM_ADC_P5V_STBY_SCALED_VOLT_V,
	SENSOR_NUM_ADC_P3V3_AUX_SCALED_VOLT_V,
	SENSOR_NUM_ADC_P1V5_PEX_SCALED_VOLT_V,
	SENSOR_NUM_ADC_P1V2_PEX_SCALED_VOLT_V,
	SENSOR_NUM_ADC_P1V8_PEX_SCALED_VOLT_V,
};

#define TMP75_TEMP_OFFSET 0x00
#define UPDATE_INTERVAL_1S 1
#define UPDATE_INTERVAL_5S 5
#define UPDATE_INTERVAL_60S 60

enum SENSOR_THREAD_LIST {
	TEMP_SENSOR_THREAD_ID = 0,
	VR_SENSOR_THREAD_ID,
	ADC_SENSOR_THREAD_ID,
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
void plat_pldm_sensor_change_retimer_dev();
bool is_dc_access(uint8_t sensor_num);
void set_plat_sensor_polling_enable_flag(bool value);
void set_plat_sensor_temp_polling_enable_flag(bool value);
void set_plat_sensor_vr_polling_enable_flag(bool value);
void set_plat_sensor_adc_polling_enable_flag(bool value);
bool get_plat_sensor_polling_enable_flag();
bool get_plat_sensor_temp_polling_enable_flag();
bool get_plat_sensor_vr_polling_enable_flag();
bool get_plat_sensor_adc_polling_enable_flag();
bool is_temp_access(uint8_t cfg_idx);
bool is_pcie_switch_access(uint8_t cfg_idx);
bool is_vr_access(uint8_t sensor_num);
bool is_adc_access(uint8_t sensor_num);
bool get_sensor_info_by_sensor_id(uint8_t sensor_id, uint8_t *vr_bus, uint8_t *vr_addr,
				  uint8_t *sensor_dev);
size_t char16_strlen(const char16_t *str);
char16_t *char16_strcpy(char16_t *dest, const char16_t *src);
char16_t *char16_strcat_char(char16_t *dest, char16_t ch);

#endif
