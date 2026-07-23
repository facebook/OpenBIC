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

#define NUMBER_OF_ADC_CHANNEL 16
#define AST1030_ADC_BASE_ADDR 0x7e6e9000
#define ADC_PCLK_NS 42 // ~24MHz APB clock, adjust if your board differs
#define ADC_SAMPLE_COUNT 8
#define BLADE_CONFIG_VOLTAGE_RANGE 0.1 // range: +-0.1V, half of 0.25V spacing

enum ADC_REF_VOL_SELECTION {
	REF_VOL_2_5V = 0x0, // 2.5V reference voltage selection
	REF_VOL_1_2V = 0x40 // 1.2V reference voltage selection
};

enum ADC_CHANNEL_NUM {
	CHANNEL_0 = 0,
	CHANNEL_1 = 1,
};

enum BLADE_CONFIG {
	BLADE_CONFIG_with_ASIC = 0x00,
	BLADE_CONFIG_without_ASIC = 0x10,
	BLADE_CONFIG_UNKNOWN = 0xff,
};

enum BOARD_REV_ID {
	BOARD_POC = 0,
	BOARD_POC2,
	BOARD_EVT,
	BOARD_DVT,
	BOARD_PVT,
	BOARD_MP,
	UNKNOWN,
};

struct adc_info {
	long unsigned int offset;
	int shift;
};

struct blade_config_mapping_table {
	float voltage;
	float range_val;
	uint8_t blade_config;
};

struct board_rev_mappting_table {
	float voltage;
	float range_val;
	uint8_t board_rev;
};

bool get_adc_voltage(int channel, float *voltage);
bool get_blade_config(uint8_t *blade_config);
uint8_t get_board_revision();
uint8_t get_blade_configuration();
int init_platform_config();

#endif
