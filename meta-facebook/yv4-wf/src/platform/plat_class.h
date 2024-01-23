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

enum ADC_REF_VOL_SELECTION {
	REF_VOL_2_5V = 0x0, // 2.5V reference voltage selection
	REF_VOL_1_2V = 0x40 // 1.2V reference voltage selection
};

enum ADC_CHANNEL_NUM {
	CHANNEL_1 = 1,
};

enum BOARD_REV_ID {
	BOARD_POC = 0,
	BOARD_EVT,
	UNKNOWN,
};

struct adc_info {
	long unsigned int offset;
	int shift;
};

struct board_rev_mappting_table {
	float voltage;
	float range_val;
	uint8_t board_rev;
};

bool get_adc_voltage(int channel, float *voltage);
uint8_t get_board_revision();
int init_platform_config();

#endif
