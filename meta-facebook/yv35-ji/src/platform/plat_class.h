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

#include <stdbool.h>
#include <stdint.h>

#define SYS_CLASS_1 1
#define SYS_CLASS_2 2

#define CPLD_ADDR 0x21 // 7-bit address
#define CPLD_CLASS_TYPE_REG 0x05
#define CPLD_2OU_EXPANSION_CARD_REG 0x06
#define CPLD_BOARD_REV_ID_REG 0x08
#define CPLD_1OU_CARD_DETECTION 0x09
#define CPLD_1OU_VPP_POWER_STATUS 0x11
#define I2C_DATA_SIZE 5
#define NUMBER_OF_ADC_CHANNEL 16
#define AST1030_ADC_BASE_ADDR 0x7e6e9000
#define HSC_TYPE_ADC_TOLERANCE 0.2 // V

#define MAX_1OU_M2_COUNT 4

enum BIC_BOARD_REVISION {
	SYS_BOARD_POC = 0x0,
	SYS_BOARD_EVT,
	SYS_BOARD_EVT2,
	SYS_BOARD_EVT3_HOTSWAP,
	SYS_BOARD_EVT3_EFUSE,
	SYS_BOARD_DVT_HOTSWAP,
	SYS_BOARD_DVT_EFUSE,
	SYS_BOARD_PVT_HOTSWAP,
	SYS_BOARD_PVT_EFUSE,
	SYS_BOARD_MP_HOTSWAP,
	SYS_BOARD_MP_EFUSE,
	SYS_BOARD_MP_2_HOTSWAP,
	SYS_BOARD_MP_2_EFUSE,
};

typedef struct _CARD_STATUS_ {
	bool present;
	uint8_t card_type;
} CARD_STATUS;

enum _1OU_CARD_TYPE_ {
	TYPE_1OU_SI_TEST_CARD = 0x0,
	TYPE_1OU_EXP_WITH_6_M2,
	TYPE_1OU_RAINBOW_FALLS,
	TYPE_1OU_VERNAL_FALLS_WITH_TI, // TI BIC
	TYPE_1OU_VERNAL_FALLS_WITH_AST, // AST1030 BIC
	TYPE_1OU_KAHUNA_FALLS,
	TYPE_1OU_WAIMANO_FALLS,
	TYPE_1OU_EXP_WITH_NIC,
	TYPE_1OU_ABSENT = 0xFE,
	TYPE_1OU_UNKNOWN = 0xFF,
};

enum _2OU_CARD_TYPE_ {
	TYPE_2OU_DPV2_8 = 0x07, // DPV2x8
	TYPE_2OU_DPV2_16 = 0x70, // DPV2x16
	TYPE_2OU_ABSENT = 0xFE,
	TYPE_2OU_UNKNOWN = 0xFF,
};

/* ADC channel number */
enum ADC_CHANNEL {
	CHANNEL_6 = 6,
	CHANNEL_7 = 7,
};

enum HSC_MODULE {
	HSC_MODULE_ADM1278,
	HSC_MODULE_LTC4282,
	HSC_MODULE_MP5990,
	HSC_MODULE_LTC4286 = 0x04,
	HSC_MODULE_UNKNOWN = 0xFF,
};

enum BIC_CARD_PRESENT {
	CARD_UNPRESENT = false,
	CARD_PRESENT = true,
};

uint8_t get_system_class();
CARD_STATUS get_1ou_status();
CARD_STATUS get_2ou_status();
uint8_t get_board_revision();
uint8_t get_hsc_module();
bool get_adc_voltage(int channel, float *voltage);
void init_hsc_module(uint8_t board_revision);
void init_platform_config();

#endif
