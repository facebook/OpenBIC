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
#define CPLD_CLASS_TYPE_REG 0x0D
#define CPLD_1OU_VPP_POWER_STATUS 0x11
#define CPLD_2OU_EXPANSION_CARD_REG 0x13

#define MAX_1OU_M2_COUNT 4

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
	TYPE_1OU_EXP_WITH_NIC = 0x07,
	TYPE_1OU_ABSENT = 0xFE,
	TYPE_1OU_UNKNOWN = 0xFF,
};

enum _2OU_CARD_TYPE_ {
	TYPE_2OU_EXP = 0x1,
	TYPE_2OU_EXP_E1S = 0x2,
	TYPE_2OU_HSM = 0x6, //Hardware Security Module
	TYPE_2OU_ABSENT = 0xFE,
	TYPE_2OU_UNKNOWN = 0xFF,
};

enum BIC_CARD_PRESENT {
	CARD_UNPRESENT = false,
	CARD_PRESENT = true,
};

uint8_t get_system_class();
CARD_STATUS get_1ou_status();
CARD_STATUS get_2ou_status();

void init_platform_config();

#endif
