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

typedef struct _CARD_STATUS_ {
	bool present;
	uint8_t card_type;
} CARD_STATUS;

enum _1OU_CARD_TYPE_ {
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

enum HSC_MODULE {
	HSC_MODULE_ADM1278,
	HSC_MODULE_LTC4282,
	HSC_MODULE_MP5990,
	HSC_MODULE_ADM1276,
	HSC_MODULE_UNKNOWN = 0xFF,
};

enum BIC_CARD_PRESENT {
	CARD_UNPRESENT = false,
	CARD_PRESENT = true,
};

uint8_t get_system_class();
CARD_STATUS get_1ou_status();
CARD_STATUS get_2ou_status();
uint8_t get_hsc_module();
void init_hsc_module();
void init_platform_config();

#endif
