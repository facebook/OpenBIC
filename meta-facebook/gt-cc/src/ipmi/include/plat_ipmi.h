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

#ifndef PLAT_IPMI_H
#define PLAT_IPMI_H

#include <stdbool.h>
#include <stdint.h>

#define IS_SECTOR_END_MASK 0x80
#define WITHOUT_SENCTOR_END_MASK 0x7F
#define BIC_UPDATE_MAX_OFFSET 0x50000

#define CC_PEX_NOT_POWER_ON 0xB0
#define CC_PEX_PRE_READING_FAIL 0xB1
#define CC_PEX_ACCESS_FAIL 0xB2

enum GT_FIRMWARE_COMPONENT {
	GT_COMPNT_VR0,
	GT_COMPNT_VR1,
	GT_COMPNT_BIC,
	GT_COMPNT_PEX0,
	GT_COMPNT_PEX1,
	GT_COMPNT_PEX2,
	GT_COMPNT_PEX3,
	GT_COMPNT_CPLD,
	GT_COMPNT_NIC0,
	GT_COMPNT_NIC1,
	GT_COMPNT_NIC2,
	GT_COMPNT_NIC3,
	GT_COMPNT_NIC4,
	GT_COMPNT_NIC5,
	GT_COMPNT_NIC6,
	GT_COMPNT_NIC7,
	GT_COMPNT_MAX,
};

#endif
