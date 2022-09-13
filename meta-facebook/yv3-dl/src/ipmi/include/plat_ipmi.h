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

enum REQ_GET_CARD_TYPE {
	GET_1OU_CARD_TYPE = 0x0,
	GET_2OU_CARD_TYPE,
};

enum DL_FIRMWARE_COMPONENT {
	DL_COMPNT_BIOS = 0,
	DL_COMPNT_CPLD,
	DL_COMPNT_BIC,
	DL_COMPNT_ME,
	DL_COMPNT_BOOTLOADER, // Align to Yv3 TI BIC
	DL_COMPNT_PVCCIN,
	DL_COMPNT_PVCCSA,
	DL_COMPNT_PVCCIO,
	DL_COMPNT_P3V3_STBY,
	DL_COMPNT_PVDDR_ABC,
	DL_COMPNT_PVDDR_DEF
};

#endif
