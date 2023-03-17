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

enum E1S_CONTROL {
	DEVICE_POWER_OFF = 0x0,
	DEVICE_POWER_ON,
	DEVICE_PRESENT,
	DEVICE_POWER_GOOD,
};

enum OL2_FIRMWARE_COMPONENT {
	//OL2_COMPNT_CPLD = 1,
	OL2_COMPNT_BIC = 2,
	//OL2_COMPNT_ME = 3,
	//OL2_COMPNT_BIOS = 4,
	OL2_COMPNT_RETIMER = 5,
};

#define BIC_UPDATE_MAX_OFFSET 0x50000
#define IS_SECTOR_END_MASK 0x80

#endif
