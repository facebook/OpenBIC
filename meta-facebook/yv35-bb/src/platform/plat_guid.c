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

#include <stdio.h>
#include "guid.h"
#include "plat_guid.h"
#include "fru.h"

#define BB_GUID_PORT 0x01
#define BB_GUID_ADDR (0xA2 >> 1)

const EEPROM_CFG guid_config[] = {
	{
		NV_ATMEL_24C128,
		BB_GUID_ID,
		BB_GUID_PORT,
		BB_GUID_ADDR,
		GUID_ACCESS_BYTE,
		GUID_START,
		GUID_SIZE,
	},
};
