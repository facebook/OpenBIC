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

#ifndef PLAT_FRU_H
#define PLAT_FRU_H

#include "plat_modbus.h"

uint8_t modbus_read_fruid_data(modbus_command_mapping *cmd);
uint8_t modbus_write_fruid_data(modbus_command_mapping *cmd);

enum FRU_ID {
	MB_FRU_ID = 0x00,
	BB_FRU_ID,
	BPB_FRU_ID,
	PDB_FRU_ID,	
	SB_FRU_ID,
	PB_1_FRU_ID,
	PB_2_FRU_ID,
	PB_3_FRU_ID,				
	FB_1_FRU_ID,
	FB_2_FRU_ID,
	FB_3_FRU_ID,
	FB_4_FRU_ID,
	FB_5_FRU_ID,
	FB_6_FRU_ID,
	FB_7_FRU_ID,
	FB_8_FRU_ID,
	FB_9_FRU_ID,
	FB_10_FRU_ID,
	FB_11_FRU_ID,
	FB_12_FRU_ID,
	FB_13_FRU_ID,
	FB_14_FRU_ID,	
	MAX_FRU_ID,
};

#define FRU_CFG_NUM MAX_FRU_ID

#define MB_FRU_ADDR 0xA6
#define BB_FRU_ADDR 0xA4
#define BPB_FRU_ADDR 0xA6
#define SB_FRU_ADDR 0xA6
#define PDB_FRU_ADDR 0XAA
#define PB_FRU_ADDR 0xA6
#define FB_FRU_ADDR 0xA6

#define FB_1_4_MUX_ADDR 0xE0
#define FB_5_8_MUX_ADDR 0xE2
#define FB_9_12_MUX_ADDR 0xE4
#define FB_13_14_MUX_ADDR 0xE6
#define PB_MUX_ADDR 0xE8
#define SB_MUX_ADDR 0xE8

#endif
