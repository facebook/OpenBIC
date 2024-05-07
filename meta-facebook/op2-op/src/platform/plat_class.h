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

#define RETIMER_UNKNOWN_VERSION -1
#define RETIMER_VERSION_MAX_LENGTH 5
#define CLOCK_BUFFER_ADDR 0x6B
#define CLOCK_BUFFER_REG_LOC 0x01
#define CLOCK_BUFFER_BYPASS_DATA_1 0x08
#define CLOCK_BUFFER_BYPASS_DATA_2 0x2E

enum CARD_POSITION {
	CARD_POSITION_1OU,
	CARD_POSITION_2OU,
	CARD_POSITION_3OU,
	CARD_POSITION_4OU,
	CARD_POSITION_UNKNOWN,
};

enum CARD_TYPE {
	CARD_TYPE_OPA,
	CARD_TYPE_OPB,
	CARD_TYPE_UNKNOWN,
};

enum RETIMER_TYPE {
	RETIMER_TYPE_PT5161L,
	RETIMER_TYPE_M88RT51632,
	RETIMER_TYPE_UNKNOWN,
};

enum I3C_HUB_TYPE {
	I3C_HUB_TYPE_RNS,
	I3C_HUB_TYPE_NXP,
	I3C_HUB_TYPE_UNKNOWN,
};

enum E1S_NUMBER {
	E1S_0,
	E1S_1,
	E1S_2,
	E1S_3,
	E1S_4,
};

enum BOARD_REVISION_ID {
	EVT_STAGE = 0b000,
	DVT_STAGE = 0b001,
	PVT_STAGE = 0b010,
	MP_STAGE = 0b011,
	UNKNOWN_STAGE = 0xFF,
};

int init_platform_config();
uint8_t get_card_type();
uint8_t get_card_position();
uint16_t get_i3c_hub_type();
int check_pcie_retimer_type(void);
uint8_t get_pcie_retimer_type(void);
uint8_t get_board_revision();
void init_board_revision();
void init_i3c_hub_type();
uint32_t get_pcie_retimer_version();
void cache_pcie_retimer_version();
void set_clock_buffer_bypass_mode();

#endif
