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

#ifndef EXPANSION_BOARD_H
#define EXPANSION_BOARD_H

#include "hal_gpio.h"
#include "plat_gpio.h"

enum EXPANSION_BOARD_ID {
	RAINBOW_FALLS = 0x0A,
	WAIMANO_FALLS = 0x0C,
	VERNAL_FALLS = 0x0E,
	UNKNOWN_BOARD = 0xFF,
};

#define VERNAL_FALLS_BOARD_TYPE 0x07 // VF return board_type instead of board_id

void init_platform_config();
void init_sys_board_id(uint8_t board_id);
uint8_t get_board_id();

#endif
