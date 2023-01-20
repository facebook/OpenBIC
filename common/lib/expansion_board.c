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
#include "hal_gpio.h"
#include "expansion_board.h"

#include <logging/log.h>

LOG_MODULE_REGISTER(expansion_board);

static uint8_t system_board_id = 0;

void init_sys_board_id(uint8_t board_id)
{
	switch (board_id) {
	case RAINBOW_FALLS:
		system_board_id = RAINBOW_FALLS;
		break;
	case VERNAL_FALLS:
		system_board_id = VERNAL_FALLS_BOARD_TYPE;
		break;
	default:
		LOG_ERR("Input board id not support: 0x%x", board_id);
		system_board_id = UNKNOWN_BOARD;
		break;
	}
}

void init_platform_config()
{
	uint8_t board_id = 0;

	board_id = (gpio_get(BOARD_ID3) << 3);
	board_id |= (gpio_get(BOARD_ID2) << 2);
	board_id |= (gpio_get(BOARD_ID1) << 1);
	board_id |= (gpio_get(BOARD_ID0) << 0);

	init_sys_board_id(board_id);
	LOG_ERR("Board id 0x%x", system_board_id);
	return;
}

uint8_t get_board_id()
{
	return system_board_id;
}
