#include <stdio.h>
#include "hal_gpio.h"
#include "expansion_board.h"

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
		printf("[%s] input board id not support: 0x%x\n", __func__, board_id);
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
	printf("[%s] board id 0x%x\n", __func__, system_board_id);
	return;
}

uint8_t get_board_id()
{
	return system_board_id;
}
