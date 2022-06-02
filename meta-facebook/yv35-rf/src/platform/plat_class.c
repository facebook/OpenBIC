#include <stdio.h>
#include "hal_gpio.h"
#include "plat_class.h"

static uint8_t system_board_id = 0;

void init_sys_board_id(uint8_t board_id)
{
	switch (board_id) {
	case RAINBOW_FALLS:
		system_board_id = RAINBOW_FALLS;
		break;
	case WAIMANO_FALLS:
		system_board_id = WAIMANO_FALLS;
		break;
	case VERNAL_FALLS:
		system_board_id = VERNAL_FALLS;
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
	board_id = ((gpio_get(BOARD_ID_BIT_3) << 3) | (gpio_get(BOARD_ID_BIT_2) << 2) |
		    (gpio_get(BOARD_ID_BIT_1) << 1) | (gpio_get(BOARD_ID_BIT_0) << 0));

	init_sys_board_id(board_id);
	printf("[%s] board id 0x%x\n", __func__, system_board_id);
	return;
}

uint8_t get_board_id()
{
	return system_board_id;
}
