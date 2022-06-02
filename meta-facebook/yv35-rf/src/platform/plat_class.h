#ifndef PLAT_CLASS_H
#define PLAT_CLASS_H

#include "hal_gpio.h"
#include "plat_gpio.h"

enum BOARD_ID {
	RAINBOW_FALLS = 0x0A,
	WAIMANO_FALLS = 0x0C,
	VERNAL_FALLS = 0x0E,
	UNKNOWN_BOARD = 0xFF,
};

enum BOARD_ID_BIT_MAPPING {
	BOARD_ID_BIT_0 = BOARD_ID0,
	BOARD_ID_BIT_1 = BOARD_ID1,
	BOARD_ID_BIT_2 = BOARD_ID2,
	BOARD_ID_BIT_3 = BOARD_ID3,
};

void init_platform_config();
void init_sys_board_id(uint8_t board_id);
uint8_t get_board_id();

#endif
