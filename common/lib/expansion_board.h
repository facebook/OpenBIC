#ifndef EXPANSION_BOARD_H
#define EXPANSION_BOARD_H

#include "hal_gpio.h"
#include "plat_gpio.h"

enum BOARD_ID {
	RAINBOW_FALLS = 0x0A,
	WAIMANO_FALLS = 0x0C,
	VERNAL_FALLS = 0x0E,
	UNKNOWN_BOARD = 0xFF,
};

#define VERNAL_FALLS_BOARD_TYPE 0x07	// VF return board_type instead of board_id

void init_platform_config();
void init_sys_board_id(uint8_t board_id);
uint8_t get_board_id();

#endif
