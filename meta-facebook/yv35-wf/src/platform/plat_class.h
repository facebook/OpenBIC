#ifndef PLAT_CLASS_H
#define PLAT_CLASS_H

#include "hal_gpio.h"
#include "plat_gpio.h"

enum BOARD_ID {
	RAINBOW_FALLS = 0xA,
	WAIMANO_FALLS = 0xC,
	VERNAL_FALLS = 0xE,
	UNKNOWN_BOARD = 0xFF,
};


uint8_t get_board_id();

#endif
