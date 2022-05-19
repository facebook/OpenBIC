#ifndef PLAT_CLASS_H
#define PLAT_CLASS_H

#include <stdbool.h>
#include <stdint.h>

#define SYS_CLASS_1 1
#define SYS_CLASS_2 2

enum BIC_BOARD_REVISION {
	SYS_BOARD_POC = 0x0,
	SYS_BOARD_EVT,
	SYS_BOARD_EVT2,
	SYS_BOARD_EVT3_HOTSWAP,
	SYS_BOARD_EVT3_EFUSE,
	SYS_BOARD_DVT_HOTSWAP,
	SYS_BOARD_DVT_EFUSE,
	SYS_BOARD_MP_HOTSWAP,
	SYS_BOARD_MP_EFUSE,
};

enum BIC_CLASS_TYPE {
	TYPE_2OU_EXP = 0x1,
	TYPE_2OU_SPE = 0x2,
	TYPE_2OU_DPV2_8 = 0x7,
	TYPE_2OU_DPV2_16 = 0x70,
	TYPE_UNKNOWN = 0xFF,
};

enum BIC_CARD_PRESENT {
	CARD_UNPRESENT = false,
	CARD_PRESENT = true,
};

uint8_t get_system_class();
bool get_1ou_status();
bool get_2ou_status();
uint8_t get_board_revision();
uint8_t get_2ou_cardtype();
float get_hsc_type_adc_voltage();

void init_platform_config();

#endif
