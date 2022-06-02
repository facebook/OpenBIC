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

typedef struct _CARD_STATUS_ {
	bool present;
	uint8_t card_type;
} CARD_STATUS;

enum _1OU_CARD_TYPE_ {
	TYPE_1OU_SI_TEST_CARD = 0x0,
	TYPE_1OU_EXP_WITH_6_M2,
	TYPE_1OU_RAINBOW_FALLS,
	TYPE_1OU_VERNAL_FALLS_WITH_TI, // TI BIC
	TYPE_1OU_VERNAL_FALLS_WITH_AST, // AST1030 BIC
	TYPE_1OU_KAHUNA_FALLS,
	TYPE_1OU_WAIMANO_FALLS,
	TYPE_1OU_EXP_WITH_NIC,
	TYPE_1OU_ABSENT = 0xFE,
	TYPE_1OU_UNKNOWN = 0xFF,
};

enum _2OU_CARD_TYPE_ {
	TYPE_2OU_DPV2_8 = 0x07, // DPV2x8
	TYPE_2OU_DPV2_16 = 0x70, // DPV2x16
	TYPE_2OU_ABSENT = 0xFE,
	TYPE_2OU_UNKNOWN = 0xFF,
};

/* ADC channel number */
enum ADC_CHANNEL {
	CHANNEL_6 = 6,
	CHANNEL_7 = 7,
};

enum BIC_CARD_PRESENT {
	CARD_UNPRESENT = false,
	CARD_PRESENT = true,
};

uint8_t get_system_class();
CARD_STATUS get_1ou_status();
CARD_STATUS get_2ou_status();
uint8_t get_board_revision();
bool get_adc_voltage(int channel, float *voltage);

void init_platform_config();

#endif
