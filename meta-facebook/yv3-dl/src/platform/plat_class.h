#ifndef PLAT_CLASS_H
#define PLAT_CLASS_H

#include <stdbool.h>
#include <stdint.h>

#define SYS_CLASS_1 1
#define SYS_CLASS_2 2

typedef struct _CARD_STATUS_ {
	bool present;
	uint8_t card_type;
} CARD_STATUS;

enum _1OU_CARD_TYPE_ {
	TYPE_1OU_ABSENT = 0xFE,
	TYPE_1OU_UNKNOWN = 0xFF,
};

enum _2OU_CARD_TYPE_ {
	TYPE_2OU_EXP = 0x1,
	TYPE_2OU_EXP_E1S = 0x2,
	TYPE_2OU_HSM = 0x6,     //Hardware Security Module
	TYPE_2OU_ABSENT = 0xFE,
	TYPE_2OU_UNKNOWN = 0xFF,
};

enum BIC_CARD_PRESENT {
	CARD_UNPRESENT = false,
	CARD_PRESENT = true,
};

uint8_t get_system_class();
CARD_STATUS get_1ou_status();
CARD_STATUS get_2ou_status();

void init_platform_config();

#endif
