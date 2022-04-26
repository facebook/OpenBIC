#ifndef PLAT_CLASS_H
#define PLAT_CLASS_H

#include <stdbool.h>
#include <stdint.h>

#define SYS_CLASS_1 0
#define SYS_CLASS_2 1

enum BIC_CLASS_TYPE {
	TYPE_2OU_EXP = 0x1,
	TYPE_2OU_SPE = 0x2,
	TYPE_2OU_DPV2 = 0x77,
	TYPE_2OU_DPV2_8 = 0x7,
	TYPE_2OU_DPV2_16 = 0x70,
	TYPE_UNKNOWN = 0xFF,
};

bool get_bic_class();
bool get_1ou_status();
bool get_2ou_status();
uint8_t get_2ou_cardtype();

void init_platform_config();

#endif
