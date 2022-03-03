#ifndef PLAT_CLASS_H
#define PLAT_CLASS_H

#include <stdbool.h>
#include <stdint.h>

#define sys_class_1 0
#define sys_class_2 1

enum {
	type_2ou_exp = 0x1,
	type_2ou_spe = 0x2,
	type_2ou_dpv2 = 0x77,
	type_2ou_dpv2_8 = 0x7,
	type_2ou_dpv2_16 = 0x70,
};

bool get_bic_class();
bool get_1ou_status();
bool get_2ou_status();
uint8_t get_2ou_cardtype();

void init_platform_config();

#endif
