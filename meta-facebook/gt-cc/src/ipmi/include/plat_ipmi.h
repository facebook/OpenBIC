#ifndef PLAT_IPMI_H
#define PLAT_IPMI_H

#include <stdbool.h>
#include <stdint.h>

#define IS_SECTOR_END_MASK 0x80
#define WITHOUT_SENCTOR_END_MASK 0x7F
#define BIC_UPDATE_MAX_OFFSET 0x50000

enum GT_FIRWARE_UPDATE_TARGET {
	SWB_BIC_UPDATE = 2,
	PEX0_UPDATE = 3,
	PEX1_UPDATE = 4,
	PEX2_UPDATE = 5,
	PEX3_UPDATE = 6,
};

#endif
