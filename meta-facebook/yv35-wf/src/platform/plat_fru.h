#ifndef PLAT_FRU_H
#define PLAT_FRU_H
#include "plat_i2c.h"

#define WF_FRU_PORT I2C_BUS3
#define WF_FRU_ADDR (0xA8 >> 1)

enum FRU_ID {
	WF_FRU_ID,
	MAX_FRU_ID,
};

#endif
