#ifndef PLAT_IPMB_H
#define PLAT_IPMB_H

#include "plat_i2c.h"

#define IPMB_SLOT1_BIC_BUS I2C_BUS7
#define IPMB_SLOT3_BIC_BUS I2C_BUS8

// preserve for later use
#define BMC_I2C_ADDRESS 0x10
#define SELF_I2C_ADDRESS 0x20
#define SLOT1_BIC_I2C_ADDRESS 0x20
#define SLOT3_BIC_I2C_ADDRESS 0x20
#define MAX_IPMB_IDX 2

enum {
	SLOT1_BIC_IPMB_IDX,
	SLOT3_BIC_IPMB_IDX,
	RESERVE_IPMB_IDX,
};

#endif
