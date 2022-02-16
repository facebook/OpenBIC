#ifndef PLAT_IPMB_H
#define PLAT_IPMB_H

#include "plat_i2c.h"

#define IPMB_SLOT1_BIC_BUS i2c_bus7
#define IPMB_SLOT3_BIC_BUS i2c_bus8
#define Reserve_BUS 0xff

// preserve for later use
#define BMC_I2C_ADDRESS 0x10
#define Self_I2C_ADDRESS 0x20
#define SLOT1_BIC_I2C_ADDRESS 0x20
#define SLOT3_BIC_I2C_ADDRESS 0x20
#define Reserve_ADDRESS 0xff

enum {
	SLOT1_BIC_IPMB_IDX,
	SLOT3_BIC_IPMB_IDX,
	RESERVE_IPMB_IDX,
};

#endif
