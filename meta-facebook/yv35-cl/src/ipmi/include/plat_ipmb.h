#ifndef PLAT_IPMB_H
#define PLAT_IPMB_H

#include "plat_i2c.h"

#define IPMB_BMC_BUS  i2c_bus7
#define IPMB_ME_BUS   i2c_bus3
#define IPMB_EXP1_BUS i2c_bus8
#define IPMB_EXP2_BUS i2c_bus9
#define Reserve_BUS 0xff

#define BMC_I2C_ADDRESS  0x10
#define ME_I2C_ADDRESS   0x16
#define Self_I2C_ADDRESS 0x20
#define BIC0_I2C_ADDRESS 0x2A
#define BIC1_I2C_ADDRESS 0x2B
#define Reserve_ADDRESS  0xff

enum {
	BMC_IPMB_IDX,
	ME_IPMB_IDX,
	EXP1_IPMB_IDX,
	EXP2_IPMB_IDX,
	RESERVE_IPMB_IDX,
};

#endif
