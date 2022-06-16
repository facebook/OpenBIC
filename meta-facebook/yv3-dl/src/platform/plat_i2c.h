#ifndef PLAT_I2C_h
#define PLAT_I2C_h

#include "hal_i2c.h"

enum {
	I2C_BUS0,
	I2C_BUS1,
	I2C_BUS2,
	I2C_BUS3,
	I2C_BUS4,
	I2C_BUS5,
	I2C_BUS6,
	I2C_BUS7,
	I2C_BUS8,
	I2C_BUS9,
};

#define IPMB_I2C_BMC I2C_BUS1
#define I2C_BUS_MAX_NUM 10

#endif
