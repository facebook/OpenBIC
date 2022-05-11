#ifndef PLAT_I2C_h
#define PLAT_I2C_h

#include "hal_i2c.h"

// map i2c bus to peripherial bus
// i2c peripheral 1 based, as used i2c index 0 in firmware.
#define I2C_BUS1 0
#define I2C_BUS2 1
#define I2C_BUS3 2
#define I2C_BUS4 3
#define I2C_BUS5 4
#define I2C_BUS6 5
#define I2C_BUS7 6
#define I2C_BUS8 7
#define I2C_BUS9 8
#define I2C_BUS10 9

#define I2C_BUS_MAX_NUM 10

#define CPLD_IO_I2C_BUS I2C_BUS1
#define CPLD_IO_I2C_ADDR (0x1E >> 1)

#define CPLD_IO_REG_OFS_HSC_EN_SLOT1 0x09
#define CPLD_IO_REG_OFS_HSC_EN_SLOT3 0x0B
#define CPLD_IO_REG_OFS_SLED_CYCLE 0x2B
#define CPLD_IO_REG_CABLE_PRESENT 0x30

#endif
