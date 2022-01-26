#ifndef PLAT_I2C_h
#define PLAT_I2C_h

#include "hal_i2c.h"

// map i2c bus to peripherial bus
// i2c peripheral 1 based, as used i2c index 0 in firmware.
#define i2c_bus1  0
#define i2c_bus2  1
#define i2c_bus3  2
#define i2c_bus4  3
#define i2c_bus5  4
#define i2c_bus6  5
#define i2c_bus7  6
#define i2c_bus8  7
#define i2c_bus9  8
#define i2c_bus10 9

#define I2C_BUS_NUM 10

#define CPLD_IO_I2C_BUS   i2c_bus1
#define CPLD_IO_I2C_ADDR  (0x1E >> 1)

#define CPLD_IO_REG_OFS_HSC_EN_SLOT1 0x09
#define CPLD_IO_REG_OFS_HSC_EN_SLOT3 0x0B
#define CPLD_IO_REG_OFS_SLED_CYCLE 0x2B

#endif
