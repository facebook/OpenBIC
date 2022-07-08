#ifndef PLAT_IPMB_H
#define PLAT_IPMB_H

#include "plat_i2c.h"
#include "ipmb.h"

#define IPMB_BMC_BUS I2C_BUS7
#define IPMB_EXP1_BUS I2C_BUS8
#define IPMB_EXP2_BUS I2C_BUS9

#define BMC_I2C_ADDRESS 0x10
#define SELF_I2C_ADDRESS 0x20
#define BIC1_I2C_ADDRESS 0x20
#define BIC2_I2C_ADDRESS 0x20
#define MAX_IPMB_IDX 4

enum {
	BMC_IPMB_IDX,
	EXP1_IPMB_IDX,
	EXP2_IPMB_IDX,
};

extern IPMB_config pal_IPMB_config_table[];
#endif
