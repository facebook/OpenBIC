#ifndef PLAT_IPMB_H
#define PLAT_IPMB_H

#include "plat_i2c.h"
#include "ipmb.h"

#define IPMB_BMC_BUS i2c_bus7
#define IPMB_ME_BUS i2c_bus3
#define IPMB_EXP1_BUS i2c_bus8
#define IPMB_EXP2_BUS i2c_bus9
#define IPMB_BB_BIC_BUS i2c_bus8

#define BMC_I2C_ADDRESS 0x10
#define ME_I2C_ADDRESS 0x16
#define SELF_I2C_ADDRESS 0x20
#define BIC1_I2C_ADDRESS 0x20
#define BIC2_I2C_ADDRESS 0x20
#define BB_BIC_I2C_ADDRESS 0x20
#define MAX_IPMB_IDX 5

enum {
	BMC_IPMB_IDX,
	ME_IPMB_IDX,
	EXP1_IPMB_IDX,
	BB_IPMB_IDX = EXP1_IPMB_IDX,
	EXP2_IPMB_IDX,
};

extern IPMB_config pal_IPMB_config_table[];
#endif
