#ifndef PLAT_IPMB_H
#define PLAT_IPMB_H

#include "plat_i2c.h"
#include "ipmb.h"

#define IPMB_BMC_BUS I2C_BUS1
#define IPMB_ME_BUS I2C_BUS6
#define IPMB_EXP1_BUS I2C_BUS3
#define IPMB_EXP2_BUS I2C_BUS4
#define IPMB_BB_BIC_BUS I2C_BUS3

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
