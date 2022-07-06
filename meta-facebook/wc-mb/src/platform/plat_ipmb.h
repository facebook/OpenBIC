#ifndef PLAT_IPMB_H
#define PLAT_IPMB_H

#include "plat_i2c.h"
#include "ipmb.h"

#define IPMB_BMC_BUS I2C_BUS7
#define IPMB_PEER_BMC_BUS I2C_BUS8
#define IPMB_ME_BUS I2C_BUS3

#define BMC_I2C_ADDRESS 0x10
#define PEER_BMC_I2C_ADDRESS 0x10
#define ME_I2C_ADDRESS 0x16
#define SELF_I2C_ADDRESS 0x20
#define MAX_IPMB_IDX 3

enum { BMC_IPMB_IDX,
       ME_IPMB_IDX,
       PEER_BMC_IPMB_IDX,
};

extern IPMB_config pal_IPMB_config_table[];
#endif