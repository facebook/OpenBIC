#ifndef PLAT_IPMB_H
#define PLAT_IPMB_H

#include "plat_i2c.h"
#include "ipmb.h"

#define SELF_I2C_ADDRESS 0x20

#define MAX_IPMB_IDX 0

extern IPMB_config pal_IPMB_config_table[];

#endif
