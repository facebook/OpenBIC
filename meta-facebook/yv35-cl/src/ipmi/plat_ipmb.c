#include <stdio.h>
#include "cmsis_os2.h"
#include "plat_i2c.h"
#include "plat_ipmi.h"
#include <string.h>
#include "ipmi_def.h"

IPMB_config pal_IPMB_config_table[] = {
	//   index             interface         interface_source  bus              Target_addr          EnStatus  slave_addr            Rx_attr_name          Tx_attr_name//
	{ BMC_IPMB_IDX,     I2C_IF,           BMC_IPMB_IFs,     IPMB_I2C_BMC,    BMC_I2C_ADDRESS,     Enable,   Self_I2C_ADDRESS,     "RX_BMC_IPMB_TASK",   "TX_BMC_IPMB_TASK"  },
	{ ME_IPMB_IDX,      I2C_IF,           ME_IPMB_IFs,      IPMB_ME_BUS,     ME_I2C_ADDRESS,      Enable,   BMC_I2C_ADDRESS,     "RX_ME_IPMB_TASK",    "TX_ME_IPMB_TASK"   },
	{ EXP1_IPMB_IDX,    I2C_IF,           EXP1_IPMB_IFs,    IPMB_EXP1_BUS,   BIC0_I2C_ADDRESS,    Disable,  BIC1_I2C_ADDRESS,     "RX_EPX0_IPMB_TASK",  "TX_EXP0_IPMB_TASK" },
	{ EXP2_IPMB_IDX,    I2C_IF,           EXP2_IPMB_IFs,    IPMB_EXP2_BUS,   BIC1_I2C_ADDRESS,    Disable,  BIC0_I2C_ADDRESS,     "RX_EPX1_IPMB_TASK",  "TX_EXP1_IPMB_TASK" },
	{ RESERVE_IPMB_IDX, Reserve_IF,       Reserve_IFs,      Reserve_BUS,     Reserve_ADDRESS,     Disable,  Reserve_ADDRESS,      "Reserve_ATTR",       "Reserve_ATTR"      },
};

bool pal_load_IPMB_config(void)
{
	memcpy(&IPMB_config_table[0], &pal_IPMB_config_table[0], sizeof(pal_IPMB_config_table));
	return 1;
};

