#include <stdio.h>
#include "cmsis_os2.h"
#include "plat_i2c.h"
#include "plat_ipmi.h"
#include <string.h>
#include "ipmi_def.h"

IPMB_config pal_IPMB_config_table[] = {
	//  index                  interface      interface_source    bus                    Target_addr              EnStatus    target_addr           Rx_attr_name                Tx_attr_name
	{ SLOT1_BIC_IPMB_IDX, I2C_IF, SLOT1_BIC_IFs, IPMB_SLOT1_BIC_BUS, SLOT1_BIC_I2C_ADDRESS,
	  Enable, Self_I2C_ADDRESS, "RX_SLOT1_BIC_IPMB_TASK", "TX_SLOT1_BIC_IPMB_TASK" },
	{ SLOT3_BIC_IPMB_IDX, I2C_IF, SLOT3_BIC_IFs, IPMB_SLOT3_BIC_BUS, SLOT3_BIC_I2C_ADDRESS,
	  Enable, Self_I2C_ADDRESS, "RX_SLOT3_BIC_IPMB_TASK", "TX_SLOT3_BIC_IPMB_TASK" },
	{ IPMB_RESERVE_IDX, Reserve_IF, Reserve_IFs, Reserve_BUS, Reserve_ADDRESS, Disable,
	  Reserve_ADDRESS, "Reserve_ATTR", "Reserve_ATTR" },
};

bool pal_load_ipmb_config(void)
{
	memcpy(&IPMB_config_table[0], &pal_IPMB_config_table[0], sizeof(pal_IPMB_config_table));
	return true;
};
