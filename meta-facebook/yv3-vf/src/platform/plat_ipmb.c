#include <stdio.h>
#include "cmsis_os2.h"
#include <string.h>
#include "ipmb.h"
#include "plat_i2c.h"
#include "plat_ipmb.h"
#include "plat_ipmi.h"

IPMB_config pal_IPMB_config_table[] = {
	// index, interface, channel, bus, channel_target_address, enable_status, self_address,
	// rx_thread_name, tx_thread_name
	{ CL_BIC_IPMB_IDX, I2C_IF, CL_BIC_IPMB, IPMB_CL_BIC_BUS, CL_BIC_I2C_ADDRESS, ENABLE,
	  SELF_I2C_ADDRESS, "RX_CL_BIC_IPMB_TASK", "TX_CL_BIC_IPMB_TASK" },
	{ RESERVED_IDX, RESERVED_IF, RESERVED, RESERVED_BUS, RESERVED_ADDRESS, DISABLE,
	  RESERVED_ADDRESS, "RESERVED_ATTR", "RESERVED_ATTR" },
};

bool pal_load_ipmb_config(void)
{
	memcpy(&IPMB_config_table[0], &pal_IPMB_config_table[0], sizeof(pal_IPMB_config_table));
	return true;
};
