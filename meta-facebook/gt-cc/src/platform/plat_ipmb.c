#include <stdio.h>
#include "cmsis_os2.h"
#include <string.h>
#include "plat_i2c.h"
#include "plat_ipmb.h"
#include "plat_ipmi.h"
#include "plat_class.h"

IPMB_config pal_IPMB_config_table[] = {
	// index, interface, channel, bus, channel_target_address, enable_status, self_address,
	// rx_thread_name, tx_thread_name
};
