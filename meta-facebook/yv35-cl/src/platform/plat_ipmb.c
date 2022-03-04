#include <stdio.h>
#include "cmsis_os2.h"
#include <string.h>
#include "plat_i2c.h"
#include "plat_ipmb.h"
#include "plat_ipmi.h"
#include "plat_class.h"

IPMB_config pal_IPMB_config_table[] = {
	{ BMC_IPMB_IDX, I2C_IF, BMC_IPMB, IPMB_I2C_BMC, BMC_I2C_ADDRESS, ENABLE, SELF_I2C_ADDRESS,
	  "RX_BMC_IPMB_TASK", "TX_BMC_IPMB_TASK" },
	{ ME_IPMB_IDX, I2C_IF, ME_IPMB, IPMB_ME_BUS, ME_I2C_ADDRESS, ENABLE, SELF_I2C_ADDRESS,
	  "RX_ME_IPMB_TASK", "TX_ME_IPMB_TASK" },
	{ EXP1_IPMB_IDX, I2C_IF, EXP1_IPMB, IPMB_EXP1_BUS, BIC1_I2C_ADDRESS, DISABLE,
	  SELF_I2C_ADDRESS, "RX_EPX1_IPMB_TASK", "TX_EXP1_IPMB_TASK" },
	{ EXP2_IPMB_IDX, I2C_IF, EXP2_IPMB, IPMB_EXP2_BUS, BIC2_I2C_ADDRESS, DISABLE,
	  SELF_I2C_ADDRESS, "RX_EPX2_IPMB_TASK", "TX_EXP2_IPMB_TASK" },
	{ RESERVED_IDX, RESERVED_IF, RESERVED, RESERVED_BUS, RESERVED_ADDRESS, DISABLE,
	  RESERVED_ADDRESS, "RESERVED_ATTR", "RESERVED_ATTR" },
};

bool pal_load_ipmb_config(void)
{
	bool bic_class;
	bic_class = get_bic_class();

	// class1 1ou ipmi bus and class2 bb ipmi bus shared same i2c bus
	if (get_1ou_status() && (bic_class == sys_class_1)) {
		pal_IPMB_config_table[EXP1_IPMB_IDX].enable_status = ENABLE;
	} else if (get_1ou_status() && (bic_class == sys_class_2)) {
		pal_IPMB_config_table[EXP1_IPMB_IDX].index = BB_IPMB_IDX;
		pal_IPMB_config_table[BB_IPMB_IDX].channel = BB_IPMB;
		pal_IPMB_config_table[BB_IPMB_IDX].bus = IPMB_BB_BIC_BUS;
		pal_IPMB_config_table[BB_IPMB_IDX].channel_target_address = BB_BIC_I2C_ADDRESS;
		pal_IPMB_config_table[BB_IPMB_IDX].rx_thread_name = "RX_BB_BIC_IPMB_TASK";
		pal_IPMB_config_table[BB_IPMB_IDX].tx_thread_name = "TX_BB_BIC_IPMB_TASK";
		pal_IPMB_config_table[BB_IPMB_IDX].enable_status = ENABLE;
	}

	if (get_2ou_status()) { // check present status
		uint8_t card_type_2ou;
		card_type_2ou = get_2ou_cardtype();

		// for dpv2 sku, disable ipmb and set i2c freq to 400Khz for slave devices reading
		// for reset of expansion board, enable ipmb and set i2c freq to 1Mhz
		if ((card_type_2ou == type_2ou_dpv2) || (card_type_2ou == type_2ou_dpv2_8) ||
		    (card_type_2ou == type_2ou_dpv2_16)) {
			i2c_freq_set(pal_IPMB_config_table[EXP2_IPMB_IDX].bus, I2C_SPEED_FAST);
			pal_IPMB_config_table[EXP2_IPMB_IDX].enable_status = DISABLE;
		} else {
			pal_IPMB_config_table[EXP2_IPMB_IDX].enable_status = ENABLE;
		}
	}

	memcpy(&IPMB_config_table[0], &pal_IPMB_config_table[0], sizeof(pal_IPMB_config_table));
	return 1;
};
