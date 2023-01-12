/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <string.h>
#include "cmsis_os2.h"
#include "plat_i2c.h"
#include "plat_ipmb.h"
#include "plat_ipmi.h"
#include "plat_class.h"

IPMB_config pal_IPMB_config_table[] = {
	// index, interface, channel, bus, channel_target_address, enable_status, self_address,
	// rx_thread_name, tx_thread_name
	{ HD_BIC_IPMB_IDX, I2C_IF, HD_BIC_IPMB, IPMB_HD_BIC_BUS, HD_BIC_I2C_ADDRESS, DISABLE,
	  SELF_I2C_ADDRESS, "RX_HD_BIC_IPMB_TASK", "TX_HD_BIC_IPMB_TASK" },
	{ OPA_BIC_IPMB_IDX, I2C_IF, EXP3_IPMB, IPMB_OPA_BIC_BUS, OPA_BIC_I2C_ADDRESS, DISABLE,
	  SELF_I2C_ADDRESS, "RX_OPA_BIC_IPMB_TASK", "TX_OPA_BIC_IPMB_TASK" },
	{ OPB_BIC_IPMB_IDX, I2C_IF, EXP4_IPMB, IPMB_OPB_BIC_BUS, OPB_BIC_I2C_ADDRESS, DISABLE,
	  SELF_I2C_ADDRESS, "RX_OPB_BIC_IPMB_TASK", "TX_OPB_BIC_IPMB_TASK" },
	{ RESERVED_IDX, RESERVED_IF, RESERVED, RESERVED_BUS, RESERVED_ADDRESS, DISABLE,
	  RESERVED_ADDRESS, "RESERVED_ATTR", "RESERVED_ATTR" },
};

bool pal_load_ipmb_config(void)
{
	switch (get_card_type()) {
	// OPA BIC need communicate with SB BIC and OPB BIC
	case CARD_TYPE_OPA:
		pal_IPMB_config_table[HD_BIC_IPMB_IDX].enable_status = ENABLE;
		pal_IPMB_config_table[OPB_BIC_IPMB_IDX].enable_status = ENABLE;
		if (get_card_position() == CARD_POSITION_1OU) {
			pal_IPMB_config_table[OPB_BIC_IPMB_IDX].channel = EXP2_IPMB;
		}
		break;
	// OPB BIC need communicate with OPA BIC
	case CARD_TYPE_OPB:
		pal_IPMB_config_table[OPA_BIC_IPMB_IDX].enable_status = ENABLE;
		if (get_card_position() == CARD_POSITION_2OU) {
			pal_IPMB_config_table[OPA_BIC_IPMB_IDX].channel = EXP1_IPMB;
		}
		break;
	default:
		break;
	}

	memcpy(&IPMB_config_table[0], &pal_IPMB_config_table[0], sizeof(pal_IPMB_config_table));
	return true;
};
