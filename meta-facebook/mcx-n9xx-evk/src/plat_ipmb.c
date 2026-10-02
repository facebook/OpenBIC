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

#include "ipmb.h"
#include "plat_ipmb.h"
#include "plat_i2c.h"

bool pal_load_ipmb_config(void)
{
	IPMB_config_table[BIC_IPMB_IDX] = (IPMB_config){
		.index = BIC_IPMB_IDX,
		.interface = I2C_IF,
		.channel = BMC_IPMB,
		.bus = I2C_BUS_LCD_HEADER,
		.channel_target_address = 0x20, /* traditional IPMI BMC address */
		.enable_status = true,
		.self_address = BIC_IPMB_ADDRESS,
		.rx_thread_name = "IPMB_RX0",
		.tx_thread_name = "IPMB_TX0",
	};

	/* Terminator entry - channel_index_mapping() and other common/
	 * service/ipmb/ipmb.c loops scan for RESERVED_IDX to know where
	 * the real entries end; without this they walk off the end of
	 * the array into whatever heap memory follows it. */
	IPMB_config_table[BIC_IPMB_IDX + 1] = (IPMB_config){
		.index = RESERVED_IDX,
		.interface = RESERVED_IF,
		.channel = RESERVED,
		.bus = RESERVED_BUS,
		.channel_target_address = RESERVED_ADDRESS,
		.enable_status = false,
	};

	return true;
}
