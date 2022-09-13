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
#include "cmsis_os2.h"
#include "plat_i2c.h"
#include "plat_ipmi.h"
#include "plat_ipmb.h"
#include <string.h>
#include "ipmb.h"

IPMB_config pal_IPMB_config_table[] = {
	{ SLOT1_BIC_IPMB_IDX, I2C_IF, SLOT1_BIC, IPMB_SLOT1_BIC_BUS, SLOT1_BIC_I2C_ADDRESS, ENABLE,
	  SELF_I2C_ADDRESS, "RX_SLOT1_BIC_IPMB_TASK", "TX_SLOT1_BIC_IPMB_TASK" },
	{ SLOT3_BIC_IPMB_IDX, I2C_IF, SLOT3_BIC, IPMB_SLOT3_BIC_BUS, SLOT3_BIC_I2C_ADDRESS, ENABLE,
	  SELF_I2C_ADDRESS, "RX_SLOT3_BIC_IPMB_TASK", "TX_SLOT3_BIC_IPMB_TASK" },
	{ RESERVED_IDX, RESERVED_IF, RESERVED, RESERVED_BUS, RESERVED_ADDRESS, DISABLE,
	  RESERVED_ADDRESS, "RESERVED_ATTR", "RESERVED_ATTR" },
};

bool pal_load_ipmb_config(void)
{
	memcpy(&IPMB_config_table[0], &pal_IPMB_config_table[0], sizeof(pal_IPMB_config_table));
	return true;
};
