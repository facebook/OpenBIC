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

#include "plat_altera.h"
#include "altera.h"

altera_max10_attr plat_altera_max10_config = { CPLD_UPDATE_I2C_BUS, CPLD_UPDATE_ADDR,
					       M04_CFM1_START_ADDR, M04_CFM1_END_ADDR };

int pal_load_altera_max10_attr(altera_max10_attr *altera_max10_config)
{
	if (altera_max10_config == NULL) {
		return -1;
	}
	*altera_max10_config = plat_altera_max10_config;
	return 0;
};
