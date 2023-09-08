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

#include "plat_pdr_table.h"

#include <stdio.h>
#include <string.h>

#include "pdr.h"
#include "sensor.h"
#include "plat_sensor_table.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(plat_pdr_table);

PDR_numeric_sensor plat_pdr_table[] = {
	{
		//PDR common header
		{

		},
		//numeric sensor format
	},
};

const int PDR_TABLE_SIZE = ARRAY_SIZE(plat_pdr_table);
uint16_t plat_get_pdr_size()
{
	return PDR_TABLE_SIZE;
}

void plat_load_pdr_table(PDR_numeric_sensor *numeric_sensor_table)
{
	memcpy(numeric_sensor_table, plat_pdr_table, sizeof(plat_pdr_table));
}
