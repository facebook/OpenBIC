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

#include <stdlib.h>
#include <stdio.h>
#include <logging/log.h>
#include "plat_sensor_table.h"
#include "sensor.h"

LOG_MODULE_REGISTER(plat_sensor_table);

sensor_cfg plat_sensor_config[] = {
	/* number, type, port, address, offset, access check, arg0, arg1, cache, cache_status,
	   pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_fn,
	   init_arg */
};

const int SENSOR_CONFIG_SIZE = ARRAY_SIZE(plat_sensor_config);

void load_sensor_config(void)
{
	memcpy(sensor_config, plat_sensor_config, SENSOR_CONFIG_SIZE * sizeof(sensor_cfg));
	sensor_config_count = SENSOR_CONFIG_SIZE;
}
