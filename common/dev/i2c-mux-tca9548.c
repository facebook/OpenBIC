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
#include "sensor.h"
#include "hal_i2c.h"
#include "i2c-mux-tca9548.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(i2c_mux_tca9548);

bool tca9548_select_chan(uint8_t sensor_num, void *args)
{
	if (!args || (sensor_num > SENSOR_NUM_MAX)) {
		if (sensor_num > SENSOR_NUM_MAX) {
			LOG_ERR("Invalid channel num");
		}
		else {
			LOG_ERR("args pointer is NULL");
		}
		return false;
	}

	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
	struct tca9548 *p = (struct tca9548 *)args;

	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	msg.bus = cfg->port;
	/* change address to 7-bit */
	msg.target_addr = ((p->addr) >> 1);
	msg.tx_len = 1;
	msg.data[0] = (1 << (p->chan));

	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("I2C master write failed");
		return false;
	}

	return true;
}
