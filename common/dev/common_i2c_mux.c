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
#include <logging/log.h>
#include "libutil.h"
#include "hal_i2c.h"
#include "common_i2c_mux.h"

LOG_MODULE_REGISTER(common_i2c_mux);

bool set_mux_channel(mux_config mux_cfg)
{
	int status = 0;
	int retry = 5;

	/* Set channel */
	I2C_MSG mux_msg = { 0 };
	mux_msg.bus = mux_cfg.bus;
	mux_msg.target_addr = mux_cfg.target_addr;
	mux_msg.tx_len = 1;
	mux_msg.data[0] = mux_cfg.channel;

	status = i2c_master_write(&mux_msg, retry);
	if (status != 0) {
		LOG_ERR("set channel fail, status: %d, bus: %d, addr: 0x%x", status, mux_cfg.bus,
			mux_cfg.target_addr);
		return false;
	}

	return true;
}
