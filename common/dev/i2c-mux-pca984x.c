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
#include "sensor.h"
#include "libutil.h"
#include "hal_i2c.h"
#include "i2c-mux-pca984x.h"

LOG_MODULE_REGISTER(i2c_mux_pca9846);

bool set_pca9846_channel_and_transfer(uint8_t bus, uint8_t mux_addr, uint8_t mux_channel,
				      uint8_t tran_type, I2C_MSG *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);

	bool ret = true;
	int status = 0;
	int retry = 5;

	/* Set channel */
	I2C_MSG mux_msg = { 0 };
	mux_msg.bus = bus;
	mux_msg.target_addr = mux_addr;
	mux_msg.tx_len = 1;
	mux_msg.data[0] = mux_channel;

	/* Mutex lock */
	status = k_mutex_lock(&msg->lock, K_MSEC(PCA9846_MUTEX_LOCK_MS));
	if (status != 0) {
		LOG_ERR("Mutex lock fail, status: %d, bus: %d, mux addr: 0x%x", status, bus,
			mux_addr);
		return false;
	}

	status = i2c_master_write(&mux_msg, retry);
	if (status != 0) {
		LOG_ERR("Set channel fail, status: %d", status);
		ret = false;
		goto mutex_unlock;
	}

	/* Transfer via i2c */
	switch (tran_type) {
	case I2C_READ:
		status = i2c_master_read(msg, retry);
		break;
	case I2C_WRITE:
		status = i2c_master_write(msg, retry);
		break;
	default:
		LOG_ERR("Transfer type is invalid, transfer type: %d", tran_type);
		ret = false;
		goto mutex_unlock;
	}

	if (status != 0) {
		LOG_ERR("Transfer msg fail, status: %d", status);
		ret = false;
		goto mutex_unlock;
	}

	/* Disable all channels */
	mux_msg.bus = bus;
	mux_msg.target_addr = mux_addr;
	mux_msg.tx_len = 1;
	mux_msg.data[0] = PCA9846_DEFAULT_CHANNEL;

	status = i2c_master_write(&mux_msg, retry);
	if (status != 0) {
		LOG_ERR("Disable all channels fail, status: %d", status);
		ret = false;
	}

mutex_unlock:
	status = k_mutex_unlock(&msg->lock);
	if (status != 0) {
		LOG_ERR("Mutex unlock fail, status: %d", status);
		ret = false;
	}

	return ret;
}
