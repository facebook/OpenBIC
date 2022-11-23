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
#include "eeprom.h"
#include "hal_i2c.h"
#include <string.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(dev_eeprom);

bool eeprom_mux_check(EEPROM_ENTRY *entry)
{
	if (entry == NULL) {
		LOG_DBG("entry pointer passed in as NULL");
		return false;
	}
	I2C_MSG msg;
	uint8_t retry = 5;
	if (entry->config.mux_present) {
		msg.bus = entry->config.port;
		msg.target_addr = entry->config.mux_addr;
		msg.tx_len = 1;
		msg.data[0] = (1 << (entry->config.mux_channel));
		return ((i2c_master_write(&msg, retry) == 0) ? true : false);
	} else {
		return true;
	}
}

bool eeprom_write(EEPROM_ENTRY *entry)
{
	if (entry == NULL) {
		LOG_DBG("entry pointer passed in as NULL");
		return false;
	}
	I2C_MSG msg;
	uint8_t retry = 5;
	uint8_t i;

	if (entry->config.bus_mutex) {
		if (k_mutex_lock(entry->config.bus_mutex, K_MSEC(1000))) {
			LOG_ERR("Failed to lock mutex on bus %d", entry->config.port);
			return false;
		}
	}

	for (i = 0; i < retry; i++) {
		/* Check if there have a MUX before EEPROM and access it to change channel first */
		if (eeprom_mux_check(entry) == false)
			continue;

		msg.bus = entry->config.port;
		msg.target_addr = entry->config.target_addr;
		msg.tx_len = entry->data_len + 2; // write 2 byte offset to EEPROM
		msg.data[0] =
			((entry->config.start_offset + entry->offset) >> 8) & 0xFF; // offset msb
		msg.data[1] = (entry->config.start_offset + entry->offset) & 0xFF; // offset lsb
		memcpy(&msg.data[2], &entry->data[0], entry->data_len);

		if (i2c_master_write(&msg, retry) == 0)
			break;
	}

	if (entry->config.bus_mutex) {
		if (k_mutex_unlock(entry->config.bus_mutex))
			LOG_ERR("Failed to unlock mutex on bus %d", entry->config.port);
	}

	return ((i == retry) ? false : true);
}

bool eeprom_read(EEPROM_ENTRY *entry)
{
	if (entry == NULL) {
		LOG_DBG("entry pointer passed in as NULL");
		return false;
	}

	I2C_MSG msg;
	uint8_t retry = 5;
	uint8_t i;

	if (entry->config.bus_mutex) {
		if (k_mutex_lock(entry->config.bus_mutex, K_MSEC(1000))) {
			LOG_ERR("Failed to lock mutex on bus %d", entry->config.port);
			return false;
		}
	}

	for (i = 0; i < retry; i++) {
		/* Check if there have a MUX before EEPROM and access it to change channel first */
		if (eeprom_mux_check(entry) == false)
			continue;

		msg.bus = entry->config.port;
		msg.target_addr = entry->config.target_addr;
		msg.tx_len = 2; // write 2 byte offset to EEPROM
		msg.rx_len = entry->data_len;
		msg.data[0] =
			((entry->config.start_offset + entry->offset) >> 8) & 0xFF; // offset msb
		msg.data[1] = (entry->config.start_offset + entry->offset) & 0xFF; // offset lsb

		if (i2c_master_read(&msg, retry) == 0) {
			entry->data_len = msg.rx_len;
			memcpy(&entry->data, &msg.data, msg.rx_len);
			break;
		}
	}

	if (entry->config.bus_mutex) {
		if (k_mutex_unlock(entry->config.bus_mutex))
			LOG_ERR("Failed to unlock mutex on bus %d", entry->config.port);
	}

	return ((i == retry) ? false : true);
}
