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
#include <logging/log.h>

#include "rg3mxxb12.h"
#include "hal_i2c.h"
#include "libutil.h"

LOG_MODULE_REGISTER(dev_rg3mxxb12);

#define RETRY 3

static bool rg3mxxb12_register_read(uint8_t bus, uint8_t offset, uint8_t *value)
{
	CHECK_NULL_ARG_WITH_RETURN(value, false);
	int ret = -1;
	I2C_MSG msg = { 0 };

	msg.bus = bus;
	msg.target_addr = RG3MXXB12_DEFAULT_STATIC_ADDRESS;
	msg.tx_len = 1;
	msg.data[0] = offset;
	msg.rx_len = 1;
	ret = i2c_master_read(&msg, RETRY);
	if (ret != 0) {
		LOG_ERR("Failed to read rg3mxxb12 register 0x%x, bus-%d", offset, bus);
		return false;
	}
	*value = msg.data[0];

	return true;
}

static bool rg3mxxb12_register_write(uint8_t bus, uint8_t offset, uint8_t value)
{
	int ret = -1;
	I2C_MSG msg = { 0 };

	msg.bus = bus;
	msg.target_addr = RG3MXXB12_DEFAULT_STATIC_ADDRESS;
	msg.tx_len = 2;
	msg.data[0] = offset;
	msg.data[1] = value;
	ret = i2c_master_write(&msg, RETRY);
	if (ret != 0) {
		LOG_ERR("Failed to write rg3mxxb12 register 0x%x, bus-%d", offset, bus);
		return false;
	}

	msg.tx_len = 1;
	msg.data[0] = offset;
	msg.rx_len = 1;
	ret = i2c_master_read(&msg, RETRY);
	if ((ret != 0) || (msg.data[0] != value)) {
		LOG_ERR("Failed to read rg3mxxb12 register 0x%x after setting,"
			"bus-%d",
			offset, bus);
		return false;
	}

	return true;
}

static bool rg3mxxb12_protected_register(uint8_t bus, bool lock)
{
	uint8_t val = lock ? PROTECTION_LOCK : PROTECTION_UNLOCK;
	if (!rg3mxxb12_register_write(bus, RG3MXXB12_PROTECTION_REG, val)) {
		return false;
	}
	return true;
}

bool rg3mxxb12_select_slave_port_connect(uint8_t bus, uint8_t slave_port)
{
	// Connect selected slave port to HUB network
	if (!rg3mxxb12_register_write(bus, RG3MXXB12_SSPORTS_HUB_NETWORK_CONNECTION, slave_port)) {
		return false;
	}
	return true;
}

bool rg3mxxb12_i2c_mode_only_init(uint8_t bus, uint8_t slave_port, uint8_t ldo_volt,
				  uint8_t pullup_resistor)
{
	bool ret = false;
	uint8_t value;
	// Unlock protected regsiter
	if (!rg3mxxb12_protected_register(bus, false)) {
		return false;
	}

	// Set Low-Dropout Regulators(LDO) voltage to VIOM and VIOS
	value = (ldo_volt << VIOM0_OFFSET) | (ldo_volt << VIOM1_OFFSET) |
		(ldo_volt << VIOS0_OFFSET) | (ldo_volt << VIOS1_OFFSET);
	if (!rg3mxxb12_register_write(bus, RG3MXXB12_VOLT_LDO_SETTING, value)) {
		goto out;
	}

	// Disable all slave ports
	if (!rg3mxxb12_register_write(bus, RG3MXXB12_SLAVE_PORT_ENABLE, 0x00)) {
		goto out;
	}

	// Disable the SMBus agent mode for selected slave port
	if (!rg3mxxb12_register_write(bus, RG3MXXB12_SSPORTS_AGENT_ENABLE, 0x00)) {
		goto out;
	}

	// Disable GPIO mode for selected slave port
	if (!rg3mxxb12_register_write(bus, RG3MXXB12_SSPORTS_GPIO_ENABLE, 0x00)) {
		goto out;
	}

	// Enable master port OD only mode
	if (!rg3mxxb12_register_read(bus, RG3MXXB12_MASTER_PORT_CONFIG, &value)) {
		goto out;
	}
	value = SETBIT(value, MPORT_OD_ONLY);
	if (!rg3mxxb12_register_write(bus, RG3MXXB12_MASTER_PORT_CONFIG, value)) {
		goto out;
	}

	// Clear I3C HUB network traffic protocol setting
	if (!rg3mxxb12_register_read(bus, RG3MXXB12_HUB_NETWORK_OPERATION_MODE, &value)) {
		goto out;
	}
	value = CLEARBIT(value, HUB_NETWORK_ALWAYS_I3C);
	if (!rg3mxxb12_register_write(bus, RG3MXXB12_HUB_NETWORK_OPERATION_MODE, value)) {
		goto out;
	}

	// Setting pull up resistor for slave ports
	if (!rg3mxxb12_register_read(bus, RG3MXXB12_SSPORTS_PULLUP_SETTING, &value)) {
		goto out;
	}
	value = SETBITS(value, pullup_resistor, SSPORTS_RESISTOR0_OFFSET);
	value = SETBITS(value, pullup_resistor, SSPORTS_RESISTOR1_OFFSET);
	if (!rg3mxxb12_register_write(bus, RG3MXXB12_SSPORTS_PULLUP_SETTING, value)) {
		goto out;
	}

	// Enable internal pull up resistor connection for slave ports
	if (!rg3mxxb12_register_write(bus, RG3MXXB12_SSPORTS_PULLUP_ENABLE, 0xFF)) {
		goto out;
	}

	// Set open drain and push pull compatible mode for selected salve port
	if (!rg3mxxb12_register_write(bus, RG3MXXB12_SSPORTS_OD_ONLY, slave_port)) {
		goto out;
	}

	// Enable selected slave port
	if (!rg3mxxb12_register_write(bus, RG3MXXB12_SLAVE_PORT_ENABLE, slave_port)) {
		goto out;
	}

	// Connect selected slave port to HUB network
	if (!rg3mxxb12_register_write(bus, RG3MXXB12_SSPORTS_HUB_NETWORK_CONNECTION, slave_port)) {
		goto out;
	}

	ret = true;
out:
	// Lock protected register
	if (!rg3mxxb12_protected_register(bus, true)) {
		return false;
	}

	return ret;
}
