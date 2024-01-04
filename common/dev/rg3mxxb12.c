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
#include "libutil.h"

LOG_MODULE_REGISTER(dev_rg3mxxb12);

#define RETRY 3

static bool rg3mxxb12_register_read_i3c(uint8_t bus, uint8_t offset, uint8_t *value)
{
	CHECK_NULL_ARG_WITH_RETURN(value, false);
	int ret = -1;
	I3C_MSG i3c_msg = { 0 };

	i3c_msg.bus = bus;
	i3c_msg.target_addr = RG3MXXB12_DEFAULT_STATIC_ADDRESS;
	i3c_msg.tx_len = 1;
	i3c_msg.data[0] = offset;
	i3c_msg.rx_len = 1;
	ret = i3c_transfer(&i3c_msg);
	if (ret != 0) {
		LOG_ERR("Failed to read rg3mxxb12 register via I3C 0x%x, bus-%d ret = %d", offset,
			bus, ret);
		return false;
	}
	*value = i3c_msg.data[0];

	return true;
}

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
	uint8_t val = lock ? RG3MXXB12_PROTECTION_LOCK : RG3MXXB12_PROTECTION_UNLOCK;
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

bool rg3mxxb12_get_device_info(uint8_t bus, uint16_t *i3c_hub_type)
{
	uint8_t device_info0, device_info1 = 0;

	if (rg3mxxb12_register_read(bus, RG3MXXB12_DEVICE_INFO0_REG, &device_info0)) {
		rg3mxxb12_register_read(bus, RG3MXXB12_DEVICE_INFO1_REG, &device_info1);
	} else {
		return false;
	}

	*i3c_hub_type = (device_info1 << 8) | device_info0;
	return true;
}

bool rg3mxxb12_get_device_info_i3c(uint8_t bus, uint16_t *i3c_hub_type)
{
	uint8_t device_info0, device_info1 = 0;

	if (rg3mxxb12_register_read_i3c(bus, RG3MXXB12_DEVICE_INFO0_REG, &device_info0)) {
		rg3mxxb12_register_read_i3c(bus, RG3MXXB12_DEVICE_INFO1_REG, &device_info1);
	} else {
		return false;
	}

	*i3c_hub_type = (device_info1 << 8) | device_info0;
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
	value = CLEARBIT(value, RG3MXXB12_HUB_NETWORK_ALWAYS_I3C);
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

bool rg3mxxb12_i3c_mode_only_init(I3C_MSG *i3c_msg, uint8_t ldo_volt)
{
	bool ret = false;

	uint8_t cmd_unprotect[2] = { RG3MXXB12_PROTECTION_REG, 0x69 };
	uint8_t cmd_protect[2] = { RG3MXXB12_PROTECTION_REG, 0x00 };
	uint8_t cmd_initial[][2] = {
		/* 
		 * Refer to RG3MxxB12 datasheet page 13, LDO voltage depends
		 * on each project's hard design
		 */
		{ RG3MXXB12_VOLT_LDO_SETTING, ldo_volt },
		{ RG3MXXB12_SSPORTS_AGENT_ENABLE, 0x0 },
		{ RG3MXXB12_SSPORTS_GPIO_ENABLE, 0x0 },
		{ RG3MXXB12_SLAVE_PORT_ENABLE, 0x0 },
		{ RG3MXXB12_SSPORTS_PULLUP_ENABLE, 0xFF },
		{ RG3MXXB12_SSPORTS_OD_ONLY, 0x0 },
		{ RG3MXXB12_SLAVE_PORT_ENABLE, 0xFF },
		{ RG3MXXB12_SSPORTS_HUB_NETWORK_CONNECTION, 0xFF },
	};

	i3c_msg->tx_len = 2;
	memcpy(i3c_msg->data, cmd_unprotect, 2);
	int initial_cmd_size = sizeof(cmd_initial) / sizeof(cmd_initial[0]);

	// Unlock protected regsiter
	if (!i3c_controller_write(i3c_msg)) {
		goto out;
	}

	for (int cmd = 0; cmd < initial_cmd_size; cmd++) {
		i3c_msg->tx_len = 2;
		memcpy(i3c_msg->data, cmd_initial[cmd], 2);
		if (!i3c_controller_write(i3c_msg)) {
			LOG_ERR("Failed to initial i3c mode. offset = 0x%02x, value = 0x%02x",
				cmd_initial[cmd][0], cmd_initial[cmd][1]);
			goto out;
		}
		k_msleep(10);
	}

	ret = true;
out:
	memcpy(i3c_msg->data, cmd_protect, 2);
	if (!i3c_controller_write(i3c_msg)) {
		goto out;
	}

	return ret;
}

bool rg3mxxb12_set_slave_port(uint8_t bus, uint8_t addr, uint8_t setting)
{
	I3C_MSG i3c_msg = { 0 };
	int ret = 0;
	uint8_t cmd_unprotect[2] = { RG3MXXB12_PROTECTION_REG, 0x69 };
	uint8_t cmd_protect[2] = { RG3MXXB12_PROTECTION_REG, 0x00 };

	i3c_msg.bus = bus;
	i3c_msg.target_addr = addr;
	i3c_msg.tx_len = 2;
	memcpy(&i3c_msg.data, cmd_unprotect, 2);
	if (!i3c_controller_write(&i3c_msg)) {
		LOG_ERR("Failed to unprotect slave port register");
		ret = false;
		goto out;
	}

	memset(&i3c_msg.data, 0, 2);
	i3c_msg.tx_len = 2;
	i3c_msg.data[0] = RG3MXXB12_SLAVE_PORT_ENABLE;
	i3c_msg.data[1] = setting;
	if (!i3c_controller_write(&i3c_msg)) {
		LOG_ERR("Failed to set slave port settings");
		ret = false;
		goto out;
	}

	ret = true;
out:
	memset(&i3c_msg.data, 0, 2);
	i3c_msg.tx_len = 2;
	memcpy(&i3c_msg.data, cmd_protect, 2);
	if (!i3c_controller_write(&i3c_msg)) {
		goto out;
	}

	return ret;
}
