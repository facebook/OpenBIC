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

#include "p3h284x.h"
#include "libutil.h"

LOG_MODULE_REGISTER(dev_p3h284x);

#define RETRY 3

static bool p3h284x_register_read_i3c(uint8_t bus, uint8_t offset, uint8_t *value)
{
	CHECK_NULL_ARG_WITH_RETURN(value, false);
	int ret = -1;
	I3C_MSG i3c_msg = { 0 };

	i3c_msg.bus = bus;
	i3c_msg.target_addr = P3H284X_DEFAULT_STATIC_ADDRESS;
	i3c_msg.tx_len = 1;
	i3c_msg.data[0] = offset;
	i3c_msg.rx_len = 1;
	ret = i3c_transfer(&i3c_msg);
	if (ret != 0) {
		LOG_ERR("Failed to read p3h284x register via I3C 0x%x, bus-%d ret = %d", offset,
			bus, ret);
		return false;
	}
	*value = i3c_msg.data[0];

	return true;
}

static bool p3h284x_register_read(uint8_t bus, uint8_t offset, uint8_t *value)
{
	CHECK_NULL_ARG_WITH_RETURN(value, false);
	int ret = -1;
	I2C_MSG msg = { 0 };

	msg.bus = bus;
	msg.target_addr = P3H284X_DEFAULT_STATIC_ADDRESS;
	msg.tx_len = 1;
	msg.data[0] = offset;
	msg.rx_len = 1;
	ret = i2c_master_read(&msg, RETRY);
	if (ret != 0) {
		LOG_ERR("Failed to read p3h284x register 0x%x, bus-%d", offset, bus);
		return false;
	}
	*value = msg.data[0];

	return true;
}

static bool p3h284x_register_write(uint8_t bus, uint8_t offset, uint8_t value)
{
	int ret = -1;
	I2C_MSG msg = { 0 };

	msg.bus = bus;
	msg.target_addr = P3H284X_DEFAULT_STATIC_ADDRESS;
	msg.tx_len = 2;
	msg.data[0] = offset;
	msg.data[1] = value;
	ret = i2c_master_write(&msg, RETRY);
	if (ret != 0) {
		LOG_ERR("Failed to write p3h284x register 0x%x, bus-%d", offset, bus);
		return false;
	}

	msg.tx_len = 1;
	msg.data[0] = offset;
	msg.rx_len = 1;
	ret = i2c_master_read(&msg, RETRY);
	if ((ret != 0) || (msg.data[0] != value)) {
		LOG_ERR("Failed to read p3h284x register 0x%x after setting,"
			"bus-%d",
			offset, bus);
		return false;
	}
	return true;
}

static bool p3h284x_protected_register(uint8_t bus, bool lock)
{
	uint8_t val = lock ? P3H284X_PROTECTION_LOCK : P3H284X_PROTECTION_UNLOCK;
	if (!p3h284x_register_write(bus, P3H284X_PROTECTION_REG, val)) {
		return false;
	}
	return true;
}

bool p3h284x_select_slave_port_connect(uint8_t bus, uint8_t slave_port)
{
	// Connect selected slave port to HUB network
	if (!p3h284x_register_write(bus, P3H284X_TARGET_PORTSHUB_NETWORK_CONNECTION, slave_port)) {
		return false;
	}
	return true;
}

bool p3h284x_get_device_info(uint8_t bus, uint16_t *i3c_hub_type)
{
	uint8_t device_info0, device_info1 = 0;

	if (p3h284x_register_read(bus, P3H284X_DEVICE_INFO0_REG, &device_info0)) {
		p3h284x_register_read(bus, P3H284X_DEVICE_INFO1_REG, &device_info1);
	} else {
		return false;
	}

	*i3c_hub_type = (device_info1 << 8) | device_info0;
	return true;
}

bool p3h284x_get_device_info_i3c(uint8_t bus, uint16_t *i3c_hub_type)
{
	uint8_t device_info0, device_info1 = 0;

	if (p3h284x_register_read_i3c(bus, P3H284X_DEVICE_INFO0_REG, &device_info0)) {
		p3h284x_register_read_i3c(bus, P3H284X_DEVICE_INFO1_REG, &device_info1);
	} else {
		return false;
	}

	*i3c_hub_type = (device_info1 << 8) | device_info0;
	return true;
}

bool p3h284x_i2c_mode_only_init(uint8_t bus, uint8_t slave_port, uint8_t ldo_volt,
				uint8_t pullup_resistor, uint8_t mode)
{
	bool ret = false;
	uint8_t value;
	// Device Registers Protection Code
	// 0x69: Unlocks protected registers
	if (!p3h284x_protected_register(bus, false)) { //0x10 0x69
		return false;
	}

	// VCCIO LDO Setting
	// 1111 1111 : TP0 & TP1/CP0 & CP1 _LDO_VOLTAGE =1.8V
	value = (ldo_volt << CP0_OFFSET) | (ldo_volt << CP1_OFFSET) |
		(ldo_volt << TP_VCCIO0_OFFSET) | (ldo_volt << TP_VCCIO1_OFFSET);
	if (!p3h284x_register_write(bus, P3H284X_VOLT_LDO_SETTING, value)) {
		goto out;
	}

	// Target Port SMBus Agent Enable
	// 0000 0000 : All Target Port IO is OD/PP compatible mode (I3C).
	if (!p3h284x_register_write(bus, P3H284X_TARGET_PORTS_AGENT_ENABLE, 0x00)) {
		goto out;
	}

	// Target Port GPIO Mode Enable
	// 0000 0000 : The GPIO mode on this port is disabled.
	if (!p3h284x_register_write(bus, P3H284X_TARGET_PORTS_GPIO_ENABLE, 0x00)) {
		goto out;
	}

	// Controller Port Configuration
	// 0001 0000 : OD only mode. I2C open drain only operation supported
	if (!p3h284x_register_write(bus, P3H284X_CONTROLLER_PORT_CONFIG, 0x10)) {
		goto out;
	}

	// Hub Network Operation Mode Configuration
	// 0000 0000 : default setting
	if (!p3h284x_register_write(bus, P3H284X_HUB_NETWORK_OPERATION_MODE, 0x00)) {
		goto out;
	}

	// On-die LDO Enable and Pull up Resistor Value Setting
	// 1111 0000 : TARGET_PORTS_VCCIO0&1 _PULLUP = 2K
	if (!p3h284x_register_write(bus, P3H284X_TARGET_PORTS_PULLUP_SETTING, 0xf0)) {
		goto out;
	}

	if (mode == P3H2840_BYPASS_MODE) {
		// Target Port Pull Up Enable
		if (!p3h284x_register_write(bus, P3H284X_TARGET_PORTS_PULLUP_ENABLE, 0xFD)) {
			goto out;
		}

		// Bypass Mode Enable
		if (!p3h284x_register_write(bus, P3H284X_BYPASS_MODE_ENABLE, 0x1D)) {
			goto out;
		}
	} else {
		// Target Port Pull Up Enable
		if (!p3h284x_register_write(bus, P3H284X_TARGET_PORTS_PULLUP_ENABLE, 0xFF)) {
			goto out;
		}
	}

	// Target Port IO Signaling Mode Setting
	if (!p3h284x_register_write(bus, P3H284X_TARGET_PORTS_MODE_SETTING, slave_port)) {
		goto out;
	}

	// Target Port Enable
	if (!p3h284x_register_write(bus, P3H284X_TARGET_PORT_ENABLE, slave_port)) {
		goto out;
	}

	// Target Port Hub Network Connection Enable
	if (!p3h284x_register_write(bus, P3H284X_TARGET_PORTSHUB_NETWORK_CONNECTION, slave_port)) {
		goto out;
	}

	ret = true;
out:
	// Lock protected register
	if (!p3h284x_protected_register(bus, true)) {
		return false;
	}

	return ret;
}

bool p3h284x_i3c_mode_only_init(I3C_MSG *i3c_msg, const uint8_t (*cmd_initial)[2], int cmd_count)
{
	bool ret = false;

	// Device Registers Protection Code
	const uint8_t cmd_unprotect[2] = { P3H284X_PROTECTION_REG, P3H284X_PROTECTION_UNLOCK };
	const uint8_t cmd_protect[2] = { P3H284X_PROTECTION_REG, P3H284X_PROTECTION_LOCK };

	i3c_msg->tx_len = 2;
	memcpy(i3c_msg->data, cmd_unprotect, 2);

	// Unlock protected regsiter
	if (i3c_controller_write(i3c_msg) != 0) {
		goto out;
	}

	for (int cmd = 0; cmd < cmd_count; cmd++) {
		i3c_msg->tx_len = 2;
		memcpy(i3c_msg->data, cmd_initial[cmd], 2);
		if (i3c_controller_write(i3c_msg) != 0) {
			LOG_ERR("Failed to initial i3c mode. offset = 0x%02x, value = 0x%02x",
				cmd_initial[cmd][0], cmd_initial[cmd][1]);
			goto out;
		}
		k_msleep(10);
	}

	ret = true;
out:
	memcpy(i3c_msg->data, cmd_protect, 2);
	if (i3c_controller_write(i3c_msg) != 0) {
		LOG_ERR("Failed to set protect. offset = 0x%02x, value = 0x%02x", cmd_protect[0],
			cmd_protect[1]);
	}

	return ret;
}

bool p3h284x_set_slave_port(uint8_t bus, uint8_t addr, uint8_t setting)
{
	I3C_MSG i3c_msg = { 0 };
	int ret = 0;
	const uint8_t cmd_unprotect[2] = { P3H284X_PROTECTION_REG, P3H284X_PROTECTION_UNLOCK };
	const uint8_t cmd_protect[2] = { P3H284X_PROTECTION_REG, P3H284X_PROTECTION_LOCK };

	i3c_msg.bus = bus;
	i3c_msg.target_addr = addr;
	i3c_msg.tx_len = 2;
	memcpy(&i3c_msg.data, cmd_unprotect, 2);
	if (i3c_controller_write(&i3c_msg) != 0) {
		LOG_ERR("Failed to unprotect slave port register");
		ret = false;
		goto out;
	}

	memset(&i3c_msg.data, 0, 2);
	i3c_msg.tx_len = 2;
	i3c_msg.data[0] = P3H284X_TARGET_PORT_ENABLE;
	i3c_msg.data[1] = setting;
	if (i3c_controller_write(&i3c_msg) != 0) {
		LOG_ERR("Failed to set slave port settings");
		ret = false;
		goto out;
	}

	ret = true;
out:
	memset(&i3c_msg.data, 0, 2);
	i3c_msg.tx_len = 2;
	memcpy(&i3c_msg.data, cmd_protect, 2);
	if (i3c_controller_write(&i3c_msg) != 0) {
		goto out;
	}

	return ret;
}
