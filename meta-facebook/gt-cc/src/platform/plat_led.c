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
#include <zephyr.h>
#include <stdio.h>
#include <logging/log.h>

#include "hal_gpio.h"
#include "sensor.h"
#include "plat_gpio.h"
#include "plat_i2c.h"
#include "plat_hook.h"
#include "plat_led.h"
#include "plat_class.h"
#include "plat_pldm_monitor.h"

LOG_MODULE_REGISTER(plat_led);

static void led_io_expander_init()
{
	I2C_MSG msg = { 0 };

	msg.bus = LED_IO_EXPANDER_BUS;
	msg.target_addr = LED_IO_EXPANDER_ADDR;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = LED_IO_EXPANDER_OUTPUT_PORT1_REG;

	if (i2c_master_read(&msg, 5)) {
		LOG_ERR("Read LED I/O expander output port1 register failed");
		return;
	}

	msg.data[1] = msg.data[0] |=
		(BIT(LED_IO_EXPANDER_FAULT_BIT) | BIT(LED_IO_EXPANDER_PWR_BIT));
	msg.data[0] = LED_IO_EXPANDER_OUTPUT_PORT1_REG;
	msg.tx_len = 2;
	msg.rx_len = 0;

	if (i2c_master_write(&msg, 5)) {
		LOG_ERR("Write LED I/O expander output port1 register failed");
		return;
	}
}

static bool sys_led_control(uint8_t reg, uint8_t offset, bool status)
{
	if ((reg != LED_IO_EXPANDER_CONFIG_PORT0_REG) &&
	    (reg != LED_IO_EXPANDER_CONFIG_PORT1_REG)) {
		LOG_ERR("Unsupport register(0x%x) set", reg);
		return false;
	}

	if (offset & 0xF8) {
		LOG_ERR("Unsupport offset(%d) set", offset);
		return false;
	}

	I2C_MSG msg = { 0 };
	msg.bus = LED_IO_EXPANDER_BUS;
	msg.target_addr = LED_IO_EXPANDER_ADDR;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = reg;

	if (i2c_master_read(&msg, 5)) {
		LOG_ERR("Read register(0x%x) failed", reg);
		return false;
	}

	/* The LED pin is low active */
	WRITE_BIT(msg.data[0], offset, !status);
	msg.data[1] = msg.data[0];
	msg.data[0] = reg;
	msg.tx_len = 2;
	msg.rx_len = 0;

	if (i2c_master_write(&msg, 5)) {
		LOG_ERR("Write register(0x%x) failed", reg);
		return false;
	}

	return true;
}

static uint8_t pwr_led_ctrl_src = 0;
static uint8_t fault_led_ctrl_src = 0;

bool pwr_led_control(uint8_t src, uint8_t status)
{
	if ((src >= LED_CTRL_SRC_MAX) || (status >= LED_CTRL_MAX)) {
		LOG_ERR("Unsupport source(%d) or status(%d)", src, status);
		return false;
	}

	WRITE_BIT(pwr_led_ctrl_src, src, status);

	return sys_led_control(LED_IO_EXPANDER_CONFIG_PORT1_REG, LED_IO_EXPANDER_PWR_BIT,
			       pwr_led_ctrl_src);
}

bool fault_led_control(uint8_t src, uint8_t status)
{
	if ((src >= LED_CTRL_SRC_MAX) || (status >= LED_CTRL_MAX)) {
		LOG_ERR("Unsupport source(%d) or status(%d)", src, status);
		return false;
	}

	WRITE_BIT(fault_led_ctrl_src, src, status);

	return sys_led_control(LED_IO_EXPANDER_CONFIG_PORT1_REG, LED_IO_EXPANDER_FAULT_BIT,
			       fault_led_ctrl_src);
}

void light_fault_led_check()
{
	bool is_alert = 0;
	const uint8_t sys_alert_pin[] = {
		NIC_ADC_ALERT_N, SSD_0_7_ADC_ALERT_N, SSD_8_15_ADC_ALERT_N,
		PEX_ADC_ALERT_N, SMB_ALERT_PMBUS_R_N,
	};

	for (uint8_t i = 0; i < ARRAY_SIZE(sys_alert_pin); i++)
		is_alert |= !gpio_get(sys_alert_pin[i]);

	is_alert |= (get_hsc_type() == HSC_MP5990) ? !gpio_get(SMB_ALERT_HSC_R_N) : false;

	if (!fault_led_control(LED_CTRL_SRC_BIC, is_alert ? LED_CTRL_ON : LED_CTRL_OFF))
		LOG_ERR("Control fault LED failed");
}

void pwr_led_check()
{
	if (!pwr_led_control(LED_CTRL_SRC_BIC, (is_mb_dc_on() ? LED_CTRL_ON : LED_CTRL_OFF)))
		LOG_ERR("Control power LED failed");
}

uint8_t get_sys_led_status(uint8_t type)
{
	uint8_t ctrl_src;
	if (type == SYS_POWER_LED) {
		ctrl_src = pwr_led_ctrl_src;
	} else if (type == SYS_FAULT_LED) {
		ctrl_src = fault_led_ctrl_src;
	} else {
		return LED_STATUS_ERROR;
	}

	return (ctrl_src ? LED_CTRL_ON : LED_CTRL_OFF);
}

static void e1s_led_io_expander_init()
{
	I2C_MSG msg = { 0 };

	msg.bus = E1S_LED_IO_EXPANDER_BUS;
	msg.target_addr = E1S_LED_IO_EXPANDER_ADDR;
	msg.tx_len = 3;
	msg.data[0] = LED_IO_EXPANDER_CONFIG_PORT0_REG;
	msg.data[1] = 0x00;
	msg.data[2] = 0x00;

	if (i2c_master_write(&msg, 5)) {
		LOG_ERR("Write SSD LED I/O expander output register failed");
		return;
	}
}

bool e1s_led_control(uint8_t effector_id, uint8_t status)
{
	if ((effector_id < PLAT_EFFECTER_ID_LED_E1S_0) ||
	    (effector_id > PLAT_EFFECTER_ID_LED_E1S_15)) {
		LOG_ERR("Unsupport E1S LED effector id(%d)", effector_id);
		return false;
	}

	if (status >= LED_CTRL_MAX) {
		LOG_ERR("Unsupport control status(%d)", status);
		return false;
	}

	uint8_t port = (effector_id & BIT_MASK(4)) / 8;
	uint8_t offset = (effector_id & BIT_MASK(4)) % 8;
	uint8_t reg = (port ? LED_IO_EXPANDER_OUTPUT_PORT1_REG : LED_IO_EXPANDER_OUTPUT_PORT0_REG);

	I2C_MSG msg = { 0 };
	msg.bus = E1S_LED_IO_EXPANDER_BUS;
	msg.target_addr = E1S_LED_IO_EXPANDER_ADDR;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = reg;

	if (i2c_master_read(&msg, 5)) {
		LOG_ERR("Read register(0x%x) failed", reg);
		return false;
	}

	/* The LED pin is low active */
	WRITE_BIT(msg.data[0], offset, !status);
	msg.data[1] = msg.data[0];
	msg.data[0] = reg;
	msg.tx_len = 2;
	msg.rx_len = 0;

	if (i2c_master_write(&msg, 5)) {
		LOG_ERR("Write register(0x%x) failed", reg);
		return false;
	}

	return true;
}

uint8_t get_ssd_led_status(uint8_t effector_id)
{
	if ((effector_id < PLAT_EFFECTER_ID_LED_E1S_0) ||
	    (effector_id > PLAT_EFFECTER_ID_LED_E1S_15)) {
		LOG_ERR("Unsupport E1S LED effector id(%d)", effector_id);
		return LED_STATUS_ERROR;
	}

	uint8_t port = (effector_id & BIT_MASK(4)) / 8;
	uint8_t offset = (effector_id & BIT_MASK(4)) % 8;
	uint8_t reg = (port ? LED_IO_EXPANDER_OUTPUT_PORT1_REG : LED_IO_EXPANDER_OUTPUT_PORT0_REG);

	I2C_MSG msg = { 0 };
	msg.bus = E1S_LED_IO_EXPANDER_BUS;
	msg.target_addr = E1S_LED_IO_EXPANDER_ADDR;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = reg;

	if (i2c_master_read(&msg, 5)) {
		LOG_ERR("Read register(0x%x) failed", reg);
		return LED_STATUS_ERROR;
	}

	return (((msg.data[0] >> offset) & 0x01) ? LED_CTRL_OFF : LED_CTRL_ON);
}

void sys_led_init_and_check()
{
	led_io_expander_init();
	e1s_led_io_expander_init();
	pwr_led_check();
	light_fault_led_check();
}
