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
#include "util_worker.h"
#include "hal_gpio.h"
#include "hal_i2c.h"
#include "plat_gpio.h"
#include "plat_power_seq.h"
#include "plat_isr.h"
#include "plat_i2c.h"

LOG_MODULE_REGISTER(plat_isr);

IOE_CFG ioe_cfg[] = {
	{ ADDR_IOE1, TCA9555_CONFIG_REG_0, 0x18, TCA9555_OUTPUT_PORT_REG_0, 0x18 },
	{ ADDR_IOE1, TCA9555_CONFIG_REG_1, 0xC0, TCA9555_OUTPUT_PORT_REG_1, 0xFE },
	{ ADDR_IOE2, TCA9555_CONFIG_REG_0, 0xF0, TCA9555_OUTPUT_PORT_REG_0, 0xF0 },
	{ ADDR_IOE2, TCA9555_CONFIG_REG_1, 0xC0, TCA9555_OUTPUT_PORT_REG_1, 0x78 },
	{ ADDR_IOE3, TCA9555_CONFIG_REG_0, 0xFF, TCA9555_OUTPUT_PORT_REG_0, 0xFF },
	{ ADDR_IOE3, TCA9555_CONFIG_REG_1, 0xF0, TCA9555_OUTPUT_PORT_REG_1, 0xF4 },
	{ ADDR_IOE4, TCA9555_CONFIG_REG_0, 0x12, TCA9555_OUTPUT_PORT_REG_0, 0x3F },
	{ ADDR_IOE4, TCA9555_CONFIG_REG_1, 0x04, TCA9555_OUTPUT_PORT_REG_1, 0xBE },
};

int get_ioe_value(uint8_t ioe_addr, uint8_t ioe_reg, uint8_t *value)
{
	int ret = 0;
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	msg.bus = I2C_BUS6;
	msg.target_addr = ioe_addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = ioe_reg;

	ret = i2c_master_read(&msg, retry);

	if (ret != 0) {
		LOG_ERR("Failed to read value from IOE(0x%X)", ioe_addr);
		return -1;
	}

	*value = msg.data[0];

	return 0;
}

int set_ioe_value(uint8_t ioe_addr, uint8_t ioe_reg, uint8_t value)
{
	int ret = 0;
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	msg.bus = I2C_BUS6;
	msg.target_addr = ioe_addr;
	msg.tx_len = 2;
	msg.rx_len = 1;
	msg.data[0] = ioe_reg;
	msg.data[1] = value;

	ret = i2c_master_write(&msg, retry);

	if (ret != 0) {
		LOG_ERR("Failed to write value(0x%X) into IOE(0x%X)", value, ioe_addr);
		return -1;
	}

	return 0;
}

int set_ioe4_control(int cmd)
{
	int ret = 0;
	uint8_t retry = 5;
	uint8_t io_output_status = 0;
	I2C_MSG msg = { 0 };

	msg.bus = I2C_BUS6;
	msg.target_addr = ADDR_IOE4;
	msg.rx_len = 1;
	msg.tx_len = 1;
	msg.data[0] = TCA9555_OUTPUT_PORT_REG_1;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Unable to read ioexp bus when setting IOE4: %u addr: 0x%02x", msg.bus,
			msg.target_addr);
		return -1;
	}

	io_output_status = msg.data[0];

	memset(&msg, 0, sizeof(I2C_MSG));

	msg.bus = I2C_BUS6;
	msg.target_addr = ADDR_IOE4;
	msg.tx_len = 2;
	msg.data[0] = TCA9555_OUTPUT_PORT_REG_1;

	switch (cmd) {
	case SET_CLK:
		msg.data[1] = ((io_output_status & (~ASIC_CLK_BIT)) & (~E1S_CLK_BIT));
		break;
	case SET_PE_RST:
		msg.data[1] = (io_output_status | E1S_PE_RESET_BIT);
		break;
	default:
		LOG_ERR("Unknown command to set IOE4. cmd: %d", cmd);
		return -1;
	}

	ret = i2c_master_write(&msg, retry);

	if (ret != 0) {
		LOG_ERR("Unable to write ioexp bus when setting IOE4: %u addr: 0x%02x", msg.bus,
			msg.target_addr);
		return -1;
	}

	return 0;
}

int check_e1s_present_status()
{
	int ret = 0;
	uint8_t retry = 5;
	uint8_t e1s_present_status = 0;
	I2C_MSG msg = { 0 };

	memset(&msg, 0, sizeof(I2C_MSG));
	msg.bus = I2C_BUS6;
	msg.target_addr = ADDR_IOE4;
	msg.rx_len = 1;
	msg.tx_len = 1;
	msg.data[0] = TCA9555_INPUT_PORT_REG_1;

	ret = i2c_master_read(&msg, retry);

	if (ret != 0) {
		LOG_ERR("Unable to read ioexp bus when checking E1S present: %u addr: 0x%02x",
			msg.bus, msg.target_addr);
		return -1;
	}

	e1s_present_status = msg.data[0] & TCA9555_PORT_2;

	if (e1s_present_status != 0) { // e1s is not present when present pin is low.
		LOG_WRN("E1S is not present");
		return -1;
	} else {
		return 0;
	}
}

void set_ioe_init()
{
	int ret = 0;
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };
	msg.bus = I2C_BUS6;
	msg.tx_len = 2;
	msg.rx_len = 1;

	for (int i = 0; i < ARRAY_SIZE(ioe_cfg); i++) {
		msg.target_addr = ioe_cfg[i].addr;
		msg.data[0] = ioe_cfg[i].conf_reg;
		msg.data[1] = ioe_cfg[i].conf_dir;
		ret = i2c_master_write(&msg, retry);
		if (ret != 0) {
			LOG_ERR("Unable to write ioexp bus when initializing IOE, addr: 0x%02x",
					msg.target_addr);
			return;
		}

		if ((ioe_cfg[i].addr == ADDR_IOE4) && (ioe_cfg[i].output_reg == TCA9555_OUTPUT_PORT_REG_1)) {
			msg.tx_len = 1;
			ret = i2c_master_read(&msg, retry);
			if (ret != 0) {
				LOG_ERR("Unable to read ioexp bus when checking E1S present: %u addr: 0x%02x",
					msg.bus, msg.target_addr);
				return;
			}

			msg.tx_len = 2;
			msg.data[1] = (ioe_cfg[i].output_val & 0xCF) | (msg.data[0] & 0x30);
			msg.data[0] = TCA9555_OUTPUT_PORT_REG_1;
			ret = i2c_master_write(&msg, retry);
		} else {
			msg.data[0] = ioe_cfg[i].output_reg;
			msg.data[1] = ioe_cfg[i].output_val;
			ret = i2c_master_write(&msg, retry);
		}

		if (ret != 0) {
			LOG_ERR("Unable to write ioexp bus when initializing IOE, addr: 0x%02x",
					msg.target_addr);
			return;
		}
	}

	return;
}

void check_ioexp_status()
{
	if (check_e1s_present_status() == 0) {
		set_ioe4_control(SET_PE_RST);
	}
}

void set_asic_and_e1s_clk_handler()
{
	set_ioe4_control(SET_CLK);
}

static void CXL_READY_handler()
{
	/* TODO:
	 * In normal states, DIMM and PMIC muxs should be switch to BIC after checking CXL heartbeat is ready. However, WF's heartbeat is not ready yet
	 * ,so we need to make the workaround for switch muxs.
	 */

	uint8_t value = 0x0;

	if (get_ioe_value(ADDR_IOE2, TCA9555_OUTPUT_PORT_REG_0, &value) == 0) {
		value |= IOE_SWITCH_MUX_TO_BIC; // Enable P0~P3 to switch mux to BIC.
		set_ioe_value(ADDR_IOE2, TCA9555_OUTPUT_PORT_REG_0, value);
	}

	return;
}

K_WORK_DEFINE(cxl_power_on_work, execute_power_on_sequence);
K_WORK_DEFINE(cxl_power_off_work, execute_power_off_sequence);

void ISR_MB_DC_STAGUS_CHAGNE()
{
	set_mb_dc_status(FM_POWER_EN_R);

	if (gpio_get(FM_POWER_EN_R) == POWER_ON) {
		//init_power_on_thread(CLK_POWER_ON_STAGE);
		k_work_submit(&cxl_power_on_work);
	} else {
		//init_power_off_thread(DIMM_POWER_OFF_STAGE_1);
		k_work_submit(&cxl_power_off_work);
	}
}

K_WORK_DEFINE(ioe_power_on_work, check_ioexp_status);

void ISR_MB_PCIE_RST()
{
	gpio_set(PERST_ASIC1_N_R, gpio_get(RST_PCIE_MB_EXP_N));
	gpio_set(PERST_ASIC2_N_R, gpio_get(RST_PCIE_MB_EXP_N));

	if (gpio_get(RST_PCIE_MB_EXP_N) == GPIO_HIGH) {
		k_work_submit(&ioe_power_on_work);
	}
}

K_WORK_DEFINE(e1s_pwr_on_work, set_asic_and_e1s_clk_handler);

void ISR_E1S_PWR_ON()
{
	k_work_submit(&e1s_pwr_on_work);
}

K_WORK_DELAYABLE_DEFINE(CXL_READY_thread, CXL_READY_handler);
void ISR_CXL_PG_ON()
{
	if (k_work_cancel_delayable(&CXL_READY_thread) != 0) {
		LOG_ERR("Failed to cancel CXL_READY thread");
	}
	k_work_schedule(&CXL_READY_thread, K_SECONDS(CXL_READY_SECONDS));
}
