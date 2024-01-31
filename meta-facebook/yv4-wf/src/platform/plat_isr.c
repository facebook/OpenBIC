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

LOG_MODULE_REGISTER(plat_isr);

void enable_e1s_pe_reset()
{
	if (check_ioe4_e1s_prsnt_pin() == 0) {
		uint8_t ioe_reg_value = 0;
		int ret = 0;

		ret = get_ioe_value(ADDR_IOE4, TCA9555_OUTPUT_PORT_REG_1, &ioe_reg_value);

		if (ret != 0) {
			LOG_ERR("Failed to enable E1S PE reset while reading IOE4 register");
		}

		ioe_reg_value = (ioe_reg_value | E1S_PE_RESET_BIT);

		ret = set_ioe_value(ADDR_IOE4, TCA9555_OUTPUT_PORT_REG_1, ioe_reg_value);

		if (ret != 0) {
			LOG_ERR("Failed to enable E1S PE reset while writing IOE4 register");
		}
	}
}

void set_asic_and_e1s_clk_handler()
{
	uint8_t ioe_reg_value = 0;
	int ret = 0;

	ret = get_ioe_value(ADDR_IOE4, TCA9555_OUTPUT_PORT_REG_1, &ioe_reg_value);

	if (ret != 0) {
		LOG_ERR("Failed to get ASIC and E1S clock while reading IOE4 register");
	}

	ioe_reg_value = ((ioe_reg_value & (~ASIC_CLK_BIT)) & (~E1S_CLK_BIT));

	ret = set_ioe_value(ADDR_IOE4, TCA9555_OUTPUT_PORT_REG_1, ioe_reg_value);

	if (ret != 0) {
		LOG_ERR("Failed to set ASIC and E1S clock while writing IOE4 register");
	}
}

void set_cxl_led()
{
	uint8_t ioe_reg_value = 0;

	get_ioe_value(ADDR_IOE3, TCA9555_OUTPUT_PORT_REG_1, &ioe_reg_value);

	if ((gpio_get(PWRGD_PVTT_CD_ASIC1) == HIGH_ACTIVE) &&
	    (gpio_get(PWRGD_PVTT_CD_ASIC2) == HIGH_ACTIVE)) {
		ioe_reg_value = (ioe_reg_value | CXL_LED_BIT);
	} else { // If any of the ASIC is not powered up, turn off the CXL LED
		ioe_reg_value = (ioe_reg_value & (~CXL_LED_BIT));
	}

	set_ioe_value(ADDR_IOE3, TCA9555_OUTPUT_PORT_REG_1, ioe_reg_value);
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

K_WORK_DEFINE(_enable_e1s_pe_reset, enable_e1s_pe_reset);

void ISR_MB_PCIE_RST()
{
	gpio_set(PERST_ASIC1_N_R, gpio_get(RST_PCIE_MB_EXP_N));
	gpio_set(PERST_ASIC2_N_R, gpio_get(RST_PCIE_MB_EXP_N));

	if (gpio_get(RST_PCIE_MB_EXP_N) == GPIO_HIGH) {
		k_work_submit(&_enable_e1s_pe_reset);
	}
}

K_WORK_DEFINE(e1s_pwr_on_work, set_asic_and_e1s_clk_handler);

void ISR_E1S_PWR_ON()
{
	k_work_submit(&e1s_pwr_on_work);
}

K_WORK_DEFINE(_set_cxl_led, set_cxl_led);

void ISR_SET_CXL_LED()
{
	k_work_submit(&_set_cxl_led);
}
