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
#include "power_status.h"
#include "plat_gpio.h"
#include "plat_power_seq.h"
#include "plat_mctp.h"
#include "plat_isr.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_isr);

K_TIMER_DEFINE(set_eid_timer, send_cmd_to_dev, NULL);

void set_e1s_pe_reset()
{
	if (check_ioe4_e1s_prsnt_pin() == 0) {
		uint8_t ioe_reg_value = 0;
		int ret = 0;

		ret = get_ioe_value(ADDR_IOE4, TCA9555_OUTPUT_PORT_REG_1, &ioe_reg_value);

		if (ret != 0) {
			LOG_ERR("Failed to enable E1S PE reset while reading IOE4 register");
		}

		// The value of E1S_PE_RESET needs to be the same as RST_PCIE_MB_EXP_N
		if (gpio_get(RST_PCIE_MB_EXP_N) == GPIO_HIGH) {
			ioe_reg_value = (ioe_reg_value | E1S_PE_RESET_BIT);
		} else {
			ioe_reg_value = (ioe_reg_value & (~E1S_PE_RESET_BIT));
		}

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

static void set_clk_buf_bypass_handler(struct k_work *work)
{
	uint8_t retry = 3;
	I2C_MSG msg = { 0 };
	msg.bus = CLK_BUFFER_BUS;
	msg.target_addr = CLK_BUFFER_ADDR;
	msg.tx_len = 1;
	msg.rx_len = 3;
	msg.data[0] = PLL_OPERATING_OFFSET;
	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("Failed to set clock buffer.");
		return;
	}

	msg.tx_len = 4;
	msg.rx_len = 0;
	msg.data[3] = msg.data[2] | 0x2E;
	msg.data[2] = msg.data[1];
	msg.data[1] = msg.data[0];
	msg.data[0] = PLL_OPERATING_OFFSET;
	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("Failed to set clock buffer.");
	}
}

K_WORK_DEFINE(cxl_power_on_work, execute_power_on_sequence);
K_WORK_DEFINE(cxl_power_off_work, execute_power_off_sequence);
K_WORK_DELAYABLE_DEFINE(set_clk_buf_bypass_work, set_clk_buf_bypass_handler);
void ISR_MB_DC_STAGUS_CHAGNE()
{
	set_mb_dc_status(FM_POWER_EN_R);

	if (gpio_get(FM_POWER_EN_R) == POWER_ON) {
		k_work_submit(&cxl_power_on_work);
		k_work_schedule_for_queue(&plat_work_q, &set_clk_buf_bypass_work,
					  K_MSEC(SET_CLK_BUF_DELAY_MS));
	} else {
		k_work_submit(&cxl_power_off_work);
	}
}

K_WORK_DEFINE(set_e1s_pe_reset_work, set_e1s_pe_reset);

void ISR_MB_PCIE_RST()
{
	gpio_set(PERST_ASIC1_N_R, gpio_get(RST_PCIE_MB_EXP_N));
	gpio_set(PERST_ASIC2_N_R, gpio_get(RST_PCIE_MB_EXP_N));

	k_work_submit(&set_e1s_pe_reset_work);

	// Monitor CXL ready and set CXL EID again
	if (gpio_get(RST_PCIE_MB_EXP_N) == GPIO_HIGH) {
		// If CXL didn't ready, check again
		if ((!get_cxl_ready_status(CXL_ID_1)) || (!get_cxl_ready_status(CXL_ID_2))) {
			create_check_cxl_ready_thread();
			k_timer_start(&set_eid_timer, K_MSEC(25000), K_NO_WAIT);
		}
	} else {
		// host reset, cxl also reset
		set_cxl_ready_status(CXL_ID_1, false);
		set_cxl_ready_status(CXL_ID_2, false);
		set_cxl_vr_access(CXL_ID_1, false);
		set_cxl_vr_access(CXL_ID_2, false);
	}
}

K_WORK_DEFINE(e1s_pwr_on_work, set_asic_and_e1s_clk_handler);

void ISR_P3V3_E1S_PWR_CHANGE()
{
	if (gpio_get(POC_EN_P3V3_E1S_0_R) == GPIO_HIGH || gpio_get(EN_P3V3_E1S_0_R) == GPIO_HIGH) {
		k_work_submit(&e1s_pwr_on_work);
	}
	if (get_board_revision() == BOARD_POC) {
		set_P3V3_E1S_power_status(POC_PWRGD_P3V3_E1S_0_R);
	} else {
		set_P3V3_E1S_power_status(PWRGD_P3V3_E1S_0_R);
	}
}

void ISR_P12V_E1S_PWR_CHANGE()
{
	set_P12V_E1S_power_status(PWRGD_P12V_E1S_0_R);
}

K_WORK_DEFINE(_set_cxl_led, set_cxl_led);

void ISR_SET_CXL_LED()
{
	k_work_submit(&_set_cxl_led);
}
