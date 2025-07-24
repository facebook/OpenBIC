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
#include <pmbus.h>
#include "util_worker.h"
#include "hal_gpio.h"
#include "hal_i2c.h"
#include "power_status.h"
#include "plat_gpio.h"
#include "plat_pldm_sensor.h"
#include "plat_power_seq.h"
#include "plat_mctp.h"
#include "plat_isr.h"
#include "plat_class.h"
#include "sensor.h"

LOG_MODULE_REGISTER(plat_isr);

add_vr_sel_info vr_event_work_items[] = {
	{
		.is_init = false,
		.vr_num = PMBUS_VR_IOE1_INT,
		.gpio_num = IOE1_INT_R_N,
	},
	{
		.is_init = false,
		.vr_num = NON_PMBUS_VR_PVTT_AB_ASIC1,
		.gpio_num = PWRGD_PVTT_AB_ASIC1,
		.vr_source = PVTT_AB_ASIC1,
	},
	{
		.is_init = false,
		.vr_num = NON_PMBUS_VR_PVTT_AB_ASIC2,
		.gpio_num = PWRGD_PVTT_AB_ASIC2,
		.vr_source = PVTT_AB_ASIC2,
	},
	{
		.is_init = false,
		.vr_num = NON_PMBUS_VR_PVTT_CD_ASIC1,
		.gpio_num = PWRGD_PVTT_CD_ASIC1,
		.vr_source = PVTT_CD_ASIC1,
	},
	{
		.is_init = false,
		.vr_num = NON_PMBUS_VR_PVTT_CD_ASIC2,
		.gpio_num = PWRGD_PVTT_CD_ASIC2,
		.vr_source = PVTT_CD_ASIC2,
	},
	{
		.is_init = false,
		.vr_num = NON_PMBUS_VR_PVPP_AB_ASIC1,
		.gpio_num = PWRGD_PVPP_AB_ASIC1,
		.vr_source = PVPP_AB_ASIC1,
	},
	{
		.is_init = false,
		.vr_num = NON_PMBUS_VR_PVPP_AB_ASIC2,
		.gpio_num = PWRGD_PVPP_AB_ASIC2,
		.vr_source = PVPP_AB_ASIC2,
	},
	{
		.is_init = false,
		.vr_num = NON_PMBUS_VR_PVPP_CD_ASIC1,
		.gpio_num = PWRGD_PVPP_CD_ASIC1,
		.vr_source = PVPP_CD_ASIC1,
	},
	{
		.is_init = false,
		.vr_num = NON_PMBUS_VR_PVPP_CD_ASIC2,
		.gpio_num = PWRGD_PVPP_CD_ASIC2,
		.vr_source = PVPP_CD_ASIC2,
	},
	{
		.is_init = false,
		.vr_num = NON_PMBUS_VR_P12V_E1S_0_R,
		.gpio_num = PWRGD_P12V_E1S_0_R,
		.vr_source = P12V_E1S_0,
	},
	{
		.is_init = false,
		.vr_num = NON_PMBUS_VR_P3V3_E1S_0_R,
		.gpio_num = PWRGD_P3V3_E1S_0_R,
		.vr_source = P3V3_E1S_0,
	},
};

void init_vr_event_work()
{
	for (int index = 0; index < ARRAY_SIZE(vr_event_work_items); ++index) {
		if (vr_event_work_items[index].is_init != true) {
			if (vr_event_work_items[index].gpio_num == IOE1_INT_R_N) {
				k_work_init_delayable(&vr_event_work_items[index].add_sel_work,
						      process_pmbus_vr_event_handler);
			} else {
				k_work_init_delayable(&vr_event_work_items[index].add_sel_work,
						      process_non_pmbus_vr_event_handler);
			}

			vr_event_work_items[index].is_init = true;
		}
	}
}

const vr_fault_info vr_fault_table[] = {
	// { vr_source, ioe_pin_num, vr_pwrgd_gpio, vr_mux_sel, vr_i2c_bus, vr_addr, vr_page }
	// ALT_SMB_BIC_PMIC1
	{ PVDDQ_AB_ASIC1, IOE_P03, PWRGD_PVDDQ_AB_ASIC1, IOE_SWITCH_CXL1_VR_TO_BIC, I2C_BUS8,
	  ADDR_VR_PVDDQ_AB_ASIC1, 0 },
	{ P0V85_ASIC1, IOE_P03, PWRGD_P0V85_ASIC1, IOE_SWITCH_CXL1_VR_TO_BIC, I2C_BUS8,
	  ADDR_VR_P0V85_ASIC1, 1 },
	{ PVDDQ_CD_ASIC1, IOE_P03, PWRGD_PVDDQ_CD_ASIC1, IOE_SWITCH_CXL1_VR_TO_BIC, I2C_BUS8,
	  ADDR_VR_PVDDQ_CD_ASIC1, 0 },
	{ P0V8_ASIC1, IOE_P03, PWRGD_P0V8_ASIC1, IOE_SWITCH_CXL1_VR_TO_BIC, I2C_BUS8,
	  ADDR_VR_P0V8_ASIC1, 1 },
	// ALT_SMB_BIC_PMIC2
	{ PVDDQ_AB_ASIC2, IOE_P04, PWRGD_PVDDQ_AB_ASIC2, IOE_SWITCH_CXL2_VR_TO_BIC, I2C_BUS3,
	  ADDR_VR_PVDDQ_AB_ASIC2, 0 },
	{ P0V85_ASIC2, IOE_P04, PWRGD_P0V85_ASIC2, IOE_SWITCH_CXL2_VR_TO_BIC, I2C_BUS3,
	  ADDR_VR_P0V85_ASIC2, 1 },
	{ PVDDQ_CD_ASIC2, IOE_P04, PWRGD_PVDDQ_CD_ASIC2, IOE_SWITCH_CXL2_VR_TO_BIC, I2C_BUS3,
	  ADDR_VR_PVDDQ_CD_ASIC2, 0 },
	{ P0V8_ASIC2, IOE_P04, PWRGD_P0V8_ASIC2, IOE_SWITCH_CXL2_VR_TO_BIC, I2C_BUS3,
	  ADDR_VR_P0V8_ASIC2, 1 },
};
const uint8_t vr_reg_list[][8] = {
	// INFINEON
	{ PMBUS_STATUS_WORD, PMBUS_STATUS_BYTE, PMBUS_STATUS_VOUT, PMBUS_STATUS_IOUT,
	  PMBUS_STATUS_INPUT, PMBUS_STATUS_TEMPERATURE, PMBUS_STATUS_CML,
	  PMBUS_STATUS_MFR_SPECIFIC },
	// MPS
	{ PMBUS_STATUS_WORD, PMBUS_STATUS_BYTE, PMBUS_STATUS_VOUT, PMBUS_STATUS_IOUT,
	  PMBUS_STATUS_INPUT, PMBUS_STATUS_TEMPERATURE, PMBUS_STATUS_CML, PMBUS_DRMOS_FAULT }
};

void process_pmbus_vr_event_handler(struct k_work *work_item)
{
	LOG_INF("Entering process pmbus vr event handler...");

	uint8_t retry = 5;
	uint8_t ioe1_reg_data = 0;
	uint8_t ioe2_reg_data = 0;

	get_ioe_value(ADDR_IOE1, TCA9555_INPUT_PORT_REG_0, &ioe1_reg_data);
	get_ioe_value(ADDR_IOE2, TCA9555_OUTPUT_PORT_REG_0, &ioe2_reg_data);

	for (int i = 0; i < ARRAY_SIZE(vr_fault_table); ++i) {
		bool is_vr_alert =
			(GETBIT(ioe1_reg_data, vr_fault_table[i].ioe_pin_num) == GPIO_LOW) ? true :
											     false;
		LOG_INF("Check VR alert, src: %d, ioe1_reg_data: %x, ioe_pin_num: %d, is_vr_alert: %d",
			vr_fault_table[i].vr_source, ioe1_reg_data, vr_fault_table[i].ioe_pin_num,
			is_vr_alert);
		if (is_vr_alert == false && is_cxl_power_on_success == true) {
			continue;
		}

		// check VR SMBUS select pin
		if ((ioe2_reg_data & vr_fault_table[i].vr_mux_sel) == 0) {
			LOG_ERR("The ASIC VR SMBus has not yet been switched to BIC, src: %d",
				vr_fault_table[i].vr_source);
			continue;
		}

		disable_sensor_poll();
		// wait 10ms for vr monitor stop
		k_msleep(10);

		// situation3 power fault, record SEL message
		I2C_MSG msg = { 0 };
		msg.bus = vr_fault_table[i].vr_i2c_bus;
		msg.target_addr = vr_fault_table[i].vr_addr;

		// set page for power rail
		msg.tx_len = 2;
		msg.data[0] = PMBUS_PAGE;
		msg.data[1] = vr_fault_table[i].vr_page;
		if (i2c_master_write(&msg, retry)) {
			LOG_ERR("Set page failed, bus: 0x%x, addr: 0x%x, page: %d", msg.bus,
				msg.target_addr, vr_fault_table[i].vr_page);
			enable_sensor_poll();
			continue;
		}

		bool is_vr_pwrgd =
			(gpio_get(vr_fault_table[i].vr_pwrgd_gpio) == GPIO_HIGH) ? true : false;

		if (is_vr_pwrgd) {
			// VR power rail PWRGD is alive
			// Clear VR fault to reset VR alert, fault is considered as a misinform, system normal
			msg.tx_len = 1;
			msg.data[0] = PMBUS_CLEAR_FAULTS;
			if (i2c_master_write(&msg, retry)) {
				LOG_ERR("Clear fault failed, bus: 0x%x, addr: 0x%x", msg.bus,
					msg.target_addr);
			} else {
				LOG_INF("Clear fault success, bus: 0x%x, addr: 0x%x", msg.bus,
					msg.target_addr);
			}
			enable_sensor_poll();
			continue;
		}

		// VR power rail PWRGD is drop
		uint8_t vr_reg_list_idx = 0;
		uint8_t vr_reg_list_len = 0;
		uint8_t vr_dev;

		plat_pldm_sensor_get_vr_dev(&vr_dev);
		switch (vr_dev) {
		case sensor_dev_xdpe12284c: // main-source: INFINEON
			vr_reg_list_idx = 0;
			vr_reg_list_len = 8;
			break;
		case sensor_dev_mp2971: // 2nd-source: MPS
			vr_reg_list_idx = 1;
			vr_reg_list_len = 8;
			break;
		default: // other-source: INFINEON (Default)
			vr_reg_list_idx = 0;
			vr_reg_list_len = 8;
		}

		// collect status data
		struct pldm_addsel_data sel_msg[vr_reg_list_len];
		memset(sel_msg, 0, sizeof(sel_msg));
		uint8_t sel_msg_idx = 0;
		for (int curr_reg = 0; curr_reg < vr_reg_list_len; curr_reg++) {
			msg.tx_len = 1;
			msg.rx_len = 1;
			msg.data[0] = vr_reg_list[vr_reg_list_idx][curr_reg];
			if (msg.data[0] == PMBUS_STATUS_WORD) {
				msg.rx_len = 2;
			}
			if (((vr_dev == sensor_dev_mp2971)) && (msg.data[0] == PMBUS_DRMOS_FAULT)) {
				// MPS MP2971: The DRMOS_FAULT command on Page 0
				if (vr_fault_table[i].vr_page != PMBUS_PAGE_0) {
					msg.tx_len = 2;
					msg.data[0] = PMBUS_PAGE;
					msg.data[1] = PMBUS_PAGE_0;
					if (i2c_master_write(&msg, retry)) {
						LOG_ERR("Set page failed, bus: 0x%x, addr: 0x%x, page: %d",
							vr_fault_table[i].vr_i2c_bus,
							vr_fault_table[i].vr_addr, PMBUS_PAGE_0);
						continue;
					}
				}
				// provide 2 bytes to return the Intelli-Phase fault on both rails
				msg.rx_len = 2;
			}
			if (i2c_master_read(&msg, retry)) {
				LOG_ERR("Failed to get vr reg, bus: %d, addr: 0x%x, reg: 0x%x",
					msg.bus, msg.target_addr,
					vr_reg_list[vr_reg_list_idx][curr_reg]);
				continue;
			}

			if (((vr_dev == sensor_dev_mp2971)) &&
			    (vr_reg_list[vr_reg_list_idx][curr_reg] == PMBUS_DRMOS_FAULT)) {
				sel_msg[sel_msg_idx].event_type = VR_FAULT;
				sel_msg[sel_msg_idx].assert_type = EVENT_ASSERTED;
				sel_msg[sel_msg_idx].event_data_1 = vr_fault_table[i].vr_source;
				sel_msg[sel_msg_idx].event_data_2 =
					vr_reg_list[vr_reg_list_idx][curr_reg];
				sel_msg[sel_msg_idx].event_data_3 = msg.data[1];

				sel_msg_idx += 1;
			}

			sel_msg[sel_msg_idx].event_type = VR_FAULT;
			sel_msg[sel_msg_idx].assert_type = EVENT_ASSERTED;
			sel_msg[sel_msg_idx].event_data_1 = vr_fault_table[i].vr_source;
			sel_msg[sel_msg_idx].event_data_2 =
				(vr_reg_list[vr_reg_list_idx][curr_reg] == PMBUS_STATUS_WORD) ?
					msg.data[1] :
					vr_reg_list[vr_reg_list_idx][curr_reg];
			sel_msg[sel_msg_idx].event_data_3 = msg.data[0];

			sel_msg_idx += 1;
		}
		enable_sensor_poll();

		// send SEL to BMC
		for (int record = 0; record < sel_msg_idx; record++) {
			if (PLDM_SUCCESS != send_event_log_to_bmc(sel_msg[record])) {
				LOG_ERR("Failed to send VR FAULT assert SEL, event data: 0x%x 0x%x 0x%x",
					sel_msg[record].event_data_1, sel_msg[record].event_data_2,
					sel_msg[record].event_data_3);
			} else {
				LOG_INF("Send VR FAULT assert SEL, event data: 0x%x 0x%x 0x%x",
					sel_msg[record].event_data_1, sel_msg[record].event_data_2,
					sel_msg[record].event_data_3);
			}
		}
	}
}

void process_non_pmbus_vr_event_handler(struct k_work *work_item)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work_item);
	add_vr_sel_info *work_info = CONTAINER_OF(dwork, add_vr_sel_info, add_sel_work);
	LOG_INF("Handle GPIO(%d) interrupt", work_info->gpio_num);

	if (get_DC_status() == POWER_OFF) {
		LOG_INF("GPIO(%d) low due to power off, skip VR fault handling",
			work_info->gpio_num);
		return;
	}

	struct pldm_addsel_data sel_msg = { 0 };
	sel_msg.event_type = VR_FAULT;
	sel_msg.assert_type = EVENT_ASSERTED;
	sel_msg.event_data_1 = work_info->vr_source;

	if (PLDM_SUCCESS != send_event_log_to_bmc(sel_msg)) {
		LOG_ERR("Failed to send VR FAULT assert SEL, event data: 0x%x 0x%x 0x%x",
			sel_msg.event_data_1, sel_msg.event_data_2, sel_msg.event_data_3);
	} else {
		LOG_INF("Send VR FAULT assert SEL, event data: 0x%x 0x%x 0x%x",
			sel_msg.event_data_1, sel_msg.event_data_2, sel_msg.event_data_3);
	}
}

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
		LOG_INF("PERST+");
		// If CXL didn't ready, check again
		if ((!get_cxl_ready_status(CXL_ID_1)) || (!get_cxl_ready_status(CXL_ID_2))) {
			create_check_cxl_ready_thread();
			create_set_dev_endpoint_thread();
		}
	} else {
		LOG_INF("PERST-");
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

	if (gpio_get(PWRGD_P3V3_E1S_0_R) == GPIO_LOW) {
		LOG_ERR("PWRGD_P3V3_E1S_0_R event triggered");
		k_work_schedule_for_queue(
			&plat_work_q, &vr_event_work_items[NON_PMBUS_VR_P3V3_E1S_0_R].add_sel_work,
			K_MSEC(VR_EVENT_DELAY_MS));
	}
}

void ISR_P12V_E1S_PWR_CHANGE()
{
	set_P12V_E1S_power_status(PWRGD_P12V_E1S_0_R);

	if (gpio_get(PWRGD_P12V_E1S_0_R) == GPIO_LOW) {
		LOG_ERR("PWRGD_P12V_E1S_0_R event triggered");
		k_work_schedule_for_queue(
			&plat_work_q, &vr_event_work_items[NON_PMBUS_VR_P12V_E1S_0_R].add_sel_work,
			K_MSEC(VR_EVENT_DELAY_MS));
	}
}

K_WORK_DEFINE(_set_cxl_led, set_cxl_led);

void ISR_PVTT_CD_ASIC1_PWR_CHANGE()
{
	k_work_submit(&_set_cxl_led);

	if (gpio_get(PWRGD_PVTT_CD_ASIC1) == GPIO_LOW) {
		LOG_ERR("PWRGD_PVTT_CD_ASIC1 event triggered");
		k_work_schedule_for_queue(
			&plat_work_q, &vr_event_work_items[NON_PMBUS_VR_PVTT_CD_ASIC1].add_sel_work,
			K_MSEC(VR_EVENT_DELAY_MS));
	}
}

void ISR_PVTT_CD_ASIC2_PWR_CHANGE()
{
	k_work_submit(&_set_cxl_led);

	if (gpio_get(PWRGD_PVTT_CD_ASIC2) == GPIO_LOW) {
		LOG_ERR("PWRGD_PVTT_CD_ASIC2 event triggered");
		k_work_schedule_for_queue(
			&plat_work_q, &vr_event_work_items[NON_PMBUS_VR_PVTT_CD_ASIC2].add_sel_work,
			K_MSEC(VR_EVENT_DELAY_MS));
	}
}

void ISR_IOE1_INT()
{
	uint8_t ioe_reg_value = 0;

	LOG_INF("IOE1_INT_R_N event triggered");

	if (get_ioe_value(ADDR_IOE2, TCA9555_OUTPUT_PORT_REG_0, &ioe_reg_value) == 0) {
		if (ioe_reg_value & IOE_SWITCH_MUX_TO_BIC) {
			k_work_schedule_for_queue(
				&plat_work_q, &vr_event_work_items[PMBUS_VR_IOE1_INT].add_sel_work,
				K_MSEC(VR_EVENT_DELAY_MS));
		} else {
			LOG_ERR("The MUX has not been switched to BIC yet");
			if (get_ioe_value(ADDR_IOE1, TCA9555_INPUT_PORT_REG_0, &ioe_reg_value) ==
			    -1) {
				LOG_ERR("Failed to read IOE1 register");
				return;
			}
			LOG_INF("Clear IOE1_INT register current data: 0x%x", ioe_reg_value);
			return;
		}
	}
}
void ISR_PVTT_AB_ASIC1_VR_FAULT()
{
	LOG_INF("PWRGD_PVTT_AB_ASIC1 event triggered");
	k_work_schedule_for_queue(&plat_work_q,
				  &vr_event_work_items[NON_PMBUS_VR_PVTT_AB_ASIC1].add_sel_work,
				  K_MSEC(VR_EVENT_DELAY_MS));
}

void ISR_PVTT_AB_ASIC2_VR_FAULT()
{
	LOG_INF("PWRGD_PVTT_AB_ASIC2 event triggered");
	k_work_schedule_for_queue(&plat_work_q,
				  &vr_event_work_items[NON_PMBUS_VR_PVTT_AB_ASIC2].add_sel_work,
				  K_MSEC(VR_EVENT_DELAY_MS));
}

void ISR_PVPP_AB_ASIC1_VR_FAULT()
{
	LOG_INF("PWRGD_PVPP_AB_ASIC1 event triggered");
	k_work_schedule_for_queue(&plat_work_q,
				  &vr_event_work_items[NON_PMBUS_VR_PVPP_AB_ASIC1].add_sel_work,
				  K_MSEC(VR_EVENT_DELAY_MS));
}

void ISR_PVPP_AB_ASIC2_VR_FAULT()
{
	LOG_INF("PWRGD_PVPP_AB_ASIC2 event triggered");
	k_work_schedule_for_queue(&plat_work_q,
				  &vr_event_work_items[NON_PMBUS_VR_PVPP_AB_ASIC2].add_sel_work,
				  K_MSEC(VR_EVENT_DELAY_MS));
}

void ISR_PVPP_CD_ASIC1_VR_FAULT()
{
	LOG_INF("PWRGD_PVPP_CD_ASIC1 event triggered");
	k_work_schedule_for_queue(&plat_work_q,
				  &vr_event_work_items[NON_PMBUS_VR_PVPP_CD_ASIC1].add_sel_work,
				  K_MSEC(VR_EVENT_DELAY_MS));
}

void ISR_PVPP_CD_ASIC2_VR_FAULT()
{
	LOG_INF("PWRGD_PVPP_CD_ASIC2 event triggered");
	k_work_schedule_for_queue(&plat_work_q,
				  &vr_event_work_items[NON_PMBUS_VR_PVPP_CD_ASIC2].add_sel_work,
				  K_MSEC(VR_EVENT_DELAY_MS));
}
