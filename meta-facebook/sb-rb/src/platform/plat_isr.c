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
#include <stdlib.h>
#include <logging/log.h>

#include "plat_gpio.h"
#include "plat_cpld.h"
#include "plat_log.h"
#include "plat_event.h"
#include "plat_ioexp.h"
#include "plat_hook.h"
#include "plat_iris_smbus.h"
#include "plat_util.h"
#include "plat_i2c.h"
#include "shell_iris_power.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_isr);

K_TIMER_DEFINE(check_ubc_delayed_timer, check_ubc_delayed_timer_handler, NULL);

uint8_t pwr_steps_on_flag = 0;

void set_pwr_steps_on_flag(uint8_t flag_value)
{
	pwr_steps_on_flag = flag_value;
	LOG_DBG("set pwr_steps_on_flag = %d", pwr_steps_on_flag);
	//check value
	if (pwr_steps_on_flag != flag_value)
		LOG_ERR("set pwr_steps_on_flag failed, now pwr_steps_on_flag = %d",
			pwr_steps_on_flag);
}

uint8_t get_pwr_steps_on_flag(void)
{
	return pwr_steps_on_flag;
}

void ISR_GPIO_ALL_VR_PM_ALERT_R_N()
{
	LOG_DBG("gpio_%d_isr called, val=%d , dir= %d", ALL_VR_PM_ALERT_R_N,
		gpio_get(ALL_VR_PM_ALERT_R_N), gpio_get_direction(ALL_VR_PM_ALERT_R_N));

	check_cpld_polling_alert_status();
	if (gpio_get(ALL_VR_PM_ALERT_R_N) == GPIO_LOW) {
		give_all_vr_pm_alert_sem();
	}
}

void ISR_GPIO_FM_PLD_UBC_EN_R()
{
	// check step on setting flag
	if (get_pwr_steps_on_flag() == 1)
		return;

	LOG_DBG("gpio_%d_isr called, val=%d , dir= %d", FM_PLD_UBC_EN_R, gpio_get(FM_PLD_UBC_EN_R),
		gpio_get_direction(FM_PLD_UBC_EN_R));

	LOG_INF("FM_PLD_UBC_EN_R = %d", gpio_get(FM_PLD_UBC_EN_R));

	if (gpio_get(FM_PLD_UBC_EN_R) == GPIO_HIGH) {
		//plat_clock_init();
		plat_set_dc_on_log(LOG_ASSERT);
	}

	if (gpio_get(FM_PLD_UBC_EN_R) == GPIO_LOW) {
		plat_set_dc_on_log(LOG_DEASSERT);
	}

	k_timer_start(&check_ubc_delayed_timer, K_MSEC(1000), K_NO_WAIT);
}

void ISR_GPIO_RST_IRIS_PWR_ON_PLD_R1_N()
{
	// dc on
	if (gpio_get(RST_IRIS_PWR_ON_PLD_R1_N)) {
		ioexp_init();
		set_bootstrap_table_val_to_ioexp();
		if (get_asic_board_id() == ASIC_BOARD_ID_EVB) {
			init_U200052_IO();
			power_on_p3v3_osfp();
		}
	} else {
		LOG_INF("dc off, clear io expander init flag");
		set_ioe_init_flag(0);
	}
}

void ISR_GPIO_SMB_HAMSA_MMC_LVC33_ALERT_N()
{
	uint8_t data[8] = { 0 };
	LOG_INF("smb hamsa mmc lvc33 alert triggered");
	//i2c_burst_read(smb_hamsa_i2c_dev, HAMSA_BOOT1_ADDR, SMBUS_ERROR, data, ERROR_CODE_LEN);

	// bus, uint8_t addr, uint8_t offset, uint8_t *data, uint8_t len
	plat_i2c_read(I2C_BUS12, HAMSA_BOOT1_ADDR, SMBUS_ERROR, data, ERROR_CODE_LEN);
	struct pldm_addsel_data smb_hamsa_sel_msg = { 0 };
	smb_hamsa_sel_msg.assert_type = LOG_ASSERT;
	smb_hamsa_sel_msg.event_type = ASIC_MODULE_ERROR; //ASIC_MODULE_ERROR;
	smb_hamsa_sel_msg.event_data_1 = HAMSA_SMB_ERR_EVENT_HEADER;
	smb_hamsa_sel_msg.event_data_2 = data[1]; // 123
	smb_hamsa_sel_msg.event_data_3 = data[2]; // 124
	if (send_event_log_to_bmc(smb_hamsa_sel_msg) != PLDM_SUCCESS) {
		LOG_ERR("Failed to send hamsa smb error code to bmc, event data: 0x%x 0x%x 0x%x\n",
			smb_hamsa_sel_msg.event_data_1, smb_hamsa_sel_msg.event_data_2,
			smb_hamsa_sel_msg.event_data_3);
	}
	// print all data received
	for (int i = 0; i < ERROR_CODE_LEN; i++) {
		LOG_DBG("data[%d] = 0x%x", i, data[i]);
	}
}

bool plat_gpio_immediate_int_cb(uint8_t gpio_num)
{
	bool ret = false;

	switch (gpio_num) {
	case ALL_VR_PM_ALERT_R_N:
		ret = true;
		break;
	default:
		break;
	}

	return ret;
}