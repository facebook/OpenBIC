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
#include <string.h>
#include <stdlib.h>
#include <ipmi.h>
#include <pmbus.h>
#include <libipmi.h>
#include <ipmb.h>
#include <libutil.h>

#include <hal_i2c.h>
#include <logging/log.h>
#include "plat_power_seq.h"
#include "power_status.h"
#include "plat_gpio.h"
#include "plat_isr.h"
#include "plat_i2c.h"
#include "plat_sensor_table.h"
#include "util_worker.h"

#define POWER_SEQ_CTRL_STACK_SIZE 1000
#define DC_ON_5_SECOND 5
#define I2C_RETRY 5
#define P0V8_P0V9_VR 3
#define PVDDQ_AB_P0V8_VR 4
#define PVDDQ_CD_VR 5
#define NON_ABOVE_ERROR 0b00000001 /*None of the Above error occurred*/
#define MFR_SPECIFIC_ERROR 0b00010000 /*MFR_SPECIFIC error occurred*/
#define ADCUNLOCK_BBEVENT_ERROR 0b10001000 /*ADCUNLOCK and BBEVENT occurred*/

LOG_MODULE_REGISTER(plat_isr);
K_WORK_DEFINE(cxl_power_on_work, control_power_on_sequence);
K_WORK_DEFINE(cxl_power_off_work, control_power_off_sequence);

void check_power_abnormal(uint8_t power_good_gpio_num, uint8_t control_power_gpio_num)
{
	if (gpio_get(power_good_gpio_num) == POWER_OFF) {
		// If the control power pin is high when the power good pin is falling
		// This means that the behavior of turning off is not expected
		if (gpio_get(control_power_gpio_num) == CONTROL_ON) {
			// submit power off work
			k_work_submit_to_queue(&plat_work_q, &cxl_power_off_work);
		}
	}
}

void ISR_MB_DC_STATE()
{
	set_MB_DC_status(FM_POWER_EN);
	if (gpio_get(FM_POWER_EN) == GPIO_HIGH)
		k_work_submit_to_queue(&plat_work_q, &cxl_power_on_work);
	else
		k_work_submit_to_queue(&plat_work_q, &cxl_power_off_work);
}

void ISR_MB_RST()
{
	if (gpio_get(RST_MB_N) == HIGH_ACTIVE) {
		// Enable ASIC reset pin
		gpio_set(ASIC_PERST0_N, HIGH_ACTIVE);
	} else {
		// Disable ASIC reset pin
		gpio_set(ASIC_PERST0_N, HIGH_INACTIVE);
	}
}

void ISR_P0V8_ASICA_POWER_GOOD_LOST()
{
	check_power_abnormal(P0V8_ASICA_PWRGD, FM_P0V8_ASICA_EN);
}

void ISR_P0V8_ASICD_POWER_GOOD_LOST()
{
	check_power_abnormal(P0V8_ASICD_PWRGD, FM_P0V8_ASICD_EN);
}

void ISR_P0V9_ASICA_POWER_GOOD_LOST()
{
	check_power_abnormal(P0V9_ASICA_PWRGD, FM_P0V9_ASICA_EN);
}

void ISR_P1V8_ASIC_POWER_GOOD_LOST()
{
	check_power_abnormal(P1V8_ASIC_PG_R, P1V8_ASIC_EN_R);
}

void ISR_PVPP_AB_POWER_GOOD_LOST()
{
	check_power_abnormal(PVPP_AB_PG_R, PVPP_AB_EN_R);
}

void ISR_PVPP_CD_POWER_GOOD_LOST()
{
	check_power_abnormal(PVPP_CD_PG_R, PVPP_CD_EN_R);
}

void ISR_PVDDQ_AB_POWER_GOOD_LOST()
{
	check_power_abnormal(PWRGD_PVDDQ_AB, FM_PVDDQ_AB_EN);
}

void ISR_PVDDQ_CD_POWER_GOOD_LOST()
{
	check_power_abnormal(PWRGD_PVDDQ_CD, FM_PVDDQ_CD_EN);
}

void ISR_PVTT_AB_POWER_GOOD_LOST()
{
	check_power_abnormal(PVTT_AB_PG_R, PVTT_AB_EN_R);
}

void ISR_PVTT_CD_POWER_GOOD_LOST()
{
	check_power_abnormal(PVTT_CD_PG_R, PVTT_CD_EN_R);
}

static void add_vr_pmalert_sel(uint8_t gpio_num, uint8_t vr_addr, uint8_t vr_num)
{
	I2C_MSG msg = { 0 };

	msg.bus = I2C_BUS10;
	msg.target_addr = vr_addr;

	for (int page = 0; page < 2; page++) {
		msg.tx_len = 4;
		msg.rx_len = 2;

		memset(&msg.data, 0, I2C_BUFF_SIZE);
		msg.data[0] = PMBUS_PAGE_PLUS_READ;
		msg.data[1] = 0x02;
		msg.data[2] = page;
		msg.data[3] = PMBUS_STATUS_WORD;

		if (i2c_master_read(&msg, I2C_RETRY)) {
			LOG_ERR("Failed to read PMBUS_STATUS_WORD.");
			continue;
		}

		if (check_vr_type() == VR_RNS) {
			if (msg.data[1] == NON_ABOVE_ERROR &&
			    msg.data[2] ==
				    MFR_SPECIFIC_ERROR) { /*exceptional case: MFR_SPECIFIC occurred*/
				msg.tx_len = 4;
				msg.rx_len = 1;
				memset(&msg.data, 0, I2C_BUFF_SIZE);
				msg.data[0] = PMBUS_PAGE_PLUS_READ;
				msg.data[1] = 0x02;
				msg.data[2] = page;
				msg.data[3] = PMBUS_STATUS_MFR_SPECIFIC;
				if (i2c_master_read(&msg, I2C_RETRY)) {
					LOG_ERR("Failed to read PMBUS_STATUS_WORD.");
					continue;
				}
				if (msg.data[1] ==
				    ADCUNLOCK_BBEVENT_ERROR) { /*exceptional case: ADCUNLOCK and BBEVENT occurred*/
					msg.tx_len = 4;
					memset(&msg.data, 0, I2C_BUFF_SIZE);
					msg.data[0] = PMBUS_PAGE_PLUS_WRITE;
					msg.data[1] = 0x02;
					msg.data[2] = page;
					msg.data[3] = PMBUS_CLEAR_FAULTS;
					if (i2c_master_write(&msg, I2C_RETRY)) {
						LOG_ERR("Failed to read PMBUS_STATUS_WORD.");
					}
					continue;
				}
			}
		}

		common_addsel_msg_t sel_msg = { 0 };
		if (gpio_get(gpio_num) == GPIO_HIGH) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
		} else {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		}

		sel_msg.InF_target = CL_BIC_IPMB;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_VR;
		sel_msg.sensor_number = SENSOR_NUM_VR_ALERT;
		sel_msg.event_data1 = (vr_num << 1) | page;
		sel_msg.event_data2 = msg.data[1];
		sel_msg.event_data3 = msg.data[2];

		if (!common_add_sel_evt_record(&sel_msg)) {
			LOG_ERR("Failed to add VR PMALERT sel.");
		}
	}
}

void ISR_PASICA_PMALT()
{
	if (get_DC_status() == false) {
		return;
	}

	add_vr_pmalert_sel(SMB_VR_PASICA_ALERT_N, VR_A0V9_ADDR, P0V8_P0V9_VR);
}

void ISR_PVDDQ_AB_PMALT()
{
	if (get_DC_status() == false) {
		return;
	}

	add_vr_pmalert_sel(SMB_VR_PVDDQ_AB_ALERT_N, VR_VDDQAB_ADDR, PVDDQ_AB_P0V8_VR);
}

void ISR_PVDDQ_CD_PMALT()
{
	if (get_DC_status() == false) {
		return;
	}

	add_vr_pmalert_sel(SMB_VR_PVDDQ_CD_ALERT_N, VR_VDDQCD_ADDR, PVDDQ_CD_VR);
}
