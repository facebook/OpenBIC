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

#include "plat_sys.h"

#include <stdio.h>
#include <stdlib.h>

#include "util_sys.h"
#include "ipmi.h"
#include "plat_i2c.h"
#include "plat_gpio.h"
#include "libipmi.h"
#include "plat_sensor_table.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(plat_sys);

void control_slot_12V_power(uint8_t slot_id, uint8_t control_mode)
{
	int ret = -1;
	uint8_t retry = 5;
	uint8_t isolator_num = 0;
	I2C_MSG msg = { 0 };

	msg.bus = CPLD_IO_I2C_BUS;
	msg.target_addr = CPLD_IO_I2C_ADDR;
	msg.tx_len = 2;

	switch (slot_id) {
	case SLOT1_SLOT_BUTTON:
		isolator_num = FM_BIC_SLOT1_ISOLATED_EN_R;
		msg.data[0] = CPLD_IO_REG_OFS_HSC_EN_SLOT1;
		break;
	case SLOT3_SLOT_BUTTON:
		isolator_num = FM_BIC_SLOT3_ISOLATED_EN_R;
		msg.data[0] = CPLD_IO_REG_OFS_HSC_EN_SLOT3;
		break;
	default:
		LOG_ERR("Input slot id is invalid, slot id: %d", slot_id);
		return;
	}

	switch (control_mode) {
	case SLOT_12V_ON:
		msg.data[1] = SLOT_12V_ON;
		break;
	case SLOT_12V_OFF:
		msg.data[1] = SLOT_12V_OFF;
		break;
	default:
		LOG_ERR("Input control mode is invalid, control mode: %d", control_mode);
		return;
	}

	// Disable SMBus isolator before 12V-off
	if (control_mode == SLOT_12V_OFF) {
		gpio_set(isolator_num, CONTROL_OFF);
	}

	ret = i2c_master_write(&msg, retry);
	if (ret < 0) {
		LOG_ERR("I2C write failed, ret: %d", ret);
		return;
	}

	// Enable SMBus isolator after 12V-on
	if (control_mode == SLOT_12V_ON) {
		gpio_set(isolator_num, CONTROL_ON);
	}

	return;
}

void submit_button_event(uint8_t button_id, uint8_t target_slot, uint8_t event_type)
{
	common_addsel_msg_t sel_msg = { 0 };
	sel_msg.sensor_type = IPMI_SENSOR_TYPE_POWER_UNIT;
	sel_msg.event_type = event_type;
	sel_msg.sensor_number = SENSOR_NUM_BUTTON_DETECT;
	sel_msg.event_data2 = 0xFF;
	sel_msg.event_data3 = 0xFF;

	if ((target_slot != SLOT1_BIC) && (target_slot != SLOT3_BIC)) {
		LOG_ERR("Input target slot is invalid, button id: %d, target slot: %d", button_id,
			target_slot);
		return;
	}
	sel_msg.InF_target = target_slot;

	switch (button_id) {
	case SLOT1_SLOT_BUTTON:
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_PRESS_SLOT1_BUTTON;
		break;
	case SLOT3_SLOT_BUTTON:
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_PRESS_SLOT3_BUTTON;
		break;
	case BASEBOARD_SLED_BUTTON:
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_PRESS_SLED_BUTTON;
		break;
	default:
		LOG_ERR("Input button id is invalid, button id: %d", button_id);
		return;
	}

	common_add_sel_evt_record(&sel_msg);
}

int pal_submit_12v_cycle_slot(ipmi_msg *msg)
{
	uint8_t retry = 3, isolator_num;
	I2C_MSG i2c_msg;

	i2c_msg.bus = CPLD_IO_I2C_BUS;
	i2c_msg.target_addr = CPLD_IO_I2C_ADDR;
	i2c_msg.tx_len = 2;

	if (msg->InF_source == SLOT1_BIC) {
		isolator_num = FM_BIC_SLOT1_ISOLATED_EN_R;
		i2c_msg.data[0] = CPLD_IO_REG_OFS_HSC_EN_SLOT1; // offset

	} else if (msg->InF_source == SLOT3_BIC) {
		isolator_num = FM_BIC_SLOT3_ISOLATED_EN_R;
		i2c_msg.data[0] = CPLD_IO_REG_OFS_HSC_EN_SLOT3; // offset
	} else {
		return NOT_SUPPORT_12V_CYCLE_SLOT;
	}

	// Before slot 12V off, disable isolator
	i2c_msg.data[1] = 0x00;
	gpio_set(isolator_num, GPIO_LOW);
	if (i2c_master_write(&i2c_msg, retry) < 0) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return SLOT_OFF_FAILED;
	}

	k_msleep(2000);

	// After slot 12V on, enable isolator
	i2c_msg.data[1] = 0x01;
	if (i2c_master_write(&i2c_msg, retry) < 0) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return SLOT_ON_FAILED;
	}
	gpio_set(isolator_num, GPIO_HIGH);

	msg->completion_code = CC_SUCCESS;
	return SUCCESS_12V_CYCLE_SLOT;
}
