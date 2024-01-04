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

#include <stdio.h>
#include <stdlib.h>
#include <logging/log.h>
#include "ipmi.h"
#include "ipmb.h"
#include "libipmi.h"
#include "libutil.h"
#include "expansion_board.h"
#include "plat_gpio.h"
#include "plat_sensor_table.h"
#include "plat_power_seq.h"

LOG_MODULE_REGISTER(plat_power_seq);

K_THREAD_STACK_DEFINE(power_thread_stack, POWER_SEQ_HANDLER_STACK_SIZE);
struct k_thread power_thread_handler;

k_tid_t power_tid = NULL;
static bool is_MB_DC_ON = false;

void set_MB_DC_status(uint8_t gpio_num)
{
	is_MB_DC_ON = (gpio_get(gpio_num) == POWER_ON) ? true : false;
	LOG_INF("MB DC status is %s", is_MB_DC_ON ? "ON" : "OFF");
}

// Using same thread to enable/disable power
void control_power_sequence()
{
	// If server board DC on, enable CXL power
	if (gpio_get(POWER_EN_R) == POWER_ON) {
		if (gpio_get(PG_CARD_OK) == POWER_OFF) {
			init_power_thread(POWER_ON);
		}
	} else { // If server board DC off, disable CXL power
		if (gpio_get(PG_CARD_OK) == POWER_ON) {
			init_power_thread(POWER_OFF);
		}
	}
}

void abort_power_thread()
{
	if ((power_tid != NULL) && (strcmp(k_thread_state_str(power_tid), "dead") != 0)) {
		k_thread_abort(power_tid);
	}
}

void init_power_thread(uint8_t power_status)
{
	abort_power_thread();

	if (power_status == POWER_ON) {
		power_tid = k_thread_create(&power_thread_handler, power_thread_stack,
					    K_THREAD_STACK_SIZEOF(power_thread_stack),
					    execute_power_on_sequence, NULL, NULL, NULL,
					    CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
		k_thread_name_set(&power_thread_handler, "power_on_sequence_thread");

	} else {
		power_tid = k_thread_create(&power_thread_handler, power_thread_stack,
					    K_THREAD_STACK_SIZEOF(power_thread_stack),
					    execute_power_off_sequence, NULL, NULL, NULL,
					    CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
		k_thread_name_set(&power_thread_handler, "power_off_sequence_thread");
	}

	return;
}

void execute_power_on_sequence()
{
	if (power_on_handler(ASIC_POWER_ON_STAGE1) == 0) {
		gpio_set(PG_CARD_OK, POWER_ON);
		LOG_INF("Power on success");
	} else {
		LOG_ERR("Power on fail");
	}
}

void execute_power_off_sequence()
{
	if (power_off_handler(DIMM_POWER_OFF_STAGE1) == 0) {
		gpio_set(PG_CARD_OK, POWER_OFF);
		LOG_INF("Power off success");
	} else {
		LOG_ERR("Power off fail");
	}
}

/* Power on sequence
 * ASIC_POWER_ON_STAGE1: P0V75_ASIC, P0V8_ASIC, P0V85_ASIC, SYS_CLK_OSC
 * ASIC_POWER_ON_STAGE2: P1V2_ASIC, P1V8_ASIC
 * DIMM_POWER_ON_STAGE1: PVPP_AB, PVPP_CD
 * DIMM_POWER_ON_STAGE2: PVDDQ_AB, PVDDQ_CD
 * DIMM_POWER_ON_STAGE3: PVTT_AB, PVTT_CD
*/
int power_on_handler(uint8_t control_stage)
{
	int ret = 0, stage = 0;

	// Enable power and check power is ON
	for (stage = control_stage; stage <= DIMM_POWER_ON_STAGE3; stage++) {
		ret = enable_power(stage);
		if (ret < 0) {
			break;
		}

		k_msleep(CHECKK_POWER_DELAY_MS);

		ret = check_power_enable(stage);
		if (ret < 0) {
			break;
		}
	}

	// If power on failed, disable power
	if (ret < 0) {
		switch (stage) {
		case ASIC_POWER_ON_STAGE1:
		case ASIC_POWER_ON_STAGE2:
			power_off_handler(ASIC_POWER_OFF_STAGE1);
			break;
		// Other stages close all power
		default:
			power_off_handler(DIMM_POWER_OFF_STAGE1);
			break;
		}
	}

	return ret;
}

/* Power off sequence
 * DIMM_POWER_OFF_STAGE1: PVTT_AB, PVTT_CD
 * DIMM_POWER_OFF_STAGE2: PVDDQ_AB, PVDDQ_CD
 * DIMM_POWER_OFF_STAGE3: PVPP_AB, PVPP_CD
 * ASIC_POWER_OFF_STAGE1: P1V2_ASIC, P1V8_ASIC
 * ASIC_POWER_OFF_STAGE2: P0V75_ASIC, P0V8_ASIC, P0V85_ASIC
 * BOARD_POWER_OFF_STAGE: SYS_CLK_OSC
*/
int power_off_handler(uint8_t control_stage)
{
	int ret = 0, stage = 0;

	// Disable power and check power is OFF
	for (stage = control_stage; stage <= BOARD_POWER_OFF_STAGE; stage++) {
		ret = disable_power(stage);
		if (ret < 0) {
			break;
		}

		k_msleep(CHECKK_POWER_DELAY_MS);

		ret = check_power_disable(stage);
		if (ret < 0) {
			break;
		}
	}

	return ret;
}

int enable_power(uint8_t control_stage)
{
	int ret = 0;

	switch (control_stage) {
	case ASIC_POWER_ON_STAGE1:
		gpio_set(CLK_100M_OSC_EN, POWER_ON);
		gpio_set(CONTROL_POWER_SEQ_01, POWER_ON);
		gpio_set(CONTROL_POWER_SEQ_02, POWER_ON);
		gpio_set(CONTROL_POWER_SEQ_03, POWER_ON);
		break;
	case ASIC_POWER_ON_STAGE2:
		gpio_set(CONTROL_POWER_SEQ_04, POWER_ON);
		gpio_set(CONTROL_POWER_SEQ_05, POWER_ON);
		break;
	case DIMM_POWER_ON_STAGE1:
		gpio_set(CONTROL_POWER_SEQ_06, POWER_ON);
		gpio_set(CONTROL_POWER_SEQ_07, POWER_ON);
		break;
	case DIMM_POWER_ON_STAGE2:
		gpio_set(CONTROL_POWER_SEQ_08, POWER_ON);
		gpio_set(CONTROL_POWER_SEQ_09, POWER_ON);
		break;
	case DIMM_POWER_ON_STAGE3:
		gpio_set(CONTROL_POWER_SEQ_10, POWER_ON);
		gpio_set(CONTROL_POWER_SEQ_11, POWER_ON);
		break;
	default:
		LOG_ERR("Unknow control power stage 0x%x", control_stage);
		ret = -2;
		break;
	}

	return ret;
}

int check_power_enable(uint8_t control_stage)
{
	int ret = 0;

	switch (control_stage) {
	case ASIC_POWER_ON_STAGE1:
		ret = check_power_status(POWER_ON, CHECK_POWER_SEQ_01);
		if (ret < 0) {
			break;
		}

		ret = check_power_status(POWER_ON, CHECK_POWER_SEQ_02);
		if (ret < 0) {
			break;
		}

		ret = check_power_status(POWER_ON, CHECK_POWER_SEQ_03);
		if (ret < 0) {
			break;
		}

		break;
	case ASIC_POWER_ON_STAGE2:
		ret = check_power_status(POWER_ON, CHECK_POWER_SEQ_04);
		if (ret < 0) {
			break;
		}

		ret = check_power_status(POWER_ON, CHECK_POWER_SEQ_05);
		if (ret < 0) {
			break;
		}
		break;
	case DIMM_POWER_ON_STAGE1:
		ret = check_power_status(POWER_ON, CHECK_POWER_SEQ_06);
		if (ret < 0) {
			break;
		}

		ret = check_power_status(POWER_ON, CHECK_POWER_SEQ_07);
		if (ret < 0) {
			break;
		}
		break;
	case DIMM_POWER_ON_STAGE2:
		ret = check_power_status(POWER_ON, CHECK_POWER_SEQ_08);
		if (ret < 0) {
			break;
		}

		ret = check_power_status(POWER_ON, CHECK_POWER_SEQ_09);
		if (ret < 0) {
			break;
		}
		break;
	case DIMM_POWER_ON_STAGE3:
		ret = check_power_status(POWER_ON, CHECK_POWER_SEQ_10);
		if (ret < 0) {
			break;
		}

		ret = check_power_status(POWER_ON, CHECK_POWER_SEQ_11);
		if (ret < 0) {
			break;
		}
		break;
	default:
		LOG_ERR("Unknow control stage 0x%x", control_stage);
		ret = -2;
		break;
	}

	return ret;
}

int disable_power(uint8_t control_stage)
{
	int ret = 0;

	switch (control_stage) {
	case DIMM_POWER_OFF_STAGE1:
		gpio_set(CONTROL_POWER_SEQ_10, POWER_OFF);
		gpio_set(CONTROL_POWER_SEQ_11, POWER_OFF);
		break;
	case DIMM_POWER_OFF_STAGE2:
		gpio_set(CONTROL_POWER_SEQ_08, POWER_OFF);
		gpio_set(CONTROL_POWER_SEQ_09, POWER_OFF);
		break;
	case DIMM_POWER_OFF_STAGE3:
		gpio_set(CONTROL_POWER_SEQ_06, POWER_OFF);
		gpio_set(CONTROL_POWER_SEQ_07, POWER_OFF);
		break;
	case ASIC_POWER_OFF_STAGE1:
		gpio_set(SYS_RST_N_R, POWER_OFF);
		gpio_set(PWR_ON_RST_N_R, POWER_OFF);
		gpio_set(CONTROL_POWER_SEQ_04, POWER_OFF);
		gpio_set(CONTROL_POWER_SEQ_05, POWER_OFF);
		break;
	case ASIC_POWER_OFF_STAGE2:
		gpio_set(CONTROL_POWER_SEQ_01, POWER_OFF);
		gpio_set(CONTROL_POWER_SEQ_02, POWER_OFF);
		gpio_set(CONTROL_POWER_SEQ_03, POWER_OFF);
		break;
	case BOARD_POWER_OFF_STAGE:
		gpio_set(CLK_100M_OSC_EN, POWER_OFF);
		break;
	default:
		LOG_ERR("Unknow control stage 0x%x", control_stage);
		ret = -1;
		break;
	}

	return ret;
}

int check_power_disable(uint8_t control_stage)
{
	int ret = 0;

	switch (control_stage) {
	case DIMM_POWER_OFF_STAGE1:
		ret = check_power_status(POWER_OFF, CHECK_POWER_SEQ_10);
		if (ret < 0) {
			break;
		}
		ret = check_power_status(POWER_OFF, CHECK_POWER_SEQ_11);
		if (ret < 0) {
			break;
		}
		break;
	case DIMM_POWER_OFF_STAGE2:
		ret = check_power_status(POWER_OFF, CHECK_POWER_SEQ_08);
		if (ret < 0) {
			break;
		}
		ret = check_power_status(POWER_OFF, CHECK_POWER_SEQ_09);
		if (ret < 0) {
			break;
		}
		break;
	case DIMM_POWER_OFF_STAGE3:
		ret = check_power_status(POWER_OFF, CHECK_POWER_SEQ_06);
		if (ret < 0) {
			break;
		}
		ret = check_power_status(POWER_OFF, CHECK_POWER_SEQ_07);
		if (ret < 0) {
			break;
		}
		break;
	case ASIC_POWER_OFF_STAGE1:
		ret = check_power_status(POWER_OFF, CHECK_POWER_SEQ_04);
		if (ret < 0) {
			break;
		}
		ret = check_power_status(POWER_OFF, CHECK_POWER_SEQ_05);
		if (ret < 0) {
			break;
		}
		break;
	case ASIC_POWER_OFF_STAGE2:
		ret = check_power_status(POWER_OFF, CHECK_POWER_SEQ_01);
		if (ret < 0) {
			break;
		}
		ret = check_power_status(POWER_OFF, CHECK_POWER_SEQ_02);
		if (ret < 0) {
			break;
		}
		ret = check_power_status(POWER_OFF, CHECK_POWER_SEQ_03);
		if (ret < 0) {
			break;
		}
		break;
	case BOARD_POWER_OFF_STAGE:
		// No power good pins need to check
		break;
	default:
		LOG_ERR("Unknow control stage 0x%x", control_stage);
		ret = -1;
		break;
	}

	return ret;
}

int check_power_status(uint8_t power_status, uint8_t power_seq)
{
	int ret = 0;

	// Read power good pin to check power status is correct
	if (gpio_get(power_seq) != power_status) {
		LOG_ERR("power pin num %d is %s failed", power_seq,
			(power_status == POWER_ON ? "on" : "off"));
		ret = -1;
	}

	// Add SEL to BMC if power check failed
	if (ret < 0) {
		common_addsel_msg_t *sel_msg =
			(common_addsel_msg_t *)malloc(sizeof(common_addsel_msg_t));
		if (sel_msg == NULL) {
			LOG_ERR("Memory allocation failed!");
			return -1;
		}

		memset(sel_msg, 0, sizeof(common_addsel_msg_t));
		sel_msg->InF_target = GL_BIC_IPMB;
		sel_msg->sensor_type = IPMI_OEM_SENSOR_TYPE_OEM_C3;
		sel_msg->event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		sel_msg->sensor_number = SENSOR_NUM_POWER_ERROR;
		sel_msg->event_data1 =
			((power_status == POWER_ON) ? IPMI_OEM_EVENT_OFFSET_EXP_PWRON_FAIL :
						      IPMI_OEM_EVENT_OFFSET_EXP_PWROFF_FAIL);
		sel_msg->event_data2 = power_seq;
		sel_msg->event_data3 = get_board_id();

		if (!common_add_sel_evt_record(sel_msg)) {
			LOG_ERR("Failed to add power fault event, power status: 0x%x  seq: 0x%x",
				power_status, power_seq);
		}

		SAFE_FREE(sel_msg);
	}

	return ret;
}
