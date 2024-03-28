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
#include "ipmi.h"
#include "ipmb.h"
#include "pldm.h"
#include "libipmi.h"
#include "plat_isr.h"
#include "plat_gpio.h"
#include "plat_mctp.h"
#include "plat_ipmi.h"
#include "plat_class.h"
#include "util_worker.h"
#include "plat_sensor_table.h"
#include "plat_dev.h"
#include "plat_pldm_monitor.h"
#include "xdpe15284.h"
#include "q50sn120a1.h"
#include "pmbus.h"

LOG_MODULE_REGISTER(plat_isr);

#define VR_WRITE_RETRY_MAX_COUNT 5
#define ALERT_EVENT_DEFAULT_DELAY_MS 0
#define POWER_BRICK_ALERT_DELAY_MS 400
#define NORMAL_POWER_GOOD_CHECK_DELAY_MS 5000

typedef struct _alert_sensor_info {
	uint8_t sensor_num;
	uint16_t last_status;
} alert_sensor_info;

typedef struct _alert_event_cfg {
	uint8_t event_type;
	uint16_t bit_map;
} alert_event_cfg;

void check_accl_card_pwr_good_work_handler();

alert_sensor_info vr_info[] = {
	{ .sensor_num = SENSOR_NUM_TEMP_P0V8_VDD_1, .last_status = 0 },
	{ .sensor_num = SENSOR_NUM_TEMP_P0V8_VDD_2, .last_status = 0 },
};

alert_sensor_info power_brick_info[] = {
	{ .sensor_num = SENSOR_NUM_TEMP_POWER_BRICK_1, .last_status = 0 },
	{ .sensor_num = SENSOR_NUM_TEMP_POWER_BRICK_2, .last_status = 0 },
};

add_sel_info add_sel_work_item[] = {
	{ .is_init = false,
	  .delay_ms = ALERT_EVENT_DEFAULT_DELAY_MS,
	  .gpio_num = SMB_P0V8_ALERT_N,
	  .device_type = PLDM_ADDSEL_DEVICE_TYPE_DEFAULT,
	  .event_type = PLDM_ADDSEL_EVENT_TYPE_DEFAULT },
	{ .is_init = false,
	  .delay_ms = POWER_BRICK_ALERT_DELAY_MS,
	  .gpio_num = SMB_PMBUS_ALERT_N_R,
	  .device_type = PLDM_ADDSEL_DEVICE_TYPE_POWER_BRICK_0_ALERT,
	  .event_type = PLDM_ADDSEL_EVENT_TYPE_DEFAULT },
	{ .is_init = false,
	  .delay_ms = ALERT_EVENT_DEFAULT_DELAY_MS,
	  .gpio_num = SMB_P1V25_ALRT_N_R,
	  .device_type = PLDM_ADDSEL_DEVICE_TYPE_P1V25_MONITOR_ALERT,
	  .event_type = PLDM_ADDSEL_EVENT_TYPE_DEFAULT },
	{ .is_init = false,
	  .delay_ms = ALERT_EVENT_DEFAULT_DELAY_MS,
	  .gpio_num = INA233_ACCL1_ALRT_N_R,
	  .device_type = PLDM_ADDSEL_DEVICE_TYPE_P12V_ACCL1_MONITOR_ALERT,
	  .event_type = PLDM_ADDSEL_OVER_POWER_EVENT },
	{ .is_init = false,
	  .delay_ms = ALERT_EVENT_DEFAULT_DELAY_MS,
	  .gpio_num = INA233_ACCL2_ALRT_N_R,
	  .device_type = PLDM_ADDSEL_DEVICE_TYPE_P12V_ACCL2_MONITOR_ALERT,
	  .event_type = PLDM_ADDSEL_OVER_POWER_EVENT },
	{ .is_init = false,
	  .delay_ms = ALERT_EVENT_DEFAULT_DELAY_MS,
	  .gpio_num = INA233_ACCL3_ALRT_N_R,
	  .device_type = PLDM_ADDSEL_DEVICE_TYPE_P12V_ACCL3_MONITOR_ALERT,
	  .event_type = PLDM_ADDSEL_OVER_POWER_EVENT },
	{ .is_init = false,
	  .delay_ms = ALERT_EVENT_DEFAULT_DELAY_MS,
	  .gpio_num = INA233_ACCL4_ALRT_N_R,
	  .device_type = PLDM_ADDSEL_DEVICE_TYPE_P12V_ACCL4_MONITOR_ALERT,
	  .event_type = PLDM_ADDSEL_OVER_POWER_EVENT },
	{ .is_init = false,
	  .delay_ms = ALERT_EVENT_DEFAULT_DELAY_MS,
	  .gpio_num = INA233_ACCL5_ALRT_N_R,
	  .device_type = PLDM_ADDSEL_DEVICE_TYPE_P12V_ACCL5_MONITOR_ALERT,
	  .event_type = PLDM_ADDSEL_OVER_POWER_EVENT },
	{ .is_init = false,
	  .delay_ms = ALERT_EVENT_DEFAULT_DELAY_MS,
	  .gpio_num = INA233_ACCL6_ALRT_N_R,
	  .device_type = PLDM_ADDSEL_DEVICE_TYPE_P12V_ACCL6_MONITOR_ALERT,
	  .event_type = PLDM_ADDSEL_OVER_POWER_EVENT },
	{ .is_init = false,
	  .delay_ms = ALERT_EVENT_DEFAULT_DELAY_MS,
	  .gpio_num = INA233_ACCL7_ALRT_N_R,
	  .device_type = PLDM_ADDSEL_DEVICE_TYPE_P12V_ACCL7_MONITOR_ALERT,
	  .event_type = PLDM_ADDSEL_OVER_POWER_EVENT },
	{ .is_init = false,
	  .delay_ms = ALERT_EVENT_DEFAULT_DELAY_MS,
	  .gpio_num = INA233_ACCL8_ALRT_N_R,
	  .device_type = PLDM_ADDSEL_DEVICE_TYPE_P12V_ACCL8_MONITOR_ALERT,
	  .event_type = PLDM_ADDSEL_OVER_POWER_EVENT },
	{ .is_init = false,
	  .delay_ms = ALERT_EVENT_DEFAULT_DELAY_MS,
	  .gpio_num = INA233_ACCL9_ALRT_N_R,
	  .device_type = PLDM_ADDSEL_DEVICE_TYPE_P12V_ACCL9_MONITOR_ALERT,
	  .event_type = PLDM_ADDSEL_OVER_POWER_EVENT },
	{ .is_init = false,
	  .delay_ms = ALERT_EVENT_DEFAULT_DELAY_MS,
	  .gpio_num = INA233_ACCL10_ALRT_N_R,
	  .device_type = PLDM_ADDSEL_DEVICE_TYPE_P12V_ACCL10_MONITOR_ALERT,
	  .event_type = PLDM_ADDSEL_OVER_POWER_EVENT },
	{ .is_init = false,
	  .delay_ms = ALERT_EVENT_DEFAULT_DELAY_MS,
	  .gpio_num = INA233_ACCL11_ALRT_N_R,
	  .device_type = PLDM_ADDSEL_DEVICE_TYPE_P12V_ACCL11_MONITOR_ALERT,
	  .event_type = PLDM_ADDSEL_OVER_POWER_EVENT },
	{ .is_init = false,
	  .delay_ms = ALERT_EVENT_DEFAULT_DELAY_MS,
	  .gpio_num = INA233_ACCL12_ALRT_N_R,
	  .device_type = PLDM_ADDSEL_DEVICE_TYPE_P12V_ACCL12_MONITOR_ALERT,
	  .event_type = PLDM_ADDSEL_OVER_POWER_EVENT },
};

add_sel_info *get_addsel_work(uint8_t gpio_num)
{
	uint8_t index = 0;

	for (index = 0; index < ARRAY_SIZE(add_sel_work_item); ++index) {
		if (gpio_num == add_sel_work_item[index].gpio_num) {
			return &add_sel_work_item[index];
		}
	}

	return NULL;
}

#define ISR_SENSOR_ALERT(device, gpio_pin_name, board_id)                                          \
	void ISR_##device##_ALERT()                                                                \
	{                                                                                          \
		add_sel_info *work = get_addsel_work(gpio_pin_name);                               \
		if (work == NULL) {                                                                \
			LOG_ERR("Fail to find addsel work, gpio num: %d", gpio_pin_name);          \
			return;                                                                    \
		}                                                                                  \
		if (work->is_init != true) {                                                       \
			k_work_init_delayable(&(work->add_sel_work), add_sel_work_handler);        \
			work->is_init = true;                                                      \
		}                                                                                  \
		work->board_info = gpio_get(board_id);                                             \
		if (gpio_get(work->gpio_num) == LOW_ACTIVE) {                                      \
			work->event_type |= PLDM_ADDSEL_ASSERT_MASK;                               \
		} else {                                                                           \
			work->event_type = work->event_type & PLDM_ADDSEL_DEASSERT_MASK;           \
		}                                                                                  \
		if (work->delay_ms != ALERT_EVENT_DEFAULT_DELAY_MS) {                              \
			k_work_schedule_for_queue(&plat_work_q, &work->add_sel_work,               \
						  K_MSEC(work->delay_ms));                         \
		} else {                                                                           \
			k_work_schedule_for_queue(&plat_work_q, &work->add_sel_work, K_NO_WAIT);   \
		}                                                                                  \
	}

K_WORK_DELAYABLE_DEFINE(fio_power_button_work, fio_power_button_work_handler);
void fio_power_button_work_handler()
{
	int ret = 0;
	uint8_t gpio_num = FIO_PWRBTN_N_R;
	uint8_t button_status = gpio_get(gpio_num);

	/* Check FIO button press time for power control */
	if (button_status == LOW_ACTIVE) {
		ipmi_msg msg = { 0 };
		msg.InF_source = SELF;
		msg.InF_target = MCTP;
		msg.netfn = NETFN_OEM_1S_REQ;
		msg.cmd = CMD_OEM_1S_SEND_INTERRUPT_TO_BMC;

		msg.data_len = 6;
		msg.data[0] = IANA_ID & 0xFF;
		msg.data[1] = (IANA_ID >> 8) & 0xFF;
		msg.data[2] = (IANA_ID >> 16) & 0xFF;
		msg.data[3] = gpio_num;
		msg.data[4] = button_status;
		msg.data[5] = OPTIONAL_AC_OFF;

		ret = pal_pldm_send_ipmi_request(&msg, MCTP_EID_BMC);
		if (ret < 0) {
			LOG_ERR("Failed to send GPIO interrupt event to BMC, gpio number(%d) ret(%d)",
				gpio_num, ret);
		}
	}
}

void vr_alert_addsel(uint8_t sensor_num, uint8_t device_type, uint8_t board_info,
		     uint8_t event_type, uint8_t status)
{
	uint8_t type = event_type;
	uint8_t index = 0;
	alert_event_cfg vr_event_cfg[] = {
		{ .event_type = PLDM_ADDSEL_OVER_TEMPERATURE_EVENT,
		  .bit_map = XDPE15284_TEMPERATURE_FAULT_BIT },
		{ .event_type = PLDM_ADDSEL_UNDER_VOLTAGE_EVENT,
		  .bit_map = XDPE15284_UNDER_VOLTAGE_FAULT_BIT },
		{ .event_type = PLDM_ADDSEL_OVER_CURRENT_EVENT,
		  .bit_map = XDPE15284_OVER_CURRENT_FAULT_BIT },
		{ .event_type = PLDM_ADDSEL_OVER_VOLTAGE_EVENT,
		  .bit_map = XDPE15284_OVER_VOLTAGE_FAULT_BIT },
	};

	for (index = 0; index < ARRAY_SIZE(vr_event_cfg); ++index) {
		if (status & vr_event_cfg[index].bit_map) {
			type = event_type | vr_event_cfg[index].event_type;
			if (plat_set_effecter_states_req(device_type, board_info, type) !=
			    PLDM_SUCCESS) {
				LOG_ERR("Fail to addsel VR: 0x%x event, event type: 0x%x",
					sensor_num, vr_event_cfg[index].event_type);
			}
		}
	}
}

void parse_vr_alert_event(add_sel_info *work_info)
{
	CHECK_NULL_ARG(work_info);

	bool ret = 0;
	uint8_t retry = 0;
	uint8_t index = 0;
	uint8_t status = 0;
	uint8_t sensor_num = 0;

	for (index = 0; index < ARRAY_SIZE(vr_info); ++index) {
		sensor_num = vr_info[index].sensor_num;
		work_info->device_type = ((sensor_num == SENSOR_NUM_TEMP_P0V8_VDD_1) ?
						  PLDM_ADDSEL_DEVICE_TYPE_P0V8_VDD1_ALERT :
						  PLDM_ADDSEL_DEVICE_TYPE_P0V8_VDD2_ALERT);

		sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
		if (cfg->pre_sensor_read_hook) {
			// Add retry to set VR page to avoid it busy to unable response
			for (retry = 0; retry < VR_WRITE_RETRY_MAX_COUNT; ++retry) {
				if (cfg->pre_sensor_read_hook(cfg, cfg->pre_sensor_read_args) !=
				    true) {
					LOG_ERR("VR sensor: 0x%x pre-read fail", sensor_num);
					continue;
				} else {
					break;
				}
			}
		}

		ret = xdpe15284_get_status_byte(cfg->port, cfg->target_addr, &status);
		if (cfg->post_sensor_read_hook) {
			if (cfg->post_sensor_read_hook(cfg, cfg->post_sensor_read_args, NULL) !=
			    true) {
				LOG_ERR("VR sensor: 0x%x post-read fail", sensor_num);
			}
		}

		if (ret != true) {
			LOG_ERR("Get VR sensor: 0x%x status fail", sensor_num);
			continue;
		}

		if (work_info->event_type & PLDM_ADDSEL_ASSERT_MASK) {
			// Alert pin is triggered (Active)
			vr_info[index].last_status = status;
			vr_alert_addsel(sensor_num, work_info->device_type, work_info->board_info,
					PLDM_ADDSEL_ASSERT_MASK, status);
		} else {
			// Alert pin is triggered (INACTIVE)
			// Send deassert event based on the previous assert event
			uint8_t tmp = vr_info[index].last_status;
			vr_info[index].last_status = status;
			status = tmp;
			vr_alert_addsel(sensor_num, work_info->device_type, work_info->board_info,
					0, status);
		}
	}
}

void parse_power_brick_alert_event(add_sel_info *work_info)
{
	CHECK_NULL_ARG(work_info);

	int ret = 0;
	uint8_t type = 0;
	uint8_t index = 0;
	uint8_t sensor_num = 0;
	uint8_t sensor_count = 0;
	uint16_t status = 0;
	alert_event_cfg power_brick_event_cfg[] = {
		{ .event_type = PLDM_ADDSEL_CML_FAULT, .bit_map = PMBUS_CML_FAULT },
		{ .event_type = PLDM_ADDSEL_TEMPERATURE_WARNING_FAULT,
		  .bit_map = PMBUS_TEMPERATURE_WARNING_FAULT },
		{ .event_type = PLDM_ADDSEL_UNDER_VOLTAGE_EVENT,
		  .bit_map = PMBUS_UNDER_VOLTAGE_FAULT },
		{ .event_type = PLDM_ADDSEL_OVER_CURRENT_EVENT,
		  .bit_map = PMBUS_OVER_CURRENT_FAULT },
		{ .event_type = PLDM_ADDSEL_OVER_VOLTAGE_EVENT,
		  .bit_map = PMBUS_OVER_VOLTAGE_FAULT },
		{ .event_type = PLDM_ADDSEL_POWER_OFF_FAULT, .bit_map = PMBUS_POWER_OFF_FAULT },
		{ .event_type = PLDM_ADDSEL_POWER_GOOD_FAULT, .bit_map = PMBUS_POWER_GOOD_FAULT },
		{ .event_type = PLDM_ADDSEL_INPUT_VOLTAGE_FAULT,
		  .bit_map = PMBUS_INPUT_VOLTAGE_FAULT },
		{ .event_type = PLDM_ADDSEL_OUTPUT_CURRENT_WARNING_FAULT,
		  .bit_map = PMBUS_OUTPUT_CURRENT_WARNING_FAULT },
		{ .event_type = PLDM_ADDSEL_OUTPUT_VOLTAGE_WARNING_FAULT,
		  .bit_map = PMBUS_OUTPUT_VOLTAGE_WARNING_FAULT },
	};
	alert_event_cfg bmr351_additional_event_cfg[] = {
		{ .event_type = PLDM_ADDSEL_MFR_SPECIFIC_FAULT,
		  .bit_map = PMBUS_MFR_SPECIFIC_FAULT },
		{ .event_type = PLDM_ADDSEL_NO_LISTED_FAULT, .bit_map = PMBUS_NO_LISTED_FAULT },
	};

	for (sensor_count = 0; sensor_count < ARRAY_SIZE(power_brick_info); ++sensor_count) {
		sensor_num = power_brick_info[sensor_count].sensor_num;
		work_info->device_type = ((sensor_num == SENSOR_NUM_TEMP_POWER_BRICK_1) ?
						  PLDM_ADDSEL_DEVICE_TYPE_POWER_BRICK_0_ALERT :
						  PLDM_ADDSEL_DEVICE_TYPE_POWER_BRICK_1_ALERT);

		sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
		ret = q50sn120a1_get_status_word(cfg->port, cfg->target_addr, &status);
		if (ret != 0) {
			LOG_ERR("Get Power brick sensor: 0x%x status fail", sensor_num);
			continue;
		}

		if (work_info->event_type & PLDM_ADDSEL_ASSERT_MASK) {
			// Alert pin is triggered (Active)
			power_brick_info[sensor_count].last_status = status;
			type = PLDM_ADDSEL_ASSERT_MASK;
		} else {
			// Alert pin is triggered (INACTIVE)
			// Send deassert event based on the previous assert event
			uint16_t tmp = power_brick_info[sensor_count].last_status;
			power_brick_info[sensor_count].last_status = status;
			status = tmp;
			type = 0;
		}

		for (index = 0; index < ARRAY_SIZE(power_brick_event_cfg); ++index) {
			if (status & power_brick_event_cfg[index].bit_map) {
				type |= power_brick_event_cfg[index].event_type;
				if (plat_set_effecter_states_req(work_info->device_type,
								 work_info->board_info,
								 type) != PLDM_SUCCESS) {
					LOG_ERR("Fail to addsel power brick: 0x%x event, event type: 0x%x",
						sensor_num,
						power_brick_event_cfg[index].event_type);
				}
			}
		}

		if (get_pwr_brick_module() == POWER_BRICK_BMR3512202) {
			for (index = 0; index < ARRAY_SIZE(bmr351_additional_event_cfg); ++index) {
				if (status & bmr351_additional_event_cfg[index].bit_map) {
					type |= bmr351_additional_event_cfg[index].event_type;
					if (plat_set_effecter_states_req(work_info->device_type,
									 work_info->board_info,
									 type) != PLDM_SUCCESS) {
						LOG_ERR("Fail to addsel power brick: 0x%x event, event type: 0x%x",
							sensor_num,
							power_brick_event_cfg[index].event_type);
					}
				}
			}
		}
	}
}

void add_sel_work_handler(struct k_work *work_item)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work_item);
	add_sel_info *work_info = CONTAINER_OF(dwork, add_sel_info, add_sel_work);

	uint8_t ret = 0;

	if (work_info->gpio_num == SMB_P0V8_ALERT_N) {
		parse_vr_alert_event(work_info);
		return;
	}

	if (work_info->gpio_num == SMB_PMBUS_ALERT_N_R) {
		parse_power_brick_alert_event(work_info);
		return;
	}

	ret = plat_set_effecter_states_req(work_info->device_type, work_info->board_info,
					   work_info->event_type);
	if (ret != PLDM_SUCCESS) {
		LOG_ERR("Failed to addsel to BMC, ret: %d, gpio num: 0x%x", ret,
			work_info->gpio_num);
	}
}

K_WORK_DELAYABLE_DEFINE(check_accl_card_pwr_good_work, check_accl_card_pwr_good_work_handler);
void check_accl_card_pwr_good_work_handler()
{
	int ret = 0;
	uint8_t val = 0;
	uint8_t index = 0;
	uint8_t card_id = 0;
	accl_power_fault_info power_timeout_info[] = {
		{ .check_bit = CPLD_ACCL_3V3_POWER_TOUT_BIT,
		  .power_fault_state = PLDM_STATE_SET_OEM_DEVICE_3V3_NO_POWER_GOOD },
		{ .check_bit = CPLD_ACCL_12V_POWER_TOUT_BIT,
		  .power_fault_state = PLDM_STATE_SET_OEM_DEVICE_12V_NO_POWER_GOOD },
		{ .check_bit = CPLD_ACCL_3V3_AUX_POWER_TOUT_BIT,
		  .power_fault_state = PLDM_STATE_SET_OEM_DEVICE_3V3_AUX_NO_POWER_GOOD },
	};

	for (card_id = 0; card_id < ARRAY_SIZE(asic_card_info); ++card_id) {
		if (is_accl_cable_power_good_timeout(card_id)) {
			plat_accl_cable_power_good_fail_event(
				card_id, PLDM_STATE_SET_OEM_DEVICE_NO_POWER_GOOD);
			continue;
		}

		ret = get_cpld_register(asic_card_info[card_id].power_fault_reg, &val);
		if (ret != 0) {
			LOG_ERR("Failed to check power fault register, card id: 0x%x, reg: 0x%x",
				card_id, asic_card_info[card_id].power_fault_reg);
			continue;
		}

		for (index = 0; index < ARRAY_SIZE(power_timeout_info); ++index) {
			if (val & power_timeout_info[index].check_bit) {
				plat_accl_power_good_fail_event(
					card_id, power_timeout_info[index].power_fault_state);
			}
		}
	}
}

void ISR_FIO_BUTTON()
{
	k_work_schedule_for_queue(&plat_work_q, &fio_power_button_work,
				  K_MSEC(PRESS_FIO_BUTTON_DELAY_MS));
}

void ISR_POWER_STATUS_CHANGE()
{
	get_acb_power_status();
	if (get_acb_power_good_flag()) {
		init_clk_gen_spread_spectrum_control_register();
		k_work_schedule_for_queue(&plat_work_q, &check_accl_card_pwr_good_work,
					  K_MSEC(NORMAL_POWER_GOOD_CHECK_DELAY_MS));
	} else {
		if (k_work_cancel_delayable(&check_accl_card_pwr_good_work) != 0) {
			LOG_ERR("Cancel check_accl_card_pwr_good_work fail");
		}
	}
};

ISR_SENSOR_ALERT(VR, SMB_P0V8_ALERT_N, BOARD_ID0)
ISR_SENSOR_ALERT(P1V25, SMB_P1V25_ALRT_N_R, BOARD_ID0)
ISR_SENSOR_ALERT(P12V_ACCL1, INA233_ACCL1_ALRT_N_R, BOARD_ID0)
ISR_SENSOR_ALERT(P12V_ACCL2, INA233_ACCL2_ALRT_N_R, BOARD_ID0)
ISR_SENSOR_ALERT(P12V_ACCL3, INA233_ACCL3_ALRT_N_R, BOARD_ID0)
ISR_SENSOR_ALERT(P12V_ACCL4, INA233_ACCL4_ALRT_N_R, BOARD_ID0)
ISR_SENSOR_ALERT(P12V_ACCL5, INA233_ACCL5_ALRT_N_R, BOARD_ID0)
ISR_SENSOR_ALERT(P12V_ACCL6, INA233_ACCL6_ALRT_N_R, BOARD_ID0)
ISR_SENSOR_ALERT(P12V_ACCL7, INA233_ACCL7_ALRT_N_R, BOARD_ID0)
ISR_SENSOR_ALERT(P12V_ACCL8, INA233_ACCL8_ALRT_N_R, BOARD_ID0)
ISR_SENSOR_ALERT(P12V_ACCL9, INA233_ACCL9_ALRT_N_R, BOARD_ID0)
ISR_SENSOR_ALERT(P12V_ACCL10, INA233_ACCL10_ALRT_N_R, BOARD_ID0)
ISR_SENSOR_ALERT(P12V_ACCL11, INA233_ACCL11_ALRT_N_R, BOARD_ID0)
ISR_SENSOR_ALERT(P12V_ACCL12, INA233_ACCL12_ALRT_N_R, BOARD_ID0)
ISR_SENSOR_ALERT(PMBUS, SMB_PMBUS_ALERT_N_R, BOARD_ID0)
