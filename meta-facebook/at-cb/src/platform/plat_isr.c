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

LOG_MODULE_REGISTER(plat_isr);

add_sel_info add_sel_work_item[] = {
	{ .is_init = false, .gpio_num = SMB_P0V8_ALERT_N, .sel_msg = { 0 } },
	{ .is_init = false, .gpio_num = SMB_P1V25_ALRT_N_R, .sel_msg = { 0 } },
	{ .is_init = false, .gpio_num = INA233_ACCL1_ALRT_N_R, .sel_msg = { 0 } },
	{ .is_init = false, .gpio_num = INA233_ACCL2_ALRT_N_R, .sel_msg = { 0 } },
	{ .is_init = false, .gpio_num = INA233_ACCL3_ALRT_N_R, .sel_msg = { 0 } },
	{ .is_init = false, .gpio_num = INA233_ACCL4_ALRT_N_R, .sel_msg = { 0 } },
	{ .is_init = false, .gpio_num = INA233_ACCL5_ALRT_N_R, .sel_msg = { 0 } },
	{ .is_init = false, .gpio_num = INA233_ACCL6_ALRT_N_R, .sel_msg = { 0 } },
	{ .is_init = false, .gpio_num = INA233_ACCL7_ALRT_N_R, .sel_msg = { 0 } },
	{ .is_init = false, .gpio_num = INA233_ACCL8_ALRT_N_R, .sel_msg = { 0 } },
	{ .is_init = false, .gpio_num = INA233_ACCL9_ALRT_N_R, .sel_msg = { 0 } },
	{ .is_init = false, .gpio_num = INA233_ACCL10_ALRT_N_R, .sel_msg = { 0 } },
	{ .is_init = false, .gpio_num = INA233_ACCL11_ALRT_N_R, .sel_msg = { 0 } },
	{ .is_init = false, .gpio_num = INA233_ACCL12_ALRT_N_R, .sel_msg = { 0 } },
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

#define ISR_SENSOR_ALERT(device, gpio_pin_name, event_name, board_id)                              \
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
		work->sel_msg.InF_target = MCTP;                                                   \
		work->sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;                          \
		work->sel_msg.sensor_number = SENSOR_NUM_BIC_SENSOR_ALERT;                         \
		work->sel_msg.event_data1 = OEM_IPMI_EVENT_##event_name##_ALERT;                   \
		work->sel_msg.event_data2 = gpio_get(board_id);                                    \
		work->sel_msg.event_data3 = ADDSEL_EVENT_DATA_DEFAULT;                             \
		if (gpio_get(work->gpio_num) == LOW_ACTIVE) {                                      \
			work->sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;                \
		} else {                                                                           \
			work->sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;                   \
		}                                                                                  \
		k_work_schedule_for_queue(&plat_work_q, &work->add_sel_work, K_NO_WAIT);           \
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

void add_sel_work_handler(struct k_work *work_item)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work_item);
	add_sel_info *work_info = CONTAINER_OF(dwork, add_sel_info, add_sel_work);

	int ret = 0;
	ipmi_msg msg = { 0 };

	pal_construct_ipmi_add_sel_msg(&msg, &work_info->sel_msg);
	ret = pal_pldm_send_ipmi_request(&msg, MCTP_EID_BMC);
	if (ret < 0) {
		LOG_ERR("Failed to addsel to BMC, ret: %d, gpio num: 0x%x", ret,
			work_info->gpio_num);
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
};

ISR_SENSOR_ALERT(VR, SMB_P0V8_ALERT_N, P0V8_VR, BOARD_ID0)
ISR_SENSOR_ALERT(P1V25, SMB_P1V25_ALRT_N_R, P1V25_MONITOR, BOARD_ID0)
ISR_SENSOR_ALERT(P12V_ACCL1, INA233_ACCL1_ALRT_N_R, P12V_ACCL1_MONITOR, BOARD_ID0)
ISR_SENSOR_ALERT(P12V_ACCL2, INA233_ACCL2_ALRT_N_R, P12V_ACCL2_MONITOR, BOARD_ID0)
ISR_SENSOR_ALERT(P12V_ACCL3, INA233_ACCL3_ALRT_N_R, P12V_ACCL3_MONITOR, BOARD_ID0)
ISR_SENSOR_ALERT(P12V_ACCL4, INA233_ACCL4_ALRT_N_R, P12V_ACCL4_MONITOR, BOARD_ID0)
ISR_SENSOR_ALERT(P12V_ACCL5, INA233_ACCL5_ALRT_N_R, P12V_ACCL5_MONITOR, BOARD_ID0)
ISR_SENSOR_ALERT(P12V_ACCL6, INA233_ACCL6_ALRT_N_R, P12V_ACCL6_MONITOR, BOARD_ID0)
ISR_SENSOR_ALERT(P12V_ACCL7, INA233_ACCL7_ALRT_N_R, P12V_ACCL7_MONITOR, BOARD_ID0)
ISR_SENSOR_ALERT(P12V_ACCL8, INA233_ACCL8_ALRT_N_R, P12V_ACCL8_MONITOR, BOARD_ID0)
ISR_SENSOR_ALERT(P12V_ACCL9, INA233_ACCL9_ALRT_N_R, P12V_ACCL9_MONITOR, BOARD_ID0)
ISR_SENSOR_ALERT(P12V_ACCL10, INA233_ACCL10_ALRT_N_R, P12V_ACCL10_MONITOR, BOARD_ID0)
ISR_SENSOR_ALERT(P12V_ACCL11, INA233_ACCL11_ALRT_N_R, P12V_ACCL11_MONITOR, BOARD_ID0)
ISR_SENSOR_ALERT(P12V_ACCL12, INA233_ACCL12_ALRT_N_R, P12V_ACCL12_MONITOR, BOARD_ID0)
