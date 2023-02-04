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

#include "libipmi.h"
#include "kcs.h"
#include "power_status.h"
#include "sensor.h"
#include "snoop.h"
#include "plat_gpio.h"
#include "plat_ipmi.h"
#include "plat_sensor_table.h"
#include "oem_1s_handler.h"
#include "hal_gpio.h"
#include "util_sys.h"
#include "pex89000.h"
#include "pldm.h"
#include "plat_mctp.h"
#include "plat_hook.h"
#include "plat_pldm_monitor.h"
#include "plat_led.h"

LOG_MODULE_REGISTER(plat_isr);

void dc_on_init_pex();
void dc_on_send_cmd_to_dev();

K_WORK_DELAYABLE_DEFINE(dc_on_init_pex_work, dc_on_init_pex);
K_WORK_DELAYABLE_DEFINE(dc_on_send_cmd_to_dev_work, dc_on_send_cmd_to_dev);

#define DC_ON_5_SECOND 5
#define DC_ON_PEX_INIT_RETRY 5

void dc_on_init_pex()
{
	static uint8_t retry[PEX_MAX_NUMBER] = { 0 };

	for (int i = 0; i < PEX_MAX_NUMBER; i++) {
		uint8_t sensor_num = pex_sensor_num_table[i];
		sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
		pex89000_init_arg *init_arg = (pex89000_init_arg *)cfg->init_args;

		/* Only need initial when not initial yet */
		if (init_arg && !init_arg->is_init) {
			if (cfg->pre_sensor_read_hook) {
				if (!cfg->pre_sensor_read_hook(cfg->num,
							       cfg->pre_sensor_read_args)) {
					LOG_ERR("sensor 0x%x pre sensor read failed!", cfg->num);
					continue;
				}
			}
			if (pex89000_init(sensor_num) != SENSOR_INIT_SUCCESS) {
				LOG_ERR("PEX%d initial retry, (%d)", init_arg->idx, retry[i]);
				if (retry[i]++ < DC_ON_PEX_INIT_RETRY) {
					k_work_schedule(&dc_on_init_pex_work,
							K_SECONDS(DC_ON_5_SECOND));
				} else {
					LOG_ERR("PEX%d initial failed", init_arg->idx);
				}
			} else {
				LOG_INF("PEX%d initial success", init_arg->idx);
				retry[i] = 0;
			}
			if (cfg->post_sensor_read_hook) {
				if (!cfg->post_sensor_read_hook(sensor_num,
								cfg->post_sensor_read_args, NULL)) {
					LOG_ERR("sensor number 0x%x post_read failed", cfg->num);
				}
			}
		}
	}
}

void dc_on_send_cmd_to_dev()
{
	/**
   * Call function to set endpoint and get parameters for the device, the
   * function description is as defined by zephyr but argument is not used in
   * this function so put NULL here.
   */
	send_cmd_to_dev(NULL);
}

void ISR_DC_ON()
{
	LOG_INF("System is DC %s", is_mb_dc_on() ? "on" : "off");
	/* Check whether DC on to send work to initial PEX */
	if (is_mb_dc_on()) {
		k_work_schedule(&dc_on_send_cmd_to_dev_work, K_SECONDS(DC_ON_5_SECOND));
		k_work_schedule(&dc_on_init_pex_work, K_SECONDS(DC_ON_5_SECOND));
	}
	pwr_led_check();
}

void ISR_NIC_ADC_ALERT()
{
	struct pldm_sensor_event_state_sensor_state event;

	bool is_alert = !gpio_get(NIC_ADC_ALERT_N);

	event.sensor_offset = PLDM_STATE_SET_OFFSET_DEVICE_STATUS;
	event.event_state = is_alert ? PLDM_STATE_SET_OEM_DEVICE_STATUS_ALERT :
				       PLDM_STATE_SET_OEM_DEVICE_STATUS_NORMAL;
	event.previous_event_state = is_alert ? PLDM_STATE_SET_OEM_DEVICE_STATUS_NORMAL :
						PLDM_STATE_SET_OEM_DEVICE_STATUS_ALERT;

	LOG_WRN("NIC ADC is %s", is_alert ? "alert" : "non-alert");

	if (pldm_send_platform_event(PLDM_SENSOR_EVENT, PLDM_EVENT_SENSOR_NIC_0_7,
				     PLDM_STATE_SENSOR_STATE, (uint8_t *)&event,
				     sizeof(struct pldm_sensor_event_state_sensor_state))) {
		LOG_ERR("Send NIC ADC alert event log failed");
	}
	light_fault_led_check();
}

void ISR_SSD_0_7_ADC_ALERT()
{
	ssd_alert_check(0);
}

void ISR_SSD_8_15_ADC_ALERT()
{
	ssd_alert_check(1);
}

void ISR_PEX_ADC_ALERT()
{
	struct pldm_sensor_event_state_sensor_state event;

	bool is_alert = !gpio_get(PEX_ADC_ALERT_N);

	event.sensor_offset = PLDM_STATE_SET_OFFSET_DEVICE_STATUS;
	event.event_state = is_alert ? PLDM_STATE_SET_OEM_DEVICE_STATUS_ALERT :
				       PLDM_STATE_SET_OEM_DEVICE_STATUS_NORMAL;
	event.previous_event_state = is_alert ? PLDM_STATE_SET_OEM_DEVICE_STATUS_NORMAL :
						PLDM_STATE_SET_OEM_DEVICE_STATUS_ALERT;

	LOG_WRN("PEX ADC is %s", is_alert ? "alert" : "non-alert");

	if (pldm_send_platform_event(PLDM_SENSOR_EVENT, PLDM_EVENT_SENSOR_PEX,
				     PLDM_STATE_SENSOR_STATE, (uint8_t *)&event,
				     sizeof(struct pldm_sensor_event_state_sensor_state))) {
		LOG_ERR("Send PEX ADC alert event log failed");
	}
	light_fault_led_check();
}

void ISR_SMB_FPGA_ALERT()
{
	struct pldm_sensor_event_state_sensor_state event;

	bool is_alert = !gpio_get(SMB_FPGA_ALERT_R_N);

	event.sensor_offset = PLDM_STATE_SET_OFFSET_DEVICE_STATUS;
	event.event_state = is_alert ? PLDM_STATE_SET_OEM_DEVICE_STATUS_ALERT :
				       PLDM_STATE_SET_OEM_DEVICE_STATUS_NORMAL;
	event.previous_event_state = is_alert ? PLDM_STATE_SET_OEM_DEVICE_STATUS_NORMAL :
						PLDM_STATE_SET_OEM_DEVICE_STATUS_ALERT;

	LOG_WRN("FPGA SMB is %s", is_alert ? "alert" : "non-alert");

	if (pldm_send_platform_event(PLDM_SENSOR_EVENT, PLDM_EVENT_SENSOR_CPLD,
				     PLDM_STATE_SENSOR_STATE, (uint8_t *)&event,
				     sizeof(struct pldm_sensor_event_state_sensor_state))) {
		LOG_ERR("Send FPGA SMB alert event log failed");
	}
	light_fault_led_check();
}

void ISR_VR_PMBUS_ALERT()
{
	struct pldm_sensor_event_state_sensor_state event;

	bool is_alert = !gpio_get(SMB_ALERT_PMBUS_R_N);

	event.sensor_offset = PLDM_STATE_SET_OFFSET_DEVICE_STATUS;
	event.event_state = is_alert ? PLDM_STATE_SET_OEM_DEVICE_STATUS_ALERT :
				       PLDM_STATE_SET_OEM_DEVICE_STATUS_NORMAL;
	event.previous_event_state = is_alert ? PLDM_STATE_SET_OEM_DEVICE_STATUS_NORMAL :
						PLDM_STATE_SET_OEM_DEVICE_STATUS_ALERT;

	LOG_WRN("VR PMBUS is %s", is_alert ? "alert" : "non-alert");

	if (pldm_send_platform_event(PLDM_SENSOR_EVENT, PLDM_EVENT_SENSOR_VR,
				     PLDM_STATE_SENSOR_STATE, (uint8_t *)&event,
				     sizeof(struct pldm_sensor_event_state_sensor_state))) {
		LOG_ERR("Send VR PMBUS alert event log failed");
	}
	light_fault_led_check();
}

void ISR_HSC_SMB_ALERT()
{
	struct pldm_sensor_event_state_sensor_state event;

	bool is_alert = !gpio_get(SMB_ALERT_HSC_R_N);

	event.sensor_offset = PLDM_STATE_SET_OFFSET_DEVICE_STATUS;
	event.event_state = is_alert ? PLDM_STATE_SET_OEM_DEVICE_STATUS_ALERT :
				       PLDM_STATE_SET_OEM_DEVICE_STATUS_NORMAL;
	event.previous_event_state = is_alert ? PLDM_STATE_SET_OEM_DEVICE_STATUS_NORMAL :
						PLDM_STATE_SET_OEM_DEVICE_STATUS_ALERT;

	LOG_WRN("HSC SMB is %s", is_alert ? "alert" : "non-alert");

	if (pldm_send_platform_event(PLDM_SENSOR_EVENT, PLDM_EVENT_SENSOR_HSC,
				     PLDM_STATE_SENSOR_STATE, (uint8_t *)&event,
				     sizeof(struct pldm_sensor_event_state_sensor_state))) {
		LOG_ERR("Send HSC SMB alert event log failed");
	}
	light_fault_led_check();
}

#define ISR_SSD_PRESENT_HANDLER(idx)                                                               \
	void ISR_SSD##idx##_PRESENT()                                                              \
	{                                                                                          \
		bool is_present = !gpio_get(e1s_prsnt_pin[idx / 4][idx % 4]);                      \
		struct pldm_sensor_event_state_sensor_state event;                                 \
                                                                                                   \
		event.sensor_offset = PLDM_STATE_SET_OFFSET_DEVICE_PRESENCE;                       \
		event.event_state =                                                                \
			is_present ? PLDM_STATE_SET_PRESENT : PLDM_STATE_SET_NOT_PRESENT;          \
		event.previous_event_state =                                                       \
			is_present ? PLDM_STATE_SET_NOT_PRESENT : PLDM_STATE_SET_PRESENT;          \
                                                                                                   \
		if (pldm_send_platform_event(                                                      \
			    PLDM_SENSOR_EVENT, PLDM_EVENT_SENSOR_E1S_##idx,                        \
			    PLDM_STATE_SENSOR_STATE, (uint8_t *)&event,                            \
			    sizeof(struct pldm_sensor_event_state_sensor_state))) {                \
			LOG_ERR("Send SSD%d presence event log failed", idx);                      \
		}                                                                                  \
	}

ISR_SSD_PRESENT_HANDLER(0);
ISR_SSD_PRESENT_HANDLER(1);
ISR_SSD_PRESENT_HANDLER(2);
ISR_SSD_PRESENT_HANDLER(3);
ISR_SSD_PRESENT_HANDLER(4);
ISR_SSD_PRESENT_HANDLER(5);
ISR_SSD_PRESENT_HANDLER(6);
ISR_SSD_PRESENT_HANDLER(7);
ISR_SSD_PRESENT_HANDLER(8);
ISR_SSD_PRESENT_HANDLER(9);
ISR_SSD_PRESENT_HANDLER(10);
ISR_SSD_PRESENT_HANDLER(11);
ISR_SSD_PRESENT_HANDLER(12);
ISR_SSD_PRESENT_HANDLER(13);
ISR_SSD_PRESENT_HANDLER(14);
ISR_SSD_PRESENT_HANDLER(15);

#undef ISR_SSD_PRESENT_HANDLER