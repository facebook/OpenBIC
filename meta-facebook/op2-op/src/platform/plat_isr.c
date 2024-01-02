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
#include <logging/log.h>
#include "ipmi.h"
#include "pmbus.h"
#include "libipmi.h"
#include "ipmb.h"
#include "libutil.h"
#include "hal_i2c.h"
#include "plat_power_seq.h"
#include "power_status.h"
#include "plat_gpio.h"
#include "plat_isr.h"
#include "plat_i2c.h"
#include "plat_sensor_table.h"
#include "plat_util.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_isr);

K_THREAD_STACK_DEFINE(power_thread, POWER_SEQ_CTRL_STACK_SIZE);
struct k_thread power_thread_handler;
k_tid_t power_tid;

static bool is_e1s_P12V_fault_assert[MAX_E1S_IDX] = { false, false, false, false, false };
static bool is_e1s_P3V3_fault_assert[MAX_E1S_IDX] = { false, false, false, false, false };

#define OPA_E1S_POWER_FAULT_HANDLER(device, power)                                                 \
	void OPA_ISR_E1S_##device##_##power##_POWER_FAULT()                                        \
	{                                                                                          \
		uint8_t log = 0;                                                                   \
		log = gpio_get(OPA_PWRGD_##power##_E1S_##device##_R);                              \
		LOG_DBG("OPA GPIO = %d, OPA = %d", OPA_PWRGD_##power##_E1S_##device##_R, log);     \
		if ((gpio_get(OPA_PWRGD_##power##_E1S_##device##_R) == POWER_OFF) &&               \
		    (gpio_get(OPA_E1S_##device##_##power##_POWER_EN) == POWER_ON)) {               \
			send_system_status_event(IPMI_EVENT_TYPE_SENSOR_SPECIFIC,                  \
						 IPMI_EVENT_OFFSET_SYS_E1S_##power##_FAULT,        \
						 E1S_##device);                                    \
			is_e1s_##power##_fault_assert[E1S_##device] = true;                        \
		} else {                                                                           \
			if ((gpio_get(OPA_E1S_##device##_##power##_POWER_EN) == POWER_ON) &&       \
			    (gpio_get(OPA_PWRGD_##power##_E1S_##device##_R) == POWER_ON)) {        \
				if (is_e1s_##power##_fault_assert[E1S_##device]) {                 \
					send_system_status_event(                                  \
						IPMI_OEM_EVENT_TYPE_DEASSERT,                      \
						IPMI_EVENT_OFFSET_SYS_E1S_##power##_FAULT,         \
						E1S_##device);                                     \
					is_e1s_##power##_fault_assert[E1S_##device] = false;       \
				}                                                                  \
			}                                                                          \
		}                                                                                  \
	}

#define OPB_E1S_POWER_FAULT_HANDLER(device, power)                                                 \
	void OPB_ISR_E1S_##device##_##power##_POWER_FAULT()                                        \
	{                                                                                          \
		uint8_t log = 0;                                                                   \
		log = gpio_get(OPB_PWRGD_##power##_E1S_##device##_R);                              \
		LOG_DBG("OPB GPIO = %d, OPB = %d", OPB_PWRGD_##power##_E1S_##device##_R, log);     \
		if ((gpio_get(OPB_PWRGD_##power##_E1S_##device##_R) == POWER_OFF) &&               \
		    (gpio_get(OPB_##power##_E1S_##device##_EN_R) == POWER_ON)) {                   \
			send_system_status_event(IPMI_EVENT_TYPE_SENSOR_SPECIFIC,                  \
						 IPMI_EVENT_OFFSET_SYS_E1S_##power##_FAULT,        \
						 E1S_##device);                                    \
			is_e1s_##power##_fault_assert[E1S_##device] = true;                        \
		} else {                                                                           \
			if ((gpio_get(OPB_##power##_E1S_##device##_EN_R) == POWER_ON) &&           \
			    (gpio_get(OPB_PWRGD_##power##_E1S_##device##_R) == POWER_ON)) {        \
				if (is_e1s_##power##_fault_assert[E1S_##device]) {                 \
					send_system_status_event(                                  \
						IPMI_OEM_EVENT_TYPE_DEASSERT,                      \
						IPMI_EVENT_OFFSET_SYS_E1S_##power##_FAULT,         \
						E1S_##device);                                     \
					is_e1s_##power##_fault_assert[E1S_##device] = false;       \
				}                                                                  \
			}                                                                          \
		}                                                                                  \
	}

OPA_E1S_POWER_FAULT_HANDLER(0, P12V);
OPA_E1S_POWER_FAULT_HANDLER(1, P12V);
OPA_E1S_POWER_FAULT_HANDLER(2, P12V);
OPA_E1S_POWER_FAULT_HANDLER(0, P3V3);
OPA_E1S_POWER_FAULT_HANDLER(1, P3V3);
OPA_E1S_POWER_FAULT_HANDLER(2, P3V3);

OPB_E1S_POWER_FAULT_HANDLER(0, P12V);
OPB_E1S_POWER_FAULT_HANDLER(1, P12V);
OPB_E1S_POWER_FAULT_HANDLER(2, P12V);
OPB_E1S_POWER_FAULT_HANDLER(3, P12V);
OPB_E1S_POWER_FAULT_HANDLER(4, P12V);
OPB_E1S_POWER_FAULT_HANDLER(0, P3V3);
OPB_E1S_POWER_FAULT_HANDLER(1, P3V3);
OPB_E1S_POWER_FAULT_HANDLER(2, P3V3);
OPB_E1S_POWER_FAULT_HANDLER(3, P3V3);
OPB_E1S_POWER_FAULT_HANDLER(4, P3V3);

void control_power_sequence()
{
	uint8_t board_revision = get_board_revision();

	if (gpio_get(FM_EXP_MAIN_PWR_EN) == POWER_ON) { // op power on
		if (!is_all_sequence_done(POWER_ON)) {
			abort_power_thread();
			init_power_on_thread(BOARD_POWER_ON_STAGE0);
		}
	} else { // op power off
		if (board_revision != EVT_STAGE) {
			abort_power_thread();
			init_power_off_thread();
		}
	}
}

void init_power_on_thread(uint8_t initial_stage)
{
	// Avoid re-create thread by checking thread status and thread id
	if (power_tid != NULL && strcmp(k_thread_state_str(power_tid), "dead") != 0) {
		return;
	}

	int stage = initial_stage + 1;
	power_tid = k_thread_create(&power_thread_handler, power_thread,
				    K_THREAD_STACK_SIZEOF(power_thread), control_power_on_sequence,
				    (void *)stage, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0,
				    K_NO_WAIT);
	k_thread_name_set(&power_thread_handler, "power_on_sequence_thread");
}

void init_power_off_thread()
{
	// Avoid re-create thread by checking thread status and thread id
	if (power_tid != NULL && strcmp(k_thread_state_str(power_tid), "dead") != 0) {
		return;
	}

	power_tid = k_thread_create(&power_thread_handler, power_thread,
				    K_THREAD_STACK_SIZEOF(power_thread), control_power_off_sequence,
				    NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&power_thread_handler, "power_off_sequence_thread");
}

void abort_power_thread()
{
	if (power_tid != NULL && strcmp(k_thread_state_str(power_tid), "dead") != 0) {
		k_thread_abort(power_tid);
	}
}

void ISR_FM_EXP_MAIN_PWR_EN()
{
	set_DC_status(FM_EXP_MAIN_PWR_EN);
	control_power_sequence();
}

void ISR_CPU_PCIE_PERST()
{
	uint8_t card_type = get_card_type();
	uint8_t gpio_num =
		(card_type == CARD_TYPE_OPA) ? OPA_RST_PCIE_EXP_PERST0_N : OPB_RST_CPLD_PERST1_N;
	if (gpio_get(gpio_num) == GPIO_HIGH) {
		abort_power_thread();
		init_power_on_thread(RETIMER_POWER_ON_STAGE1);
	} else {
		abort_cpu_perst_low_thread();
		cpu_perst_low_thread();
	}
}

void ISR_E1S_0_INA233_ALERT()
{
	send_system_status_event(IPMI_EVENT_TYPE_SENSOR_SPECIFIC,
				 IPMI_EVENT_OFFSET_SYS_INA233_ALERT, INA233_ALERT_E1S_0);
}

void ISR_E1S_1_INA233_ALERT()
{
	send_system_status_event(IPMI_EVENT_TYPE_SENSOR_SPECIFIC,
				 IPMI_EVENT_OFFSET_SYS_INA233_ALERT, INA233_ALERT_E1S_1);
}

void ISR_E1S_2_INA233_ALERT()
{
	send_system_status_event(IPMI_EVENT_TYPE_SENSOR_SPECIFIC,
				 IPMI_EVENT_OFFSET_SYS_INA233_ALERT, INA233_ALERT_E1S_2);
}

void ISR_E1S_3_INA233_ALERT()
{
	send_system_status_event(IPMI_EVENT_TYPE_SENSOR_SPECIFIC,
				 IPMI_EVENT_OFFSET_SYS_INA233_ALERT, INA233_ALERT_E1S_3);
}

void ISR_E1S_4_INA233_ALERT()
{
	send_system_status_event(IPMI_EVENT_TYPE_SENSOR_SPECIFIC,
				 IPMI_EVENT_OFFSET_SYS_INA233_ALERT, INA233_ALERT_E1S_4);
}

void ISR_E1S_P12V_EDGE_INA233_ALERT()
{
	send_system_status_event(IPMI_EVENT_TYPE_SENSOR_SPECIFIC,
				 IPMI_EVENT_OFFSET_SYS_INA233_ALERT, INA233_ALERT_E1S_P12V_EDGE);
}

void ISR_E1S_P12V_MAIN_INA233_ALERT()
{
	send_system_status_event(IPMI_EVENT_TYPE_SENSOR_SPECIFIC,
				 IPMI_EVENT_OFFSET_SYS_INA233_ALERT, INA233_ALERT_E1S_P12V_MAIN);
}

void ISR_E1S_0_PRSNT_N()
{
	uint8_t gpio_num;
	uint8_t card_type = get_card_type();

	switch (card_type) {
	case CARD_TYPE_OPA:
		gpio_num = OPA_E1S_0_PRSNT_N;
		break;
	case CARD_TYPE_OPB:
		gpio_num = OPB_E1S_0_PRSNT_N;
		break;
	default:
		return;
	}

	notify_cpld_e1s_present(E1S_0, gpio_get(gpio_num));
	if (gpio_get(gpio_num) == GPIO_LOW) {
		send_system_status_event(IPMI_EVENT_TYPE_SENSOR_SPECIFIC,
					 IPMI_EVENT_OFFSET_STS_E1S_PRESENT, E1S_0);
		if (gpio_get(FM_EXP_MAIN_PWR_EN) == GPIO_HIGH) {
			abort_e1s_power_thread(E1S_0);
			e1s_power_on_thread(E1S_0, E1S_POWER_ON_STAGE0);
		}
	} else {
		send_system_status_event(IPMI_OEM_EVENT_TYPE_DEASSERT,
					 IPMI_EVENT_OFFSET_STS_E1S_PRESENT, E1S_0);

		abort_e1s_power_thread(E1S_0);
		e1s_power_off_thread(E1S_0);
	}
}

void ISR_E1S_1_PRSNT_N()
{
	uint8_t gpio_num;
	uint8_t card_type = get_card_type();

	switch (card_type) {
	case CARD_TYPE_OPA:
		gpio_num = OPA_E1S_1_PRSNT_N;
		break;
	case CARD_TYPE_OPB:
		gpio_num = OPB_E1S_1_PRSNT_N;
		break;
	default:
		return;
	}

	notify_cpld_e1s_present(E1S_1, gpio_get(gpio_num));
	if (gpio_get(gpio_num) == GPIO_LOW) {
		send_system_status_event(IPMI_EVENT_TYPE_SENSOR_SPECIFIC,
					 IPMI_EVENT_OFFSET_STS_E1S_PRESENT, E1S_1);
		if (gpio_get(FM_EXP_MAIN_PWR_EN) == GPIO_HIGH) {
			abort_e1s_power_thread(E1S_1);
			e1s_power_on_thread(E1S_1, E1S_POWER_ON_STAGE0);
		}
	} else {
		send_system_status_event(IPMI_OEM_EVENT_TYPE_DEASSERT,
					 IPMI_EVENT_OFFSET_STS_E1S_PRESENT, E1S_1);

		abort_e1s_power_thread(E1S_1);
		e1s_power_off_thread(E1S_1);
	}
}

void ISR_E1S_2_PRSNT_N()
{
	uint8_t gpio_num;
	uint8_t card_type = get_card_type();

	switch (card_type) {
	case CARD_TYPE_OPA:
		gpio_num = OPA_E1S_2_PRSNT_N;
		break;
	case CARD_TYPE_OPB:
		gpio_num = OPB_E1S_2_PRSNT_N;
		break;
	default:
		return;
	}

	notify_cpld_e1s_present(E1S_2, gpio_get(gpio_num));
	if (gpio_get(gpio_num) == GPIO_LOW) {
		send_system_status_event(IPMI_EVENT_TYPE_SENSOR_SPECIFIC,
					 IPMI_EVENT_OFFSET_STS_E1S_PRESENT, E1S_2);
		if (gpio_get(FM_EXP_MAIN_PWR_EN) == GPIO_HIGH) {
			abort_e1s_power_thread(E1S_2);
			e1s_power_on_thread(E1S_2, E1S_POWER_ON_STAGE0);
		}
	} else {
		send_system_status_event(IPMI_OEM_EVENT_TYPE_DEASSERT,
					 IPMI_EVENT_OFFSET_STS_E1S_PRESENT, E1S_2);

		abort_e1s_power_thread(E1S_2);
		e1s_power_off_thread(E1S_2);
	}
}

void ISR_E1S_3_PRSNT_N()
{
	notify_cpld_e1s_present(E1S_3, gpio_get(OPB_E1S_3_PRSNT_N));
	if (gpio_get(OPB_E1S_3_PRSNT_N) == GPIO_LOW) {
		send_system_status_event(IPMI_EVENT_TYPE_SENSOR_SPECIFIC,
					 IPMI_EVENT_OFFSET_STS_E1S_PRESENT, E1S_3);
		if (gpio_get(FM_EXP_MAIN_PWR_EN) == GPIO_HIGH) {
			abort_e1s_power_thread(E1S_3);
			e1s_power_on_thread(E1S_3, E1S_POWER_ON_STAGE0);
		}
	} else {
		send_system_status_event(IPMI_OEM_EVENT_TYPE_DEASSERT,
					 IPMI_EVENT_OFFSET_STS_E1S_PRESENT, E1S_3);

		abort_e1s_power_thread(E1S_3);
		e1s_power_off_thread(E1S_3);
	}
}

void ISR_E1S_4_PRSNT_N()
{
	notify_cpld_e1s_present(E1S_4, gpio_get(OPB_E1S_4_PRSNT_N));
	if (gpio_get(OPB_E1S_4_PRSNT_N) == GPIO_LOW) {
		send_system_status_event(IPMI_EVENT_TYPE_SENSOR_SPECIFIC,
					 IPMI_EVENT_OFFSET_STS_E1S_PRESENT, E1S_4);
		if (gpio_get(FM_EXP_MAIN_PWR_EN) == GPIO_HIGH) {
			abort_e1s_power_thread(E1S_4);
			e1s_power_on_thread(E1S_4, E1S_POWER_ON_STAGE0);
		}
	} else {
		send_system_status_event(IPMI_OEM_EVENT_TYPE_DEASSERT,
					 IPMI_EVENT_OFFSET_STS_E1S_PRESENT, E1S_4);

		abort_e1s_power_thread(E1S_4);
		e1s_power_off_thread(E1S_4);
	}
}
