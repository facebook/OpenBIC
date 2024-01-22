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

#ifndef PLAT_FUNC_H
#define PLAT_FUNC_H

#include <stdint.h>
#include "ipmi.h"

#define PRESS_FIO_BUTTON_DELAY_MS 4000

enum BUTTON_OPTIONAL {
	OPTIONAL_AC_OFF = 0x01,
};

typedef struct _add_sel_info {
	bool is_init;
	int delay_ms;
	uint8_t gpio_num;
	uint8_t device_type;
	uint8_t board_info;
	uint8_t event_type;
	struct k_work_delayable add_sel_work;
} add_sel_info;

typedef struct _accl_power_fault_info {
	uint8_t check_bit;
	uint8_t power_fault_state;
} accl_power_fault_info;

void ISR_FIO_BUTTON();
void ISR_POWER_STATUS_CHANGE();
void ISR_VR_ALERT();
void ISR_PMBUS_ALERT();
void ISR_P1V25_ALERT();
void ISR_P12V_ACCL1_ALERT();
void ISR_P12V_ACCL2_ALERT();
void ISR_P12V_ACCL3_ALERT();
void ISR_P12V_ACCL4_ALERT();
void ISR_P12V_ACCL5_ALERT();
void ISR_P12V_ACCL6_ALERT();
void ISR_P12V_ACCL7_ALERT();
void ISR_P12V_ACCL8_ALERT();
void ISR_P12V_ACCL9_ALERT();
void ISR_P12V_ACCL10_ALERT();
void ISR_P12V_ACCL11_ALERT();
void ISR_P12V_ACCL12_ALERT();
void fio_power_button_work_handler();
void vr_alert_work_handler();

#endif
