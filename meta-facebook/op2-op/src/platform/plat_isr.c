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

LOG_MODULE_REGISTER(plat_isr);

K_THREAD_STACK_DEFINE(power_thread, POWER_SEQ_CTRL_STACK_SIZE);
struct k_thread power_thread_handler;
k_tid_t power_tid;

void control_power_sequence()
{
	if (gpio_get(FM_EXP_MAIN_PWR_EN) == POWER_ON) { // op power on
		if (!is_all_sequence_done(POWER_ON)) {
			abort_power_thread();
			init_power_on_thread();
		}
	} else { // op power off
		if (!is_all_sequence_done(POWER_OFF)) {
			abort_power_thread();
			init_power_off_thread();
		}
	}
}

void init_power_on_thread()
{
	// Avoid re-create thread by checking thread status and thread id
	if (power_tid != NULL && strcmp(k_thread_state_str(power_tid), "dead") != 0) {
		return;
	}
	power_tid = k_thread_create(&power_thread_handler, power_thread,
				    K_THREAD_STACK_SIZEOF(power_thread), control_power_on_sequence,
				    NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
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
