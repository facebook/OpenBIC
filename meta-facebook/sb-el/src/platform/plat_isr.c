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
#include "shell_arke_power.h"

LOG_MODULE_REGISTER(plat_isr);

void ISR_GPIO_ALL_VR_PM_ALERT_R_N()
{
	LOG_DBG("gpio_%d_isr called, val=%d , dir= %d", ALL_VR_PM_ALERT_R_N,
		gpio_get(ALL_VR_PM_ALERT_R_N), gpio_get_direction(ALL_VR_PM_ALERT_R_N));

	check_cpld_polling_alert_status();
}

void ISR_GPIO_RST_IRIS_PWR_ON_PLD_R1_N()
{
	// dc on
	if (gpio_get(RST_ARKE_PWR_ON_PLD_R1_N)) {
		// ioexp_init();
		for (int i = 0; i < CLK_COMPONENT_MAX; i++) {
			clear_clock_status(NULL, i);
		}
		// add_sync_oc_warn_to_work();
		// if board id == EVB , ctrl fan pwm
		// // when dc on clear cpld polling alert status
		// uint8_t err_type = CPLD_UNEXPECTED_VAL_TRIGGER_CAUSE;
		// LOG_DBG("cpld_polling_alert_status: true -> false, reset_error_log_states: %x",
		// 	err_type);
		// reset_error_log_states(err_type);
	} else {
		LOG_INF("dc off");
		// set_ioe_init_flag(0);
	}
}
