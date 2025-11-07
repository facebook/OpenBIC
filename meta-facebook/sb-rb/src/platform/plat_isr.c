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
#include "plat_log.h"
#include "plat_event.h"

LOG_MODULE_REGISTER(plat_isr);

K_TIMER_DEFINE(check_ubc_delayed_timer, check_ubc_delayed_timer_handler, NULL);

void ISR_GPIO_ALL_VR_PM_ALERT_R_N()
{
	LOG_DBG("gpio_%d_isr called, val=%d , dir= %d", ALL_VR_PM_ALERT_R_N,
		gpio_get(ALL_VR_PM_ALERT_R_N), gpio_get_direction(ALL_VR_PM_ALERT_R_N));

	check_cpld_polling_alert_status();
	if (gpio_get(ALL_VR_PM_ALERT_R_N) == GPIO_LOW) {
		give_all_vr_pm_alert_sem();
	}
}

void ISR_GPIO_FM_PLD_UBC_EN_R()
{
	LOG_DBG("gpio_%d_isr called, val=%d , dir= %d", FM_PLD_UBC_EN_R, gpio_get(FM_PLD_UBC_EN_R),
		gpio_get_direction(FM_PLD_UBC_EN_R));

	LOG_INF("FM_PLD_UBC_EN_R = %d", gpio_get(FM_PLD_UBC_EN_R));

	if (gpio_get(FM_PLD_UBC_EN_R) == GPIO_HIGH) {
		//plat_clock_init();
		plat_set_dc_on_log(LOG_ASSERT);
	}

	if (gpio_get(FM_PLD_UBC_EN_R) == GPIO_LOW) {
		plat_set_dc_on_log(LOG_DEASSERT);
	}

	k_timer_start(&check_ubc_delayed_timer, K_MSEC(1000), K_NO_WAIT);
}

void ISR_GPIO_RST_IRIS_PWR_ON_PLD_R1_N()
{
	LOG_INF("dc off, clear io expander init flag");
	set_ioe_init_flag(0);
}

bool plat_gpio_immediate_int_cb(uint8_t gpio_num)
{
	bool ret = false;

	switch (gpio_num) {
	case ALL_VR_PM_ALERT_R_N:
		ret = true;
		break;
	default:
		break;
	}

	return ret;
}