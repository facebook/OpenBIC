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
#include "libutil.h"
#include "libipmi.h"
#include "power_status.h"
#include "sensor.h"

#include "plat_gpio.h"
#include "plat_i2c.h"
#include "plat_class.h"
#include "plat_isr.h"
#include "plat_hwmon.h"
#include "plat_event.h"

extern uint8_t ina230_init(sensor_cfg *cfg);

LOG_MODULE_REGISTER(plat_isr);

K_TIMER_DEFINE(check_ubc_delayed_timer, check_ubc_delayed, NULL);

void ISR_GPIO_FM_ASIC_0_THERMTRIP_R_N()
{
	LOG_INF("gpio_%d_isr called, val=%d , dir= %d", FM_ASIC_0_THERMTRIP_R_N,
		gpio_get(FM_ASIC_0_THERMTRIP_R_N), gpio_get_direction(FM_ASIC_0_THERMTRIP_R_N));
}

void ISR_GPIO_RST_ATH_PWR_ON_PLD_R1_N()
{
	LOG_INF("gpio_%d_isr called, val=%d , dir= %d", RST_ATH_PWR_ON_PLD_R1_N,
		gpio_get(RST_ATH_PWR_ON_PLD_R1_N), gpio_get_direction(RST_ATH_PWR_ON_PLD_R1_N));
}

void ISR_GPIO_ATH_CURRENT_SENSE_0_NPCM_R()
{
	LOG_INF("gpio_%d_isr called, val=%d , dir= %d", ATH_CURRENT_SENSE_0_NPCM_R,
		gpio_get(ATH_CURRENT_SENSE_0_NPCM_R),
		gpio_get_direction(ATH_CURRENT_SENSE_0_NPCM_R));
}

void ISR_GPIO_ATH_CURRENT_SENSE_1_NPCM_R()
{
	LOG_INF("gpio_%d_isr called, val=%d , dir= %d", ATH_CURRENT_SENSE_1_NPCM_R,
		gpio_get(ATH_CURRENT_SENSE_1_NPCM_R),
		gpio_get_direction(ATH_CURRENT_SENSE_1_NPCM_R));
}

void ISR_GPIO_FM_ATH_HBM3_CATTRIP_ALARM_LV33_R()
{
	LOG_INF("gpio_%d_isr called, val=%d , dir= %d", FM_ATH_HBM3_CATTRIP_ALARM_LV33_R,
		gpio_get(FM_ATH_HBM3_CATTRIP_ALARM_LV33_R),
		gpio_get_direction(FM_ATH_HBM3_CATTRIP_ALARM_LV33_R));
}

void ISR_GPIO_ALL_VR_PM_ALERT_R_N()
{
	LOG_INF("gpio_%d_isr called, val=%d , dir= %d", ALL_VR_PM_ALERT_R_N,
		gpio_get(ALL_VR_PM_ALERT_R_N), gpio_get_direction(ALL_VR_PM_ALERT_R_N));
}

void ISR_GPIO_ATH_SMB_ALERT_NPCM_LVC33_R_N()
{
	LOG_INF("gpio_%d_isr called, val=%d , dir= %d", ATH_SMB_ALERT_NPCM_LVC33_R_N,
		gpio_get(ATH_SMB_ALERT_NPCM_LVC33_R_N),
		gpio_get_direction(ATH_SMB_ALERT_NPCM_LVC33_R_N));
}

void ISR_GPIO_FM_PLD_UBC_EN_R()
{
	LOG_INF("gpio_%d_isr called, val=%d , dir= %d", FM_PLD_UBC_EN_R, gpio_get(FM_PLD_UBC_EN_R),
		gpio_get_direction(FM_PLD_UBC_EN_R));

	k_timer_start(&check_ubc_delayed_timer, K_MSEC(3000), K_NO_WAIT);
	set_dc_status_changing_status(true);
}
