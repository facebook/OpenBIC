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
#include <drivers/sensor.h>
#include <drivers/pwm.h>

#include "hal_gpio.h"
#include "plat_gpio.h"
#include "plat_power_seq.h"
#include "plat_isr.h"

LOG_MODULE_REGISTER(plat_isr);

K_WORK_DEFINE(cxl_power_on_work, execute_power_on_sequence);
K_WORK_DEFINE(cxl_power_off_work, execute_power_off_sequence);

void ISR_MB_DC_STATUS_CHANGE()
{
	set_mb_dc_status(POWER_EN_R);

	if (gpio_get(POWER_EN_R) == POWER_ON) {
		k_work_submit(&cxl_power_on_work);
	} else {
		k_work_submit(&cxl_power_off_work);
	}
}

void ISR_MB_PCIE_RST()
{
	gpio_set(PERST_ASIC_N_R, gpio_get(RST_PCIE_MB_EXP_N));
}
