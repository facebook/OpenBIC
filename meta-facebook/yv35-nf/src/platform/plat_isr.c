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

#include "hal_gpio.h"
#include "plat_gpio.h"
#include "plat_power_seq.h"
#include "plat_isr.h"

void ISR_MB_DC_STATE()
{
	set_MB_DC_status(POWER_EN_R);
	control_power_sequence();
}

void ISR_MB_PCIE_RST()
{
	gpio_set(PERST_ASIC_N_R, gpio_get(RST_PCIE_MB_EXP_N));
}

static void handle_pwr_on_rst(struct k_work *work)
{
	gpio_set(PWR_ON_RST_N_R, GPIO_HIGH);
}

// If ASIC P1V8 high, rising SYS_RST_N
// And rising PWR_ON_RST_N after they have risen after 25 ms
K_WORK_DELAYABLE_DEFINE(handle_pwr_on_rst_work, handle_pwr_on_rst);
void ISR_PWRGD_P1V8_ASIC()
{
	gpio_set(SYS_RST_N_R, GPIO_HIGH);
	k_work_schedule(&handle_pwr_on_rst_work, K_MSEC(RISING_PWR_ON_RST_N_DELAY_MS));
}
