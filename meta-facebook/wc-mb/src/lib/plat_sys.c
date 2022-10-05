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

#include "plat_sys.h"

#include "util_sys.h"
#include "hal_gpio.h"
#include "plat_gpio.h"

/* HOST control */
int pal_host_power_control(power_ctl_t ctl_type)
{
	uint32_t reg_val = sys_read32(PASSTHROUGH_REG);

	/* disable passthrough */
	sys_write32(reg_val & PASSTHROUGH_DISABLE, PASSTHROUGH_REG);

	switch (ctl_type) {
	case POWER_CTL_ON:
		gpio_set(PWR_BTN_BIC_OUT_R_N, GPIO_LOW);
		k_msleep(100);
		gpio_set(PWR_BTN_BIC_OUT_R_N, GPIO_HIGH);
		break;

	case POWER_CTL_OFF:
		gpio_set(PWR_BTN_BIC_OUT_R_N, GPIO_LOW);
		k_msleep(5000);
		gpio_set(PWR_BTN_BIC_OUT_R_N, GPIO_HIGH);
		break;

	case POWER_CTL_RESET:
		gpio_set(RST_BIC_RSTBTN_OUT_R_N, GPIO_LOW);
		k_msleep(100);
		gpio_set(RST_BIC_RSTBTN_OUT_R_N, GPIO_HIGH);
		break;

	default:
		break;
	}

	/* enable passthrough */
	sys_write32(reg_val | PASSTHROUGH_ENABLE, PASSTHROUGH_REG);

	return 0;
}

/* BMC present */
bool pal_is_bmc_present()
{
	if (gpio_get(FM_SCM_PRSNT_R_N) == GPIO_LOW)
		return true;
	return false;
}

/* BMC ready */
bool pal_is_bmc_ready()
{
	if (gpio_get(FM_BMC_READY) == GPIO_HIGH)
		return true;
	return false;
}

/* BMC reset */
void BMC_reset_handler()
{
	gpio_set(RST_BMC_R_N, GPIO_LOW);
	k_msleep(1000);
	gpio_set(RST_BMC_R_N, GPIO_HIGH);
}

K_WORK_DELAYABLE_DEFINE(BMC_reset_work, BMC_reset_handler);
int pal_submit_bmc_cold_reset()
{
	k_work_schedule(&BMC_reset_work, K_MSEC(1000));
	return 0;
}
/* BMC reset */
