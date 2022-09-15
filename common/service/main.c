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

#include "fru.h"
#include "hal_i2c.h"
#include "hal_wdt.h"
#include "ipmi.h"
#include "kcs.h"
#include "sensor.h"
#include "timer.h"
#include "usb.h"
#include <logging/log.h>

__weak void pal_pre_init()
{
	return;
}

__weak void pal_post_init()
{
	return;
}

__weak void pal_device_init()
{
	return;
}

__weak void pal_set_sys_status()
{
	return;
}

void main(void)
{
	printf("Hello, welcome to %s %s %x%x.%x.%x\n", PLATFORM_NAME, PROJECT_NAME, BIC_FW_YEAR_MSB,
	       BIC_FW_YEAR_LSB, BIC_FW_WEEK, BIC_FW_VER);

	wdt_init();
	util_init_timer();
	util_init_I2C();
	pal_pre_init();
	sensor_init();
	FRU_init();
	ipmi_init();
#ifdef CONFIG_IPMI_KCS_ASPEED
	kcs_init();
#endif
#ifdef CONFIG_USB
	usb_dev_init();
#endif
	pal_device_init();
	pal_set_sys_status();
	pal_post_init();
}
