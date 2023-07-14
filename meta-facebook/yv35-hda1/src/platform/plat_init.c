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

#include <stdlib.h>
#include "hal_gpio.h"
#include "power_status.h"
#include "util_sys.h"
#include "plat_gpio.h"
#include "plat_class.h"
#include "mpro.h"
#include "plat_i2c.h"
#include "plat_mctp.h"
#include "plat_ssif.h"
#include "plat_power_status.h"
#include "util_worker.h"

SCU_CFG scu_cfg[] = {
	//register    value
	/* Set GPIOA/B/C/D internal pull-up/down after gpio init */
	{ 0x7e6e2610, 0x0FFFFFFF },
	/* Set GPIOF/G/H internal pull-up/down after gpio init */
	{ 0x7e6e2614, 0xFFFFBBFF },
	/* Set GPIOJ/K/L internal pull-up/down after gpio init */
	{ 0x7e6e2618, 0xCF000000 },
	/* Set GPIOM/N/O/P internal pull-up/down after gpio init */
	{ 0x7e6e261c, 0x00000032 },
	/* Set GPIOQ/R/S/T internal pull-up/down after gpio init */
	{ 0x7e6e2630, 0xFF000000 },
	/* Set GPIOU/V/X internal pull-up/down after gpio init */
	{ 0x7e6e2634, 0x000000FF },
};

void pal_pre_init()
{
	bic_heart_beat_init();

	mpro_heart_beat_check_init();

	init_platform_config();

	scu_init(scu_cfg, sizeof(scu_cfg) / sizeof(SCU_CFG));

	mpro_postcode_read_init();

	init_plat_worker(CONFIG_MAIN_THREAD_PRIORITY + 1); // work queue for low priority jobs
}

void pal_post_init()
{
	plat_mctp_init();

	ssif_init();
}

void pal_set_sys_status()
{
	set_DC_status(BMC_GPIOL1_SYS_PWRGD);
	set_DC_on_delayed_status();
	set_post_status(FM_BIOS_POST_CMPLT_BIC_N);
	set_sys_ready_pin(BMC_GPIOO2_READY);
	set_mpro_status();
}

#define DEF_PROJ_GPIO_PRIORITY 78

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);
