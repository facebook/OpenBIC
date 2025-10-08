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
#include "util_worker.h"

/* BMC reset */
void BMC_reset_handler()
{
	gpio_set(RST_BMC_R_N, GPIO_LOW);
	k_msleep(10);
	gpio_set(RST_BMC_R_N, GPIO_HIGH);
}

K_WORK_DELAYABLE_DEFINE(BMC_reset_work, BMC_reset_handler);
int pal_submit_bmc_cold_reset()
{
	k_work_schedule_for_queue(&plat_work_q, &BMC_reset_work, K_MSEC(BMC_COLD_RESET_DELAY_MS));
	return 0;
}
/* BMC reset */
