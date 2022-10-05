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

#include <stdio.h>
#include <stdlib.h>
#include <logging/log.h>
#include "hal_gpio.h"
#include "plat_gpio.h"
#include "libutil.h"

LOG_MODULE_REGISTER(plat_sys);

/* BMC reset */
void BMC_reset_handler()
{
	LOG_WRN("[%s] BMC reset not supported from here\n", __func__);
}

K_WORK_DELAYABLE_DEFINE(BMC_reset_work, BMC_reset_handler);
int pal_submit_bmc_cold_reset()
{
	k_work_schedule(&BMC_reset_work, K_MSEC(1000));
	return 0;
}
/* BMC reset */
