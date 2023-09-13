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

#include "plat_isr.h"
#include <logging/log.h>
#include "libipmi.h"
#include "kcs.h"
#include "power_status.h"
#include "sensor.h"
#include "snoop.h"
#include "plat_gpio.h"
#include "plat_class.h"
#include "plat_sensor_table.h"
#include "plat_i2c.h"
#include "hal_gpio.h"
#include "hal_i2c.h"
#include "util_sys.h"
#include "util_worker.h"
#include "plat_mctp.h"

LOG_MODULE_REGISTER(plat_isr, LOG_LEVEL_DBG);

K_WORK_DELAYABLE_DEFINE(set_DC_on_5s_work, set_DC_on_delayed_status);
#define DC_ON_5_SECOND 5
void ISR_DC_ON()
{
	set_DC_status(PWRGD_CPU_LVC3);

	bool dc_status = get_DC_status();

	if (dc_status) {
		k_work_schedule(&set_DC_on_5s_work, K_SECONDS(DC_ON_5_SECOND));
	} else {
		set_DC_on_delayed_status();
	}
}
