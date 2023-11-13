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

#include "util_worker.h"
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

static void CXL_READY_handler()
{
	const struct device *heartbeat = NULL;
	int heartbeat_status = 0;

	heartbeat = device_get_binding(CXL_HEART_BEAT_LABEL);
	if (heartbeat == NULL) {
		LOG_ERR("%s device not found", CXL_HEART_BEAT_LABEL);
		return;
	}

	for (int times = 0; times < CXL_READY_RETRY_TIMES; times++) {
		heartbeat_status = sensor_sample_fetch(heartbeat);
		if (heartbeat_status < 0) {
			k_sleep(K_SECONDS(CXL_READY_INTERVAL_SECONDS));
			continue;
		}
		/* Switch muxs to BIC*/
		gpio_set(SEL_SMB_MUX_PMIC_R, GPIO_HIGH);
		gpio_set(SEL_SMB_MUX_DIMM_R, GPIO_HIGH);
		return;
	}
	LOG_ERR("Failed to read %s due to sensor_sample_fetch failed, ret: %d",
		CXL_HEART_BEAT_LABEL, heartbeat_status);
	return;
}

K_WORK_DELAYABLE_DEFINE(CXL_READY_thread, CXL_READY_handler);

void ISR_CXL_PG_ON()
{
	if (k_work_cancel_delayable(&CXL_READY_thread) != 0) {
		LOG_ERR("Failed to cancel CXL_READY thread");
	}
	k_work_schedule(&CXL_READY_thread, K_SECONDS(CXL_READY_INTERVAL_SECONDS));
}
