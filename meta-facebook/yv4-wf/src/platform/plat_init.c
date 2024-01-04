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
#include "util_sys.h"
#include "pldm_monitor.h"
#include "power_status.h"
#include "plat_gpio.h"
#include "plat_power_seq.h"
#include "plat_pldm_monitor.h"
#include "plat_isr.h"
#include "plat_mctp.h"
#include "plat_i2c_target.h"

#define DEF_PROJ_GPIO_PRIORITY 78

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);

K_WORK_DELAYABLE_DEFINE(cxl_ready_check, cxl_ready_handler);
void pal_set_sys_status()
{
	set_mb_dc_status(FM_POWER_EN_R);
	set_DC_status(PG_CARD_OK);
	set_DC_on_delayed_status();
	set_ioe_init();
	if (gpio_get(PG_CARD_OK) == POWER_ON) {
		k_work_schedule(&cxl_ready_check, K_SECONDS(CXL_READY_SECONDS));
	}
	set_sys_ready_pin(BIC_READY_R);
}

void pal_pre_init()
{
	/* init i2c target */
	for (int index = 0; index < MAX_TARGET_NUM; index++) {
		if (I2C_TARGET_ENABLE_TABLE[index])
			i2c_target_control(
				index, (struct _i2c_target_config *)&I2C_TARGET_CONFIG_TABLE[index],
				1);
	}
}

void pal_post_init()
{
	pldm_load_state_effecter_table(PLAT_PLDM_MAX_STATE_EFFECTER_IDX);
	pldm_assign_gpio_effecter_id(PLAT_EFFECTER_ID_GPIO_HIGH_BYTE);
	plat_mctp_init();
}
