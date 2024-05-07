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
#include "hal_peci.h"
#include "power_status.h"
#include "util_sys.h"
#include "plat_gpio.h"
#include "plat_mctp.h"
#include "plat_class.h"
#include "plat_i2c_target.h"
#include "util_worker.h"
#include "plat_pldm_monitor.h"
#include "plat_dev.h"
#include "plat_pldm_fw_update.h"

SCU_CFG scu_cfg[] = {
	//register    value
	{ 0x7e6e2618, 0xFF000000 },
	{ 0x7e6e261c, 0x0000003A },
};

void pal_pre_init()
{
	/* init i2c target */
	for (int index = 0; index < MAX_TARGET_NUM; index++) {
		if (I2C_TARGET_ENABLE_TABLE[index])
			i2c_target_control(
				index, (struct _i2c_target_config *)&I2C_TARGET_CONFIG_TABLE[index],
				1);
	}

	scu_init(scu_cfg, sizeof(scu_cfg) / sizeof(SCU_CFG));
	init_asic_jtag_select_ioexp();
	check_accl_device_presence_status_via_ioexp();
	get_acb_power_status();
	init_sw_heartbeat_work();
	init_plat_worker(CONFIG_MAIN_THREAD_PRIORITY + 1); // work queue for low priority jobs
}

void pal_post_init()
{
	uint8_t board_revision = get_board_revision();
	plat_mctp_init();
	pldm_load_state_effecter_table(MAX_STATE_EFFECTER_IDX);
	pldm_assign_gpio_effecter_id(PLAT_EFFECTER_ID_GPIO_HIGH_BYTE);
	/* Send device presence log when the BIC is AC on */
	if (is_ac_lost()) {
		plat_fio_present_check();
		plat_accl_present_check();
		if (board_revision > EVT2_STAGE) {
			plat_accl_power_cable_present_check();
		}
	}

	if (board_revision > EVT2_STAGE) {
		init_accl_presence_check_work();
	}
}

void pal_device_init()
{
	return;
}

void pal_set_sys_status()
{
	gpio_set(ACB_BIC_READY_N, GPIO_LOW);
	return;
}

#define DEF_PLAT_CONFIG_PRIORITY 77
#define DEF_PROJ_GPIO_PRIORITY 78

DEVICE_DEFINE(PRE_DEF_PLAT_CONFIG, "PRE_DEF_PLATFOMR", &init_platform_config, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PLAT_CONFIG_PRIORITY, NULL);

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);
