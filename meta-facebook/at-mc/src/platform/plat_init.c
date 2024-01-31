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
#include "libutil.h"
#include "hal_gpio.h"
#include "hal_peci.h"
#include "power_status.h"
#include "util_sys.h"
#include "plat_gpio.h"
#include "plat_mctp.h"
#include "plat_class.h"
#include "plat_dev.h"
#include "plat_i2c_target.h"
#include "util_worker.h"
#include "plat_isr.h"
#include "plat_sys.h"
#include "plat_pldm_monitor.h"

LOG_MODULE_REGISTER(plat_init);

SCU_CFG scu_cfg[] = {
	//register    value
};

void check_mb_reset_status()
{
	int ret = 0;
	int power_status = 0;
	uint8_t index = 0;
	uint8_t cxl_id = 0;
	uint8_t card_type = 0;

	for (index = CARD_1_INDEX; index <= CARD_12_INDEX; ++index) {
		ret = get_pcie_card_type(index, &card_type);
		if (ret != 0) {
			continue;
		}

		if (card_type == CXL_CARD) {
			power_status = get_pcie_card_power_status(index);
			if (power_status & PCIE_CARD_POWER_GOOD_BIT) {
				ret = pcie_card_id_to_cxl_id(index, &cxl_id);
				if (ret != 0) {
					continue;
				}

				cxl_mb_status_init(cxl_id);
			}
		}
	}
}

void check_cxl_ioexp_is_initialized()
{
	int ret = 0;
	uint8_t index = 0;
	uint8_t cxl_id = 0;
	uint8_t card_type = 0;

	for (index = CARD_1_INDEX; index <= CARD_12_INDEX; ++index) {
		ret = get_pcie_card_type(index, &card_type);
		if (ret != 0) {
			continue;
		}

		if (card_type == CXL_CARD) {
			ret = pcie_card_id_to_cxl_id(index, &cxl_id);
			if (ret != 0) {
				continue;
			}

			init_cxl_card_ioexp(cxl_id);
		}
	}
}

void pal_pre_init()
{
	check_pcie_card_type();
	check_cxl_ioexp_is_initialized();

	/* init i2c target */
	for (int index = 0; index < MAX_TARGET_NUM; index++) {
		if (index == CXL_I2C_TARGET_INDEX) {
			/* BIC will not register target bus if CXL is not present */
			if (is_cxl_present() == false) {
				continue;
			}

			init_cxl_work();
		}

		if (I2C_TARGET_ENABLE_TABLE[index])
			i2c_target_control(
				index, (struct _i2c_target_config *)&I2C_TARGET_CONFIG_TABLE[index],
				1);
	}

	init_plat_worker(CONFIG_MAIN_THREAD_PRIORITY + 1); // work queue for low priority jobs
}

void pal_post_init()
{
	plat_mctp_init();
	check_mb_reset_status();
	check_debug_sel_mode_status();
	if (is_ac_lost()) {
		plat_ssd_present_check();
	}
	init_accl_presence_check_work();
}

void pal_device_init()
{
	return;
}

void pal_set_sys_status()
{
	set_DC_status(MEB_NORMAL_PWRGD_BIC);
	set_reset_smb4_mux_pin();
	init_ssd_power_fault_work();
}

#define DEF_PLAT_CONFIG_PRIORITY 77
#define DEF_PROJ_GPIO_PRIORITY 78

DEVICE_DEFINE(PRE_DEF_PLAT_CONFIG, "PRE_DEF_PLATFOMR", &init_platform_config, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PLAT_CONFIG_PRIORITY, NULL);

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);
