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

LOG_MODULE_REGISTER(plat_init);

#define CXL_IOEXP_INIT_RETRY_COUNT 5
#define CXL_IOEXP_INIT_DELAY_MS 10

SCU_CFG scu_cfg[] = {
	//register    value
};

void check_cxl_ioexp_is_initialized()
{
	int ret = 0;
	uint8_t retry_count = 0;
	uint8_t index = 0;
	uint8_t cxl_id = 0;
	uint8_t card_type = 0;
	uint8_t cxl_channel = 0;
	uint8_t gpio_alert_pin = 0;

	for (index = CARD_1_INDEX; index <= CARD_12_INDEX; ++index) {
		ret = get_pcie_card_type(index, &card_type);
		if (ret != 0) {
			continue;
		}

		if (card_type == CXL_CARD) {
			ret = get_cxl_ioexp_alert_pin(index, &gpio_alert_pin);
			if (ret != 0) {
				continue;
			}

			ret = pcie_card_id_to_cxl_e1s_id(index, &cxl_id);
			if (ret != 0) {
				continue;
			}

			for (retry_count = 0; retry_count < CXL_IOEXP_INIT_RETRY_COUNT;
			     ++retry_count) {
				if (gpio_get(gpio_alert_pin) != HIGH_ACTIVE) {
					cxl_channel = BIT(cxl_id);
					ret = cxl_ioexp_init(cxl_channel);
					if (ret != 0) {
						LOG_ERR("cxl: 0x%x ioexp initial fail", cxl_id);
					}
				} else {
					break;
				}
			}

			k_msleep(CXL_IOEXP_INIT_DELAY_MS);
		}
	}
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

	check_pcie_card_type();
	check_cxl_ioexp_is_initialized();
}

void pal_post_init()
{
	plat_mctp_init();
}

void pal_device_init()
{
	return;
}

void pal_set_sys_status()
{
	return;
}

#define DEF_PROJ_GPIO_PRIORITY 78

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);
