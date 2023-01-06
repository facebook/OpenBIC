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
#include "plat_gpio.h"
#include "plat_class.h"
#include "plat_i2c.h"
#include "rg3mxxb12.h"

#define DEF_PLAT_CONFIG_PRIORITY 77
#define DEF_PROJ_GPIO_PRIORITY 78

void pal_pre_init()
{
	/* Initialize I3C HUB (connects to E1.s)
	 * For OPA expansion,
	 * the I3C HUB slave port-0/1/5 should be enabled.
	 * For OPB expansion,
	 * the I3C HUB slave port-0/1/2/3/4 should be enabled.
	 */
	uint8_t type = get_card_type();
	uint8_t slave_port = 0x0;
	if (type == CARD_TYPE_OPA) {
		slave_port = BIT(0)|BIT(1)|BIT(5);
	}
	else if(type == CARD_TYPE_OPB) {
		slave_port = BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4);
	}
	else {
		printk("No need to initialize rg3mxxb12\n");
		return;
	}

	if (!rg3mxxb12_i2c_mode_only_init(I2C_BUS1, slave_port)) {
		printk("failed to initialize rg3mxxb12\n");
	}
}

DEVICE_DEFINE(PRE_DEF_PLAT_CONFIG, "PRE_DEF_PLATFOMR", &init_platform_config, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PLAT_CONFIG_PRIORITY, NULL);

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);
