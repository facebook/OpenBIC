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
#include "plat_vw_gpio.h"
#include "power_status.h"
#include "util_sys.h"
#include "plat_class.h"
#include "plat_gpio.h"
#include "plat_dimm.h"
#include "plat_i3c.h"
#include "snoop.h"
#include "pcc.h"
#include "plat_i2c.h"
#include "plat_pmic.h"
#include "plat_cpu.h"
#include "plat_kcs.h"
#include "rg3mxxb12.h"
#include "p3h284x.h"
#include "util_worker.h"

/*
 * The operating voltage of GPIO input pins are lower than actual voltage because the chip
 * internal pull-down is enabled.
 * BIC disables the internal GPIO pull-down for all input pins.
 *
 * Disabled GPIO list (Refer to chapter 18 of AST1030's datasheet) :
 * GPIOA0 GPIOA1 GPIOA2 GPIOA4
 * GPIOB5 GPIOB6
 * GPIOC0 GPIOC1 GPIOC2 GPIOC4 GPIOC5 GPIOC6 GPIOC7
 * GPIOD0 GPIOD2
 * GPIOF1 GPIOF4
 * GPIOG0 GPIOG2 GPIOG5 GPIOG6 GPIOG7
 * GPIOH1 GPIOH2 GPIOH3
 * GPIOL3 GPIOL4 GPIOL5 GPIOL6 GPIOL7
 * GPIOM1 GPIOM4 GPIOM5
 */
SCU_CFG scu_cfg[] = {
	//register    value
	{ 0x7e6e2610, 0x05F76017 },
	{ 0x7e6e2614, 0x0EE51200 },
	{ 0x7e6e2618, 0xF8000000 },
	{ 0x7e6e261C, 0x00000032 },
};

void pal_pre_init()
{
	uint16_t exp_i3c_hub_type = I3C_HUB_TYPE_UNKNOWN;
	init_platform_config();
	init_i3c_hub_type();
	init_i3c_hub();
	exp_i3c_hub_type = get_exp_i3c_hub_type();
	CARD_STATUS _1ou_status = get_1ou_status();
	CARD_STATUS _2ou_status = get_2ou_status();
	if (_1ou_status.present && (_1ou_status.card_type == TYPE_1OU_OLMSTED_POINT)) {
		if (exp_i3c_hub_type == RG3M87B12_DEVICE_INFO) {
			// Initialize I3C HUB (HD BIC connects to Olympic2 1ou expension-A and B)
			if (!rg3mxxb12_i2c_mode_only_init(I2C_BUS8, BIT(2), rg3mxxb12_ldo_1_8_volt,
							  rg3mxxb12_pullup_1k_ohm)) {
				printk("failed to initialize 1ou rg3mxxb12\n");
			}
		} else {
			if (!p3h284x_i2c_mode_only_init(I2C_BUS8, BIT(2), p3g284x_ldo_1_8_volt,
							p3g284x_pullup_1k_ohm)) {
				printk("failed to initialize 1ou p3h284x\n");
			}
		}
	}
	if (_2ou_status.present && (_1ou_status.card_type == TYPE_1OU_OLMSTED_POINT)) {
		// Initialize I3C HUB (HD BIC connects to Olympic2 3ou expension-A and B)
		if (exp_i3c_hub_type == RG3M87B12_DEVICE_INFO) {
			if (!rg3mxxb12_i2c_mode_only_init(I2C_BUS9, BIT(2), rg3mxxb12_ldo_1_8_volt,
							  rg3mxxb12_pullup_1k_ohm)) {
				printk("failed to initialize 3ou rg3mxxb12\n");
			}
		} else {
			if (!p3h284x_i2c_mode_only_init(I2C_BUS9, BIT(2), p3g284x_ldo_1_8_volt,
							p3g284x_pullup_1k_ohm)) {
				printk("failed to initialize 1ou p3h284x\n");
			}
		}
	}
	scu_init(scu_cfg, ARRAY_SIZE(scu_cfg));
	if (!pal_load_vw_gpio_config()) {
		printk("failed to initialize vw gpio\n");
	}

	init_plat_worker(CONFIG_MAIN_THREAD_PRIORITY + 1); // work queue for low priority jobs
}

void pal_post_init()
{
	kcs_init();
}

void pal_set_sys_status()
{
	set_DC_status(PWRGD_CPU_LVC3);
	set_DC_on_delayed_status();
	set_CPU_power_status(PWRGD_CPU_LVC3);
	set_post_thread();
	set_sys_ready_pin(BIC_READY);
}

void pal_device_init()
{
	init_i3c_dimm_prsnt_status();
	start_get_dimm_info_thread();
	start_monitor_pmic_error_thread();
	start_monitor_cpu_thread();
	start_monitor_smi_thread();
}

#define DEF_PROJ_GPIO_PRIORITY 78

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);
