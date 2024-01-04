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
#include "power_status.h"
#include "rg3mxxb12.h"
#include "p3h284x.h"
#include "plat_gpio.h"
#include "plat_class.h"
#include "plat_i2c.h"
#include "plat_isr.h"
#include "plat_power_seq.h"

#define DEF_PLAT_CONFIG_PRIORITY 77
#define DEF_PROJ_GPIO_PRIORITY 78

SCU_CFG opa_scu_cfg[] = {
	//register    value
	{ 0x7e6e2610, 0xffffffff }, //disable GPIO internal pull down #0
	{ 0x7e6e2614, 0xffffffff }, //disable GPIO internal pull down #1
	{ 0x7e6e2618, 0xffffffff }, //disable GPIO internal pull down #2
	{ 0x7e6e261c, 0xffffffff }, //disable GPIO internal pull down #3
	{ 0x7e78001c, 0x5000010f }, //enable GPIO A/B/C/D reset tolerant
	{ 0x7e78003c, 0x0000007f }, //enable GPIO E/F/G/H reset tolerant
	{ 0x7e7800ac, 0x1c000000 }, //enable GPIO I/J/K/L reset tolerant
};

SCU_CFG opb_scu_cfg[] = {
	//register    value
	{ 0x7e6e2610, 0xffffffff }, //disable GPIO internal pull down #0
	{ 0x7e6e2614, 0xffffffff }, //disable GPIO internal pull down #1
	{ 0x7e6e2618, 0xffffffff }, //disable GPIO internal pull down #2
	{ 0x7e6e261c, 0xffffffff }, //disable GPIO internal pull down #3
	{ 0x7e78001c, 0xc000001f }, //enable GPIO A/B/C/D reset tolerant
	{ 0x7e78003c, 0x000000ff }, //enable GPIO E/F/G/H reset tolerant
	{ 0x7e7800ac, 0x7c000000 }, //enable GPIO I/J/K/L reset tolerant
};

void pal_pre_init()
{
	uint8_t type = get_card_type();
	uint8_t slave_port = 0x0;
	uint16_t i3c_hub_type = I3C_HUB_TYPE_UNKNOWN;

	init_board_revision();
	init_i3c_hub_type();
	i3c_hub_type = get_i3c_hub_type();

	/* Initialize I3C HUB (connects to E1.s)
	 * For OPA expansion,
	 * the I3C HUB slave port-0/1/5 should be enabled.
	 * For OPB expansion,
	 * the I3C HUB slave port-0/1/2/3/4 should be enabled.
	 */

	switch (type) {
	case CARD_TYPE_OPA:
		//Disable GPIO internal pull down and enable GPIO tolerance
		scu_init(opa_scu_cfg, sizeof(opa_scu_cfg) / sizeof(SCU_CFG));

		slave_port = BIT(0) | BIT(1) | BIT(5);
		break;
	case CARD_TYPE_OPB:
		//Disable GPIO internal pull down and enable GPIO tolerance
		scu_init(opb_scu_cfg, sizeof(opb_scu_cfg) / sizeof(SCU_CFG));

		slave_port = BIT(0) | BIT(1) | BIT(2) | BIT(3) | BIT(4);
		break;
	default:
		printk("No need to initialize I3C hub rg3mxxb12\n");
		return;
	}

	if (i3c_hub_type == RG3M87B12_DEVICE_INFO) {
		if (!rg3mxxb12_i2c_mode_only_init(I2C_BUS2, slave_port, rg3mxxb12_ldo_1_2_volt,
						  rg3mxxb12_pullup_1k_ohm)) {
			printk("failed to initialize rg3mxxb12\n");
		}
	} else {
		if (!p3h284x_i2c_mode_only_init(I2C_BUS2, slave_port, p3g284x_ldo_1_2_volt,
						p3g284x_pullup_1k_ohm)) {
			printk("failed to initialize p3h284x\n");
		}
	}
}

DEVICE_DEFINE(PRE_DEF_PLAT_CONFIG, "PRE_DEF_PLATFOMR", &init_platform_config, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PLAT_CONFIG_PRIORITY, NULL);

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);

void pal_set_sys_status()
{
	init_sequence_status();
	set_DC_status(FM_EXP_MAIN_PWR_EN);
	control_power_sequence();
	set_DC_on_delayed_status();
	set_DC_off_delayed_status();
}
