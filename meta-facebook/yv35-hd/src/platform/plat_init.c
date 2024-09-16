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
#include "util_sys.h"
#include "plat_gpio.h"
#include "plat_class.h"
#include "snoop.h"
#include "pcc.h"
#include "plat_i2c.h"
#include "plat_pmic.h"
#include "plat_apml.h"
#include "plat_kcs.h"
#include "rg3mxxb12.h"
#include "util_worker.h"
#include <drivers/flash.h>
#include "util_spi.h"
#include "hal_gpio.h"

#define WINBOND_MFG_ID 0xEF

SCU_CFG scu_cfg[] = {
	//register    value
	{ 0x7e6e2610, 0xffffffff },
	{ 0x7e6e2614, 0xffffffff },
	{ 0x7e6e2618, 0xdc000000 },
	{ 0x7e6e261c, 0x00000F32 },
};

void pal_device_init()
{
	start_monitor_pmic_error_thread();
}

void pal_pre_init()
{
	init_platform_config();
	CARD_STATUS _1ou_status = get_1ou_status();
	CARD_STATUS _2ou_status = get_2ou_status();
	if (_1ou_status.present && (_1ou_status.card_type == TYPE_1OU_EXP_WITH_E1S)) {
		// Initialize I3C HUB (HD BIC connects to Olympic2 1ou expension-A and B)
		if (!rg3mxxb12_i2c_mode_only_init(I2C_BUS8, BIT(2), rg3mxxb12_ldo_1_8_volt,
						  rg3mxxb12_pullup_1k_ohm)) {
			printk("failed to initialize 1ou rg3mxxb12\n");
		}
	}
	if (_2ou_status.present && (_1ou_status.card_type == TYPE_1OU_EXP_WITH_E1S)) {
		// Initialize I3C HUB (HD BIC connects to Olympic2 3ou expension-A and B)
		if (!rg3mxxb12_i2c_mode_only_init(I2C_BUS9, BIT(2), rg3mxxb12_ldo_1_8_volt,
						  rg3mxxb12_pullup_1k_ohm)) {
			printk("failed to initialize 3ou rg3mxxb12\n");
		}
	}
	scu_init(scu_cfg, sizeof(scu_cfg) / sizeof(SCU_CFG));
	pcc_init();
	apml_init();
	init_plat_worker(CONFIG_MAIN_THREAD_PRIORITY + 1); // work queue for low priority jobs
}

void config_spi_flash()
{
	const struct device *flash_dev;

	uint8_t buf[1] = { 0 };
	uint8_t jedec_id[3] = { 0 };

	pal_switch_bios_spi_mux(GPIO_HIGH);

	flash_dev = device_get_binding("spi1_cs0");
	if (!device_is_ready(flash_dev)) {
		printk("%s device not ready\n", __func__);
		pal_switch_bios_spi_mux(GPIO_LOW);
		return;
	}

	spi_nor_re_init(flash_dev);

	flash_read_jedec_id(flash_dev, jedec_id);

	if (jedec_id[0] == WINBOND_MFG_ID) {
		// Set winbond flash driver strength to 75%
		buf[0] = 0x20;
		flash_reg_write(flash_dev, 0x11, buf, 1);
	}

	pal_switch_bios_spi_mux(GPIO_LOW);
	return;
}

void pal_post_init()
{
	kcs_init();
	config_spi_flash();
}

void pal_set_sys_status()
{
	set_DC_status(PWRGD_CPU_LVC3);
	set_DC_on_delayed_status();
	set_post_status(FM_BIOS_POST_CMPLT_BIC_N);
	if (get_post_status()) {
		apml_recovery();
		set_tsi_threshold();
		read_cpuid();
	}
	set_sys_ready_pin(BIC_READY);
}

#define DEF_PROJ_GPIO_PRIORITY 78

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);
