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

#include "util_sys.h"
#include "power_status.h"
#include "hal_gpio.h"
#include "pldm_monitor.h"
#include "plat_mctp.h"
#include "plat_i2c_target.h"
#include "plat_pldm_monitor.h"
#include "plat_power_seq.h"
#include "plat_gpio.h"

#define DEF_PROJ_GPIO_PRIORITY 78

/*
 * The operating voltage of GPIO input pins are lower than actual voltage because the chip 
 * internal pull-down is enabled.
 * BIC disables the internal GPIO pull-down for all input pins.
 *
 * Disabled GPIO list (Refer to chapter 18 of AST1030's datasheet) :
 * GPIOA3
 * GPIOE0~GPIOE7
 * GPIOH1~GPIOU7
 */

// clang-format off
SCU_CFG scu_cfg[] = {
	//register    value
	{ 0x7e6e2610, 0xFFFFFFF7 },
	{ 0x7e6e2614, 0x01FFFF00 },
	{ 0x7e6e2618, 0x00000000 },
	{ 0x7e6e261c, 0x00000000 },
	{ 0x7e6e2630, 0x00000000 },
	{ 0x7e6e2634, 0x00000000 },
};
// clang-format on

void pal_pre_init()
{
	scu_init(scu_cfg, sizeof(scu_cfg) / sizeof(SCU_CFG));
	/* init i2c target */
	for (int index = 0; index < MAX_TARGET_NUM; index++) {
		if (I2C_TARGET_ENABLE_TABLE[index])
			i2c_target_control(
				index, (struct _i2c_target_config *)&I2C_TARGET_CONFIG_TABLE[index],
				1);
	}

	if (gpio_get(SEL_SMB_MUX_PMIC_R) ==
	    GPIO_LOW) { // BIC starts monitoring VR only after the PMIC mux is switched to BIC.
		set_vr_monitor_status(false);
	}
}

void pal_post_init()
{
	plat_mctp_init();
	pldm_load_state_effecter_table(PLAT_PLDM_MAX_STATE_EFFECTER_IDX);
	pldm_assign_gpio_effecter_id(PLAT_EFFECTER_ID_GPIO_HIGH_BYTE);
}

void pal_set_sys_status()
{
	set_mb_dc_status(POWER_EN_R);
	set_DC_status(PG_CARD_OK);
	set_DC_on_delayed_status();

	// Check CXL ready status if host is DC on
	if (gpio_get(PG_CARD_OK) == POWER_ON) {
		cxl_ready_handler();
	}

	set_sys_ready_pin(BIC_READY_R);
}

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);
