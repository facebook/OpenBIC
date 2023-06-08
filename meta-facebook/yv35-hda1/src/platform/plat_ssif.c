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

#include <stdint.h>
#include <logging/log.h>
#include "plat_ssif.h"
#include "plat_i2c.h"
#include "plat_gpio.h"
#include "ssif.h"

LOG_MODULE_REGISTER(plat_ssif);
struct ssif_init_cfg ssif_cfg_table[] = {
	{ SSIF_I2C_BUS, SSIF_I2C_ADDR, 0x0A },
};

void pal_ssif_alert_trigger(uint8_t status)
{
	LOG_DBG("trigger %d", status);
	gpio_set(BIC_SALT12_L, status);
}

void ssif_init(void)
{
	ssif_device_init(ssif_cfg_table, ARRAY_SIZE(ssif_cfg_table));

	if (ssif_inst_get_by_bus(SSIF_I2C_BUS))
		gpio_set(BMC_GPIOC3_OK, GPIO_HIGH);
}
