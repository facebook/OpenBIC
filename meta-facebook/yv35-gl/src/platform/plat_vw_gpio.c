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

#include "plat_vw_gpio.h"
#include "plat_isr.h"

vw_gpio plat_vw_gpio_cfg[] = {
	{ 0, VW_GPIO_ENABLE, VW_GPIO_INPUT, VW_GPIO_UNKNOWN, ISR_POST_COMPLETE },
	{ 2, VW_GPIO_ENABLE, VW_GPIO_INPUT, VW_GPIO_UNKNOWN, ISR_FM_ADR_MODE0 },
};

bool pal_load_vw_gpio_config(void)
{
	return vw_gpio_init(plat_vw_gpio_cfg, ARRAY_SIZE(plat_vw_gpio_cfg));
};
