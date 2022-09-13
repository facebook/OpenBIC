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

#include "plat_class.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hal_gpio.h"
#include "hal_i2c.h"
#include "libutil.h"
#include "plat_gpio.h"
#include "plat_i2c.h"

static uint8_t system_class = SYS_CLASS_1;

uint8_t get_system_class()
{
	return system_class;
}

void init_platform_config()
{
	if (gpio_get(SYS_SKU_ID0) == GPIO_HIGH)
		system_class = SYS_CLASS_2;
	else
		system_class = SYS_CLASS_1;

	printf("SYS_SKU: %s Compute System\n", system_class == SYS_CLASS_2 ? "Single" : "Dual");
}
