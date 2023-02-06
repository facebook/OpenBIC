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

#include <drivers/espi.h>

#define VW_GPIO_ENABLE true
#define VW_GPIO_DISABLE false

enum vw_gpio_direction {
	VW_GPIO_INPUT = 0,
	VW_GPIO_OUTPUT,
};

enum vw_gpio_value {
	VW_GPIO_LOW = 0,
	VW_GPIO_HIGH,
};

typedef struct _vw_gpio_ {
	uint8_t number;
	bool is_enabled;
	uint8_t direction;
	uint8_t value;
	void (*int_cb)(uint8_t value);
} vw_gpio;

bool vw_gpio_get(int number, uint8_t *value);
bool vw_gpio_set(int number, uint8_t value);
bool vw_gpio_init(vw_gpio *config, uint8_t size);
