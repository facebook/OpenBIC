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

#ifndef PLAT_GPIO_H
#define PLAT_GPIO_H

#include "hal_gpio.h"

// gpio_cfg(chip, number, is_init, direction, status, int_type, int_callback)
// dedicate gpio A0~A7, B0~B7, C0~C7, D0~D7, E0~E7, total 40 gpios
// Default name: Reserve_GPIOH0

// clang-format off

#define name_gpioA \
	gpio_name_to_num(Reserve_GPIOA0) \
	gpio_name_to_num(Reserve_GPIOA1) \
	gpio_name_to_num(Reserve_GPIOA2) \
	gpio_name_to_num(PWRGD_CPU_LVC3) \
	gpio_name_to_num(Reserve_GPIOA4) \
	gpio_name_to_num(Reserve_GPIOA5) \
	gpio_name_to_num(Reserve_GPIOA6) \
	gpio_name_to_num(Reserve_GPIOA7)

// clang-format on

#define gpio_name_to_num(x) x,
enum _GPIO_NUMS_ {
	name_gpioA
};

extern enum _GPIO_NUMS_ GPIO_NUMS;
#undef gpio_name_to_num

extern char *gpio_name[];

#endif