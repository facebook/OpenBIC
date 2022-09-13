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

#ifndef ADC_H
#define ADC_H

#include <stdbool.h>
#include <stdint.h>

enum {
	ADC_PORT0 = 0,
	ADC_PORT1,
	ADC_PORT2,
	ADC_PORT3,
	ADC_PORT4,
	ADC_PORT5,
	ADC_PORT6,
	ADC_PORT7,
	ADC_PORT8,
	ADC_PORT9,
	ADC_PORT10,
	ADC_PORT11,
	ADC_PORT12,
	ADC_PORT13,
	ADC_PORT14,
	ADC_PORT15,
};

int bat_3v_set_gpio(uint8_t sensor_num, void *arg);
bool adc_init();
bool adc_sensor_read(uint8_t sensor_num, float *reading);

#endif
