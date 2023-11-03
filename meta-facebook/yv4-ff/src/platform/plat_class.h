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

#ifndef PLAT_CLASS_H
#define PLAT_CLASS_H

#include <stdio.h>

#define AST1030_ADC_BASE_ADDR 0x7e6e9000

// ADC channel number
enum ADC_CHANNEL {
	CHANNEL_5 = 5,
	NUMBER_OF_ADC_CHANNEL = 16,
};

struct ADC_INFO {
	long unsigned int offset;
	int shift;
};

bool get_adc_voltage(int channel, float *voltage);

#endif
