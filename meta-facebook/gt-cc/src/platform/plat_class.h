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

#include <stdbool.h>
#include <stdint.h>

/* ADC channel number */
enum GT_COMPONENT_TYPE_ADC_CHANNEL {
	HSC_TYPE_ADC_CHANNEL = 11,
	VR_TYPE_ADC_CHANNEL = 12,
	POWER_IC_TYPE_ADC_CHANNEL = 13,
};

typedef enum {
	EVT = 1,
	EVT2 = 2,
	DVT = 3,
	DVT2 = 4,
} GT_STAGE_REVISION_ID;

bool get_adc_voltage(int channel, float *voltage);
GT_STAGE_REVISION_ID get_stage_by_rev_id();

#endif