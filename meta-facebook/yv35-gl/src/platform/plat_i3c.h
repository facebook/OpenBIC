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

#ifndef PLAT_I3C_H
#define PLAT_I3C_H

#include "hal_i3c.h"

#define RSTDAA_COUNT 2
#define LDO_VOLT                                                                                   \
	V_LDO_SETTING(rg3mxxb12_ldo_1_0_volt, rg3mxxb12_ldo_1_8_volt, rg3mxxb12_ldo_1_2_volt,      \
		      rg3mxxb12_ldo_1_2_volt)
#define DEFAULT_SLAVE_PORT_SETTING 0x3F

void init_i3c_hub();

#endif
