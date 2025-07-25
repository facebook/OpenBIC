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

#include "stdint.h"

enum VR_MODULE {
	VR_MODULE_MPS,
	VR_MODULE_RNS,
	VR_MODULE_UNKNOWN,
};

enum UBC_MODULE {
	UBC_MODULE_DELTA,
	UBC_MODULE_MPS,
	UBC_MODULE_FLEX,
	UBC_MODULE_LUXSHARE,
	UBC_MODULE_UNKNOWN,
};

void init_plat_config();
uint8_t get_vr_module();
uint8_t get_ubc_module();
uint8_t get_mmc_slot();

#endif
