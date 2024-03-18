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

#include <stdio.h>

#include "plat_class.h"
#include <logging/log.h>

static uint8_t hsc_module = HSC_MODULE_ADM1272; // user can change hsc module here
static uint8_t temp_module = SB_TMP461; // user can change temp module here

uint8_t get_hsc_module()
{
	return hsc_module;
}

uint8_t get_temp_module()
{
	return temp_module;
}
LOG_MODULE_REGISTER(plat_class);
