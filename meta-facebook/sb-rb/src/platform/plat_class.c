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
#include <logging/log.h>

#include "plat_class.h"
#include "plat_cpld.h"

LOG_MODULE_REGISTER(plat_class);

static uint8_t vr_module = 0;
static uint8_t ubc_module = 0;

void init_plat_config()
{
	uint8_t module = 0;
	plat_read_cpld(CPLD_OFFSET_VR_VENDER_TYPE, &module);

	vr_module = (module & 0x01);
	ubc_module = (module >> 1) & 0x03;
}

uint8_t get_vr_module()
{
	return vr_module;
}

uint8_t get_ubc_module()
{
	return ubc_module;
}