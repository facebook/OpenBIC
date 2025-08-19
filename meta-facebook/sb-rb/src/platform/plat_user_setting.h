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

#ifndef PLAT_USER_SETTING_H
#define PLAT_USER_SETTING_H

#include "sensor.h"
#include "plat_pldm_sensor.h"
#include "plat_cpld.h"

#define VR_MUTEX_LOCK_TIMEOUT_MS 1000

#define THERMALTRIP_USER_SETTINGS_OFFSET 0x8500

#define CPLD_THERMALTRIP_SWITCH_ADDR 0x3A
enum USER_SETTING_OFFSET_E {
	THERMALTRIP,
};

typedef struct thermaltrip_user_settings_struct {
	uint8_t thermaltrip_user_setting_value;
} thermaltrip_user_settings_struct;

bool set_thermaltrip_user_settings(bool thermaltrip_enable, bool is_perm);
#endif
