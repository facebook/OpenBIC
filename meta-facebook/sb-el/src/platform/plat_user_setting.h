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

#include "plat_cpld.h"

#define VR_MUTEX_LOCK_TIMEOUT_MS 1000
#define TEMP_THRESHOLD_USER_SETTINGS_OFFSET 0x8100
#define VR_VOUT_USER_SETTINGS_OFFSET 0x8000
#define ALERT_LEVEL_USER_SETTINGS_OFFSET 0x8200
#define DELAY_PCIE_PERST_USER_SETTINGS_OFFSET 0x8300
#define BOOTSTRAP_USER_SETTINGS_OFFSET 0x8400
#define THERMALTRIP_USER_SETTINGS_OFFSET 0x8500
#define THROTTLE_USER_SETTINGS_OFFSET 0x8600
#define DELAY_ASIC_RST_USER_SETTINGS_OFFSET 0x8700
#define DELAY_MODULE_PG_USER_SETTINGS_OFFSET 0x8800

#define CPLD_THROTTLE_SWITCH_ADDR 0x25
#define CPLD_THERMALTRIP_SWITCH_ADDR 0x3A

void user_settings_init(void);