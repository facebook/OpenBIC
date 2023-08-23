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

#ifndef PLAT_PLDM_MONITOR_H
#define PLAT_PLDM_MONITOR_H

#define PLDM_PLATFORM_HOST_PWR_CTRL_DEFAULT 0xFF
#define PLDM_PLATFORM_HOST_PWR_BTN_LOW 0xFE
#define PLDM_PLATFORM_HOST_RST_BTN_LOW 0xFD

void plat_pldm_assign_gpio_effecter_id();

#endif
