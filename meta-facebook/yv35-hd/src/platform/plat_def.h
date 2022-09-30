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

#ifndef PLAT_DEF_H
#define PLAT_DEF_H

#include "plat_i2c.h"

#define HOST_KCS_PORT kcs3
#define BMC_USB_PORT "CDC_ACM_0"

#define ENABLE_APML
#define APML_BUS I2C_BUS14
#define SB_RMI_ADDR 0x3C
#define SB_TSI_ADDR 0x4C
#define TSI_HIGH_TEMP_THRESHOLD 0x5F

#endif
