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

#ifndef PLAT_VERSION_H
#define PLAT_VERSION_H

#include "version.h"

#define PLATFORM_NAME "AALC"
#define PROJECT_NAME "AALC"
#define PROJECT_STAGE DVT

/*
 * 0xFF AALC TBD
 */
#define BOARD_ID 0x00
#define DEVICE_ID 0x00
#define DEVICE_REVISION 0x80

#define FIRMWARE_REVISION_1 GET_FW_VERSION1(BOARD_ID, PROJECT_STAGE)
#define FIRMWARE_REVISION_2 0x07

#define IPMI_VERSION 0x02
#define ADDITIONAL_DEVICE_SUPPORT 0xBF
#define PRODUCT_ID 0x0000
#define AUXILIARY_FW_REVISION 0x00000000
#define IANA_ID 0x009C9C // same as TI BIC
#define IANA_ID2 0x00A015 // for OEM 1S command supports YV3.5

#define BIC_FW_YEAR_MSB 0x20
#define BIC_FW_YEAR_LSB 0x25
#define BIC_FW_WEEK 0x04
#define BIC_FW_VER 0x01
#define BIC_FW_platform_0 0x61 // char: a
#define BIC_FW_platform_1 0x61 // char: a
#define BIC_FW_platform_2 0x00 // char: '\0'

#define FAN_TABLE_REVISION 2

#endif
