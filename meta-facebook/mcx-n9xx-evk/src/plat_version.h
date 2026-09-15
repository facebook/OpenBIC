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

#define PLATFORM_NAME "NXP MCX-N9XX-EVK"
#define PROJECT_NAME "Minimal Bring-up"

#define BIC_FW_YEAR_MSB 0x20
#define BIC_FW_YEAR_LSB 0x26
#define BIC_FW_WEEK 0x34
#define BIC_FW_VER 0x00

/* IPMI Get Device ID (app_handler.c) response fields - this board has
 * no real product/FRU catalog, so these are placeholder-but-honest
 * values (0 = "not assigned"), same convention every real OpenBIC
 * board uses for fields it hasn't been given real IDs for. */
#define DEVICE_ID 0x00
#define DEVICE_REVISION 0x80 /* bit7=1: device provides Device SDRs */
#define FIRMWARE_REVISION_1 (BIC_FW_YEAR_LSB & 0x7F)
#define FIRMWARE_REVISION_2 BIC_FW_WEEK
#define IPMI_VERSION 0x02 /* IPMI v2.0 */
#define ADDITIONAL_DEVICE_SUPPORT 0xBF
#define PRODUCT_ID 0x0000
#define AUXILIARY_FW_REVISION 0x00000000

#endif
