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

#ifndef PLAT_FRU_H
#define PLAT_FRU_H

#include "eeprom.h"
#include "plat_i2c.h"

enum {
	MB_FRU_ID,
	// OTHER_FRU_ID,
	MAX_FRU_ID,
};

#define FRU_CFG_NUM MAX_FRU_ID

#define MB_FRU_PORT I2C_BUS10
#define MB_FRU_ADDR 0x50

#define BIOS_FW_VERSION_START 0x0A00
#define BIOS_FW_VERSION_MAX_SIZE 34
#define BIOS_FW_VERSION_BLOCK_NUM 2
#define BIOS_FW_VERSION_SECOND_BLOCK_OFFSET 17
#define BIOS_FW_VERSION_BLOCK_MAX_SIZE 17

#define VR_RM_CNT_MAX_SIZE 2
#define VR_MPS_CPUDVDD_RM_CNT_START 0x0B00
#define VR_MPS_CPUVDD_RM_CNT_START (VR_MPS_CPUDVDD_RM_CNT_START + VR_RM_CNT_MAX_SIZE)
#define VR_MPS_SOCVDD_RM_CNT_START (VR_MPS_CPUDVDD_RM_CNT_START + (VR_RM_CNT_MAX_SIZE * 2))
#define VR_MPS_FBVDDP2_RM_CNT_START (VR_MPS_CPUDVDD_RM_CNT_START + (VR_RM_CNT_MAX_SIZE * 3))
#define VR_MPS_1V2_RM_CNT_START (VR_MPS_CPUDVDD_RM_CNT_START + (VR_RM_CNT_MAX_SIZE * 4))
#define VR_MPS_MAX_RM_CNT 1000 //Approximate number from MPS vendor

#define RTC_CLR_REC_MAX_SIZE 1
#define RTC_CLR_REC_START 0x0C00

#define RTC_CLR_ASSERT 0x00
#define RTC_CLR_DEASSERT 0xFF

bool get_bios_version_area_config(EEPROM_CFG *config);
int set_bios_version(EEPROM_ENTRY *entry, uint8_t block_index);
int get_bios_version(EEPROM_ENTRY *entry, uint8_t block_index);
bool access_vr_remain_cnt(EEPROM_ENTRY *entry, uint8_t comp_id, bool update_flag);
bool access_rtc_clr_flag(uint8_t val);
void handle_rtc_clr_flag();

#endif
