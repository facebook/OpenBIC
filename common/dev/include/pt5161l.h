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

#ifndef PT5161L_H
#define PT5161L_H

#include "hal_i2c.h"

#define PT5161L_I2C_MST_INIT_CTRL_BIT_BANG_MODE_EN_GET(x) (((x) & 0x04) >> 2)
#define PT5161L_I2C_MST_INIT_CTRL_BIT_BANG_MODE_EN_SET(x) (((x) << 2) & 0x04)
#define PT5161L_I2C_MST_INIT_CTRL_BIT_BANG_MODE_EN_MODIFY(r, x) ((((x) << 2) & 0x04) | ((r) & 0xfb))

#define PT5161L_SELF_WR_CSR_CMD 1
#define PT5161L_MAIN_MICRO_FW_INFO (96 * 1024 - 128)
#define PT5161L_MM_FW_VERSION_MAJOR 0
#define PT5161L_MM_FW_VERSION_MINOR 1
#define PT5161L_MM_FW_VERSION_BUILD 2

//Main Micro codes for writing and reading EEPROM via MM-assist
#define PT5161L_MM_EEPROM_WRITE_REG_CODE 1
#define PT5161L_MM_EEPROM_WRITE_END_CODE 2
#define PT5161L_MM_EEPROM_READ_END_CODE 3
#define PT5161L_MM_EEPROM_READ_REG_CODE 4
#define PT5161L_MM_STATUS_TIME_5MS 5
#define PT5161L_MM_STATUS_TIME_10MS 10
#define PT5161L_MM_EEPROM_ASSIST_CMD_ADDR 0x920
#define PT5161L_EEPROM_BLOCK_BASE_ADDR 0x88e7
#define PT5161L_EEPROM_BLOCK_CMD_MODIFIER 0x80
#define PT5161L_EEPROM_BLOCK_WRITE_SIZE 32
#define PT5161L_EEPROM_PAGE_SIZE 256

#define PT5161L_MAIN_SRAM_DMEM_OFFSET (64 * 1024)
#define PT5161L_TG_RD_LOC_IND_SRAM 0x16
#define PT5161L_TEMP_OFFSET 0x42c
/** EEPROM ic cmd reg offset*/
#define PT5161L_I2C_MST_IC_CMD_ADDR 0xd04
/** EEPROM data0 reg offset*/
#define PT5161L_I2C_MST_DATA0_ADDR 0xd05
/** EEPROM data1 reg offset*/
#define PT5161L_I2C_MST_DATA1_ADDR 0xd06
/** EEPROM data2 reg offset*/
#define PT5161L_I2C_MST_DATA2_ADDR 0xd07
/** EEPROM data3 reg offset*/
#define PT5161L_I2C_MST_DATA3_ADDR 0xd08
/** EEPROM cmd reg offset*/
#define PT5161L_I2C_MST_CMD_ADDR 0xd09
#define PT5161L_I2C_MST_BB_OUTPUT_ADDRESS 0xd0b
#define PT5161L_I2C_MST_INIT_CTRL_ADDRESS 0xd0a
#define PT5161L_MAIN_MICRO_INDIRECT 0xd99

#define PT5161L_MUTEX_LOCK_MS 1000
#define PCIE_RETIMER_UPDATE_MAX_OFFSET 0x40000

#define PT5161L_VENDOR_ID_LENGTH 7
extern uint8_t PT5161L_VENDOR_ID[7];

bool pt5161l_get_vendor_id(I2C_MSG *msg);
bool get_retimer_fw_version(I2C_MSG *msg, uint8_t *version);
uint8_t pcie_retimer_fw_update(I2C_MSG *msg, uint32_t offset, uint16_t msg_len, uint8_t *msg_buf,
			       uint8_t flag);

#endif
