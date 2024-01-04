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

#ifndef HAL_PECI_H
#define HAL_PECI_H

#define DEBUG_PECI 0

/* Completion Code mask to check retry needs */
#define PECI_DEV_CC_RETRY_CHECK_MASK 0xf0
#define PECI_DEV_CC_NEED_RETRY 0x80
#define IS_PECI_CC_NEED_RETRY(cc)                                                                  \
	(((cc) & PECI_DEV_CC_RETRY_CHECK_MASK) == PECI_DEV_CC_NEED_RETRY) ? true : false
#define PECI_DEV_RETRY_BIT 0x01
#define PECI_DEV_RETRY_TIMEOUT 700 // ms
#define PECI_DEV_RETRY_INTERVAL_MAX_MSEC 100
#define PECI_DEV_RETRY_INTERVAL_MIN_MSEC 1

enum peci_cmd {
	PECI_PING_CMD = 0x00,
	PECI_GET_TEMP0_CMD = 0x01,
	PECI_GET_TEMP1_CMD = 0x02,
	PECI_RD_PCI_CFG0_CMD = 0x61,
	PECI_RD_PCI_CFG1_CMD = 0x62,
	PECI_WR_PCI_CFG0_CMD = 0x65,
	PECI_WR_PCI_CFG1_CMD = 0x66,
	PECI_CRASHDUMP_CMD = 0x71,
	PECI_RD_PKG_CFG0_CMD = 0xA1,
	PECI_RD_PKG_CFG1_CMD = 0xA,
	PECI_WR_PKG_CFG0_CMD = 0xA5,
	PECI_WR_PKG_CFG1_CMD = 0xA6,
	PECI_RD_IAMSR0_CMD = 0xB1,
	PECI_RD_IAMSR1_CMD = 0xB2,
	PECI_WR_IAMSR0_CMD = 0xB5,
	PECI_WR_IAMSR1_CMD = 0xB6,
	PECI_RD_ENDPOINT_CFG_CMD = 0xC1,
	PECI_WR_ENDPOINT_CFG_CMD = 0xC5,
	PECI_RD_PCI_CFG_LOCAL0_CMD = 0xE1,
	PECI_RD_PCI_CFG_LOCAL1_CMD = 0xE2,
	PECI_WR_PCI_CFG_LOCAL0_CMD = 0xE5,
	PECI_WR_PCI_CFG_LOCAL1_CMD = 0xE6,
	PECI_GET_DIB_CMD = 0xF7,
};

int peci_init();
int peci_ping(uint8_t address);
int peci_read(uint8_t cmd, uint8_t address, uint8_t u8Index, uint16_t u16Param, uint8_t u8ReadLen,
	      uint8_t *readBuf);
int peci_write(uint8_t cmd, uint8_t address, uint8_t u8ReadLen, uint8_t *readBuf,
	       uint8_t u8WriteLen, uint8_t *writeBuf);
bool peci_retry_read(uint8_t cmd, uint8_t address, uint8_t u8Index, uint16_t u16Param,
		     uint8_t u8ReadLen, uint8_t *readBuf);

#endif
