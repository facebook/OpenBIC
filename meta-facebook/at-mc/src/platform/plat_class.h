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

#ifndef PLAT_CLASS_H
#define PLAT_CLASS_H

#include "hal_i2c.h"

#define CPLD_BUS I2C_BUS10
#define CPLD_ADDR (0xA0 >> 1)

#define E1S_PRESENT 0x00
#define E1S_NOT_PRESENT 0x01
#define E1S_0_1_PRESENT 0x03
#define E1S_0_PRESENT 0x07
#define NIC_PRESENT 0x09
#define E1S_1_PRESENT 0x0B
#define CXL_PRESENT 0x0E
#define NO_DEVICE_PRESENT 0x0F

enum CARD_INFO_INDEX {
	CARD_1_INDEX,
	CARD_2_INDEX,
	CARD_3_INDEX,
	CARD_4_INDEX,
	CARD_5_INDEX,
	CARD_6_INDEX,
	CARD_7_INDEX,
	CARD_8_INDEX,
	CARD_9_INDEX,
	CARD_10_INDEX,
	CARD_11_INDEX,
	CARD_12_INDEX,
	CARD_13_INDEX,
	CARD_14_INDEX,
};

enum PCIE_DEVICE_ID {
	PCIE_DEVICE_ID1,
	PCIE_DEVICE_ID2,
	PCIE_DEVICE_ID3,
};

enum PCIE_CARD_TYPE {
	E1S_CARD,
	E1S_0_CARD,
	E1S_1_CARD,
	E1S_0_1_CARD,
	NIC_CARD,
	CXL_CARD,
	CARD_NOT_PRESENT,
	UNKNOWN_CARD = 0xFF,
};

void check_pcie_card_type();
uint8_t prsnt_status_to_card_type(uint8_t presence_status);
int get_pcie_card_type(uint8_t card_id, uint8_t *card_type);
int get_pcie_device_type(uint8_t card_id, uint8_t device_id, uint8_t *device_type);
int pcie_card_id_to_cxl_e1s_id(uint8_t pcie_card_id, uint8_t *dev_id);

#endif
