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

enum {
	SWB_FRU_ID,
	FIO_FRU_ID,
	HSC_MODULE_FRU_ID,
	NIC0_FRU_ID,
	NIC1_FRU_ID,
	NIC2_FRU_ID,
	NIC3_FRU_ID,
	NIC4_FRU_ID,
	NIC5_FRU_ID,
	NIC6_FRU_ID,
	NIC7_FRU_ID,
	// OTHER_FRU_ID,
	MAX_FRU_ID,
};

enum GT_NIC_CONFIG {
	NIC_CONFIG_UNKNOWN = 0,
	NIC_CONFIG_CX7 = 1,
	NIC_CONFIG_IB_CX7 = 2,
	NIC_CONFIG_THOR2 = 3,
};

#define FRU_CFG_NUM MAX_FRU_ID

#define SWB_FRU_PORT 0x05
#define SWB_FRU_ADDR (0xA8 >> 1)
#define SWB_FRU_MUX_ADDR (0xE0 >> 1)
#define SWB_FRU_MUX_CHAN 7
#define FIO_FRU_PORT 0x04
#define FIO_FRU_ADDR (0xA2 >> 1)
#define HSC_MODULE_FRU_PORT 0x05
#define HSC_MODULE_FRU_ADDR (0xA2 >> 1)
#define HSC_MODULE_FRU_MUX_ADDR (0xE0 >> 1)
#define HSC_MODULE_FRU_MUX_CHAN 6

#define NIC_FRU_ADDR (0xA0 >> 1)
#define NIC0_FRU_PORT 0x00
#define NIC1_FRU_PORT 0x01
#define NIC2_FRU_PORT 0x02
#define NIC3_FRU_PORT 0x03
#define NIC4_FRU_PORT 0x0A
#define NIC5_FRU_PORT 0x0B
#define NIC6_FRU_PORT 0x0C
#define NIC7_FRU_PORT 0x0D

typedef struct {
	char product_manufacturer[32];
	char product_name[32];
	char product_part_number[32];
	char product_version[32];
} ProductInfo;

uint8_t check_nic_type_by_fru();
bool get_first_nic_manufacturer(char *manufacturer, uint8_t manufacturer_size);

#endif
