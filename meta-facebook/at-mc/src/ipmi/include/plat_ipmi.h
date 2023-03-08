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

#ifndef PLAT_IPMI_H
#define PLAT_IPMI_H

#include <stdint.h>
#include <stdbool.h>

#define BIC_FW_DATA_LENGTH 7
#define PCIE_CARD_NOT_PRESENT 0
#define PCIE_CARD_PRESENT 1
#define PCIE_CARD_ID_OFFSET 30

#define PCIE_CARD_NOT_PRESENT_BIT BIT(0)
#define PCIE_CARD_NOT_ACCESSIABLE_BIT BIT(1)
#define PCIE_CARD_DEVICE_NOT_READY_BIT BIT(2)

#define IS_SECTOR_END_MASK 0x80
#define WITHOUT_SENCTOR_END_MASK 0x7F
#define BIC_UPDATE_MAX_OFFSET 0x50000
#define PM8702_UPDATE_MAX_OFFSET 0x150000
#define PM8702_INITIATE_FW_OFFSET 0x0000

#define PM8702_DEFAULT_NEXT_ACTIVE_SLOT 0
#define PM8702_NO_HBO_RUN_VAL 0
#define PM8702_TRANSFER_FW_DATA_LEN 128
#define PM8702_RETURN_SUCCESS 0x00
#define PM8702_BUSY_DELAY_MS 100
#define PM8702_TRANSFER_DELAY_MS 20
#define PM8702_TRANSFER_FW_HEADER_LEN 128

/** enum number follow GT for now since bmc hasn't ready **/
enum MC_FIRMWARE_COMPONENT {
	MC_COMPNT_BIC = 2,
	MC_COMPNT_CPLD = 7,
	MC_COMPNT_CXL1 = 16,
	MC_COMPNT_CXL2,
	MC_COMPNT_CXL3,
	MC_COMPNT_CXL4,
	MC_COMPNT_CXL5,
	MC_COMPNT_CXL6,
	MC_COMPNT_CXL7,
	MC_COMPNT_CXL8,
	MC_COMPNT_MAX,
};

enum CXL_FRU_OPTIONAL {
	CXL_FRU_WRITE,
	CXL_FRU_READ,
};

enum PCIE_CARD_SENSOR_READING_TYPE {
	PCIE_CARD_E1S,
	PCIE_CARD_CXL,
};

#endif
