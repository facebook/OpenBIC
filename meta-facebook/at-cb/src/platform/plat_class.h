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

#include <stdbool.h>
#include <stdint.h>
#include "plat_i2c.h"
#include "plat_gpio.h"
#include "i2c-mux-pca954x.h"

#define ASIC_CARD_COUNT 12
#define ASIC_CARD_FRU_ADDR 0xAC
#define ASIC_CARD_1_6_MUX_ADDR 0x70
#define ASIC_CARD_7_12_MUX_ADDR 0x74
#define ASIC_CARD_DEVICE_MUX_ADDR 0x72
#define ASIC_CARD_NOT_PRESENT_VAL 0x07
#define ASIC_DEV_NOT_PRESENT_VAL 0x06
#define ASIC_DEV_1_PRESENT_VAL 0x02
#define ASIC_DEV_2_PRESENT_VAL 0x04
#define ASIC_DEV_1_2_PRESENT_VAL 0x00

#define PEX_0_BUS I2C_BUS2
#define PEX_1_BUS I2C_BUS3
#define PEX_ADDR 0x59
#define PEX_0_INDEX 0
#define PEX_1_INDEX 1
#define PEX_0_START_ACCL_ID 0
#define PEX_1_START_ACCL_ID 6
#define PEX_ACCL_DEV_PRESENT_REG_COUNT 6
#define PEX_ACCL_DEV_PRESENT_RESP_COUNT 4
#define PEX_ACCL_DEV_PRESENT_REG 0x2A080048
#define PEX_ACCL_PRESENT_MAP_VAL 0x07

#define HSC_MODULE_PIN_NUM BOARD_ID2
#define POWER_BRICK_MODULE_PIN_NUM BOARD_ID1

enum BOARD_REVISION_ID {
	POC_STAGE = 0b000,
	EVT1_STAGE = 0b001,
	EVT2_STAGE = 0b010,
	DVT_STAGE = 0b011,
	PVT_STAGE = 0b100,
	MP_STAGE = 0b101,
	UNKNOWN_STAGE = 0xFF,
};

enum HSC_MODULE {
	HSC_MODULE_ADM1272,
	HSC_MODULE_LTC4286,
	HSC_MODULE_UNKNOWN = 0xFF,
};

enum POWER_BRICK_MODULE {
	POWER_BRICK_Q50SN120A1,
	POWER_BRICK_BMR3512202,
	POWER_BRICK_UNKNOWN = 0xFF,
};

enum ASIC_CARD_STATUS {
	ASIC_CARD_NOT_PRESENT,
	ASIC_CARD_PRESENT,
	ASIC_CARD_UNKNOWN_STATUS = 0xFF,
};

enum ASIC_CARD_DEVICE_STATUS {
	ASIC_CARD_DEVICE_NOT_PRESENT,
	ASIC_CARD_DEVICE_PRESENT,
	ASIC_CARD_DEVICE_UNKNOWN_STATUS = 0xFF,
};

enum FIO_STATUS {
	FIO_NOT_PRESENT,
	FIO_PRESENT,
};

enum PCIE_DEVICE_ID {
	PCIE_DEVICE_ID1,
	PCIE_DEVICE_ID2,
	PCIE_DEVICE_ID3,
};

struct ASIC_CARD_INFO {
	uint8_t bus;
	uint8_t mux_addr;
	uint8_t mux_channel;

	bool card_status;
	uint8_t device_mux_addr;
	uint8_t device_channel;
	uint8_t device_reg_offset;
	bool asic_1_status;
	bool asic_2_status;
};

extern struct ASIC_CARD_INFO asic_card_info[ASIC_CARD_COUNT];

void check_asic_card_status();
bool get_adc_voltage(int channel, float *voltage);
void check_accl_device_presence_status(uint8_t pex_id);
void init_platform_config();
uint8_t get_board_revision();
uint8_t get_hsc_module();
uint8_t get_pwr_brick_module();

#endif
