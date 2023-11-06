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

#ifndef PEX_89000_H
#define PEX_89000_H

#include <stdint.h>
#include "sensor.h"

typedef enum pex_dev { pex_dev_atlas1, pex_dev_atlas2, pex_dev_unknown } pex_dev_t;

typedef enum pex_access {
	pex_access_temp,
	pex_access_adc,
	pex_access_id,
	pex_access_rev_id,
	pex_access_sbr_ver,
	pex_access_flash_ver,
	pex_access_register,
	pex_access_ccr_system_error,
	pex_access_unknown
} pex_access_t;

enum pex_api_ret {
	pex_api_success,
	pex_api_unspecific_err,
	pex_api_mutex_err,
};

/* sensor offset */
enum pex_sensor_offset {
	PEX_TEMP,
	PEX_ADC,
};

typedef struct {
	uint8_t idx; // Create index based on init variable
	struct k_mutex mutex;
	pex_dev_t pex_type;
	sys_snode_t node; // linked list node
} pex89000_unit;

enum PEX_CCR_SYSTEM_ERROR_STATUS {
	PEX_SYSTEM_ERROR = BIT(0),
	PEX_FATAL_ERROR = BIT(1),
	PEX_POR_BISR_TIMEOUT = BIT(4),
	PEX_ARM_FLASH_SIGNATURE_FAIL = BIT(5),
	PEX_WDT0_CPU_RESET = BIT(6),
	PEX_WDT0_SYSTEM_RESET = BIT(7),
	PEX_WDT1_CPU_RESET = BIT(8),
	PEX_WDT1_SYSTEM_RESET = BIT(9),
	PEX_LOCAL_CPU_PARITY_ERROR = BIT(15),
	PEX_SECURE_BOOT_FAIL = BIT(16),
	PEX_SBR_LOAD_FAIL = BIT(18),
	PEX_STATION_0_FATAL_ERROR = BIT(20),
	PEX_STATION_1_FATAL_ERROR = BIT(21),
	PEX_STATION_2_FATAL_ERROR = BIT(22),
	PEX_STATION_3_FATAL_ERROR = BIT(23),
	PEX_STATION_4_FATAL_ERROR = BIT(24),
	PEX_STATION_5_FATAL_ERROR = BIT(25),
	PEX_STATION_6_FATAL_ERROR = BIT(26),
	PEX_STATION_7_FATAL_ERROR = BIT(27),
	PEX_STATION_8_FATAL_ERROR = BIT(28),
	PEX_PSB_STATION_FATAL_ERROR = BIT(31)
};

/* Note: Could be used only after pex89000 sensor init successed */
uint8_t pex_access_engine(uint8_t bus, uint8_t addr, uint8_t idx, pex_access_t key, uint32_t *resp);
uint8_t pex89000_init(sensor_cfg *cfg);

#endif
