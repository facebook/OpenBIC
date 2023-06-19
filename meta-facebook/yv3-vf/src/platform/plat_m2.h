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

#include <stdbool.h>

#define PREFIX_MASK 0xF8

#define PREFIX_M2A 0x60
#define PREFIX_M2B 0x68
#define PREFIX_M2C 0x70
#define PREFIX_M2D 0x78

#define DEV_PWR_ON 0x01
#define DEV_PWR_CTRL 0x02
#define DEV_PRSNT_SET 0x04
#define DEV_PCIE_RST 0x08
#define DEV_CHK_DISABLE 0x20
#define DEV_FORCE_3V3 0x40
#define DEV_PWRDIS_EN 0x80

enum M2_IDX_E {
	M2_IDX_E_A = 0,
	M2_IDX_E_B,
	M2_IDX_E_C,
	M2_IDX_E_D,
	M2_IDX_E_MAX,
};

uint8_t m2_bus2idx(uint8_t bus);
uint8_t m2_bus2rst(uint8_t bus);
uint8_t m2_idx2bus(uint8_t idx);
uint8_t m2_idx2sensornum(uint8_t idx);
uint8_t m2_sensornum2idx(uint8_t sensor_num);
uint8_t m2_pwrgd(uint8_t idx);
uint8_t m2_get_prefix_sen_num(uint8_t idx);
uint8_t m2_prsnt(uint8_t idx);
uint8_t rst_edsff(uint8_t idx, uint8_t val);
bool is_m2_sen_readable(uint8_t sen_num);
bool is_nvme_temp_readable(uint8_t sen_num);
uint8_t exchange_m2_idx(uint8_t idx);