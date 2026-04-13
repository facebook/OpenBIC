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

#ifndef _PLAT_HAMSA_AVDD_PCIE_H_
#define _PLAT_HAMSA_AVDD_PCIE_H_

#include "plat_vr_test_mode.h"

typedef struct {
	uint8_t vr_rail;
	uint16_t uvp;
	uint16_t ovp;
	uint16_t vout_max;
	uint16_t lcr;
	uint16_t ucr;
} vr_rns_hamsa_avdd_pcie_mode_setting_t;

typedef struct {
	uint8_t vr_rail;
	uint16_t lcr;
	uint16_t ucr;
	uint16_t vout_max;
} vr_mps_hamsa_avdd_pcie_mode_setting_t;

extern const vr_rns_hamsa_avdd_pcie_mode_setting_t vr_rns_hamsa_avdd_pcie_mode_table[];
extern const uint8_t vr_rns_hamsa_avdd_pcie_mode_table_size;

extern const vr_mps_hamsa_avdd_pcie_mode_setting_t vr_mps_hamsa_avdd_pcie_mode_table[];
extern const uint8_t vr_mps_hamsa_avdd_pcie_mode_table_size;

bool set_hamsa_avdd_pcie(uint16_t *millivolt, bool is_perm);

#endif
