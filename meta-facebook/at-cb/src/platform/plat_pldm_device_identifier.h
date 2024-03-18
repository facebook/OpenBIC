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

#ifndef _PLAT_PLDM_DEVICE_IDENTIFIER_H_
#define _PLAT_PLDM_DEVICE_IDENTIFIER_H_

#include "pldm_firmware_update.h"

extern const uint8_t bic_descriptors_count;
extern const uint8_t asic_psoc_descriptors_count;
extern const uint8_t asic_qspi_descriptors_count;
extern const uint8_t asic_boot1_descriptors_count;

extern const uint8_t downstream_table_count;

extern struct pldm_descriptor_string PLDM_DEVICE_DESCRIPTOR_TABLE[];
extern struct pldm_descriptor_string ASIC_PSOC_DESCRIPTOR_TABLE[];
extern struct pldm_descriptor_string ASIC_QSPI_DESCRIPTOR_TABLE[];
extern struct pldm_descriptor_string ASIC_BOOT1_DESCRIPTOR_TABLE[];

extern struct pldm_downstream_identifier_table downstream_table[];

#endif
