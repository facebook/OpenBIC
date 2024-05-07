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

#ifndef _PLDM_SMBIOS_H
#define _PLDM_SMBIOS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pldm.h"

// Size of the largest SMBIOS structure, DSP0134_3.6.0
#define MAXIMUM_STRUCTURE_SIZE 65535
// For version 2.1 and later, handle values are in the range 0 to 0xFEFFh.
#define MAXIMUM_HANDLE_NUM 0xFEFF

typedef enum {
	PLDM_SMBIOS_CMD_CODE_GET_SMBIOS_STRUCTURE_BY_TYPE = 0x05,
} pldm_smbios_commands;

typedef enum {
	/* GetSMBIOSStructureByType */
	PLDM_SMBIOS_INVALID_DATA_TRANSFER_HANDLE = 0x80,
	PLDM_SMBIOS_INVALID_TRANSFER_OPERATION_FLAG = 0x81,
	PLDM_SMBIOS_NO_SMBIOS_STRUCTURES = 0x86,
	PLDM_SMBIOS_INVALID_SMBIOS_STRUCTURE_TYPE = 0x87,
	PLDM_SMBIOS_INVALID_SMBIOS_STRUCTURE_INSTANCE_ID = 0x89,
} pldm_smbios_completion_codes;

typedef enum {
	PLDM_SMBIOS_TRANSFER_FLAG_START = 0x01,
	PLDM_SMBIOS_TRANSFER_FLAG_MIDDLE = 0x02,
	PLDM_SMBIOS_TRANSFER_FLAG_END = 0x04,
	PLDM_SMBIOS_TRANSFER_FLAG_START_AND_END = 0x05
} pldm_smbios_transfer_flag;

typedef enum {
	PLDM_SMBIOS_TRANSFER_OPERATION_FLAG_GET_NEXT_PART = 0x00,
	PLDM_SMBIOS_TRANSFER_OPERATION_FLAG_GET_FIRST_PART = 0x01
} pldm_smbios_transfer_operation_flag;

// Based on DSP0134 v3.6.0
typedef enum {
	SMBIOS_BIOS_INFORMATION = 0,
} smbios_structure_types;

typedef struct {
	uint8_t type;
	uint8_t length;
	uint16_t handle;
} __attribute__((packed)) smbios_structure_header;

typedef struct {
	smbios_structure_header header;
	uint8_t vendor;
	uint8_t bios_version;
	uint16_t bios_starting_address_segment;
	uint8_t bios_release_date;
	uint8_t bios_rom_size;
	uint64_t bios_characteristics;
	uint16_t bios_characteristics_extension_bytes;
	uint8_t system_bios_major_release;
	uint8_t system_bios_minor_release;
	uint8_t embedded_controller_firmware_major_release;
	uint8_t embedded_controller_firmware_minor_release;
	uint16_t extended_bios_rom_size;
	char *text_strings;
} __attribute__((packed)) smbios_bios_information;

typedef struct {
	uint32_t data_transfer_handle;
	uint8_t transfer_operation_flag;
	uint8_t type;
	uint16_t structure_instance_id;
} __attribute__((packed)) pldm_get_smbios_structure_by_type_req;

typedef struct {
	uint8_t completion_code;
	uint32_t next_data_transfer_handle;
	uint8_t transfer_flag;
} __attribute__((packed)) pldm_get_smbios_structure_by_type_resp;

void pldm_smbios_init_structures();

uint8_t pldm_smbios_handler_query(uint8_t code, void **ret_fn);

uint8_t pldm_smbios_get_text_strings_count(char *text_strings);

uint8_t pldm_smbios_get_text_strings_size(char *text_strings);

int pldm_smbios_set_bios_information(smbios_bios_information *new_bios_information);

const char *pldm_smbios_get_bios_version();

#ifdef __cplusplus
}
#endif

#endif /*_PLDM_SMBIOS_H*/
