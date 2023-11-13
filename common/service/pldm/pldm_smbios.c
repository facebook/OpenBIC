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

#include <logging/log.h>
#include <string.h>
#include <stdlib.h>

#include "plat_def.h"
#include "libutil.h"
#include "pldm.h"
#include "pldm_smbios.h"

LOG_MODULE_DECLARE(pldm);

static smbios_bios_information bios_information = {
    .header = {
        .type = 0x0,           // type BIOS Information (Type 0) structure
        .handle = 0x0000,      // Handle values are in the range 0 to 0xFEFFh.
    },
    .vendor = 0x0,             // Type STRING default value.
    .bios_version = 0x0,       // Type STRING default value.
    .bios_release_date = 0x0,  // Type STRING default value.
};

static smbios_structure_header **structures;
static uint8_t structures_count;

static void init_bios_information()
{
	bios_information.text_strings = malloc(sizeof(char) * 2);
	if (!bios_information.text_strings) {
		LOG_ERR("%s:%s:%d: Failed to allocate memory.", __FILE__, __func__, __LINE__);
		return;
	}
	bios_information.text_strings[0] = '\0';
	bios_information.text_strings[1] = '\0';

	structures_count = 1;
	structures = malloc(sizeof(smbios_structure_header *) * structures_count);
	if (!structures) {
		LOG_ERR("%s:%s:%d: Failed to allocate memory.", __FILE__, __func__, __LINE__);
		return;
	}
	(*structures) = (smbios_structure_header *)&bios_information;
}

__weak void pldm_smbios_init_structures()
{
	init_bios_information();

	return;
}

uint8_t pldm_smbios_get_text_strings_count(char *text_strings)
{
	char *iter = text_strings;
	const uint16_t MAXIMUM_COUNT = MAXIMUM_STRUCTURE_SIZE; /*Prevent infinite loop*/
	uint8_t count = 0, i = 0;

	if (!text_strings)
		return 0;

	while (*iter != '\0') {
		uint8_t len = strlen(iter);
		count++;
		if (*(iter + len) == '\0' && *(iter + len + 1) == '\0') {
			break;
		}

		iter = iter + len + 1;
		if (++i > MAXIMUM_COUNT) {
			return 0;
		}
	}
	return count;
}

uint8_t pldm_smbios_get_text_strings_size(char *text_strings)
{
	const uint8_t NULL_BYTE = 1;
	const uint16_t MAXIMUM_COUNT = MAXIMUM_STRUCTURE_SIZE; /*Prevent infinite loop*/
	char *iter = text_strings;
	uint8_t size = 0, i = 0;

	if (!text_strings)
		return 0;

	while (true) {
		uint8_t len = strlen(iter);
		size += len + NULL_BYTE;
		if (*(iter + len) == '\0' && *(iter + len + 1) == '\0') {
			size += NULL_BYTE;
			break;
		}

		iter = iter + len + NULL_BYTE;
		if (++i > MAXIMUM_COUNT) {
			return 0;
		}
	}
	return size;
}

int pldm_smbios_set_bios_information(smbios_bios_information *new_bios_information)
{
	CHECK_NULL_ARG_WITH_RETURN(new_bios_information, -1);

	uint8_t new_text_strings_length =
		pldm_smbios_get_text_strings_size(new_bios_information->text_strings);
	SAFE_FREE(bios_information.text_strings);
	bios_information.text_strings = (char *)malloc(sizeof(char) * new_text_strings_length);
	if (!bios_information.text_strings) {
		LOG_ERR("%s:%s:%d: Failed to allocate memory.", __FILE__, __func__, __LINE__);
		return -1;
	}

	memcpy(&bios_information, new_bios_information,
	       sizeof(bios_information) - sizeof(bios_information.text_strings));
	memcpy(bios_information.text_strings, new_bios_information->text_strings,
	       new_text_strings_length);

	return 0;
}

static uint8_t filterStructureDataByType(uint8_t type, smbios_structure_header **result)
{
	uint8_t filtered_structure_data_count = 0;
	for (uint8_t i = 0; i < structures_count; i++) {
		if ((*(structures + i))->type == type) {
			filtered_structure_data_count++;
			result = realloc(result, sizeof(smbios_structure_header *) *
							 filtered_structure_data_count);
			if (!result) {
				LOG_ERR("%s:%s:%d: Failed to allocate memory.", __FILE__, __func__,
					__LINE__);
				return 0;
			}
			*(result + filtered_structure_data_count - 1) = *(structures + i);
		}
	}

	return filtered_structure_data_count;
}

static int get_structure_data_index_by_handle(uint16_t target_handle,
					      smbios_structure_header **data, uint8_t count)
{
	for (uint8_t i = 0; i < count; i++) {
		if ((*(data + i))->handle == target_handle) {
			return i;
		}
	}
	LOG_ERR("%s:%s:%d: Corresponding structure data is not found.", __FILE__, __func__,
		__LINE__);
	return -1;
}

static uint8_t get_formatted_area_size(uint8_t structure_type)
{
	uint8_t formatted_area_size = 0;

	switch (structure_type) {
	case (SMBIOS_BIOS_INFORMATION):
		formatted_area_size = sizeof(smbios_bios_information) - sizeof(char *);
		break;
	default:
		LOG_ERR("%s:%s:%d: Unsupported structure type.", __FILE__, __func__, __LINE__);
		return 0;
	}

	return formatted_area_size;
}

static char *get_structure_text_string(smbios_structure_header *structure)
{
	switch (structure->type) {
	case (SMBIOS_BIOS_INFORMATION):
		return ((smbios_bios_information *)structure)->text_strings;
	default:
		LOG_ERR("%s:%s:%d: Unsupported structure type.", __FILE__, __func__, __LINE__);
		break;
	}

	return NULL;
}

static int get_maximum_data_count_under_size(const uint16_t maximum_size,
					     smbios_structure_header **filtered_structure_data,
					     const uint8_t filtered_structure_data_count,
					     const uint8_t start_index)
{
	const uint8_t type = (*filtered_structure_data)->type;
	const uint8_t formatted_area_size = get_formatted_area_size(type);
	uint8_t total_data_size = 0, current_data_size = 0, result = 0;

	if (!formatted_area_size) {
		return -1;
	}

	for (uint8_t i = start_index; i < filtered_structure_data_count; i++) {
		current_data_size = formatted_area_size +
				    pldm_smbios_get_text_strings_size(get_structure_text_string(
					    *(filtered_structure_data + i)));

		if (total_data_size + current_data_size > maximum_size)
			break;

		total_data_size += current_data_size;
		result += 1;
	}

	return result;
}

static int copy_structure_data(uint8_t *dest, smbios_structure_header **data, uint8_t start_index,
			       uint8_t count)
{
	uint8_t *iter = dest;
	uint8_t i = 0, structure_type = 0, formatted_area_size = 0, unformatted_area_size = 0;

	for (i = start_index; i < start_index + count; i++) {
		structure_type = (*(data + i))->type;

		switch (structure_type) {
		case (SMBIOS_BIOS_INFORMATION): {
			smbios_bios_information *current = (smbios_bios_information *)(*(data + i));
			formatted_area_size = sizeof(smbios_bios_information) - sizeof(char *);
			unformatted_area_size =
				pldm_smbios_get_text_strings_size(current->text_strings);
			memcpy(iter, current, formatted_area_size);
			memcpy(iter + formatted_area_size, current->text_strings,
			       unformatted_area_size);
		} break;
		default:
			return -1;
			break;
		}

		iter += formatted_area_size + unformatted_area_size;
	}

	return iter - dest;
}

uint8_t get_smbios_structure_data_by_type(void *mctp_inst, uint8_t *req, uint16_t req_len,
					  uint8_t instance_id, uint8_t *resp, uint16_t *resp_len,
					  void *ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(req, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);
	const uint16_t GET_ALL_INSTANCES_OF_SPECIFIC_TYPE = 0xFFFF;
	uint8_t ret = PLDM_SUCCESS;

	pldm_get_smbios_structure_by_type_req *req_p = (pldm_get_smbios_structure_by_type_req *)req;
	pldm_get_smbios_structure_by_type_resp *resp_p =
		(pldm_get_smbios_structure_by_type_resp *)resp;
	uint8_t *completion_code_p = resp;
	*resp_len = sizeof(*completion_code_p);

	// Initial malloc to avoid -Wmaybe-uninitialized.
	smbios_structure_header **filtered_structure_data =
		malloc(sizeof(smbios_structure_header *));
	if (!filtered_structure_data) {
		LOG_ERR("%s:%s:%d: Failed to allocate memory.", __FILE__, __func__, __LINE__);
		return -1;
	}

	if (req_p->data_transfer_handle > MAXIMUM_HANDLE_NUM) {
		*completion_code_p = ret = PLDM_SMBIOS_INVALID_DATA_TRANSFER_HANDLE;
		goto exit;
	} else if (req_p->structure_instance_id > MAXIMUM_HANDLE_NUM &&
		   req_p->structure_instance_id != GET_ALL_INSTANCES_OF_SPECIFIC_TYPE) {
		*completion_code_p = ret = PLDM_SMBIOS_INVALID_SMBIOS_STRUCTURE_INSTANCE_ID;
		goto exit;
	}

	uint8_t result_count = 0;
	int result_start_idx = 0;
	const uint8_t filtered_structure_data_count =
		filterStructureDataByType(req_p->type, filtered_structure_data);

	if (req_p->structure_instance_id != GET_ALL_INSTANCES_OF_SPECIFIC_TYPE) {
		result_start_idx =
			get_structure_data_index_by_handle(req_p->structure_instance_id,
							   filtered_structure_data,
							   filtered_structure_data_count);
		result_count = 1;
	} else {
		if (req_p->transfer_operation_flag ==
		    PLDM_SMBIOS_TRANSFER_OPERATION_FLAG_GET_FIRST_PART) {
			result_start_idx = 0;

		} else if (req_p->transfer_operation_flag ==
			   PLDM_SMBIOS_TRANSFER_OPERATION_FLAG_GET_NEXT_PART) {
			result_start_idx =
				get_structure_data_index_by_handle(req_p->data_transfer_handle,
								   filtered_structure_data,
								   filtered_structure_data_count);
		} else {
			*completion_code_p = PLDM_SMBIOS_INVALID_TRANSFER_OPERATION_FLAG;
			return PLDM_SMBIOS_INVALID_TRANSFER_OPERATION_FLAG;
		}

		result_count = get_maximum_data_count_under_size(
			PLDM_MAX_DATA_SIZE - sizeof(pldm_hdr), filtered_structure_data,
			filtered_structure_data_count, result_start_idx);
	}

	if (!filtered_structure_data_count || result_start_idx < 0) {
		*completion_code_p = ret = PLDM_SMBIOS_NO_SMBIOS_STRUCTURES;
		goto exit;
	} else if (result_count == 0) {
		LOG_ERR("%s:%s:%d: A single structure data is over maximum transfer size.",
			__FILE__, __func__, __LINE__);
		*completion_code_p = ret = PLDM_ERROR_INVALID_LENGTH;
		goto exit;
	} else if (result_count == -1) {
		*completion_code_p = ret = PLDM_SMBIOS_INVALID_SMBIOS_STRUCTURE_TYPE;
		goto exit;
	}

	*completion_code_p = PLDM_SUCCESS;
	if (result_start_idx == 0) {
		if (result_count == filtered_structure_data_count) {
			resp_p->transfer_flag = PLDM_SMBIOS_TRANSFER_FLAG_START_AND_END;
		} else {
			resp_p->transfer_flag = PLDM_SMBIOS_TRANSFER_FLAG_START;
			resp_p->next_data_transfer_handle =
				(*(filtered_structure_data + result_count))->handle;
		}
	} else {
		if (result_start_idx + result_count == filtered_structure_data_count) {
			resp_p->transfer_flag = PLDM_SMBIOS_TRANSFER_FLAG_END;
		} else if (result_start_idx + result_count < filtered_structure_data_count) {
			resp_p->transfer_flag = PLDM_SMBIOS_TRANSFER_FLAG_MIDDLE;
			resp_p->next_data_transfer_handle =
				(*(filtered_structure_data + result_start_idx + result_count))
					->handle;
		}
	}

	uint8_t *smbios_structure_data_p =
		(uint8_t *)resp_p + sizeof(pldm_get_smbios_structure_by_type_resp);
	const int structure_data_len = copy_structure_data(
		smbios_structure_data_p, filtered_structure_data, result_start_idx, result_count);

	if (!structure_data_len) {
		*completion_code_p = ret = PLDM_SMBIOS_INVALID_SMBIOS_STRUCTURE_TYPE;
		goto exit;
	}

	*resp_len = sizeof(pldm_get_smbios_structure_by_type_resp) + structure_data_len;

exit:
	SAFE_FREE(filtered_structure_data);
	return ret;
}

static pldm_cmd_handler pldm_smbios_cmd_tbl[] = {
	{ PLDM_SMBIOS_CMD_CODE_GET_SMBIOS_STRUCTURE_BY_TYPE, get_smbios_structure_data_by_type },
};

uint8_t pldm_smbios_handler_query(uint8_t code, void **ret_fn)
{
	if (!ret_fn)
		return PLDM_ERROR;

	pldm_cmd_proc_fn fn = NULL;

	for (uint8_t i = 0; i < ARRAY_SIZE(pldm_smbios_cmd_tbl); i++) {
		if (pldm_smbios_cmd_tbl[i].cmd_code == code) {
			fn = pldm_smbios_cmd_tbl[i].fn;
			break;
		}
	}

	*ret_fn = (void *)fn;
	return fn ? PLDM_SUCCESS : PLDM_ERROR;
}
