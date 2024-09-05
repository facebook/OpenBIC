#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pdr.h"
#include "sensor.h"
#include "plat_sensor_table.h"
#include "plat_ipmb.h"

#include <logging/log.h>

LOG_MODULE_REGISTER(pdr);

uint32_t total_record_count = 0;

PDR_INFO *pdr_info = NULL;
PDR_numeric_sensor *numeric_sensor_table = NULL;
PDR_sensor_auxiliary_names *sensor_auxiliary_names_table = NULL;
PDR_entity_auxiliary_names *entity_auxiliary_names_table = NULL;

int pdr_init(void)
{
	uint32_t pdr_count = 0, record_handle = 0x0, largest_record_size = 0;

	pdr_count = plat_get_pdr_size(PLDM_NUMERIC_SENSOR_PDR);
	if (pdr_count != 0) {
		total_record_count += pdr_count;
		numeric_sensor_table =
			(PDR_numeric_sensor *)malloc(pdr_count * sizeof(PDR_numeric_sensor));
		plat_load_numeric_sensor_pdr_table(numeric_sensor_table);
		if (numeric_sensor_table == NULL) {
			LOG_ERR("Failed to malloc numeric sensor PDR table");
			return -1;
		}
		pdr_info->repository_size += pdr_count * sizeof(PDR_numeric_sensor);

		for (uint32_t i = 0; i < pdr_count; i++) {
			numeric_sensor_table[i].pdr_common_header.record_handle = record_handle;
			numeric_sensor_table[i].pdr_common_header.data_length +=
				(sizeof(PDR_numeric_sensor) - sizeof(PDR_common_header));
			record_handle++;
		}

		if (largest_record_size < sizeof(PDR_numeric_sensor)) {
			largest_record_size = sizeof(PDR_numeric_sensor);
		}
	}

	pdr_count = plat_get_pdr_size(PLDM_SENSOR_AUXILIARY_NAMES_PDR);
	if (pdr_count != 0) {
		total_record_count += pdr_count;
		sensor_auxiliary_names_table = (PDR_sensor_auxiliary_names *)malloc(
			pdr_count * sizeof(PDR_sensor_auxiliary_names));
		plat_load_aux_sensor_names_pdr_table(sensor_auxiliary_names_table);
		if (sensor_auxiliary_names_table == NULL) {
			LOG_ERR("Failed to malloc sensor auxiliary names PDR table");
			return -1;
		}
		pdr_info->repository_size += pdr_count * sizeof(PDR_sensor_auxiliary_names);

		for (uint32_t i = 0; i < pdr_count; i++) {
			sensor_auxiliary_names_table[i].pdr_common_header.record_handle =
				record_handle;
			sensor_auxiliary_names_table[i].pdr_common_header.data_length +=
				(sizeof(PDR_sensor_auxiliary_names) - sizeof(PDR_common_header));

			// Convert sensor name to UTF16-BE
			for (int j = 0; sensor_auxiliary_names_table[i].sensorName[j] != 0x0000;
			     j++) {
				sensor_auxiliary_names_table[i].sensorName[j] = sys_cpu_to_be16(
					sensor_auxiliary_names_table[i].sensorName[j]);
			}

			record_handle++;
		}

		if (largest_record_size < sizeof(PDR_sensor_auxiliary_names)) {
			largest_record_size = sizeof(PDR_sensor_auxiliary_names);
		}
	}

	pdr_count = plat_get_pdr_size(PLDM_ENTITY_AUXILIARY_NAMES_PDR);
	if (pdr_count != 0) {
		plat_init_entity_aux_names_pdr_table();
		total_record_count += pdr_count;
		entity_auxiliary_names_table = (PDR_entity_auxiliary_names *)malloc(
			pdr_count * plat_get_pdr_entity_aux_names_size());
		plat_load_entity_aux_names_pdr_table(entity_auxiliary_names_table);
		if (entity_auxiliary_names_table == NULL) {
			LOG_ERR("Failed to malloc entity auxiliary names PDR table");
			return -1;
		}
		pdr_info->repository_size += pdr_count * plat_get_pdr_entity_aux_names_size();

		for (uint32_t i = 0; i < pdr_count; i++) {
			entity_auxiliary_names_table[i].pdr_common_header.record_handle =
				record_handle;
			entity_auxiliary_names_table[i].pdr_common_header.data_length +=
				(plat_get_pdr_entity_aux_names_size() - sizeof(PDR_common_header));
			// Convert entity name to UTF16-BE
			for (int j = 0; entity_auxiliary_names_table[i].entityName[j] != 0x0000;
			     j++) {
				entity_auxiliary_names_table[i].entityName[j] = sys_cpu_to_be16(
					entity_auxiliary_names_table[i].entityName[j]);
			}

			record_handle++;
		}

		if (largest_record_size < sizeof(PDR_entity_auxiliary_names)) {
			largest_record_size = sizeof(PDR_entity_auxiliary_names);
		}
	}

	pdr_info = (PDR_INFO *)malloc(sizeof(PDR_INFO));
	if (pdr_info == NULL) {
		LOG_ERR("Failed to malloc PDR info");
		return false;
	}
	pdr_info->repository_state = PDR_STATE_AVAILABLE;
	pdr_info->record_count = total_record_count;
	pdr_info->largest_record_size = largest_record_size;

	return 0;
}

PDR_INFO *get_pdr_info()
{
	return pdr_info;
}

// Function to convert char16_t string to UTF-8
void char16_to_char(const char16_t *src, char *dest, size_t max_length)
{
	CHECK_NULL_ARG(src);
	CHECK_NULL_ARG(dest);

	size_t dest_index = 0;

	while (*src && dest_index < max_length - 1) { // Leave space for null terminator
		uint32_t src_data = *src++;

		if (src_data < 0x80) {
			// Single-byte character (ASCII)
			dest[dest_index++] = (char)src_data;
		} else if (src_data < 0x800) {
			// Two-byte character
			if (dest_index + 1 >= max_length - 1)
				break;
			dest[dest_index++] = (char)(0xC0 | (src_data >> 6));
			dest[dest_index++] = (char)(0x80 | (src_data & 0x3F));
		} else if (src_data < 0x10000) {
			// Three-byte character
			if (dest_index + 2 >= max_length - 1)
				break;
			dest[dest_index++] = (char)(0xE0 | (src_data >> 12));
			dest[dest_index++] = (char)(0x80 | ((src_data >> 6) & 0x3F));
			dest[dest_index++] = (char)(0x80 | (src_data & 0x3F));
		} else {
			// Four-byte character
			if (dest_index + 3 >= max_length - 1)
				break;
			dest[dest_index++] = (char)(0xF0 | (src_data >> 18));
			dest[dest_index++] = (char)(0x80 | ((src_data >> 12) & 0x3F));
			dest[dest_index++] = (char)(0x80 | ((src_data >> 6) & 0x3F));
			dest[dest_index++] = (char)(0x80 | (src_data & 0x3F));
		}
	}

	dest[dest_index] = '\0'; // Null-terminate the destination string
}

int pldm_get_sensor_name_via_sensor_id(uint16_t sensor_id, char *sensor_name, size_t max_length)
{
	CHECK_NULL_ARG_WITH_RETURN(sensor_name, -1);

	for (size_t i = 0; i < plat_get_pdr_size(PLDM_NUMERIC_SENSOR_PDR); i++) {
		if (sensor_auxiliary_names_table[i].sensor_id == sensor_id) {
			PDR_sensor_auxiliary_names *temp_table =
				(PDR_sensor_auxiliary_names *)malloc(
					sizeof(*sensor_auxiliary_names_table) *
					plat_get_pdr_size(PLDM_SENSOR_AUXILIARY_NAMES_PDR));
			if (!temp_table) {
				LOG_ERR("temp_table malloc failed!");
				return -1;
			}
			memcpy(temp_table, sensor_auxiliary_names_table,
			       sizeof(*sensor_auxiliary_names_table) *
				       plat_get_pdr_size(PLDM_SENSOR_AUXILIARY_NAMES_PDR));

			LOG_HEXDUMP_DBG(temp_table, sizeof(*temp_table), "temp_table");

			uint32_t pdr_count = plat_get_pdr_size(PLDM_SENSOR_AUXILIARY_NAMES_PDR);
			for (uint32_t i = 0; i < pdr_count; i++) {
				// Convert sensor name to UTF16-LE
				for (int j = 0; temp_table[i].sensorName[j] != 0x0000; j++) {
					temp_table[i].sensorName[j] =
						sys_cpu_to_be16(temp_table[i].sensorName[j]);
				}
			}

			// Convert the char16_t string to UTF-8
			char16_to_char(temp_table[i].sensorName, sensor_name, max_length);

			free(temp_table);
			return 0;
		}
	}
	return -1;
}

int get_pdr_table_via_record_handle(uint8_t *record_data, uint32_t record_handle)
{
	uint32_t numeric_sensor_pdr_count = 0, aux_sensor_name_pdr_count = 0,
		 entity_aux_name_pdr_count = 0;

	numeric_sensor_pdr_count = plat_get_pdr_size(PLDM_NUMERIC_SENSOR_PDR);
	aux_sensor_name_pdr_count = plat_get_pdr_size(PLDM_SENSOR_AUXILIARY_NAMES_PDR);
	entity_aux_name_pdr_count = plat_get_pdr_size(PLDM_ENTITY_AUXILIARY_NAMES_PDR);

	if (record_handle < numeric_sensor_pdr_count) {
		for (int i = 0; i < numeric_sensor_pdr_count; i++) {
			if (numeric_sensor_table[i].pdr_common_header.record_handle ==
			    record_handle) {
				memcpy(record_data, &numeric_sensor_table[i],
				       sizeof(PDR_numeric_sensor));
				return sizeof(PDR_numeric_sensor);
			}
		}
	} else if (record_handle < numeric_sensor_pdr_count + aux_sensor_name_pdr_count) {
		for (int i = 0; i < aux_sensor_name_pdr_count; i++) {
			if (sensor_auxiliary_names_table[i].pdr_common_header.record_handle ==
			    record_handle) {
				memcpy(record_data, &sensor_auxiliary_names_table[i],
				       sizeof(PDR_sensor_auxiliary_names));
				return sizeof(PDR_sensor_auxiliary_names);
			}
		}
	} else if (record_handle < numeric_sensor_pdr_count + aux_sensor_name_pdr_count +
					   entity_aux_name_pdr_count) {
		for (int i = 0; i < entity_aux_name_pdr_count; i++) {
			if (entity_auxiliary_names_table[i].pdr_common_header.record_handle ==
			    record_handle) {
				memcpy(record_data, &entity_auxiliary_names_table[i],
				       plat_get_pdr_entity_aux_names_size());
				return plat_get_pdr_entity_aux_names_size();
			}
		}
	} else {
		LOG_ERR("Failed to get PDR via record handle: %x\n", record_handle);
		return -1;
	}

	return -1;
}

uint32_t get_record_count()
{
	return total_record_count;
}

__weak uint32_t plat_get_pdr_size(uint8_t pdr_type)
{
	//implement in platform layer
	return 0;
}

__weak void plat_load_numeric_sensor_pdr_table(PDR_numeric_sensor *numeric_sensor_table)
{
	//implement in platform layer
	numeric_sensor_table = NULL;
}

__weak void plat_load_aux_sensor_names_pdr_table(PDR_sensor_auxiliary_names *aux_sensor_name_table)
{
	//implement in platform layer
	aux_sensor_name_table = NULL;
}

__weak void plat_init_entity_aux_names_pdr_table()
{
	//implement in platform layer
}

__weak void plat_load_entity_aux_names_pdr_table(PDR_entity_auxiliary_names *entity_aux_name_table)
{
	//implement in platform layer
	entity_aux_name_table = NULL;
}

__weak uint16_t plat_get_pdr_entity_aux_names_size()
{
	//implement in platform layer
	return 0;
}

// Function to get a pointer to the entity_auxiliary_names_table
PDR_entity_auxiliary_names *get_entity_auxiliary_names_table()
{
	return entity_auxiliary_names_table;
}
