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

int get_pdr_table_via_record_handle(uint8_t *record_data, uint32_t record_handle)
{
	uint32_t numeric_sensor_pdr_count = 0, aux_sensor_name_pdr_count = 0;

	numeric_sensor_pdr_count = plat_get_pdr_size(PLDM_NUMERIC_SENSOR_PDR);
	aux_sensor_name_pdr_count = plat_get_pdr_size(PLDM_SENSOR_AUXILIARY_NAMES_PDR);

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
