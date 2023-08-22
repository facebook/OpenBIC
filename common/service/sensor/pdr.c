#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pdr.h"
#include "sensor.h"
#include "plat_sensor_table.h"
#include "plat_ipmb.h"

#include <logging/log.h>

LOG_MODULE_REGISTER(pdr);

uint16_t pdr_count = 0;
PDR_INFO *pdr_info = NULL;
PDR_numeric_sensor *numeric_sensor_table = NULL;

uint8_t pdr_init(void)
{
	pdr_count = plat_get_pdr_size();
	numeric_sensor_table =
		(PDR_numeric_sensor *)malloc(pdr_count * sizeof(PDR_numeric_sensor));
	plat_load_pdr_table(numeric_sensor_table);
	if (numeric_sensor_table == NULL) {
		LOG_ERR("Failed to load PDR table");
		return false;
	}

	for (uint32_t i = 0; i < pdr_count; i++) {
		numeric_sensor_table[i].pdr_common_header.record_handle = i & 0xFFFFFFFF;
		numeric_sensor_table[i].pdr_common_header.data_length +=
			(sizeof(PDR_numeric_sensor) - sizeof(PDR_common_header));
	}

	pdr_info->repository_state = PDR_STATE_AVAILABLE;
	pdr_info->record_count = pdr_count;
	pdr_info->repository_size = pdr_count * sizeof(PDR_numeric_sensor);
	pdr_info->largest_record_size = sizeof(PDR_numeric_sensor);

	return true;
}

PDR_INFO* get_pdr_info()
{
	return pdr_info;
}

PDR_numeric_sensor* get_pdr_table()
{
	return numeric_sensor_table;
}

uint16_t get_pdr_size()
{
	return pdr_count;
}

__weak uint16_t plat_get_pdr_size()
{
	//implement in platform layer
	return 0;
}

__weak void plat_load_pdr_table(PDR_numeric_sensor* numeric_sensor_table)
{
	//implement in platform layer
	numeric_sensor_table = NULL;
}
