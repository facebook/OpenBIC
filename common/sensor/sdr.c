#include <stdio.h>
#include <string.h>
#include "sdr.h"
#include "sensor.h"
#include "sensor_def.h"
#include "plat_ipmb.h"
#include "pal.h"

SDR_INFO sdr_info;
static uint16_t RSV_ID[2] = { 0 };
uint8_t is_SDR_not_init = 1;

SDR_Full_sensor full_sensor_table[MAX_SENSOR_SIZER];

uint8_t SDR_NUM = 0;

void SDR_clear_ID(void)
{
	sdr_info.current_ID = sdr_info.start_ID;
	return;
}

uint16_t SDR_get_record_ID(uint16_t current_ID)
{
	if (current_ID < sdr_info.last_ID) {
		return ++current_ID;
	} else if (current_ID == sdr_info.last_ID) {
		return SDR_END_ID;
	}

	return SDR_INVALID_ID;
}

uint16_t SDR_check_record_ID(uint16_t current_ID)
{
	if (current_ID > sdr_info.last_ID) {
		return false;
	}

	return true;
}

uint16_t SDR_get_RSV_ID(uint8_t rsv_table_index)
{
	return (++RSV_ID[rsv_table_index]);
}

bool SDR_RSV_ID_check(uint16_t ID, uint8_t rsv_table_index)
{
	return (RSV_ID[rsv_table_index] == ID) ? true : false;
}

uint8_t SDR_init(void)
{
	int i;

	SDR_NUM = pal_load_sdr_table();
	pal_fix_fullSDR_table();
	sdr_info.start_ID = 0x0000;
	sdr_info.current_ID = sdr_info.start_ID;

	for (i = 0; i < SDR_NUM; i++) {
		full_sensor_table[i].record_id_h = (i >> 8) & 0xFF;
		full_sensor_table[i].record_id_l = (i & 0xFF);
		full_sensor_table[i].ID_len += strlen(full_sensor_table[i].ID_str);
		full_sensor_table[i].record_len += strlen(full_sensor_table[i].ID_str);

		if (DEBUG_SENSOR) {
			printf("%s ID: 0x%x%x, size: %d, recordlen: %d\n",
			       full_sensor_table[i].ID_str, full_sensor_table[i].record_id_h,
			       full_sensor_table[i].record_id_l, full_sensor_table[i].ID_len,
			       full_sensor_table[i].record_len);
		}
	}

	i--;
	sdr_info.last_ID =
		(full_sensor_table[i].record_id_h << 8) | (full_sensor_table[i].record_id_l);
	if (DEBUG_SENSOR) {
		printf("%s ID: 0x%x%x, size: %d, recordlen: %d\n", full_sensor_table[i].ID_str,
		       full_sensor_table[i].record_id_h, full_sensor_table[i].record_id_l,
		       full_sensor_table[i].ID_len, full_sensor_table[i].record_len);
	}

	is_SDR_not_init = 0;
	return true;
}
