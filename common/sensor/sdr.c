#include <stdio.h>
#include <string.h>
#include "sdr.h"
#include "sensor.h"
#include "plat_sdr_table.h"
#include "plat_sensor_table.h"
#include "plat_ipmb.h"

SDR_INFO sdr_info;
static uint16_t RSV_ID = 0;
uint8_t is_SDR_not_init = 1;

SDR_Full_sensor full_sdr_table[MAX_SNR_SIZE];

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

uint16_t SDR_get_RSV_ID(void)
{
	return (++RSV_ID);
}

bool SDR_RSV_ID_check(uint16_t ID)
{
	return (RSV_ID == ID) ? 1 : 0;
}

__weak void pal_fix_full_sdr_table(void)
{
	return;
}

uint8_t get_sdr_index(uint8_t sensor_num)
{
	uint8_t i, j;
	for (i = 0; i < SENSOR_NUM_MAX; i++) {
		for (j = 0; j < SDR_NUM; ++j) {
			if (sensor_num == full_sdr_table[j].sensor_num) {
				return j;
			} else if (i == SDR_NUM) {
				return SENSOR_NUM_MAX;
			}
		}
	}

	return SENSOR_NUM_MAX;
}

void add_full_sdr_table(SDR_Full_sensor add_item)
{
	if (get_sdr_index(add_item.sensor_num) != SENSOR_NUM_MAX) {
		printk("add sensor num [%x] is already exists\n", add_item.sensor_num);
	} else {
		full_sdr_table[SDR_NUM++] = add_item;
	}
}

void change_sensor_threshold(uint8_t sensor_num, uint8_t threshold_type, uint8_t change_value)
{
	uint8_t sdr_index = get_sdr_index(sensor_num);
	if (sdr_index == 0xFF) {
		printf("Not found change threshold sensor\n");
		return;
	}
	if (threshold_type == threshold_UNR) {
		full_sdr_table[sdr_index].UNRT = change_value;
	} else if (threshold_type == threshold_UCR) {
		full_sdr_table[sdr_index].UCT = change_value;
	} else if (threshold_type == threshold_UNC) {
		full_sdr_table[sdr_index].UNCT = change_value;
	} else if (threshold_type == threshold_LNR) {
		full_sdr_table[sdr_index].LNRT = change_value;
	} else if (threshold_type == threshold_LCR) {
		full_sdr_table[sdr_index].LCT = change_value;
	} else if (threshold_type == threshold_LNC) {
		full_sdr_table[sdr_index].LNCT = change_value;
	} else {
		printf("Not found want changing threshold\n");
		return;
	}
}

void change_sensor_MBR(uint8_t sensor_num, uint8_t MBR_type, uint16_t change_value)
{
	uint8_t sdr_index = get_sdr_index(sensor_num);
	if (sdr_index == 0xFF) {
		printf("Not found change threshold sensor\n");
		return;
	}
	if (MBR_type == MBR_M) {
		full_sdr_table[sdr_index].M = change_value & 0xFF;
		if (change_value >> 8) {
			full_sdr_table[sdr_index].M_tolerance = ((change_value >> 8) << 6) & 0xFF;
		} else {
			full_sdr_table[sdr_index].M_tolerance = 0;
		}
	} else if (MBR_type == MBR_B) {
		full_sdr_table[sdr_index].B = change_value;
		if (change_value >> 8) {
			full_sdr_table[sdr_index].B_accuracy = ((change_value >> 8) << 6) & 0xFF;
		} else {
			full_sdr_table[sdr_index].B_accuracy = 0;
		}
	} else if (MBR_type == MBR_R) {
		full_sdr_table[sdr_index].RexpBexp = change_value & 0xFF;
	} else {
		printf("Not found want changing MBR type\n");
		return;
	}
}

uint8_t SDR_init(void)
{
	int i;

	SDR_NUM = load_sdr_table();
	pal_fix_full_sdr_table();
	sdr_info.start_ID = 0x0000;
	sdr_info.current_ID = sdr_info.start_ID;

	for (i = 0; i < SDR_NUM; i++) {
		full_sdr_table[i].record_id_h = (i >> 8) & 0xFF;
		full_sdr_table[i].record_id_l = (i & 0xFF);
		full_sdr_table[i].ID_len += strlen(full_sdr_table[i].ID_str);
		full_sdr_table[i].record_len += strlen(full_sdr_table[i].ID_str);

		if (DEBUG_SNR) {
			printf("%s ID: 0x%x%x, size: %d, recordlen: %d\n", full_sdr_table[i].ID_str,
			       full_sdr_table[i].record_id_h, full_sdr_table[i].record_id_l,
			       full_sdr_table[i].ID_len, full_sdr_table[i].record_len);
		}
	}

	i--;
	sdr_info.last_ID = (full_sdr_table[i].record_id_h << 8) | (full_sdr_table[i].record_id_l);
	if (DEBUG_SNR) {
		printf("%s ID: 0x%x%x, size: %d, recordlen: %d\n", full_sdr_table[i].ID_str,
		       full_sdr_table[i].record_id_h, full_sdr_table[i].record_id_l,
		       full_sdr_table[i].ID_len, full_sdr_table[i].record_len);
	}

	is_SDR_not_init = 0;
	return true;
}
