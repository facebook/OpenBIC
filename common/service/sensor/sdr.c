#include <stdio.h>
#include <string.h>
#include "sdr.h"
#include "sensor.h"
#include "plat_sdr_table.h"
#include "plat_sensor_table.h"
#include "plat_ipmb.h"

SDR_INFO sdr_info;
static uint16_t RSV_ID[2] = { 0 };
bool is_sdr_not_init = true;

extern SDR_Full_sensor plat_sdr_table[];
extern const int SDR_TABLE_SIZE;

SDR_Full_sensor *full_sdr_table;

uint8_t sensor_config_size = 0;
uint8_t sdr_count = 0;

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

bool SDR_check_record_ID(uint16_t current_ID)
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

__weak void pal_extend_full_sdr_table(void)
{
	/* This function is for the BIC which connects to expansion boards and need to add expansion board's sensor to itself.
   * For the BIC (eg. RF/WF) which doesn't connect expandion boards has no need to implement this function. 
   */
	return;
}

int get_sdr_index(uint8_t sensor_num)
{
	if (full_sdr_table == NULL) {
		printf("[%s] full_sdr_table is NULL\n", __func__);
		return -1;
	}

	uint8_t i = 0;
	for (i = 0; i < sdr_count; ++i) {
		if (sensor_num == full_sdr_table[i].sensor_num) {
			return i;
		}
	}
	return SENSOR_NUM_MAX;
}

void add_full_sdr_table(SDR_Full_sensor add_item)
{
	if (full_sdr_table == NULL) {
		printf("[%s] full_sdr_table is NULL\n", __func__);
		return;
	}

	int index = get_sdr_index(add_item.sensor_num);
	if (index == -1) {
		printf("[%s] fail to get sdr index\n", __func__);
		return;
	}

	if (index != SENSOR_NUM_MAX) {
		memcpy(&full_sdr_table[index], &add_item, sizeof(SDR_Full_sensor));
		printf("[%s] replace the sensor[0x%02x] SDR\n", __func__, add_item.sensor_num);
		return;
	}
	// Check SDR table size before adding SDR
	if (sdr_count + 1 <= sensor_config_size) {
		full_sdr_table[sdr_count++] = add_item;
	} else {
		printf("[%s] add SDR would over SDR max size\n", __func__);
	}
}

void change_sensor_threshold(uint8_t sensor_num, uint8_t threshold_type, uint8_t change_value)
{
	if (full_sdr_table == NULL) {
		printf("[%s] full_sdr_table is NULL\n", __func__);
		return;
	}

	int sdr_index = get_sdr_index(sensor_num);
	if ((sdr_index == SENSOR_NUM_MAX) || (sdr_index == -1)) {
		printf("[%s] Failed to find sensor index, sensor number(0x%02x), sdr index(%d)\n",
		       __func__, sensor_num, sdr_index);
		return;
	}
	switch (threshold_type) {
	case THRESHOLD_UNR:
		full_sdr_table[sdr_index].UNRT = change_value;
		break;
	case THRESHOLD_UCR:
		full_sdr_table[sdr_index].UCT = change_value;
		break;
	case THRESHOLD_UNC:
		full_sdr_table[sdr_index].UNCT = change_value;
		break;
	case THRESHOLD_LNR:
		full_sdr_table[sdr_index].LNRT = change_value;
		break;
	case THRESHOLD_LCR:
		full_sdr_table[sdr_index].LCT = change_value;
		break;
	case THRESHOLD_LNC:
		full_sdr_table[sdr_index].LNCT = change_value;
		break;
	default:
		printf("Invalid threshold type during changing sensor threshold\n");
		break;
	}
}

void change_sensor_mbr(uint8_t sensor_num, uint8_t mbr_type, uint16_t change_value)
{
	if (full_sdr_table == NULL) {
		printf("[%s] full_sdr_table is NULL\n", __func__);
		return;
	}

	int sdr_index = get_sdr_index(sensor_num);
	if ((sdr_index == SENSOR_NUM_MAX) || (sdr_index == -1)) {
		printf("[%s] Failed to find sensor index, sensor number(0x%02x), sdr index(%d)\n",
		       __func__, sensor_num, sdr_index);
		return;
	}
	switch (mbr_type) {
	case MBR_M:
		full_sdr_table[sdr_index].M = change_value & 0xFF;
		if (change_value >> 8) {
			full_sdr_table[sdr_index].M_tolerance = ((change_value >> 8) << 6) & 0xFF;
		} else {
			full_sdr_table[sdr_index].M_tolerance = 0;
		}
		break;
	case MBR_B:
		full_sdr_table[sdr_index].B = change_value;
		if (change_value >> 8) {
			full_sdr_table[sdr_index].B_accuracy = ((change_value >> 8) << 6) & 0xFF;
		} else {
			full_sdr_table[sdr_index].B_accuracy = 0;
		}
		break;
	case MBR_R:
		full_sdr_table[sdr_index].RexpBexp = change_value & 0xFF;
		break;
	default:
		printf("Invalid MBR type during changing sensor MBR\n");
		break;
	}
}

uint8_t sdr_init(void)
{
	int i;

	load_sdr_table();
	sdr_info.start_ID = 0x0000;
	sdr_info.current_ID = sdr_info.start_ID;

	if (full_sdr_table == NULL) {
		printf("[%s] full_sdr_table is NULL\n", __func__);
		return false;
	}

	for (i = 0; i < sdr_count; i++) {
		full_sdr_table[i].record_id_h = (i >> 8) & 0xFF;
		full_sdr_table[i].record_id_l = (i & 0xFF);
		full_sdr_table[i].ID_len += strlen(full_sdr_table[i].ID_str);
		full_sdr_table[i].record_len += strlen(full_sdr_table[i].ID_str);

		if (DEBUG_SENSOR) {
			printf("%s ID: 0x%x%x, size: %d, recordlen: %d\n", full_sdr_table[i].ID_str,
			       full_sdr_table[i].record_id_h, full_sdr_table[i].record_id_l,
			       full_sdr_table[i].ID_len, full_sdr_table[i].record_len);
		}
	}

	// Record last SDR record ID to sdr_info
	sdr_info.last_ID = (full_sdr_table[sdr_count - 1].record_id_h << 8) |
			   (full_sdr_table[sdr_count - 1].record_id_l);

	is_sdr_not_init = false;
	return true;
}

uint8_t plat_get_sdr_size()
{
	return SDR_TABLE_SIZE;
}

__weak void load_sdr_table(void)
{
	memcpy(full_sdr_table, plat_sdr_table, SDR_TABLE_SIZE * sizeof(SDR_Full_sensor));
	sdr_count = SDR_TABLE_SIZE;
}
