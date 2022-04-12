#include "sensor.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "power_status.h"
#include "sdr.h"
#include "hal_i2c.h"
#include "plat_sensor_table.h"
#include "plat_sdr_table.h"
#include "adm1278.h"
#include "isl69260.h"
#include "adc.h"
#include "nvme.h"
#include "pch.h"
#include "peci.h"
#include "tmp75.h"

#define RETRY 5

struct k_thread sensor_poll;
K_KERNEL_STACK_MEMBER(sensor_poll_stack, SENSOR_POLL_STACK_SIZE);

uint8_t sensor_config_index_map[SENSOR_NUM_MAX];
uint8_t sdr_index_map[SENSOR_NUM_MAX];

bool enable_sensor_poll_thread = true;
static bool sensor_poll_enable_flag = true;

const int negative_ten_power[16] = { 1,	    1,		1,	   1,	     1,	      1,
				     1,	    1000000000, 100000000, 10000000, 1000000, 100000,
				     10000, 1000,	100,	   10 };

snr_cfg *sensor_config;
uint8_t sensor_config_num;

static void init_sensor_num(void)
{
	for (int i = 0; i < SENSOR_NUM_MAX; i++) {
		sdr_index_map[i] = 0xFF;
		sensor_config_index_map[i] = 0xFF;
	}
}

void map_sensor_num_to_sdr_cfg(void)
{
	uint8_t i, j;

	for (i = 0; i < SENSOR_NUM_MAX; i++) {
		for (j = 0; j < SDR_NUM; j++) {
			if (i == full_sdr_table[j].sensor_num) {
				sdr_index_map[i] = j;
				break;
			} else if (i == SDR_NUM) {
				sdr_index_map[i] = SENSOR_NULL;
			} else {
				continue;
			}
		}
		for (j = 0; j < SDR_NUM; j++) {
			if (i == sensor_config[j].num) {
				sensor_config_index_map[i] = j;
				break;
			} else if (i == SDR_NUM) {
				sensor_config_index_map[i] = SENSOR_NULL;
			} else {
				continue;
			}
		}
	}
	return;
}

bool access_check(uint8_t sensor_num)
{
	bool (*access_checker)(uint8_t);

	access_checker = sensor_config[sensor_config_index_map[sensor_num]].access_checker;
	return (access_checker)(sensor_config[sensor_config_index_map[sensor_num]].num);
}

void clear_unaccessible_sensor_cache()
{
	uint8_t poll_num;

	for (poll_num = 0; poll_num < SENSOR_NUM_MAX; poll_num++) {
		if (sensor_config_index_map[poll_num] == SENSOR_NULL) { // sensor not exist
			continue;
		}

		if (!access_check(poll_num)) {
			sensor_config[sensor_config_index_map[poll_num]].cache = SENSOR_FAIL;
			sensor_config[sensor_config_index_map[poll_num]].cache_status =
				SENSOR_INIT_STATUS;
		}
	}
}

bool sensor_read(uint8_t sensor_num, float *reading)
{
	if (reading == NULL) {
		return false;
	}

	bool status;
	snr_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];

	// Switch i2c mux
	if ((cfg->mux_address != NONE) && (cfg->mux_offset != NONE)) {
		I2C_MSG msg;
		msg.bus = cfg->port;
		msg.target_addr = cfg->mux_address;
		msg.data[0] = cfg->mux_offset;
		msg.tx_len = 1;
		msg.rx_len = 0;
		if (i2c_master_write(&msg, RETRY)) {
			cfg->cache_status = SENSOR_FAIL_TO_ACCESS;
			return false;
		}
	}

	if (cfg->pre_sensor_read_fn) {
		if (cfg->pre_sensor_read_fn(sensor_num, cfg->pre_sensor_read_args)) {
			cfg->cache_status = SENSOR_FAIL_TO_ACCESS;
			return false;
		}
	}

	switch (cfg->type) {
	case TYPE_TMP75:
		status = tmp75_read(sensor_num, reading);
		break;
	case TYPE_ADC:
		status = adc_sensor_read(sensor_num, reading);
		break;
	case TYPE_PECI:
		status = peci_sensor_read(sensor_num, reading);
		break;
	case TYPE_ISL69260:
		status = isl69260_read(sensor_num, reading);
		break;
	case TYPE_PCH:
		status = pch_read(sensor_num, reading);
		break;
	case TYPE_HSC:
		status = adm1278_read(sensor_num, reading);
		break;
	case TYPE_NVME:
		status = nvme_read(sensor_num, reading);
		break;
	case TYPE_MEDUSA:
		//TODO: get sensors data of medusa
		reading = 0;
		status = true;
		break;
	case TYPE_FAN:
		//TODO: get sensors data of fan
		reading = 0;
		status = true;
		break;

	default:
		printf("sensor_read with unexpected sensor type\n");
		return false;
		break;
	}

	if (cfg->post_sensor_read_fn) {
		if (cfg->post_sensor_read_fn(sensor_num, cfg->post_sensor_read_args)) {
			cfg->cache_status = SENSOR_FAIL_TO_ACCESS;
			return false;
		}
	}

	if (!status) {
		cfg->cache_status = SENSOR_FAIL_TO_ACCESS;
		return false;
	}

	return true;
}

uint8_t get_sensor_reading(uint8_t sensor_num, float *reading, uint8_t read_mode)
{
	if (reading == NULL) {
		return false;
	}

	uint8_t status;

	if (sdr_index_map[sensor_num] == 0xFF) { // look for sensor in SDR table
		return SENSOR_NOT_FOUND;
	}

	if (!access_check(sensor_num)) { // sensor not accessable
		return SENSOR_NOT_ACCESSIBLE;
	}

	switch (read_mode) {
	case GET_FROM_SENSOR:
		status = sensor_read(sensor_num, reading);
		if (status) {
			if (!access_check(
				    sensor_num)) { // double check access to avoid not accessible read at same moment status change
				return SENSOR_NOT_ACCESSIBLE;
			}
			return sensor_config[sensor_config_index_map[sensor_num]].cache_status;
		} else {
			printf("Failed to read sensor value from device, sensor number: 0x%x\n",
			       sensor_num);
			return sensor_config[sensor_config_index_map[sensor_num]].cache_status;
		}
		break;
	case GET_FROM_CACHE:
		switch (sensor_config[sensor_config_index_map[sensor_num]].cache_status) {
		case SENSOR_READ_SUCCESS:
			*reading = sensor_config[sensor_config_index_map[sensor_num]].cache;
			if (!access_check(
				    sensor_num)) { // double check access to avoid not accessible read at same moment status change
				return SENSOR_NOT_ACCESSIBLE;
			}
			return sensor_config[sensor_config_index_map[sensor_num]].cache_status;
		case SENSOR_INIT_STATUS:
			sensor_config[sensor_config_index_map[sensor_num]].cache = SENSOR_FAIL;
			return sensor_config[sensor_config_index_map[sensor_num]].cache_status;
		default:
			sensor_config[sensor_config_index_map[sensor_num]].cache = SENSOR_FAIL;
			printf("Failed to read sensor value from cache, sensor number: 0x%x\n, cache status: 0x%x",
			       sensor_num,
			       sensor_config[sensor_config_index_map[sensor_num]].cache_status);
			return sensor_config[sensor_config_index_map[sensor_num]].cache_status;
		}
		break;
	default:
		printf("Invalid mbr type during changing sensor mbr\n");
		break;
	}

	return SENSOR_UNSPECIFIED_ERROR; // should not reach here
}

void disable_sensor_poll()
{
	sensor_poll_enable_flag = false;
}

void enable_sensor_poll()
{
	sensor_poll_enable_flag = true;
}

void sensor_poll_handler(void *arug0, void *arug1, void *arug2)
{
	uint8_t poll_num;
	int sensor_poll_interval_ms;
	float reading;
	k_msleep(1000); // delay 1 second to wait for drivers ready before start sensor polling

	pal_set_sensor_poll_interval(&sensor_poll_interval_ms);

	while (1) {
		for (poll_num = 0; poll_num < SENSOR_NUM_MAX; poll_num++) {
			if (sensor_poll_enable_flag == 0) { /* skip if disable sensor poll */
				break;
			}
			if (sensor_config_index_map[poll_num] == SENSOR_NULL) { // sensor not exist
				continue;
			}
			get_sensor_reading(poll_num, &reading, GET_FROM_SENSOR);

			k_yield();
		}
		k_msleep(sensor_poll_interval_ms);
	}
}

__weak void pal_set_sensor_poll_interval(int *interval_ms)
{
	*interval_ms = 1000;
	return;
}

__weak void pal_fix_sensor_config(void)
{
	printf("Function %s is not implemented\n", __func__);
	return;
}

bool stby_access(uint8_t sensor_number)
{
	return 1;
}

bool DC_access(uint8_t sensor_number)
{
	return get_DC_on_delayed_status();
}

bool post_access(uint8_t sensor_number)
{
	return get_post_status();
}

void sensor_poll_init()
{
	k_thread_create(&sensor_poll, sensor_poll_stack, K_THREAD_STACK_SIZEOF(sensor_poll_stack),
			sensor_poll_handler, NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0,
			K_NO_WAIT);
	k_thread_name_set(&sensor_poll, "sensor_poll");
	return;
}

uint8_t get_sensor_config_index(uint8_t sensor_num)
{
	uint8_t i, j;
	for (i = 0; i < SENSOR_NUM_MAX; i++) {
		for (j = 0; j < sensor_config_num; ++j) {
			if (sensor_num == sensor_config[j].num) {
				return j;
			} else if (i == sensor_config_num) {
				return SENSOR_NUM_MAX;
			} else {
				continue;
			}
		}
	}

	return SENSOR_NUM_MAX;
}

void add_sensor_config(snr_cfg config)
{
	if (get_sensor_config_index(config.num) != SENSOR_NUM_MAX) {
		printf("add sensor num [%x] is already exists\n", config.num);
	} else {
		sensor_config[sensor_config_num++] = config;
	}
}

bool sensor_init(void)
{
	init_sensor_num();
	sdr_init();

	if (SDR_NUM != 0) {
		sensor_config = (snr_cfg *)malloc(SDR_NUM * sizeof(snr_cfg));
		if (sensor_config != NULL) {
			sensor_config_num = load_sensor_config();
		} else {
			printf("sensor_config alloc fail\n");
			return false;
		}
	} else {
		printf("SDR_NUM == 0\n");
		return false;
	}

	pal_fix_sensor_config();
	map_sensor_num_to_sdr_cfg();

	if (DEBUG_SNR) {
		printf("SNR0: %s\n", full_sdr_table[sdr_index_map[1]].ID_str);
	}

	if (enable_sensor_poll_thread) {
		sensor_poll_init();
	}

	return true;
}
