#include "sensor.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "power_status.h"
#include "sdr.h"
#include "hal_i2c.h"
#include "plat_sensor_table.h"
#include "plat_sdr_table.h"
#include "ast_adc.h"
#include "intel_peci.h"

#define SENSOR_DRIVE_INIT_DECLARE(name) uint8_t name##_init(uint8_t sensor_num)

#define SENSOR_DRIVE_TYPE_INIT_MAP(name)                                                           \
	{                                                                                          \
		sensor_dev_##name, name##_init                                                     \
	}

#define SENSOR_READ_RETRY_MAX 3

struct k_thread sensor_poll;
K_KERNEL_STACK_MEMBER(sensor_poll_stack, SENSOR_POLL_STACK_SIZE);

uint8_t sensor_config_index_map[SENSOR_NUM_MAX];
uint8_t sdr_index_map[SENSOR_NUM_MAX];

bool enable_sensor_poll_thread = true;
static bool sensor_poll_enable_flag = true;

const int negative_ten_power[16] = { 1,	    1,		1,	   1,	     1,	      1,
				     1,	    1000000000, 100000000, 10000000, 1000000, 100000,
				     10000, 1000,	100,	   10 };

sensor_cfg *sensor_config;
uint8_t sensor_config_num;

SENSOR_DRIVE_INIT_DECLARE(tmp75);
SENSOR_DRIVE_INIT_DECLARE(ast_adc);
SENSOR_DRIVE_INIT_DECLARE(isl69259);
SENSOR_DRIVE_INIT_DECLARE(nvme);
SENSOR_DRIVE_INIT_DECLARE(mp5990);
SENSOR_DRIVE_INIT_DECLARE(isl28022);
SENSOR_DRIVE_INIT_DECLARE(pex89000);
SENSOR_DRIVE_INIT_DECLARE(intel_peci);
SENSOR_DRIVE_INIT_DECLARE(pch);
SENSOR_DRIVE_INIT_DECLARE(adm1278);
SENSOR_DRIVE_INIT_DECLARE(tps53689);
SENSOR_DRIVE_INIT_DECLARE(xdpe15284);
SENSOR_DRIVE_INIT_DECLARE(ltc4282);

struct sensor_drive_api {
	enum SENSOR_DEV dev;
	uint8_t (*init)(uint8_t);
} sensor_drive_tbl[] = {
	SENSOR_DRIVE_TYPE_INIT_MAP(tmp75),    SENSOR_DRIVE_TYPE_INIT_MAP(ast_adc),
	SENSOR_DRIVE_TYPE_INIT_MAP(isl69259), SENSOR_DRIVE_TYPE_INIT_MAP(nvme),
	SENSOR_DRIVE_TYPE_INIT_MAP(mp5990),   SENSOR_DRIVE_TYPE_INIT_MAP(isl28022),
	SENSOR_DRIVE_TYPE_INIT_MAP(pex89000), SENSOR_DRIVE_TYPE_INIT_MAP(intel_peci),
	SENSOR_DRIVE_TYPE_INIT_MAP(pch),      SENSOR_DRIVE_TYPE_INIT_MAP(adm1278),
	SENSOR_DRIVE_TYPE_INIT_MAP(tps53689), SENSOR_DRIVE_TYPE_INIT_MAP(xdpe15284),
	SENSOR_DRIVE_TYPE_INIT_MAP(ltc4282),
};

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

uint8_t get_sensor_reading(uint8_t sensor_num, int *reading, uint8_t read_mode)
{
	if (reading == NULL) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	if (sdr_index_map[sensor_num] == 0xFF) { // look for sensor in SDR table
		return SENSOR_NOT_FOUND;
	}

	if (!access_check(sensor_num)) { // sensor not accessable
		return SENSOR_NOT_ACCESSIBLE;
	}

	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
	switch (read_mode) {
	case GET_FROM_SENSOR:
		if (cfg->pre_sensor_read_hook) {
			if (cfg->pre_sensor_read_hook(sensor_num, cfg->pre_sensor_read_args) ==
			    false) {
				printf("Failed to do pre sensor read function, sensor number: 0x%x\n",
				       sensor_num);
				return SENSOR_PRE_READ_ERROR;
			}
		}

		int status = SENSOR_READ_API_UNREGISTER;
		if (cfg->read) {
			status = cfg->read(sensor_num, reading);
		}

		if (status == SENSOR_READ_SUCCESS || status == SENSOR_READ_ACUR_SUCCESS) {
			cfg->retry = 0;
			if (!access_check(
				    sensor_num)) { // double check access to avoid not accessible read at same moment status change
				return SENSOR_NOT_ACCESSIBLE;
			}

			if (cfg->post_sensor_read_hook) {
				if (cfg->post_sensor_read_hook(sensor_num,
							       cfg->post_sensor_read_args,
							       reading) == false) {
					printf("Failed to do post sensor read function, sensor number: 0x%x\n",
					       sensor_num);
					cfg->cache_status = SENSOR_POST_READ_ERROR;
					return SENSOR_POST_READ_ERROR;
				}
			}
			memcpy(&cfg->cache, reading, sizeof(*reading));
			status = SENSOR_READ_4BYTE_ACUR_SUCCESS;
			cfg->cache_status = status;
			return cfg->cache_status;
		} else {
			/* common retry */
			if (cfg->retry >= SENSOR_READ_RETRY_MAX)
				cfg->cache_status = status;
			else
				cfg->retry++;

			return cfg->cache_status;
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
	int reading;
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
	return true;
}

bool dc_access(uint8_t sensor_number)
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

void add_sensor_config(sensor_cfg config)
{
	if (get_sensor_config_index(config.num) != SENSOR_NUM_MAX) {
		printf("Failed to add sensor because the sensor number(0x%x) is already exists\n",
		       config.num);
	} else {
		sensor_config[sensor_config_num++] = config;
	}
}

static void drive_init(void)
{
	uint16_t drive_num = ARRAY_SIZE(sensor_drive_tbl);
	uint16_t i, j;
	uint8_t ret;

	for (i = 0; i < SDR_NUM; i++) {
		sensor_cfg *p = sensor_config + i;
		for (j = 0; j < drive_num; j++) {
			if (p->type == sensor_drive_tbl[j].dev) {
				ret = sensor_drive_tbl[j].init(p->num);
				if (ret != SENSOR_INIT_SUCCESS)
					printf("sensor num %d initial fail, ret %d\n", p->num, ret);
				break;
			}
		}

		if (j == drive_num) {
			printf("sensor %d, type = %d is not supported!\n", i, p->type);
			p->read = NULL;
		}
	}
}

bool sensor_init(void)
{
	init_sensor_num();
	sdr_init();

	if (SDR_NUM != 0) {
		sensor_config = (sensor_cfg *)malloc(SDR_NUM * sizeof(sensor_cfg));
		if (sensor_config != NULL) {
			sensor_config_num = load_sensor_config();
		} else {
			printf("Failed to allocate memory\n");
			return false;
		}
	} else {
		printf("The SDR number is equal to 0\n");
		return false;
	}

	pal_fix_sensor_config();
	map_sensor_num_to_sdr_cfg();

	/* register read api of sensor_config */
	drive_init();

	if (DEBUG_SENSOR) {
		printf("[%s] sensor name: %s\n", __func__, full_sdr_table[sdr_index_map[1]].ID_str);
	}

	if (enable_sensor_poll_thread) {
		sensor_poll_init();
	}

	return true;
}
