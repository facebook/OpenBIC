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
#include "util_sys.h"
#include "plat_def.h"
#include "libutil.h"

#define SENSOR_DRIVE_INIT_DECLARE(name) uint8_t name##_init(uint8_t sensor_num)

#define SENSOR_DRIVE_TYPE_INIT_MAP(name)                                                           \
	{                                                                                          \
		sensor_dev_##name, name##_init                                                     \
	}

#define SENSOR_READ_RETRY_MAX 3

extern sensor_cfg plat_sensor_config[];
extern const int SENSOR_CONFIG_SIZE;

struct k_thread sensor_poll;
K_KERNEL_STACK_MEMBER(sensor_poll_stack, SENSOR_POLL_STACK_SIZE);

uint8_t sensor_config_index_map[SENSOR_NUM_MAX];
uint8_t sdr_index_map[SENSOR_NUM_MAX];

bool enable_sensor_poll_thread = true;
static bool sensor_poll_enable_flag = true;

static bool is_sensor_ready_flag = false;

const int negative_ten_power[16] = { 1,	    1,		1,	   1,	     1,	      1,
				     1,	    1000000000, 100000000, 10000000, 1000000, 100000,
				     10000, 1000,	100,	   10 };

sensor_cfg *sensor_config;
uint8_t sensor_config_count;

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
SENSOR_DRIVE_INIT_DECLARE(tmp431);
SENSOR_DRIVE_INIT_DECLARE(pmic);
SENSOR_DRIVE_INIT_DECLARE(ina233);
SENSOR_DRIVE_INIT_DECLARE(isl69254iraz_t);
SENSOR_DRIVE_INIT_DECLARE(max16550a);
SENSOR_DRIVE_INIT_DECLARE(ina230);
SENSOR_DRIVE_INIT_DECLARE(xdpe12284c);
SENSOR_DRIVE_INIT_DECLARE(raa229621);
SENSOR_DRIVE_INIT_DECLARE(nct7718w);
SENSOR_DRIVE_INIT_DECLARE(ltc4286);
#ifdef ENABLE_APML
SENSOR_DRIVE_INIT_DECLARE(amd_tsi);
SENSOR_DRIVE_INIT_DECLARE(apml_mailbox);
#endif
SENSOR_DRIVE_INIT_DECLARE(xdpe19283b);
SENSOR_DRIVE_INIT_DECLARE(g788p81u);
SENSOR_DRIVE_INIT_DECLARE(mp2856gut);

struct sensor_drive_api {
	enum SENSOR_DEV dev;
	uint8_t (*init)(uint8_t);
} sensor_drive_tbl[] = {
	SENSOR_DRIVE_TYPE_INIT_MAP(tmp75),
	SENSOR_DRIVE_TYPE_INIT_MAP(ast_adc),
	SENSOR_DRIVE_TYPE_INIT_MAP(isl69259),
	SENSOR_DRIVE_TYPE_INIT_MAP(nvme),
	SENSOR_DRIVE_TYPE_INIT_MAP(mp5990),
	SENSOR_DRIVE_TYPE_INIT_MAP(isl28022),
	SENSOR_DRIVE_TYPE_INIT_MAP(pex89000),
	SENSOR_DRIVE_TYPE_INIT_MAP(intel_peci),
	SENSOR_DRIVE_TYPE_INIT_MAP(pch),
	SENSOR_DRIVE_TYPE_INIT_MAP(adm1278),
	SENSOR_DRIVE_TYPE_INIT_MAP(tps53689),
	SENSOR_DRIVE_TYPE_INIT_MAP(xdpe15284),
	SENSOR_DRIVE_TYPE_INIT_MAP(ltc4282),
	SENSOR_DRIVE_TYPE_INIT_MAP(tmp431),
	SENSOR_DRIVE_TYPE_INIT_MAP(pmic),
	SENSOR_DRIVE_TYPE_INIT_MAP(ina233),
	SENSOR_DRIVE_TYPE_INIT_MAP(isl69254iraz_t),
	SENSOR_DRIVE_TYPE_INIT_MAP(max16550a),
	SENSOR_DRIVE_TYPE_INIT_MAP(ina230),
	SENSOR_DRIVE_TYPE_INIT_MAP(xdpe12284c),
	SENSOR_DRIVE_TYPE_INIT_MAP(raa229621),
	SENSOR_DRIVE_TYPE_INIT_MAP(nct7718w),
	SENSOR_DRIVE_TYPE_INIT_MAP(ltc4286),
#ifdef ENABLE_APML
	SENSOR_DRIVE_TYPE_INIT_MAP(amd_tsi),
	SENSOR_DRIVE_TYPE_INIT_MAP(apml_mailbox),
#endif
	SENSOR_DRIVE_TYPE_INIT_MAP(xdpe19283b),
	SENSOR_DRIVE_TYPE_INIT_MAP(g788p81u),
	SENSOR_DRIVE_TYPE_INIT_MAP(mp2856gut),
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
		for (j = 0; j < sdr_count; j++) {
			if (i == full_sdr_table[j].sensor_num) {
				sdr_index_map[i] = j;
				break;
			} else if (i == sdr_count) {
				sdr_index_map[i] = SENSOR_NULL;
			} else {
				continue;
			}
		}
		for (j = 0; j < sdr_count; j++) {
			if (i == sensor_config[j].num) {
				sensor_config_index_map[i] = j;
				break;
			} else if (i == sdr_count) {
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

void clear_unaccessible_sensor_cache(uint8_t sensor_num)
{
	if (sensor_config[sensor_config_index_map[sensor_num]].cache_status != SENSOR_INIT_STATUS) {
		sensor_config[sensor_config_index_map[sensor_num]].cache = SENSOR_FAIL;
		sensor_config[sensor_config_index_map[sensor_num]].cache_status =
			SENSOR_INIT_STATUS;
	}
}

uint8_t get_sensor_reading(uint8_t sensor_num, int *reading, uint8_t read_mode)
{
	if (reading == NULL) {
		printf("[%s] input pointer reading is NULL\n", __func__);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	// Check sensor information in sensor config table
	// Block BMC send invalid sensor number by OEM accurate read command
	if (sensor_config_index_map[sensor_num] == SENSOR_FAIL) {
		printf("[%s] fail to find sensor info in config table, sensor_num: 0x%x\n",
		       __func__, sensor_num);
		return SENSOR_NOT_FOUND;
	}

	*reading = 0; // Initial return reading value
	uint8_t current_status = SENSOR_UNSPECIFIED_ERROR;
	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
	bool post_ret = false;

	if (!access_check(sensor_num)) { // sensor not accessable
		clear_unaccessible_sensor_cache(sensor_num);
		cfg->cache_status = SENSOR_NOT_ACCESSIBLE;
		return cfg->cache_status;
	}

	switch (read_mode) {
	case GET_FROM_SENSOR:
		if (cfg->pre_sensor_read_hook) {
			if (cfg->pre_sensor_read_hook(sensor_num, cfg->pre_sensor_read_args) ==
			    false) {
				printf("Failed to do pre sensor read function, sensor number: 0x%x\n",
				       sensor_num);
				cfg->cache_status = SENSOR_PRE_READ_ERROR;
				return cfg->cache_status;
			}
			if (cfg->cache_status == SENSOR_NOT_PRESENT) {
				return cfg->cache_status;
			}
		}

		if (cfg->read) {
			current_status = cfg->read(sensor_num, reading);
		}

		if (current_status == SENSOR_READ_SUCCESS ||
		    current_status == SENSOR_READ_ACUR_SUCCESS) {
			cfg->retry = 0;
			if (cfg->post_sensor_read_hook) { // makesure post hook function be called
				post_ret = cfg->post_sensor_read_hook(
					sensor_num, cfg->post_sensor_read_args, reading);
			}

			if (!access_check(
				    sensor_num)) { // double check access to avoid not accessible read at same moment status change
				clear_unaccessible_sensor_cache(sensor_num);
				cfg->cache_status = SENSOR_NOT_ACCESSIBLE;
				return cfg->cache_status;
			}

			if (cfg->post_sensor_read_hook && post_ret == false) {
				printf("Failed to do post sensor read function, sensor number: 0x%x\n",
				       sensor_num);
				cfg->cache_status = SENSOR_POST_READ_ERROR;
				return cfg->cache_status;
			}
			memcpy(&cfg->cache, reading, sizeof(*reading));
			cfg->cache_status = SENSOR_READ_4BYTE_ACUR_SUCCESS;
			return cfg->cache_status;
		} else {
			/* If sensor read fails, let the reading argument in the
       * post_sensor_read_hook function to NULL.
       * All post_sensor_read_hook function define in each platform should check
       * reading whether is NULL to do the corresponding thing. (Ex: mutex_unlock)
       */
			if (cfg->post_sensor_read_hook) {
				if (cfg->post_sensor_read_hook(sensor_num,
							       cfg->post_sensor_read_args,
							       NULL) == false) {
					printf("[%s]sensor number 0x%x reading and post_read fail\n",
					       __func__, sensor_num);
				}
			}
			/* common retry */
			// Return current status if retry reach max retry count, otherwise return cache status instead of current status
			if (cfg->retry >= SENSOR_READ_RETRY_MAX) {
				cfg->cache_status = current_status;
			} else {
				cfg->retry++;
			}
			return cfg->cache_status;
		}
		break;
	case GET_FROM_CACHE:
		switch (cfg->cache_status) {
		case SENSOR_READ_SUCCESS:
		case SENSOR_READ_ACUR_SUCCESS:
		case SENSOR_READ_4BYTE_ACUR_SUCCESS:
			*reading = cfg->cache;
			if (!access_check(
				    sensor_num)) { // double check access to avoid not accessible read at same moment status change
				cfg->cache_status = SENSOR_NOT_ACCESSIBLE;
			}
			return cfg->cache_status;
			;
		case SENSOR_INIT_STATUS:
		case SENSOR_NOT_PRESENT:
		case SENSOR_NOT_ACCESSIBLE:
			cfg->cache = SENSOR_FAIL;
			return cfg->cache_status;
		default:
			cfg->cache = SENSOR_FAIL;
			printf("Failed to read sensor value from cache, sensor number: 0x%x, cache status: 0x%x\n",
			       sensor_num, cfg->cache_status);
			return cfg->cache_status;
		}
		break;
	default:
		printf("Invalid mbr type during changing sensor mbr\n");
		break;
	}

	cfg->cache_status = current_status;
	return cfg->cache_status;
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
	uint8_t index = 0, sensor_num = 0;
	int sensor_poll_interval_ms;
	int reading;

	k_msleep(1000); // delay 1 second to wait for drivers ready before start sensor polling

	pal_set_sensor_poll_interval(&sensor_poll_interval_ms);

	while (1) {
		for (index = 0; index < sensor_config_count; index++) {
			// Perform sensor polling according to the sensor number of the sensor config table
			sensor_num = sensor_config[index].num;
			if (sensor_poll_enable_flag == false) { /* skip if disable sensor poll */
				break;
			}

			sensor_cfg *config = &sensor_config[sensor_config_index_map[sensor_num]];
			if (config->cache_status == SENSOR_NOT_PRESENT) {
				continue;
			}

			// Check whether monitoring sensor is enabled
			if (config->is_enable_polling == DISABLE_SENSOR_POLLING) {
				config->cache = SENSOR_FAIL;
				config->cache_status = SENSOR_POLLING_DISABLE;
				continue;
			}

			if (sdr_index_map[sensor_num] == SENSOR_NULL) { // Check sensor info
				printf("[%s] fail to find sensor SDR info, sensor number: 0x%x\n",
				       __func__, sensor_num);
				continue;
			}

			if (sensor_config[index].poll_time != POLL_TIME_DEFAULT) {
				if (pal_is_time_to_poll(sensor_num,
							sensor_config[index].poll_time) == false) {
					continue;
				}
			}

			get_sensor_reading(sensor_num, &reading, GET_FROM_SENSOR);

			k_yield();
		}

		is_sensor_ready_flag = true;

		k_msleep(sensor_poll_interval_ms);
	}
}

__weak bool pal_is_time_to_poll(uint8_t sensor_num, int poll_time)
{
	return true;
}

__weak void pal_set_sensor_poll_interval(int *interval_ms)
{
	*interval_ms = 1000;
	return;
}

__weak void pal_extend_sensor_config(void)
{
	return;
}

__weak uint8_t pal_get_extend_sdr()
{
	return 0;
}

__weak uint8_t pal_get_extend_sensor_config()
{
	return 0;
}

void check_init_sensor_size()
{
	uint8_t init_sdr_size = plat_get_sdr_size();
	uint8_t init_sensor_config_size = plat_get_config_size();
	uint8_t extend_sdr_size = pal_get_extend_sdr();
	uint8_t extend_sensor_config_size = pal_get_extend_sensor_config();

	init_sdr_size += extend_sdr_size;
	init_sensor_config_size += extend_sensor_config_size;

	if (init_sdr_size != init_sensor_config_size) {
		enable_sensor_poll_thread = false;
		printf("[%s] init sdr size is not equal to config size, sdr size: 0x%x, config size: 0x%x\n",
		       __func__, init_sdr_size, init_sensor_config_size);
		printf("BIC should not monitor sensors if SDR size and sensor config size is not match, BIC would not start sensor thread\n");
		return;
	}
	sensor_config_size = init_sdr_size;
}

bool stby_access(uint8_t sensor_num)
{
	return true;
}

bool dc_access(uint8_t sensor_num)
{
	return get_DC_on_delayed_status();
}

bool post_access(uint8_t sensor_num)
{
	return get_post_status();
}

bool me_access(uint8_t sensor_num)
{
	if (get_me_mode() == ME_NORMAL_MODE) {
		return get_post_status();
	} else {
		return false;
	}
}

bool vr_access(uint8_t sensor_num)
{
	if (get_DC_on_delayed_status() == false) {
		return false;
	}
	return get_vr_monitor_status();
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
	uint8_t i = 0;
	for (i = 0; i < sensor_config_count; ++i) {
		if (sensor_num == sensor_config[i].num) {
			return i;
		}
	}
	return SENSOR_NUM_MAX;
}

void add_sensor_config(sensor_cfg config)
{
	uint8_t index = get_sensor_config_index(config.num);
	if (index != SENSOR_NUM_MAX) {
		memcpy(&sensor_config[index], &config, sizeof(sensor_cfg));
		printf("[%s] replace the sensor[0x%02x] configuration\n", __func__, config.num);
		return;
	}
	// Check config table size before adding sensor config
	if (sensor_config_count + 1 <= sdr_count) {
		sensor_config[sensor_config_count++] = config;
	} else {
		printf("[%s] add config would over config max size\n", __func__);
	}
}

static inline bool init_drive_type(sensor_cfg *p, uint16_t current_drive)
{
	int ret = -1;
	if (p->type != sensor_drive_tbl[current_drive].dev) {
		return false;
	}

	if (p->pre_sensor_read_hook) {
		if (p->pre_sensor_read_hook(p->num, p->pre_sensor_read_args) == false) {
			printf("[%s] sensor 0x%x pre sensor read failed!\n", __func__, p->num);
			return false;
		}
	}

	ret = sensor_drive_tbl[current_drive].init(p->num);
	if (ret != SENSOR_INIT_SUCCESS) {
		printf("sensor num %d initial fail, ret %d\n", p->num, ret);
	}

	if (p->post_sensor_read_hook) {
		if (p->post_sensor_read_hook(p->num, p->post_sensor_read_args, NULL) == false) {
			printf("[%s] sensor 0x%x post sensor read failed!\n", __func__, p->num);
		}
	}

	return true;
}

static void drive_init(void)
{
	const uint16_t max_drive_num = ARRAY_SIZE(sensor_drive_tbl);
	uint16_t current_drive;

	for (int i = 0; i < sdr_count; i++) {
		sensor_cfg *p = sensor_config + i;
		for (current_drive = 0; current_drive < max_drive_num; current_drive++) {
			if (init_drive_type(p, current_drive)) {
				break;
			}
		}

		if (current_drive == max_drive_num) {
			printf("sensor %d, type = %d is not supported!\n", i, p->type);
			p->read = NULL;
		}
	}
}

bool sensor_init(void)
{
	init_sensor_num();
	// Check init SDR size is equal to sensor config size
	check_init_sensor_size();
	if (sensor_config_size != 0) {
		full_sdr_table =
			(SDR_Full_sensor *)malloc(sensor_config_size * sizeof(SDR_Full_sensor));
		if (full_sdr_table != NULL) {
			sdr_init();
		} else {
			printf("[%s] fail to allocate memory to SDR table\n", __func__);
			return false;
		}
	} else {
		printf("[%s] init sensor size is zero\n", __func__);
		return false;
	}

	if (sdr_count != 0) {
		sensor_config = (sensor_cfg *)malloc(sdr_count * sizeof(sensor_cfg));
		if (sensor_config != NULL) {
			load_sensor_config();
		} else {
			SAFE_FREE(full_sdr_table);
			printf("[%s] fail to allocate memory to config table\n", __func__);
			return false;
		}
	} else {
		printf("[%s] SDR number is zero\n", __func__);
		return false;
	}

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

bool check_is_sensor_ready()
{
	return is_sensor_ready_flag;
}

uint8_t plat_get_config_size()
{
	return SENSOR_CONFIG_SIZE;
}

__weak void load_sensor_config(void)
{
	memcpy(sensor_config, plat_sensor_config, sizeof(sensor_cfg) * SENSOR_CONFIG_SIZE);
	sensor_config_count = SENSOR_CONFIG_SIZE;
}

void control_sensor_polling(uint8_t sensor_num, uint8_t optional, uint8_t cache_status)
{
	if ((sensor_num == SENSOR_NOT_SUPPORT) ||
	    (sensor_config_index_map[sensor_num] == SENSOR_FAIL)) {
		return;
	}

	if ((optional != DISABLE_SENSOR_POLLING) && (optional != ENABLE_SENSOR_POLLING)) {
		printf("[%s] input optional is not support, optional: %d\n", __func__, optional);
		return;
	}

	sensor_cfg *config = &sensor_config[sensor_config_index_map[sensor_num]];
	config->is_enable_polling = optional;
	config->cache_status = cache_status;
}
