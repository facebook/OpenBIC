#include "peci.h"

#include <drivers/peci.h>
#include <stdio.h>
#include <stdlib.h>

#include "sensor.h"
#include "hal_peci.h"
#include "plat_sensor_table.h"

#define PECI_READ_LEN 5
#define PECI_RETRY_MAX 6

#ifdef CONFIG_PECI
static bool peci_get_power(uint8_t sensor_num, int *reading)
{
	snr_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
	uint8_t cmd = PECI_RD_PKG_CFG0_CMD;
	uint8_t *read_buf = malloc(2 * PECI_READ_LEN * sizeof(uint8_t));
	uint8_t index[2] = { 0x03, 0x1F };
	uint16_t param[2] = { 0x00FF, 0x0000 };
	uint8_t ret;
	uint32_t pkg_energy, run_time, diff_energy, diff_time;
	static uint32_t last_pkg_energy = 0, last_run_time = 0;

	if (read_buf == NULL) {
		printf("PECI buffer alloc fail\n");
		return false;
	}

	uint8_t *complete_code = &read_buf[0];
	int retry_count = PECI_RETRY_MAX;
	while (1) {
		ret = peci_read(cmd, cfg->port, index[0], param[0], PECI_READ_LEN, read_buf);
		if (ret == 0) {
			if (*complete_code == PECI_CC_RSP_SUCCESS) {
				break;
			}
		} else if (--retry_count) { // wait for 10ms before next retry
			k_msleep(10);
		} else { // all retries failed exit the loop
			break;
		}
	}
	if (retry_count == 0) {
		if (ret) {
			printf("PECI sensor [0x%x] retry read fail, retry time: %d, completion_code: 0x%x\n",
			       sensor_num, PECI_RETRY_MAX, *complete_code);
			cfg->cache_status = SENSOR_FAIL_TO_ACCESS;
		} else {
			switch (*complete_code) {
			case PECI_CC_RSP_TIMEOUT:
			case PECI_CC_OUT_OF_RESOURCES_TIMEOUT:
				printf("PECI sensor [%x] response timeout\n", sensor_num);
				cfg->cache_status = SENSOR_FAIL_TO_ACCESS;
				break;
			case PECI_CC_ILLEGAL_REQUEST:
				printf("PECI sensor [%x] unknown request\n", sensor_num);
				cfg->cache_status = SENSOR_NOT_FOUND;
				break;
			default:
				printf("PECI sensor [%x] control hardware, firmware or associated logic error\n",
				       sensor_num);
				cfg->cache_status = SENSOR_UNSPECIFIED_ERROR;
				break;
			}
		}

		free(read_buf);
		return false;
	}

	complete_code = &read_buf[PECI_READ_LEN];
	retry_count = PECI_RETRY_MAX;
	while (1) {
		ret = peci_read(cmd, cfg->port, index[1], param[1], PECI_READ_LEN,
				&read_buf[PECI_READ_LEN]);
		if (ret == 0) {
			if (*complete_code == PECI_CC_RSP_SUCCESS) {
				break;
			}
		} else if (--retry_count) { // wait for 10ms before next retry
			k_msleep(10);
		} else { // all retries failed exit the loop
			break;
		}
	}
	if (retry_count == 0) {
		if (ret) {
			printf("PECI sensor [0x%x] retry read fail, retry time: %d, completion_code: 0x%x\n",
			       sensor_num, PECI_RETRY_MAX, *complete_code);
			cfg->cache_status = SENSOR_FAIL_TO_ACCESS;
		} else {
			switch (*complete_code) {
			case PECI_CC_RSP_TIMEOUT:
			case PECI_CC_OUT_OF_RESOURCES_TIMEOUT:
				printf("PECI sensor [0x%x] response timeout\n", sensor_num);
				cfg->cache_status = SENSOR_FAIL_TO_ACCESS;
				break;
			case PECI_CC_ILLEGAL_REQUEST:
				printf("PECI sensor [0x%x] unknown request\n", sensor_num);
				cfg->cache_status = SENSOR_NOT_FOUND;
				break;
			default:
				printf("PECI sensor [0x%x] control hardware, firmware or associated logic error\n",
				       sensor_num);
				cfg->cache_status = SENSOR_UNSPECIFIED_ERROR;
				break;
			}
		}

		free(read_buf);
		return false;
	}

	pkg_energy = read_buf[4];
	pkg_energy = (pkg_energy << 8) | read_buf[3];
	pkg_energy = (pkg_energy << 8) | read_buf[2];
	pkg_energy = (pkg_energy << 8) | read_buf[1];

	run_time = read_buf[9];
	run_time = (run_time << 8) | read_buf[8];
	run_time = (run_time << 8) | read_buf[7];
	run_time = (run_time << 8) | read_buf[6];

	// first read, need second data to calculate
	if ((last_pkg_energy == 0) && (last_run_time == 0)) {
		last_pkg_energy = pkg_energy;
		last_run_time = run_time;
		free(read_buf);
		return false;
	}

	if (pkg_energy >= last_pkg_energy) {
		diff_energy = pkg_energy - last_pkg_energy;
	} else {
		diff_energy = pkg_energy + (0xffffffff - last_pkg_energy + 1);
	}
	last_pkg_energy = pkg_energy;

	if (run_time >= last_run_time) {
		diff_time = run_time - last_run_time;
	} else {
		diff_time = run_time + (0xffffffff - last_run_time + 1);
	}
	last_run_time = run_time;

	if (diff_time == 0) {
		free(read_buf);
		return false;
	} else {
		// energy / unit time
		*reading = ((float)diff_energy / (float)diff_time * 0.06103515625);
		free(read_buf);
		return true;
	}
}

bool peci_sensor_read(uint8_t sensor_num, float *reading)
{
	snr_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
	uint8_t *read_buf;
	uint8_t cmd = PECI_RD_PKG_CFG0_CMD;
	int val, ret;

	if (sensor_num == SENSOR_NUM_TEMP_CPU) { // CPU temp = tjmax - margin
		if ((sensor_config[sensor_config_index_map[SENSOR_NUM_TEMP_CPU_TJMAX]]
			     .cache_status == SENSOR_READ_SUCCESS) &&
		    (sensor_config[sensor_config_index_map[SENSOR_NUM_TEMP_CPU_MARGIN]]
			     .cache_status == SENSOR_READ_SUCCESS)) {
			*reading =
				sensor_config[sensor_config_index_map[SENSOR_NUM_TEMP_CPU_TJMAX]]
					.cache -
				sensor_config[sensor_config_index_map[SENSOR_NUM_TEMP_CPU_MARGIN]]
					.cache;
			cfg->cache = *reading;
			cfg->cache_status = SENSOR_READ_SUCCESS;
			return true;
		} else {
			cfg->cache_status = SENSOR_FAIL_TO_ACCESS;
			return false;
		}
	} else if (sensor_num == SENSOR_NUM_PWR_CPU) {
		ret = peci_get_power(sensor_num, &val);
		if (!ret) {
			cfg->cache_status = SENSOR_FAIL_TO_ACCESS;
			return false;
		}
		*reading = (float)val;
		cfg->cache = *reading;
		cfg->cache_status = SENSOR_READ_SUCCESS;
		return true;
	} else if ((sensor_num == SENSOR_NUM_TEMP_CPU_TJMAX) &&
		   (cfg->cache_status == SENSOR_READ_SUCCESS)) { // Tjmax only need to get once
		*reading = cfg->cache;
		return true;
	} else { // The other sensors
		if (cfg->port == NULL) {
			cfg->cache_status = SENSOR_NOT_FOUND;
			return false;
		}

		read_buf = (uint8_t *)malloc(PECI_READ_LEN * sizeof(uint8_t));
		if (read_buf == NULL) {
			printf("PECI buffer alloc fail\n");
			return false;
		}

		uint8_t *complete_code = &read_buf[0];
		int retry_count = PECI_RETRY_MAX;
		while (1) {
			ret = peci_read(cmd, cfg->port, cfg->target_addr, cfg->offset,
					PECI_READ_LEN, read_buf);
			if (ret == 0) {
				if (*complete_code == PECI_CC_RSP_SUCCESS) {
					break;
				}
			} else if (--retry_count) { // wait for 10ms before next retry
				k_msleep(10);
			} else { // all retries failed exit the loop
				break;
			}
		}
		if (retry_count == 0) {
			if (ret) {
				printf("PECI sensor [0x%x] retry read fail, retry time: %d, completion_code: 0x%x\n",
				       sensor_num, PECI_RETRY_MAX, *complete_code);
				cfg->cache_status = SENSOR_FAIL_TO_ACCESS;
			} else {
				switch (*complete_code) {
				case PECI_CC_RSP_TIMEOUT:
				case PECI_CC_OUT_OF_RESOURCES_TIMEOUT:
					printf("PECI sensor [0x%x] response timeout\n", sensor_num);
					cfg->cache_status = SENSOR_FAIL_TO_ACCESS;
					break;
				case PECI_CC_ILLEGAL_REQUEST:
					printf("PECI sensor [0x%x] unknown request\n", sensor_num);
					cfg->cache_status = SENSOR_NOT_FOUND;
					break;
				default:
					printf("PECI sensor [0x%x] control hardware, firmware or associated logic error\n",
					       sensor_num);
					cfg->cache_status = SENSOR_UNSPECIFIED_ERROR;
					break;
				}
			}

			free(read_buf);
			return false;
		}

		// PECI_CC_RSP_SUCCESS
		if (sensor_num == SENSOR_NUM_TEMP_CPU_MARGIN) {
			val = ((0xFFFF - ((read_buf[2] << 8) | read_buf[1])) >> 6) + 1;
		} else if (sensor_num == SENSOR_NUM_TEMP_CPU_TJMAX) {
			val = read_buf[3];
		} else {
			val = read_buf[1];
		}

		*reading = (float)val;
		cfg->cache = *reading;
		cfg->cache_status = SENSOR_READ_SUCCESS;
		free(read_buf);
		return true;
	}
}
#endif
