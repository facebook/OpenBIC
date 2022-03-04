#include "peci.h"

#include <drivers/peci.h>
#include <stdio.h>
#include <stdlib.h>

#include "sensor.h"
#include "hal_peci.h"
#include "plat_sensor_table.h"

#define PECI_READ_LEN 5

#ifdef CONFIG_PECI
static bool peci_get_power(uint8_t sensor_num, int *reading)
{
	snr_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
	uint8_t cmd = PECI_RD_PKG_CFG0_CMD;
	uint8_t *read_buf = malloc(2 * PECI_READ_LEN * sizeof(uint8_t));
	uint8_t index[2] = { 0x03, 0x1F };
	uint16_t param[2] = { 0x00FF, 0x0000 };
	uint8_t ret, complete_code;
	bool is_retry_success = false;
	uint32_t pkg_energy, run_time, diff_energy, diff_time;
	static uint32_t last_pkg_energy = 0, last_run_time = 0;

	if (read_buf == NULL) {
		printf("PECI buffer alloc fail\n");
		return false;
	}

	ret = peci_read(cmd, cfg->port, index[0], param[0], PECI_READ_LEN, read_buf);
	if (ret) {
		cfg->cache_status = SNR_FAIL_TO_ACCESS;
		free(read_buf);
		return false;
	}
	complete_code = read_buf[0];
	if (complete_code == PECI_CC_RSP_TIMEOUT ||
	    complete_code == PECI_CC_OUT_OF_RESOURCES_TIMEOUT) {
		is_retry_success = peci_retry_read(cmd, cfg->port, index[0], param[0],
						   PECI_READ_LEN, read_buf);
		if (!is_retry_success) {
			printf("PECI sensor [%x] response timeout. Reach Max retry.\n", sensor_num);
			cfg->cache_status = SNR_FAIL_TO_ACCESS;
			free(read_buf);
			return false;
		}
	}

	ret = peci_read(cmd, cfg->port, index[1], param[1], PECI_READ_LEN, &read_buf[5]);
	if (ret) {
		cfg->cache_status = SNR_FAIL_TO_ACCESS;
		free(read_buf);
		return false;
	}
	complete_code = read_buf[5];
	if (complete_code == PECI_CC_RSP_TIMEOUT ||
	    complete_code == PECI_CC_OUT_OF_RESOURCES_TIMEOUT) {
		is_retry_success = peci_retry_read(cmd, cfg->port, index[1], param[1],
						   PECI_READ_LEN, &read_buf[5]);
		if (!is_retry_success) {
			printf("PECI sensor [%x] response timeout. Reach Max retry.\n", sensor_num);
			cfg->cache_status = SNR_FAIL_TO_ACCESS;
			free(read_buf);
			return false;
		}
	}

	if ((read_buf[0] != PECI_CC_RSP_SUCCESS) || (read_buf[5] != PECI_CC_RSP_SUCCESS)) {
		if ((read_buf[0] == PECI_CC_ILLEGAL_REQUEST) ||
		    (read_buf[5] == PECI_CC_ILLEGAL_REQUEST)) {
			printf("PECI unknown request\n");
			cfg->cache_status = SNR_NOT_FOUND;
		} else {
			printf("PECI control hardware, firmware or associated logic error\n");
			cfg->cache_status = SNR_UNSPECIFIED_ERROR;
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
	int val, ret, complete_code;

	if (sensor_num == SENSOR_NUM_TEMP_CPU) { // CPU temp = tjmax - margin
		if ((sensor_config[sensor_config_index_map[SENSOR_NUM_TEMP_CPU_TJMAX]]
			     .cache_status == SNR_READ_SUCCESS) &&
		    (sensor_config[sensor_config_index_map[SENSOR_NUM_TEMP_CPU_MARGIN]]
			     .cache_status == SNR_READ_SUCCESS)) {
			*reading =
				sensor_config[sensor_config_index_map[SENSOR_NUM_TEMP_CPU_TJMAX]]
					.cache -
				sensor_config[sensor_config_index_map[SENSOR_NUM_TEMP_CPU_MARGIN]]
					.cache;
			cfg->cache = *reading;
			cfg->cache_status = SNR_READ_SUCCESS;
			return true;
		} else {
			cfg->cache_status = SNR_FAIL_TO_ACCESS;
			return false;
		}
	} else if (sensor_num == SENSOR_NUM_PWR_CPU) {
		ret = peci_get_power(sensor_num, &val);
		if (!ret) {
			cfg->cache_status = SNR_FAIL_TO_ACCESS;
			return false;
		}
		*reading = (float)val;
		cfg->cache = *reading;
		cfg->cache_status = SNR_READ_SUCCESS;
		return true;
	} else if ((sensor_num == SENSOR_NUM_TEMP_CPU_TJMAX) &&
		   (cfg->cache_status == SNR_READ_SUCCESS)) { // Tjmax only need to get once
		*reading = cfg->cache;
		return true;
	} else { // The other sensors
		if (cfg->port == NONE) {
			cfg->cache_status = SNR_NOT_FOUND;
			return false;
		}

		read_buf = (uint8_t *)malloc(PECI_READ_LEN * sizeof(uint8_t));
		if (read_buf == NULL) {
			printf("PECI buffer alloc fail\n");
			return false;
		}

		ret = peci_read(cmd, cfg->port, cfg->slave_addr, cfg->offset, PECI_READ_LEN,
				read_buf);
		if (ret) {
			cfg->cache_status = SNR_FAIL_TO_ACCESS;
			return false;
		}

		complete_code = read_buf[0];
		if (complete_code != PECI_CC_RSP_SUCCESS) {
			bool is_retry_success = false;
			if ((complete_code == PECI_CC_RSP_TIMEOUT) ||
			    (complete_code == PECI_CC_OUT_OF_RESOURCES_TIMEOUT)) {
				is_retry_success =
					peci_retry_read(cmd, cfg->port, cfg->slave_addr,
							cfg->offset, PECI_READ_LEN, read_buf);
				if (!is_retry_success) {
					printf("PECI sensor [%x] response timeout. Reach Max retry.\n",
					       sensor_num);
					cfg->cache_status = SNR_FAIL_TO_ACCESS;
					free(read_buf);
					return false;
				}
			} else if (complete_code == PECI_CC_ILLEGAL_REQUEST) {
				printf("PECI unknown request\n");
				cfg->cache_status = SNR_NOT_FOUND;
				free(read_buf);
				return false;
			} else {
				printf("PECI control hardware, firmware or associated logic error\n");
				cfg->cache_status = SNR_UNSPECIFIED_ERROR;
				free(read_buf);
				return false;
			}
		}

		// PECI_CC_RSP_SUCCESS
		if (sensor_num == SENSOR_NUM_TEMP_CPU_MARGIN) {
			val = ((0xFFFF - ((read_buf[2] << 8) | read_buf[1])) >> 6) + 1;
		} else if (sensor_num == SENSOR_NUM_TEMP_CPU_TJMAX) {
			val = read_buf[3];
		} else {
			val = read_buf[1];
		}

		free(read_buf);
		*reading = (float)val;
		cfg->cache = *reading;
		cfg->cache_status = SNR_READ_SUCCESS;
		return true;
	}

	return false;
}
#endif
