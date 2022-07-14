#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <drivers/peci.h>
#include "sensor.h"
#include "hal_peci.h"
#include "libutil.h"
#include "intel_peci.h"

#define RDPKG_IDX_PKG_TEMP 0x02
#define RDPKG_IDX_DIMM_TEMP 0x0E
#define RDPKG_IDX_TJMAX_TEMP 0x10

#define DIMM_TEMP_OFS_0 0x01
#define DIMM_TEMP_OFS_1 0x02

static bool read_cpu_power(uint8_t addr, int *reading)
{
	if (reading == NULL) {
		return false;
	}

	uint8_t command = PECI_RD_PKG_CFG0_CMD;
	uint8_t readlen = 0x05;
	uint8_t *readbuf = (uint8_t *)malloc(2 * readlen * sizeof(uint8_t));
	uint8_t u8index[2] = { 0x03, 0x1F };
	uint16_t u16Param[2] = { 0x00FF, 0x0000 };
	int ret = 0;
	uint32_t pkg_energy, run_time, diff_energy, diff_time;
	static uint32_t last_pkg_energy = 0, last_run_time = 0;

	ret = peci_read(command, addr, u8index[0], u16Param[0], readlen, readbuf);
	if (ret) {
		goto cleanup;
	}
	ret = peci_read(command, addr, u8index[1], u16Param[1], readlen, &readbuf[5]);
	if (ret) {
		goto cleanup;
	}
	if (readbuf[0] != PECI_CC_RSP_SUCCESS || readbuf[5] != PECI_CC_RSP_SUCCESS) {
		if (readbuf[0] == PECI_CC_ILLEGAL_REQUEST ||
		    readbuf[5] == PECI_CC_ILLEGAL_REQUEST) {
			printf("Unknown request\n");
		} else {
			printf("PECI control hardware, firmware or associated logic error\n");
		}
		goto cleanup;
	}

	pkg_energy = readbuf[4];
	pkg_energy = (pkg_energy << 8) | readbuf[3];
	pkg_energy = (pkg_energy << 8) | readbuf[2];
	pkg_energy = (pkg_energy << 8) | readbuf[1];

	run_time = readbuf[9];
	run_time = (run_time << 8) | readbuf[8];
	run_time = (run_time << 8) | readbuf[7];
	run_time = (run_time << 8) | readbuf[6];

	if (last_pkg_energy == 0 &&
	    last_run_time == 0) { // first read, need second data to calculate
		last_pkg_energy = pkg_energy;
		last_run_time = run_time;
		goto cleanup;
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
		goto cleanup;
	} else {
		*reading = ((float)diff_energy / (float)diff_time *
			    0.06103515625); // energy / unit time
		SAFE_FREE(readbuf);
		return true;
	}
cleanup:
	SAFE_FREE(readbuf);
	return false;
}

static bool get_cpu_tjmax(uint8_t addr, int *reading)
{
	if (!reading) {
		return false;
	}

	const uint16_t param = 0x00;
	const uint8_t rlen = 0x05;
	uint8_t rbuf[rlen];
	memset(rbuf, 0, sizeof(rbuf));

	int ret = peci_read(PECI_CMD_RD_PKG_CFG0, addr, RDPKG_IDX_TJMAX_TEMP, param, rlen, rbuf);
	if (ret != 0) {
		return false;
	}

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = rbuf[3];
	return true;
}

static bool get_cpu_margin(uint8_t addr, int *reading)
{
	if (!reading) {
		return false;
	}

	const uint16_t param = 0xFF;
	const uint8_t rlen = 0x05;
	uint8_t rbuf[rlen];
	memset(rbuf, 0, sizeof(rbuf));

	int ret = peci_read(PECI_CMD_RD_PKG_CFG0, addr, RDPKG_IDX_PKG_TEMP, param, rlen, rbuf);
	if (ret != 0) {
		return false;
	}

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = ((int16_t)((rbuf[2] << 8) | rbuf[1]) >> 6) + 1;
	return true;
}

static bool get_cpu_pwr(uint8_t addr, int *reading)
{
	if (!reading) {
		return false;
	}

	int pwr = 0;
	if (read_cpu_power(addr, &pwr) == false) {
		return false;
	}

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (int16_t)pwr;
	return true;
}

static bool get_cpu_temp(uint8_t addr, int *reading)
{
	if (!reading) {
		return false;
	}

	sensor_val tjmax = { 0 };
	if (get_cpu_tjmax(addr, (int *)&tjmax) == false)
		return false;

	sensor_val margin = { 0 };
	if (get_cpu_margin(addr, (int *)&margin) == false)
		return false;

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = tjmax.integer + margin.integer;
	return true;
}

static bool get_dimm_temp(uint8_t addr, uint8_t type, int *reading)
{
	if (!reading) {
		return false;
	}

	uint8_t temp_ofs = 0xFF;
	uint16_t param = 0xFF;
	switch (type) {
	case PECI_TEMP_CHANNEL0_DIMM0:
		param = 0x00;
		temp_ofs = DIMM_TEMP_OFS_0;
		break;
	case PECI_TEMP_CHANNEL0_DIMM1:
		param = 0x00;
		temp_ofs = DIMM_TEMP_OFS_1;
		break;
	case PECI_TEMP_CHANNEL1_DIMM0:
		param = 0x01;
		temp_ofs = DIMM_TEMP_OFS_0;
		break;
	case PECI_TEMP_CHANNEL1_DIMM1:
		param = 0x01;
		temp_ofs = DIMM_TEMP_OFS_1;
		break;
	case PECI_TEMP_CHANNEL2_DIMM0:
		param = 0x02;
		temp_ofs = DIMM_TEMP_OFS_0;
		break;
	case PECI_TEMP_CHANNEL2_DIMM1:
		param = 0x02;
		temp_ofs = DIMM_TEMP_OFS_1;
		break;
	case PECI_TEMP_CHANNEL3_DIMM0:
		param = 0x03;
		temp_ofs = DIMM_TEMP_OFS_0;
		break;
	case PECI_TEMP_CHANNEL3_DIMM1:
		param = 0x03;
		temp_ofs = DIMM_TEMP_OFS_1;
		break;
	case PECI_TEMP_CHANNEL4_DIMM0:
		param = 0x04;
		temp_ofs = DIMM_TEMP_OFS_0;
		break;
	case PECI_TEMP_CHANNEL4_DIMM1:
		param = 0x04;
		temp_ofs = DIMM_TEMP_OFS_1;
		break;
	case PECI_TEMP_CHANNEL5_DIMM0:
		param = 0x05;
		temp_ofs = DIMM_TEMP_OFS_0;
		break;
	case PECI_TEMP_CHANNEL5_DIMM1:
		param = 0x05;
		temp_ofs = DIMM_TEMP_OFS_1;
		break;
	case PECI_TEMP_CHANNEL6_DIMM0:
		param = 0x06;
		temp_ofs = DIMM_TEMP_OFS_0;
		break;
	case PECI_TEMP_CHANNEL6_DIMM1:
		param = 0x06;
		temp_ofs = DIMM_TEMP_OFS_1;
		break;
	case PECI_TEMP_CHANNEL7_DIMM0:
		param = 0x07;
		temp_ofs = DIMM_TEMP_OFS_0;
		break;
	case PECI_TEMP_CHANNEL7_DIMM1:
		param = 0x07;
		temp_ofs = DIMM_TEMP_OFS_1;
		break;
	default:
		break;
	}

	if (param == 0xFF || temp_ofs == 0xFF)
		return false;

	const uint8_t rlen = 0x05;
	uint8_t rbuf[rlen];
	memset(rbuf, 0, sizeof(rbuf));

	if (peci_read(PECI_CMD_RD_PKG_CFG0, addr, RDPKG_IDX_DIMM_TEMP, param, rlen, rbuf) != 0) {
		return false;
	}
	sensor_val *sval = (sensor_val *)reading;
	sval->integer = rbuf[temp_ofs];
	return true;
}

uint8_t intel_peci_read(uint8_t sensor_num, int *reading)
{
	if (!reading || (sensor_num > SENSOR_NUM_MAX)) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	bool ret_val = false;
	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
	const uint8_t read_type = cfg->offset;
	if (read_type <= PECI_UNKNOWN || read_type >= PECI_MAX)
		return SENSOR_NOT_FOUND;

	switch (read_type) {
	case PECI_TEMP_CHANNEL0_DIMM0:
	case PECI_TEMP_CHANNEL0_DIMM1:
	case PECI_TEMP_CHANNEL1_DIMM0:
	case PECI_TEMP_CHANNEL1_DIMM1:
	case PECI_TEMP_CHANNEL2_DIMM0:
	case PECI_TEMP_CHANNEL2_DIMM1:
	case PECI_TEMP_CHANNEL3_DIMM0:
	case PECI_TEMP_CHANNEL3_DIMM1:
	case PECI_TEMP_CHANNEL4_DIMM0:
	case PECI_TEMP_CHANNEL4_DIMM1:
	case PECI_TEMP_CHANNEL5_DIMM0:
	case PECI_TEMP_CHANNEL5_DIMM1:
	case PECI_TEMP_CHANNEL6_DIMM0:
	case PECI_TEMP_CHANNEL6_DIMM1:
	case PECI_TEMP_CHANNEL7_DIMM0:
	case PECI_TEMP_CHANNEL7_DIMM1:
		ret_val = get_dimm_temp(cfg->target_addr, read_type, reading);
		break;
	case PECI_TEMP_CPU_MARGIN:
		ret_val = get_cpu_margin(cfg->target_addr, reading);
		break;
	case PECI_TEMP_CPU_TJMAX:
		ret_val = get_cpu_tjmax(cfg->target_addr, reading);
		break;
	case PECI_TEMP_CPU:
		ret_val = get_cpu_temp(cfg->target_addr, reading);
		break;
	case PECI_PWR_CPU:
		ret_val = get_cpu_pwr(cfg->target_addr, reading);
		break;
	default:
		break;
	}
	return ret_val ? SENSOR_READ_SUCCESS : SENSOR_FAIL_TO_ACCESS;
}

uint8_t intel_peci_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	static bool is_init = false;
	sensor_config[sensor_config_index_map[sensor_num]].read = intel_peci_read;
	if (!is_init) {
		int ret;
		ret = peci_init();
		if (ret)
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		is_init = true;
	}
	return SENSOR_INIT_SUCCESS;
}
