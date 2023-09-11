#include "plat_cpu.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <drivers/peci.h>
#include "intel_peci.h"
#include "libutil.h"
#include "hal_peci.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(pal_cpu);

bool pal_get_cpu_energy(uint8_t addr, uint32_t *pkg_energy, uint32_t *run_time)
{
	uint8_t command = PECI_RD_PKG_CFG0_CMD;
	uint8_t readlen = 0x09;
	uint8_t index = 0x03;
	uint16_t parameter = 0x00FF;
	int ret = 0;

	uint8_t *readbuf = (uint8_t *)malloc(readlen * sizeof(uint8_t));
	if (!readbuf) {
		LOG_ERR("%s fail to allocate readbuf memory", __func__);
		return false;
	}

	ret = peci_read(command, addr, index, parameter, readlen, readbuf);
	if (ret) {
		LOG_ERR("PECI read cpu energy and time error");
		goto cleanup;
	}
	if (readbuf[0] != PECI_CC_RSP_SUCCESS) {
		if (readbuf[0] == PECI_CC_ILLEGAL_REQUEST) {
			LOG_ERR("%s unknown request", __func__);
		} else {
			LOG_ERR("%s peci control hardware, firmware or associated logic error",
				__func__);
		}
		goto cleanup;
	}

	*pkg_energy = readbuf[4];
	*pkg_energy = (*pkg_energy << 8) | readbuf[3];
	*pkg_energy = (*pkg_energy << 8) | readbuf[2];
	*pkg_energy = (*pkg_energy << 8) | readbuf[1];

	*run_time = readbuf[8];
	*run_time = (*run_time << 8) | readbuf[7];
	*run_time = (*run_time << 8) | readbuf[6];
	*run_time = (*run_time << 8) | readbuf[5];

	SAFE_FREE(readbuf);
	return true;
cleanup:
	SAFE_FREE(readbuf);
	return false;
}

void pal_cal_cpu_power(intel_peci_unit unit_info, uint32_t diff_energy, uint32_t diff_time,
		       int *reading)
{
	float time_unit_energy = (float)((float)diff_energy * (float)CPU_TIME_UNIT);

	float pwr_scale = (float)(1 / (float)(1 << unit_info.energy_unit));

	*reading = (time_unit_energy / (float)diff_time) * pwr_scale;
}
