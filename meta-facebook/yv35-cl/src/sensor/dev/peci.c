#include <stdio.h>
#include <stdlib.h>
#include "sensor.h"
#include "plat_sensor.h"
#include "pal.h"
#include <drivers/peci.h>
#include "hal_peci.h"
#include <string.h>

bool pal_peci_read(uint8_t sensor_num, int *reading)
{
	uint8_t u8Index, u8ReadLen = 0x05, address, *readBuf;
	uint8_t cmd = PECI_CMD_RD_PKG_CFG0;
	uint16_t u16Param;
	int val, ret, complete_code;
	static uint8_t cpu_temp_tjmax = 0;
	if (sensor_num == SENSOR_NUM_TEMP_CPU_MARGIN) {
		u8Index = 0x02;
		u16Param = 0x00ff;
	} else if (sensor_num == SENSOR_NUM_TEMP_CPU) {
		if ((sensor_config[SnrNum_SnrCfg_map[SENSOR_NUM_TEMP_CPU_TJMAX]].cache_status ==
		     SNR_READ_ACUR_SUCCESS) ||
		    (sensor_config[SnrNum_SnrCfg_map[SENSOR_NUM_TEMP_CPU_MARGIN]].cache_status ==
		     SNR_READ_ACUR_SUCCESS)) {
			return true;
		} else {
			return false;
		}
	} else if (sensor_num == SENSOR_NUM_TEMP_CPU_TJMAX) {
		if (sensor_config[SnrNum_SnrCfg_map[SENSOR_NUM_TEMP_CPU]].cache_status ==
		    SNR_READ_ACUR_SUCCESS) {
			return true;
		} else {
			u8Index = 0x10;
			u16Param = 0x0000;
		}
	} else if (sensor_num == SENSOR_NUM_TEMP_DIMM_A) {
		u8Index = 0x0E;
		u16Param = 0x0000;
	} else if (sensor_num == SENSOR_NUM_TEMP_DIMM_C) {
		u8Index = 0x0E;
		u16Param = 0x0002;
	} else if (sensor_num == SENSOR_NUM_TEMP_DIMM_D) {
		u8Index = 0x0E;
		u16Param = 0x0003;
	} else if (sensor_num == SENSOR_NUM_TEMP_DIMM_E) {
		u8Index = 0x0E;
		u16Param = 0x0004;
	} else if (sensor_num == SENSOR_NUM_TEMP_DIMM_G) {
		u8Index = 0x0E;
		u16Param = 0x0006;
	} else if (sensor_num == SENSOR_NUM_TEMP_DIMM_H) {
		u8Index = 0x0E;
		u16Param = 0x0007;
	} else if (sensor_num == SENSOR_NUM_PWR_CPU) {
		ret = peci_getPwr(sensor_num, &val);
		if (ret) {
			*reading = (calculate_accurate_MBR(sensor_num, val)) & 0xffff;
			sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache = *reading;
			sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status =
				SNR_READ_ACUR_SUCCESS;
			return true;
		} else {
			sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status =
				SNR_FAIL_TO_ACCESS;
			return false;
		}
	} else {
		printf("Unrecognized sensor reading\n");
		return false;
	}
	address = sensor_config[SnrNum_SnrCfg_map[sensor_num]].slave_addr;
	readBuf = (uint8_t *)malloc(u8ReadLen * sizeof(uint8_t));
	if (readBuf == NULL) {
		return false;
	}

	ret = peci_read(cmd, address, u8Index, u16Param, u8ReadLen, readBuf);
	complete_code = readBuf[0];

	if (ret) {
		if (sensor_num == SENSOR_NUM_TEMP_CPU_MARGIN) {
			sensor_config[SnrNum_SnrCfg_map[SENSOR_NUM_TEMP_CPU]].cache_status =
				SNR_FAIL_TO_ACCESS;
		}
		if (readBuf != NULL) {
			free(readBuf);
		}
		sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status = SNR_FAIL_TO_ACCESS;
		return false;
	}

	if (complete_code != PECI_CC_RSP_SUCCESS) {
		// retry block
		bool is_retry_success = false;
		if (complete_code == PECI_CC_RSP_TIMEOUT ||
		    complete_code == PECI_CC_OUT_OF_RESOURCES_TIMEOUT) {
			is_retry_success = peci_retry_read(cmd, address, u8Index, u16Param,
							   u8ReadLen, readBuf);
			// retry block

			if (!is_retry_success) {
				printf("PECI sensor [%x] response timeout. Reach Max retry.\n",
				       sensor_num);
				sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status =
					SNR_FAIL_TO_ACCESS;
				if (readBuf != NULL) {
					free(readBuf);
				}
				return false;
			}

		} else if (complete_code == PECI_CC_ILLEGAL_REQUEST) {
			printf("Unknown request\n");
			sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status = SNR_NOT_FOUND;
			if (readBuf != NULL) {
				free(readBuf);
			}
			return false;
		} else {
			printf("PECI control hardware, firmware or associated logic error\n");
			sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status =
				SNR_UNSPECIFIED_ERROR;
			if (readBuf != NULL) {
				free(readBuf);
			}
			return false;
		}
	}

	// PECI_CC_RSP_SUCCESS
	if (sensor_num == SENSOR_NUM_TEMP_CPU_MARGIN) {
		val = ((0xFFFF - ((readBuf[2] << 8) | readBuf[1])) >> 6) + 1;
		if (sensor_config[SnrNum_SnrCfg_map[SENSOR_NUM_TEMP_CPU_TJMAX]].cache_status ==
		    SNR_READ_ACUR_SUCCESS) {
			sensor_config[SnrNum_SnrCfg_map[SENSOR_NUM_TEMP_CPU]].cache =
				cpu_temp_tjmax - (calculate_MBR(sensor_num, val) & 0xff);
			sensor_config[SnrNum_SnrCfg_map[SENSOR_NUM_TEMP_CPU]].cache_status =
				SNR_READ_SUCCESS;
		}
	} else if (sensor_num == SENSOR_NUM_TEMP_CPU_TJMAX) {
		val = readBuf[3];
		cpu_temp_tjmax = val;
	} else {
		val = readBuf[1];
	}
	if (readBuf != NULL) {
		free(readBuf);
	}
	*reading = (calculate_accurate_MBR(sensor_num, val)) & 0xffff;
	sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache = *reading;
	sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status = SNR_READ_ACUR_SUCCESS;
	return true;
}
