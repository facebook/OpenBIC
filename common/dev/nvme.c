/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include "libutil.h"
#include "sensor.h"
#include "hal_i2c.h"
#include "nvme.h"

#include <logging/log.h>
#include <sys/crc.h>

#define NVMe_NOT_AVAILABLE 0x80
#define NVMe_TMP_SENSOR_FAILURE 0x81
#define NVMe_DRIVE_NOT_READY_BIT BIT(6)

#define NVMe_TEMP_READ_LEN 8
#define NVMe_VOLTAGE_RAIL_READ_LEN 2
#define NVMe_PEC_INDEX 7
#define NVMe_STATUS_INDEX 1
#define NVMe_TEMPERATURE_INDEX 3
#define NVMe_PEC_NOT_SUPPORT 0x00

LOG_MODULE_REGISTER(nvme);

__weak void plat_nvme_bus_reset(uint8_t bus)
{
	return;
}

int read_nvme_info(uint8_t bus, uint8_t addr, uint8_t offset, uint8_t read_len, uint8_t *data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, -1);

	int ret = 0;
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	msg.bus = bus;
	msg.target_addr = addr;
	msg.tx_len = 1;
	msg.rx_len = read_len;
	msg.data[0] = offset;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("I2C read bus: 0x%x, addr: 0x%x, offset: 0x%x fail", bus, addr, offset);
		plat_nvme_bus_reset(bus);
		return -1;
	}

	memcpy(data, &msg.data[0], read_len * sizeof(uint8_t));
	return 0;
}

static uint8_t nvme_cal_pec(uint8_t addr, uint8_t oft, uint8_t *buf, uint32_t len, uint8_t *pec)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, 1);
	CHECK_NULL_ARG_WITH_RETURN(pec, 1);

	uint8_t pec_buf[len + 3];
	pec_buf[0] = addr << 1;
	pec_buf[1] = oft;
	pec_buf[2] = (addr << 1) + 1;

	memcpy(pec_buf + 3, buf, len);

	*pec = crc8(pec_buf, sizeof(pec_buf), 0x07, 0x00, false);

	return 0;
}

uint8_t nvme_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor nvme 0x%x input parameter is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	int ret = 0;
	uint8_t pec = 0;
	uint8_t rx_len = 0;
	uint16_t val = 0;

	switch (cfg->offset) {
	case NVME_TEMP_OFFSET:
		rx_len = NVMe_TEMP_READ_LEN;
		break;
	case NVME_CORE_VOLTAGE_1_OFFSET:
	case NVME_CORE_VOLTAGE_2_OFFSET:
	case NVME_VOLTAGE_RAIL_1_OFFSET:
	case NVME_VOLTAGE_RAIL_2_OFFSET:
		rx_len = NVMe_VOLTAGE_RAIL_READ_LEN;
		break;
	default:
		LOG_ERR("Invalid nvme offset: 0x%x, snesor num: 0x%x", cfg->offset, cfg->num);
		return SENSOR_PARAMETER_NOT_VALID;
	}

	uint8_t read_resp[rx_len];
	memset(&read_resp[0], 0, rx_len);

	ret = read_nvme_info(cfg->port, cfg->target_addr, cfg->offset, rx_len, read_resp);
	if (ret != 0) {
		LOG_ERR("Nvme read sensor num: 0x%x, bus: 0x%x, addr: 0x%x, offset: 0x%x, rx_len: 0x%x fail",
			cfg->num, cfg->port, cfg->target_addr, cfg->offset, rx_len);
		return SENSOR_FAIL_TO_ACCESS;
	}

	sensor_val *sval = (sensor_val *)reading;
	switch (cfg->offset) {
	case NVME_TEMP_OFFSET: {
		if (read_resp[NVMe_PEC_INDEX] != NVMe_PEC_NOT_SUPPORT) {
			if (nvme_cal_pec(cfg->target_addr, cfg->offset, read_resp, rx_len - 1,
					 &pec) != 0) {
				LOG_ERR("sensor_num 0x%02x cal nvme pec fail!", cfg->num);
				return SENSOR_UNSPECIFIED_ERROR;
			}

			if (read_resp[NVMe_PEC_INDEX] != pec) {
				LOG_ERR("sensor_num 0x%02x check nvme pec error! (%02x/%02x)",
					cfg->num, read_resp[NVMe_PEC_INDEX], pec);
				return SENSOR_PEC_ERROR;
			}
		}

		uint8_t nvme_status = read_resp[NVMe_STATUS_INDEX];
		val = read_resp[NVMe_TEMPERATURE_INDEX];

		/* Check SSD drive ready */
		if ((nvme_status & NVMe_DRIVE_NOT_READY_BIT) != 0) {
			return SENSOR_NOT_ACCESSIBLE;
		}

		/* Check reading value */
		switch (val) {
		case NVMe_NOT_AVAILABLE:
			return SENSOR_FAIL_TO_ACCESS;
		case NVMe_TMP_SENSOR_FAILURE:
			return SENSOR_UNSPECIFIED_ERROR;
		default:
			break;
		}

		sval->integer = (int8_t)val;
		sval->fraction = 0;
		return SENSOR_READ_SUCCESS;
	}
	case NVME_CORE_VOLTAGE_1_OFFSET:
	case NVME_CORE_VOLTAGE_2_OFFSET:
	case NVME_VOLTAGE_RAIL_1_OFFSET:
	case NVME_VOLTAGE_RAIL_2_OFFSET:
		// 1 mV/LSB
		// Voltage rail reading value can correct on post-read function
		val = ((read_resp[0] << 8) | read_resp[1]);

		sval->integer = (val / 1000) & 0xFFFF;
		sval->fraction = (val - (sval->integer * 1000)) & 0xFFFF;
		return SENSOR_READ_SUCCESS;
	default:
		LOG_ERR("Invalid offset: 0x%x", cfg->offset);
		return SENSOR_PARAMETER_NOT_VALID;
	}
}

uint8_t nvme_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	cfg->read = nvme_read;
	return SENSOR_INIT_SUCCESS;
}
