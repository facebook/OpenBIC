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

/* 
  PEX89000 Hardware I2C Slave UG_v1.0.pdf
  PEX89000_RM100.pdf
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <zephyr.h>
#include <sys/util.h>
#include <sys/byteorder.h>
#include <logging/log.h>
#include "libutil.h"

#include "sensor.h"
#include "hal_i2c.h"
#include "pex89000.h"

LOG_MODULE_REGISTER(dev_pex89000);

#define BRCM_I2C5_CMD_READ 0b100
#define BRCM_I2C5_CMD_WRITE 0b011

#define BRCM_CHIME_AXI_CSR_ADDR 0x001F0100
#define BRCM_CHIME_AXI_CSR_DATA 0x001F0104
#define BRCM_CHIME_AXI_CSR_CTL 0x001F0108

/* Control register of Chime to AXI by SMBus */
#define BRCM_REG_SMB_WR_CMD 0xFFE00004
#define BRCM_REG_SMB_WR_DATA 0xFFE00008
#define BRCM_REG_SMB_RD_CMD 0xFFE0000C
#define BRCM_REG_SMB_RD_DATA 0xFFE00010

#define BRCM_REG_TEMP_SNR0_CTL 0xFFE78504
#define BRCM_REG_TEMP_SNR0_STAT 0xFFE78538
#define BRCM_REG_CHIP_ID 0xFFF00000
#define BRCM_REG_CHIP_REVID 0xFFF00004
#define BRCM_REG_SBR_ID 0xFFF00008
#define BRCM_REG_FLASH_VER 0x100005f8
#define BRCM_REG_CCR_SYSTEM_ERR 0xFFF000A8

#define BRCM_VAL_TEMP_SNR0_CTL_RESET 0x000653E8

static sys_slist_t pex89000_list;

typedef struct {
	uint8_t cmd : 3;
	uint8_t reserve1 : 5;
	uint8_t oft21_14bit : 8;
	uint8_t oft11_10bit : 2;
	uint8_t be : 4;
	uint8_t oft13_12bit : 2;
	uint8_t oft9_2bit : 8;
} __packed HW_I2C_Cmd;

static uint8_t pex_dev_get(uint8_t bus, uint8_t addr, uint8_t idx, pex_dev_t *dev);
static void pex89000_i2c_encode(uint32_t oft, uint8_t be, uint8_t cmd, HW_I2C_Cmd *buf);
static uint8_t pex89000_chime_read(uint8_t bus, uint8_t addr, uint32_t oft, uint8_t *resp,
				   uint16_t resp_len);
static uint8_t pex89000_chime_write(uint8_t bus, uint8_t addr, uint32_t oft, uint8_t *data,
				    uint8_t data_len);
static uint8_t pend_for_read_valid(uint8_t bus, uint8_t addr);
static uint8_t pex89000_chime_to_axi_write(uint8_t bus, uint8_t addr, uint32_t oft, uint32_t data);
static uint8_t pex89000_chime_to_axi_read(uint8_t bus, uint8_t addr, uint32_t oft, uint32_t *resp);
static uint8_t pex89000_temp(uint8_t bus, uint8_t addr, pex_dev_t dev, uint32_t *val);
pex89000_unit *find_pex89000_from_idx(uint8_t idx);

static uint8_t pex_dev_get(uint8_t bus, uint8_t addr, uint8_t idx, pex_dev_t *dev)
{
	CHECK_NULL_ARG_WITH_RETURN(dev, pex_api_unspecific_err);

	uint32_t resp;
	if (pex_access_engine(bus, addr, idx, pex_access_id, &resp)) {
		return pex_api_unspecific_err;
	};

	uint16_t dev_id = (resp >> 16) & 0xFFFF;
	if (dev_id == 0xC010 || dev_id == 0xC012)
		*dev = pex_dev_atlas1;
	else if (dev_id == 0xC030)
		*dev = pex_dev_atlas2;
	else
		*dev = pex_dev_unknown;

	return pex_api_success;
}

/*
 * be: byte enables
 * oft: Atlas register address
 * cmd: read or write command
 * buf: encoded byte array to send to pesw
 */
static void pex89000_i2c_encode(uint32_t oft, uint8_t be, uint8_t cmd, HW_I2C_Cmd *buf)
{
	CHECK_NULL_ARG(buf);

	buf->reserve1 = 0;
	buf->cmd = cmd;
	buf->oft21_14bit = (oft >> 14) & 0xFF;
	buf->oft13_12bit = (oft >> 12) & 0x3;
	buf->be = be;
	buf->oft11_10bit = (oft >> 10) & 0x3;
	buf->oft9_2bit = (oft >> 2) & 0xFF;
}

static uint8_t pex89000_chime_read(uint8_t bus, uint8_t addr, uint32_t oft, uint8_t *resp,
				   uint16_t resp_len)
{
	CHECK_NULL_ARG_WITH_RETURN(resp, pex_api_unspecific_err);

	HW_I2C_Cmd cmd;
	pex89000_i2c_encode(oft, 0xF, BRCM_I2C5_CMD_READ, &cmd);

	uint8_t retry = 5;
	I2C_MSG msg;

	msg.bus = bus;
	msg.target_addr = addr;
	msg.tx_len = sizeof(cmd);
	msg.rx_len = resp_len;
	memcpy(&msg.data[0], &cmd, sizeof(cmd));

	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("Chime read failed");
		return pex_api_unspecific_err;
	}

	memcpy(resp, &msg.data[0], resp_len);

	return pex_api_success;
}

static uint8_t pex89000_chime_write(uint8_t bus, uint8_t addr, uint32_t oft, uint8_t *data,
				    uint8_t data_len)
{
	CHECK_NULL_ARG_WITH_RETURN(data, pex_api_unspecific_err);

	HW_I2C_Cmd cmd;
	pex89000_i2c_encode(oft, 0xF, BRCM_I2C5_CMD_WRITE, &cmd);

	uint8_t retry = 5;
	I2C_MSG msg;

	msg.bus = bus;
	msg.target_addr = addr;
	msg.tx_len = sizeof(cmd) + data_len;
	memcpy(&msg.data[0], &cmd, sizeof(cmd));
	memcpy(&msg.data[4], data, data_len);

	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("Chime write failed");
		return pex_api_unspecific_err;
	}

	return pex_api_success;
}

static uint8_t pend_for_read_valid(uint8_t bus, uint8_t addr)
{
	uint8_t rty = 50;
	uint32_t resp = 0;

	for (int i = rty; i > 0; i--) {
		if (pex89000_chime_read(bus, addr, BRCM_CHIME_AXI_CSR_CTL, (uint8_t *)&resp,
					sizeof(resp))) {
			k_msleep(10);
			continue;
		}

		if (resp & BIT(27)) { // CHIME_to_AXI_CSR Control Status -> Read_data_vaild
			return pex_api_success;
		}

		k_msleep(10);
	}

	return pex_api_unspecific_err;
}

static uint8_t pex89000_chime_to_axi_write(uint8_t bus, uint8_t addr, uint32_t oft, uint32_t data)
{
	uint8_t rc = pex_api_unspecific_err;
	uint32_t wbuf;

	wbuf = sys_cpu_to_be32(oft);
	if (pex89000_chime_write(bus, addr, BRCM_CHIME_AXI_CSR_ADDR, (uint8_t *)&wbuf,
				 sizeof(wbuf))) {
		goto exit;
	}
	wbuf = sys_cpu_to_be32(data);
	if (pex89000_chime_write(bus, addr, BRCM_CHIME_AXI_CSR_DATA, (uint8_t *)&wbuf,
				 sizeof(wbuf))) {
		goto exit;
	}
	wbuf = sys_cpu_to_be32(0x1); // CHIME_to_AXI_CSR Control Status: write command
	if (pex89000_chime_write(bus, addr, BRCM_CHIME_AXI_CSR_CTL, (uint8_t *)&wbuf,
				 sizeof(wbuf))) {
		goto exit;
	}

	rc = pex_api_success;
exit:
	return rc;
}

static uint8_t pex89000_chime_to_axi_read(uint8_t bus, uint8_t addr, uint32_t oft, uint32_t *resp)
{
	CHECK_NULL_ARG_WITH_RETURN(resp, pex_api_unspecific_err);

	uint8_t rc = pex_api_unspecific_err;

	uint32_t data;
	data = sys_cpu_to_be32(oft);

	if (pex89000_chime_write(bus, addr, BRCM_CHIME_AXI_CSR_ADDR, (uint8_t *)&data,
				 sizeof(data))) {
		goto exit;
	}
	data = sys_cpu_to_be32(0x2); // CHIME_to_AXI_CSR Control Status: read command
	if (pex89000_chime_write(bus, addr, BRCM_CHIME_AXI_CSR_CTL, (uint8_t *)&data,
				 sizeof(data))) {
		goto exit;
	}

	k_msleep(10);
	if (pend_for_read_valid(bus, addr)) {
		LOG_ERR("Read data invaild");
		goto exit;
	}

	if (pex89000_chime_read(bus, addr, BRCM_CHIME_AXI_CSR_DATA, (uint8_t *)resp,
				sizeof(resp))) {
		goto exit;
	}

	*resp = sys_cpu_to_be32(*resp);
	rc = pex_api_success;

exit:
	return rc;
}

uint8_t pex_access_engine(uint8_t bus, uint8_t addr, uint8_t idx, pex_access_t key, uint32_t *resp)
{
	CHECK_NULL_ARG_WITH_RETURN(resp, pex_api_unspecific_err);

	pex89000_unit *p = find_pex89000_from_idx(idx);
	if (!p) {
		LOG_ERR("Node %d not found", idx);
		return pex_api_unspecific_err;
	}

	if (k_mutex_lock(&p->mutex, K_MSEC(5000))) {
		LOG_ERR("pex(%d) mutex lock failed", p->idx);
		return pex_api_mutex_err;
	}

	uint8_t rc = pex_api_success;

	switch (key) {
	case pex_access_temp:
		if (pex89000_temp(bus, addr, p->pex_type, resp)) {
			LOG_ERR("Read temperature failed at pex(%d)", p->idx);
			rc = pex_api_unspecific_err;
		}
		break;

	case pex_access_adc:
		LOG_ERR("Access ADC value not support yet");
		rc = pex_api_unspecific_err;
		break;

	case pex_access_id:
		if (pex89000_chime_to_axi_read(bus, addr, BRCM_REG_CHIP_ID, resp)) {
			LOG_ERR("Access chip id register(0x%x) failed at pex(%d)", BRCM_REG_CHIP_ID,
				p->idx);
			rc = pex_api_unspecific_err;
		}
		break;

	case pex_access_rev_id:
		if (pex89000_chime_to_axi_read(bus, addr, BRCM_REG_CHIP_REVID, resp)) {
			LOG_ERR("Access chip revision id register(0x%x) failed at pex(%d)",
				BRCM_REG_CHIP_REVID, p->idx);
			rc = pex_api_unspecific_err;
		}
		break;

	case pex_access_sbr_ver:
		if (pex89000_chime_to_axi_read(bus, addr, BRCM_REG_SBR_ID, resp)) {
			LOG_ERR("Access SBR id register(0x%x) failed at pex(%d)", BRCM_REG_SBR_ID,
				p->idx);
			rc = pex_api_unspecific_err;
		}
		break;

	case pex_access_flash_ver:
		if (pex89000_chime_to_axi_read(bus, addr, BRCM_REG_FLASH_VER, resp)) {
			LOG_ERR("Access flash register(0x%x) failed at pex(%d)", BRCM_REG_FLASH_VER,
				p->idx);
			rc = pex_api_unspecific_err;
		}
		break;
	case pex_access_register: {
		uint32_t reg = *resp;
		if (pex89000_chime_to_axi_read(bus, addr, reg, resp)) {
			LOG_ERR("Access register(0x%x) failed at pex(%d)", reg, p->idx);
			rc = pex_api_unspecific_err;
		}
		break;
	}
	case pex_access_ccr_system_error:
		if (pex89000_chime_to_axi_read(bus, addr, BRCM_REG_CCR_SYSTEM_ERR, resp)) {
			LOG_ERR("Access CCR system error register(0x%x) failed at pex(%d)",
				BRCM_REG_CCR_SYSTEM_ERR, p->idx);
			rc = pex_api_unspecific_err;
		}
		break;
	default:
		LOG_ERR("Invalid key, (%d)", key);
		rc = pex_api_unspecific_err;
		break;
	}

	if (k_mutex_unlock(&p->mutex))
		LOG_ERR("pex(%d) mutex unlock failed", p->idx);

	return rc;
}

static uint8_t pex89000_temp(uint8_t bus, uint8_t addr, pex_dev_t dev, uint32_t *val)
{
	CHECK_NULL_ARG_WITH_RETURN(val, pex_api_unspecific_err);

	uint8_t rc = pex_api_unspecific_err;

	float highest_temp = 0;
	float temp = 0;
	uint32_t CmdAddr;
	uint32_t resp = 0;

	if (dev == pex_dev_atlas1) {
		if (pex89000_chime_to_axi_read(bus, addr, BRCM_REG_TEMP_SNR0_CTL, &resp)) {
			LOG_ERR("CHIME to AXI Read 0x%x failed", BRCM_REG_TEMP_SNR0_CTL);
			goto exit;
		}
		if (resp != BRCM_VAL_TEMP_SNR0_CTL_RESET) {
			LOG_ERR("ADC temperature control register1 check failed");
			goto exit;
		}

		if (pex89000_chime_to_axi_write(bus, addr, BRCM_REG_TEMP_SNR0_CTL,
						BRCM_VAL_TEMP_SNR0_CTL_RESET)) {
			LOG_ERR("CHIME to AXI Write 0x%x failed", BRCM_REG_TEMP_SNR0_CTL);
			goto exit;
		}

		if (pex89000_chime_to_axi_read(bus, addr, BRCM_REG_TEMP_SNR0_STAT, &resp)) {
			LOG_ERR("CHIME to AXI Write 0x%x failed", BRCM_REG_TEMP_SNR0_STAT);
			goto exit;
		}

		temp = (resp & 0xFFFF) / 128;
	} else if (dev == pex_dev_atlas2) {
		for (int8_t i = 7; i < 12; i++) {
			CmdAddr = (0x21 << 16) | (0x4C << 8) | (0x0B);
			if (pex89000_chime_to_axi_write(bus, addr, BRCM_REG_SMB_WR_CMD, CmdAddr)) {
				LOG_ERR("CHIME to AXI Write 0x%x failed", BRCM_REG_SMB_WR_CMD);
				goto exit;
			}

			if (pex89000_chime_to_axi_write(bus, addr, BRCM_REG_SMB_WR_DATA,
							i | 0x10000)) {
				LOG_ERR("CHIME to AXI Write 0x%x failed", BRCM_REG_SMB_WR_DATA);
				goto exit;
			}

			CmdAddr = (0x22 << 16) | (0x4C << 8) | (0x14);
			if (pex89000_chime_to_axi_write(bus, addr, BRCM_REG_SMB_RD_CMD, CmdAddr)) {
				LOG_ERR("CHIME to AXI Write 0x%x failed", BRCM_REG_SMB_RD_CMD);
				goto exit;
			}

			if (pex89000_chime_to_axi_read(bus, addr, BRCM_REG_SMB_RD_DATA, &resp)) {
				LOG_ERR("CHIME to AXI Write 0x%x failed", BRCM_REG_SMB_RD_DATA);
				goto exit;
			}

			if (resp == 0)
				continue;

			temp = (float)(366.812 - 0.23751 * (float)(resp & 0x7FF));

			if (temp > highest_temp)
				highest_temp = temp;
		}

		if (highest_temp == 0)
			goto exit;
	} else {
		LOG_ERR("The device type is not support, (%d)", dev);
		goto exit;
	}

	sensor_val *sval = (sensor_val *)val;
	sval->integer = (int16_t)highest_temp;
	sval->fraction = (highest_temp - sval->integer) * 1000;

	rc = pex_api_success;
exit:
	return rc;
}

pex89000_unit *find_pex89000_from_idx(uint8_t idx)
{
	sys_snode_t *node = NULL;
	SYS_SLIST_FOR_EACH_NODE (&pex89000_list, node) {
		pex89000_unit *p;
		p = CONTAINER_OF(node, pex89000_unit, node);
		if (p->idx == idx) {
			return p;
		}
	}

	return NULL;
}

uint8_t pex89000_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->priv_data, SENSOR_UNSPECIFIED_ERROR);

	uint8_t rc = SENSOR_UNSPECIFIED_ERROR;

	pex89000_unit *p = (pex89000_unit *)cfg->priv_data;

	switch (cfg->offset) {
	case PEX_TEMP:
		if (pex_access_engine(cfg->port, cfg->target_addr, p->idx, pex_access_temp,
				      reading)) {
			LOG_ERR("Read temperature failed");
			rc = SENSOR_FAIL_TO_ACCESS;
			goto exit;
		}

		break;
	default:
		LOG_ERR("Invalid sensor type, (%d)", cfg->offset);
		goto exit;
	}

	rc = SENSOR_READ_SUCCESS;
exit:
	return rc;
}

uint8_t pex89000_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_INIT_UNSPECIFIED_ERROR);

	pex89000_init_arg *init_arg = (pex89000_init_arg *)cfg->init_args;

	pex89000_unit *p;
	p = find_pex89000_from_idx(init_arg->idx);
	if (p == NULL) {
		p = (pex89000_unit *)malloc(sizeof(pex89000_unit));
		if (!p) {
			LOG_ERR("The pex89000_unit malloc failed at index %d", init_arg->idx);
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}

		p->idx = init_arg->idx;

		if (k_mutex_init(&p->mutex)) {
			LOG_ERR("pex(%d) mutex initial failed", p->idx);
			SAFE_FREE(p);
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}

		sys_slist_append(&pex89000_list, &p->node);

		if (pex_dev_get(cfg->port, cfg->target_addr, p->idx, &p->pex_type)) {
			LOG_ERR("Get pex type failed");
			sys_slist_find_and_remove(&pex89000_list, &p->node);
			SAFE_FREE(p);
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}
	}

	cfg->priv_data = p;
	cfg->read = pex89000_read;
	init_arg->is_init = true;

	return SENSOR_INIT_SUCCESS;
}
