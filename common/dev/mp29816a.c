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
#include <stdlib.h>
#include <string.h>
#include <logging/log.h>
#include "libutil.h"
#include "sensor.h"
#include "hal_i2c.h"
#include "pmbus.h"
#include "mp29816a.h"
#include "util_pmbus.h"

LOG_MODULE_REGISTER(mp29816a);

#define READ_VOUT_MASK GENMASK(11, 0)
#define MFR_VID_RES_MASK GENMASK(12, 10)
#define MFR_VOUT_SCALE_LOOP 0x29
#define MFR_VOUT_LOOP_CTRL 0x8D
#define MFR_VDIFF_GAIN_HALF_R2_BIT BIT(10)

#define VR_MPS_PAGE_0 0x00
#define VR_MPS_PAGE_1 0x01

#define MP29816A_VOUT_MAX_REG 0x24
#define MP29816A_VOUT_MIN_REG 0x2B

/* --------- PAGE1 ---------- */
#define VR_REG_EXPECTED_USER_CRC 0xED

#define MAX_CFG_LEN 1024

enum CFG_PARAM_IDX {
	CFG_PARAM_IDX_PAGE = 1,
	CFG_PARAM_IDX_REG_ADDR = 2,
	CFG_PARAM_IDX_REG_NAME = 4,
	CFG_PARAM_IDX_REG_VAL = 5,
	CFG_PARAM_IDX_REG_LEN = 7
};

struct cfg_data {
	uint8_t cfg_page;
	uint8_t cfg_idx;
	uint8_t reg_addr;
	uint16_t reg_val;
	uint8_t reg_len;
};

static int cnt_char(char *s, char c)
{
	int cnt = 0;
	while (*s) {
		if (*s == c)
			cnt++;
		s++;
	}
	return cnt;
}

float mp29816a_get_resolution(sensor_cfg *cfg, bool is_vout_max_or_min);

uint8_t parsing_line(char *str, uint16_t len, struct cfg_data *cfg_data)
{
	if (!str || !cfg_data)
		return 1;

	char *p = str;
	char *save_ptr;
	char *k = strtok_r(p, "\t", &save_ptr);
	uint8_t idx = 0;
	do {
		switch (idx++) {
		case CFG_PARAM_IDX_PAGE:
			cfg_data->cfg_page = strtol(k, NULL, 16) & 0x0F;
			cfg_data->cfg_idx = (strtol(k, NULL, 16) & 0xF0) >> 4;
			break;
		case CFG_PARAM_IDX_REG_ADDR:
			cfg_data->reg_addr = strtol(k, NULL, 16);
			break;
		case CFG_PARAM_IDX_REG_NAME:
			if (!strncmp(k, "CRC", strlen("CRC"))) {
				LOG_ERR("CRC: %s", k);
				return 1;
			} else if (!strncmp(k, "TRIM", strlen("TRIM"))) {
				LOG_INF("TRIM: %s", k);
				return 1;
			}
			break;
		case CFG_PARAM_IDX_REG_VAL:
			cfg_data->reg_val = strtol(k, NULL, 16);
			break;
		case CFG_PARAM_IDX_REG_LEN:
			cfg_data->reg_len = strtol(k, NULL, 10);
			break;
		default:
			// don't care value
			break;
		}
	} while ((k = strtok_r(NULL, "\t", &save_ptr)));

	return 0;
}

static uint32_t parsing_image(const uint8_t *img_buff, uint32_t img_size,
			      struct cfg_data *cfg_data_list, uint32_t max_cfg_len)
{
	if (!img_buff || !cfg_data_list)
		return 0;

	uint32_t i = 0;
	uint32_t cfg_cnt = 0;

	while (i < img_size) {
		// get the line length, only support windows format next line (\r\n)
		int len = 0;
		while (((i + len) < img_size) &&
		       ((img_buff[i + len] != '\r') && (img_buff[i + len] != '\n')))
			len++;

		if (!len) {
			// just skip the empty line
			i = i + 2;
			continue;
		}

		char tmp[128] = { 0 };
		strncpy(tmp, img_buff + i, len);

		// only handle the config line
		if (cnt_char(tmp, '\t') == 7) {
			if (!parsing_line(tmp, len, cfg_data_list + cfg_cnt))
				cfg_cnt++;
		}

		if (cfg_cnt >= max_cfg_len) {
			LOG_ERR("Too many config data");
			return 0;
		}

		i = i + len + 2;
	}

	return cfg_cnt;
}

#define MP29816A_MAX_READ_BYTES (6 + 1) //Block Read 6 bytes + 1 byte for the length of the data

bool mp29816a_i2c_read(uint8_t bus, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	if (len > MP29816A_MAX_READ_BYTES) {
		LOG_ERR("mp29816a read len is too long");
		return false;
	}

	memset(data, 0, len);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = len;
	i2c_msg.data[0] = reg;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read mp29816a, bus: %d, addr: 0x%x, reg: 0x%x", bus, addr, reg);
		return false;
	}

	memcpy(data, i2c_msg.data, len);
	return true;
}

bool mp29816a_i2c_write(uint8_t bus, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = len + 1;

	i2c_msg.data[0] = reg;

	if (len > 0)
		memcpy(&i2c_msg.data[1], data, len);

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Failed to write mp29816a, bus: %d, addr: 0x%x, reg: 0x%x", bus, addr, reg);
		return false;
	}

	return true;
}

bool mp29816a_set_page(uint8_t bus, uint8_t addr, uint8_t page)
{
	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 2;
	i2c_msg.data[0] = PMBUS_PAGE;
	i2c_msg.data[1] = page;

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Failed to set mp29816a page, bus: %d, addr: 0x%x, page: 0x%x", bus, addr,
			page);
		return false;
	}

	return true;
}

bool mp29816a_get_vout_max(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millivolt, false);

	uint8_t data[2] = { 0 };
	if (!mp29816a_i2c_read(cfg->port, cfg->target_addr, MP29816A_VOUT_MAX_REG, data,
			       sizeof(data))) {
		return false;
	}

	uint16_t read_value = data[0] | (data[1] << 8);
	float resolution = mp29816a_get_resolution(cfg, true);
	if (resolution == 0)
		return false;
	float val = (float)read_value * resolution * 1000;
	uint16_t val_int = (uint16_t)val;

	*millivolt = val_int;

	return true;
}

bool mp29816a_get_vout_min(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millivolt, false);

	uint8_t data[2] = { 0 };
	if (!mp29816a_i2c_read(cfg->port, cfg->target_addr, MP29816A_VOUT_MIN_REG, data,
			       sizeof(data))) {
		return false;
	}

	uint16_t read_value = data[0] | (data[1] << 8);
	float resolution = mp29816a_get_resolution(cfg, true);
	if (resolution == 0)
		return false;
	float val = (float)read_value * resolution * 1000;
	uint16_t val_int = (uint16_t)val;

	*millivolt = val_int;

	return true;
}

bool mp29816a_set_vout_max(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millivolt, false);

	float resolution = mp29816a_get_resolution(cfg, true);
	if (resolution == 0)
		return false;
	uint16_t read_value = (*millivolt / resolution) / 1000;

	uint8_t data[2] = { 0 };
	data[0] = read_value & 0xFF;
	data[1] = (read_value >> 8) & 0xFF;

	if (!mp29816a_i2c_write(cfg->port, cfg->target_addr, MP29816A_VOUT_MAX_REG, data,
				sizeof(data))) {
		return false;
	}

	return true;
}

bool mp29816a_set_vout_min(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millivolt, false);

	float resolution = mp29816a_get_resolution(cfg, true);
	if (resolution == 0)
		return false;
	uint16_t read_value = (*millivolt / resolution) / 1000;

	uint8_t data[2] = { 0 };
	data[0] = read_value & 0xFF;
	data[1] = (read_value >> 8) & 0xFF;

	if (!mp29816a_i2c_write(cfg->port, cfg->target_addr, MP29816A_VOUT_MIN_REG, data,
				sizeof(data))) {
		return false;
	}

	return true;
}

static uint8_t mp29816a_do_update(struct cfg_data *cfg_data_list, uint32_t cfg_cnt, uint8_t bus,
				  uint8_t addr)
{
	if (!cfg_data_list || !cfg_cnt)
		return 1;

	uint8_t data[MP29816A_MAX_READ_BYTES] = { 0 };

	// check page0@99 is 0x4d5053
	mp29816a_set_page(bus, addr, 0);
	if (!mp29816a_i2c_read(bus, addr, 0x99, data, (3 + 1))) { //block read read len +1
		LOG_ERR("Failed to read page0@99");
		return 1;
	} else {
		uint32_t read_value = data[1] | (data[2] << 8) | (data[3] << 16);
		if (read_value != 0x4D5053) {
			LOG_ERR("page0@99 is not 0x4D5053, read_value: 0x%x", read_value);
			return 1;
		}
	}

	// check page2@1d is 0xa816 or 0xaa16
	mp29816a_set_page(bus, addr, 2);
	if (!mp29816a_i2c_read(bus, addr, 0x1d, data, 2)) {
		LOG_ERR("Failed to read page2@1d");
		return 1;
	} else {
		uint16_t read_value = data[0] | (data[1] << 8);
		if ((read_value != 0xa816) && (read_value != 0xaa16) && (read_value != 0xc816)) {
			LOG_ERR("page2@1d is not 0xa816 or 0xaa16 or 0xc816");
			return 1;
		}
	}

	uint8_t cfg_idx = 0;
	uint8_t page = 0xff;

	for (int i = 0; i < cfg_cnt; i++) {
		const struct cfg_data *p = cfg_data_list + i;

		if (p->cfg_idx != cfg_idx) {
			// switch config
			// before switching config, store the previous cfg value except for shared register (cfg 0)
			if (cfg_idx) {
				// write page1@cc with 0x1200
				if (!mp29816a_set_page(bus, addr, 1))
					return 1;
				memcpy(data, (uint8_t[]){ 0x00, 0x12 }, 2);
				mp29816a_i2c_write(bus, addr, 0xcc, data, 2);

				// write page0@17\n");
				if (!mp29816a_set_page(bus, addr, 0))
					return 1;
				memset(data, 0, sizeof(data));
				mp29816a_i2c_write(bus, addr, 0x17, data, 0);

				k_msleep(1000);
			}

			// write p2@1a with 800f/900f/a00f/b00f/c00f/d00f
			cfg_idx = p->cfg_idx;
			//  write page1@cc with 0x128c
			if (!mp29816a_set_page(bus, addr, 1))
				return 1;
			memcpy(data, (uint8_t[]){ 0x8c, 0x12 }, 2);
			mp29816a_i2c_write(bus, addr, 0xcc, data, 2);

			//  write page3@81 with 0x0082
			if (!mp29816a_set_page(bus, addr, 3))
				return 1;
			memcpy(data, (uint8_t[]){ 0x82, 0x00 }, 2);
			mp29816a_i2c_write(bus, addr, 0x81, data, 2);

			// write page2@1a with %04x\n", 0x0f80 | ((cfg_idx - 1) << 4)
			if (!mp29816a_set_page(bus, addr, 2))
				return 1;
			uint16_t temp_val = 0x0f80 | ((cfg_idx - 1) << 4);
			memcpy(data, (uint8_t[]){ (temp_val & 0x00FF), (temp_val & 0xFF00) >> 8 },
			       2);
			mp29816a_i2c_write(bus, addr, 0x1a, data, 2);
		}

		if (p->cfg_page != page) {
			// switch page
			page = p->cfg_page;
			// set page with page
			if (!mp29816a_set_page(bus, addr, page))
				return 1;
		}

		// write register
		// write reg p->reg_addr with p->reg_val with p->reg_len
		memcpy(data, &p->reg_val, p->reg_len);
		mp29816a_i2c_write(bus, addr, p->reg_addr, data, p->reg_len);
	}

	// store last config result
	// write page1@cc with 0x1200
	if (!mp29816a_set_page(bus, addr, 1))
		return 1;
	memcpy(data, (uint8_t[]){ 0x00, 0x12 }, 2);
	mp29816a_i2c_write(bus, addr, 0xcc, data, 2);

	// write page0@17
	if (!mp29816a_set_page(bus, addr, 0))
		return 1;
	memset(data, 0, sizeof(data));
	mp29816a_i2c_write(bus, addr, 0x17, data, 0);
	k_msleep(1000);

	// restore selected config map to 0
	// write page2@1a with 0f02
	if (!mp29816a_set_page(bus, addr, 2))
		return 1;
	memcpy(data, (uint8_t[]){ 0x02, 0x0f }, 2);
	mp29816a_i2c_write(bus, addr, 0x1a, data, 2);

	// lock trim
	// write page3@81 with 0x0000
	if (!mp29816a_set_page(bus, addr, 3))
		return 1;
	memcpy(data, (uint8_t[]){ 0x00, 0x00 }, 2);
	mp29816a_i2c_write(bus, addr, 0x81, data, 2);

	// restore register from selected config map
	// write page0@18
	if (!mp29816a_set_page(bus, addr, 0))
		return 1;
	memset(data, 0, sizeof(data));
	mp29816a_i2c_write(bus, addr, 0x18, data, 0);

	return 0;
}

bool mp29816a_fwupdate(uint8_t bus, uint8_t addr, uint8_t *img_buff, uint32_t img_size)
{
	CHECK_NULL_ARG_WITH_RETURN(img_buff, false);

	uint8_t ret = false;

	struct cfg_data *cfg_data_list =
		(struct cfg_data *)malloc(MAX_CFG_LEN * sizeof(struct cfg_data));
	if (!cfg_data_list) {
		LOG_ERR("malloc fail");
		goto exit;
	}

	uint32_t cfg_cnt = parsing_image(img_buff, img_size, cfg_data_list, MAX_CFG_LEN);

	LOG_INF("cfg_cnt %d", cfg_cnt);
	if (!cfg_cnt) {
		LOG_ERR("parsing image fail");
		goto exit;
	}

	mp29816a_do_update(cfg_data_list, cfg_cnt, bus, addr);

	ret = true;
exit:
	free(cfg_data_list);
	return ret;
}

bool mp29816a_get_fw_version(uint8_t bus, uint8_t addr, uint32_t *rev)
{
	CHECK_NULL_ARG_WITH_RETURN(rev, false);

	if (mp29816a_set_page(bus, addr, VR_MPS_PAGE_0) == false) {
		LOG_ERR("Failed to set page before reading config revision");
		return false;
	}

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 2;
	i2c_msg.data[0] = VR_REG_EXPECTED_USER_CRC;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_INF("Failed to read config revision");
		return false;
	}

	*rev = (i2c_msg.data[1] << 8) | i2c_msg.data[0];

	return true;
}

float mp29816a_get_resolution(sensor_cfg *cfg, bool is_vout_max_or_min)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_FAIL_TO_ACCESS);

	uint8_t offset = cfg->offset;
	if (is_vout_max_or_min) {
		offset = PMBUS_VOUT_MAX;
	}
	float reso = 0;

	I2C_MSG msg;
	uint8_t i2c_max_retry = 5;
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;

	switch (offset) {
	case PMBUS_READ_VOUT:
	case PMBUS_VOUT_MAX:
		msg.data[0] = MFR_VOUT_SCALE_LOOP;

		if (i2c_master_read(&msg, i2c_max_retry)) {
			LOG_WRN("I2C read failed");
			break;
		}
		uint16_t mfr_vout_scale_loop = (msg.data[1] << 8) | msg.data[0];
		switch ((mfr_vout_scale_loop & MFR_VID_RES_MASK) >> 10) {
		case 0:
			reso = 0.00625;
			break;
		case 1:
			reso = 0.005;
			break;
		case 2:
			reso = 0.0025;
			break;
		case 3:
			reso = 0.002;
			break;
		case 4:
			reso = 0.001;
			break;
		case 5:
			reso = 0.00390625;
			break;
		case 6:
			reso = 0.001953125;
			break;
		case 7:
			reso = 0.0009765625;
			break;
		default:
			LOG_WRN("vout_reso_set not supported: 0x%x", mfr_vout_scale_loop);
			return reso;
		}
		return reso;
	default:
		LOG_WRN("offset not supported: 0x%x", offset);
		break;
	}
	return reso;
}

bool mp29816a_get_vout_command(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millivolt, false);

	uint8_t data[2] = { 0 };
	if (!mp29816a_i2c_read(cfg->port, cfg->target_addr, PMBUS_VOUT_COMMAND, data,
			       sizeof(data))) {
		return false;
	}

	uint16_t read_value = data[0] | (data[1] << 8);

	float resolution = mp29816a_get_resolution(cfg, false);
	if (resolution == 0)
		return false;
	float val = read_value * resolution * 1000;
	*millivolt = (int)val;
	return true;
}

bool mp29816a_set_vout_command(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millivolt, false);

	float resolution = mp29816a_get_resolution(cfg, false);
	if (resolution == 0)
		return false;
	uint16_t read_value = (*millivolt / resolution) / 1000;
	read_value = read_value & READ_VOUT_MASK;

	uint8_t data[2] = { 0 };
	data[0] = read_value & 0xFF;
	data[1] = (read_value >> 8) & 0xFF;

	if (!mp29816a_i2c_write(cfg->port, cfg->target_addr, PMBUS_VOUT_COMMAND, data,
				sizeof(data))) {
		return false;
	}

	return true;
}

bool mp29816a_get_vr_status(sensor_cfg *cfg, uint8_t rail, uint8_t vr_status_rail,
			    uint16_t *vr_status)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(vr_status, false);

	uint16_t val = 0;

	switch (vr_status_rail) {
	case PMBUS_STATUS_BYTE: {
		uint8_t data[1] = { 0 };
		if (!mp29816a_i2c_read(cfg->port, cfg->target_addr, PMBUS_STATUS_BYTE, data,
				       sizeof(data))) {
			return false;
		}
		val = (uint16_t)data[0];
	} break;
	case PMBUS_STATUS_WORD: {
		uint8_t data[2] = { 0 };
		if (!mp29816a_i2c_read(cfg->port, cfg->target_addr, PMBUS_STATUS_WORD, data,
				       sizeof(data))) {
			return false;
		}
		val = data[0] | (data[1] << 8);
	} break;
	case PMBUS_STATUS_VOUT: {
		uint8_t data[1] = { 0 };
		if (!mp29816a_i2c_read(cfg->port, cfg->target_addr, PMBUS_STATUS_VOUT, data,
				       sizeof(data))) {
			return false;
		}
		val = (uint16_t)data[0];
	} break;
	case PMBUS_STATUS_IOUT: {
		uint8_t data[1] = { 0 };
		if (!mp29816a_i2c_read(cfg->port, cfg->target_addr, PMBUS_STATUS_IOUT, data,
				       sizeof(data))) {
			return false;
		}
		val = (uint16_t)data[0];
	} break;
	case PMBUS_STATUS_INPUT: {
		uint8_t data[1] = { 0 };
		if (!mp29816a_i2c_read(cfg->port, cfg->target_addr, PMBUS_STATUS_INPUT, data,
				       sizeof(data))) {
			return false;
		}
		val = (uint16_t)data[0];
	} break;
	case PMBUS_STATUS_TEMPERATURE: {
		uint8_t data[1] = { 0 };
		if (!mp29816a_i2c_read(cfg->port, cfg->target_addr, PMBUS_STATUS_TEMPERATURE, data,
				       sizeof(data))) {
			return false;
		}
		val = (uint16_t)data[0];
	} break;
	case PMBUS_STATUS_CML: {
		uint8_t data[1] = { 0 };
		if (!mp29816a_i2c_read(cfg->port, cfg->target_addr, PMBUS_STATUS_CML, data,
				       sizeof(data))) {
			return false;
		}
		val = (uint16_t)data[0];
	} break;
	default:
		LOG_ERR("VR[0x%x] not support vr status:0x%x.", cfg->num, vr_status_rail);
		return false;
		break;
	}
	*vr_status = val;
	return true;
}

bool mp29816a_clear_vr_status(sensor_cfg *cfg, uint8_t rail)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = cfg->port;
	i2c_msg.target_addr = cfg->target_addr;
	i2c_msg.tx_len = 1;
	i2c_msg.data[0] = PMBUS_CLEAR_FAULTS;

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("VR[0x%x] clear fault failed.", cfg->num);
		return false;
	}

	return true;
}

uint8_t mp29816a_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	sensor_val *sval = (sensor_val *)reading;
	I2C_MSG msg;
	memset(sval, 0, sizeof(sensor_val));

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = cfg->offset;

	if (i2c_master_read(&msg, retry)) {
		LOG_WRN("I2C read failed");
		return SENSOR_FAIL_TO_ACCESS;
	}

	float val;
	if (cfg->offset == PMBUS_READ_VOUT) {
		uint16_t read_value = ((msg.data[1] << 8) | msg.data[0]) & READ_VOUT_MASK;
		float resolution = mp29816a_get_resolution(cfg, false);
		if (resolution == 0)
			return SENSOR_FAIL_TO_ACCESS;
		val = (float)read_value * resolution;
	} else if (cfg->offset == PMBUS_READ_TEMPERATURE_1 || cfg->offset == PMBUS_READ_POUT ||
		   cfg->offset == PMBUS_READ_IOUT) {
		uint16_t read_value = (msg.data[1] << 8) | msg.data[0];
		val = slinear11_to_float(read_value);
	} else {
		LOG_ERR("Sensor num 0x%x, offset 0x%x not supported.", cfg->num, cfg->offset);
		return SENSOR_FAIL_TO_ACCESS;
	}

	sval->integer = val;
	sval->fraction = (val - sval->integer) * 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t mp29816a_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	cfg->read = mp29816a_read;
	return SENSOR_INIT_SUCCESS;
}
