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
#define UVP_THRESHOLD_MFR_UVP_DELTA_R1 GENMASK(12, 9)
#define UVP_THRESHOLD_MASK GENMASK(8, 0)
#define IOUT_OCP_MASK GENMASK(7, 0)
#define IOUT_SCALE_BIT_A_MASK GENMASK(2, 0)
#define OVP_2_THRESHOLD_MASK GENMASK(12, 9)
#define OVP_1_ABS_MASK GENMASK(8, 0)
#define MFR_VOUT_SCALE_LOOP 0x29
#define MFR_VOUT_LOOP_CTRL 0x8D
#define MFR_VDIFF_GAIN_HALF_R2_BIT BIT(10)
#define OVP_2_ACTION_MASK GENMASK(7, 6)

#define VR_MPS_PAGE_0 0x00
#define VR_MPS_PAGE_1 0x01

#define MP29816A_VOUT_MAX_REG 0x24
#define MP29816A_VOUT_MIN_REG 0x2B
#define MP29816A_SVI3_IOUT_RPT_REG 0x67

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
				LOG_INF("CRC: %s", log_strdup(k));
				return 1;
			} else if (!strncmp(k, "TRIM", strlen("TRIM"))) {
				LOG_INF("TRIM: %s", log_strdup(k));
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
	float tmp_value = ((*millivolt / 1000.0f) / resolution) + 0.5f;
	uint16_t read_value = (uint16_t)tmp_value;

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

bool mp29816a_get_iout_oc_warn_limit(sensor_cfg *cfg, uint16_t *value)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(value, false);

	uint8_t data[2] = { 0 };
	uint8_t scale_bit = 0;
	if (!mp29816a_i2c_read(cfg->port, cfg->target_addr, MP29816A_SVI3_IOUT_RPT_REG, data,
			       sizeof(data))) {
		return false;
	}
	scale_bit = data[0] & 0x07;

	if (!mp29816a_i2c_read(cfg->port, cfg->target_addr, PMBUS_IOUT_OC_WARN_LIMIT, data,
			       sizeof(data))) {
		return false;
	}

	// final value: 8 * data * scale_bit
	if (scale_bit == 7) {
		// 2A for scale_bit
		*value = 8 * (data[0] | (data[1] << 8)) * 2;
	} else if (scale_bit < 6 && scale_bit > 0) {
		// 1/32, 1/16, 1/8 ...
		*value = 8 * (data[0] | (data[1] << 8)) / (2 ^ (6 - scale_bit));
	} else {
		// 1A
		*value = 8 * (data[0] | (data[1] << 8));
	}

	return true;
}

bool mp29816a_set_iout_oc_warn_limit(sensor_cfg *cfg, uint16_t value)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);

	uint8_t data[2] = { 0 };
	uint8_t scale_bit = 0;
	if (!mp29816a_i2c_read(cfg->port, cfg->target_addr, MP29816A_SVI3_IOUT_RPT_REG, data,
			       sizeof(data))) {
		return false;
	}
	scale_bit = data[0] & 0x07;

	// final value: (data / scale_bit) / 8
	if (scale_bit == 7) {
		// 2A for scale_bit
		value = (value / 2) / 8;
	} else if (scale_bit < 6 && scale_bit > 0) {
		// 1/32, 1/16, 1/8 ...
		value = (value * (2 ^ (6 - scale_bit))) / 8;
	} else {
		// 1A
		value = value / 8;
	}

	data[0] = value & 0xFF;
	data[1] = value >> 8;

	if (!mp29816a_i2c_write(cfg->port, cfg->target_addr, PMBUS_IOUT_OC_WARN_LIMIT, data,
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
#ifdef MP29816C_MTP_MUTI_CONFIG_PROGRAM_ENABLE
				memcpy(data, (uint8_t[]){ 0x00, 0x02 }, 2);
#else
				memcpy(data, (uint8_t[]){ 0x00, 0x12 }, 2);
#endif
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
#ifdef MP29816C_MTP_MUTI_CONFIG_PROGRAM_ENABLE
	memcpy(data, (uint8_t[]){ 0x00, 0x02 }, 2);
#else
	memcpy(data, (uint8_t[]){ 0x00, 0x12 }, 2);
#endif
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

static float get_vout_cal_offset(uint16_t reg_value, float vid_step)
{
	int8_t raw = (int8_t)(reg_value & 0xFF); // bits[7:0]

	return (float)raw * vid_step;
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

	uint8_t data_cal_offset[2] = { 0 };
	if (!mp29816a_i2c_read(cfg->port, cfg->target_addr, PMBUS_VOUT_TRIM, data_cal_offset,
			       sizeof(data_cal_offset))) {
		return false;
	}

	uint16_t read_value = data[0] | (data[1] << 8);
	uint16_t val_cal_offset = data_cal_offset[0] | (data_cal_offset[1] << 8);

	float vid_step = mp29816a_get_resolution(cfg, false);
	if (vid_step == 0)
		return false;

	float offset_mv = get_vout_cal_offset(val_cal_offset, vid_step) * 1000.0f;
	float val = (read_value * 1000.0f) * vid_step + offset_mv;
	*millivolt = (int)val;
	return true;
}

bool mp29816a_set_vout_command(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millivolt, false);

	float vid_step = mp29816a_get_resolution(cfg, false);
	if (vid_step == 0)
		return false;

	uint8_t data_cal_offset[2] = { 0 };
	if (!mp29816a_i2c_read(cfg->port, cfg->target_addr, PMBUS_VOUT_TRIM, data_cal_offset,
			       sizeof(data_cal_offset))) {
		return false;
	}

	uint16_t val_cal_offset = data_cal_offset[0] | (data_cal_offset[1] << 8);
	float offset_mv = get_vout_cal_offset(val_cal_offset, vid_step);

	float tmp_value = ((*millivolt - offset_mv) / 1000.0f) / vid_step;
	uint16_t set_value = (uint16_t)(tmp_value + 0.5f);
	set_value = set_value & READ_VOUT_MASK;

	uint8_t data[2] = { 0 };
	data[0] = set_value & 0xFF;
	data[1] = (set_value >> 8) & 0xFF;

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
		mp29816a_set_page(cfg->port, cfg->target_addr, 0);
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
		mp29816a_set_page(cfg->port, cfg->target_addr, 0);
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
// to do get_Vout_offset (0x22)
bool mp29816a_get_vout_offset(sensor_cfg *cfg, uint16_t *vout_offset)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(vout_offset, false);
	uint8_t data_cal_offset[2] = { 0 };
	if (!mp29816a_i2c_read(cfg->port, cfg->target_addr, PMBUS_VOUT_TRIM, data_cal_offset,
			       sizeof(data_cal_offset))) {
		return false;
	}

	uint16_t val_cal_offset = data_cal_offset[0];

	float vid_step = mp29816a_get_resolution(cfg, false);
	if (vid_step == 0)
		return false;

	float offset_mv = val_cal_offset * (vid_step * 1000); // mV
	*vout_offset = (uint16_t)offset_mv;
	return true;
}

// to do set_Vout_offset (0x22)
bool mp29816a_set_vout_offset(sensor_cfg *cfg, uint16_t *write_vout_offset)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(write_vout_offset, false);

	uint8_t mfr_special_offset[2] = { 0 };
	float vid_step;
	int16_t write_value;
	uint16_t raw;

	vid_step = mp29816a_get_resolution(cfg, false);
	if (vid_step == 0)
		return false;

	write_value = (int16_t)((float)(*write_vout_offset) / vid_step);
	if (write_value > INT16_MAX || write_value < INT16_MIN) {
		LOG_ERR("Vout offset overflow: %d", write_value);
		return false;
	}
	raw = (uint16_t)write_value;
	mfr_special_offset[0] = raw & 0xFF;
	mfr_special_offset[1] = (raw >> 8) & 0xFF;

	if (!mp29816a_i2c_write(cfg->port, cfg->target_addr, PMBUS_VOUT_TRIM, mfr_special_offset,
				sizeof(mfr_special_offset))) {
		return false;
	}

	return true;
}

bool mp29816a_get_uvp(sensor_cfg *cfg, uint16_t *uvp)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(uvp, false);
	uint8_t data[2] = { 0, 0 };
	if (!mp29816a_i2c_read(cfg->port, cfg->target_addr, PMBUS_VOUT_UV_FAULT_LIMIT, data,
			       sizeof(data))) {
		return false;
	}
	uint16_t read_value = data[0] | (data[1] << 8);
	/*
	Delta value to set rail1 under voltage protection threshold. Max protection
	Value is -500mV.
	3'b000: disable UVP offset
	Others: UVP offset = MFR_UVP_DELTA_R1 * (-50mV) - 50 mV
	*/
	uint8_t uvp_offset_value = (read_value & UVP_THRESHOLD_MFR_UVP_DELTA_R1) >> 9;
	/*
	value = Vout cmd + UVP offset + Vout offset
	*/
	int16_t uvp_offset = (uvp_offset_value * (-50)) - 50;
	uint16_t vout_cmd = 0;
	mp29816a_get_vout_command(cfg, 0, &vout_cmd);
	uint16_t vout_offset = 0;
	mp29816a_get_vout_offset(cfg, &vout_offset);

	*uvp = (uint16_t)(vout_cmd + uvp_offset + vout_offset);
	return true;
}

bool mp29816a_set_uvp_threshold(sensor_cfg *cfg, uint16_t *write_uvp_threshold)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(write_uvp_threshold, false);
	uint8_t data[2] = { 0, 0 };
	if (!mp29816a_i2c_read(cfg->port, cfg->target_addr, PMBUS_VOUT_UV_FAULT_LIMIT, data,
			       sizeof(data))) {
		return false;
	}
	uint16_t read_value = data[0] | (data[1] << 8);
	/*
	3'b000: disable UVP offset
	Others: UVP offset = MFR_UVP_DELTA_R1 * (-50mV) - 50 mV
	*/
	// let write_uvp_threshold be negative

	uint16_t write_in_value = (-*write_uvp_threshold + 50) / (-50);
	if (write_in_value > 9) {
		LOG_ERR("UVP threshold overflow: %d, set to 9", write_in_value);
		write_in_value = 9;
	}

	read_value &= ~UVP_THRESHOLD_MFR_UVP_DELTA_R1;
	read_value |= write_in_value << 9;
	data[0] = read_value & 0xFF;
	data[1] = (read_value >> 8) & 0xFF;
	if (!mp29816a_i2c_write(cfg->port, cfg->target_addr, PMBUS_VOUT_UV_FAULT_LIMIT, data,
				sizeof(data))) {
		return false;
	}
	return true;
}

float get_iout_scale_bit_a(sensor_cfg *cfg)
{
	uint8_t iout_scale_bit[2] = { 0, 0 };
	if (!mp29816a_i2c_read(cfg->port, cfg->target_addr, PMBUS_IOUT_SCALE_BIT_A, iout_scale_bit,
			       sizeof(iout_scale_bit))) {
		LOG_ERR("VR[0x%x] get iout scale bit failed", cfg->num);
	}
	/*
	Output current scaling selection of rail1.
	3'b000: 1 A/LSB (Reserved)
	3'b001: (1/32) A/LSB
	3'b010: (1/16) A/LSB
	3'b011: (1/8) A/LSB
	3'b100: (1/4) A/LSB
	3'b101: (1/2) A/LSB
	3'b110: 1 A/LSB
	3'b111: 2 A/LSB
	*/
	uint8_t scale_bit = iout_scale_bit[0] & IOUT_SCALE_BIT_A_MASK;
	float return_value = 0;
	switch (scale_bit) {
	case 0:
		return_value = 1;
		break;
	case 1:
		return_value = 0.03125;
		break;
	case 2:
		return_value = 0.0625;
		break;
	case 3:
		return_value = 0.125;
		break;
	case 4:
		return_value = 0.25;
		break;
	case 5:
		return_value = 0.5;
		break;
	case 6:
		return_value = 1;
		break;
	case 7:
		return_value = 2;
		break;
	default:
		LOG_ERR("VR[0x%x] get unknown iout scale bit failed", cfg->num);
		break;
	}

	return return_value;
}
// to do get_total_ocp (0x46) PMBUS_IOUT_OC_FAULT_LIMIT
bool mp29816a_get_total_ocp(sensor_cfg *cfg, uint16_t *total_ocp)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(total_ocp, false);
	uint8_t iout_oc_fault_limit[2] = { 0 };
	//OCP threshold =  total_ocp_reg_value[7:0] * 8 *IOUT_SCALE_BIT_A(P1/67h bit[2:0])

	if (!mp29816a_i2c_read(cfg->port, cfg->target_addr, PMBUS_IOUT_OC_FAULT_LIMIT,
			       iout_oc_fault_limit, sizeof(iout_oc_fault_limit))) {
		return false;
	}
	//total_ocp_reg_value[7:0]
	uint16_t val_total_ocp = iout_oc_fault_limit[0];
	float iout_scale_bit_a = get_iout_scale_bit_a(cfg);

	*total_ocp = (uint16_t)(val_total_ocp * 8 * iout_scale_bit_a);

	return true;
}
// to do set_ocp (0x46) PMBUS_IOUT_OC_FAULT_LIMIT
bool mp29816a_set_total_ocp(sensor_cfg *cfg, uint16_t *write_total_ocp)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(write_total_ocp, false);

	uint8_t iout_oc_fault_limit[2] = { 0 };
	//OCP threshold =  total_ocp_reg_value[7:0] * 8 *IOUT_SCALE_BIT_A(P1/67h bit[2:0])
	float iout_scale_bit_a = get_iout_scale_bit_a(cfg);
	uint16_t write_value = (uint16_t)(*write_total_ocp / (8 * iout_scale_bit_a));
	if (write_value > INT16_MAX || write_value < INT16_MIN) {
		LOG_ERR("total ocp overflow: %d", write_value);
		return false;
	}
	//total_ocp_reg_value[7:0]
	iout_oc_fault_limit[0] = write_value;
	if (!mp29816a_i2c_write(cfg->port, cfg->target_addr, PMBUS_IOUT_OC_FAULT_LIMIT,
				iout_oc_fault_limit, sizeof(iout_oc_fault_limit))) {
		return false;
	}

	return true;
}

// to do get_ovp_1 (0x40) PMBUS_VOUT_OV_FAULT_LIMIT
bool mp29816a_get_ovp_1(sensor_cfg *cfg, uint16_t *ovp_1)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(ovp_1, false);
	uint8_t ovp_value[2] = { 0 };
	if (!mp29816a_i2c_read(cfg->port, cfg->target_addr, PMBUS_VOUT_OV_FAULT_LIMIT, ovp_value,
			       sizeof(ovp_value))) {
		return false;
	}
	uint16_t val_ovp_1 = ovp_value[0] | (ovp_value[1] << 8);
	/*
	12:9 bit
	Max offset value is 500mV
	3'b000: disable OVP offset
	Others: OVP offset =MFR_OVP_DELTA_R2 * (+50mV) + 50 mV
	*/
	uint16_t ovp2_threshold = (val_ovp_1 & OVP_2_THRESHOLD_MASK) >> 9;
	if (ovp2_threshold > 9) {
		LOG_INF("threshold overflow > 500mV, ovp2_threshold data:0x%x", ovp2_threshold);
		return false;
	}
	ovp2_threshold = (ovp2_threshold * 50) + 50;
	/*
	Set the absolutely voltage level of rail1 OVP.
	10mV/LSB,
	OVP1 threshold =MFR_OVP_ABS_LIMIT_R1+ MFR_OVP_DELTA_R1
	*/

	*ovp_1 = (uint16_t)((val_ovp_1 & OVP_1_ABS_MASK) * 10) + ovp2_threshold;

	return true;
}
// to do set_ovp_1 (0x40) PMBUS_VOUT_OV_FAULT_LIMIT
bool mp29816a_set_ovp_1(sensor_cfg *cfg, uint16_t *write_ovp_1)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(write_ovp_1, false);
	uint8_t ovp_value[2] = { 0 };
	if (!mp29816a_i2c_read(cfg->port, cfg->target_addr, PMBUS_VOUT_OV_FAULT_LIMIT, ovp_value,
			       sizeof(ovp_value))) {
		return false;
	}
	uint16_t val_ovp_1 = ovp_value[0] | (ovp_value[1] << 8);
	uint16_t ovp2_threshold = (val_ovp_1 & OVP_2_THRESHOLD_MASK) >> 9;
	if (ovp2_threshold > 9) {
		LOG_INF("threshold overflow > 500mV, ovp2_threshold data:0x%x", ovp2_threshold);
		return false;
	}
	ovp2_threshold = (ovp2_threshold * 50) + 50;

	if (*write_ovp_1 > INT16_MAX || *write_ovp_1 < INT16_MIN) {
		LOG_ERR("ovp 1 overflow: %d", *write_ovp_1);
		return false;
	}
	uint16_t write_value = (uint16_t)((*write_ovp_1 - ovp2_threshold) / 10);
	uint8_t write_abs_ovp1[2] = { 0 };
	//ovp reg value[8:0]
	val_ovp_1 = (val_ovp_1 & ~OVP_1_ABS_MASK) | (write_value & OVP_1_ABS_MASK);
	write_abs_ovp1[1] = (val_ovp_1 >> 8) & 0xFF;
	write_abs_ovp1[0] = val_ovp_1 & 0xFF;
	if (!mp29816a_i2c_write(cfg->port, cfg->target_addr, PMBUS_VOUT_OV_FAULT_LIMIT,
				write_abs_ovp1, sizeof(write_abs_ovp1))) {
		return false;
	}

	return true;
}
// to do get_ovp_2
bool mp29816a_get_ovp_2_action(sensor_cfg *cfg, uint16_t *ovp_2_action)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(ovp_2_action, false);
	uint8_t ovp_2_response[2] = { 0 };
	if (!mp29816a_i2c_read(cfg->port, cfg->target_addr, PMBUS_VOUT_OV_FAULT_RESPONSE,
			       ovp_2_response, sizeof(ovp_2_response))) {
		return false;
	}
	/*
	7:6 bit
	2’b00: No action
	2’b01: Latch off
	*/
	uint16_t ovp_2_action_value = ovp_2_response[0] | (ovp_2_response[1] << 8);
	*ovp_2_action = (ovp_2_action_value & OVP_2_ACTION_MASK) >> 6;

	return true;
}
// to do set_ovp_2
bool mp29816a_set_ovp_2_action(sensor_cfg *cfg, uint16_t *write_ovp_2_action)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(write_ovp_2_action, false);
	// if write_ovp_2_action is not 0 or 1, return false
	if (*write_ovp_2_action != 0 && *write_ovp_2_action != 1) {
		LOG_ERR("ovp 2 action write value is not 0 or 1: value input:%d",
			*write_ovp_2_action);
		return false;
	}

	uint8_t ovp_2_response[2] = { 0 };
	if (!mp29816a_i2c_read(cfg->port, cfg->target_addr, PMBUS_VOUT_OV_FAULT_RESPONSE,
			       ovp_2_response, sizeof(ovp_2_response))) {
		return false;
	}
	/*
	7:6 bit
	2’b00: No action
	2’b01: Latch off
	// only change 7:6 bit
	*/
	uint16_t ovp_2_action_value = ovp_2_response[0] | (ovp_2_response[1] << 8);
	ovp_2_action_value = (ovp_2_action_value & ~OVP_2_ACTION_MASK) | (*write_ovp_2_action << 6);
	if (!mp29816a_i2c_write(cfg->port, cfg->target_addr, PMBUS_VOUT_OV_FAULT_RESPONSE,
				(uint8_t *)&ovp_2_action_value, sizeof(ovp_2_action_value))) {
		return false;
	}

	return true;
}

bool mp29816a_get_ovp_2(sensor_cfg *cfg, uint16_t *ovp_2)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(ovp_2, false);
	uint8_t ovp_value[2] = { 0 };
	if (!mp29816a_i2c_read(cfg->port, cfg->target_addr, PMBUS_VOUT_OV_FAULT_LIMIT, ovp_value,
			       sizeof(ovp_value))) {
		return false;
	}
	uint16_t val_ovp_1 = ovp_value[0] | (ovp_value[1] << 8);
	/*
	12:9 bit
	Max offset value is 500mV
	3'b000: disable OVP offset
	Others: OVP offset =MFR_OVP_DELTA_R2 * (+50mV) + 50 mV
	*/
	uint16_t ovp2_threshold = (val_ovp_1 & OVP_2_THRESHOLD_MASK) >> 9;
	if (ovp2_threshold > 9) {
		LOG_INF("threshold overflow > 500mV, ovp2_threshold data:0x%x", ovp2_threshold);
		return false;
	}
	ovp2_threshold = (ovp2_threshold * 50) + 50;
	/*
	OVP2  = Vout command + ovp2_threshold + Vout offset
	*/
	uint16_t vout_cmd = 0;
	uint16_t vout_offset = 0;

	if (!mp29816a_get_vout_command(cfg, 0, &vout_cmd)) {
		return false;
	}
	mp29816a_get_vout_offset(cfg, &vout_offset);
	*ovp_2 = (uint16_t)(vout_cmd + ovp2_threshold + vout_offset);

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
	} else if (cfg->offset == PMBUS_READ_VIN) {
		uint16_t read_value = (msg.data[1] << 8) | msg.data[0];
		read_value = read_value & BIT_MASK(10);
		val = slinear11_to_float(read_value);
		val *= 0.03125; // 31.25mV/LSB
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
