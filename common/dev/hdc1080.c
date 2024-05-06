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
#include <logging/log.h>
#include "sensor.h"
#include "hal_i2c.h"

#include "hdc1080.h"

LOG_MODULE_REGISTER(dev_hdc1080);

static sys_slist_t hdc1080_list;

#define RAW2TEMP(raw) (((float)raw) * 165 / 65536 - 40)
#define RAW2HUM(raw) (((float)raw) * 100 / 65536)

hdc1080_data *find_hdc1080_from_idx(uint8_t idx)
{
	sys_snode_t *hdc1080_node = NULL;
	SYS_SLIST_FOR_EACH_NODE (&hdc1080_list, hdc1080_node) {
		hdc1080_data *p;
		p = CONTAINER_OF(hdc1080_node, hdc1080_data, node);
		if (p->idx == idx) {
			return p;
		}
	}

	return NULL;
}

static uint8_t get_temp_conversion_time(uint8_t resolution)
{
	return (resolution == TEMP_11_BIT) ? 4 : 7;
}

static uint8_t get_hum_conversion_time(uint8_t resolution)
{
	return (resolution == HUM_8_BIT) ? 3 : (resolution == HUM_11_BIT) ? 4 : 7;
}

static bool hdc1080_read_val(sensor_cfg *cfg, float *val)
{
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	hdc1080_data *priv_data = (hdc1080_data *)cfg->priv_data;

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.data[0] = cfg->offset;

	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("hdc1080 read offset 0x%02x fail", cfg->offset);
		return false;
	}

	// wait conversion time
	switch (cfg->offset) {
	case HDC1080_TEMP_OFFSET:
		k_msleep(get_temp_conversion_time(priv_data->tres));
		break;
	case HDC1080_HUM_OFFSET:
		k_msleep(get_hum_conversion_time(priv_data->hres));
		break;
	default:
		LOG_ERR("0x%02x get %d conversion time fail", cfg->num, cfg->offset);
		k_msleep(7); // wait max conversion time
		break;
	}

	msg.tx_len = 0;
	msg.rx_len = (priv_data->mode == TEMP_OR_HUM) ? 2 : 4;

	if (i2c_master_read(&msg, retry))
		return false;

	switch (cfg->offset) {
	case HDC1080_TEMP_OFFSET:
		*val = RAW2TEMP((msg.data[0] << 8 | msg.data[1]));
		break;
	case HDC1080_HUM_OFFSET:
		*val = (priv_data->mode == TEMP_OR_HUM) ?
			       RAW2HUM((msg.data[0] << 8 | msg.data[1])) :
			       RAW2HUM((msg.data[2] << 8 | msg.data[3]));
		break;
	default:
		LOG_ERR("0x%02x offset %d read fail", cfg->num, cfg->offset);
		break;
	}

	return true;
}

uint8_t hdc1080_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor nvme 0x%x input parameter is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	float val;
	if (!hdc1080_read_val(cfg, &val)) {
		LOG_ERR("Invalid offset: 0x%x", cfg->offset);
		return SENSOR_FAIL_TO_ACCESS;
	}

	*reading = (int)val;
	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (int)val & 0xFFFF;
	sval->fraction = (val - sval->integer) * 1000;
	return SENSOR_READ_SUCCESS;
}

uint8_t hdc1080_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	hdc1080_init_arg *init_arg = (hdc1080_init_arg *)cfg->init_args;

	hdc1080_data *p;
	p = find_hdc1080_from_idx(init_arg->idx);
	if (p == NULL) {
		p = (hdc1080_data *)malloc(sizeof(hdc1080_data));
		if (!p) {
			LOG_ERR("The hdc1080_data malloc failed at index %d", init_arg->idx);
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}

		p->idx = init_arg->idx;
		p->mode = init_arg->mode;
		p->tres = init_arg->tres;
		p->hres = init_arg->hres;

		sys_slist_append(&hdc1080_list, &p->node);

		// write cfg reg
		uint8_t retry = 5;
		I2C_MSG msg = { 0 };

		msg.bus = cfg->port;
		msg.target_addr = cfg->target_addr;
		msg.tx_len = 3;

		msg.data[0] = HDC1080_CONFIGURE_OFFSET;
		msg.data[1] = (init_arg->mode << 4) | (init_arg->tres << 2) | (init_arg->hres);
		msg.data[2] = 0; //Reserved

		if (i2c_master_write(&msg, retry)) {
			LOG_ERR("Write config reg failed");
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}
	}

	cfg->priv_data = p;
	cfg->read = hdc1080_read;
	return SENSOR_INIT_SUCCESS;
}