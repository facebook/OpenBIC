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
#include <string.h>
#include <logging/log.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "pmbus.h"
#include "adm1272.h"

LOG_MODULE_REGISTER(dev_adm1272);

#define REG_PWR_MONITOR_CFG 0xD4
#define ADM1272_EIN_ROLLOVER_CNT_MAX 0x100
#define ADM1272_EIN_SAMPLE_CNT_MAX 0x1000000
#define ADM1272_EIN_ENERGY_CNT_MAX 0x8000
#define OPERATION_REGISTER 0x01;
#define HSC_ENABLE_BIT BIT(7)

bool enable_adm1272_hsc(uint8_t bus, uint8_t addr, uint8_t enable_flag)
{
	uint8_t retry = 5;
	int ret = -1;
	I2C_MSG msg = { 0 };
	msg.bus = bus;
	msg.target_addr = addr;
	msg.tx_len = 2;
	msg.data[0] = OPERATION_REGISTER;
	msg.data[1] = HSC_DISABLE;
	if (enable_flag == HSC_ENABLE)
		msg.data[1] = HSC_ENABLE_BIT;

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Set enable hsc fail");
		return false;
	}

	return true;
}

static int adm1272_convert_real_value(uint8_t vrange, uint8_t irange, float rsense, uint8_t offset,
				      float *val)
{
	CHECK_NULL_ARG_WITH_RETURN(val, -1);

	float convert_coefficient_m = 0;
	int convert_coefficient_b = 0;
	int convert_coefficient_R = 0;

	switch (offset) {
	case PMBUS_READ_VIN:
	case PMBUS_READ_VOUT:
		switch (vrange) {
		case VRANGE_0V_TO_60V:
			convert_coefficient_m = 6770;
			break;
		case VRANGE_0V_TO_100V:
			convert_coefficient_m = 4062;
			break;
		default:
			LOG_ERR("Vrange 0x%x is invalid", vrange);
			return -1;
		}
		convert_coefficient_b = 0;
		convert_coefficient_R = 100;
		break;

	case PMBUS_READ_IOUT:
		switch (irange) {
		case IRANGE_0MV_TO_15MV:
			convert_coefficient_m = 1326;
			break;
		case IRANGE_0MV_TO_30MV:
			convert_coefficient_m = 663;
			break;
		default:
			LOG_ERR("Irange 0x%x is invalid", irange);
			return -1;
		}
		convert_coefficient_m *= rsense;
		convert_coefficient_b = 20480;
		convert_coefficient_R = 10;
		break;

	case PMBUS_READ_TEMPERATURE_1:
		convert_coefficient_m = 42;
		convert_coefficient_b = 31871;
		convert_coefficient_R = 10;
		break;

	case PMBUS_READ_PIN:
		switch (vrange) {
		case VRANGE_0V_TO_60V:
			switch (irange) {
			case IRANGE_0MV_TO_15MV:
				convert_coefficient_m = 3512;
				convert_coefficient_R = 100;
				break;
			case IRANGE_0MV_TO_30MV:
				convert_coefficient_m = 17561;
				convert_coefficient_R = 1000;
				break;
			default:
				LOG_ERR("Irange 0x%x is invalid", irange);
				return -1;
			}
			break;
		case VRANGE_0V_TO_100V:
			switch (irange) {
			case IRANGE_0MV_TO_15MV:
				convert_coefficient_m = 21071;
				convert_coefficient_R = 1000;
				break;
			case IRANGE_0MV_TO_30MV:
				convert_coefficient_m = 10535;
				convert_coefficient_R = 1000;
				break;
			default:
				LOG_ERR("Irange 0x%x is invalid", irange);
				return -1;
			}
			break;
		default:
			LOG_ERR("Vrange 0x%x is invalid", vrange);
			return -1;
		}
		convert_coefficient_m *= rsense;
		convert_coefficient_b = 0;
		break;

	default:
		LOG_ERR("Offset 0x%x is invalid", offset);
		return -1;
	}

	if (convert_coefficient_m == 0) {
		LOG_ERR("Convert coefficient m is invalid, vrange: 0x%x, irange: 0x%x", vrange,
			irange);
		return -1;
	}

	*val = ((*val) * convert_coefficient_R - convert_coefficient_b) / convert_coefficient_m;

	// Only positive IOUT values are used to avoid returning a negative power
	if ((offset == PMBUS_READ_IOUT) && (*val < 0)) {
		*val = 0;
	}

	return 0;
}

static int adm1272_read_pout(sensor_cfg *cfg, float *val)
{
	CHECK_NULL_ARG_WITH_RETURN(val, -1);
	CHECK_NULL_ARG_WITH_RETURN(cfg, -1);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, -1);

	adm1272_init_arg *init_arg = (adm1272_init_arg *)cfg->init_args;

	uint8_t retry = 5;
	int ret = 0;
	float vout = 0;
	float iout = 0;
	I2C_MSG msg = { 0 };
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = PMBUS_READ_VOUT;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Read vout fail, ret: %d", ret);
		return -1;
	}
	vout = (msg.data[1] << 8) | msg.data[0];
	ret = adm1272_convert_real_value(init_arg->pwr_monitor_cfg.fields.VRANGE,
					 init_arg->pwr_monitor_cfg.fields.IRANGE,
					 init_arg->r_sense_mohm, PMBUS_READ_VOUT, &vout);
	if (ret != 0) {
		LOG_ERR("Convert vout value fail");
		return -1;
	}

	memset(&msg, 0, sizeof(msg));
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = PMBUS_READ_IOUT;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Read iout fail, ret: %d", ret);
		return -1;
	}
	iout = (msg.data[1] << 8) | msg.data[0];
	ret = adm1272_convert_real_value(init_arg->pwr_monitor_cfg.fields.VRANGE,
					 init_arg->pwr_monitor_cfg.fields.IRANGE,
					 init_arg->r_sense_mohm, PMBUS_READ_IOUT, &iout);
	if (ret != 0) {
		LOG_ERR("Convert iout value fail");
		return -1;
	}

	*val = vout * iout;
	return 0;
}

static int adm1272_read_iin(sensor_cfg *cfg, float *val)
{
	CHECK_NULL_ARG_WITH_RETURN(val, -1);
	CHECK_NULL_ARG_WITH_RETURN(cfg, -1);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, -1);

	adm1272_init_arg *init_arg = (adm1272_init_arg *)cfg->init_args;

	uint8_t retry = 5;
	int ret = 0;
	float vin = 0;
	float pin = 0;
	I2C_MSG msg = { 0 };

	/* Read voltage in */
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = PMBUS_READ_VIN;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Read vin fail, ret: %d", ret);
		return -1;
	}
	vin = (msg.data[1] << 8) | msg.data[0];
	ret = adm1272_convert_real_value(init_arg->pwr_monitor_cfg.fields.VRANGE,
					 init_arg->pwr_monitor_cfg.fields.IRANGE,
					 init_arg->r_sense_mohm, PMBUS_READ_VIN, &vin);
	if ((ret != 0) || (vin == 0)) {
		LOG_ERR("Convert vin value fail");
		return -1;
	}

	/* Read power in */
	memset(&msg, 0, sizeof(msg));
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = PMBUS_READ_PIN;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Read pin fail, ret: %d", ret);
		return -1;
	}
	pin = (msg.data[1] << 8) | msg.data[0];
	ret = adm1272_convert_real_value(init_arg->pwr_monitor_cfg.fields.VRANGE,
					 init_arg->pwr_monitor_cfg.fields.IRANGE,
					 init_arg->r_sense_mohm, PMBUS_READ_PIN, &pin);
	if (ret != 0) {
		LOG_ERR("Convert pin value fail");
		return -1;
	}

	*val = pin / vin;
	return 0;
}

static int adm1272_read_ein(sensor_cfg *cfg, float *val)
{
	CHECK_NULL_ARG_WITH_RETURN(val, -1);
	CHECK_NULL_ARG_WITH_RETURN(cfg, -1);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, -1);

	adm1272_init_arg *init_arg = (adm1272_init_arg *)cfg->init_args;

	uint8_t retry = 5;
	int ret = 0;
	I2C_MSG msg = { 0 };

	uint32_t energy = 0, rollover = 0, sample = 0;
	uint32_t pre_energy = 0, pre_rollover = 0, pre_sample = 0;
	uint32_t sample_diff = 0;
	double energy_diff = 0;

	/* Read EIN */
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 7;
	msg.data[0] = cfg->offset;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Read EIN fail, ret: %d, sensor num: 0x%x", ret, cfg->num);
		return -1;
	}

	/* Record the previous and current data */
#ifdef MORE_THAN_ONE_ADM1272
	pre_energy = init_arg->last_energy;
	pre_rollover = init_arg->last_rollover;
	pre_sample = init_arg->last_sample;

	init_arg->last_energy = energy = (msg.data[2] << 8) | msg.data[1];
	init_arg->last_rollover = rollover = msg.data[3];
	init_arg->last_sample = sample = (msg.data[6] << 16) | (msg.data[5] << 8) | msg.data[4];

	SET_FLAG_WITH_RETURN(init_arg->is_record_ein, true, -1);
#else
	static bool is_record_ein = false;
	static uint32_t last_energy = 0, last_rollover = 0, last_sample = 0;

	pre_energy = last_energy;
	pre_rollover = last_rollover;
	pre_sample = last_sample;

	last_energy = energy = (msg.data[2] << 8) | msg.data[1];
	last_rollover = rollover = msg.data[3];
	last_sample = sample = (msg.data[6] << 16) | (msg.data[5] << 8) | msg.data[4];

	SET_FLAG_WITH_RETURN(is_record_ein, true, -1);
#endif

	if ((pre_rollover > rollover) || ((pre_rollover == rollover) && (pre_energy > energy))) {
		rollover += ADM1272_EIN_ROLLOVER_CNT_MAX;
	}

	if (pre_sample > sample) {
		sample += ADM1272_EIN_SAMPLE_CNT_MAX;
	}

	energy_diff = (double)(rollover - pre_rollover) * ADM1272_EIN_ENERGY_CNT_MAX +
		      (double)energy - (double)pre_energy;
	if (energy_diff < 0) {
		LOG_DBG("Energy difference is less than zero.");
		return -1;
	}

	sample_diff = sample - pre_sample;
	if (sample_diff == 0) {
		LOG_DBG("Sample difference is less than zero.");
		return -1;
	}

	*val = (double)(energy_diff / sample_diff);
	return adm1272_convert_real_value(init_arg->pwr_monitor_cfg.fields.VRANGE,
					  init_arg->pwr_monitor_cfg.fields.IRANGE,
					  init_arg->r_sense_mohm, PMBUS_READ_PIN, val);
}

uint8_t adm1272_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	adm1272_init_arg *init_arg = (adm1272_init_arg *)cfg->init_args;
	if (init_arg->is_init == false) {
		LOG_WRN("Device isn't initialized");
		return SENSOR_UNSPECIFIED_ERROR;
	}

	float val = 0;
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };
	int ret = 0;
	uint8_t offset = cfg->offset;

	if ((offset != PMBUS_READ_POUT) && (offset != PMBUS_READ_IIN) &&
	    (offset != PMBUS_READ_EIN)) {
		msg.bus = cfg->port;
		msg.target_addr = cfg->target_addr;
		msg.tx_len = 1;
		msg.data[0] = offset;
		msg.rx_len = 2;

		ret = i2c_master_read(&msg, retry);
		if (ret != 0) {
			LOG_ERR("I2c read fail, ret: %d, offset: 0x%x", ret, offset);
			return SENSOR_FAIL_TO_ACCESS;
		}
	}

	switch (offset) {
	case PMBUS_READ_VIN:
	case PMBUS_READ_VOUT:
	case PMBUS_READ_IOUT:
	case PMBUS_READ_TEMPERATURE_1:
	case PMBUS_READ_PIN:
		val = (msg.data[1] << 8) | msg.data[0];
		ret = adm1272_convert_real_value(init_arg->pwr_monitor_cfg.fields.VRANGE,
						 init_arg->pwr_monitor_cfg.fields.IRANGE,
						 init_arg->r_sense_mohm, offset, &val);
		break;
	case PMBUS_READ_IIN:
		ret = adm1272_read_iin(cfg, &val);
		break;
	case PMBUS_READ_POUT:
		ret = adm1272_read_pout(cfg, &val);
		break;
	case PMBUS_READ_EIN:
		ret = adm1272_read_ein(cfg, &val);
		break;
	default:
		LOG_ERR("Invalid sensor 0x%x offset 0x%x", cfg->num, offset);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	if (ret != 0) {
		return SENSOR_FAIL_TO_ACCESS;
	}
	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (int)val & 0xFFFF;
	sval->fraction = (val - sval->integer) * 1000;
	return SENSOR_READ_SUCCESS;
}

uint8_t adm1272_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	adm1272_init_arg *init_args = (adm1272_init_arg *)cfg->init_args;
	uint8_t retry = 5;
	int ret = -1;
	I2C_MSG msg = { 0 };

	if (init_args->is_need_set_pwr_cfg == false) {
		goto skip_set_pwr_cfg;
	}

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 3;
	msg.data[0] = REG_PWR_MONITOR_CFG;
	msg.data[1] = init_args->pwr_monitor_cfg.value & 0xFF;
	msg.data[2] = (init_args->pwr_monitor_cfg.value >> 8) & 0xFF;

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Set power monitor config fail, ret: %d", ret);
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}
	memset(&msg, 0, sizeof(msg));

skip_set_pwr_cfg:
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.data[0] = REG_PWR_MONITOR_CFG;
	msg.rx_len = 2;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Read power monitor config fail, ret: %d", ret);
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	init_args->pwr_monitor_cfg.value = ((msg.data[1] << 8) | msg.data[0]);
	init_args->is_init = 1;

	cfg->read = adm1272_read;
	return SENSOR_INIT_SUCCESS;
}
