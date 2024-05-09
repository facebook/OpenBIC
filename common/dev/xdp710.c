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

#include <stdlib.h>
#include <logging/log.h>
#include "pmbus.h"
#include "sensor.h"
#include "xdp710.h"

LOG_MODULE_REGISTER(dev_xdp710);

#define VSNS_CFG_REG 0xD4
#define ISNS_CFG_REG 0xD5

#define VSNS_CFG_RESET_VAL 0x28
#define ISNS_CFG_RESET_VAL 0xB418

static const uint16_t vin_vtlm_to_m[VTLM_RNG_RESERVED] = { 4653, 9307, 18614 };
static const uint16_t iout_vsns_to_m[VSNS_CS_MAX] = { 23165, 11582, 5791, 28956 };
static const uint16_t pin_vtlm_vsns_to_m[VTLM_RNG_RESERVED][VSNS_CS_MAX] = {
	{ 4211, 21054, 10527, 5263 },
	{ 8422, 4211, 21054, 10527 },
	{ 16843, 8422, 4211, 21054 }
};
static const uint16_t pin_vtlm_vsns_to_r[VTLM_RNG_RESERVED][VSNS_CS_MAX] = {
	{ 100, 1000, 1000, 1000 },
	{ 100, 100, 1000, 1000 },
	{ 100, 100, 100, 1000 }
};

static uint8_t xdp710_read_vsns_cfg(uint8_t bus, uint8_t addr)
{
	uint8_t val = VSNS_CFG_RESET_VAL;

	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	msg.bus = bus;
	msg.target_addr = addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = VSNS_CFG_REG;

	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("read vsns cfg fail!");
		return val;
	}

	val = msg.data[0];
	return val;
}

static uint16_t xdp710_read_isns_cfg(uint8_t bus, uint8_t addr)
{
	uint16_t val = ISNS_CFG_RESET_VAL;

	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	msg.bus = bus;
	msg.target_addr = addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = ISNS_CFG_REG;

	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("read isns cfg fail!");
		return val;
	}

	val = ((msg.data[1] << 8) | msg.data[0]);
	return val;
}

/* set mbr
	1/m * Y * (10^-r) - b
	if r = -2, return (10^-(-2)) -> 100
*/
static bool xdp710_set_mbr(uint8_t offset, uint8_t vtlm, uint8_t vsns, uint16_t *m, uint16_t *b,
			   uint16_t *r)
{
	CHECK_NULL_ARG_WITH_RETURN(m, false);
	CHECK_NULL_ARG_WITH_RETURN(b, false);
	CHECK_NULL_ARG_WITH_RETURN(r, false);

	if (vtlm >= VTLM_RNG_RESERVED || vsns >= VSNS_CS_MAX) {
		LOG_ERR("vtlm_rng: %x, vsns_cs: %x out of range", vtlm, vsns);
		return false;
	}

	switch (offset) {
	case PMBUS_READ_TEMPERATURE_1:
		*m = 52;
		*b = 14321;
		*r = 10;
		break;
	case PMBUS_READ_VIN:
		*m = vin_vtlm_to_m[vtlm];
		*b = 0;
		*r = 100;
		break;
	case PMBUS_READ_IOUT:
		*m = iout_vsns_to_m[vsns];
		*b = 0;
		*r = (vsns == VSNS_CS_100) ? 1000 : 100;
		break;
	case PMBUS_READ_PIN:
		*m = pin_vtlm_vsns_to_m[vtlm][vsns];
		*b = 0;
		*r = pin_vtlm_vsns_to_r[vtlm][vsns];
		break;
	default:
		LOG_WRN("Invalid offset 0x%x", offset);
		return false;
	}

	return true;
}

uint8_t xdp710_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("Sensor number out of range.");
		return SENSOR_UNSPECIFIED_ERROR;
	}

	xdp710_priv *priv_data = (xdp710_priv *)cfg->priv_data;
	if (priv_data->mbr_init == false) {
		LOG_ERR("0x%02x mbr isn't initialized", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	float r_sense = priv_data->r_sense; // mohm
	float val;
	uint8_t retry = 5;
	I2C_MSG msg;
	uint8_t offset = cfg->offset;

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.data[0] = offset;
	msg.rx_len = 2;

	if (i2c_master_read(&msg, retry))
		return SENSOR_FAIL_TO_ACCESS;

	switch (offset) {
	case PMBUS_READ_TEMPERATURE_1:
		val = (float)(((msg.data[1] << 8) | msg.data[0]) * priv_data->r - priv_data->b) /
		      priv_data->m;
		break;
	case PMBUS_READ_VIN:
		val = (float)((msg.data[1] << 8) | msg.data[0]) * priv_data->r / priv_data->m;
		break;
	case PMBUS_READ_IOUT:
	case PMBUS_READ_PIN:
		val = (float)(((msg.data[1] << 8) | msg.data[0]) * priv_data->r) /
		      (priv_data->m * r_sense);
		break;
	default:
		LOG_WRN("Invalid offset 0x%x", offset);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (int)val & 0xFFFF;
	sval->fraction = (val - sval->integer) * 1000;
	return SENSOR_READ_SUCCESS;
}

uint8_t xdp710_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("Sensor number out of range.");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	xdp710_init_arg *init_args = (xdp710_init_arg *)cfg->init_args;
	if (!init_args->r_sense) {
		LOG_ERR("0x%02x r_sense not provided", cfg->num);
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	/* re-init, free priv_data */
	if (cfg->priv_data)
		SAFE_FREE(cfg->priv_data);

	xdp710_priv *priv_data = (xdp710_priv *)malloc(sizeof(priv_data));
	if (priv_data == NULL) {
		LOG_ERR("malloc xdp710 priv_data fail!");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}
	priv_data->mbr_init = false;

	uint8_t vtlm = (xdp710_read_vsns_cfg(cfg->port, cfg->target_addr) & BIT_MASK(2));
	uint8_t vsns = (xdp710_read_isns_cfg(cfg->port, cfg->target_addr) & GENMASK(7, 6));

	uint16_t m = 0, b = 0, r = 0;
	if (xdp710_set_mbr(cfg->offset, vtlm, vsns, &m, &b, &r)) {
		LOG_INF("set 0x%02x to m: %d, b: %d,r: %d", cfg->num, m, b, r);
		priv_data->mbr_init = true;
	}

	/* set priv data */
	priv_data->m = m;
	priv_data->b = b;
	priv_data->r = r;
	priv_data->r_sense = init_args->r_sense;

	cfg->priv_data = priv_data;

	cfg->read = xdp710_read;
	return SENSOR_INIT_SUCCESS;
}
