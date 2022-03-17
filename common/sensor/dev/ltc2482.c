#include <stdio.h>
#include <string.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "sensor_def.h"

uint8_t ltc4282_read(uint8_t sensor_num, int *reading)
{
	if ((reading == NULL) ||
	    (sensor_config[SensorNum_SensorCfg_map[sensor_num]].init_args == NULL)) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	ltc4282_init_arg *init_arg =
		(ltc4282_init_arg *)sensor_config[SensorNum_SensorCfg_map[sensor_num]].init_args;

	if (!init_arg->r_sense) {
		printf("%s, Rsense hasn't given\n", __func__);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	float Rsense = init_arg->r_sense;
	uint8_t retry = 5;
	double val = 0;
	I2C_MSG msg = { 0 };

	sensor_cfg *cfg = &sensor_config[SensorNum_SensorCfg_map[sensor_num]];

	msg.bus = cfg->port;
	msg.slave_addr = cfg->slave_addr;
	msg.tx_len = 1;
	msg.data[0] = cfg->offset;
	msg.rx_len = 2;

	if (i2c_master_read(&msg, retry) != 0)
		return SENSOR_FAIL_TO_ACCESS;

	switch (cfg->offset) {
	case LTC4282_VSENSE_OFFSET:
		val = (((msg.data[0] << 8) | msg.data[1]) * 0.4 / 65535 / Rsense);
		break;
	case LTC4282_POWER_OFFSET:
		val = (((msg.data[0] << 8) | msg.data[1]) * 16.64 * 0.4 / 65535 / Rsense);
		break;
	case LTC4282_VSOURCE_OFFSET:
		val = (((msg.data[0] << 8) | msg.data[1]) * 16.64 / 65535);
		break;
	default:
		printf("Invalid sensor 0x%x offset 0x%x\n", sensor_num, cfg->offset);
		return SENSOR_NOT_FOUND;
	}

	sensor_val *sval = (sensor_val *)reading;
	memset(sval, 0, sizeof(*sval));

	sval->integer = (int)val & 0xFFFF;
	sval->fraction = (val - sval->integer) * 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t ltc4282_init(uint8_t sensor_num)
{
	sensor_config[SensorNum_SensorCfg_map[sensor_num]].read = ltc4282_read;
	return SENSOR_INIT_SUCCESS;
}

