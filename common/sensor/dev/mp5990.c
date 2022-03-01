#include <stdio.h>
#include <string.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "pal.h"

uint8_t mp5990_read(uint8_t sensor_num, int *reading)
{
	uint8_t retry = 5;
	double val;
	I2C_MSG msg = { 0 };

	sensor_cfg *cfg = &sensor_config[SensorNum_SensorCfg_map[sensor_num]];

	msg.bus = cfg->port;
	msg.slave_addr = cfg->slave_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = cfg->offset;

	if (i2c_master_read(&msg, retry))
		return SENSOR_FAIL_TO_ACCESS;

	switch (cfg->offset) {
	case PMBUS_READ_VOUT:
		/* 31.25 mv/LSB */
		val = ((msg.data[1] << 8) | msg.data[0]) * 31.25;
		break;
	case PMBUS_READ_IOUT:
		/* 62.5 mA/LSB */
		val = ((msg.data[1] << 8) | msg.data[0]) * 62.5;
		break;
	case PMBUS_READ_TEMPERATURE_1:
		/* 1 degree c/LSB */
		val = msg.data[0];
		break;
	case PMBUS_READ_POUT:
		/* 1 W/LSB */
		val = ((msg.data[1] << 8) | msg.data[0]);
		break;
	default:
		return SENSOR_NOT_FOUND;
	}

	sensor_val *sval = (sensor_val *)reading;
	memset(sval, 0, sizeof(*sval));

	sval->integer = (int32_t)val;
	sval->fraction = (int32_t)(val * 1000) % 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t mp5990_init(uint8_t sensor_num)
{
	sensor_config[SensorNum_SensorCfg_map[sensor_num]].read = mp5990_read;
	return SENSOR_INIT_SUCCESS;
}
