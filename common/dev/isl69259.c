#include <stdio.h>
#include <string.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "pmbus.h"

uint8_t isl69259_read(uint8_t sensor_num, int *reading)
{
	if (reading == NULL || (sensor_num > SENSOR_NUM_MAX)) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	sensor_val *sval = (sensor_val *)reading;
	I2C_MSG msg;
	memset(sval, 0, sizeof(sensor_val));

	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = sensor_config[sensor_config_index_map[sensor_num]].offset;

	if (i2c_master_read(&msg, retry)) {
		/* read fail */
		return SENSOR_FAIL_TO_ACCESS;
	}

	uint8_t offset = sensor_config[sensor_config_index_map[sensor_num]].offset;
	if (offset == PMBUS_READ_VOUT) {
		/* 1 mV/LSB, unsigned integer */
		sval->integer = ((msg.data[1] << 8) | msg.data[0]) / 1000;
		sval->fraction = ((msg.data[1] << 8) | msg.data[0]) % 1000;
	} else if (offset == PMBUS_READ_IOUT) {
		/* 0.1 A/LSB, 2's complement */
		sval->integer = (int16_t)((msg.data[1] << 8) | msg.data[0]) / 10;
		sval->fraction =
			(int16_t)(((msg.data[1] << 8) | msg.data[0]) - (sval->integer * 10)) * 100;
	} else if (offset == PMBUS_READ_TEMPERATURE_1) {
		/* 1 Degree C/LSB, 2's complement */
		sval->integer = ((msg.data[1] << 8) | msg.data[0]);
	} else if (offset == PMBUS_READ_POUT) {
		/* 1 Watt/LSB, 2's complement */
		sval->integer = ((msg.data[1] << 8) | msg.data[0]);
	} else {
		return SENSOR_FAIL_TO_ACCESS;
	}

	return SENSOR_READ_SUCCESS;
}

uint8_t isl69259_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	sensor_config[sensor_config_index_map[sensor_num]].read = isl69259_read;
	return SENSOR_INIT_SUCCESS;
}
