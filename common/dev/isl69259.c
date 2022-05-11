#include <stdio.h>
#include <string.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "pmbus.h"
#include "isl69259.h"

void adjust_of_two_complement(uint8_t offset, int *val)
{
	if (val == NULL) {
		printf("[%s] input value is NULL\n", __func__);
		return;
	}
	int adjust_val = *val;
	if ((adjust_val & TWO_COMPLEMENT_NEGATIVE_BIT) != 0) {
		// Convert two's complement to usigned integer
		adjust_val = ~(adjust_val - 1);
	}

	if (offset == PMBUS_READ_IOUT) {
		// Set reading value to 0 if reading value in adjust range
		if (adjust_val < ADJUST_IOUT_RANGE) {
			*val = 0;
		}
	}
}

uint8_t isl69259_read(uint8_t sensor_num, int *reading)
{
	if (reading == NULL || (sensor_num > SENSOR_NUM_MAX)) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	int val = 0;
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
	val = (msg.data[1] << 8) | msg.data[0];
	if (offset == PMBUS_READ_VOUT) {
		/* 1 mV/LSB, unsigned integer */
		sval->integer = val / 1000;
		sval->fraction = val % 1000;
	} else if (offset == PMBUS_READ_IOUT) {
		/* 0.1 A/LSB, 2's complement */
		adjust_of_two_complement(PMBUS_READ_IOUT, &val);
		sval->integer = (int16_t)val / 10;
		sval->fraction = (int16_t)(val - (sval->integer * 10)) * 100;
	} else if (offset == PMBUS_READ_TEMPERATURE_1) {
		/* 1 Degree C/LSB, 2's complement */
		sval->integer = val;
	} else if (offset == PMBUS_READ_POUT) {
		/* 1 Watt/LSB, 2's complement */
		sval->integer = val;
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
