#include <stdio.h>
#include <string.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "util_pmbus.h"

uint8_t xdpe15284_read(uint8_t sensor_num, int *reading)
{
	if (reading == NULL) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	sensor_val *sval = (sensor_val *)reading;
	I2C_MSG msg;
	memset(sval, 0, sizeof(sensor_val));

	msg.bus = sensor_config[SensorNum_SensorCfg_map[sensor_num]].port;
	msg.slave_addr = sensor_config[SensorNum_SensorCfg_map[sensor_num]].slave_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = sensor_config[SensorNum_SensorCfg_map[sensor_num]].offset;

	if (i2c_master_read(&msg, retry))
		return SENSOR_FAIL_TO_ACCESS;

	uint8_t offset = sensor_config[SensorNum_SensorCfg_map[sensor_num]].offset;
	if (offset == PMBUS_READ_VOUT) {
		/* ULINEAR16, get exponent from VOUT_MODE */
		float exponent;
		if (!get_exponent_from_vout_mode(sensor_num, &exponent))
			return SENSOR_FAIL_TO_ACCESS;

		float actual_value = ((msg.data[1] << 8) | msg.data[0]) * exponent;
		sval->integer = actual_value;
		sval->fraction = (actual_value - sval->integer) * 1000;
	} else if (offset == PMBUS_READ_IOUT || offset == PMBUS_READ_TEMPERATURE_1 ||
		   offset == PMBUS_READ_POUT) {
		/* SLINEAR11 */
		uint16_t read_value = (msg.data[1] << 8) | msg.data[0];
		float actual_value = slinear11_to_float(read_value);
		if (offset == PMBUS_READ_IOUT && actual_value < 0) {
			/* In the case POUT is 0, IOUT may read small negative value, replace this case with 0 */
			sval->integer = 0;
			sval->fraction = 0;
		} else {
			sval->integer = actual_value;
			sval->fraction = (actual_value - sval->integer) * 1000;
		}
	} else {
		return SENSOR_FAIL_TO_ACCESS;
	}

	return SENSOR_READ_SUCCESS;
}

uint8_t xdpe15284_init(uint8_t sensor_num)
{
	sensor_config[SensorNum_SensorCfg_map[sensor_num]].read = xdpe15284_read;
	return SENSOR_INIT_SUCCESS;
}
