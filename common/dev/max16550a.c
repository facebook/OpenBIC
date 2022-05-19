#include <stdio.h>
#include <string.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "pmbus.h"

uint8_t max16550a_read(uint8_t sensor_num, int *reading)
{
	if (reading == NULL) {
		printf("[%s] input parameter reading is NULL\n", __func__);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	if ((sensor_num > SENSOR_NUM_MAX) ||
	    (sensor_config[sensor_config_index_map[sensor_num]].init_args == NULL)) {
		printf("[%s] sensor 0x%x input parameter is invalid\n", __func__, sensor_num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	*reading = 0;
	uint8_t retry = 5;
	uint8_t offset = sensor_config[sensor_config_index_map[sensor_num]].offset;
	int ret = 0;
	float val = 0, r_load = 0;
	I2C_MSG msg;
	memset(&msg, 0, sizeof(I2C_MSG));

	max16550a_init_arg *init_arg =
		(max16550a_init_arg *)sensor_config[sensor_config_index_map[sensor_num]].init_args;
	// R_load is the value of resistance connected to EFUSE , and EFUSE would adjust the reading accuracy according to r_load
	r_load = init_arg->r_load;

	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = offset;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		printf("[%s] i2c read fail  sensor number 0x%x  ret: %d\n", __func__, sensor_num,
		       ret);
		return SENSOR_FAIL_TO_ACCESS;
	}

	switch (offset) {
	case PMBUS_READ_VIN:
		// m = +7578, b = +0, R = -2
		val = (float)((msg.data[1] << 8) | msg.data[0]) * 100 / 7578;
		break;
	case PMBUS_READ_VOUT:
		// m = +7578, b = +0, R = -2
		val = (float)((msg.data[1] << 8) | msg.data[0]) * 100 / 7578;
		break;
	case PMBUS_READ_IOUT:
		// m = +3.824 * Rload, b = -4300, R = -3
		val = (float)(((msg.data[1] << 8) | msg.data[0]) * 1000 + 4300) / (3.824 * r_load);
		break;
	case PMBUS_READ_TEMPERATURE_1:
		// m = +199, b = +7046, R = -2
		val = (float)(((msg.data[1] << 8) | msg.data[0]) * 100 - 7046) / 199;
		break;
	case PMBUS_READ_PIN:
		// m = +0.895 * Rload, b = -9100, R = -2
		val = (float)(((msg.data[1] << 8) | msg.data[0]) * 100 + 9100) / (0.895 * r_load);
		break;
	default:
		printf("[%s] not support sensor number 0x%x  offset: 0x%x\n", __func__, sensor_num,
		       offset);
		return SENSOR_NOT_FOUND;
	}

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (int)val & 0xFFFF;
	sval->fraction = (val - sval->integer) * 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t max16550a_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		printf("[%s] input sensor number 0x%x is invalid\n", __func__, sensor_num);
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	sensor_config[sensor_config_index_map[sensor_num]].read = max16550a_read;
	return SENSOR_INIT_SUCCESS;
}
