#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "sensor.h"
#include "pmbus.h"
#include "hal_i2c.h"

uint8_t ina233_read(uint8_t sensor_num, int *reading)
{
	if (reading == NULL) {
		printf("[%s] input parameter reading is NULL\n", __func__);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	if (sensor_num > SENSOR_NUM_MAX) {
		printf("[%s] sensor 0x%x input parameter is invalid\n", __func__, sensor_num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	int ret = 0;
	float val = 0;
	I2C_MSG msg;
	memset(&msg, 0, sizeof(I2C_MSG));
	*reading = 0;

	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = sensor_config[sensor_config_index_map[sensor_num]].offset;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		printf("[%s] i2c read fail  ret: %d\n", __func__, ret);
		return SENSOR_FAIL_TO_ACCESS;
	}

	uint8_t offset = sensor_config[sensor_config_index_map[sensor_num]].offset;
	val = (float)((msg.data[1] << 8) | msg.data[0]);
	sensor_val *sval = (sensor_val *)reading;
	switch (offset) {
	case PMBUS_READ_VOUT:
		// 1 mV/LSB, unsigned integer
		// m = 8 , b = 0 , r = 2
		val = val / 800;
		break;
	case PMBUS_READ_IOUT:
		// 1 mA/LSB, 2's complement
		// current_lsb = 0.001 , current convert formula = val / (1 / 0.001) = val / 1000
		val = val / 1000;
		break;
	case PMBUS_READ_POUT:
		// 1 Watt/LSB, 2's complement
		// current_lsb = 0.001 , power convert formula = val / (1 / (0.001 * 25)) = val / 40
		val = val / 40;
		break;
	default:
		printf("[%s] not support offset 0x%x\n", __func__, offset);
		return SENSOR_FAIL_TO_ACCESS;
		break;
	}

	sval->integer = (int)val & 0xFFFF;
	sval->fraction = (val - sval->integer) * 1000;
	return SENSOR_READ_SUCCESS;
}

uint8_t ina233_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		printf("[%s] input sensor number is invalid\n", __func__);
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	sensor_config[sensor_config_index_map[sensor_num]].read = ina233_read;
	return SENSOR_INIT_SUCCESS;
}
