#include <stdio.h>
#include <string.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "isl28022.h"

uint8_t isl28022_read(uint8_t sensor_num, int *reading)
{
	if ((reading == NULL) || (sensor_num > SENSOR_NUM_MAX) ||
	    (sensor_config[sensor_config_index_map[sensor_num]].init_args == NULL)) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	isl28022_init_arg *init_arg =
		(isl28022_init_arg *)sensor_config[sensor_config_index_map[sensor_num]].init_args;
	if (init_arg->is_init == false) {
		printf("isl28022_read, device isn't initialized\n");
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	I2C_MSG msg;
	uint8_t offset = sensor_config[sensor_config_index_map[sensor_num]].offset;
	sensor_val *sval = (sensor_val *)reading;
	memset(sval, 0, sizeof(sensor_val));

	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = offset;
	if (i2c_master_read(&msg, retry)) {
		/* read fail */
		return SENSOR_FAIL_TO_ACCESS;
	}

	if (offset == ISL28022_BUS_VOLTAGE_REG) {
		/* unsigned */
		uint16_t read_mv;

		if ((init_arg->config.fields.BRNG == 0b11) ||
		    (init_arg->config.fields.BRNG == 0b10)) {
			read_mv = ((msg.data[0] << 6) | (msg.data[1] >> 2)) * 4;
		} else if (init_arg->config.fields.BRNG == 0b01) {
			read_mv = ((msg.data[0] << 5) | (msg.data[1] >> 3)) * 4;
		} else {
			read_mv = (((msg.data[0] & BIT_MASK(7)) << 5) | (msg.data[1] >> 3)) * 4;
		}
		sval->integer = read_mv / 1000;
		sval->fraction = read_mv % 1000;
	} else if (offset == ISL28022_CURRENT_REG) {
		/* signed */
		float read_current =
			((int16_t)(msg.data[0] << 8) | msg.data[1]) * init_arg->current_LSB;
		sval->integer = read_current;
		sval->fraction = (read_current - sval->integer) * 1000;
	} else if (offset == ISL28022_POWER_REG) {
		/* unsigned */
		float read_power = ((msg.data[0] << 8) | msg.data[1]) * init_arg->current_LSB * 20;

		if (init_arg->config.fields.BRNG > 1) {
			/* follow from spec, the power multiplied by 2 if usable full scale range is 60 */
			read_power *= 2;
		}
		sval->integer = read_power;
		sval->fraction = ((read_power - sval->integer) * 1000);
	} else {
		return SENSOR_FAIL_TO_ACCESS;
	}

	return SENSOR_READ_SUCCESS;
}

uint8_t isl28022_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	if (sensor_config[sensor_config_index_map[sensor_num]].init_args == NULL) {
		printf("isl28022_init: init_arg is NULL sensor_num = 0x%x\n", sensor_num);
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	sensor_config[sensor_config_index_map[sensor_num]].read = isl28022_read;
	isl28022_init_arg *init_arg =
		(isl28022_init_arg *)sensor_config[sensor_config_index_map[sensor_num]].init_args;
	if (init_arg->is_init == true) {
		return SENSOR_INIT_SUCCESS;
	}

	I2C_MSG msg;
	uint8_t retry = 5;

	/* set configuration register */
	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 3;
	msg.data[0] = ISL28022_CONFIG_REG;
	msg.data[1] = (init_arg->config.value >> 8) & 0xFF;
	msg.data[2] = init_arg->config.value & 0xFF;
	if (i2c_master_write(&msg, retry)) {
		printf("isl28022_init, set configuration register fail\n");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	/* calculate and set calibration */
	uint16_t v_shunt_fs, adc_res, calibration;

	v_shunt_fs = 40 << (init_arg->config.fields.PG);
	if (!(init_arg->config.fields.SADC & BIT(3)) &&
	    ((init_arg->config.fields.SADC & BIT_MASK(2)) < 3)) {
		adc_res = 1 << (12 + (init_arg->config.fields.SADC & BIT_MASK(2)));
	} else {
		adc_res = 32768;
	}
	init_arg->current_LSB = (float)v_shunt_fs / (init_arg->r_shunt * adc_res);
	calibration = (40.96 / (init_arg->current_LSB * init_arg->r_shunt));
	calibration = calibration & 0xFFFE; /* 16 bits, bit[0] is fix to 0 */

	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 3;
	msg.data[0] = ISL28022_CALIBRATION_REG;
	msg.data[1] = (calibration >> 8) & 0xFF;
	msg.data[2] = calibration & 0xFF;
	if (i2c_master_write(&msg, retry)) {
		printf("isl28022_init, set calibration register fail\n");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	init_arg->is_init = true;
	return SENSOR_INIT_SUCCESS;
}
