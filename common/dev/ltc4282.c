#include <stdio.h>
#include <string.h>
#include "sensor.h"
#include "hal_i2c.h"

enum {
	SET_BIT = 0,
	CLEAR_BIT,
};

#define LTC4282_METER_HALT_BIT 5
#define LTC4282_METER_RESET_BIT 6

static int set_clear_bit(I2C_MSG *msg, uint8_t reg, uint8_t bit, uint8_t op, uint8_t retry)
{
	if (msg == NULL) {
		printf("%s null parameter\n", __func__);
		return -1;
	}

	msg->data[0] = reg;
	msg->tx_len = 1;
	msg->rx_len = 1;

	if (i2c_master_read(msg, retry) != 0) {
		return -1;
	}

	msg->data[1] = msg->data[0];
	msg->data[0] = reg;
	msg->tx_len = 2;

	if (op == SET_BIT) {
		msg->data[1] = SETBIT(msg->data[1], bit);
	} else if (op == CLEAR_BIT) {
		msg->data[1] = CLEARBIT(msg->data[1], bit);
	} else {
		return -1;
	}

	if (i2c_master_write(msg, retry) != 0) {
		return -1;
	}

	return 0;
}

static int ltc4282_read_ein(I2C_MSG *msg, double *val, uint8_t retry)
{
	int ret = -1;
	uint64_t energy = 0;
	uint32_t counter = 0;
	bool ticker_overflow = false;
	bool meter_overflow = false;

	if (msg == NULL || val == NULL) {
		printf("%s null parameter\n", __func__);
		return -1;
	}

	// halt meter and tick counter
	if (set_clear_bit(msg, LTC4282_ADC_CONTROL_OFFSET, LTC4282_METER_HALT_BIT, SET_BIT, retry) <
	    0) {
		printf("%s Failed to halt\n", __func__);
		return -1;
	}

	// check overflow
	msg->data[0] = LTC4282_STATUS_OFFSET;
	msg->tx_len = 1;
	msg->rx_len = 1;
	if (i2c_master_read(msg, retry) != 0) {
		goto exit;
	}

	meter_overflow = (msg->data[0] & BIT(0));
	ticker_overflow = (msg->data[0] & BIT(1));

	// tick counter or meter accumulator has overflowed, reset energy meter and counter
	if (meter_overflow || ticker_overflow) {
		printf("%s Reset meter counter and status register\n", __func__);
		if (set_clear_bit(msg, LTC4282_ADC_CONTROL_OFFSET, LTC4282_METER_RESET_BIT, SET_BIT,
				  retry) < 0) {
			printf("%s Failed to reset meter counter and status register\n", __func__);
		}
		if (set_clear_bit(msg, LTC4282_ADC_CONTROL_OFFSET, LTC4282_METER_RESET_BIT,
				  CLEAR_BIT, retry) < 0) {
			printf("%s Failed to reset meter counter and status register\n", __func__);
		}
		goto exit;
	}

	// read energy and counter
	msg->data[0] = LTC4282_ENERGY_OFFSET;
	msg->tx_len = 1;
	msg->rx_len = 10;
	if (i2c_master_read(msg, retry) != 0) {
		goto exit;
	}

	energy = ((uint64_t)msg->data[0] << (uint64_t)40) |
		 ((uint64_t)msg->data[1] << (uint64_t)32) |
		 ((uint64_t)msg->data[2] << (uint64_t)24) |
		 ((uint64_t)msg->data[3] << (uint64_t)16) |
		 ((uint64_t)msg->data[4] << (uint64_t)8) | ((uint64_t)msg->data[5]);
	counter = ((uint32_t)msg->data[6] << (uint32_t)24) |
		  ((uint32_t)msg->data[7] << (uint32_t)16) |
		  ((uint32_t)msg->data[8] << (uint32_t)8) | ((uint32_t)msg->data[9]);

	if (counter == 0) {
		printf("%s No available data\n", __func__);
		goto exit;
	}

	*val = (double)(energy / counter);
	ret = 0;
exit:
	// continue meter and tick counter
	if (set_clear_bit(msg, LTC4282_ADC_CONTROL_OFFSET, LTC4282_METER_HALT_BIT, CLEAR_BIT,
			  retry) < 0) {
		printf("%s Failed to continue\n", __func__);
		return -1;
	}

	return ret;
}

uint8_t ltc4282_read(uint8_t sensor_num, int *reading)
{
	if ((reading == NULL) || (sensor_num > SENSOR_NUM_MAX) ||
	    (sensor_config[sensor_config_index_map[sensor_num]].init_args == NULL)) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	ltc4282_init_arg *init_arg =
		(ltc4282_init_arg *)sensor_config[sensor_config_index_map[sensor_num]].init_args;

	if (!init_arg->r_sense) {
		printf("%s, Rsense hasn't given\n", __func__);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	float Rsense = init_arg->r_sense;
	uint8_t retry = 5;
	double val = 0;
	I2C_MSG msg = { 0 };

	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.data[0] = cfg->offset;
	msg.rx_len = 2;

	if (cfg->offset != LTC4282_ENERGY_OFFSET) {
		if (i2c_master_read(&msg, retry) != 0)
			return SENSOR_FAIL_TO_ACCESS;
	}
	// Refer to LTC4282 datasheet page 23.
	switch (cfg->offset) {
	case LTC4282_VSENSE_OFFSET:
		val = (((msg.data[0] << 8) | msg.data[1]) * 0.04 / 65535 / Rsense);
		break;
	case LTC4282_POWER_OFFSET:
		val = (((msg.data[0] << 8) | msg.data[1]) * 16.64 * 0.04 * 65536 / 65535 / 65535 /
		       Rsense);
		break;
	case LTC4282_VSOURCE_OFFSET:
		val = (((msg.data[0] << 8) | msg.data[1]) * 16.64 / 65535);
		break;
	case LTC4282_ENERGY_OFFSET:
		if (ltc4282_read_ein(&msg, &val, retry) < 0) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		val = (val * 16.64 * 0.04 * 256 / 65535 / 65535 / Rsense);
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
	if (sensor_num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	sensor_config[sensor_config_index_map[sensor_num]].read = ltc4282_read;
	return SENSOR_INIT_SUCCESS;
}
