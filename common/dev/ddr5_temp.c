#include <stdio.h>
#include <logging/log.h>
#include "sensor.h"
#include "hal_i2c.h"

#define TS0_LID 0b0010
#define TS1_LID 0b0110
#define TS_SENSE_OFFSET 0x31

LOG_MODULE_REGISTER(dev_ddr5_temp);

uint8_t ddr5_temp_read(uint8_t sensor_num, int *reading)
{
	if (!reading || (sensor_num > SENSOR_NUM_MAX)) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
	if (cfg->init_args == NULL) {
		return SENSOR_UNSPECIFIED_ERROR;
	}
	ddr5_init_temp_arg *init_arg = (ddr5_init_temp_arg *)cfg->init_args;
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	msg.bus = cfg->port;
	msg.target_addr = ((TS0_LID << 3) | (init_arg->HID_code & 0x07));
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = TS_SENSE_OFFSET;

	if (i2c_master_read(&msg, retry)) {
		LOG_WRN("i2c read failed.\n");
		return SENSOR_FAIL_TO_ACCESS;
	}

	float tmp;
	// 0.25 degree C/LSB
	tmp = 0.25 * ((((msg.data[1] << 8) | msg.data[0]) >> 2) & 0x3FF);
	init_arg->ts0_temp = tmp;

	msg.bus = cfg->port;
	msg.target_addr = ((TS1_LID << 3) | (init_arg->HID_code & 0x07));
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = TS_SENSE_OFFSET;

	if (i2c_master_read(&msg, retry)) {
		LOG_WRN("i2c read failed.\n");
		return SENSOR_FAIL_TO_ACCESS;
	}

	tmp = 0.25 * ((((msg.data[1] << 8) | msg.data[0]) >> 2) & 0x3FF);
	init_arg->ts1_temp = tmp;

	float val =
		(init_arg->ts0_temp > init_arg->ts1_temp) ? init_arg->ts0_temp : init_arg->ts1_temp;
	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (int32_t)val;
	sval->fraction = (int32_t)(val * 1000) % 1000;
	return SENSOR_READ_SUCCESS;
}

uint8_t ddr5_temp_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	sensor_config[sensor_config_index_map[sensor_num]].read = ddr5_temp_read;
	return SENSOR_INIT_SUCCESS;
}
