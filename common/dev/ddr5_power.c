#include <stdio.h>
#include <logging/log.h>
#include "sensor.h"
#include "hal_i2c.h"

#define PMIC_LID 0b1001
#define PMIC_POWER_REG 0x0C

LOG_MODULE_REGISTER(dev_ddr5_power);

uint8_t ddr5_power_read(uint8_t sensor_num, int *reading)
{
	if (!reading || (sensor_num > SENSOR_NUM_MAX)) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
	if (cfg->init_args == NULL) {
		return SENSOR_UNSPECIFIED_ERROR;
	}
	ddr5_init_power_arg *init_arg = (ddr5_init_power_arg *)cfg->init_args;

	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	msg.bus = cfg->port;
	msg.target_addr = ((PMIC_LID << 3) | (init_arg->HID_code & 0x07));
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = PMIC_POWER_REG;

	if (i2c_master_read(&msg, retry)) {
		LOG_WRN("i2c read failed.\n");
		return SENSOR_FAIL_TO_ACCESS;
	}

	// 125 mW/LSB
	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (msg.data[0] * 125) / 1000;
	sval->fraction = (msg.data[0] * 125) % 1000;
	return SENSOR_READ_SUCCESS;
}

uint8_t ddr5_power_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	sensor_config[sensor_config_index_map[sensor_num]].read = ddr5_power_read;
	return SENSOR_INIT_SUCCESS;
}
