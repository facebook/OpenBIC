#include "tmp75.h"

#include <stdio.h>

#include "sensor.h"
#include "hal_i2c.h"

#define RETRY 5

bool tmp75_read(uint8_t sensor_num, float *reading)
{
	int8_t val;
	I2C_MSG msg;
	snr_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = cfg->offset;

	if (i2c_master_read(&msg, RETRY)) {
		cfg->cache_status = SENSOR_FAIL_TO_ACCESS;
		printf("Sensor number %x read tmp75 fail\n", sensor_num);
		return false;
	}

	// convert uint8_t to int8_t, tmp75 reading is signed.
	val = (int8_t)msg.data[0];

	// For now report 0 when reading is negative.
	if (val < 0) {
		val = 0;
	}

	*reading = (float)val;
	cfg->cache = *reading;
	cfg->cache_status = SENSOR_READ_SUCCESS;

	return true;
}
