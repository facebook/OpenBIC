#include "tmp75.h"

#include <stdio.h>

#include "sensor.h"
#include "hal_i2c.h"

#define RETRY 5

bool tmp75_read(uint8_t sensor_num, float *reading)
{
	int val;
	I2C_MSG msg;
	snr_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];

	msg.bus = cfg->port;
	msg.slave_addr = cfg->slave_addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = cfg->offset;

	if (i2c_master_read(&msg, RETRY)) {
		cfg->cache_status = SNR_FAIL_TO_ACCESS;
		printf("Snr num %x read tmp75 fail\n", sensor_num);
		return false;
	}

	val = msg.data[0];

	*reading = (float)val;
	cfg->cache = *reading;
	cfg->cache_status = SNR_READ_SUCCESS;

	return true;
}
