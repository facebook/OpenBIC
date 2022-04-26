#include <stdio.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "pal.h"

bool pal_tmp75_read(uint8_t sensor_num, int *reading)
{
	uint8_t retry = 5;
	int val;
	I2C_MSG msg;

	msg.bus = sensor_config[SnrNum_SnrCfg_map[sensor_num]].port;
	msg.slave_addr = sensor_config[SnrNum_SnrCfg_map[sensor_num]].slave_addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = sensor_config[SnrNum_SnrCfg_map[sensor_num]].offset;

	if (i2c_master_read(&msg, retry)) {
		sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status = SNR_FAIL_TO_ACCESS;
		printf("Snr num %x read tmp75 fail\n", sensor_num);
		return false;
	}

	val = msg.data[0];

	*reading = calculate_MBR(sensor_num, val) & 0xff;
	sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache = *reading;
	sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status = SNR_READ_SUCCESS;

	return true;
}
