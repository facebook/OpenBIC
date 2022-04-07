#include "adm1278.h"

#include <stdio.h>
#include <string.h>

#include "sensor.h"
#include "hal_i2c.h"
#include "plat_sensor_table.h"

#define RETRY 5

bool adm1278_init()
{
	snr_cfg *cfg = &sensor_config[sensor_config_index_map[SENSOR_NUM_TEMP_HSC]];
	I2C_MSG msg;

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 3;
	// Power monitor configuration register
	msg.data[0] = 0xD4;
	// Enable vin, temp, Set power monitor mode to continuous sampling
	msg.data[1] = 0x1C;
	// Set sample averageing for current, voltage, power to 128 samples
	msg.data[2] = 0x3F;

	if (!i2c_master_write(&msg, RETRY)) {
		memset(&msg, 0, sizeof(msg));
		msg.bus = cfg->port;
		msg.target_addr = cfg->target_addr;
		msg.tx_len = 1;
		msg.data[0] = 0xD4;
		msg.rx_len = 2;
		if (!i2c_master_read(&msg, RETRY)) {
			if ((msg.data[0] == 0x1C) && (msg.data[1] == 0x3F)) {
				return true;
			}
		}
	}

	printf("adm1278 initial fail\n");
	return false;
}

bool adm1278_read(uint8_t sensor_num, float *reading)
{
	snr_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
	float val = 0;
	I2C_MSG msg;

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.data[0] = cfg->offset;
	msg.rx_len = 2;
	if (!i2c_master_read(&msg, RETRY)) {
		// Rsense 0.25m
		if (sensor_num == SENSOR_NUM_VOL_HSCIN) {
			// m = +19599, b = 0, R = -2
			val = (((msg.data[1] << 8) | msg.data[0]) * 100 / 19599.0);
		} else if (sensor_num == SENSOR_NUM_CUR_HSCOUT) {
			// m = +800 * Rsense(mohm), b = +20475, R = -1
			val = ((((msg.data[1] << 8) | msg.data[0]) * 10 - 20475) / 200.0);
		} else if (sensor_num == SENSOR_NUM_TEMP_HSC) {
			// m = +42, b = +31880, R = -1
			val = ((((msg.data[1] << 8) | msg.data[0]) * 10 - 31880) / 42.0);
		} else if (sensor_num == SENSOR_NUM_PWR_HSCIN) {
			// m = +6123 * Rsense(mohm), b = 0, R = -2
			val = ((((msg.data[1] << 8) | msg.data[0]) * 100) / 1530.75);
		}
	} else {
		cfg->cache_status = SENSOR_FAIL_TO_ACCESS;
		printf("Sensor number %x read fail\n", sensor_num);
		return false;
	}
	*reading = val;
	cfg->cache = *reading;
	cfg->cache_status = SENSOR_READ_SUCCESS;
	return true;
}
