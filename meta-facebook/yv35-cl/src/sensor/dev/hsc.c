#include <stdio.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "plat_sensor.h"
#include <string.h>

bool hsc_init()
{
	uint8_t retry = 5;
	I2C_MSG msg;

	msg.bus = sensor_config[SnrNum_SnrCfg_map[SENSOR_NUM_TEMP_HSC]].port;
	msg.slave_addr = sensor_config[SnrNum_SnrCfg_map[SENSOR_NUM_TEMP_HSC]].slave_addr;
	msg.tx_len = 3;
	msg.data[0] = 0xD4;
	msg.data[1] = 0x1C;
	msg.data[2] = 0x3F;

	if (!i2c_master_write(&msg, retry)) {
		memset(&msg, 0, sizeof(msg));
		msg.bus = sensor_config[SnrNum_SnrCfg_map[SENSOR_NUM_TEMP_HSC]].port;
		msg.slave_addr = sensor_config[SnrNum_SnrCfg_map[SENSOR_NUM_TEMP_HSC]].slave_addr;
		msg.tx_len = 1;
		msg.data[0] = 0xD4;
		msg.rx_len = 2;
		if (!i2c_master_read(&msg, retry)) {
			if ((msg.data[0] == 0x1C) && (msg.data[1] == 0x3F)) {
				return true;
			}
		}
	}
	printf("HSC initail fail\n");
	return false;
}

bool pal_hsc_read(uint8_t sensor_num, int *reading)
{
	uint8_t retry = 5;
	int val;
	I2C_MSG msg;

	msg.bus = sensor_config[SnrNum_SnrCfg_map[sensor_num]].port;
	msg.slave_addr = sensor_config[SnrNum_SnrCfg_map[sensor_num]].slave_addr;
	msg.tx_len = 1;
	msg.data[0] = sensor_config[SnrNum_SnrCfg_map[sensor_num]].offset;
	msg.rx_len = 2;
	if (!i2c_master_read(&msg, retry)) {
		// Rsense 0.25m
		if (sensor_num == SENSOR_NUM_VOL_HSCIN) {
			// m = +19599, b = 0, R = -2
			val = (((msg.data[1] << 8) | msg.data[0]) * 100 * 1000 / 19599);
		} else if (sensor_num == SENSOR_NUM_CUR_HSCOUT) {
			// m = +800 * Rsense(mohm), b = +20475, R = -1
			val = ((((msg.data[1] << 8) | msg.data[0]) * 10 - 20475) * 1000 / 200);
		} else if (sensor_num == SENSOR_NUM_TEMP_HSC) {
			// m = +42, b = +31880, R = -1
			val = ((((msg.data[1] << 8) | msg.data[0]) * 10 - 31880) * 1000 / 42);
		} else if (sensor_num == SENSOR_NUM_PWR_HSCIN) {
			// m = +6123 * Rsense(mohm), b = 0, R = -2
			val = ((((msg.data[1] << 8) | msg.data[0]) * 100) * 1000 / 1530.75);
		}
	} else {
		sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status = SNR_FAIL_TO_ACCESS;
		printf("Snr num %x read fail\n", sensor_num);
		return false;
	}
	*reading = (acur_cal_MBR(sensor_num, val) / 1000) & 0xffff;
	sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache = *reading;
	sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status = SNR_READ_ACUR_SUCCESS;
	return true;
}
