#include "isl69260.h"

#include <stdio.h>

#include "sensor.h"
#include "hal_i2c.h"
#include "plat_sensor_table.h"

#define RETRY 5

struct isl69260_page_data isl69260_page_data_args[] = { [0] = { 0x0, 0x0 }, [1] = { 0x0, 0x1 } };

int isl69260_switch_page(uint8_t sensor_num, void *args)
{
	snr_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
	struct isl69260_page_data *page = args;
	I2C_MSG msg;
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.data[0] = page->data0;
	msg.data[1] = page->data1;
	msg.tx_len = 2;

	if (i2c_master_write(&msg, RETRY)) {
		printf("sensor num %x switch page fail\n", sensor_num);
		return -1;
	}

	return 0;
}

bool isl69260_read(uint8_t sensor_num, float *reading)
{
	snr_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
	int val;
	I2C_MSG msg;

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.data[0] = cfg->offset;
	msg.tx_len = 1;
	msg.rx_len = 2;

	if (!i2c_master_read(&msg, RETRY)) {
		switch (cfg->offset) {
		case VR_VOL_CMD:
			val = ((msg.data[1] << 8) | msg.data[0]);
			*reading = (float)val / 1000;
			break;
		case VR_CUR_CMD:
			val = ((msg.data[1] << 8) | msg.data[0]);
			*reading = (float)val / 10;
			break;
		case VR_TEMP_CMD:
			val = (((msg.data[1] << 8) | msg.data[0]));
			*reading = (float)val;
			break;
		case VR_PWR_CMD:
			val = (((msg.data[1] << 8) | msg.data[0]));
			*reading = (float)val;
			break;
		default:
			printf("Unknown isl69260 type\n");
			return false;
		}
	} else {
		cfg->cache_status = SENSOR_FAIL_TO_ACCESS;
		printf("sensor num %x read fail\n", sensor_num);
		return false;
	}

	cfg->cache = *reading;
	cfg->cache_status = SENSOR_READ_SUCCESS;

	return true;
}
