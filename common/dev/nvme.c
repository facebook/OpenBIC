#include "nvme.h"

#include <stdio.h>

#include "sensor.h"
#include "hal_i2c.h"

#define RETRY 5

#define NVME_NOT_AVAILABLE 0x80
#define NVME_TEMP_SENSOR_FAILURE 0x81

bool nvme_read(uint8_t sensor_num, float *reading)
{
	snr_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
	static uint8_t not_available_retry_num = 0, drive_notready_retry = 0;
	int val;
	bool is_drive_ready;

	I2C_MSG msg;
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.data[0] = cfg->offset;
	msg.tx_len = 1;
	msg.rx_len = 4;
	if (!i2c_master_read(&msg, RETRY)) {
		// Check SSD drive ready
		is_drive_ready = ((msg.data[1] & 0x40) == 0 ? true : false);
		if (!is_drive_ready) {
			if (drive_notready_retry >= 3) {
				cfg->cache_status = SENSOR_NOT_ACCESSIBLE;
				return false;
			} else {
				if (cfg->cache_status == SENSOR_READ_SUCCESS) {
					drive_notready_retry += 1;
					return true;
				} else {
					cfg->cache_status = SENSOR_FAIL_TO_ACCESS;
					return false;
				}
			}
		} else {
			if (drive_notready_retry != 0) {
				drive_notready_retry = 0;
			}
		}

		// Check reading value
		val = msg.data[3];
		if (val == NVME_NOT_AVAILABLE) {
			if (not_available_retry_num >= 3) {
				cfg->cache_status = SENSOR_NOT_ACCESSIBLE;
				return false;
			} else {
				not_available_retry_num += 1;
				if (cfg->cache != 0xFF) {
					cfg->cache_status = SENSOR_READ_SUCCESS;
					return true;
				} else {
					cfg->cache_status = SENSOR_FAIL_TO_ACCESS;
					return false;
				}
			}
		} else if (val == NVME_TEMP_SENSOR_FAILURE) {
			cfg->cache_status = SENSOR_UNSPECIFIED_ERROR;
			return false;
		}
		not_available_retry_num = 0;
	} else {
		cfg->cache_status = SENSOR_FAIL_TO_ACCESS;
		return false;
	}

	*reading = (float)val;
	cfg->cache = *reading;
	cfg->cache_status = SENSOR_READ_SUCCESS;
	return true;
}
