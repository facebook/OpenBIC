#include <stdio.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "pal.h"

#define SSD0_mux_addr (0xE2 >> 1)
#define SSD0_channel 2
#define NVMe_NOT_AVAILABLE 0x80
#define NVMe_TMPSNR_FAILURE 0x81

bool pal_nvme_read(uint8_t sensor_num, int *reading)
{
	uint8_t retry = 5;
	static uint8_t NOT_AVAILABLE_retry_num = 0, drive_notready_retry = 0;
	int val;
	bool is_drive_ready;
	I2C_MSG msg;
	msg.bus = sensor_config[SnrNum_SnrCfg_map[sensor_num]].port;
	// Mux
	msg.slave_addr = SSD0_mux_addr;
	msg.data[0] = SSD0_channel;
	msg.tx_len = 1;
	msg.rx_len = 0;

	if (!i2c_master_write(&msg, retry)) {
		// SSD0
		msg.slave_addr = sensor_config[SnrNum_SnrCfg_map[sensor_num]].slave_addr;
		msg.data[0] = sensor_config[SnrNum_SnrCfg_map[sensor_num]].offset;
		msg.tx_len = 1;
		msg.rx_len = 4;
		if (!i2c_master_read(&msg, retry)) {
			is_drive_ready = ((msg.data[1] & 0x40) == 0 ? true : false);
			val = msg.data[3];

			// Check SSD drive ready
			if (!is_drive_ready) {
				if (drive_notready_retry >= 3) {
					sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status =
						SNR_NOT_ACCESSIBLE;
					return false;
				} else {
					if (sensor_config[SnrNum_SnrCfg_map[sensor_num]]
						    .cache_status == SNR_READ_SUCCESS) {
						drive_notready_retry += 1;
						return true;
					} else {
						sensor_config[SnrNum_SnrCfg_map[sensor_num]]
							.cache_status = SNR_FAIL_TO_ACCESS;
						return false;
					}
				}
			} else {
				if (drive_notready_retry != 0) {
					drive_notready_retry = 0;
				}
			}
			// Check reading value
			if (val == NVMe_NOT_AVAILABLE) {
				if (NOT_AVAILABLE_retry_num >= 3) {
					sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status =
						SNR_NOT_ACCESSIBLE;
					return false;
				} else {
					NOT_AVAILABLE_retry_num += 1;
					if (sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache !=
					    0xFF) {
						sensor_config[SnrNum_SnrCfg_map[sensor_num]]
							.cache_status = SNR_READ_SUCCESS;
						return true;
					} else {
						sensor_config[SnrNum_SnrCfg_map[sensor_num]]
							.cache_status = SNR_FAIL_TO_ACCESS;
						return false;
					}
				}
			} else if (val == NVMe_TMPSNR_FAILURE) {
				sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status =
					SNR_UNSPECIFIED_ERROR;
				return false;
			}
			NOT_AVAILABLE_retry_num = 0;
		} else {
			sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status =
				SNR_FAIL_TO_ACCESS;
			return false;
		}
	} else {
		sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status = SNR_FAIL_TO_ACCESS;
		return false;
	}

	*reading = cal_MBR(sensor_num, val) & 0xff;
	sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache = *reading;
	sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status = SNR_READ_SUCCESS;
	return true;
}
