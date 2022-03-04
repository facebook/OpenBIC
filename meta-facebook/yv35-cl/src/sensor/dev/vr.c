#include <stdio.h>
#include <string.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "sensor_def.h"
#include "pal.h"

bool pal_vr_read(uint8_t sensor_num, int *reading)
{
	uint8_t retry = 5;
	int val;
	I2C_MSG msg;

	if ((sensor_num == SENSOR_NUM_PWR_PVCCD_HV) || (sensor_num == SENSOR_NUM_TEMP_PVCCD_HV) ||
	    (sensor_num == SENSOR_NUM_CUR_PVCCD_HV) || (sensor_num == SENSOR_NUM_VOL_PVCCD_HV) ||
	    (sensor_num == SENSOR_NUM_PWR_PVCCINFAON) ||
	    (sensor_num == SENSOR_NUM_TEMP_PVCCINFAON) ||
	    (sensor_num == SENSOR_NUM_CUR_PVCCINFAON) ||
	    (sensor_num == SENSOR_NUM_VOL_PVCCINFAON) || (sensor_num == SENSOR_NUM_PWR_PVCCIN) ||
	    (sensor_num == SENSOR_NUM_TEMP_PVCCIN) || (sensor_num == SENSOR_NUM_CUR_PVCCIN) ||
	    (sensor_num == SENSOR_NUM_VOL_PVCCIN)) {
		msg.data[0] = 0x00;
		msg.data[1] = 0x00; //switch to page 0
	} else if ((sensor_num == SENSOR_NUM_PWR_PVCCFA_EHV) ||
		   (sensor_num == SENSOR_NUM_TEMP_PVCCFA_EHV) ||
		   (sensor_num == SENSOR_NUM_CUR_PVCCFA_EHV) ||
		   (sensor_num == SENSOR_NUM_VOL_PVCCFA_EHV) ||
		   (sensor_num == SENSOR_NUM_PWR_PVCCFA_EHV_FIVRA) ||
		   (sensor_num == SENSOR_NUM_TEMP_PVCCFA_EHV_FIVRA) ||
		   (sensor_num == SENSOR_NUM_CUR_PVCCFA_EHV_FIVRA) ||
		   (sensor_num == SENSOR_NUM_VOL_PVCCFA_EHV_FIVRA)) {
		msg.data[0] = 0x00;
		msg.data[1] = 0x01; //switch to page 1
	}

	msg.bus = sensor_config[SensorNum_SensorCfg_map[sensor_num]].port;
	msg.slave_addr = sensor_config[SensorNum_SensorCfg_map[sensor_num]].slave_addr;
	msg.tx_len = 2;

	if (!i2c_master_write(&msg, retry)) {
		memset(&msg.data[0], 0, 2);
		msg.tx_len = 1;
		msg.rx_len = 2;
		msg.data[0] = sensor_config[SensorNum_SensorCfg_map[sensor_num]].offset;

		if (!i2c_master_read(&msg, retry)) {
			// voltage
			if ((sensor_num == SENSOR_NUM_VOL_PVCCD_HV) ||
			    (sensor_num == SENSOR_NUM_VOL_PVCCINFAON) ||
			    (sensor_num == SENSOR_NUM_VOL_PVCCFA_EHV) ||
			    (sensor_num == SENSOR_NUM_VOL_PVCCIN) ||
			    (sensor_num == SENSOR_NUM_VOL_PVCCFA_EHV_FIVRA)) {
				val = ((msg.data[1] << 8) | msg.data[0]);
				*reading = (acur_cal_MBR(sensor_num, val) / 1000) & 0xffff;

				// current
			} else if ((sensor_num == SENSOR_NUM_CUR_PVCCD_HV) ||
				   (sensor_num == SENSOR_NUM_CUR_PVCCINFAON) ||
				   (sensor_num == SENSOR_NUM_CUR_PVCCFA_EHV) ||
				   (sensor_num == SENSOR_NUM_CUR_PVCCIN) ||
				   (sensor_num == SENSOR_NUM_CUR_PVCCFA_EHV_FIVRA)) {
				val = ((msg.data[1] << 8) | msg.data[0]);
				*reading = (acur_cal_MBR(sensor_num, val) / 10) & 0xffff;

				// temperature
			} else if ((sensor_num == SENSOR_NUM_TEMP_PVCCD_HV) ||
				   (sensor_num == SENSOR_NUM_TEMP_PVCCINFAON) ||
				   (sensor_num == SENSOR_NUM_TEMP_PVCCFA_EHV) ||
				   (sensor_num == SENSOR_NUM_TEMP_PVCCIN) ||
				   (sensor_num == SENSOR_NUM_TEMP_PVCCFA_EHV_FIVRA)) {
				val = (((msg.data[1] << 8) | msg.data[0]));
				*reading = (acur_cal_MBR(sensor_num, val)) & 0xffff;

				// power
			} else if ((sensor_num == SENSOR_NUM_PWR_PVCCD_HV) ||
				   (sensor_num == SENSOR_NUM_PWR_PVCCINFAON) ||
				   (sensor_num == SENSOR_NUM_PWR_PVCCFA_EHV) ||
				   (sensor_num == SENSOR_NUM_PWR_PVCCIN) ||
				   (sensor_num == SENSOR_NUM_PWR_PVCCFA_EHV_FIVRA)) {
				val = (((msg.data[1] << 8) | msg.data[0]));
				*reading = (acur_cal_MBR(sensor_num, val)) & 0xffff;
			}
		} else {
			sensor_config[SensorNum_SensorCfg_map[sensor_num]].cache_status =
				SENSOR_FAIL_TO_ACCESS;
			printf("Sensor num %x read fail\n", sensor_num);
			return false;
		}

	} else {
		sensor_config[SensorNum_SensorCfg_map[sensor_num]].cache_status =
			SENSOR_FAIL_TO_ACCESS;
		printf("i2c write fail\n");
		return false;
	}

	sensor_config[SensorNum_SensorCfg_map[sensor_num]].cache = *reading;
	sensor_config[SensorNum_SensorCfg_map[sensor_num]].cache_status = SENSOR_READ_ACUR_SUCCESS;

	return true;
}
