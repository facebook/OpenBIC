/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "plat_sensor_table.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "sensor.h"
#include "ast_adc.h"
#include "intel_peci.h"
#include "hal_gpio.h"
#include "plat_class.h"
#include "plat_gpio.h"
#include "plat_hook.h"
#include "plat_i2c.h"
#include "power_status.h"
#include "pmbus.h"
#include "tmp431.h"
#include "libutil.h"

SET_GPIO_VALUE_CFG pre_bat_3v = { A_P3V_BAT_SCALED_EN_R, GPIO_HIGH };
SET_GPIO_VALUE_CFG post_bat_3v = { A_P3V_BAT_SCALED_EN_R, GPIO_LOW };

sensor_poll_time_cfg diff_poll_time_sensor_table[] = {
	// sensor_number, last_access_time
	{ SENSOR_NUM_VOL_BAT3V, 0 },
};

dimm_pmic_mapping_cfg dimm_pmic_map_table[] = {
	// dimm_sensor_num, mapping_pmic_sensor_num
};

sensor_cfg plat_sensor_config[] = {
	/* number,                  type,       port,      address,      offset,
	   access check arg0, arg1, sample_count, cache, cache_status, mux_ADDRess, mux_offset,
	   pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_fn  */

	// temperature
	{ SENSOR_NUM_TEMP_TMP75_IN, sensor_dev_tmp75, I2C_BUS2, TMP75_IN_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	// PECI
	{ SENSOR_NUM_TEMP_CPU, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR, PECI_TEMP_CPU,
	  post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_CPU_MARGIN, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CPU_MARGIN, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, post_cpu_margin_read, NULL,
	  NULL },
	{ SENSOR_NUM_TEMP_CPU_TJMAX, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CPU_TJMAX, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	// adc voltage
	{ SENSOR_NUM_VOL_STBY12V, sensor_dev_ast_adc, ADC_PORT0, NONE, NONE, stby_access, 667, 100,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_STBY3V, sensor_dev_ast_adc, ADC_PORT2, NONE, NONE, stby_access, 2, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_STBY1V05, sensor_dev_ast_adc, ADC_PORT3, NONE, NONE, stby_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_BAT3V, sensor_dev_ast_adc, ADC_PORT4, NONE, NONE, stby_access, 3, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_BAT3V, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_vol_bat3v_read, NULL, post_vol_bat3v_read, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_STBY5V, sensor_dev_ast_adc, ADC_PORT9, NONE, NONE, stby_access, 711, 200,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_DIMM12V, sensor_dev_ast_adc, ADC_PORT11, NONE, NONE, dc_access, 667, 100,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_STBY1V2, sensor_dev_ast_adc, ADC_PORT13, NONE, NONE, stby_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_STBY1V8, sensor_dev_ast_adc, ADC_PORT15, NONE, NONE, stby_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	  
	// ME
	{ SENSOR_NUM_TEMP_PCH, sensor_dev_pch, I2C_BUS3, PCH_ADDR, ME_SENSOR_NUM_TEMP_PCH,
	  me_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

};

sensor_cfg mp5990_sensor_config_table[] = {
	/* number,                  type,       port,      address,      offset,
	   access check arg0, arg1, sample_count, cache, cache_status, mux_address, mux_offset,
	   pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_fn  */
	{ SENSOR_NUM_TEMP_HSC, sensor_dev_mp5990, I2C_BUS2, MPS_MP5990_ADDR,
	  PMBUS_READ_TEMPERATURE_1, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &mp5990_init_args[0] },
	{ SENSOR_NUM_VOL_HSCIN, sensor_dev_mp5990, I2C_BUS2, MPS_MP5990_ADDR, PMBUS_READ_VIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &mp5990_init_args[0] },
	{ SENSOR_NUM_CUR_HSCOUT, sensor_dev_mp5990, I2C_BUS2, MPS_MP5990_ADDR, PMBUS_READ_IOUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &mp5990_init_args[0] },
	{ SENSOR_NUM_PWR_HSCIN, sensor_dev_mp5990, I2C_BUS2, MPS_MP5990_ADDR, PMBUS_READ_PIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &mp5990_init_args[0] },
};

sensor_cfg adm1278_sensor_config_table[] = {
	/* number,                  type,       port,      address,      offset,
	   access check arg0, arg1, sample_count, cache, cache_status, mux_address, mux_offset,
	   pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_fn  */
	{ SENSOR_NUM_TEMP_HSC, sensor_dev_adm1278, I2C_BUS2, ADI_ADM1278_ADDR,
	  PMBUS_READ_TEMPERATURE_1, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &adm1278_init_args[0] },
	{ SENSOR_NUM_VOL_HSCIN, sensor_dev_adm1278, I2C_BUS2, ADI_ADM1278_ADDR, PMBUS_READ_VIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1278_init_args[0] },
	{ SENSOR_NUM_CUR_HSCOUT, sensor_dev_adm1278, I2C_BUS2, ADI_ADM1278_ADDR, PMBUS_READ_IOUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, post_adm1278_current_read, NULL, &adm1278_init_args[0] },
	{ SENSOR_NUM_PWR_HSCIN, sensor_dev_adm1278, I2C_BUS2, ADI_ADM1278_ADDR, PMBUS_READ_PIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, post_adm1278_power_read, NULL, &adm1278_init_args[0] },
};

sensor_cfg ltc4286_sensor_config_table[] = {
	/* number,                  type,       port,      address,      offset,
	   access check arg0, arg1, sample_count, cache, cache_status, mux_address, mux_offset,
	   pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_fn  */
	{ SENSOR_NUM_TEMP_HSC, sensor_dev_ltc4286, I2C_BUS2, ADI_LTC4286_ADDR,
	  PMBUS_READ_TEMPERATURE_1, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ltc4286_init_args[0] },
	{ SENSOR_NUM_VOL_HSCIN, sensor_dev_ltc4286, I2C_BUS2, ADI_LTC4286_ADDR, PMBUS_READ_VIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ltc4286_init_args[0] },
	{ SENSOR_NUM_CUR_HSCOUT, sensor_dev_ltc4286, I2C_BUS2, ADI_LTC4286_ADDR, PMBUS_READ_IOUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, post_ltc4286_read, NULL, &ltc4286_init_args[0] },
	{ SENSOR_NUM_PWR_HSCIN, sensor_dev_ltc4286, I2C_BUS2, ADI_LTC4286_ADDR, PMBUS_READ_PIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, post_ltc4286_read, NULL, &ltc4286_init_args[0] },
};

sensor_cfg ltc4282_sensor_config_table[] = {
	/* number,                  type,       port,      address,      offset,
	   access check arg0, arg1, sample_count, cache, cache_status, mux_address, mux_offset,
	   pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_fn  */
	{ SENSOR_NUM_TEMP_HSC, sensor_dev_ltc4282, I2C_BUS2, ADI_LTC4282_ADDR,
	  PMBUS_READ_TEMPERATURE_1, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, post_ltc4282_read, NULL,
	  &ltc4282_init_args[0] },
	{ SENSOR_NUM_VOL_HSCIN, sensor_dev_ltc4282, I2C_BUS2, ADI_LTC4282_ADDR,
	  LTC4282_VSOURCE_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, post_ltc4282_read, NULL,
	  &ltc4282_init_args[0] },
	{ SENSOR_NUM_CUR_HSCOUT, sensor_dev_ltc4282, I2C_BUS2, ADI_LTC4282_ADDR,
	  LTC4282_VSENSE_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, post_ltc4282_read, NULL,
	  &ltc4282_init_args[0] },
	{ SENSOR_NUM_PWR_HSCIN, sensor_dev_ltc4282, I2C_BUS2, ADI_LTC4282_ADDR,
	  LTC4282_POWER_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, post_ltc4282_read, NULL,
	  &ltc4282_init_args[0] },
};

sensor_cfg evt3_class1_adi_temperature_sensor_table[] = {
	{ SENSOR_NUM_TEMP_TMP75_OUT, sensor_dev_tmp431, I2C_BUS2, TMP431_ADDR,
	  TMP431_LOCAL_TEMPERATRUE, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_HSC, sensor_dev_tmp431, I2C_BUS2, TMP431_ADDR, TMP431_REMOTE_TEMPERATRUE,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
};

sensor_cfg DPV2_sensor_config_table[] = {
	{ SENSOR_NUM_VOL_DPV2_12VIN, sensor_dev_max16550a, I2C_BUS9, DPV2_16_ADDR, PMBUS_READ_VIN,
	  dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &max16550a_init_args[0] },
	{ SENSOR_NUM_VOL_DPV2_12VOUT, sensor_dev_max16550a, I2C_BUS9, DPV2_16_ADDR, PMBUS_READ_VOUT,
	  dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &max16550a_init_args[0] },
	{ SENSOR_NUM_CUR_DPV2OUT, sensor_dev_max16550a, I2C_BUS9, DPV2_16_ADDR, PMBUS_READ_IOUT,
	  dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &max16550a_init_args[0] },
	{ SENSOR_NUM_TEMP_DPV2_EFUSE, sensor_dev_max16550a, I2C_BUS9, DPV2_16_ADDR,
	  PMBUS_READ_TEMPERATURE_1, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &max16550a_init_args[0] },
	{ SENSOR_NUM_PWR_DPV2, sensor_dev_max16550a, I2C_BUS9, DPV2_16_ADDR, PMBUS_READ_PIN,
	  dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &max16550a_init_args[0] },
};

const int SENSOR_CONFIG_SIZE = ARRAY_SIZE(plat_sensor_config);

void load_sensor_config(void)
{
	memcpy(sensor_config, plat_sensor_config, sizeof(plat_sensor_config));
	sensor_config_count = ARRAY_SIZE(plat_sensor_config);

	// Fix config table in different system/config
	pal_extend_sensor_config();
}

void check_vr_type(uint8_t index)
{
	uint8_t retry = 5;
	I2C_MSG msg;

	isl69259_pre_proc_arg *args = sensor_config[index].pre_sensor_read_args;
	memset(&msg, 0, sizeof(msg));
	msg.bus = sensor_config[index].port;
	msg.target_addr = sensor_config[index].target_addr;
	msg.tx_len = 2;
	msg.data[0] = 0x00;
	msg.data[1] = args->vr_page;
	if (i2c_master_write(&msg, retry)) {
		printf("Failed to switch to VR page %d\n", args->vr_page);
		return;
	}

	/* Get IC Device ID from VR chip
	 * - Command code: 0xAD
	 * - The response data 
	 *   byte-1: Block read count
	 *   byte-2: Device ID
	 * For the ISL69259 chip,
	 * the byte-1 of response data is 4 and the byte-2 to 5 is 49D28100h.
	 * For the TPS53689 chip,
	 * the byte-1 of response data is 6 and the byte-2 to 7 is 544953689000h.
	 * For the XDPE15284 chip,
	 * the byte-1 is returned as 2 and the byte-2 is 8Ah(XDPE15284).
	 */
	memset(&msg, 0, sizeof(msg));
	msg.bus = sensor_config[index].port;
	msg.target_addr = sensor_config[index].target_addr;
	msg.tx_len = 1;
	msg.rx_len = 7;
	msg.data[0] = PMBUS_IC_DEVICE_ID;

	if (i2c_master_read(&msg, retry)) {
		printf("Failed to read VR IC_DEVICE_ID: register(0x%x)\n", PMBUS_IC_DEVICE_ID);
		return;
	}

	if ((msg.data[0] == 0x06) && (msg.data[1] == 0x54) && (msg.data[2] == 0x49) &&
	    (msg.data[3] == 0x53) && (msg.data[4] == 0x68) && (msg.data[5] == 0x90) &&
	    (msg.data[6] == 0x00)) {
		sensor_config[index].type = sensor_dev_tps53689;
	} else if ((msg.data[0] == 0x02) && (msg.data[2] == 0x8A)) {
		sensor_config[index].type = sensor_dev_xdpe15284;
	} else if ((msg.data[0] == 0x04) && (msg.data[1] == 0x00) && (msg.data[2] == 0x81) &&
		   (msg.data[3] == 0xD2) && (msg.data[4] == 0x49)) {
	} else {
		printf("Unknown VR type\n");
	}
}

void check_outlet_temp_type(uint8_t index)
{
	if (index >= sensor_config_count) {
		printf("Out of sensor_config_count\n");
		return;
	}

	uint8_t retry = 5;
	I2C_MSG msg;
	uint8_t CID = 0;
	uint8_t VID = 0;

	/* Get Chip ID and Manufacturer ID
	 * - Command code: 0xFD, 0xFE
	 * TMP431: 0x31 0x55
	 * NCT7718W: 0x50 0x50
	 * G788P81U: 0x50 0x47
	 */
	memset(&msg, 0, sizeof(msg));
	msg.bus = sensor_config[index].port;
	msg.target_addr = sensor_config[index].target_addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = NCT7718W_CHIP_ID_OFFSET;

	if (i2c_master_read(&msg, retry)) {
		printf("Failed to read Outlet_Temp chip ID: register(0x%x)\n",
		       NCT7718W_CHIP_ID_OFFSET);
		return;
	}
	CID = msg.data[0];

	memset(&msg, 0, sizeof(msg));
	msg.bus = sensor_config[index].port;
	msg.target_addr = sensor_config[index].target_addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = NCT7718W_VENDOR_ID_OFFSET;

	if (i2c_master_read(&msg, retry)) {
		printf("Failed to read Outlet_Temp vendor ID: register(0x%x)\n",
		       NCT7718W_VENDOR_ID_OFFSET);
		return;
	}
	VID = msg.data[0];

	if ((CID == 0x31) && (VID == 0x55)) {
		sensor_config[index].type = sensor_dev_tmp431;
	} else if ((CID == 0x50) && (VID == 0x50)) {
		sensor_config[index].type = sensor_dev_nct7718w;
	} else if ((CID == 0x50) && (VID == 0x47)) {
		sensor_config[index].type = sensor_dev_g788p81u;
	} else {
		printf("Unknown Outlet_Temp type\n");
	}
}

void pal_extend_sensor_config()
{

	if (sensor_config_count != sdr_count) {
		printf("[%s] extend sensor SDR and config table not match, sdr size: 0x%x, sensor config size: 0x%x\n",
		       __func__, sdr_count, sensor_config_count);
	}
}

bool pal_is_time_to_poll(uint8_t sensor_num, int poll_time)
{
	int i = 0;
	int table_size = sizeof(diff_poll_time_sensor_table) / sizeof(sensor_poll_time_cfg);

	for (i = 0; i < table_size; i++) {
		if (sensor_num == diff_poll_time_sensor_table[i].sensor_num) {
			int64_t current_access_time = k_uptime_get();
			int64_t last_access_time = diff_poll_time_sensor_table[i].last_access_time;
			int64_t diff_time = (current_access_time - last_access_time) / 1000; // sec
			if ((last_access_time != 0) && (diff_time < poll_time)) {
				return false;
			} else {
				diff_poll_time_sensor_table[i].last_access_time =
					current_access_time;
				return true;
			}
		}
	}

	printf("[%s] can't find sensor 0x%x last accest time\n", __func__, sensor_num);
	return true;
}

uint8_t get_hsc_pwr_reading(int *reading)
{
	// add template function for future use
	return 0;
}

bool disable_dimm_pmic_sensor(uint8_t sensor_num)
{
	uint8_t table_size = ARRAY_SIZE(dimm_pmic_map_table);

	for (uint8_t index = 0; index < table_size; ++index) {
		if (sensor_num == dimm_pmic_map_table[index].dimm_sensor_num) {
			control_sensor_polling(dimm_pmic_map_table[index].dimm_sensor_num,
					       DISABLE_SENSOR_POLLING, SENSOR_NOT_PRESENT);
			control_sensor_polling(dimm_pmic_map_table[index].mapping_pmic_sensor_num,
					       DISABLE_SENSOR_POLLING, SENSOR_NOT_PRESENT);
			return true;
		}
	}

	printf("[%s] input sensor 0x%x can't find in dimm pmic mapping table\n", __func__,
	       sensor_num);
	return false;
}
