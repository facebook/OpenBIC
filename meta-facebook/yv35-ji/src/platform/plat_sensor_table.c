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
#include "pmbus.h"
#include "plat_def.h"
#include "tmp461.h"
#include "plat_class.h"
#include "plat_hook.h"
#include "plat_i2c.h"
#include "plat_i3c.h"
#include "libutil.h"

#include <logging/log.h>

LOG_MODULE_REGISTER(plat_sensor_table);

sensor_poll_time_cfg diff_poll_time_sensor_table[] = {
	// sensor_number, last_access_time
	{ SENSOR_NUM_VOL_ADC4_P3V_BAT, 0 },
};

sensor_cfg plat_sensor_config[] = {
	/* number,                  type,       port,      address,      offset,
	   access check arg0, arg1, sample_count, cache, cache_status, mux_ADDRess, mux_offset,
	   pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_fn  */

	/* NVME */
	{ SENSOR_NUM_TEMP_E1S_SSD, sensor_dev_nvme, I2C_BUS10, E1S_SSD_ADDR, SSD_TEMP_OFFSET,
	  post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	/* adc voltage */
	{ SENSOR_NUM_VOL_ADC0_P12V_STBY, sensor_dev_ast_adc, ADC_PORT0, NONE, NONE, stby_access, 66,
	  10, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC1_VDD_1V8, sensor_dev_ast_adc, ADC_PORT1, NONE, NONE, stby_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC2_P3V3_STBY, sensor_dev_ast_adc, ADC_PORT2, NONE, NONE, stby_access, 2,
	  1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC3_SOCVDD, sensor_dev_ast_adc, ADC_PORT3, NONE, NONE, stby_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC4_P3V_BAT, sensor_dev_ast_adc, ADC_PORT4, NONE, NONE, stby_access, 301,
	  100, SAMPLE_COUNT_DEFAULT, POLL_TIME_BAT3V, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_vol_bat3v_read, NULL, post_vol_bat3v_read, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC5_CPUVDD, sensor_dev_ast_adc, ADC_PORT5, NONE, NONE, stby_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC6_FPGA_VCC_AO, sensor_dev_ast_adc, ADC_PORT6, NONE, NONE, stby_access,
	  1, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC7_1V2, sensor_dev_ast_adc, ADC_PORT7, NONE, NONE, stby_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC9_VDD_M2, sensor_dev_ast_adc, ADC_PORT9, NONE, NONE, stby_access, 2, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC10_P1V2_STBY, sensor_dev_ast_adc, ADC_PORT10, NONE, NONE, stby_access,
	  1, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC11_FBVDDQ, sensor_dev_ast_adc, ADC_PORT11, NONE, NONE, stby_access, 1,
	  1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC12_FBVDDP2, sensor_dev_ast_adc, ADC_PORT12, NONE, NONE, stby_access, 1,
	  1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC13_FBVDD1, sensor_dev_ast_adc, ADC_PORT13, NONE, NONE, stby_access, 1,
	  1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC14_P5V_STBY, sensor_dev_ast_adc, ADC_PORT14, NONE, NONE, stby_access,
	  711, 200, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC15_CPU_DVDD, sensor_dev_ast_adc, ADC_PORT15, NONE, NONE, stby_access, 1,
	  1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },

	// INA230
	{ SENSOR_NUM_PWR_E1S, sensor_dev_ina230, I2C_BUS2, INA230_ADDR, INA230_PWR_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ina230_init_args[0] },
	{ SENSOR_NUM_CUR_E1S, sensor_dev_ina230, I2C_BUS2, INA230_ADDR, INA230_CUR_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ina230_init_args[0] },
	{ SENSOR_NUM_VOL_E1S, sensor_dev_ina230, I2C_BUS2, INA230_ADDR, INA230_BUS_VOL_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ina230_init_args[0] },

	// TMP
	{ SENSOR_NUM_TEMP_TMP451_IN, sensor_dev_tmp461, I2C_BUS12, TMP451_ADDR,
	  TMP461_REMOTE_TEMPERATRUE, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_tmp451_read, &mux_conf_addr_0xe0[0],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_TMP451_OUT, sensor_dev_tmp461, I2C_BUS12, TMP451_ADDR,
	  TMP461_REMOTE_TEMPERATRUE, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_tmp451_read, &mux_conf_addr_0xe0[1],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_FPGA, sensor_dev_tmp461, I2C_BUS12, TMP451_ADDR,
	  TMP461_REMOTE_TEMPERATRUE, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_tmp451_read, &mux_conf_addr_0xe0[2],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_TMP75_FIO, sensor_dev_tmp75, I2C_BUS2, TMP75_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_tmp75_read, &mux_conf_addr_0xe2[0], NULL, NULL, NULL },

#ifdef ENABLE_NVIDIA
	/* SatMC */
	{ SENSOR_NUM_TEMP_CPU, sensor_dev_nv_satmc, MCTP_I2C_SATMC_BUS, (MCTP_I2C_SATMC_ADDR >> 1),
	  NONE, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING,
	  0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &satmc_init_args[0] },
	{ SENSOR_NUM_VOL_CPUVDD, sensor_dev_nv_satmc, MCTP_I2C_SATMC_BUS,
	  (MCTP_I2C_SATMC_ADDR >> 1), NONE, post_access, 0, 0, SAMPLE_COUNT_DEFAULT,
	  POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &satmc_init_args[2] },
	{ SENSOR_NUM_VOL_SOCVDD, sensor_dev_nv_satmc, MCTP_I2C_SATMC_BUS,
	  (MCTP_I2C_SATMC_ADDR >> 1), NONE, post_access, 0, 0, SAMPLE_COUNT_DEFAULT,
	  POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &satmc_init_args[4] },
	{ SENSOR_NUM_PWR_CPUVDD, sensor_dev_nv_satmc, MCTP_I2C_SATMC_BUS,
	  (MCTP_I2C_SATMC_ADDR >> 1), NONE, post_access, 0, 0, SAMPLE_COUNT_DEFAULT,
	  POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &satmc_init_args[1] },
	{ SENSOR_NUM_PWR_SOCVDD, sensor_dev_nv_satmc, MCTP_I2C_SATMC_BUS,
	  (MCTP_I2C_SATMC_ADDR >> 1), NONE, post_access, 0, 0, SAMPLE_COUNT_DEFAULT,
	  POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &satmc_init_args[3] },
	{ SENSOR_NUM_PWR_CPU, sensor_dev_nv_satmc, MCTP_I2C_SATMC_BUS, (MCTP_I2C_SATMC_ADDR >> 1),
	  NONE, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING,
	  0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &satmc_init_args[5] },
	{ SENSOR_NUM_OTH_CPU_THROTTLE, sensor_dev_nv_satmc, MCTP_I2C_SATMC_BUS,
	  (MCTP_I2C_SATMC_ADDR >> 1), NONE, post_access, 0, 0, SAMPLE_COUNT_DEFAULT,
	  POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &satmc_init_args[6] },
	{ SENSOR_NUM_OTH_POWER_BREAK, sensor_dev_nv_satmc, MCTP_I2C_SATMC_BUS,
	  (MCTP_I2C_SATMC_ADDR >> 1), NONE, post_access, 0, 0, SAMPLE_COUNT_DEFAULT,
	  POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &satmc_init_args[7] },
	{ SENSOR_NUM_OTH_SPARE_CHANNEL, sensor_dev_nv_satmc, MCTP_I2C_SATMC_BUS,
	  (MCTP_I2C_SATMC_ADDR >> 1), NONE, post_access, 0, 0, SAMPLE_COUNT_DEFAULT,
	  POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &satmc_init_args[8] },
#endif
};

sensor_cfg mp5990_sensor_config_table[] = {
	{ SENSOR_NUM_VOL_HSCIN, sensor_dev_mp5990, I2C_BUS2, MP5990_ADDR, PMBUS_READ_VIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &mp5990_init_args[0] },
	{ SENSOR_NUM_CUR_HSCOUT, sensor_dev_mp5990, I2C_BUS2, MP5990_ADDR, PMBUS_READ_IOUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, post_mp5990_cur_read, NULL, &mp5990_init_args[0] },
	{ SENSOR_NUM_PWR_HSCIN, sensor_dev_mp5990, I2C_BUS2, MP5990_ADDR, PMBUS_READ_PIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, post_mp5990_pwr_read, NULL, &mp5990_init_args[0] },
};

sensor_cfg mp5990_temp_sensor_config_table[] = {
	{ SENSOR_NUM_TEMP_HSC, sensor_dev_mp5990, I2C_BUS2, MP5990_ADDR, PMBUS_READ_TEMPERATURE_1,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &mp5990_init_args[0] },
};

void pal_extend_sensor_config()
{
	uint8_t sensor_count = 0;
	uint8_t hsc_module = get_hsc_module();

	/* Determine which HSC module is used */
	switch (hsc_module) {
	case HSC_MODULE_MP5990:
		LOG_INF("HSC vendor: MP5990");
		sensor_count = ARRAY_SIZE(mp5990_sensor_config_table);
		for (int index = 0; index < sensor_count; index++) {
			add_sensor_config(mp5990_sensor_config_table[index]);
		}
		/* MP5990 can read HSC temperature */
		add_sensor_config(mp5990_temp_sensor_config_table[0]);
		break;
	case HSC_MODULE_RS31380R:
		LOG_INF("HSC vendor: RS31380R");
		LOG_WRN("RS31380R HSC module is not supported yet");
		break;
	default:
		LOG_ERR("Unsupported HSC module, HSC module: 0x%x", hsc_module);
		break;
	}

	if (sensor_config_count != sdr_count) {
		LOG_ERR("Extend sensor SDR and config table not match, sdr size: 0x%x, sensor config size: 0x%x",
			sdr_count, sensor_config_count);
	}
}

uint8_t pal_get_extend_sensor_config()
{
	uint8_t extend_sensor_config_size = 0;
	uint8_t hsc_module = get_hsc_module();
	switch (hsc_module) {
	case HSC_MODULE_MP5990:
		LOG_INF("HSC vendor: MP5990");
		extend_sensor_config_size += ARRAY_SIZE(mp5990_sensor_config_table);
		/* MP5990 can read HSC temperature */
		extend_sensor_config_size += ARRAY_SIZE(mp5990_temp_sensor_config_table);
		break;
	case HSC_MODULE_RS31380R:
		LOG_INF("HSC vendor: RS31380R");
		LOG_WRN("RS31380R HSC module is not supported yet");
		break;
	default:
		LOG_ERR("Unsupported HSC module, HSC module: 0x%x", hsc_module);
		break;
	}

	return extend_sensor_config_size;
}

bool pal_is_time_to_poll(uint8_t sensor_num, int poll_time)
{
	int i = 0;
	int table_size = sizeof(diff_poll_time_sensor_table) / sizeof(sensor_poll_time_cfg);

	for (i = 0; i < table_size; i++) {
		if (sensor_num == diff_poll_time_sensor_table[i].sensor_num) {
			int64_t current_access_time = k_uptime_get();
			int64_t last_access_time = diff_poll_time_sensor_table[i].last_access_time;
			int64_t diff_time = (current_access_time - last_access_time) / 1000;
			if ((last_access_time != 0) && (diff_time < poll_time)) {
				return false;
			} else {
				diff_poll_time_sensor_table[i].last_access_time =
					current_access_time;
				return true;
			}
		}
	}

	LOG_ERR("Can't find sensor 0x%x last access time", sensor_num);
	return true;
}

const int SENSOR_CONFIG_SIZE = ARRAY_SIZE(plat_sensor_config);

void load_sensor_config(void)
{
	memcpy(sensor_config, plat_sensor_config, sizeof(plat_sensor_config));
	sensor_config_count = ARRAY_SIZE(plat_sensor_config);

	pal_extend_sensor_config();
}
