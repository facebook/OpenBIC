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
#include "pt5161l.h"
#include "ds160pt801.h"
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
	{ SENSOR_NUM_VOL_ADC1_VDD_1V8, sensor_dev_ast_adc, ADC_PORT1, NONE, NONE, dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC2_P3V3_STBY, sensor_dev_ast_adc, ADC_PORT2, NONE, NONE, stby_access, 2,
	  1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC3_SOCVDD, sensor_dev_ast_adc, ADC_PORT3, NONE, NONE, dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC4_P3V_BAT, sensor_dev_ast_adc, ADC_PORT4, NONE, NONE, stby_access, 301,
	  100, SAMPLE_COUNT_DEFAULT, POLL_TIME_BAT3V, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_vol_bat3v_read, NULL, post_vol_bat3v_read, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC5_CPUVDD, sensor_dev_ast_adc, ADC_PORT5, NONE, NONE, dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC7_1V2, sensor_dev_ast_adc, ADC_PORT7, NONE, NONE, dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC10_P1V2_STBY, sensor_dev_ast_adc, ADC_PORT10, NONE, NONE, stby_access,
	  1, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC11_FBVDDQ, sensor_dev_ast_adc, ADC_PORT11, NONE, NONE, dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC12_FBVDDP2, sensor_dev_ast_adc, ADC_PORT12, NONE, NONE, dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC13_FBVDD1, sensor_dev_ast_adc, ADC_PORT13, NONE, NONE, dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC14_P5V_STBY, sensor_dev_ast_adc, ADC_PORT14, NONE, NONE, stby_access,
	  711, 200, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC15_CPU_DVDD, sensor_dev_ast_adc, ADC_PORT15, NONE, NONE, dc_access, 1,
	  1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },

	// INA230
	{ SENSOR_NUM_PWR_E1S, sensor_dev_ina230, I2C_BUS2, INA230_ADDR, INA230_PWR_OFFSET,
	  dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ina230_init_args[0] },
	{ SENSOR_NUM_CUR_E1S, sensor_dev_ina230, I2C_BUS2, INA230_ADDR, INA230_CUR_OFFSET,
	  dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ina230_init_args[0] },
	{ SENSOR_NUM_VOL_E1S, sensor_dev_ina230, I2C_BUS2, INA230_ADDR, INA230_BUS_VOL_OFFSET,
	  dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ina230_init_args[0] },

	// TMP
	{ SENSOR_NUM_TEMP_TMP451_IN, sensor_dev_tmp461, I2C_BUS12, TMP451_ADDR,
	  TMP461_REMOTE_TEMPERATRUE, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_tmp451_read, &mux_conf_addr_0xe0[1],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_TMP451_OUT, sensor_dev_tmp461, I2C_BUS12, TMP451_ADDR,
	  TMP461_REMOTE_TEMPERATRUE, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_tmp451_read, &mux_conf_addr_0xe0[0],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_LPDDR5_UP, sensor_dev_tmp461, I2C_BUS12, TMP451_ADDR,
	  TMP461_LOCAL_TEMPERATRUE, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_tmp451_read, &mux_conf_addr_0xe0[2],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_LPDDR5_DOWN, sensor_dev_tmp461, I2C_BUS12, TMP451_ADDR,
	  TMP461_LOCAL_TEMPERATRUE, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_tmp451_read, &mux_conf_addr_0xe0[3],
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

sensor_cfg pt4080l_sensor_config_table[] = {
	{ SENSOR_NUM_TEMP_RETIMER, sensor_dev_pt5161l, I2C_BUS2, AL_RETIMER_ADDR,
	  PT5161L_TEMP_OFFSET, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_pt4080l_read, &mux_conf_addr_0xe2[1],
	  NULL, NULL, &pt5161l_init_args[0] },
};

sensor_cfg ds160pt801_sensor_config_table[] = {
	{ SENSOR_NUM_TEMP_RETIMER, sensor_dev_ds160pt801, I2C_BUS2, TI_RETIMER_ADDR,
	  DS160PT801_READ_TEMP, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ds160pt801_read, &mux_conf_addr_0xe2[1],
	  NULL, NULL, NULL },
};

static sensor_cfg *change_retimer_sensor_cfg(uint8_t module)
{
	int i = 0;
	for (i = 0; i < sensor_config_count; i++) {
		if (sensor_config[i].num == SENSOR_NUM_TEMP_RETIMER) {
			if (module == RETIMER_MODULE_PT4080L)
				sensor_config[i] = pt4080l_sensor_config_table[0];
			else if (module == RETIMER_MODULE_DS160PT801)
				sensor_config[i] = ds160pt801_sensor_config_table[0];
			else {
				LOG_ERR("Unsupported Retimer module, Retimer module: 0x%x", module);
				return NULL;
			}
			break;
		}
	}

	if (i == sensor_config_count)
		return NULL;

	return &sensor_config[i];
}

bool modify_sensor_cfg()
{
	/* Need to switch channel after EVT2 */
	if (get_board_revision() >= SYS_BOARD_EVT2) {
		I2C_MSG msg = { 0 };
		msg.bus = I2C_BUS2;
		msg.target_addr = (0xE2 >> 1); //switch address
		msg.data[0] = 0x02; //channel2
		msg.tx_len = 1;
		msg.rx_len = 0;

		if (i2c_master_write(&msg, 3)) {
			LOG_ERR("Failed to switch channel for retimer");
			return false;
		}
	}

	uint8_t addr_list[10] = { 0 };
	uint8_t addr_len = 0;
	uint8_t retimer_module = RETIMER_MODULE_UNKNOWN;

	i2c_scan(I2C_BUS2, addr_list, &addr_len);

	int i = 0;
	for (i = 0; i < addr_len; i++) {
		if (addr_list[i] == (AL_RETIMER_ADDR << 1)) {
			LOG_WRN("Found AL retimer device at 0x40");
			retimer_module = RETIMER_MODULE_PT4080L;
			break;
		} else if (addr_list[i] == (TI_RETIMER_ADDR << 1)) {
			LOG_WRN("Found TI retimer device at 0x20");
			retimer_module = RETIMER_MODULE_DS160PT801;
			break;
		}
	}

	if (i == ARRAY_SIZE(addr_list)) {
		LOG_WRN("No retimer device found!!");
		return false;
	}

	set_retimer_module(retimer_module);

	sensor_cfg *retimer_cfg = NULL;
	retimer_cfg = change_retimer_sensor_cfg(retimer_module);
	if (!retimer_cfg) {
		LOG_WRN("Retimer sensor config not found!!");
		return false;
	}

	if (init_drive_type_delayed(retimer_cfg) == false) {
		LOG_ERR("Retimer initial fail");
		return false;
	}

	return true;
}

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
			if (get_board_revision() >= SYS_BOARD_EVT2)
				mp5990_sensor_config_table[index].target_addr = MP5990_ADDR_1;
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

	uint8_t board_revision = get_board_revision();
	if (board_revision != SYS_BOARD_POC) {
		uint8_t retimer_module = get_retimer_module();

		/* Determine which Retimer module is used */
		switch (retimer_module) {
		case RETIMER_MODULE_PT4080L:
			LOG_INF("Retimer vendor: PT4080L");
			add_sensor_config(pt4080l_sensor_config_table[0]);
			break;
		case RETIMER_MODULE_DS160PT801:
			LOG_INF("Retimer vendor: DS160PT801");
			add_sensor_config(ds160pt801_sensor_config_table[0]);
			break;
		default:
			LOG_ERR("Unsupported Retimer module, Retimer module: 0x%x", retimer_module);
			break;
		}
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

	uint8_t board_revision = get_board_revision();
	if (board_revision != SYS_BOARD_POC) {
		uint8_t retimer_module = get_retimer_module();
		switch (retimer_module) {
		case RETIMER_MODULE_PT4080L:
			LOG_INF("Retimer vendor: PT4080L");
			extend_sensor_config_size += ARRAY_SIZE(pt4080l_sensor_config_table);
			break;
		case RETIMER_MODULE_DS160PT801:
			LOG_INF("Retimer vendor: DS160PT801");
			extend_sensor_config_size += ARRAY_SIZE(ds160pt801_sensor_config_table);
			break;
		default:
			LOG_ERR("Unsupported Retimer module, Retimer module: 0x%x", retimer_module);
			break;
		}
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
