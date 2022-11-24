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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <logging/log.h>
#include "pmbus.h"
#include "ast_adc.h"
#include "sensor.h"
#include "hal_gpio.h"
#include "plat_i2c.h"
#include "pex89000.h"
#include "plat_gpio.h"
#include "plat_hook.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_sensor_table);

struct k_mutex i2c_2_tca9543_mutex;
struct k_mutex i2c_3_tca9543_mutex;
struct k_mutex i2c_4_pi4msd5v9542_mutex;

sensor_cfg plat_sensor_config[] = {
	/* number,                  type,       port,      address,      offset,
	   access check arg0, arg1, sample_count, cache, cache_status, mux_ADDRess, mux_offset,
	   pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_fn  */

	/** Temperature **/
	{ SENSOR_NUM_TEMP_TMP75_IN, sensor_dev_tmp75, I2C_BUS2, TMP75_IN_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_TMP75_OUT, sensor_dev_tmp75, I2C_BUS2, TMP75_OUT_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	/** ADC **/
	{ SENSOR_NUM_VOL_P3V3_AUX, sensor_dev_ast_adc, ADC_PORT3, NONE, NONE, stby_access, 2, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_P1V8_PEX, sensor_dev_ast_adc, ADC_PORT4, NONE, NONE, is_dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_P1V2_AUX, sensor_dev_ast_adc, ADC_PORT5, NONE, NONE, stby_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_P5V_AUX, sensor_dev_ast_adc, ADC_PORT7, NONE, NONE, stby_access, 711, 200,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_P1V8_VDD_1, sensor_dev_ast_adc, ADC_PORT10, NONE, NONE, is_dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_P1V8_VDD_2, sensor_dev_ast_adc, ADC_PORT11, NONE, NONE, is_dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_P1V25_VDD_1, sensor_dev_ast_adc, ADC_PORT12, NONE, NONE, is_dc_access, 1,
	  1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_P1V25_VDD_2, sensor_dev_ast_adc, ADC_PORT13, NONE, NONE, is_dc_access, 1,
	  1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },

	/** HSC 1 **/
	{ SENSOR_NUM_TEMP_HSC_1, sensor_dev_adm1272, I2C_BUS1, ADM1272_1_ADDR,
	  PMBUS_READ_TEMPERATURE_1, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &adm1272_init_args[0] },
	{ SENSOR_NUM_VOL_P51V_STBY_L, sensor_dev_adm1272, I2C_BUS1, ADM1272_1_ADDR, PMBUS_READ_VIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1272_init_args[0] },
	{ SENSOR_NUM_VOL_P51V_AUX_L, sensor_dev_adm1272, I2C_BUS1, ADM1272_1_ADDR, PMBUS_READ_VOUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1272_init_args[0] },
	{ SENSOR_NUM_CUR_P51V_STBY_L, sensor_dev_adm1272, I2C_BUS1, ADM1272_1_ADDR, PMBUS_READ_IIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1272_init_args[0] },
	{ SENSOR_NUM_CUR_P51V_AUX_L, sensor_dev_adm1272, I2C_BUS1, ADM1272_1_ADDR, PMBUS_READ_IOUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1272_init_args[0] },
	{ SENSOR_NUM_PWR_P51V_STBY_L, sensor_dev_adm1272, I2C_BUS1, ADM1272_1_ADDR, PMBUS_READ_PIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1272_init_args[0] },
	{ SENSOR_NUM_PWR_P51V_AUX_L, sensor_dev_adm1272, I2C_BUS1, ADM1272_1_ADDR, PMBUS_READ_POUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1272_init_args[0] },

	/** HSC 2 **/
	{ SENSOR_NUM_TEMP_HSC_2, sensor_dev_adm1272, I2C_BUS1, ADM1272_2_ADDR,
	  PMBUS_READ_TEMPERATURE_1, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &adm1272_init_args[1] },
	{ SENSOR_NUM_VOL_P51V_STBY_R, sensor_dev_adm1272, I2C_BUS1, ADM1272_2_ADDR, PMBUS_READ_VIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1272_init_args[1] },
	{ SENSOR_NUM_VOL_P51V_AUX_R, sensor_dev_adm1272, I2C_BUS1, ADM1272_2_ADDR, PMBUS_READ_VOUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1272_init_args[1] },
	{ SENSOR_NUM_CUR_P51V_STBY_R, sensor_dev_adm1272, I2C_BUS1, ADM1272_2_ADDR, PMBUS_READ_IIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1272_init_args[1] },
	{ SENSOR_NUM_CUR_P51V_AUX_R, sensor_dev_adm1272, I2C_BUS1, ADM1272_2_ADDR, PMBUS_READ_IOUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1272_init_args[1] },
	{ SENSOR_NUM_PWR_P51V_STBY_R, sensor_dev_adm1272, I2C_BUS1, ADM1272_2_ADDR, PMBUS_READ_PIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1272_init_args[1] },
	{ SENSOR_NUM_PWR_P51V_AUX_R, sensor_dev_adm1272, I2C_BUS1, ADM1272_2_ADDR, PMBUS_READ_POUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1272_init_args[1] },

	/** Power module **/
	{ SENSOR_NUM_TEMP_P12V_AUX_1, sensor_dev_q50sn120a1, I2C_BUS1, Q50SN120A1_1_ADDR,
	  PMBUS_READ_TEMPERATURE_1, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_P12V_AUX_1, sensor_dev_q50sn120a1, I2C_BUS1, Q50SN120A1_1_ADDR,
	  PMBUS_READ_VOUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_P12V_AUX_1, sensor_dev_q50sn120a1, I2C_BUS1, Q50SN120A1_1_ADDR,
	  PMBUS_READ_IOUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_P12V_AUX_1, sensor_dev_q50sn120a1, I2C_BUS1, Q50SN120A1_1_ADDR,
	  PMBUS_READ_POUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	{ SENSOR_NUM_TEMP_P12V_AUX_2, sensor_dev_q50sn120a1, I2C_BUS1, Q50SN120A1_2_ADDR,
	  PMBUS_READ_TEMPERATURE_1, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_P12V_AUX_2, sensor_dev_q50sn120a1, I2C_BUS1, Q50SN120A1_2_ADDR,
	  PMBUS_READ_VOUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_P12V_AUX_2, sensor_dev_q50sn120a1, I2C_BUS1, Q50SN120A1_2_ADDR,
	  PMBUS_READ_IOUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_P12V_AUX_2, sensor_dev_q50sn120a1, I2C_BUS1, Q50SN120A1_2_ADDR,
	  PMBUS_READ_POUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	/** VR **/
	{ SENSOR_NUM_VOL_P0V8_VDD_1, sensor_dev_xdpe15284, I2C_BUS1, XDPE15284D_ADDR,
	  PMBUS_READ_VOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_page[0],
	  post_xdpe15284_read, NULL, NULL },
	{ SENSOR_NUM_CUR_P0V8_VDD_1, sensor_dev_xdpe15284, I2C_BUS1, XDPE15284D_ADDR,
	  PMBUS_READ_IOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_page[0],
	  post_xdpe15284_read, NULL, NULL },
	{ SENSOR_NUM_PWR_P0V8_VDD_1, sensor_dev_xdpe15284, I2C_BUS1, XDPE15284D_ADDR,
	  PMBUS_READ_POUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_page[0],
	  post_xdpe15284_read, NULL, NULL },
	{ SENSOR_NUM_VOL_P0V8_VDD_2, sensor_dev_xdpe15284, I2C_BUS1, XDPE15284D_ADDR,
	  PMBUS_READ_VOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_page[1],
	  post_xdpe15284_read, NULL, NULL },
	{ SENSOR_NUM_CUR_P0V8_VDD_2, sensor_dev_xdpe15284, I2C_BUS1, XDPE15284D_ADDR,
	  PMBUS_READ_IOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_page[1],
	  post_xdpe15284_read, NULL, NULL },
	{ SENSOR_NUM_PWR_P0V8_VDD_2, sensor_dev_xdpe15284, I2C_BUS1, XDPE15284D_ADDR,
	  PMBUS_READ_POUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_page[1],
	  post_xdpe15284_read, NULL, NULL },

	/** PEX temp **/
	{ SENSOR_NUM_TEMP_PEX_0, sensor_dev_pex89000, I2C_BUS2, PEX89144_I2CS_ADDR, PEX_TEMP,
	  is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_pex89000_read, &tca9543_configs[0], post_pex89000_read,
	  &tca9543_configs[0], &pex_sensor_init_args[0] },
	{ SENSOR_NUM_TEMP_PEX_1, sensor_dev_pex89000, I2C_BUS3, PEX89144_I2CS_ADDR, PEX_TEMP,
	  is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_pex89000_read, &tca9543_configs[1], post_pex89000_read,
	  &tca9543_configs[1], &pex_sensor_init_args[1] },

	/** INA233 12V 1 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_1, sensor_dev_ina233, I2C_BUS4, INA233_12V_1_7_ADDR,
	  PMBUS_READ_VOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[0], &ina233_init_args[0] },
	{ SENSOR_NUM_CUR_P12V_ACCL_1, sensor_dev_ina233, I2C_BUS4, INA233_12V_1_7_ADDR,
	  PMBUS_READ_IOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[0], &ina233_init_args[0] },
	{ SENSOR_NUM_PWR_P12V_ACCL_1, sensor_dev_ina233, I2C_BUS4, INA233_12V_1_7_ADDR,
	  PMBUS_READ_POUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[0], &ina233_init_args[0] },

	/** INA233 12V 2 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_2, sensor_dev_ina233, I2C_BUS4, INA233_12V_2_8_ADDR,
	  PMBUS_READ_VOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[0], &ina233_init_args[1] },
	{ SENSOR_NUM_CUR_P12V_ACCL_2, sensor_dev_ina233, I2C_BUS4, INA233_12V_2_8_ADDR,
	  PMBUS_READ_IOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[0], &ina233_init_args[1] },
	{ SENSOR_NUM_PWR_P12V_ACCL_2, sensor_dev_ina233, I2C_BUS4, INA233_12V_2_8_ADDR,
	  PMBUS_READ_POUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[0], &ina233_init_args[1] },

	/** INA233 12V 3 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_3, sensor_dev_ina233, I2C_BUS4, INA233_12V_3_9_ADDR,
	  PMBUS_READ_VOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[0], &ina233_init_args[2] },
	{ SENSOR_NUM_CUR_P12V_ACCL_3, sensor_dev_ina233, I2C_BUS4, INA233_12V_3_9_ADDR,
	  PMBUS_READ_IOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[0], &ina233_init_args[2] },
	{ SENSOR_NUM_PWR_P12V_ACCL_3, sensor_dev_ina233, I2C_BUS4, INA233_12V_3_9_ADDR,
	  PMBUS_READ_POUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[0], &ina233_init_args[2] },

	/** INA233 12V 4 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_4, sensor_dev_ina233, I2C_BUS4, INA233_12V_4_10_ADDR,
	  PMBUS_READ_VOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[0], &ina233_init_args[3] },
	{ SENSOR_NUM_CUR_P12V_ACCL_4, sensor_dev_ina233, I2C_BUS4, INA233_12V_4_10_ADDR,
	  PMBUS_READ_IOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[0], &ina233_init_args[3] },
	{ SENSOR_NUM_PWR_P12V_ACCL_4, sensor_dev_ina233, I2C_BUS4, INA233_12V_4_10_ADDR,
	  PMBUS_READ_POUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[0], &ina233_init_args[3] },

	/** INA233 12V 5 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_5, sensor_dev_ina233, I2C_BUS4, INA233_12V_5_11_ADDR,
	  PMBUS_READ_VOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[0], &ina233_init_args[4] },
	{ SENSOR_NUM_CUR_P12V_ACCL_5, sensor_dev_ina233, I2C_BUS4, INA233_12V_5_11_ADDR,
	  PMBUS_READ_IOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[0], &ina233_init_args[4] },
	{ SENSOR_NUM_PWR_P12V_ACCL_5, sensor_dev_ina233, I2C_BUS4, INA233_12V_5_11_ADDR,
	  PMBUS_READ_POUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[0], &ina233_init_args[4] },

	/** INA233 12V 6 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_6, sensor_dev_ina233, I2C_BUS4, INA233_12V_6_ADDR,
	  PMBUS_READ_VOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[0], &ina233_init_args[5] },
	{ SENSOR_NUM_CUR_P12V_ACCL_6, sensor_dev_ina233, I2C_BUS4, INA233_12V_6_ADDR,
	  PMBUS_READ_IOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[0], &ina233_init_args[5] },
	{ SENSOR_NUM_PWR_P12V_ACCL_6, sensor_dev_ina233, I2C_BUS4, INA233_12V_6_ADDR,
	  PMBUS_READ_POUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[0], &ina233_init_args[5] },

	/** INA233 12V 7 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_7, sensor_dev_ina233, I2C_BUS4, INA233_12V_1_7_ADDR,
	  PMBUS_READ_VOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[0], &ina233_init_args[6] },
	{ SENSOR_NUM_CUR_P12V_ACCL_7, sensor_dev_ina233, I2C_BUS4, INA233_12V_1_7_ADDR,
	  PMBUS_READ_IOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[0], &ina233_init_args[6] },
	{ SENSOR_NUM_PWR_P12V_ACCL_7, sensor_dev_ina233, I2C_BUS4, INA233_12V_1_7_ADDR,
	  PMBUS_READ_POUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[1], &ina233_init_args[6] },

	/** INA233 12V 8 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_8, sensor_dev_ina233, I2C_BUS4, INA233_12V_2_8_ADDR,
	  PMBUS_READ_VOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[1], &ina233_init_args[7] },
	{ SENSOR_NUM_CUR_P12V_ACCL_8, sensor_dev_ina233, I2C_BUS4, INA233_12V_2_8_ADDR,
	  PMBUS_READ_IOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[1], &ina233_init_args[7] },
	{ SENSOR_NUM_PWR_P12V_ACCL_8, sensor_dev_ina233, I2C_BUS4, INA233_12V_2_8_ADDR,
	  PMBUS_READ_POUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[1], &ina233_init_args[7] },

	/** INA233 12V 9 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_9, sensor_dev_ina233, I2C_BUS4, INA233_12V_3_9_ADDR,
	  PMBUS_READ_VOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[1], &ina233_init_args[8] },
	{ SENSOR_NUM_CUR_P12V_ACCL_9, sensor_dev_ina233, I2C_BUS4, INA233_12V_3_9_ADDR,
	  PMBUS_READ_IOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[1], &ina233_init_args[8] },
	{ SENSOR_NUM_PWR_P12V_ACCL_9, sensor_dev_ina233, I2C_BUS4, INA233_12V_3_9_ADDR,
	  PMBUS_READ_POUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[1], &ina233_init_args[8] },

	/** INA233 12V 10 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_10, sensor_dev_ina233, I2C_BUS4, INA233_12V_4_10_ADDR,
	  PMBUS_READ_VOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[1], &ina233_init_args[9] },
	{ SENSOR_NUM_CUR_P12V_ACCL_10, sensor_dev_ina233, I2C_BUS4, INA233_12V_4_10_ADDR,
	  PMBUS_READ_IOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[1], &ina233_init_args[9] },
	{ SENSOR_NUM_PWR_P12V_ACCL_10, sensor_dev_ina233, I2C_BUS4, INA233_12V_4_10_ADDR,
	  PMBUS_READ_POUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[1], &ina233_init_args[9] },

	/** INA233 12V 11 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_11, sensor_dev_ina233, I2C_BUS4, INA233_12V_5_11_ADDR,
	  PMBUS_READ_VOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[1], &ina233_init_args[10] },
	{ SENSOR_NUM_CUR_P12V_ACCL_11, sensor_dev_ina233, I2C_BUS4, INA233_12V_5_11_ADDR,
	  PMBUS_READ_IOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[1], &ina233_init_args[10] },
	{ SENSOR_NUM_PWR_P12V_ACCL_11, sensor_dev_ina233, I2C_BUS4, INA233_12V_5_11_ADDR,
	  PMBUS_READ_POUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[1], &ina233_init_args[10] },

	/** INA233 12V 12 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_12, sensor_dev_ina233, I2C_BUS4, INA233_12V_12_ADDR,
	  PMBUS_READ_VOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[1], &ina233_init_args[11] },
	{ SENSOR_NUM_CUR_P12V_ACCL_12, sensor_dev_ina233, I2C_BUS4, INA233_12V_12_ADDR,
	  PMBUS_READ_IOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[1], &ina233_init_args[11] },
	{ SENSOR_NUM_PWR_P12V_ACCL_12, sensor_dev_ina233, I2C_BUS4, INA233_12V_12_ADDR,
	  PMBUS_READ_POUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pi4msd5v9542_configs[0],
	  post_ina233_read, &pi4msd5v9542_configs[1], &ina233_init_args[11] },
};

const int SENSOR_CONFIG_SIZE = ARRAY_SIZE(plat_sensor_config);

void load_sensor_config(void)
{
	memcpy(sensor_config, plat_sensor_config, sizeof(plat_sensor_config));
	sensor_config_count = ARRAY_SIZE(plat_sensor_config);

	// Fix config table in different system/config
	pal_extend_sensor_config();
}

bool is_mb_dc_on()
{
	// BIC can check motherboard dc power status by P1V8_PEX voltage
	// Motherboard is dc off if ADC-4 voltage is lower than 1.8V - (1.8V * 7%) = 1.674
	bool ret = false;
	uint8_t channel = P1V8_PEX_ADC_CHANNEL;
	float mb_dc_low_threshold = P1V8_PEX_LOW_THRESHOLD;
	float voltage = 0;

	ret = get_adc_voltage(channel, &voltage);
	if (ret == true) {
		if (voltage < mb_dc_low_threshold) {
			ret = false;
		}
	} else {
		LOG_ERR("Ret false");
	}

	return ret;
}

bool is_dc_access(uint8_t sensor_num)
{
	return is_mb_dc_on();
}

struct k_mutex *get_i2c_mux_mutex(uint8_t i2c_bus)
{
	struct k_mutex *mutex = NULL;

	switch (i2c_bus) {
	case I2C_BUS2:
		mutex = &i2c_2_tca9543_mutex;
		break;
	case I2C_BUS3:
		mutex = &i2c_3_tca9543_mutex;
		break;
	case I2C_BUS4:
		mutex = &i2c_4_pi4msd5v9542_mutex;
		break;
	default:
		LOG_ERR("No support for i2c bus %d mutex", i2c_bus);
		break;
	}

	return mutex;
}
