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

#include <stdio.h>
#include "sensor.h"
#include "ast_adc.h"
#include "pmbus.h"
#include "plat_i2c.h"
#include "plat_hook.h"
#include "plat_sensor_table.h"

sensor_cfg plat_sensor_config[] = {
	/* number,                  type,       port,      address,      offset,
     access check arg0, arg1, cache, cache_status, mux_ADDRess, mux_offset,
     pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_fn  */

	// Temperature =========================================================================
	{ SENSOR_NUM_TEMP_TMP75, sensor_dev_tmp75, I2C_BUS3, TMP75_1OU_BOARD_ADDR,
	  TMP75_TEMP_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	// ADC =================================================================================
	{ SENSOR_NUM_VOL_P1V2_STBY, sensor_dev_ast_adc, ADC_PORT3, NONE, NONE, stby_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_P1V2_ASIC, sensor_dev_ast_adc, ADC_PORT5, NONE, NONE, dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_P1V8_ASIC, sensor_dev_ast_adc, ADC_PORT6, NONE, NONE, dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_PVPP_AB, sensor_dev_ast_adc, ADC_PORT8, NONE, NONE, dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_PVPP_CD, sensor_dev_ast_adc, ADC_PORT9, NONE, NONE, dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_PVTT_AB, sensor_dev_ast_adc, ADC_PORT10, NONE, NONE, dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_PVTT_CD, sensor_dev_ast_adc, ADC_PORT11, NONE, NONE, dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_P0V75_ASIC, sensor_dev_ast_adc, ADC_PORT14, NONE, NONE, dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },

	// INA233 ==============================================================================
	{ SENSOR_NUM_VOL_P12V_STBY, sensor_dev_ina233, I2C_BUS1, INA233_12V_ADDR, PMBUS_READ_VOUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ina233_init_args[0] },
	{ SENSOR_NUM_VOL_P3V3_STBY, sensor_dev_ina233, I2C_BUS1, INA233_3V3_ADDR, PMBUS_READ_VOUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ina233_init_args[1] },
	{ SENSOR_NUM_CUR_P12V_STBY, sensor_dev_ina233, I2C_BUS1, INA233_12V_ADDR, PMBUS_READ_IOUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ina233_init_args[0] },
	{ SENSOR_NUM_CUR_P3V3_STBY, sensor_dev_ina233, I2C_BUS1, INA233_3V3_ADDR, PMBUS_READ_IOUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ina233_init_args[1] },
	{ SENSOR_NUM_PWR_P12V_STBY, sensor_dev_ina233, I2C_BUS1, INA233_12V_ADDR, PMBUS_READ_POUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ina233_init_args[0] },
	{ SENSOR_NUM_PWR_P3V3_STBY, sensor_dev_ina233, I2C_BUS1, INA233_3V3_ADDR, PMBUS_READ_POUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ina233_init_args[1] },

	// VR ===============================================================================
	{ SENSOR_NUM_TEMP_P0V85_ASIC, sensor_dev_xdpe12284c, I2C_BUS6, VR_P0V85_ASIC_ADDR,
	  PMBUS_READ_TEMPERATURE_1, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_xdpe12284c_read,
	  &xdpe12284c_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_PVDDQ_AB, sensor_dev_xdpe12284c, I2C_BUS6, VR_PVDDQAB_ADDR,
	  PMBUS_READ_TEMPERATURE_1, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_xdpe12284c_read,
	  &xdpe12284c_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_P0V8_ASIC, sensor_dev_xdpe12284c, I2C_BUS6, VR_P0V8_ASIC_ADDR,
	  PMBUS_READ_TEMPERATURE_1, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_xdpe12284c_read,
	  &xdpe12284c_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_PVDDQ_CD, sensor_dev_xdpe12284c, I2C_BUS6, VR_PVDDQCD_ADDR,
	  PMBUS_READ_TEMPERATURE_1, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_xdpe12284c_read,
	  &xdpe12284c_pre_read_args[0], NULL, NULL, NULL },

	{ SENSOR_NUM_VOL_P0V85_ASIC, sensor_dev_xdpe12284c, I2C_BUS6, VR_P0V85_ASIC_ADDR,
	  PMBUS_READ_VOUT, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_xdpe12284c_read,
	  &xdpe12284c_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_PVDDQ_AB, sensor_dev_xdpe12284c, I2C_BUS6, VR_PVDDQAB_ADDR,
	  PMBUS_READ_VOUT, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_xdpe12284c_read,
	  &xdpe12284c_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_P0V8_ASIC, sensor_dev_xdpe12284c, I2C_BUS6, VR_P0V8_ASIC_ADDR,
	  PMBUS_READ_VOUT, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_xdpe12284c_read,
	  &xdpe12284c_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_PVDDQ_CD, sensor_dev_xdpe12284c, I2C_BUS6, VR_PVDDQCD_ADDR,
	  PMBUS_READ_VOUT, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_xdpe12284c_read,
	  &xdpe12284c_pre_read_args[0], NULL, NULL, NULL },

	{ SENSOR_NUM_PWR_P0V85_ASIC, sensor_dev_xdpe12284c, I2C_BUS6, VR_P0V85_ASIC_ADDR,
	  PMBUS_READ_POUT, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_xdpe12284c_read,
	  &xdpe12284c_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_PVDDQ_AB, sensor_dev_xdpe12284c, I2C_BUS6, VR_PVDDQAB_ADDR,
	  PMBUS_READ_POUT, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_xdpe12284c_read,
	  &xdpe12284c_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_P0V8_ASIC, sensor_dev_xdpe12284c, I2C_BUS6, VR_P0V8_ASIC_ADDR,
	  PMBUS_READ_POUT, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_xdpe12284c_read,
	  &xdpe12284c_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_PVDDQ_CD, sensor_dev_xdpe12284c, I2C_BUS6, VR_PVDDQCD_ADDR,
	  PMBUS_READ_POUT, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_xdpe12284c_read,
	  &xdpe12284c_pre_read_args[0], NULL, NULL, NULL },

	{ SENSOR_NUM_CUR_P0V85_ASIC, sensor_dev_xdpe12284c, I2C_BUS6, VR_P0V85_ASIC_ADDR,
	  PMBUS_READ_IOUT, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_xdpe12284c_read,
	  &xdpe12284c_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_PVDDQ_AB, sensor_dev_xdpe12284c, I2C_BUS6, VR_PVDDQAB_ADDR,
	  PMBUS_READ_IOUT, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_xdpe12284c_read,
	  &xdpe12284c_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_P0V8_ASIC, sensor_dev_xdpe12284c, I2C_BUS6, VR_P0V8_ASIC_ADDR,
	  PMBUS_READ_IOUT, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_xdpe12284c_read,
	  &xdpe12284c_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_PVDDQ_CD, sensor_dev_xdpe12284c, I2C_BUS6, VR_PVDDQCD_ADDR,
	  PMBUS_READ_IOUT, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_xdpe12284c_read,
	  &xdpe12284c_pre_read_args[0], NULL, NULL, NULL },
};

const int SENSOR_CONFIG_SIZE = ARRAY_SIZE(plat_sensor_config);
