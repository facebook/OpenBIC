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
#include <string.h>
#include "ast_adc.h"
#include "sensor.h"
#include "plat_hook.h"
#include "plat_i2c.h"
#include "plat_sensor_table.h"
#include "pmbus.h"
#include "util_sys.h"
#include "cci.h"
#include "plat_mctp.h"
#include "pm8702.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(plat_sensor_table);

static uint8_t INA233_DEVICE_ID[4] = { 0x02, 0x54, 0x49, 0xe2 };

sensor_cfg plat_sensor_config[] = {
	/* number,                  type,       port,      address,      offset,
     access check arg0, arg1, cache, cache_status, mux_ADDRess, mux_offset,
     pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_fn  */

	// temperature
	{ SENSOR_NUM_TEMP_TMP75, sensor_dev_tmp75, I2C_BUS3, TMP75_MB_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_CXL, sensor_dev_pm8702, CXL_EID, NONE, chip_temp, dc_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_pm8702_read, NULL, post_pm8702_read, NULL, NULL },
	{ SENSOR_NUM_TEMP_DIMMA, sensor_dev_pm8702, CXL_EID, DIMMA_SPD_ADDR, dimm_temp_from_pioneer,
	  dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_pm8702_read, NULL, post_pm8702_read, NULL,
	  &pm8702_dimm_init_args[0] },
	{ SENSOR_NUM_TEMP_DIMMB, sensor_dev_pm8702, CXL_EID, DIMMB_SPD_ADDR, dimm_temp_from_pioneer,
	  dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_pm8702_read, NULL, post_pm8702_read, NULL,
	  &pm8702_dimm_init_args[1] },
	{ SENSOR_NUM_TEMP_DIMMC, sensor_dev_pm8702, CXL_EID, DIMMC_SPD_ADDR, dimm_temp_from_pioneer,
	  dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_pm8702_read, NULL, post_pm8702_read, NULL,
	  &pm8702_dimm_init_args[2] },
	{ SENSOR_NUM_TEMP_DIMMD, sensor_dev_pm8702, CXL_EID, DIMMD_SPD_ADDR, dimm_temp_from_pioneer,
	  dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_pm8702_read, NULL, post_pm8702_read, NULL,
	  &pm8702_dimm_init_args[3] },
	{ SENSOR_NUM_TEMP_CXL_CNTR, sensor_dev_tmp75, I2C_BUS2, TMP75_ASIC_ADDR, TMP75_TEMP_OFFSET,
	  dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_pm8702_read, NULL, post_pm8702_read, NULL, NULL },

	// ADC
	{ SENSOR_NUM_VOL_STBY5V, sensor_dev_ast_adc, ADC_PORT14, NONE, NONE, stby_access, 711, 200,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_STBY1V2, sensor_dev_ast_adc, ADC_PORT6, NONE, NONE, stby_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_ASIC_1V8, sensor_dev_ast_adc, ADC_PORT7, NONE, NONE, dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_PVPP_AB, sensor_dev_ast_adc, ADC_PORT10, NONE, NONE, dc_access, 2, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_PVPP_CD, sensor_dev_ast_adc, ADC_PORT11, NONE, NONE, dc_access, 2, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_PVTT_AB, sensor_dev_ast_adc, ADC_PORT12, NONE, NONE, dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_PVTT_CD, sensor_dev_ast_adc, ADC_PORT13, NONE, NONE, dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },

};

sensor_cfg ina233_sensor_config_table[] = {
	// INA233
	{ SENSOR_NUM_VOL_STBY12V, sensor_dev_ina233, I2C_BUS1, INA233_12V_ADDR, SMBUS_VOL_CMD,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ina233_init_args[0] },
	{ SENSOR_NUM_VOL_STBY3V3, sensor_dev_ina233, I2C_BUS1, INA233_3V3_ADDR, SMBUS_VOL_CMD,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ina233_init_args[1] },
	{ SENSOR_NUM_CUR_STBY12V, sensor_dev_ina233, I2C_BUS1, INA233_12V_ADDR, SMBUS_CUR_CMD,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ina233_init_args[0] },
	{ SENSOR_NUM_CUR_STBY3V3, sensor_dev_ina233, I2C_BUS1, INA233_3V3_ADDR, SMBUS_CUR_CMD,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ina233_init_args[1] },
	{ SENSOR_NUM_PWR_STBY12V, sensor_dev_ina233, I2C_BUS1, INA233_12V_ADDR, SMBUS_PWR_CMD,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ina233_init_args[0] },
	{ SENSOR_NUM_PWR_STBY3V3, sensor_dev_ina233, I2C_BUS1, INA233_3V3_ADDR, SMBUS_PWR_CMD,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ina233_init_args[1] },
};

sensor_cfg SGY_SQ5220x_sensor_config_table[] = {
	// SGY_SQ5220x
	{ SENSOR_NUM_VOL_STBY12V, sensor_dev_ina230, I2C_BUS1, INA233_12V_ADDR,
	  INA230_BUS_VOL_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &SQ5220x_init_args[0] },
	{ SENSOR_NUM_VOL_STBY3V3, sensor_dev_ina230, I2C_BUS1, INA233_3V3_ADDR,
	  INA230_BUS_VOL_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &SQ5220x_init_args[1] },
	{ SENSOR_NUM_CUR_STBY12V, sensor_dev_ina230, I2C_BUS1, INA233_12V_ADDR, INA230_CUR_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &SQ5220x_init_args[0] },
	{ SENSOR_NUM_CUR_STBY3V3, sensor_dev_ina230, I2C_BUS1, INA233_3V3_ADDR, INA230_CUR_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &SQ5220x_init_args[1] },
	{ SENSOR_NUM_PWR_STBY12V, sensor_dev_ina230, I2C_BUS1, INA233_12V_ADDR, INA230_PWR_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &SQ5220x_init_args[0] },
	{ SENSOR_NUM_PWR_STBY3V3, sensor_dev_ina230, I2C_BUS1, INA233_3V3_ADDR, INA230_PWR_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &SQ5220x_init_args[1] },
};

sensor_cfg VR_RNS_sensor_config_table[] = {
	// VR temperature
	{ SENSOR_NUM_TEMP_VR0V9A, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_A0V9_ADDR,
	  SMBUS_TEMP_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_isl69254iraz_t_read,
	  &isl69254iraz_t_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_VR0V8A, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_A0V8_ADDR,
	  SMBUS_TEMP_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_isl69254iraz_t_read,
	  &isl69254iraz_t_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_VR0V8D, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_D0V8_ADDR,
	  SMBUS_TEMP_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_isl69254iraz_t_read,
	  &isl69254iraz_t_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_VRVDDQAB, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_VDDQAB_ADDR,
	  SMBUS_TEMP_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_isl69254iraz_t_read,
	  &isl69254iraz_t_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_VRVDDQCD, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_VDDQCD_ADDR,
	  SMBUS_TEMP_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_isl69254iraz_t_read,
	  &isl69254iraz_t_pre_read_args[0], NULL, NULL, NULL },

	// VR Voltage
	{ SENSOR_NUM_VOL_VR0V9A, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_A0V9_ADDR, SMBUS_VOL_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_isl69254iraz_t_read, &isl69254iraz_t_pre_read_args[0], NULL, NULL,
	  NULL },
	{ SENSOR_NUM_VOL_VR0V8A, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_A0V8_ADDR, SMBUS_VOL_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_isl69254iraz_t_read, &isl69254iraz_t_pre_read_args[1], NULL, NULL,
	  NULL },
	{ SENSOR_NUM_VOL_VR0V8D, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_D0V8_ADDR, SMBUS_VOL_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_isl69254iraz_t_read, &isl69254iraz_t_pre_read_args[0], NULL, NULL,
	  NULL },
	{ SENSOR_NUM_VOL_VRVDDQAB, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_VDDQAB_ADDR,
	  SMBUS_VOL_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_isl69254iraz_t_read,
	  &isl69254iraz_t_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_VRVDDQCD, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_VDDQCD_ADDR,
	  SMBUS_VOL_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_isl69254iraz_t_read,
	  &isl69254iraz_t_pre_read_args[0], NULL, NULL, NULL },

	// VR Current
	{ SENSOR_NUM_CUR_VR0V9A, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_A0V9_ADDR, SMBUS_CUR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_isl69254iraz_t_read, &isl69254iraz_t_pre_read_args[0], NULL, NULL,
	  NULL },
	{ SENSOR_NUM_CUR_VR0V8A, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_A0V8_ADDR, SMBUS_CUR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_isl69254iraz_t_read, &isl69254iraz_t_pre_read_args[1], NULL, NULL,
	  NULL },
	{ SENSOR_NUM_CUR_VR0V8D, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_D0V8_ADDR, SMBUS_CUR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_isl69254iraz_t_read, &isl69254iraz_t_pre_read_args[0], NULL, NULL,
	  NULL },
	{ SENSOR_NUM_CUR_VRVDDQAB, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_VDDQAB_ADDR,
	  SMBUS_CUR_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_isl69254iraz_t_read,
	  &isl69254iraz_t_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_VRVDDQCD, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_VDDQCD_ADDR,
	  SMBUS_CUR_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_isl69254iraz_t_read,
	  &isl69254iraz_t_pre_read_args[0], NULL, NULL, NULL },

	// VR Power
	{ SENSOR_NUM_PWR_VR0V9A, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_A0V9_ADDR, SMBUS_PWR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_isl69254iraz_t_read, &isl69254iraz_t_pre_read_args[0],
	  post_isl69254iraz_t_read, NULL, NULL },
	{ SENSOR_NUM_PWR_VR0V8A, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_A0V8_ADDR, SMBUS_PWR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_isl69254iraz_t_read, &isl69254iraz_t_pre_read_args[1],
	  post_isl69254iraz_t_read, NULL, NULL },
	{ SENSOR_NUM_PWR_VR0V8D, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_D0V8_ADDR, SMBUS_PWR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_isl69254iraz_t_read, &isl69254iraz_t_pre_read_args[0],
	  post_isl69254iraz_t_read, NULL, NULL },
	{ SENSOR_NUM_PWR_VRVDDQAB, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_VDDQAB_ADDR,
	  SMBUS_PWR_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_isl69254iraz_t_read,
	  &isl69254iraz_t_pre_read_args[1], post_isl69254iraz_t_read, NULL, NULL },
	{ SENSOR_NUM_PWR_VRVDDQCD, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_VDDQCD_ADDR,
	  SMBUS_PWR_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_isl69254iraz_t_read,
	  &isl69254iraz_t_pre_read_args[0], post_isl69254iraz_t_read, NULL, NULL },
};

sensor_cfg VR_INF_sensor_config_table[] = {
	// VR temperature
	{ SENSOR_NUM_TEMP_VR0V9A, sensor_dev_xdpe12284c, I2C_BUS10, VR_A0V9_ADDR, SMBUS_TEMP_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_VR0V8A, sensor_dev_xdpe12284c, I2C_BUS10, VR_A0V8_ADDR, SMBUS_TEMP_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[1], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_VR0V8D, sensor_dev_xdpe12284c, I2C_BUS10, VR_D0V8_ADDR, SMBUS_TEMP_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_VRVDDQAB, sensor_dev_xdpe12284c, I2C_BUS10, VR_VDDQAB_ADDR,
	  SMBUS_TEMP_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[1], NULL, NULL,
	  NULL },
	{ SENSOR_NUM_TEMP_VRVDDQCD, sensor_dev_xdpe12284c, I2C_BUS10, VR_VDDQCD_ADDR,
	  SMBUS_TEMP_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0], NULL, NULL,
	  NULL },

	// VR Voltage
	{ SENSOR_NUM_VOL_VR0V9A, sensor_dev_xdpe12284c, I2C_BUS10, VR_A0V9_ADDR, SMBUS_VOL_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0], NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_VR0V8A, sensor_dev_xdpe12284c, I2C_BUS10, VR_A0V8_ADDR, SMBUS_VOL_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[1], NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_VR0V8D, sensor_dev_xdpe12284c, I2C_BUS10, VR_D0V8_ADDR, SMBUS_VOL_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0], NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_VRVDDQAB, sensor_dev_xdpe12284c, I2C_BUS10, VR_VDDQAB_ADDR, SMBUS_VOL_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[1], NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_VRVDDQCD, sensor_dev_xdpe12284c, I2C_BUS10, VR_VDDQCD_ADDR, SMBUS_VOL_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0], NULL, NULL, NULL },

	// VR Current
	{ SENSOR_NUM_CUR_VR0V9A, sensor_dev_xdpe12284c, I2C_BUS10, VR_A0V9_ADDR, SMBUS_CUR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0], NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_VR0V8A, sensor_dev_xdpe12284c, I2C_BUS10, VR_A0V8_ADDR, SMBUS_CUR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[1], NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_VR0V8D, sensor_dev_xdpe12284c, I2C_BUS10, VR_D0V8_ADDR, SMBUS_CUR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0], NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_VRVDDQAB, sensor_dev_xdpe12284c, I2C_BUS10, VR_VDDQAB_ADDR, SMBUS_CUR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[1], NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_VRVDDQCD, sensor_dev_xdpe12284c, I2C_BUS10, VR_VDDQCD_ADDR, SMBUS_CUR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0], NULL, NULL, NULL },

	// VR Power
	{ SENSOR_NUM_PWR_VR0V9A, sensor_dev_xdpe12284c, I2C_BUS10, VR_A0V9_ADDR, SMBUS_PWR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0], NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_VR0V8A, sensor_dev_xdpe12284c, I2C_BUS10, VR_A0V8_ADDR, SMBUS_PWR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[1], NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_VR0V8D, sensor_dev_xdpe12284c, I2C_BUS10, VR_D0V8_ADDR, SMBUS_PWR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0], NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_VRVDDQAB, sensor_dev_xdpe12284c, I2C_BUS10, VR_VDDQAB_ADDR, SMBUS_PWR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[1], NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_VRVDDQCD, sensor_dev_xdpe12284c, I2C_BUS10, VR_VDDQCD_ADDR, SMBUS_PWR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0], NULL, NULL, NULL },
};

int check_pwr_monitor_type(void)
{
	uint8_t retry = 5;
	uint8_t ret = 0;
	I2C_MSG msg = { 0 };
	msg.bus = I2C_BUS1;
	msg.target_addr = INA233_12V_ADDR;
	msg.tx_len = 1;
	msg.rx_len = 7;
	msg.data[0] = PMBUS_MFR_ID;

	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("Failed to read Power moniter IC_DEVICE_ID: register(0x%x)",
			PMBUS_IC_DEVICE_ID);
		return -1;
	}

	if (memcmp(msg.data, INA233_DEVICE_ID, sizeof(INA233_DEVICE_ID)) == 0) {
		LOG_ERR("Power Monitor type: INA233");
		ret = PWR_INA233;
	} else {
		LOG_ERR("Power Monitor type: SGY");
		ret = PWR_SGY;
	}

	return ret;
}

int check_vr_type(void)
{
	uint8_t retry = 5;
	uint8_t ret = 0;
	I2C_MSG msg = { 0 };
	msg.bus = I2C_BUS10;
	msg.target_addr = VR_A0V9_ADDR;
	msg.tx_len = 1;
	msg.rx_len = 7;
	msg.data[0] = PMBUS_IC_DEVICE_ID;

	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("Failed to read VR IC_DEVICE_ID: register(0x%x)", PMBUS_IC_DEVICE_ID);
		return -1;
	}

	if (memcmp(msg.data, ISL69254_DEVICE_ID, sizeof(ISL69254_DEVICE_ID)) == 0) {
		LOG_ERR("VR type: RNS");
		ret = VR_RNS;
	} else if (memcmp(msg.data, XDPE12284C_DEVICE_ID, sizeof(XDPE12284C_DEVICE_ID)) == 0) {
		LOG_ERR("VR type: INF");
		ret = VR_INF;
	} else {
		LOG_ERR("Unknown VR type");
		ret = -1;
	}

	return ret;
}

void pal_extend_sensor_config()
{
	uint8_t sensor_count = 0;
	uint8_t board_pwr_version = check_pwr_monitor_type();
	if (board_pwr_version == PWR_INA233) {
		sensor_count = ARRAY_SIZE(ina233_sensor_config_table);
		for (int index = 0; index < sensor_count; index++) {
			add_sensor_config(ina233_sensor_config_table[index]);
		}
	} else {
		sensor_count = ARRAY_SIZE(SGY_SQ5220x_sensor_config_table);
		for (int index = 0; index < sensor_count; index++) {
			add_sensor_config(SGY_SQ5220x_sensor_config_table[index]);
		}
	}

	uint8_t board_VR_version = check_vr_type();
	switch (board_VR_version) {
	case VR_RNS:
		sensor_count = ARRAY_SIZE(VR_RNS_sensor_config_table);
		for (int index = 0; index < sensor_count; index++) {
			add_sensor_config(VR_RNS_sensor_config_table[index]);
		}
		break;
	case VR_INF:
		sensor_count = ARRAY_SIZE(VR_INF_sensor_config_table);
		for (int index = 0; index < sensor_count; index++) {
			add_sensor_config(VR_INF_sensor_config_table[index]);
		}
		break;
	default:
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
	extend_sensor_config_size += ARRAY_SIZE(ina233_sensor_config_table);
	extend_sensor_config_size += ARRAY_SIZE(VR_RNS_sensor_config_table);

	return extend_sensor_config_size;
}

const int SENSOR_CONFIG_SIZE = ARRAY_SIZE(plat_sensor_config);

void load_sensor_config(void)
{
	memcpy(sensor_config, plat_sensor_config, SENSOR_CONFIG_SIZE * sizeof(sensor_cfg));
	sensor_config_count = SENSOR_CONFIG_SIZE;

	pal_extend_sensor_config();
}
