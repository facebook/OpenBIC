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
#include <string.h>
#include <logging/log.h>

#include "ast_adc.h"
#include "sensor.h"
#include "power_status.h"
#include "pmbus.h"
#include "pt5161l.h"
#include "ina233.h"
#include "plat_i2c.h"
#include "plat_sensor_table.h"
#include "plat_hook.h"
#include "plat_class.h"
#include "plat_power_seq.h"

LOG_MODULE_REGISTER(plat_sensor);

bool e1s_access(uint8_t sensor_num);
bool retimer_access(uint8_t sensor_num);
bool edge_access(uint8_t sensor_num);

sensor_cfg plat_sensor_config[] = {
	/*  number,
        type,
        port,
        address,
        offset,
        access check,
        arg0,
        arg1,
        sample_count,
        cache,
        cache_status,
        mux_ADDRess,
        mux_offset,
        pre_sensor_read_fn,
        pre_sensor_read_args,
        post_sensor_read_fn,
        post_sensor_read_args
    */

	//INA233 VOL
	{ SENSOR_NUM_1OU_P12V_EDGE_VOLT, sensor_dev_ina233, I2C_BUS3, INA233_EXPA_MAIN_ADDR,
	  INA233_VOLT_OFFSET, edge_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[5] },

	//INA233 CURR
	{ SENSOR_NUM_1OU_P12V_EDGE_CURR, sensor_dev_ina233, I2C_BUS3, INA233_EXPA_MAIN_ADDR,
	  INA233_CURR_OFFSET, edge_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[5] },

	//INA233 PWR
	{ SENSOR_NUM_1OU_P12V_EDGE_PWR, sensor_dev_ina233, I2C_BUS3, INA233_EXPA_MAIN_ADDR,
	  INA233_PWR_OFFSET, edge_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[5] },
};
sensor_cfg plat_expansion_A_sensor_config[] = {

	//E1S Temp
	{ SENSOR_NUM_1OU_E1S_SSD0_TEMP_C, sensor_dev_nvme, I2C_BUS2, NVME_ADDR, NVME_TEMP_OFFSET,
	  e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &i2c_proc_args[5], post_i2c_bus_read,
	  &i2c_proc_args[5], NULL },

	{ SENSOR_NUM_1OU_E1S_SSD1_TEMP_C, sensor_dev_nvme, I2C_BUS2, NVME_ADDR, NVME_TEMP_OFFSET,
	  e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &i2c_proc_args[1], post_i2c_bus_read,
	  &i2c_proc_args[1], NULL },

	{ SENSOR_NUM_1OU_E1S_SSD2_TEMP_C, sensor_dev_nvme, I2C_BUS2, NVME_ADDR, NVME_TEMP_OFFSET,
	  e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &i2c_proc_args[0], post_i2c_bus_read,
	  &i2c_proc_args[0], NULL },

	//Temp
	{ SENSOR_NUM_1OU_TEMP, sensor_dev_tmp75, I2C_BUS4, TMP75_EXPA_TEMP_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	//reconfig the retimer when the first read retimer temperature
	{ SENSOR_NUM_1OU_RE_TIMER_TEMP_C, sensor_dev_max, I2C_BUS4, EXPA_RETIMER_ADDR,
	  PT5161L_TEMP_OFFSET, retimer_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_retimer_read, NULL, NULL, NULL,
	  &pt5161l_init_args[0] },

	//adc
	{ SENSOR_NUM_1OU_P3V3_STBY_ADC_VOLT, sensor_dev_ast_adc, ADC_PORT5, NONE, NONE, stby_access,
	  2, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_expa_asd_init_args[0] },

	{ SENSOR_NUM_1OU_E1S_SSD0_P3V3_ADC_VOLT, sensor_dev_ast_adc, ADC_PORT2, NONE, NONE,
	  e1s_access, 2, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_expa_asd_init_args[0] },

	{ SENSOR_NUM_1OU_E1S_SSD1_P3V3_ADC_VOLT, sensor_dev_ast_adc, ADC_PORT1, NONE, NONE,
	  e1s_access, 2, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_expa_asd_init_args[0] },

	{ SENSOR_NUM_1OU_E1S_SSD2_P3V3_ADC_VOLT, sensor_dev_ast_adc, ADC_PORT0, NONE, NONE,
	  e1s_access, 2, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_expa_asd_init_args[0] },

	{ SENSOR_NUM_1OU_P1V8_ADC_VOLT, sensor_dev_ast_adc, ADC_PORT6, NONE, NONE, dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_expa_asd_init_args[0] },

	{ SENSOR_NUM_1OU_P0V9_ADC_VOLT, sensor_dev_ast_adc, ADC_PORT7, NONE, NONE, dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_expa_asd_init_args[0] },

	{ SENSOR_NUM_1OU_P1V2_ADC_VOLT, sensor_dev_ast_adc, ADC_PORT8, NONE, NONE, dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_expa_asd_init_args[1] },

	//INA233 VOL
	{ SENSOR_NUM_1OU_E1S_SSD0_P12V_VOLT, sensor_dev_ina233, I2C_BUS3, INA233_EXPA_E1S_0_ADDR,
	  INA233_VOLT_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[0] },

	{ SENSOR_NUM_1OU_E1S_SSD1_P12V_VOLT, sensor_dev_ina233, I2C_BUS3, INA233_EXPA_E1S_1_ADDR,
	  INA233_VOLT_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[1] },

	{ SENSOR_NUM_1OU_E1S_SSD2_P12V_VOLT, sensor_dev_ina233, I2C_BUS3, INA233_EXPA_E1S_2_ADDR,
	  INA233_VOLT_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[2] },

	//INA233 CURR
	{ SENSOR_NUM_1OU_E1S_SSD0_P12V_CURR, sensor_dev_ina233, I2C_BUS3, INA233_EXPA_E1S_0_ADDR,
	  INA233_CURR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[0] },

	{ SENSOR_NUM_1OU_E1S_SSD1_P12V_CURR, sensor_dev_ina233, I2C_BUS3, INA233_EXPA_E1S_1_ADDR,
	  INA233_CURR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[1] },

	{ SENSOR_NUM_1OU_E1S_SSD2_P12V_CURR, sensor_dev_ina233, I2C_BUS3, INA233_EXPA_E1S_2_ADDR,
	  INA233_CURR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[2] },

	//INA233 PWR
	{ SENSOR_NUM_1OU_E1S_SSD0_P12V_PWR, sensor_dev_ina233, I2C_BUS3, INA233_EXPA_E1S_0_ADDR,
	  INA233_PWR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[0] },

	{ SENSOR_NUM_1OU_E1S_SSD1_P12V_PWR, sensor_dev_ina233, I2C_BUS3, INA233_EXPA_E1S_1_ADDR,
	  INA233_PWR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[1] },

	{ SENSOR_NUM_1OU_E1S_SSD2_P12V_PWR, sensor_dev_ina233, I2C_BUS3, INA233_EXPA_E1S_2_ADDR,
	  INA233_PWR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[2] },

};

sensor_cfg plat_expansion_B_sensor_config[] = {

	//E1S Temp
	{ SENSOR_NUM_2OU_E1S_SSD0_TEMP_C, sensor_dev_nvme, I2C_BUS2, NVME_ADDR, NVME_TEMP_OFFSET,
	  e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &i2c_proc_args[4], post_i2c_bus_read,
	  &i2c_proc_args[4], NULL },

	{ SENSOR_NUM_2OU_E1S_SSD1_TEMP_C, sensor_dev_nvme, I2C_BUS2, NVME_ADDR, NVME_TEMP_OFFSET,
	  e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &i2c_proc_args[3], post_i2c_bus_read,
	  &i2c_proc_args[3], NULL },

	{ SENSOR_NUM_2OU_E1S_SSD2_TEMP_C, sensor_dev_nvme, I2C_BUS2, NVME_ADDR, NVME_TEMP_OFFSET,
	  e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &i2c_proc_args[2], post_i2c_bus_read,
	  &i2c_proc_args[2], NULL },

	{ SENSOR_NUM_2OU_E1S_SSD3_TEMP_C, sensor_dev_nvme, I2C_BUS2, NVME_ADDR, NVME_TEMP_OFFSET,
	  e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &i2c_proc_args[1], post_i2c_bus_read,
	  &i2c_proc_args[1], NULL },

	{ SENSOR_NUM_2OU_E1S_SSD4_TEMP_C, sensor_dev_nvme, I2C_BUS2, NVME_ADDR, NVME_TEMP_OFFSET,
	  e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &i2c_proc_args[0], post_i2c_bus_read,
	  &i2c_proc_args[0], NULL },

	//Temp
	{ SENSOR_NUM_2OU_TEMP, sensor_dev_tmp75, I2C_BUS4, TMP75_EXPB_TEMP_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	//adc
	{ SENSOR_NUM_2OU_E1S_SSD0_P3V3_ADC_VOLT, sensor_dev_ast_adc, ADC_PORT4, NONE, NONE,
	  e1s_access, 2, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_expb_asd_init_args[0] },

	{ SENSOR_NUM_2OU_E1S_SSD1_P3V3_ADC_VOLT, sensor_dev_ast_adc, ADC_PORT3, NONE, NONE,
	  e1s_access, 2, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_expb_asd_init_args[0] },

	{ SENSOR_NUM_2OU_E1S_SSD2_P3V3_ADC_VOLT, sensor_dev_ast_adc, ADC_PORT2, NONE, NONE,
	  e1s_access, 2, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_expb_asd_init_args[0] },

	{ SENSOR_NUM_2OU_E1S_SSD3_P3V3_ADC_VOLT, sensor_dev_ast_adc, ADC_PORT1, NONE, NONE,
	  e1s_access, 2, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_expb_asd_init_args[0] },

	{ SENSOR_NUM_2OU_E1S_SSD4_P3V3_ADC_VOLT, sensor_dev_ast_adc, ADC_PORT0, NONE, NONE,
	  e1s_access, 2, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_expb_asd_init_args[0] },

	{ SENSOR_NUM_2OU_P3V3_STBY_ADC_VOLT, sensor_dev_ast_adc, ADC_PORT5, NONE, NONE, stby_access,
	  2, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_expb_asd_init_args[0] },

	{ SENSOR_NUM_2OU_P1V8_STBY_ADC_VOLT, sensor_dev_ast_adc, ADC_PORT11, NONE, NONE,
	  stby_access, 1, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_expb_asd_init_args[1] },

	{ SENSOR_NUM_2OU_P1V2_STBY_ADC_VOLT, sensor_dev_ast_adc, ADC_PORT14, NONE, NONE,
	  stby_access, 1, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_expb_asd_init_args[1] },

	//INA233 VOL
	{ SENSOR_NUM_2OU_E1S_SSD0_P12V_VOLT, sensor_dev_ina233, I2C_BUS3, INA233_EXPB_E1S_0_ADDR,
	  INA233_VOLT_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[0] },

	{ SENSOR_NUM_2OU_E1S_SSD1_P12V_VOLT, sensor_dev_ina233, I2C_BUS3, INA233_EXPB_E1S_1_ADDR,
	  INA233_VOLT_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[1] },

	{ SENSOR_NUM_2OU_E1S_SSD2_P12V_VOLT, sensor_dev_ina233, I2C_BUS3, INA233_EXPB_E1S_2_ADDR,
	  INA233_VOLT_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[2] },

	{ SENSOR_NUM_2OU_E1S_SSD3_P12V_VOLT, sensor_dev_ina233, I2C_BUS3, INA233_EXPB_E1S_3_ADDR,
	  INA233_VOLT_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[3] },

	{ SENSOR_NUM_2OU_E1S_SSD4_P12V_VOLT, sensor_dev_ina233, I2C_BUS3, INA233_EXPB_E1S_4_ADDR,
	  INA233_VOLT_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[4] },

	//INA233 CURR
	{ SENSOR_NUM_2OU_E1S_SSD0_P12V_CURR, sensor_dev_ina233, I2C_BUS3, INA233_EXPB_E1S_0_ADDR,
	  INA233_CURR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[0] },

	{ SENSOR_NUM_2OU_E1S_SSD1_P12V_CURR, sensor_dev_ina233, I2C_BUS3, INA233_EXPB_E1S_1_ADDR,
	  INA233_CURR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[1] },

	{ SENSOR_NUM_2OU_E1S_SSD2_P12V_CURR, sensor_dev_ina233, I2C_BUS3, INA233_EXPB_E1S_2_ADDR,
	  INA233_CURR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[2] },

	{ SENSOR_NUM_2OU_E1S_SSD3_P12V_CURR, sensor_dev_ina233, I2C_BUS3, INA233_EXPB_E1S_3_ADDR,
	  INA233_CURR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[3] },

	{ SENSOR_NUM_2OU_E1S_SSD4_P12V_CURR, sensor_dev_ina233, I2C_BUS3, INA233_EXPB_E1S_4_ADDR,
	  INA233_CURR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[4] },

	//INA233 PWR
	{ SENSOR_NUM_2OU_E1S_SSD0_P12V_PWR, sensor_dev_ina233, I2C_BUS3, INA233_EXPB_E1S_0_ADDR,
	  INA233_PWR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[0] },

	{ SENSOR_NUM_2OU_E1S_SSD1_P12V_PWR, sensor_dev_ina233, I2C_BUS3, INA233_EXPB_E1S_1_ADDR,
	  INA233_PWR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[1] },

	{ SENSOR_NUM_2OU_E1S_SSD2_P12V_PWR, sensor_dev_ina233, I2C_BUS3, INA233_EXPB_E1S_2_ADDR,
	  INA233_PWR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[2] },

	{ SENSOR_NUM_2OU_E1S_SSD3_P12V_PWR, sensor_dev_ina233, I2C_BUS3, INA233_EXPB_E1S_3_ADDR,
	  INA233_PWR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[3] },

	{ SENSOR_NUM_2OU_E1S_SSD4_P12V_PWR, sensor_dev_ina233, I2C_BUS3, INA233_EXPB_E1S_4_ADDR,
	  INA233_PWR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[4] },
};

const int SENSOR_CONFIG_SIZE = ARRAY_SIZE(plat_sensor_config);
const static uint8_t ina233_addr_opa[] = { INA233_EXPA_E1S_0_ADDR, INA233_EXPA_E1S_1_ADDR,
					   INA233_EXPA_E1S_2_ADDR };
const static uint8_t ina233_addr_opb[] = { INA233_EXPB_E1S_0_ADDR, INA233_EXPB_E1S_1_ADDR,
					   INA233_EXPB_E1S_2_ADDR, INA233_EXPB_E1S_3_ADDR,
					   INA233_EXPB_E1S_4_ADDR };
const static uint8_t e1s_adc_channel_opa[] = { ADC_PORT2, ADC_PORT1, ADC_PORT0 };
const static uint8_t e1s_adc_channel_opb[] = { ADC_PORT4, ADC_PORT3, ADC_PORT2, ADC_PORT1,
					       ADC_PORT0 };

void load_sensor_config(void)
{
	// According to card type change address of INA233 sensor
	change_ina233_sensor_addr();

	// According to card position change sensor number
	pal_change_sensor_config_number();

	memcpy(sensor_config, plat_sensor_config, sizeof(plat_sensor_config));
	sensor_config_count = ARRAY_SIZE(plat_sensor_config);

	// Fix config table in different system/config
	pal_extend_sensor_config();

	uint8_t power_monitor_ic_type = check_pwr_monitor_type();
	if (power_monitor_ic_type == PWR_SQ5220X) {
		change_power_monitor_config_for_sq5220x();
	}
}

uint8_t pal_get_extend_sensor_config()
{
	uint8_t extend_sensor_table_size = 0;
	uint8_t card_type = get_card_type();
	switch (card_type) {
	case CARD_TYPE_OPA:
		extend_sensor_table_size += ARRAY_SIZE(plat_expansion_A_sensor_config);
		break;

	case CARD_TYPE_OPB:
		extend_sensor_table_size += ARRAY_SIZE(plat_expansion_B_sensor_config);
		break;
	default:
		LOG_ERR("Unsupported card type, Card type: 0x%x", card_type);
		break;
	}

	return extend_sensor_table_size;
}

int check_pwr_monitor_type(void)
{
	uint8_t retry = 5;
	uint8_t ret = 0;
	I2C_MSG msg = { 0 };
	msg.bus = I2C_BUS3;
	msg.target_addr = INA233_EXPA_E1S_1_ADDR;
	msg.tx_len = 1;
	msg.rx_len = 3;
	msg.data[0] = PMBUS_MFR_ID;

	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("Failed to read Power moniter IC_DEVICE_ID: register(0x%x)",
			PMBUS_IC_DEVICE_ID);
		return -1;
	}

	if (memcmp(msg.data, INA233_DEVICE_ID, sizeof(INA233_DEVICE_ID)) == 0) {
		LOG_INF("Power Monitor type: INA233");
		ret = PWR_INA233;
	} else {
		LOG_INF("Power Monitor type: SQ5220X");
		ret = PWR_SQ5220X;
	}

	return ret;
}

void change_ina233_sensor_addr()
{
	int index = 0;
	uint8_t card_type = get_card_type();

	// Default sensor table is defined for OPA
	// if the card type is OPB, change INA233 device address
	if (card_type == CARD_TYPE_OPB) {
		for (index = 0; index < SENSOR_CONFIG_SIZE; index++) {
			if (plat_sensor_config[index].type == sensor_dev_ina233) {
				switch (plat_sensor_config[index].target_addr) {
				case INA233_EXPA_E1S_0_ADDR:
					plat_sensor_config[index].target_addr =
						INA233_EXPB_E1S_0_ADDR;
					break;
				case INA233_EXPA_E1S_1_ADDR:
					plat_sensor_config[index].target_addr =
						INA233_EXPB_E1S_1_ADDR;
					break;
				case INA233_EXPA_E1S_2_ADDR:
					plat_sensor_config[index].target_addr =
						INA233_EXPB_E1S_2_ADDR;
					break;
				case INA233_EXPA_MAIN_ADDR:
					plat_sensor_config[index].target_addr =
						INA233_EXPB_MAIN_ADDR;
					break;
				default:
					break;
				}
			}
		}
	}
}

void change_power_monitor_config_for_sq5220x(void)
{
	uint8_t index;

	for (index = 0; index < sensor_config_count; ++index) {
		if (sensor_config[index].type == sensor_dev_ina233) {
			sensor_config[index].type = sensor_dev_sq52205;

			switch (sensor_config[index].offset) {
			case INA233_VOLT_OFFSET:
				sensor_config[index].offset = SQ5220X_VOL_OFFSET;
				break;
			case INA233_CURR_OFFSET:
				sensor_config[index].offset = SQ5220X_CUR_OFFSET;
				break;
			case INA233_PWR_OFFSET:
				sensor_config[index].offset = SQ5220X_PWR_OFFSET;
				break;
			default:
				LOG_ERR("UNKNOWN power monitor offset %x change table failed",
					sensor_config[index].offset);
				continue;
			}

			if (sensor_config[index].init_args == &ina233_init_args[0]) {
				sensor_config[index].init_args = &sq52205_init_args[0];
			} else if (sensor_config[index].init_args == &ina233_init_args[1]) {
				sensor_config[index].init_args = &sq52205_init_args[1];
			} else if (sensor_config[index].init_args == &ina233_init_args[2]) {
				sensor_config[index].init_args = &sq52205_init_args[2];
			} else if (sensor_config[index].init_args == &ina233_init_args[3]) {
				sensor_config[index].init_args = &sq52205_init_args[3];
			} else if (sensor_config[index].init_args == &ina233_init_args[4]) {
				sensor_config[index].init_args = &sq52205_init_args[4];
			} else if (sensor_config[index].init_args == &ina233_init_args[5]) {
				sensor_config[index].init_args = &sq52205_init_args[5];
			} else {
				LOG_ERR("UNKNOWN init args change table failed");
				continue;
			}
		}
	}
}

void pal_change_sensor_config_number()
{
	uint8_t card_position = get_card_position();
	uint8_t index;
	//Change sensor number of plat_sensor_config
	if (card_position == CARD_POSITION_2OU || card_position == CARD_POSITION_3OU ||
	    card_position == CARD_POSITION_4OU) {
		for (index = 0; index < ARRAY_SIZE(plat_sensor_config); index++) {
			plat_sensor_config[index].num += (card_position * SENSOR_NUMBER_INTERVAL);
		}
	}
	//Change sensor number of plat_expansion_sensor_config
	switch (card_position) {
	case CARD_POSITION_1OU:
	case CARD_POSITION_2OU:
		break;

	case CARD_POSITION_3OU:
		for (index = 0; index < ARRAY_SIZE(plat_expansion_A_sensor_config); index++) {
			plat_expansion_A_sensor_config[index].num +=
				((CARD_POSITION_3OU - CARD_POSITION_1OU) * SENSOR_NUMBER_INTERVAL);
		}
		break;
	case CARD_POSITION_4OU:
		for (index = 0; index < ARRAY_SIZE(plat_expansion_B_sensor_config); index++) {
			plat_expansion_B_sensor_config[index].num +=
				((CARD_POSITION_4OU - CARD_POSITION_2OU) * SENSOR_NUMBER_INTERVAL);
		}
		break;

	default:
		LOG_ERR("Can't identify board's position");
	}
}

void pal_extend_sensor_config()
{
	uint8_t sensor_count = 0;
	uint8_t card_type = get_card_type();
	uint8_t board_revision = get_board_revision();
	uint8_t card_position = get_card_position();
	uint8_t sensor_number_p1v8_adc = SENSOR_NUM_1OU_P1V8_ADC_VOLT;
	uint8_t sensor_number_p1v2_adc = SENSOR_NUM_1OU_P1V2_ADC_VOLT;
	if (card_position == CARD_POSITION_3OU) {
		sensor_number_p1v8_adc +=
			((CARD_POSITION_3OU - CARD_POSITION_1OU) * SENSOR_NUMBER_INTERVAL);
		sensor_number_p1v2_adc +=
			((CARD_POSITION_3OU - CARD_POSITION_1OU) * SENSOR_NUMBER_INTERVAL);
	}

	switch (card_type) {
	case CARD_TYPE_OPA:
		sensor_count = ARRAY_SIZE(plat_expansion_A_sensor_config);
		for (int index = 0; index < sensor_count; index++) {
			if (board_revision != EVT_STAGE) {
				if (sensor_config[index].num == sensor_number_p1v8_adc) {
					sensor_config[index].port = ADC_PORT11;
				}
				if (sensor_config[index].num == sensor_number_p1v2_adc) {
					sensor_config[index].port = ADC_PORT14;
				}
			}
			add_sensor_config(plat_expansion_A_sensor_config[index]);
		}
		break;

	case CARD_TYPE_OPB:
		sensor_count = ARRAY_SIZE(plat_expansion_B_sensor_config);
		for (int index = 0; index < sensor_count; index++) {
			add_sensor_config(plat_expansion_B_sensor_config[index]);
		}
		break;

	default:
		LOG_ERR("Unsupported card type, Card type: 0x%x", card_type);
		break;
	}
}
bool retimer_access(uint8_t sensor_num)
{
	return is_retimer_done();
}

bool e1s_access(uint8_t sensor_num)
{
	sensor_cfg *sensor_cfgs = &sensor_config[sensor_config_index_map[sensor_num]];
	uint8_t e1s_index = 0xff;
	uint8_t card_type = get_card_type();
	i2c_proc_arg *nvme_i2c_mux = NULL;
	int i = 0;

	switch (sensor_cfgs->type) {
	case sensor_dev_ina233:
	case sensor_dev_sq52205:
		if (card_type == CARD_TYPE_OPA) {
			for (i = 0; i < sizeof(ina233_addr_opa); i++) {
				if (ina233_addr_opa[i] == sensor_cfgs->target_addr) {
					e1s_index = i;
					break;
				}
			}
		} else {
			for (i = 0; i < sizeof(ina233_addr_opb); i++) {
				if (ina233_addr_opb[i] == sensor_cfgs->target_addr) {
					e1s_index = i;
					break;
				}
			}
		}
		break;
	case sensor_dev_nvme:
		nvme_i2c_mux = (i2c_proc_arg *)(sensor_cfgs->pre_sensor_read_args);
		/* For OPA expansion, the I3C HUB connect slave port-0/1/5.
		 * For OPB expansion, the I3C HUB connect slave port-0/1/2/3/4.
		 */
		if (card_type == CARD_TYPE_OPA) {
			for (i = 0; i < sizeof(e1s_mux_channel_opa); i++) {
				if (e1s_mux_channel_opa[i] == nvme_i2c_mux->channel) {
					e1s_index = i;
					break;
				}
			}
		} else {
			for (i = 0; i < sizeof(e1s_mux_channel_opb); i++) {
				if (e1s_mux_channel_opb[i] == nvme_i2c_mux->channel) {
					e1s_index = i;
					break;
				}
			}
		}
		break;
	case sensor_dev_ast_adc:
		if (card_type == CARD_TYPE_OPA) {
			for (i = 0; i < sizeof(e1s_adc_channel_opa); i++) {
				if (e1s_adc_channel_opa[i] == sensor_cfgs->port) {
					e1s_index = i;
					break;
				}
			}
		} else {
			for (i = 0; i < sizeof(e1s_adc_channel_opb); i++) {
				if (e1s_adc_channel_opb[i] == sensor_cfgs->port) {
					e1s_index = i;
					break;
				}
			}
		}
		break;
	default:
		LOG_ERR("Unsupported sensor device for e1s checking.");
		break;
	}
	return get_e1s_present(e1s_index) && get_e1s_power_good(e1s_index);
}

bool edge_access(uint8_t sensor_num)
{
	return get_DC_on_delayed_status() && get_edge_power_good();
}
