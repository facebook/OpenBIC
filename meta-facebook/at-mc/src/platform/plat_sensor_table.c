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
#include "ast_adc.h"
#include "sensor.h"
#include "hal_gpio.h"
#include "plat_i2c.h"
#include "plat_gpio.h"
#include "plat_hook.h"
#include "plat_class.h"
#include "pmbus.h"
#include "sq52205.h"
#include "power_status.h"

LOG_MODULE_REGISTER(plat_sensor_table);

struct k_mutex i2c_2_pca9548a_mutex;
struct k_mutex i2c_3_pca9546a_mutex;
struct k_mutex i2c_4_pca9548a_mutex;
struct k_mutex i2c_8_pca9548a_mutex;

LOG_MODULE_REGISTER(plat_sensor_table);

struct k_mutex i2c_2_pca9548a_mutex;
struct k_mutex i2c_3_pca9546a_mutex;
struct k_mutex i2c_4_pca9546a_mutex;
struct k_mutex i2c_8_pca9548a_mutex;

sensor_cfg plat_sensor_config[] = {
	/* number,                  type,       port,      address,      offset,
	   access check arg0, arg1, sample_count, cache, cache_status, mux_ADDRess, mux_offset,
	   pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_fn  */

	/** Temperature **/
	{ SENSOR_NUM_TEMP_TMP75_IN, sensor_dev_tmp75, I2C_BUS1, TMP75_IN_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	/** NVME **/
	{ SENSOR_NUM_TEMP_E1S_0, sensor_dev_nvme, I2C_BUS4, E1S_ADDR, E1S_OFFSET, is_e1s_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_nvme_read, &bus_4_pca9548_configs[0], post_nvme_read, NULL, NULL },
	{ SENSOR_NUM_TEMP_E1S_1, sensor_dev_nvme, I2C_BUS4, E1S_ADDR, E1S_OFFSET, is_e1s_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_nvme_read, &bus_4_pca9548_configs[1], post_nvme_read, NULL, NULL },
	{ SENSOR_NUM_TEMP_E1S_2, sensor_dev_nvme, I2C_BUS4, E1S_ADDR, E1S_OFFSET, is_e1s_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_nvme_read, &bus_4_pca9548_configs[2], post_nvme_read, NULL, NULL },
	{ SENSOR_NUM_TEMP_E1S_3, sensor_dev_nvme, I2C_BUS4, E1S_ADDR, E1S_OFFSET, is_e1s_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_nvme_read, &bus_4_pca9548_configs[3], post_nvme_read, NULL, NULL },

	/** HSC **/
	{ SENSOR_NUM_TEMP_PU4, sensor_dev_mp5990, I2C_BUS6, MPS_MP5990_ADDR,
	  PMBUS_READ_TEMPERATURE_1, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &mp5990_init_args[0] },
	{ SENSOR_NUM_VOL_P12V_AUX, sensor_dev_mp5990, I2C_BUS6, MPS_MP5990_ADDR, PMBUS_READ_VOUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &mp5990_init_args[0] },
	{ SENSOR_NUM_CUR_P12V_AUX, sensor_dev_mp5990, I2C_BUS6, MPS_MP5990_ADDR, PMBUS_READ_IOUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &mp5990_init_args[0] },
	{ SENSOR_NUM_PWR_P12V_AUX, sensor_dev_mp5990, I2C_BUS6, MPS_MP5990_ADDR, PMBUS_READ_PIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &mp5990_init_args[0] },

	/** ADC **/
	{ SENSOR_NUM_VOL_P3V3_AUX, sensor_dev_ast_adc, ADC_PORT0, NONE, NONE, stby_access, 2, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_P1V2_AUX, sensor_dev_ast_adc, ADC_PORT1, NONE, NONE, stby_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_P3V3, sensor_dev_ast_adc, ADC_PORT2, NONE, NONE, stby_access, 2, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },

	/** SQ52205_01 **/
	{ SENSOR_NUM_VOL_P12V_AUX_CARD01, sensor_dev_sq52205, I2C_BUS3, SQ52205_1_ADDR,
	  SQ52205_READ_VOL_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[0],
	  post_sq52205_read, NULL, &sq52205_init_args[0] },
	{ SENSOR_NUM_CUR_P12V_AUX_CARD01, sensor_dev_sq52205, I2C_BUS3, SQ52205_1_ADDR,
	  SQ52205_READ_CUR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[0],
	  post_sq52205_read, NULL, &sq52205_init_args[0] },
	{ SENSOR_NUM_PWR_P12V_AUX_CARD01, sensor_dev_sq52205, I2C_BUS3, SQ52205_1_ADDR,
	  SQ52205_READ_PWR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[0],
	  post_sq52205_read, NULL, &sq52205_init_args[0] },

	/** SQ52205_02 **/
	{ SENSOR_NUM_VOL_P12V_AUX_CARD02, sensor_dev_sq52205, I2C_BUS3, SQ52205_2_ADDR,
	  SQ52205_READ_VOL_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[0],
	  post_sq52205_read, NULL, &sq52205_init_args[1] },
	{ SENSOR_NUM_CUR_P12V_AUX_CARD02, sensor_dev_sq52205, I2C_BUS3, SQ52205_2_ADDR,
	  SQ52205_READ_CUR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[0],
	  post_sq52205_read, NULL, &sq52205_init_args[1] },
	{ SENSOR_NUM_PWR_P12V_AUX_CARD02, sensor_dev_sq52205, I2C_BUS3, SQ52205_2_ADDR,
	  SQ52205_READ_PWR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[0],
	  post_sq52205_read, NULL, &sq52205_init_args[1] },

	/** SQ52205_03 **/
	{ SENSOR_NUM_VOL_P12V_AUX_CARD03, sensor_dev_sq52205, I2C_BUS3, SQ52205_3_ADDR,
	  SQ52205_READ_VOL_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[0],
	  post_sq52205_read, NULL, &sq52205_init_args[2] },
	{ SENSOR_NUM_CUR_P12V_AUX_CARD03, sensor_dev_sq52205, I2C_BUS3, SQ52205_3_ADDR,
	  SQ52205_READ_CUR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[0],
	  post_sq52205_read, NULL, &sq52205_init_args[2] },
	{ SENSOR_NUM_PWR_P12V_AUX_CARD03, sensor_dev_sq52205, I2C_BUS3, SQ52205_3_ADDR,
	  SQ52205_READ_PWR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[0],
	  post_sq52205_read, NULL, &sq52205_init_args[2] },

	/** SQ52205_04 **/
	{ SENSOR_NUM_VOL_P12V_AUX_CARD04, sensor_dev_sq52205, I2C_BUS3, SQ52205_4_ADDR,
	  SQ52205_READ_VOL_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[0],
	  post_sq52205_read, NULL, &sq52205_init_args[3] },
	{ SENSOR_NUM_CUR_P12V_AUX_CARD04, sensor_dev_sq52205, I2C_BUS3, SQ52205_4_ADDR,
	  SQ52205_READ_CUR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[0],
	  post_sq52205_read, NULL, &sq52205_init_args[3] },
	{ SENSOR_NUM_PWR_P12V_AUX_CARD04, sensor_dev_sq52205, I2C_BUS3, SQ52205_4_ADDR,
	  SQ52205_READ_PWR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[0],
	  post_sq52205_read, NULL, &sq52205_init_args[3] },

	/** SQ52205_05 **/
	{ SENSOR_NUM_VOL_P12V_AUX_CARD05, sensor_dev_sq52205, I2C_BUS3, SQ52205_1_ADDR,
	  SQ52205_READ_VOL_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[1],
	  post_sq52205_read, NULL, &sq52205_init_args[4] },
	{ SENSOR_NUM_CUR_P12V_AUX_CARD05, sensor_dev_sq52205, I2C_BUS3, SQ52205_1_ADDR,
	  SQ52205_READ_CUR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[1],
	  post_sq52205_read, NULL, &sq52205_init_args[4] },
	{ SENSOR_NUM_PWR_P12V_AUX_CARD05, sensor_dev_sq52205, I2C_BUS3, SQ52205_1_ADDR,
	  SQ52205_READ_PWR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[1],
	  post_sq52205_read, NULL, &sq52205_init_args[4] },

	/** SQ52205_06 **/
	{ SENSOR_NUM_VOL_P12V_AUX_CARD06, sensor_dev_sq52205, I2C_BUS3, SQ52205_2_ADDR,
	  SQ52205_READ_VOL_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[1],
	  post_sq52205_read, NULL, &sq52205_init_args[5] },
	{ SENSOR_NUM_CUR_P12V_AUX_CARD06, sensor_dev_sq52205, I2C_BUS3, SQ52205_2_ADDR,
	  SQ52205_READ_CUR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[1],
	  post_sq52205_read, NULL, &sq52205_init_args[5] },
	{ SENSOR_NUM_PWR_P12V_AUX_CARD06, sensor_dev_sq52205, I2C_BUS3, SQ52205_2_ADDR,
	  SQ52205_READ_PWR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[1],
	  post_sq52205_read, NULL, &sq52205_init_args[5] },

	/** SQ52205_07 **/
	{ SENSOR_NUM_VOL_P12V_AUX_CARD07, sensor_dev_sq52205, I2C_BUS3, SQ52205_3_ADDR,
	  SQ52205_READ_VOL_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[1],
	  post_sq52205_read, NULL, &sq52205_init_args[6] },
	{ SENSOR_NUM_CUR_P12V_AUX_CARD07, sensor_dev_sq52205, I2C_BUS3, SQ52205_3_ADDR,
	  SQ52205_READ_CUR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[1],
	  post_sq52205_read, NULL, &sq52205_init_args[6] },
	{ SENSOR_NUM_PWR_P12V_AUX_CARD07, sensor_dev_sq52205, I2C_BUS3, SQ52205_3_ADDR,
	  SQ52205_READ_PWR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[1],
	  post_sq52205_read, NULL, &sq52205_init_args[6] },

	/** SQ52205_08 **/
	{ SENSOR_NUM_VOL_P12V_AUX_CARD08, sensor_dev_sq52205, I2C_BUS3, SQ52205_4_ADDR,
	  SQ52205_READ_VOL_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[1],
	  post_sq52205_read, NULL, &sq52205_init_args[7] },
	{ SENSOR_NUM_CUR_P12V_AUX_CARD08, sensor_dev_sq52205, I2C_BUS3, SQ52205_4_ADDR,
	  SQ52205_READ_CUR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[1],
	  post_sq52205_read, NULL, &sq52205_init_args[7] },
	{ SENSOR_NUM_PWR_P12V_AUX_CARD08, sensor_dev_sq52205, I2C_BUS3, SQ52205_4_ADDR,
	  SQ52205_READ_PWR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[1],
	  post_sq52205_read, NULL, &sq52205_init_args[7] },

	/** SQ52205_09 **/
	{ SENSOR_NUM_VOL_P12V_AUX_CARD09, sensor_dev_sq52205, I2C_BUS3, SQ52205_1_ADDR,
	  SQ52205_READ_VOL_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[2],
	  post_sq52205_read, NULL, &sq52205_init_args[8] },
	{ SENSOR_NUM_CUR_P12V_AUX_CARD09, sensor_dev_sq52205, I2C_BUS3, SQ52205_1_ADDR,
	  SQ52205_READ_CUR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[2],
	  post_sq52205_read, NULL, &sq52205_init_args[8] },
	{ SENSOR_NUM_PWR_P12V_AUX_CARD09, sensor_dev_sq52205, I2C_BUS3, SQ52205_1_ADDR,
	  SQ52205_READ_PWR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[2],
	  post_sq52205_read, NULL, &sq52205_init_args[8] },

	/** SQ52205_10 **/
	{ SENSOR_NUM_VOL_P12V_AUX_CARD10, sensor_dev_sq52205, I2C_BUS3, SQ52205_2_ADDR,
	  SQ52205_READ_VOL_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[2],
	  post_sq52205_read, NULL, &sq52205_init_args[9] },
	{ SENSOR_NUM_CUR_P12V_AUX_CARD10, sensor_dev_sq52205, I2C_BUS3, SQ52205_2_ADDR,
	  SQ52205_READ_CUR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[2],
	  post_sq52205_read, NULL, &sq52205_init_args[9] },
	{ SENSOR_NUM_PWR_P12V_AUX_CARD10, sensor_dev_sq52205, I2C_BUS3, SQ52205_2_ADDR,
	  SQ52205_READ_PWR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[2],
	  post_sq52205_read, NULL, &sq52205_init_args[9] },

	/** SQ52205_11 **/
	{ SENSOR_NUM_VOL_P12V_AUX_CARD11, sensor_dev_sq52205, I2C_BUS3, SQ52205_3_ADDR,
	  SQ52205_READ_VOL_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[2],
	  post_sq52205_read, NULL, &sq52205_init_args[10] },
	{ SENSOR_NUM_CUR_P12V_AUX_CARD11, sensor_dev_sq52205, I2C_BUS3, SQ52205_3_ADDR,
	  SQ52205_READ_CUR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[2],
	  post_sq52205_read, NULL, &sq52205_init_args[10] },
	{ SENSOR_NUM_PWR_P12V_AUX_CARD11, sensor_dev_sq52205, I2C_BUS3, SQ52205_3_ADDR,
	  SQ52205_READ_PWR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[2],
	  post_sq52205_read, NULL, &sq52205_init_args[10] },

	/** SQ52205_12 **/
	{ SENSOR_NUM_VOL_P12V_AUX_CARD12, sensor_dev_sq52205, I2C_BUS3, SQ52205_4_ADDR,
	  SQ52205_READ_VOL_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[2],
	  post_sq52205_read, NULL, &sq52205_init_args[11] },
	{ SENSOR_NUM_CUR_P12V_AUX_CARD12, sensor_dev_sq52205, I2C_BUS3, SQ52205_4_ADDR,
	  SQ52205_READ_CUR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[2],
	  post_sq52205_read, NULL, &sq52205_init_args[11] },
	{ SENSOR_NUM_PWR_P12V_AUX_CARD12, sensor_dev_sq52205, I2C_BUS3, SQ52205_4_ADDR,
	  SQ52205_READ_PWR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[2],
	  post_sq52205_read, NULL, &sq52205_init_args[11] },

	/** SQ52205_13 **/
	{ SENSOR_NUM_VOL_P12V_AUX_CARD13, sensor_dev_sq52205, I2C_BUS3, SQ52205_3_ADDR,
	  SQ52205_READ_VOL_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[3],
	  post_sq52205_read, NULL, &sq52205_init_args[12] },
	{ SENSOR_NUM_CUR_P12V_AUX_CARD13, sensor_dev_sq52205, I2C_BUS3, SQ52205_3_ADDR,
	  SQ52205_READ_CUR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[3],
	  post_sq52205_read, NULL, &sq52205_init_args[12] },
	{ SENSOR_NUM_PWR_P12V_AUX_CARD13, sensor_dev_sq52205, I2C_BUS3, SQ52205_3_ADDR,
	  SQ52205_READ_PWR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[3],
	  post_sq52205_read, NULL, &sq52205_init_args[12] },

	/** SQ52205_14 **/
	{ SENSOR_NUM_VOL_P12V_AUX_CARD14, sensor_dev_sq52205, I2C_BUS3, SQ52205_4_ADDR,
	  SQ52205_READ_VOL_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[3],
	  post_sq52205_read, NULL, &sq52205_init_args[13] },
	{ SENSOR_NUM_CUR_P12V_AUX_CARD14, sensor_dev_sq52205, I2C_BUS3, SQ52205_4_ADDR,
	  SQ52205_READ_CUR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[3],
	  post_sq52205_read, NULL, &sq52205_init_args[13] },
	{ SENSOR_NUM_PWR_P12V_AUX_CARD14, sensor_dev_sq52205, I2C_BUS3, SQ52205_4_ADDR,
	  SQ52205_READ_PWR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_sq52205_read, &bus_3_pca9546_configs[3],
	  post_sq52205_read, NULL, &sq52205_init_args[13] },
};

const int SENSOR_CONFIG_SIZE = ARRAY_SIZE(plat_sensor_config);

void load_sensor_config(void)
{
	memcpy(sensor_config, plat_sensor_config, sizeof(plat_sensor_config));
	sensor_config_count = ARRAY_SIZE(plat_sensor_config);

	// Fix config table in different system/config
	pal_extend_sensor_config();
}

bool is_dc_access(uint8_t sensor_num)
{
	return get_DC_status();
}

bool is_e1s_access(uint8_t sensor_num)
{
	int ret = false;
	uint8_t card_id = 0;
	uint8_t card_type = 0;

	switch (sensor_num) {
	case SENSOR_NUM_TEMP_E1S_0:
		card_id = CARD_5_INDEX;
		break;
	case SENSOR_NUM_TEMP_E1S_1:
		card_id = CARD_6_INDEX;
		break;
	case SENSOR_NUM_TEMP_E1S_2:
		card_id = CARD_7_INDEX;
		break;
	case SENSOR_NUM_TEMP_E1S_3:
		card_id = CARD_8_INDEX;
		break;
	default:
		return false;
	}

	ret = get_pcie_card_type(card_id, &card_type);
	if (ret < 0) {
		return false;
	}

	if (card_type == CARD_NOT_PRESENT) {
		return false;
	}

	return true;
}

struct k_mutex *get_i2c_mux_mutex(uint8_t i2c_bus)
{
	struct k_mutex *mutex = NULL;

	switch (i2c_bus) {
	case I2C_BUS2:
		mutex = &i2c_2_pca9548a_mutex;
		break;
	case I2C_BUS3:
		mutex = &i2c_3_pca9546a_mutex;
		break;
	case I2C_BUS4:
		mutex = &i2c_4_pca9548a_mutex;
		break;
	case I2C_BUS8:
		mutex = &i2c_8_pca9548a_mutex;
		break;
	default:
		LOG_ERR("No support for i2c bus %d mutex", i2c_bus);
		break;
	}

	return mutex;
}
