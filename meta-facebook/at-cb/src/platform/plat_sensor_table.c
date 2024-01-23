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
#include "plat_dev.h"
#include "sq52205.h"
#include "nvme.h"
#include "plat_pldm_monitor.h"

LOG_MODULE_REGISTER(plat_sensor_table);

#define COMMON_SENSOR_MONITOR_INDEX 0

struct k_mutex i2c_4_pi4msd5v9542_mutex;
struct k_mutex i2c_7_accl_mutex;
struct k_mutex i2c_8_accl_mutex;

sensor_cfg plat_sensor_config[] = {
	/* number,                  type,       port,      address,      offset,
	   access check arg0, arg1, sample_count, cache, cache_status, mux_ADDRess, mux_offset,
	   pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_fn  */

	/** Temperature **/
	{ SENSOR_NUM_TEMP_OUTLET_1, sensor_dev_tmp75, I2C_BUS2, TMP75_IN_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_OUTLET_2, sensor_dev_tmp75, I2C_BUS2, TMP75_OUT_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_FIO_INLET, sensor_dev_lm75bd118, I2C_BUS10, FIO_THERMAL_ADDR,
	  LM75BD118_TEMP_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

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
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[1] },
	{ SENSOR_NUM_VOL_P1V8_VDD_2, sensor_dev_ast_adc, ADC_PORT11, NONE, NONE, is_dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[1] },
	{ SENSOR_NUM_VOL_P1V25_VDD_1, sensor_dev_ast_adc, ADC_PORT12, NONE, NONE, is_dc_access, 1,
	  1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[1] },
	{ SENSOR_NUM_VOL_P1V25_VDD_2, sensor_dev_ast_adc, ADC_PORT13, NONE, NONE, is_dc_access, 1,
	  1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[1] },

	/** PEX temp **/
	{ SENSOR_NUM_TEMP_PEX_0, sensor_dev_pex89000, I2C_BUS2, PEX89144_I2CS_ADDR, PEX_TEMP,
	  is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_pex89000_read, &tca9543_configs[0], NULL, NULL,
	  &pex_sensor_init_args[0] },
	{ SENSOR_NUM_TEMP_PEX_1, sensor_dev_pex89000, I2C_BUS3, PEX89144_I2CS_ADDR, PEX_TEMP,
	  is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_pex89000_read, &tca9543_configs[1], NULL, NULL,
	  &pex_sensor_init_args[1] },

	/** ACCL card 1 **/
	{ SENSOR_NUM_TEMP_ACCL_1_FREYA_1, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_TEMP_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[0],
	  post_accl_nvme_read, &accl_card_info_args[0], NULL },
	{ SENSOR_NUM_TEMP_ACCL_1_FREYA_2, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_TEMP_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[0],
	  post_accl_nvme_read, &accl_card_info_args[0], NULL },
	{ SENSOR_NUM_VOL_ACCL_1_FREYA_1_1, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_CORE_VOLTAGE_1_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[0],
	  post_accl_nvme_read, &accl_card_info_args[0], NULL },
	{ SENSOR_NUM_VOL_ACCL_1_FREYA_1_2, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_CORE_VOLTAGE_2_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[0],
	  post_accl_nvme_read, &accl_card_info_args[0], NULL },
	{ SENSOR_NUM_VOL_ACCL_1_FREYA_2_1, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_CORE_VOLTAGE_1_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[0],
	  post_accl_nvme_read, &accl_card_info_args[0], NULL },
	{ SENSOR_NUM_VOL_ACCL_1_FREYA_2_2, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_CORE_VOLTAGE_2_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[0],
	  post_accl_nvme_read, &accl_card_info_args[0], NULL },
	{ SENSOR_NUM_VOL_ACCL_1_ASIC_1_P12V_AUX, sensor_dev_nvme, I2C_BUS8,
	  ACCL_ARTEMIS_MODULE_1_ADDR, NVME_VOLTAGE_RAIL_2_OFFSET, is_dc_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_accl_nvme_read, &accl_card_info_args[0], post_accl_nvme_read, &accl_card_info_args[0],
	  NULL },
	{ SENSOR_NUM_VOL_ACCL_1_ASIC_2_P12V_AUX, sensor_dev_nvme, I2C_BUS8,
	  ACCL_ARTEMIS_MODULE_2_ADDR, NVME_VOLTAGE_RAIL_2_OFFSET, is_dc_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_accl_nvme_read, &accl_card_info_args[0], post_accl_nvme_read, &accl_card_info_args[0],
	  NULL },

	/** ACCL card 2 **/
	{ SENSOR_NUM_TEMP_ACCL_2_FREYA_1, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_TEMP_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[1],
	  post_accl_nvme_read, &accl_card_info_args[1], NULL },
	{ SENSOR_NUM_TEMP_ACCL_2_FREYA_2, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_TEMP_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[1],
	  post_accl_nvme_read, &accl_card_info_args[1], NULL },
	{ SENSOR_NUM_VOL_ACCL_2_FREYA_1_1, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_CORE_VOLTAGE_1_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[1],
	  post_accl_nvme_read, &accl_card_info_args[1], NULL },
	{ SENSOR_NUM_VOL_ACCL_2_FREYA_1_2, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_CORE_VOLTAGE_2_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[1],
	  post_accl_nvme_read, &accl_card_info_args[1], NULL },
	{ SENSOR_NUM_VOL_ACCL_2_FREYA_2_1, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_CORE_VOLTAGE_1_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[1],
	  post_accl_nvme_read, &accl_card_info_args[1], NULL },
	{ SENSOR_NUM_VOL_ACCL_2_FREYA_2_2, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_CORE_VOLTAGE_2_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[1],
	  post_accl_nvme_read, &accl_card_info_args[1], NULL },
	{ SENSOR_NUM_VOL_ACCL_2_ASIC_1_P12V_AUX, sensor_dev_nvme, I2C_BUS8,
	  ACCL_ARTEMIS_MODULE_1_ADDR, NVME_VOLTAGE_RAIL_2_OFFSET, is_dc_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_accl_nvme_read, &accl_card_info_args[1], post_accl_nvme_read, &accl_card_info_args[1],
	  NULL },
	{ SENSOR_NUM_VOL_ACCL_2_ASIC_2_P12V_AUX, sensor_dev_nvme, I2C_BUS8,
	  ACCL_ARTEMIS_MODULE_2_ADDR, NVME_VOLTAGE_RAIL_2_OFFSET, is_dc_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_accl_nvme_read, &accl_card_info_args[1], post_accl_nvme_read, &accl_card_info_args[1],
	  NULL },

	/** ACCL card 3 **/
	{ SENSOR_NUM_TEMP_ACCL_3_FREYA_1, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_TEMP_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[2],
	  post_accl_nvme_read, &accl_card_info_args[2], NULL },
	{ SENSOR_NUM_TEMP_ACCL_3_FREYA_2, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_TEMP_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[2],
	  post_accl_nvme_read, &accl_card_info_args[2], NULL },
	{ SENSOR_NUM_VOL_ACCL_3_FREYA_1_1, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_CORE_VOLTAGE_1_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[2],
	  post_accl_nvme_read, &accl_card_info_args[2], NULL },
	{ SENSOR_NUM_VOL_ACCL_3_FREYA_1_2, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_CORE_VOLTAGE_2_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[2],
	  post_accl_nvme_read, &accl_card_info_args[2], NULL },
	{ SENSOR_NUM_VOL_ACCL_3_FREYA_2_1, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_CORE_VOLTAGE_1_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[2],
	  post_accl_nvme_read, &accl_card_info_args[2], NULL },
	{ SENSOR_NUM_VOL_ACCL_3_FREYA_2_2, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_CORE_VOLTAGE_2_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[2],
	  post_accl_nvme_read, &accl_card_info_args[2], NULL },
	{ SENSOR_NUM_VOL_ACCL_3_ASIC_1_P12V_AUX, sensor_dev_nvme, I2C_BUS8,
	  ACCL_ARTEMIS_MODULE_1_ADDR, NVME_VOLTAGE_RAIL_2_OFFSET, is_dc_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_accl_nvme_read, &accl_card_info_args[2], post_accl_nvme_read, &accl_card_info_args[2],
	  NULL },
	{ SENSOR_NUM_VOL_ACCL_3_ASIC_2_P12V_AUX, sensor_dev_nvme, I2C_BUS8,
	  ACCL_ARTEMIS_MODULE_2_ADDR, NVME_VOLTAGE_RAIL_2_OFFSET, is_dc_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_accl_nvme_read, &accl_card_info_args[2], post_accl_nvme_read, &accl_card_info_args[2],
	  NULL },

	/** ACCL card 4 **/
	{ SENSOR_NUM_TEMP_ACCL_4_FREYA_1, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_TEMP_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[3],
	  post_accl_nvme_read, &accl_card_info_args[3], NULL },
	{ SENSOR_NUM_TEMP_ACCL_4_FREYA_2, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_TEMP_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[3],
	  post_accl_nvme_read, &accl_card_info_args[3], NULL },
	{ SENSOR_NUM_VOL_ACCL_4_FREYA_1_1, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_CORE_VOLTAGE_1_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[3],
	  post_accl_nvme_read, &accl_card_info_args[3], NULL },
	{ SENSOR_NUM_VOL_ACCL_4_FREYA_1_2, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_CORE_VOLTAGE_2_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[3],
	  post_accl_nvme_read, &accl_card_info_args[3], NULL },
	{ SENSOR_NUM_VOL_ACCL_4_FREYA_2_1, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_CORE_VOLTAGE_1_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[3],
	  post_accl_nvme_read, &accl_card_info_args[3], NULL },
	{ SENSOR_NUM_VOL_ACCL_4_FREYA_2_2, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_CORE_VOLTAGE_2_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[3],
	  post_accl_nvme_read, &accl_card_info_args[3], NULL },
	{ SENSOR_NUM_VOL_ACCL_4_ASIC_1_P12V_AUX, sensor_dev_nvme, I2C_BUS8,
	  ACCL_ARTEMIS_MODULE_1_ADDR, NVME_VOLTAGE_RAIL_2_OFFSET, is_dc_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_accl_nvme_read, &accl_card_info_args[3], post_accl_nvme_read, &accl_card_info_args[3],
	  NULL },
	{ SENSOR_NUM_VOL_ACCL_4_ASIC_2_P12V_AUX, sensor_dev_nvme, I2C_BUS8,
	  ACCL_ARTEMIS_MODULE_2_ADDR, NVME_VOLTAGE_RAIL_2_OFFSET, is_dc_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_accl_nvme_read, &accl_card_info_args[3], post_accl_nvme_read, &accl_card_info_args[3],
	  NULL },

	/** ACCL card 5 **/
	{ SENSOR_NUM_TEMP_ACCL_5_FREYA_1, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_TEMP_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[4],
	  post_accl_nvme_read, &accl_card_info_args[4], NULL },
	{ SENSOR_NUM_TEMP_ACCL_5_FREYA_2, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_TEMP_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[4],
	  post_accl_nvme_read, &accl_card_info_args[4], NULL },
	{ SENSOR_NUM_VOL_ACCL_5_FREYA_1_1, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_CORE_VOLTAGE_1_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[4],
	  post_accl_nvme_read, &accl_card_info_args[4], NULL },
	{ SENSOR_NUM_VOL_ACCL_5_FREYA_1_2, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_CORE_VOLTAGE_2_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[4],
	  post_accl_nvme_read, &accl_card_info_args[4], NULL },
	{ SENSOR_NUM_VOL_ACCL_5_FREYA_2_1, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_CORE_VOLTAGE_1_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[4],
	  post_accl_nvme_read, &accl_card_info_args[4], NULL },
	{ SENSOR_NUM_VOL_ACCL_5_FREYA_2_2, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_CORE_VOLTAGE_2_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[4],
	  post_accl_nvme_read, &accl_card_info_args[4], NULL },
	{ SENSOR_NUM_VOL_ACCL_5_ASIC_1_P12V_AUX, sensor_dev_nvme, I2C_BUS8,
	  ACCL_ARTEMIS_MODULE_1_ADDR, NVME_VOLTAGE_RAIL_2_OFFSET, is_dc_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_accl_nvme_read, &accl_card_info_args[4], post_accl_nvme_read, &accl_card_info_args[4],
	  NULL },
	{ SENSOR_NUM_VOL_ACCL_5_ASIC_2_P12V_AUX, sensor_dev_nvme, I2C_BUS8,
	  ACCL_ARTEMIS_MODULE_2_ADDR, NVME_VOLTAGE_RAIL_2_OFFSET, is_dc_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_accl_nvme_read, &accl_card_info_args[4], post_accl_nvme_read, &accl_card_info_args[4],
	  NULL },

	/** ACCL card 6 **/
	{ SENSOR_NUM_TEMP_ACCL_6_FREYA_1, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_TEMP_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[5],
	  post_accl_nvme_read, &accl_card_info_args[5], NULL },
	{ SENSOR_NUM_TEMP_ACCL_6_FREYA_2, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_TEMP_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[5],
	  post_accl_nvme_read, &accl_card_info_args[5], NULL },
	{ SENSOR_NUM_VOL_ACCL_6_FREYA_1_1, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_CORE_VOLTAGE_1_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[5],
	  post_accl_nvme_read, &accl_card_info_args[5], NULL },
	{ SENSOR_NUM_VOL_ACCL_6_FREYA_1_2, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_CORE_VOLTAGE_2_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[5],
	  post_accl_nvme_read, &accl_card_info_args[5], NULL },
	{ SENSOR_NUM_VOL_ACCL_6_FREYA_2_1, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_CORE_VOLTAGE_1_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[5],
	  post_accl_nvme_read, &accl_card_info_args[5], NULL },
	{ SENSOR_NUM_VOL_ACCL_6_FREYA_2_2, sensor_dev_nvme, I2C_BUS8, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_CORE_VOLTAGE_2_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[5],
	  post_accl_nvme_read, &accl_card_info_args[5], NULL },
	{ SENSOR_NUM_VOL_ACCL_6_ASIC_1_P12V_AUX, sensor_dev_nvme, I2C_BUS8,
	  ACCL_ARTEMIS_MODULE_1_ADDR, NVME_VOLTAGE_RAIL_2_OFFSET, is_dc_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_accl_nvme_read, &accl_card_info_args[5], post_accl_nvme_read, &accl_card_info_args[5],
	  NULL },
	{ SENSOR_NUM_VOL_ACCL_6_ASIC_2_P12V_AUX, sensor_dev_nvme, I2C_BUS8,
	  ACCL_ARTEMIS_MODULE_2_ADDR, NVME_VOLTAGE_RAIL_2_OFFSET, is_dc_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_accl_nvme_read, &accl_card_info_args[5], post_accl_nvme_read, &accl_card_info_args[5],
	  NULL },

	{ SENSOR_NUM_TEMP_ACCL_7_FREYA_1, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_TEMP_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[6],
	  post_accl_nvme_read, &accl_card_info_args[6], NULL },
	{ SENSOR_NUM_TEMP_ACCL_7_FREYA_2, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_TEMP_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[6],
	  post_accl_nvme_read, &accl_card_info_args[6], NULL },
	{ SENSOR_NUM_VOL_ACCL_7_FREYA_1_1, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_CORE_VOLTAGE_1_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[6],
	  post_accl_nvme_read, &accl_card_info_args[6], NULL },
	{ SENSOR_NUM_VOL_ACCL_7_FREYA_1_2, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_CORE_VOLTAGE_2_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[6],
	  post_accl_nvme_read, &accl_card_info_args[6], NULL },
	{ SENSOR_NUM_VOL_ACCL_7_FREYA_2_1, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_CORE_VOLTAGE_1_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[6],
	  post_accl_nvme_read, &accl_card_info_args[6], NULL },
	{ SENSOR_NUM_VOL_ACCL_7_FREYA_2_2, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_CORE_VOLTAGE_2_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[6],
	  post_accl_nvme_read, &accl_card_info_args[6], NULL },
	{ SENSOR_NUM_VOL_ACCL_7_ASIC_1_P12V_AUX, sensor_dev_nvme, I2C_BUS7,
	  ACCL_ARTEMIS_MODULE_1_ADDR, NVME_VOLTAGE_RAIL_2_OFFSET, is_dc_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_accl_nvme_read, &accl_card_info_args[6], post_accl_nvme_read, &accl_card_info_args[6],
	  NULL },
	{ SENSOR_NUM_VOL_ACCL_7_ASIC_2_P12V_AUX, sensor_dev_nvme, I2C_BUS7,
	  ACCL_ARTEMIS_MODULE_2_ADDR, NVME_VOLTAGE_RAIL_2_OFFSET, is_dc_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_accl_nvme_read, &accl_card_info_args[6], post_accl_nvme_read, &accl_card_info_args[6],
	  NULL },

	/** ACCL card 8 **/
	{ SENSOR_NUM_TEMP_ACCL_8_FREYA_1, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_TEMP_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[7],
	  post_accl_nvme_read, &accl_card_info_args[7], NULL },
	{ SENSOR_NUM_TEMP_ACCL_8_FREYA_2, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_TEMP_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[7],
	  post_accl_nvme_read, &accl_card_info_args[7], NULL },
	{ SENSOR_NUM_VOL_ACCL_8_FREYA_1_1, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_CORE_VOLTAGE_1_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[7],
	  post_accl_nvme_read, &accl_card_info_args[7], NULL },
	{ SENSOR_NUM_VOL_ACCL_8_FREYA_1_2, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_CORE_VOLTAGE_2_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[7],
	  post_accl_nvme_read, &accl_card_info_args[7], NULL },
	{ SENSOR_NUM_VOL_ACCL_8_FREYA_2_1, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_CORE_VOLTAGE_1_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[7],
	  post_accl_nvme_read, &accl_card_info_args[7], NULL },
	{ SENSOR_NUM_VOL_ACCL_8_FREYA_2_2, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_CORE_VOLTAGE_2_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[7],
	  post_accl_nvme_read, &accl_card_info_args[7], NULL },
	{ SENSOR_NUM_VOL_ACCL_8_ASIC_1_P12V_AUX, sensor_dev_nvme, I2C_BUS7,
	  ACCL_ARTEMIS_MODULE_1_ADDR, NVME_VOLTAGE_RAIL_2_OFFSET, is_dc_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_accl_nvme_read, &accl_card_info_args[7], post_accl_nvme_read, &accl_card_info_args[7],
	  NULL },
	{ SENSOR_NUM_VOL_ACCL_8_ASIC_2_P12V_AUX, sensor_dev_nvme, I2C_BUS7,
	  ACCL_ARTEMIS_MODULE_2_ADDR, NVME_VOLTAGE_RAIL_2_OFFSET, is_dc_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_accl_nvme_read, &accl_card_info_args[7], post_accl_nvme_read, &accl_card_info_args[7],
	  NULL },

	/** ACCL card 9 **/
	{ SENSOR_NUM_TEMP_ACCL_9_FREYA_1, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_TEMP_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[8],
	  post_accl_nvme_read, &accl_card_info_args[8], NULL },
	{ SENSOR_NUM_TEMP_ACCL_9_FREYA_2, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_TEMP_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[8],
	  post_accl_nvme_read, &accl_card_info_args[8], NULL },
	{ SENSOR_NUM_VOL_ACCL_9_FREYA_1_1, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_CORE_VOLTAGE_1_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[8],
	  post_accl_nvme_read, &accl_card_info_args[8], NULL },
	{ SENSOR_NUM_VOL_ACCL_9_FREYA_1_2, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_CORE_VOLTAGE_2_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[8],
	  post_accl_nvme_read, &accl_card_info_args[8], NULL },
	{ SENSOR_NUM_VOL_ACCL_9_FREYA_2_1, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_CORE_VOLTAGE_1_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[8],
	  post_accl_nvme_read, &accl_card_info_args[8], NULL },
	{ SENSOR_NUM_VOL_ACCL_9_FREYA_2_2, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_CORE_VOLTAGE_2_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[8],
	  post_accl_nvme_read, &accl_card_info_args[8], NULL },
	{ SENSOR_NUM_VOL_ACCL_9_ASIC_1_P12V_AUX, sensor_dev_nvme, I2C_BUS7,
	  ACCL_ARTEMIS_MODULE_1_ADDR, NVME_VOLTAGE_RAIL_2_OFFSET, is_dc_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_accl_nvme_read, &accl_card_info_args[8], post_accl_nvme_read, &accl_card_info_args[8],
	  NULL },
	{ SENSOR_NUM_VOL_ACCL_9_ASIC_2_P12V_AUX, sensor_dev_nvme, I2C_BUS7,
	  ACCL_ARTEMIS_MODULE_2_ADDR, NVME_VOLTAGE_RAIL_2_OFFSET, is_dc_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_accl_nvme_read, &accl_card_info_args[8], post_accl_nvme_read, &accl_card_info_args[8],
	  NULL },

	/** ACCL card 10 **/
	{ SENSOR_NUM_TEMP_ACCL_10_FREYA_1, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_TEMP_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[9],
	  post_accl_nvme_read, &accl_card_info_args[9], NULL },
	{ SENSOR_NUM_TEMP_ACCL_10_FREYA_2, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_TEMP_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[9],
	  post_accl_nvme_read, &accl_card_info_args[9], NULL },
	{ SENSOR_NUM_VOL_ACCL_10_FREYA_1_1, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_CORE_VOLTAGE_1_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[9],
	  post_accl_nvme_read, &accl_card_info_args[9], NULL },
	{ SENSOR_NUM_VOL_ACCL_10_FREYA_1_2, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_CORE_VOLTAGE_2_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[9],
	  post_accl_nvme_read, &accl_card_info_args[9], NULL },
	{ SENSOR_NUM_VOL_ACCL_10_FREYA_2_1, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_CORE_VOLTAGE_1_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[9],
	  post_accl_nvme_read, &accl_card_info_args[9], NULL },
	{ SENSOR_NUM_VOL_ACCL_10_FREYA_2_2, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_CORE_VOLTAGE_2_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read, &accl_card_info_args[9],
	  post_accl_nvme_read, &accl_card_info_args[9], NULL },
	{ SENSOR_NUM_VOL_ACCL_10_ASIC_1_P12V_AUX, sensor_dev_nvme, I2C_BUS7,
	  ACCL_ARTEMIS_MODULE_1_ADDR, NVME_VOLTAGE_RAIL_2_OFFSET, is_dc_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_accl_nvme_read, &accl_card_info_args[9], post_accl_nvme_read, &accl_card_info_args[9],
	  NULL },
	{ SENSOR_NUM_VOL_ACCL_10_ASIC_2_P12V_AUX, sensor_dev_nvme, I2C_BUS7,
	  ACCL_ARTEMIS_MODULE_2_ADDR, NVME_VOLTAGE_RAIL_2_OFFSET, is_dc_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_accl_nvme_read, &accl_card_info_args[9], post_accl_nvme_read, &accl_card_info_args[9],
	  NULL },

	/** ACCL card 11 **/
	{ SENSOR_NUM_TEMP_ACCL_11_FREYA_1, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_TEMP_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read,
	  &accl_card_info_args[10], post_accl_nvme_read, &accl_card_info_args[10], NULL },
	{ SENSOR_NUM_TEMP_ACCL_11_FREYA_2, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_TEMP_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read,
	  &accl_card_info_args[10], post_accl_nvme_read, &accl_card_info_args[10], NULL },
	{ SENSOR_NUM_VOL_ACCL_11_FREYA_1_1, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_CORE_VOLTAGE_1_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read,
	  &accl_card_info_args[10], post_accl_nvme_read, &accl_card_info_args[10], NULL },
	{ SENSOR_NUM_VOL_ACCL_11_FREYA_1_2, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_CORE_VOLTAGE_2_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read,
	  &accl_card_info_args[10], post_accl_nvme_read, &accl_card_info_args[10], NULL },
	{ SENSOR_NUM_VOL_ACCL_11_FREYA_2_1, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_CORE_VOLTAGE_1_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read,
	  &accl_card_info_args[10], post_accl_nvme_read, &accl_card_info_args[10], NULL },
	{ SENSOR_NUM_VOL_ACCL_11_FREYA_2_2, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_CORE_VOLTAGE_2_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read,
	  &accl_card_info_args[10], post_accl_nvme_read, &accl_card_info_args[10], NULL },
	{ SENSOR_NUM_VOL_ACCL_11_ASIC_1_P12V_AUX, sensor_dev_nvme, I2C_BUS7,
	  ACCL_ARTEMIS_MODULE_1_ADDR, NVME_VOLTAGE_RAIL_2_OFFSET, is_dc_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_accl_nvme_read, &accl_card_info_args[10], post_accl_nvme_read,
	  &accl_card_info_args[10], NULL },
	{ SENSOR_NUM_VOL_ACCL_11_ASIC_2_P12V_AUX, sensor_dev_nvme, I2C_BUS7,
	  ACCL_ARTEMIS_MODULE_2_ADDR, NVME_VOLTAGE_RAIL_2_OFFSET, is_dc_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_accl_nvme_read, &accl_card_info_args[10], post_accl_nvme_read,
	  &accl_card_info_args[10], NULL },

	/** ACCL card 12 **/
	{ SENSOR_NUM_TEMP_ACCL_12_FREYA_1, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_TEMP_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read,
	  &accl_card_info_args[11], post_accl_nvme_read, &accl_card_info_args[11], NULL },
	{ SENSOR_NUM_TEMP_ACCL_12_FREYA_2, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_TEMP_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read,
	  &accl_card_info_args[11], post_accl_nvme_read, &accl_card_info_args[11], NULL },
	{ SENSOR_NUM_VOL_ACCL_12_FREYA_1_1, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_CORE_VOLTAGE_1_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read,
	  &accl_card_info_args[11], post_accl_nvme_read, &accl_card_info_args[11], NULL },
	{ SENSOR_NUM_VOL_ACCL_12_FREYA_1_2, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_1_ADDR,
	  NVME_CORE_VOLTAGE_2_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read,
	  &accl_card_info_args[11], post_accl_nvme_read, &accl_card_info_args[11], NULL },
	{ SENSOR_NUM_VOL_ACCL_12_FREYA_2_1, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_CORE_VOLTAGE_1_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read,
	  &accl_card_info_args[11], post_accl_nvme_read, &accl_card_info_args[11], NULL },
	{ SENSOR_NUM_VOL_ACCL_12_FREYA_2_2, sensor_dev_nvme, I2C_BUS7, ACCL_ARTEMIS_MODULE_2_ADDR,
	  NVME_CORE_VOLTAGE_2_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_accl_nvme_read,
	  &accl_card_info_args[11], post_accl_nvme_read, &accl_card_info_args[11], NULL },
	{ SENSOR_NUM_VOL_ACCL_12_ASIC_1_P12V_AUX, sensor_dev_nvme, I2C_BUS7,
	  ACCL_ARTEMIS_MODULE_1_ADDR, NVME_VOLTAGE_RAIL_2_OFFSET, is_dc_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_accl_nvme_read, &accl_card_info_args[11], post_accl_nvme_read,
	  &accl_card_info_args[11], NULL },
	{ SENSOR_NUM_VOL_ACCL_12_ASIC_2_P12V_AUX, sensor_dev_nvme, I2C_BUS7,
	  ACCL_ARTEMIS_MODULE_2_ADDR, NVME_VOLTAGE_RAIL_2_OFFSET, is_dc_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_accl_nvme_read, &accl_card_info_args[11], post_accl_nvme_read,
	  &accl_card_info_args[11], NULL },

};

sensor_cfg adm1272_sensor_config_table[] = {
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
	  SENSOR_INIT_STATUS, NULL, NULL, post_adm1272_read, NULL, &adm1272_init_args[0] },
	{ SENSOR_NUM_CUR_P51V_AUX_L, sensor_dev_adm1272, I2C_BUS1, ADM1272_1_ADDR, PMBUS_READ_IOUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, post_adm1272_read, NULL, &adm1272_init_args[0] },
	{ SENSOR_NUM_PWR_P51V_STBY_L, sensor_dev_adm1272, I2C_BUS1, ADM1272_1_ADDR, PMBUS_READ_EIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, post_adm1272_read, NULL, &adm1272_init_args[0] },
	{ SENSOR_NUM_PWR_P51V_AUX_L, sensor_dev_adm1272, I2C_BUS1, ADM1272_1_ADDR, PMBUS_READ_POUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, post_adm1272_read, NULL, &adm1272_init_args[0] },

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
	  SENSOR_INIT_STATUS, NULL, NULL, post_adm1272_read, NULL, &adm1272_init_args[1] },
	{ SENSOR_NUM_CUR_P51V_AUX_R, sensor_dev_adm1272, I2C_BUS1, ADM1272_2_ADDR, PMBUS_READ_IOUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, post_adm1272_read, NULL, &adm1272_init_args[1] },
	{ SENSOR_NUM_PWR_P51V_STBY_R, sensor_dev_adm1272, I2C_BUS1, ADM1272_2_ADDR, PMBUS_READ_EIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, post_adm1272_read, NULL, &adm1272_init_args[1] },
	{ SENSOR_NUM_PWR_P51V_AUX_R, sensor_dev_adm1272, I2C_BUS1, ADM1272_2_ADDR, PMBUS_READ_POUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, post_adm1272_read, NULL, &adm1272_init_args[1] },
};

sensor_cfg ltc4286_sensor_config_table[] = {
	{ SENSOR_NUM_TEMP_HSC_1, sensor_dev_ltc4286, I2C_BUS1, LTC4286_1_ADDR,
	  PMBUS_READ_TEMPERATURE_1, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ltc4286_init_args[0] },
	{ SENSOR_NUM_VOL_P51V_STBY_L, sensor_dev_ltc4286, I2C_BUS1, LTC4286_1_ADDR, PMBUS_READ_VIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ltc4286_init_args[0] },
	{ SENSOR_NUM_VOL_P51V_AUX_L, sensor_dev_ltc4286, I2C_BUS1, LTC4286_1_ADDR, PMBUS_READ_VOUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ltc4286_init_args[0] },
	{ SENSOR_NUM_CUR_P51V_STBY_L, sensor_dev_ltc4286, I2C_BUS1, LTC4286_1_ADDR, PMBUS_READ_IIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ltc4286_init_args[0] },
	{ SENSOR_NUM_CUR_P51V_AUX_L, sensor_dev_ltc4286, I2C_BUS1, LTC4286_1_ADDR, PMBUS_READ_IOUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ltc4286_init_args[0] },
	{ SENSOR_NUM_PWR_P51V_STBY_L, sensor_dev_ltc4286, I2C_BUS1, LTC4286_1_ADDR, PMBUS_READ_PIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ltc4286_init_args[0] },
	{ SENSOR_NUM_PWR_P51V_AUX_L, sensor_dev_ltc4286, I2C_BUS1, LTC4286_1_ADDR, PMBUS_READ_POUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ltc4286_init_args[0] },

	{ SENSOR_NUM_TEMP_HSC_2, sensor_dev_ltc4286, I2C_BUS1, LTC4286_2_ADDR,
	  PMBUS_READ_TEMPERATURE_1, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ltc4286_init_args[1] },
	{ SENSOR_NUM_VOL_P51V_STBY_R, sensor_dev_ltc4286, I2C_BUS1, LTC4286_2_ADDR, PMBUS_READ_VIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ltc4286_init_args[1] },
	{ SENSOR_NUM_VOL_P51V_AUX_R, sensor_dev_ltc4286, I2C_BUS1, LTC4286_2_ADDR, PMBUS_READ_VOUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ltc4286_init_args[1] },
	{ SENSOR_NUM_CUR_P51V_STBY_R, sensor_dev_ltc4286, I2C_BUS1, LTC4286_2_ADDR, PMBUS_READ_IIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ltc4286_init_args[1] },
	{ SENSOR_NUM_CUR_P51V_AUX_R, sensor_dev_ltc4286, I2C_BUS1, LTC4286_2_ADDR, PMBUS_READ_IOUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ltc4286_init_args[1] },
	{ SENSOR_NUM_PWR_P51V_STBY_R, sensor_dev_ltc4286, I2C_BUS1, LTC4286_2_ADDR, PMBUS_READ_PIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ltc4286_init_args[1] },
	{ SENSOR_NUM_PWR_P51V_AUX_R, sensor_dev_ltc4286, I2C_BUS1, LTC4286_2_ADDR, PMBUS_READ_POUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ltc4286_init_args[1] },
};

sensor_cfg q50sn120a1_sensor_config_table[] = {
	{ SENSOR_NUM_TEMP_POWER_BRICK_1, sensor_dev_q50sn120a1, I2C_BUS1, POWER_BRICK_1_ADDR,
	  PMBUS_READ_TEMPERATURE_1, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_P12V_AUX_1, sensor_dev_q50sn120a1, I2C_BUS1, POWER_BRICK_1_ADDR,
	  PMBUS_READ_VOUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_P12V_AUX_1, sensor_dev_q50sn120a1, I2C_BUS1, POWER_BRICK_1_ADDR,
	  PMBUS_READ_IOUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_P12V_AUX_1, sensor_dev_q50sn120a1, I2C_BUS1, POWER_BRICK_1_ADDR,
	  PMBUS_READ_POUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	{ SENSOR_NUM_TEMP_POWER_BRICK_2, sensor_dev_q50sn120a1, I2C_BUS1, POWER_BRICK_2_ADDR,
	  PMBUS_READ_TEMPERATURE_1, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_P12V_AUX_2, sensor_dev_q50sn120a1, I2C_BUS1, POWER_BRICK_2_ADDR,
	  PMBUS_READ_VOUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_P12V_AUX_2, sensor_dev_q50sn120a1, I2C_BUS1, POWER_BRICK_2_ADDR,
	  PMBUS_READ_IOUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_P12V_AUX_2, sensor_dev_q50sn120a1, I2C_BUS1, POWER_BRICK_2_ADDR,
	  PMBUS_READ_POUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
};

sensor_cfg bmr351_sensor_config_table[] = {
	{ SENSOR_NUM_TEMP_POWER_BRICK_1, sensor_dev_bmr351, I2C_BUS1, POWER_BRICK_1_ADDR,
	  PMBUS_READ_TEMPERATURE_1, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_P12V_AUX_1, sensor_dev_bmr351, I2C_BUS1, POWER_BRICK_1_ADDR,
	  PMBUS_READ_VOUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_P12V_AUX_1, sensor_dev_bmr351, I2C_BUS1, POWER_BRICK_1_ADDR,
	  PMBUS_READ_IOUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_P12V_AUX_1, sensor_dev_bmr351, I2C_BUS1, POWER_BRICK_1_ADDR,
	  PMBUS_READ_POUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	{ SENSOR_NUM_TEMP_POWER_BRICK_2, sensor_dev_bmr351, I2C_BUS1, POWER_BRICK_2_ADDR,
	  PMBUS_READ_TEMPERATURE_1, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_P12V_AUX_2, sensor_dev_bmr351, I2C_BUS1, POWER_BRICK_2_ADDR,
	  PMBUS_READ_VOUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_P12V_AUX_2, sensor_dev_bmr351, I2C_BUS1, POWER_BRICK_2_ADDR,
	  PMBUS_READ_IOUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_P12V_AUX_2, sensor_dev_bmr351, I2C_BUS1, POWER_BRICK_2_ADDR,
	  PMBUS_READ_POUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
};

sensor_cfg vr_xdpe15284_sensor_config_table[] = {
	{ SENSOR_NUM_TEMP_P0V8_VDD_1, sensor_dev_xdpe15284, I2C_BUS1, XDPE15284D_ADDR,
	  PMBUS_READ_TEMPERATURE_1, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_page[0],
	  post_xdpe15284_read, NULL, NULL },
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

	{ SENSOR_NUM_TEMP_P0V8_VDD_2, sensor_dev_xdpe15284, I2C_BUS1, XDPE15284D_ADDR,
	  PMBUS_READ_TEMPERATURE_1, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_page[1],
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
};

sensor_cfg vr_mp2985h_sensor_config_table[] = {
	{ SENSOR_NUM_TEMP_P0V8_VDD_1, sensor_dev_mp2985, I2C_BUS1, XDPE15284D_ADDR,
	  PMBUS_READ_TEMPERATURE_1, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_page[0],
	  post_xdpe15284_read, NULL, &mp2985_init_args[0] },
	{ SENSOR_NUM_VOL_P0V8_VDD_1, sensor_dev_mp2985, I2C_BUS1, XDPE15284D_ADDR, PMBUS_READ_VOUT,
	  is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_page[0], post_xdpe15284_read, NULL,
	  &mp2985_init_args[0] },
	{ SENSOR_NUM_CUR_P0V8_VDD_1, sensor_dev_mp2985, I2C_BUS1, XDPE15284D_ADDR, PMBUS_READ_IOUT,
	  is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_page[0], post_xdpe15284_read, NULL,
	  &mp2985_init_args[0] },
	{ SENSOR_NUM_PWR_P0V8_VDD_1, sensor_dev_mp2985, I2C_BUS1, XDPE15284D_ADDR, PMBUS_READ_POUT,
	  is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_page[0], post_xdpe15284_read, NULL,
	  &mp2985_init_args[0] },

	{ SENSOR_NUM_TEMP_P0V8_VDD_2, sensor_dev_mp2985, I2C_BUS1, XDPE15284D_ADDR,
	  PMBUS_READ_TEMPERATURE_1, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_page[1],
	  post_xdpe15284_read, NULL, &mp2985_init_args[0] },
	{ SENSOR_NUM_VOL_P0V8_VDD_2, sensor_dev_mp2985, I2C_BUS1, XDPE15284D_ADDR, PMBUS_READ_VOUT,
	  is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_page[1], post_xdpe15284_read, NULL,
	  &mp2985_init_args[0] },
	{ SENSOR_NUM_CUR_P0V8_VDD_2, sensor_dev_mp2985, I2C_BUS1, XDPE15284D_ADDR, PMBUS_READ_IOUT,
	  is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_page[1], post_xdpe15284_read, NULL,
	  &mp2985_init_args[0] },
	{ SENSOR_NUM_PWR_P0V8_VDD_2, sensor_dev_mp2985, I2C_BUS1, XDPE15284D_ADDR, PMBUS_READ_POUT,
	  is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_page[1], post_xdpe15284_read, NULL,
	  &mp2985_init_args[0] },
};

sensor_cfg power_monitor_ina233_sq52205_sensor_config_table[] = {
	/** INA233 12V 1 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_1, sensor_dev_ina233, I2C_BUS9, INA233_12V_1_7_ADDR,
	  PMBUS_READ_VOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[0],
	  post_ina233_read, &pwr_monitor_args[0], &accl_pwr_monitor_ina233_init_args[0] },
	{ SENSOR_NUM_CUR_P12V_ACCL_1, sensor_dev_ina233, I2C_BUS9, INA233_12V_1_7_ADDR,
	  PMBUS_READ_IOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[0],
	  post_ina233_read, &pwr_monitor_args[0], &accl_pwr_monitor_ina233_init_args[0] },
	{ SENSOR_NUM_PWR_P12V_ACCL_1, sensor_dev_ina233, I2C_BUS9, INA233_12V_1_7_ADDR,
	  PMBUS_READ_EIN, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[0],
	  post_ina233_read, &pwr_monitor_args[0], &accl_pwr_monitor_ina233_init_args[0] },

	/** INA233 12V 2 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_2, sensor_dev_ina233, I2C_BUS9, INA233_12V_2_8_ADDR,
	  PMBUS_READ_VOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[1],
	  post_ina233_read, &pwr_monitor_args[1], &accl_pwr_monitor_ina233_init_args[1] },
	{ SENSOR_NUM_CUR_P12V_ACCL_2, sensor_dev_ina233, I2C_BUS9, INA233_12V_2_8_ADDR,
	  PMBUS_READ_IOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[1],
	  post_ina233_read, &pwr_monitor_args[1], &accl_pwr_monitor_ina233_init_args[1] },
	{ SENSOR_NUM_PWR_P12V_ACCL_2, sensor_dev_ina233, I2C_BUS9, INA233_12V_2_8_ADDR,
	  PMBUS_READ_EIN, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[1],
	  post_ina233_read, &pwr_monitor_args[1], &accl_pwr_monitor_ina233_init_args[1] },

	/** INA233 12V 3 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_3, sensor_dev_ina233, I2C_BUS9, INA233_12V_3_9_ADDR,
	  PMBUS_READ_VOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[2],
	  post_ina233_read, &pwr_monitor_args[2], &accl_pwr_monitor_ina233_init_args[2] },
	{ SENSOR_NUM_CUR_P12V_ACCL_3, sensor_dev_ina233, I2C_BUS9, INA233_12V_3_9_ADDR,
	  PMBUS_READ_IOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[2],
	  post_ina233_read, &pwr_monitor_args[2], &accl_pwr_monitor_ina233_init_args[2] },
	{ SENSOR_NUM_PWR_P12V_ACCL_3, sensor_dev_ina233, I2C_BUS9, INA233_12V_3_9_ADDR,
	  PMBUS_READ_EIN, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[2],
	  post_ina233_read, &pwr_monitor_args[2], &accl_pwr_monitor_ina233_init_args[2] },

	/** INA233 12V 4 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_4, sensor_dev_ina233, I2C_BUS9, INA233_12V_4_10_ADDR,
	  PMBUS_READ_VOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[3],
	  post_ina233_read, &pwr_monitor_args[3], &accl_pwr_monitor_ina233_init_args[3] },
	{ SENSOR_NUM_CUR_P12V_ACCL_4, sensor_dev_ina233, I2C_BUS9, INA233_12V_4_10_ADDR,
	  PMBUS_READ_IOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[3],
	  post_ina233_read, &pwr_monitor_args[3], &accl_pwr_monitor_ina233_init_args[3] },
	{ SENSOR_NUM_PWR_P12V_ACCL_4, sensor_dev_ina233, I2C_BUS9, INA233_12V_4_10_ADDR,
	  PMBUS_READ_EIN, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[3],
	  post_ina233_read, &pwr_monitor_args[3], &accl_pwr_monitor_ina233_init_args[3] },

	/** INA233 12V 5 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_5, sensor_dev_ina233, I2C_BUS9, INA233_12V_5_11_ADDR,
	  PMBUS_READ_VOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[4],
	  post_ina233_read, &pwr_monitor_args[4], &accl_pwr_monitor_ina233_init_args[4] },
	{ SENSOR_NUM_CUR_P12V_ACCL_5, sensor_dev_ina233, I2C_BUS9, INA233_12V_5_11_ADDR,
	  PMBUS_READ_IOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[4],
	  post_ina233_read, &pwr_monitor_args[4], &accl_pwr_monitor_ina233_init_args[4] },
	{ SENSOR_NUM_PWR_P12V_ACCL_5, sensor_dev_ina233, I2C_BUS9, INA233_12V_5_11_ADDR,
	  PMBUS_READ_EIN, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[4],
	  post_ina233_read, &pwr_monitor_args[4], &accl_pwr_monitor_ina233_init_args[4] },

	/** INA233 12V 6 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_6, sensor_dev_ina233, I2C_BUS9, INA233_12V_12_ADDR,
	  PMBUS_READ_VOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[5],
	  post_ina233_read, &pwr_monitor_args[5], &accl_pwr_monitor_ina233_init_args[5] },
	{ SENSOR_NUM_CUR_P12V_ACCL_6, sensor_dev_ina233, I2C_BUS9, INA233_12V_12_ADDR,
	  PMBUS_READ_IOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[5],
	  post_ina233_read, &pwr_monitor_args[5], &accl_pwr_monitor_ina233_init_args[5] },
	{ SENSOR_NUM_PWR_P12V_ACCL_6, sensor_dev_ina233, I2C_BUS9, INA233_12V_12_ADDR,
	  PMBUS_READ_EIN, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[5],
	  post_ina233_read, &pwr_monitor_args[5], &accl_pwr_monitor_ina233_init_args[5] },

	/** INA233 12V 7 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_7, sensor_dev_ina233, I2C_BUS4, INA233_12V_1_7_ADDR,
	  PMBUS_READ_VOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[6],
	  post_ina233_read, &pwr_monitor_args[6], &accl_pwr_monitor_ina233_init_args[6] },
	{ SENSOR_NUM_CUR_P12V_ACCL_7, sensor_dev_ina233, I2C_BUS4, INA233_12V_1_7_ADDR,
	  PMBUS_READ_IOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[6],
	  post_ina233_read, &pwr_monitor_args[6], &accl_pwr_monitor_ina233_init_args[6] },
	{ SENSOR_NUM_PWR_P12V_ACCL_7, sensor_dev_ina233, I2C_BUS4, INA233_12V_1_7_ADDR,
	  PMBUS_READ_EIN, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[6],
	  post_ina233_read, &pwr_monitor_args[6], &accl_pwr_monitor_ina233_init_args[6] },

	/** INA233 12V 8 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_8, sensor_dev_ina233, I2C_BUS4, INA233_12V_2_8_ADDR,
	  PMBUS_READ_VOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[7],
	  post_ina233_read, &pwr_monitor_args[7], &accl_pwr_monitor_ina233_init_args[7] },
	{ SENSOR_NUM_CUR_P12V_ACCL_8, sensor_dev_ina233, I2C_BUS4, INA233_12V_2_8_ADDR,
	  PMBUS_READ_IOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[7],
	  post_ina233_read, &pwr_monitor_args[7], &accl_pwr_monitor_ina233_init_args[7] },
	{ SENSOR_NUM_PWR_P12V_ACCL_8, sensor_dev_ina233, I2C_BUS4, INA233_12V_2_8_ADDR,
	  PMBUS_READ_EIN, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[7],
	  post_ina233_read, &pwr_monitor_args[7], &accl_pwr_monitor_ina233_init_args[7] },

	/** INA233 12V 9 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_9, sensor_dev_ina233, I2C_BUS4, INA233_12V_3_9_ADDR,
	  PMBUS_READ_VOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[8],
	  post_ina233_read, &pwr_monitor_args[8], &accl_pwr_monitor_ina233_init_args[8] },
	{ SENSOR_NUM_CUR_P12V_ACCL_9, sensor_dev_ina233, I2C_BUS4, INA233_12V_3_9_ADDR,
	  PMBUS_READ_IOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[8],
	  post_ina233_read, &pwr_monitor_args[8], &accl_pwr_monitor_ina233_init_args[8] },
	{ SENSOR_NUM_PWR_P12V_ACCL_9, sensor_dev_ina233, I2C_BUS4, INA233_12V_3_9_ADDR,
	  PMBUS_READ_EIN, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[8],
	  post_ina233_read, &pwr_monitor_args[8], &accl_pwr_monitor_ina233_init_args[8] },

	/** INA233 12V 10 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_10, sensor_dev_ina233, I2C_BUS4, INA233_12V_4_10_ADDR,
	  PMBUS_READ_VOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[9],
	  post_ina233_read, &pwr_monitor_args[9], &accl_pwr_monitor_ina233_init_args[9] },
	{ SENSOR_NUM_CUR_P12V_ACCL_10, sensor_dev_ina233, I2C_BUS4, INA233_12V_4_10_ADDR,
	  PMBUS_READ_IOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[9],
	  post_ina233_read, &pwr_monitor_args[9], &accl_pwr_monitor_ina233_init_args[9] },
	{ SENSOR_NUM_PWR_P12V_ACCL_10, sensor_dev_ina233, I2C_BUS4, INA233_12V_4_10_ADDR,
	  PMBUS_READ_EIN, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[9],
	  post_ina233_read, &pwr_monitor_args[9], &accl_pwr_monitor_ina233_init_args[9] },

	/** INA233 12V 11 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_11, sensor_dev_ina233, I2C_BUS4, INA233_12V_5_11_ADDR,
	  PMBUS_READ_VOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[10],
	  post_ina233_read, &pwr_monitor_args[10], &accl_pwr_monitor_ina233_init_args[10] },
	{ SENSOR_NUM_CUR_P12V_ACCL_11, sensor_dev_ina233, I2C_BUS4, INA233_12V_5_11_ADDR,
	  PMBUS_READ_IOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[10],
	  post_ina233_read, &pwr_monitor_args[10], &accl_pwr_monitor_ina233_init_args[10] },
	{ SENSOR_NUM_PWR_P12V_ACCL_11, sensor_dev_ina233, I2C_BUS4, INA233_12V_5_11_ADDR,
	  PMBUS_READ_EIN, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[10],
	  post_ina233_read, &pwr_monitor_args[10], &accl_pwr_monitor_ina233_init_args[10] },

	/** INA233 12V 12 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_12, sensor_dev_ina233, I2C_BUS4, INA233_12V_12_ADDR,
	  PMBUS_READ_VOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[11],
	  post_ina233_read, &pwr_monitor_args[11], &accl_pwr_monitor_ina233_init_args[11] },
	{ SENSOR_NUM_CUR_P12V_ACCL_12, sensor_dev_ina233, I2C_BUS4, INA233_12V_12_ADDR,
	  PMBUS_READ_IOUT, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[11],
	  post_ina233_read, &pwr_monitor_args[11], &accl_pwr_monitor_ina233_init_args[11] },
	{ SENSOR_NUM_PWR_P12V_ACCL_12, sensor_dev_ina233, I2C_BUS4, INA233_12V_12_ADDR,
	  PMBUS_READ_EIN, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[11],
	  post_ina233_read, &pwr_monitor_args[11], &accl_pwr_monitor_ina233_init_args[11] },

	/** SQ52205 **/
	{ SENSOR_NUM_VOL_P12V_1_M_AUX, sensor_dev_sq52205, I2C_BUS2, SQ52205_P1V25_1_ADDR,
	  SQ52205_READ_VOL_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &u178_179_sq52205_init_args[0] },
	{ SENSOR_NUM_CUR_P12V_1_M_AUX, sensor_dev_sq52205, I2C_BUS2, SQ52205_P1V25_1_ADDR,
	  SQ52205_READ_CUR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &u178_179_sq52205_init_args[0] },
	{ SENSOR_NUM_PWR_P12V_1_M_AUX, sensor_dev_sq52205, I2C_BUS2, SQ52205_P1V25_1_ADDR,
	  SQ52205_READ_EIN_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &u178_179_sq52205_init_args[0] },
	{ SENSOR_NUM_VOL_P12V_2_M_AUX, sensor_dev_sq52205, I2C_BUS2, SQ52205_P1V25_2_ADDR,
	  SQ52205_READ_VOL_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &u178_179_sq52205_init_args[1] },
	{ SENSOR_NUM_CUR_P12V_2_M_AUX, sensor_dev_sq52205, I2C_BUS2, SQ52205_P1V25_2_ADDR,
	  SQ52205_READ_CUR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &u178_179_sq52205_init_args[1] },
	{ SENSOR_NUM_PWR_P12V_2_M_AUX, sensor_dev_sq52205, I2C_BUS2, SQ52205_P1V25_2_ADDR,
	  SQ52205_READ_EIN_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &u178_179_sq52205_init_args[1] },
};

sensor_cfg power_monitor_sq52205_ina230_sensor_config_table[] = {
	/** SQ52205 12V 1 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_1, sensor_dev_sq52205, I2C_BUS9, INA233_12V_1_7_ADDR,
	  SQ52205_READ_VOL_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[0],
	  post_ina233_read, &pwr_monitor_args[0], &accl_pwr_monitor_sq52205_init_args[0] },
	{ SENSOR_NUM_CUR_P12V_ACCL_1, sensor_dev_sq52205, I2C_BUS9, INA233_12V_1_7_ADDR,
	  SQ52205_READ_CUR_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[0],
	  post_ina233_read, &pwr_monitor_args[0], &accl_pwr_monitor_sq52205_init_args[0] },
	{ SENSOR_NUM_PWR_P12V_ACCL_1, sensor_dev_sq52205, I2C_BUS9, INA233_12V_1_7_ADDR,
	  SQ52205_READ_EIN_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[0],
	  post_ina233_read, &pwr_monitor_args[0], &accl_pwr_monitor_sq52205_init_args[0] },

	/** SQ52205 12V 2 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_2, sensor_dev_sq52205, I2C_BUS9, INA233_12V_2_8_ADDR,
	  SQ52205_READ_VOL_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[1],
	  post_ina233_read, &pwr_monitor_args[1], &accl_pwr_monitor_sq52205_init_args[1] },
	{ SENSOR_NUM_CUR_P12V_ACCL_2, sensor_dev_sq52205, I2C_BUS9, INA233_12V_2_8_ADDR,
	  SQ52205_READ_CUR_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[1],
	  post_ina233_read, &pwr_monitor_args[1], &accl_pwr_monitor_sq52205_init_args[1] },
	{ SENSOR_NUM_PWR_P12V_ACCL_2, sensor_dev_sq52205, I2C_BUS9, INA233_12V_2_8_ADDR,
	  SQ52205_READ_EIN_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[1],
	  post_ina233_read, &pwr_monitor_args[1], &accl_pwr_monitor_sq52205_init_args[1] },

	/** SQ52205 12V 3 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_3, sensor_dev_sq52205, I2C_BUS9, INA233_12V_3_9_ADDR,
	  SQ52205_READ_VOL_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[2],
	  post_ina233_read, &pwr_monitor_args[2], &accl_pwr_monitor_sq52205_init_args[2] },
	{ SENSOR_NUM_CUR_P12V_ACCL_3, sensor_dev_sq52205, I2C_BUS9, INA233_12V_3_9_ADDR,
	  SQ52205_READ_CUR_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[2],
	  post_ina233_read, &pwr_monitor_args[2], &accl_pwr_monitor_sq52205_init_args[2] },
	{ SENSOR_NUM_PWR_P12V_ACCL_3, sensor_dev_sq52205, I2C_BUS9, INA233_12V_3_9_ADDR,
	  SQ52205_READ_EIN_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[2],
	  post_ina233_read, &pwr_monitor_args[2], &accl_pwr_monitor_sq52205_init_args[2] },

	/** SQ52205 12V 4 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_4, sensor_dev_sq52205, I2C_BUS9, INA233_12V_4_10_ADDR,
	  SQ52205_READ_VOL_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[3],
	  post_ina233_read, &pwr_monitor_args[3], &accl_pwr_monitor_sq52205_init_args[3] },
	{ SENSOR_NUM_CUR_P12V_ACCL_4, sensor_dev_sq52205, I2C_BUS9, INA233_12V_4_10_ADDR,
	  SQ52205_READ_CUR_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[3],
	  post_ina233_read, &pwr_monitor_args[3], &accl_pwr_monitor_sq52205_init_args[3] },
	{ SENSOR_NUM_PWR_P12V_ACCL_4, sensor_dev_sq52205, I2C_BUS9, INA233_12V_4_10_ADDR,
	  SQ52205_READ_EIN_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[3],
	  post_ina233_read, &pwr_monitor_args[3], &accl_pwr_monitor_sq52205_init_args[3] },

	/** SQ52205 12V 5 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_5, sensor_dev_sq52205, I2C_BUS9, INA233_12V_5_11_ADDR,
	  SQ52205_READ_VOL_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[4],
	  post_ina233_read, &pwr_monitor_args[4], &accl_pwr_monitor_sq52205_init_args[4] },
	{ SENSOR_NUM_CUR_P12V_ACCL_5, sensor_dev_sq52205, I2C_BUS9, INA233_12V_5_11_ADDR,
	  SQ52205_READ_CUR_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[4],
	  post_ina233_read, &pwr_monitor_args[4], &accl_pwr_monitor_sq52205_init_args[4] },
	{ SENSOR_NUM_PWR_P12V_ACCL_5, sensor_dev_sq52205, I2C_BUS9, INA233_12V_5_11_ADDR,
	  SQ52205_READ_EIN_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[4],
	  post_ina233_read, &pwr_monitor_args[4], &accl_pwr_monitor_sq52205_init_args[4] },

	/** SQ52205 12V 6 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_6, sensor_dev_sq52205, I2C_BUS9, INA233_12V_12_ADDR,
	  SQ52205_READ_VOL_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[5],
	  post_ina233_read, &pwr_monitor_args[5], &accl_pwr_monitor_sq52205_init_args[5] },
	{ SENSOR_NUM_CUR_P12V_ACCL_6, sensor_dev_sq52205, I2C_BUS9, INA233_12V_12_ADDR,
	  SQ52205_READ_CUR_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[5],
	  post_ina233_read, &pwr_monitor_args[5], &accl_pwr_monitor_sq52205_init_args[5] },
	{ SENSOR_NUM_PWR_P12V_ACCL_6, sensor_dev_sq52205, I2C_BUS9, INA233_12V_12_ADDR,
	  SQ52205_READ_EIN_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[5],
	  post_ina233_read, &pwr_monitor_args[5], &accl_pwr_monitor_sq52205_init_args[5] },

	/** SQ52205 12V 7 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_7, sensor_dev_sq52205, I2C_BUS4, INA233_12V_1_7_ADDR,
	  SQ52205_READ_VOL_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[6],
	  post_ina233_read, &pwr_monitor_args[6], &accl_pwr_monitor_sq52205_init_args[6] },
	{ SENSOR_NUM_CUR_P12V_ACCL_7, sensor_dev_sq52205, I2C_BUS4, INA233_12V_1_7_ADDR,
	  SQ52205_READ_CUR_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[6],
	  post_ina233_read, &pwr_monitor_args[6], &accl_pwr_monitor_sq52205_init_args[6] },
	{ SENSOR_NUM_PWR_P12V_ACCL_7, sensor_dev_sq52205, I2C_BUS4, INA233_12V_1_7_ADDR,
	  SQ52205_READ_EIN_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[6],
	  post_ina233_read, &pwr_monitor_args[6], &accl_pwr_monitor_sq52205_init_args[6] },

	/** SQ52205 12V 8 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_8, sensor_dev_sq52205, I2C_BUS4, INA233_12V_2_8_ADDR,
	  SQ52205_READ_VOL_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[7],
	  post_ina233_read, &pwr_monitor_args[7], &accl_pwr_monitor_sq52205_init_args[7] },
	{ SENSOR_NUM_CUR_P12V_ACCL_8, sensor_dev_sq52205, I2C_BUS4, INA233_12V_2_8_ADDR,
	  SQ52205_READ_CUR_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[7],
	  post_ina233_read, &pwr_monitor_args[7], &accl_pwr_monitor_sq52205_init_args[7] },
	{ SENSOR_NUM_PWR_P12V_ACCL_8, sensor_dev_sq52205, I2C_BUS4, INA233_12V_2_8_ADDR,
	  SQ52205_READ_EIN_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[7],
	  post_ina233_read, &pwr_monitor_args[7], &accl_pwr_monitor_sq52205_init_args[7] },

	/** SQ52205 12V 9 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_9, sensor_dev_sq52205, I2C_BUS4, INA233_12V_3_9_ADDR,
	  SQ52205_READ_VOL_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[8],
	  post_ina233_read, &pwr_monitor_args[8], &accl_pwr_monitor_sq52205_init_args[8] },
	{ SENSOR_NUM_CUR_P12V_ACCL_9, sensor_dev_sq52205, I2C_BUS4, INA233_12V_3_9_ADDR,
	  SQ52205_READ_CUR_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[8],
	  post_ina233_read, &pwr_monitor_args[8], &accl_pwr_monitor_sq52205_init_args[8] },
	{ SENSOR_NUM_PWR_P12V_ACCL_9, sensor_dev_sq52205, I2C_BUS4, INA233_12V_3_9_ADDR,
	  SQ52205_READ_EIN_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[8],
	  post_ina233_read, &pwr_monitor_args[8], &accl_pwr_monitor_sq52205_init_args[8] },

	/** SQ52205 12V 10 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_10, sensor_dev_sq52205, I2C_BUS4, INA233_12V_4_10_ADDR,
	  SQ52205_READ_VOL_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[9],
	  post_ina233_read, &pwr_monitor_args[9], &accl_pwr_monitor_sq52205_init_args[9] },
	{ SENSOR_NUM_CUR_P12V_ACCL_10, sensor_dev_sq52205, I2C_BUS4, INA233_12V_4_10_ADDR,
	  SQ52205_READ_CUR_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[9],
	  post_ina233_read, &pwr_monitor_args[9], &accl_pwr_monitor_sq52205_init_args[9] },
	{ SENSOR_NUM_PWR_P12V_ACCL_10, sensor_dev_sq52205, I2C_BUS4, INA233_12V_4_10_ADDR,
	  SQ52205_READ_EIN_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[9],
	  post_ina233_read, &pwr_monitor_args[9], &accl_pwr_monitor_sq52205_init_args[9] },

	/** SQ52205 12V 11 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_11, sensor_dev_sq52205, I2C_BUS4, INA233_12V_5_11_ADDR,
	  SQ52205_READ_VOL_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[10],
	  post_ina233_read, &pwr_monitor_args[10], &accl_pwr_monitor_sq52205_init_args[10] },
	{ SENSOR_NUM_CUR_P12V_ACCL_11, sensor_dev_sq52205, I2C_BUS4, INA233_12V_5_11_ADDR,
	  SQ52205_READ_CUR_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[10],
	  post_ina233_read, &pwr_monitor_args[10], &accl_pwr_monitor_sq52205_init_args[10] },
	{ SENSOR_NUM_PWR_P12V_ACCL_11, sensor_dev_sq52205, I2C_BUS4, INA233_12V_5_11_ADDR,
	  SQ52205_READ_EIN_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[10],
	  post_ina233_read, &pwr_monitor_args[10], &accl_pwr_monitor_sq52205_init_args[10] },

	/** SQ52205 12V 12 **/
	{ SENSOR_NUM_VOL_P12V_ACCL_12, sensor_dev_sq52205, I2C_BUS4, INA233_12V_12_ADDR,
	  SQ52205_READ_VOL_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[11],
	  post_ina233_read, &pwr_monitor_args[11], &accl_pwr_monitor_sq52205_init_args[11] },
	{ SENSOR_NUM_CUR_P12V_ACCL_12, sensor_dev_sq52205, I2C_BUS4, INA233_12V_12_ADDR,
	  SQ52205_READ_CUR_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[11],
	  post_ina233_read, &pwr_monitor_args[11], &accl_pwr_monitor_sq52205_init_args[11] },
	{ SENSOR_NUM_PWR_P12V_ACCL_12, sensor_dev_sq52205, I2C_BUS4, INA233_12V_12_ADDR,
	  SQ52205_READ_EIN_OFFSET, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_ina233_read, &pwr_monitor_args[11],
	  post_ina233_read, &pwr_monitor_args[11], &accl_pwr_monitor_sq52205_init_args[11] },

	/** INA230 **/
	{ SENSOR_NUM_VOL_P12V_1_M_AUX, sensor_dev_ina230, I2C_BUS2, SQ52205_P1V25_1_ADDR,
	  SQ52205_READ_VOL_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &u178_179_ina230_init_args[0] },
	{ SENSOR_NUM_CUR_P12V_1_M_AUX, sensor_dev_ina230, I2C_BUS2, SQ52205_P1V25_1_ADDR,
	  SQ52205_READ_CUR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &u178_179_ina230_init_args[0] },
	{ SENSOR_NUM_PWR_P12V_1_M_AUX, sensor_dev_ina230, I2C_BUS2, SQ52205_P1V25_1_ADDR,
	  SQ52205_READ_PWR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &u178_179_ina230_init_args[0] },
	{ SENSOR_NUM_VOL_P12V_2_M_AUX, sensor_dev_ina230, I2C_BUS2, SQ52205_P1V25_2_ADDR,
	  SQ52205_READ_VOL_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &u178_179_ina230_init_args[1] },
	{ SENSOR_NUM_CUR_P12V_2_M_AUX, sensor_dev_ina230, I2C_BUS2, SQ52205_P1V25_2_ADDR,
	  SQ52205_READ_CUR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &u178_179_ina230_init_args[1] },
	{ SENSOR_NUM_PWR_P12V_2_M_AUX, sensor_dev_ina230, I2C_BUS2, SQ52205_P1V25_2_ADDR,
	  SQ52205_READ_PWR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &u178_179_ina230_init_args[1] },
};

sensor_compatible_cfg pre_dvt_update_cfg_table[] = {
	{ SENSOR_NUM_VOL_P12V_ACCL_1, I2C_BUS4, INA233_12V_1_7_ADDR, &pwr_monitor_pre_dvt_args[0],
	  &pwr_monitor_pre_dvt_args[0] },
	{ SENSOR_NUM_CUR_P12V_ACCL_1, I2C_BUS4, INA233_12V_1_7_ADDR, &pwr_monitor_pre_dvt_args[0],
	  &pwr_monitor_pre_dvt_args[0] },
	{ SENSOR_NUM_PWR_P12V_ACCL_1, I2C_BUS4, INA233_12V_1_7_ADDR, &pwr_monitor_pre_dvt_args[0],
	  &pwr_monitor_pre_dvt_args[0] },
	{ SENSOR_NUM_VOL_P12V_ACCL_2, I2C_BUS4, INA233_12V_2_8_ADDR, &pwr_monitor_pre_dvt_args[1],
	  &pwr_monitor_pre_dvt_args[1] },
	{ SENSOR_NUM_CUR_P12V_ACCL_2, I2C_BUS4, INA233_12V_2_8_ADDR, &pwr_monitor_pre_dvt_args[1],
	  &pwr_monitor_pre_dvt_args[1] },
	{ SENSOR_NUM_PWR_P12V_ACCL_2, I2C_BUS4, INA233_12V_2_8_ADDR, &pwr_monitor_pre_dvt_args[1],
	  &pwr_monitor_pre_dvt_args[1] },
	{ SENSOR_NUM_VOL_P12V_ACCL_3, I2C_BUS4, INA233_12V_3_9_ADDR, &pwr_monitor_pre_dvt_args[2],
	  &pwr_monitor_pre_dvt_args[2] },
	{ SENSOR_NUM_CUR_P12V_ACCL_3, I2C_BUS4, INA233_12V_3_9_ADDR, &pwr_monitor_pre_dvt_args[2],
	  &pwr_monitor_pre_dvt_args[2] },
	{ SENSOR_NUM_PWR_P12V_ACCL_3, I2C_BUS4, INA233_12V_3_9_ADDR, &pwr_monitor_pre_dvt_args[2],
	  &pwr_monitor_pre_dvt_args[2] },
	{ SENSOR_NUM_VOL_P12V_ACCL_4, I2C_BUS4, INA233_12V_4_10_ADDR, &pwr_monitor_pre_dvt_args[3],
	  &pwr_monitor_pre_dvt_args[3] },
	{ SENSOR_NUM_CUR_P12V_ACCL_4, I2C_BUS4, INA233_12V_4_10_ADDR, &pwr_monitor_pre_dvt_args[3],
	  &pwr_monitor_pre_dvt_args[3] },
	{ SENSOR_NUM_PWR_P12V_ACCL_4, I2C_BUS4, INA233_12V_4_10_ADDR, &pwr_monitor_pre_dvt_args[3],
	  &pwr_monitor_pre_dvt_args[3] },
	{ SENSOR_NUM_VOL_P12V_ACCL_5, I2C_BUS4, INA233_12V_5_11_ADDR, &pwr_monitor_pre_dvt_args[4],
	  &pwr_monitor_pre_dvt_args[4] },
	{ SENSOR_NUM_CUR_P12V_ACCL_5, I2C_BUS4, INA233_12V_5_11_ADDR, &pwr_monitor_pre_dvt_args[4],
	  &pwr_monitor_pre_dvt_args[4] },
	{ SENSOR_NUM_PWR_P12V_ACCL_5, I2C_BUS4, INA233_12V_5_11_ADDR, &pwr_monitor_pre_dvt_args[4],
	  &pwr_monitor_pre_dvt_args[4] },
	{ SENSOR_NUM_VOL_P12V_ACCL_6, I2C_BUS4, INA233_12V_12_ADDR, &pwr_monitor_pre_dvt_args[5],
	  &pwr_monitor_pre_dvt_args[5] },
	{ SENSOR_NUM_CUR_P12V_ACCL_6, I2C_BUS4, INA233_12V_12_ADDR, &pwr_monitor_pre_dvt_args[5],
	  &pwr_monitor_pre_dvt_args[5] },
	{ SENSOR_NUM_PWR_P12V_ACCL_6, I2C_BUS4, INA233_12V_12_ADDR, &pwr_monitor_pre_dvt_args[5],
	  &pwr_monitor_pre_dvt_args[5] },
	{ SENSOR_NUM_VOL_P12V_ACCL_7, I2C_BUS4, INA233_12V_1_7_ADDR, &pwr_monitor_pre_dvt_args[6],
	  &pwr_monitor_pre_dvt_args[6] },
	{ SENSOR_NUM_CUR_P12V_ACCL_7, I2C_BUS4, INA233_12V_1_7_ADDR, &pwr_monitor_pre_dvt_args[6],
	  &pwr_monitor_pre_dvt_args[6] },
	{ SENSOR_NUM_PWR_P12V_ACCL_7, I2C_BUS4, INA233_12V_1_7_ADDR, &pwr_monitor_pre_dvt_args[6],
	  &pwr_monitor_pre_dvt_args[6] },
	{ SENSOR_NUM_VOL_P12V_ACCL_8, I2C_BUS4, INA233_12V_2_8_ADDR, &pwr_monitor_pre_dvt_args[7],
	  &pwr_monitor_pre_dvt_args[7] },
	{ SENSOR_NUM_CUR_P12V_ACCL_8, I2C_BUS4, INA233_12V_2_8_ADDR, &pwr_monitor_pre_dvt_args[7],
	  &pwr_monitor_pre_dvt_args[7] },
	{ SENSOR_NUM_PWR_P12V_ACCL_8, I2C_BUS4, INA233_12V_2_8_ADDR, &pwr_monitor_pre_dvt_args[7],
	  &pwr_monitor_pre_dvt_args[7] },
	{ SENSOR_NUM_VOL_P12V_ACCL_9, I2C_BUS4, INA233_12V_3_9_ADDR, &pwr_monitor_pre_dvt_args[8],
	  &pwr_monitor_pre_dvt_args[8] },
	{ SENSOR_NUM_CUR_P12V_ACCL_9, I2C_BUS4, INA233_12V_3_9_ADDR, &pwr_monitor_pre_dvt_args[8],
	  &pwr_monitor_pre_dvt_args[8] },
	{ SENSOR_NUM_PWR_P12V_ACCL_9, I2C_BUS4, INA233_12V_3_9_ADDR, &pwr_monitor_pre_dvt_args[8],
	  &pwr_monitor_pre_dvt_args[8] },
	{ SENSOR_NUM_VOL_P12V_ACCL_10, I2C_BUS4, INA233_12V_4_10_ADDR, &pwr_monitor_pre_dvt_args[9],
	  &pwr_monitor_pre_dvt_args[9] },
	{ SENSOR_NUM_CUR_P12V_ACCL_10, I2C_BUS4, INA233_12V_4_10_ADDR, &pwr_monitor_pre_dvt_args[9],
	  &pwr_monitor_pre_dvt_args[9] },
	{ SENSOR_NUM_PWR_P12V_ACCL_10, I2C_BUS4, INA233_12V_4_10_ADDR, &pwr_monitor_pre_dvt_args[9],
	  &pwr_monitor_pre_dvt_args[9] },
	{ SENSOR_NUM_VOL_P12V_ACCL_11, I2C_BUS4, INA233_12V_5_11_ADDR,
	  &pwr_monitor_pre_dvt_args[10], &pwr_monitor_pre_dvt_args[10] },
	{ SENSOR_NUM_CUR_P12V_ACCL_11, I2C_BUS4, INA233_12V_5_11_ADDR,
	  &pwr_monitor_pre_dvt_args[10], &pwr_monitor_pre_dvt_args[10] },
	{ SENSOR_NUM_PWR_P12V_ACCL_11, I2C_BUS4, INA233_12V_5_11_ADDR,
	  &pwr_monitor_pre_dvt_args[10], &pwr_monitor_pre_dvt_args[10] },
	{ SENSOR_NUM_VOL_P12V_ACCL_12, I2C_BUS4, INA233_12V_12_ADDR, &pwr_monitor_pre_dvt_args[11],
	  &pwr_monitor_pre_dvt_args[11] },
	{ SENSOR_NUM_CUR_P12V_ACCL_12, I2C_BUS4, INA233_12V_12_ADDR, &pwr_monitor_pre_dvt_args[11],
	  &pwr_monitor_pre_dvt_args[11] },
	{ SENSOR_NUM_PWR_P12V_ACCL_12, I2C_BUS4, INA233_12V_12_ADDR, &pwr_monitor_pre_dvt_args[11],
	  &pwr_monitor_pre_dvt_args[11] },
};

sensor_poll_delay_cfg sensor_poll_delay_cfgs[] = {
	{ PCIE_CARD_1, false, ACCL_POWER_GOOD_TIME_DEFAULT },
	{ PCIE_CARD_2, false, ACCL_POWER_GOOD_TIME_DEFAULT },
	{ PCIE_CARD_3, false, ACCL_POWER_GOOD_TIME_DEFAULT },
	{ PCIE_CARD_4, false, ACCL_POWER_GOOD_TIME_DEFAULT },
	{ PCIE_CARD_5, false, ACCL_POWER_GOOD_TIME_DEFAULT },
	{ PCIE_CARD_6, false, ACCL_POWER_GOOD_TIME_DEFAULT },
	{ PCIE_CARD_7, false, ACCL_POWER_GOOD_TIME_DEFAULT },
	{ PCIE_CARD_8, false, ACCL_POWER_GOOD_TIME_DEFAULT },
	{ PCIE_CARD_9, false, ACCL_POWER_GOOD_TIME_DEFAULT },
	{ PCIE_CARD_10, false, ACCL_POWER_GOOD_TIME_DEFAULT },
	{ PCIE_CARD_11, false, ACCL_POWER_GOOD_TIME_DEFAULT },
	{ PCIE_CARD_12, false, ACCL_POWER_GOOD_TIME_DEFAULT },
};

const int SENSOR_CONFIG_SIZE = ARRAY_SIZE(plat_sensor_config);

void load_sensor_config(void)
{
	memcpy(sensor_config, plat_sensor_config, sizeof(plat_sensor_config));
	sensor_config_count = ARRAY_SIZE(plat_sensor_config);

	// Fix config table in different system/config
	pal_extend_sensor_config();
	update_plat_sensor_cfg_by_stage();
}

uint8_t pal_get_extend_sensor_config()
{
	uint8_t extend_sensor_config_size = 0;
	uint8_t hsc_module = get_hsc_module();
	uint8_t vr_module = get_vr_module();
	uint8_t power_brick_module = get_pwr_brick_module();
	uint8_t power_monitor_module = get_pwr_monitor_module();

	switch (hsc_module) {
	case HSC_MODULE_ADM1272:
		extend_sensor_config_size += ARRAY_SIZE(adm1272_sensor_config_table);
		break;
	case HSC_MODULE_LTC4286:
		extend_sensor_config_size += ARRAY_SIZE(ltc4286_sensor_config_table);
		break;
	default:
		LOG_ERR("Invalid hsc module: 0x%x", hsc_module);
		break;
	}

	switch (power_brick_module) {
	case POWER_BRICK_Q50SN120A1:
		extend_sensor_config_size += ARRAY_SIZE(q50sn120a1_sensor_config_table);
		break;
	case POWER_BRICK_BMR3512202:
		extend_sensor_config_size += ARRAY_SIZE(bmr351_sensor_config_table);
		break;
	default:
		LOG_ERR("Invalid power brick module: 0x%x", power_brick_module);
		break;
	}

	switch (power_monitor_module) {
	case POWER_MONITOR_INA233_SQ52205:
		extend_sensor_config_size +=
			ARRAY_SIZE(power_monitor_ina233_sq52205_sensor_config_table);
		break;
	case POWER_MONITOR_SQ52205_INA230:
		extend_sensor_config_size +=
			ARRAY_SIZE(power_monitor_sq52205_ina230_sensor_config_table);
		break;
	default:
		LOG_ERR("Invalid power monitor module: 0x%x", power_monitor_module);
		break;
	}

	switch (vr_module) {
	case VR_XDPE15284D:
		extend_sensor_config_size += ARRAY_SIZE(vr_xdpe15284_sensor_config_table);
		break;
	case VR_MP2985H:
		extend_sensor_config_size += ARRAY_SIZE(vr_mp2985h_sensor_config_table);
		break;
	default:
		LOG_ERR("Invalid vr module: 0x%x", vr_module);
		break;
	}

	return extend_sensor_config_size;
}

void pal_extend_sensor_config()
{
	uint8_t index = 0;
	uint8_t sensor_count = 0;
	uint8_t hsc_module = get_hsc_module();
	uint8_t vr_module = get_vr_module();
	uint8_t power_brick_module = get_pwr_brick_module();
	uint8_t power_monitor_module = get_pwr_monitor_module();

	switch (hsc_module) {
	case HSC_MODULE_ADM1272:
		sensor_count = ARRAY_SIZE(adm1272_sensor_config_table);
		for (index = 0; index < sensor_count; index++) {
			add_sensor_config(adm1272_sensor_config_table[index]);
		}
		break;
	case HSC_MODULE_LTC4286:
		sensor_count = ARRAY_SIZE(ltc4286_sensor_config_table);
		for (index = 0; index < sensor_count; index++) {
			add_sensor_config(ltc4286_sensor_config_table[index]);
		}
		break;
	default:
		LOG_ERR("Invalid hsc module: 0x%x", hsc_module);
		break;
	}

	switch (power_brick_module) {
	case POWER_BRICK_Q50SN120A1:
		sensor_count = ARRAY_SIZE(q50sn120a1_sensor_config_table);
		for (index = 0; index < sensor_count; index++) {
			add_sensor_config(q50sn120a1_sensor_config_table[index]);
		}
		break;
	case POWER_BRICK_BMR3512202:
		sensor_count = ARRAY_SIZE(bmr351_sensor_config_table);
		for (index = 0; index < sensor_count; index++) {
			add_sensor_config(bmr351_sensor_config_table[index]);
		}
		break;
	default:
		LOG_ERR("Invalid power brick module: 0x%x", power_brick_module);
		break;
	}

	switch (power_monitor_module) {
	case POWER_MONITOR_INA233_SQ52205:
		sensor_count = ARRAY_SIZE(power_monitor_ina233_sq52205_sensor_config_table);
		for (index = 0; index < sensor_count; index++) {
			add_sensor_config(power_monitor_ina233_sq52205_sensor_config_table[index]);
		}
		break;
	case POWER_MONITOR_SQ52205_INA230:
		sensor_count = ARRAY_SIZE(power_monitor_sq52205_ina230_sensor_config_table);
		for (index = 0; index < sensor_count; index++) {
			add_sensor_config(power_monitor_sq52205_ina230_sensor_config_table[index]);
		}
		break;
	default:
		LOG_ERR("Invalid power monitor module: 0x%x", power_monitor_module);
		break;
	}

	switch (vr_module) {
	case VR_XDPE15284D:
		sensor_count = ARRAY_SIZE(vr_xdpe15284_sensor_config_table);
		for (index = 0; index < sensor_count; index++) {
			add_sensor_config(vr_xdpe15284_sensor_config_table[index]);
		}
		break;
	case VR_MP2985H:
		sensor_count = ARRAY_SIZE(vr_mp2985h_sensor_config_table);
		for (index = 0; index < sensor_count; index++) {
			add_sensor_config(vr_mp2985h_sensor_config_table[index]);
		}
		break;
	default:
		LOG_ERR("Invalid vr module: 0x%x", vr_module);
		break;
	}
}

void update_plat_sensor_cfg_by_stage()
{
	uint8_t index = 0;
	uint8_t board_revision = get_board_revision();

	switch (board_revision) {
	case POC_STAGE:
	case EVT1_STAGE:
	case EVT2_STAGE:
		for (index = 0; index < ARRAY_SIZE(pre_dvt_update_cfg_table); ++index) {
			sensor_cfg *cfg =
				find_sensor_cfg_via_sensor_num(sensor_config, sensor_config_count,
							       pre_dvt_update_cfg_table[index].num);
			if (cfg == NULL) {
				LOG_ERR("Fail to find sensor to update sensor table, sensor num: 0x%x",
					pre_dvt_update_cfg_table[index].num);
				continue;
			}

			cfg->port = pre_dvt_update_cfg_table[index].port;
			cfg->target_addr = pre_dvt_update_cfg_table[index].target_addr;
			cfg->pre_sensor_read_args =
				pre_dvt_update_cfg_table[index].pre_sensor_read_args;
			cfg->post_sensor_read_args =
				pre_dvt_update_cfg_table[index].post_sensor_read_args;
		}
		break;
	case DVT_STAGE:
	case PVT_STAGE:
	case MP_STAGE:
		return;
	default:
		LOG_ERR("Fail to update sensor table, board revision: 0x%x", board_revision);
		return;
	}
}

sensor_cfg *get_common_sensor_cfg_info(uint8_t sensor_num)
{
	uint8_t cfg_count = sensor_monitor_table[COMMON_SENSOR_MONITOR_INDEX].cfg_count;
	sensor_cfg *cfg_table =
		sensor_monitor_table[COMMON_SENSOR_MONITOR_INDEX].monitor_sensor_cfg;

	if (cfg_table != NULL) {
		return find_sensor_cfg_via_sensor_num(cfg_table, cfg_count, sensor_num);
	}

	return NULL;
}

sensor_cfg *get_accl_sensor_cfg_info(uint8_t card_id, uint8_t *cfg_count)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg_count, NULL);

	sensor_cfg *cfg = NULL;

	uint8_t index = 0;

	for (index = 1; index < sensor_monitor_count; ++index) {
		uint8_t *val = (uint8_t *)sensor_monitor_table[index].priv_data;

		if (*val == card_id) {
			*cfg_count = sensor_monitor_table[index].cfg_count;
			return sensor_monitor_table[index].monitor_sensor_cfg;
		}
	}

	LOG_ERR("Fail to get ACCL sensor cfg info via card id: 0x%x", card_id);
	return cfg;
}

bool is_acb_power_good()
{
	// BIC can check motherboard dc power status by CPLD power good flag
	bool ret = false;
	uint8_t board_revision = get_board_revision();

	switch (board_revision) {
	case POC_STAGE:
	case EVT1_STAGE:
		ret = get_acb_power_status();
		if (ret != true) {
			LOG_ERR("Get acb power status fail");
			return false;
		}

		return get_acb_power_good_flag();
	case EVT2_STAGE:
	case DVT_STAGE:
	case PVT_STAGE:
	case MP_STAGE:
		return get_acb_power_good_flag();
	default:
		LOG_ERR("Invalid board revision: 0x%x", board_revision);
		return false;
	}
}

bool get_accl_power_status(uint8_t card_id, uint8_t option)
{
	int ret = 0;
	uint8_t val = 0;
	uint8_t power_status_bit = 0;
	uint8_t accl_1_6_register = 0;
	uint8_t accl_7_12_register = 0;

	switch (option) {
	case ACCL_CARD_12V_POWER_GOOD:
		accl_1_6_register = CPLD_12V_ACCLB_PWRGD_OFFSET;
		accl_7_12_register = CPLD_12V_ACCLA_PWRGD_OFFSET;
		break;
	case ACCL_CARD_3V3_POWER_GOOD:
		accl_1_6_register = CPLD_ACCLB_PWRGD_OFFSET;
		accl_7_12_register = CPLD_ACCLA_PWRGD_OFFSET;
		break;
	case ACCL_CABLE_POWER_GOOD:
		accl_1_6_register = CPLD_ACCL_1_6_POWER_CABLE_PG_OFFSET;
		accl_7_12_register = CPLD_ACCL_7_12_POWER_CABLE_PG_OFFSET;
		break;
	case ACCL_CABLE_POWER_GOOD_TIMEOUT:
		accl_1_6_register = CPLD_ACCL_1_6_POWER_CABLE_PG_TIMEOUT_OFFSET;
		accl_7_12_register = CPLD_ACCL_7_12_POWER_CABLE_PG_TIMEOUT_OFFSET;
		break;
	case ACCL_CABLE_POWER_GOOD_FAULT:
		accl_1_6_register = CPLD_ACCL_1_6_POWER_CABLE_PG_FAULT_OFFSET;
		accl_7_12_register = CPLD_ACCL_7_12_POWER_CABLE_PG_FAULT_OFFSET;
		break;
	default:
		LOG_ERR("[%s] invalid option: 0x%x", __func__, option);
		return false;
	}

	if (card_id <= PCIE_CARD_6) {
		power_status_bit = (1 << (card_id - PCIE_CARD_1));
		ret = get_cpld_register(accl_1_6_register, &val);
	} else if (card_id <= PCIE_CARD_12) {
		power_status_bit = (1 << (card_id - PCIE_CARD_7));
		ret = get_cpld_register(accl_7_12_register, &val);
	} else {
		LOG_ERR("[%s] invalid card id: 0x%x", __func__, card_id);
		return false;
	}

	if (ret != 0) {
		LOG_ERR("Get cpld register value fail, card id: 0x%x, option: 0x%x", card_id,
			option);
		return false;
	}

	return (val & power_status_bit);
}

bool is_accl_power_good(uint8_t card_id)
{
	return (get_accl_power_status(card_id, ACCL_CARD_3V3_POWER_GOOD) &
		get_accl_power_status(card_id, ACCL_CARD_12V_POWER_GOOD));
}

bool is_accl_cable_power_good(uint8_t card_id)
{
	return get_accl_power_status(card_id, ACCL_CABLE_POWER_GOOD);
}

bool is_accl_cable_power_good_timeout(uint8_t card_id)
{
	return get_accl_power_status(card_id, ACCL_CABLE_POWER_GOOD_TIMEOUT);
}

bool is_accl_cable_power_good_fault(uint8_t card_id)
{
	return get_accl_power_status(card_id, ACCL_CABLE_POWER_GOOD_FAULT);
}

bool is_dc_access(uint8_t sensor_num)
{
	return is_acb_power_good();
}

struct k_mutex *get_i2c_mux_mutex(uint8_t i2c_bus)
{
	struct k_mutex *mutex = NULL;

	switch (i2c_bus) {
	case I2C_BUS4:
		mutex = &i2c_4_pi4msd5v9542_mutex;
		break;
	case I2C_BUS7:
		mutex = &i2c_7_accl_mutex;
		break;
	case I2C_BUS8:
		mutex = &i2c_8_accl_mutex;
		break;
	default:
		LOG_ERR("No support for i2c bus %d mutex", i2c_bus);
		break;
	}

	return mutex;
}

int get_accl_bus(uint8_t card_id, uint8_t sensor_number)
{
	if (card_id >= ASIC_CARD_COUNT) {
		LOG_ERR("Invalid accl card id: 0x%x", card_id);
		return -1;
	}

	return pca9548_configs[card_id].bus;
}

sensor_cfg *get_accl_sensor_config(uint8_t card_id, uint8_t sensor_num)
{
	uint8_t i = 0;
	uint8_t cfg_count = 0;
	sensor_cfg *cfg_table = NULL;

	cfg_table = get_accl_sensor_cfg_info(card_id, &cfg_count);
	if (cfg_table == NULL) {
		LOG_ERR("Fail to get ACCL sensor cfg index, card id: 0x%x", card_id);
		return NULL;
	}

	for (i = 0; i < cfg_count; ++i) {
		if (sensor_num == cfg_table[i].num) {
			return &cfg_table[i];
		}
	}

	LOG_ERR("Fail to find sensor num: 0x%x in ACCL: 0x%x sensor config", sensor_num, card_id);
	return NULL;
}

bool get_accl_mux_config(uint8_t card_id, mux_config *accl_mux)
{
	CHECK_NULL_ARG_WITH_RETURN(accl_mux, false);

	if (card_id >= ASIC_CARD_COUNT) {
		LOG_ERR("Invalid accl card id: 0x%x", card_id);
		return false;
	}

	*accl_mux = pca9548_configs[card_id];
	return true;
}

bool is_time_to_poll_card_sensor(uint8_t card_id)
{
	if (card_id >= ARRAY_SIZE(sensor_poll_delay_cfgs)) {
		LOG_ERR("Invalid card id: 0x%x to check card polling time", card_id);
		return false;
	}

	if (is_accl_power_good(card_id) != true) {
		sensor_poll_delay_cfgs[card_id].is_last_time_power_good = false;
		sensor_poll_delay_cfgs[card_id].card_first_power_good_time =
			ACCL_POWER_GOOD_TIME_DEFAULT;
		return false;
	}

	/* Record ACCL card power on time when ACCL card first power good */
	if (sensor_poll_delay_cfgs[card_id].is_last_time_power_good != true) {
		sensor_poll_delay_cfgs[card_id].card_first_power_good_time = k_uptime_get();
		sensor_poll_delay_cfgs[card_id].is_last_time_power_good = true;
		return false;
	}

	/* Polling ACCL sensor when ACCL card power on time exceeds the delay time */
	if (sensor_poll_delay_cfgs[card_id].card_first_power_good_time !=
	    ACCL_POWER_GOOD_TIME_DEFAULT) {
		int64_t diff_time = (k_uptime_get() -
				     sensor_poll_delay_cfgs[card_id].card_first_power_good_time);
		if (diff_time < ACCL_SENSOR_POLL_DELAY_MS) {
			return false;
		} else {
			sensor_poll_delay_cfgs[card_id].card_first_power_good_time =
				ACCL_POWER_GOOD_TIME_DEFAULT;
		}
	}

	return true;
}
