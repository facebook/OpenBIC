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

#include "sensor.h"
#include "ast_adc.h"
#include "plat_sensor_table.h"
#include "plat_class.h"
#include "plat_i2c.h"
#include "plat_hook.h"
#include "plat_power_seq.h"
#include "plat_m2.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(plat_sensor_table);

#define CONFIG_ISL69260 false

sensor_cfg plat_sensor_config[] = {
	/* number,                  type,       port,      address,      offset,
	   access check arg0, arg1, sample_count, cache, cache_status, mux_address, mux_offset,
	   pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_fn  */

	// temperature
	{ SENSOR_NUM_T_MB_OUTLET_TEMP_T, sensor_dev_tmp75, I2C_BUS2, TMP75_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	/*
		E1S respin doesn't have smbus for hsc, add to SDR but not sensor
 	*/

	// Voltage
	{ SENSOR_NUM_V_12_AUX, sensor_dev_ast_adc, ADC_PORT1, NONE, NONE, dc_access, 704, 100,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_V_12_EDGE, sensor_dev_ast_adc, ADC_PORT2, NONE, NONE, stby_access, 704, 100,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_V_3_3_AUX, sensor_dev_ast_adc, ADC_PORT7, NONE, NONE, stby_access, 487, 200,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_V_1_2_STBY, sensor_dev_ast_adc, ADC_PORT15, NONE, NONE, stby_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[1] },
	{ SENSOR_NUM_ADC_12V_VOL_M2A, sensor_dev_ast_adc, ADC_PORT8, NONE, NONE, is_m2_sen_readable,
	  704, 100, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[1] },
	{ SENSOR_NUM_ADC_12V_VOL_M2B, sensor_dev_ast_adc, ADC_PORT9, NONE, NONE, is_m2_sen_readable,
	  704, 100, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[1] },
	{ SENSOR_NUM_ADC_12V_VOL_M2C, sensor_dev_ast_adc, ADC_PORT10, NONE, NONE,
	  is_m2_sen_readable, 704, 100, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &adc_asd_init_args[1] },
	{ SENSOR_NUM_ADC_12V_VOL_M2D, sensor_dev_ast_adc, ADC_PORT11, NONE, NONE,
	  is_m2_sen_readable, 704, 100, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &adc_asd_init_args[1] },
	{ SENSOR_NUM_ADC_3V3_VOL_M2A, sensor_dev_ast_adc, ADC_PORT6, NONE, NONE, is_m2_sen_readable,
	  487, 200, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_ADC_3V3_VOL_M2B, sensor_dev_ast_adc, ADC_PORT5, NONE, NONE, is_m2_sen_readable,
	  487, 200, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_ADC_3V3_VOL_M2C, sensor_dev_ast_adc, ADC_PORT4, NONE, NONE, is_m2_sen_readable,
	  487, 200, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_ADC_3V3_VOL_M2D, sensor_dev_ast_adc, ADC_PORT3, NONE, NONE, is_m2_sen_readable,
	  487, 200, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },

	// m.2 power
	{ SENSOR_NUM_INA231_PWR_M2A, sensor_dev_ina230, I2C_BUS_M2A, I2C_ADDR_M2_INA231,
	  INA231_POWER_REG, is_m2_sen_readable, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina231_init_args[0] },
	{ SENSOR_NUM_INA231_PWR_M2B, sensor_dev_ina230, I2C_BUS_M2B, I2C_ADDR_M2_INA231,
	  INA231_POWER_REG, is_m2_sen_readable, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina231_init_args[1] },
	{ SENSOR_NUM_INA231_PWR_M2C, sensor_dev_ina230, I2C_BUS_M2C, I2C_ADDR_M2_INA231,
	  INA231_POWER_REG, is_m2_sen_readable, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina231_init_args[2] },
	{ SENSOR_NUM_INA231_PWR_M2D, sensor_dev_ina230, I2C_BUS_M2D, I2C_ADDR_M2_INA231,
	  INA231_POWER_REG, is_m2_sen_readable, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina231_init_args[3] },

	// m.2 voltage
	{ SENSOR_NUM_INA231_VOL_M2A, sensor_dev_ina230, I2C_BUS_M2A, I2C_ADDR_M2_INA231,
	  INA231_BUS_VOLTAGE_REG, is_m2_sen_readable, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina231_init_args[0] },
	{ SENSOR_NUM_INA231_VOL_M2B, sensor_dev_ina230, I2C_BUS_M2B, I2C_ADDR_M2_INA231,
	  INA231_BUS_VOLTAGE_REG, is_m2_sen_readable, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina231_init_args[1] },
	{ SENSOR_NUM_INA231_VOL_M2C, sensor_dev_ina230, I2C_BUS_M2C, I2C_ADDR_M2_INA231,
	  INA231_BUS_VOLTAGE_REG, is_m2_sen_readable, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina231_init_args[2] },
	{ SENSOR_NUM_INA231_VOL_M2D, sensor_dev_ina230, I2C_BUS_M2D, I2C_ADDR_M2_INA231,
	  INA231_BUS_VOLTAGE_REG, is_m2_sen_readable, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina231_init_args[3] },

	// m.2 temp
	{ SENSOR_NUM_NVME_TEMP_M2A, sensor_dev_nvme, I2C_BUS_M2A, NVME_ADDR, NVME_TEMP_REG,
	  is_nvme_temp_readable, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_NVME_TEMP_M2B, sensor_dev_nvme, I2C_BUS_M2B, NVME_ADDR, NVME_TEMP_REG,
	  is_nvme_temp_readable, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_NVME_TEMP_M2C, sensor_dev_nvme, I2C_BUS_M2C, NVME_ADDR, NVME_TEMP_REG,
	  is_nvme_temp_readable, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_NVME_TEMP_M2D, sensor_dev_nvme, I2C_BUS_M2D, NVME_ADDR, NVME_TEMP_REG,
	  is_nvme_temp_readable, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

};

const int SENSOR_CONFIG_SIZE = ARRAY_SIZE(plat_sensor_config);

void load_sensor_config(void)
{
	memcpy(sensor_config, plat_sensor_config, sizeof(plat_sensor_config));
	sensor_config_count = ARRAY_SIZE(plat_sensor_config);
}

/* For E1S re-spin ADC to ISL28022 */
sensor_cfg e1s_adc_sensor[] = {
	{ .num = SENSOR_NUM_INA231_PWR_M2A,
	  .type = sensor_dev_isl28022,
	  .target_addr = I2C_ADDR_M2_ISL28022,
	  .offset = ISL28022_POWER_REG,
	  .init_args = &isl28022_init_args[0] },
	{ .num = SENSOR_NUM_INA231_PWR_M2B,
	  .type = sensor_dev_isl28022,
	  .target_addr = I2C_ADDR_M2_ISL28022,
	  .offset = ISL28022_POWER_REG,
	  .init_args = &isl28022_init_args[1] },
	{ .num = SENSOR_NUM_INA231_PWR_M2C,
	  .type = sensor_dev_isl28022,
	  .target_addr = I2C_ADDR_M2_ISL28022,
	  .offset = ISL28022_POWER_REG,
	  .init_args = &isl28022_init_args[2] },
	{ .num = SENSOR_NUM_INA231_PWR_M2D,
	  .type = sensor_dev_isl28022,
	  .target_addr = I2C_ADDR_M2_ISL28022,
	  .offset = ISL28022_POWER_REG,
	  .init_args = &isl28022_init_args[3] },
	{ .num = SENSOR_NUM_INA231_VOL_M2A,
	  .type = sensor_dev_isl28022,
	  .target_addr = I2C_ADDR_M2_ISL28022,
	  .offset = ISL28022_BUS_VOLTAGE_REG,
	  .init_args = &isl28022_init_args[0] },
	{ .num = SENSOR_NUM_INA231_VOL_M2B,
	  .type = sensor_dev_isl28022,
	  .target_addr = I2C_ADDR_M2_ISL28022,
	  .offset = ISL28022_BUS_VOLTAGE_REG,
	  .init_args = &isl28022_init_args[1] },
	{ .num = SENSOR_NUM_INA231_VOL_M2C,
	  .type = sensor_dev_isl28022,
	  .target_addr = I2C_ADDR_M2_ISL28022,
	  .offset = ISL28022_BUS_VOLTAGE_REG,
	  .init_args = &isl28022_init_args[2] },
	{ .num = SENSOR_NUM_INA231_VOL_M2D,
	  .type = sensor_dev_isl28022,
	  .target_addr = I2C_ADDR_M2_ISL28022,
	  .offset = ISL28022_BUS_VOLTAGE_REG,
	  .init_args = &isl28022_init_args[3] },
};

void pal_fix_sensor_config() //e1s_reassemble_adc_sensor_table
{
	uint32_t i, j;

	if (get_e1s_adc_config() == CONFIG_ADC_ISL28022) {
		LOG_ERR("Reassemble adc sensor table to ISL28022");
		for (i = 0; i < ARRAY_SIZE(e1s_adc_sensor); i++) {
			for (j = 0; j < ARRAY_SIZE(plat_sensor_config); j++) {
				if (e1s_adc_sensor[i].num == plat_sensor_config[j].num) {
					/* Change sensor configuration for ADC chip isl28022*/
					plat_sensor_config[j].type = e1s_adc_sensor[i].type;
					plat_sensor_config[j].target_addr =
						e1s_adc_sensor[i].target_addr;
					plat_sensor_config[j].offset = e1s_adc_sensor[i].offset;
					plat_sensor_config[j].init_args =
						e1s_adc_sensor[i].init_args;
				}
			}
		}
	}
}
