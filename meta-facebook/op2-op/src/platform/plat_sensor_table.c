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

#include "plat_i2c.h"
#include "plat_sensor_table.h"
#include "plat_hook.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_sensor);

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
	{ SENSOR_NUM_1OU_E1S_SSD0_P12V_VOLT, sensor_dev_ina233, I2C_BUS3, INA233_E1S_0_ADDR,
	  INA233_VOLT_OFFSET, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[0] },

	{ SENSOR_NUM_1OU_E1S_SSD1_P12V_VOLT, sensor_dev_ina233, I2C_BUS3, INA233_E1S_1_ADDR,
	  INA233_VOLT_OFFSET, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[0] },

	{ SENSOR_NUM_1OU_E1S_SSD2_P12V_VOLT, sensor_dev_ina233, I2C_BUS3, INA233_E1S_2_ADDR,
	  INA233_VOLT_OFFSET, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[0] },

	{ SENSOR_NUM_1OU_P12V_EDGE_VOLT, sensor_dev_ina233, I2C_BUS3, INA233_MAIN_ADDR,
	  INA233_VOLT_OFFSET, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[0] },

	//INA233 CURR
	{ SENSOR_NUM_1OU_E1S_SSD0_P12V_CURR, sensor_dev_ina233, I2C_BUS3, INA233_E1S_0_ADDR,
	  INA233_CURR_OFFSET, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[0] },

	{ SENSOR_NUM_1OU_E1S_SSD1_P12V_CURR, sensor_dev_ina233, I2C_BUS3, INA233_E1S_1_ADDR,
	  INA233_CURR_OFFSET, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[0] },

	{ SENSOR_NUM_1OU_E1S_SSD2_P12V_CURR, sensor_dev_ina233, I2C_BUS3, INA233_E1S_2_ADDR,
	  INA233_CURR_OFFSET, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[0] },

	{ SENSOR_NUM_1OU_P12V_EDGE_CURR, sensor_dev_ina233, I2C_BUS3, INA233_MAIN_ADDR,
	  INA233_CURR_OFFSET, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[0] },

	//INA233 PWR
	{ SENSOR_NUM_1OU_E1S_SSD0_P12V_PWR, sensor_dev_ina233, I2C_BUS3, INA233_E1S_0_ADDR,
	  INA233_PWR_OFFSET, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[0] },

	{ SENSOR_NUM_1OU_E1S_SSD1_P12V_PWR, sensor_dev_ina233, I2C_BUS3, INA233_E1S_1_ADDR,
	  INA233_PWR_OFFSET, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[0] },

	{ SENSOR_NUM_1OU_E1S_SSD2_P12V_PWR, sensor_dev_ina233, I2C_BUS3, INA233_E1S_2_ADDR,
	  INA233_PWR_OFFSET, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[0] },

	{ SENSOR_NUM_1OU_P12V_EDGE_PWR, sensor_dev_ina233, I2C_BUS3, INA233_MAIN_ADDR,
	  INA233_PWR_OFFSET, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[0] },

	//E1S Temp
	{ SENSOR_NUM_1OU_E1S_SSD0_TEMP_C, sensor_dev_nvme, I2C_BUS2, NVME_ADDR, NVME_TEMP_OFFSET,
	  dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &i2c_proc_args[0], post_i2c_bus_read,
	  &i2c_proc_args[0], NULL },

	{ SENSOR_NUM_1OU_E1S_SSD1_TEMP_C, sensor_dev_nvme, I2C_BUS2, NVME_ADDR, NVME_TEMP_OFFSET,
	  dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &i2c_proc_args[1], post_i2c_bus_read,
	  &i2c_proc_args[1], NULL },
};
sensor_cfg plat_expansion_A_sensor_config[] = {

	//E1S Temp
	{ SENSOR_NUM_1OU_E1S_SSD2_TEMP_C, sensor_dev_nvme, I2C_BUS2, NVME_ADDR, NVME_TEMP_OFFSET,
	  dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &i2c_proc_args[5], post_i2c_bus_read,
	  &i2c_proc_args[5], NULL },

	//Temp
	{ SENSOR_NUM_1OU_TEMP, sensor_dev_tmp75, I2C_BUS4, TMP75_EXPA_TEMP_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	//adc
	{ SENSOR_NUM_1OU_P3V3_STBY_ADC_VOLT, sensor_dev_ast_adc, ADC_PORT0, NONE, NONE, stby_access,
	  2, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },

	{ SENSOR_NUM_1OU_E1S_SSD0_P3V3_ADC_VOLT, sensor_dev_ast_adc, ADC_PORT1, NONE, NONE,
	  dc_access, 2, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },

	{ SENSOR_NUM_1OU_E1S_SSD1_P3V3_ADC_VOLT, sensor_dev_ast_adc, ADC_PORT2, NONE, NONE,
	  dc_access, 2, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },

	{ SENSOR_NUM_1OU_E1S_SSD2_P3V3_ADC_VOLT, sensor_dev_ast_adc, ADC_PORT3, NONE, NONE,
	  dc_access, 2, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },

	{ SENSOR_NUM_1OU_P1V8_ADC_VOLT, sensor_dev_ast_adc, ADC_PORT4, NONE, NONE, dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },

	{ SENSOR_NUM_1OU_P0V9_ADC_VOLT, sensor_dev_ast_adc, ADC_PORT5, NONE, NONE, dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },

	{ SENSOR_NUM_1OU_P1V2_ADC_VOLT, sensor_dev_ast_adc, ADC_PORT6, NONE, NONE, dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
};

sensor_cfg plat_expansion_B_sensor_config[] = {

	//E1S Temp
	{ SENSOR_NUM_2OU_E1S_SSD2_TEMP_C, sensor_dev_nvme, I2C_BUS2, NVME_ADDR, NVME_TEMP_OFFSET,
	  dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &i2c_proc_args[2], post_i2c_bus_read,
	  &i2c_proc_args[2], NULL },

	{ SENSOR_NUM_2OU_E1S_SSD3_TEMP_C, sensor_dev_nvme, I2C_BUS2, NVME_ADDR, NVME_TEMP_OFFSET,
	  dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &i2c_proc_args[3], post_i2c_bus_read,
	  &i2c_proc_args[3], NULL },

	{ SENSOR_NUM_2OU_E1S_SSD4_TEMP_C, sensor_dev_nvme, I2C_BUS2, NVME_ADDR, NVME_TEMP_OFFSET,
	  dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &i2c_proc_args[4], post_i2c_bus_read,
	  &i2c_proc_args[4], NULL },

	//Temp
	{ SENSOR_NUM_2OU_TEMP, sensor_dev_tmp75, I2C_BUS4, TMP75_EXPB_TEMP_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	//adc
	{ SENSOR_NUM_2OU_E1S_SSD0_P3V3_ADC_VOLT, sensor_dev_ast_adc, ADC_PORT0, NONE, NONE,
	  dc_access, 2, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },

	{ SENSOR_NUM_2OU_E1S_SSD1_P3V3_ADC_VOLT, sensor_dev_ast_adc, ADC_PORT1, NONE, NONE,
	  dc_access, 2, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },

	{ SENSOR_NUM_2OU_E1S_SSD2_P3V3_ADC_VOLT, sensor_dev_ast_adc, ADC_PORT2, NONE, NONE,
	  dc_access, 2, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },

	{ SENSOR_NUM_2OU_E1S_SSD3_P3V3_ADC_VOLT, sensor_dev_ast_adc, ADC_PORT3, NONE, NONE,
	  dc_access, 2, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },

	{ SENSOR_NUM_2OU_E1S_SSD4_P3V3_ADC_VOLT, sensor_dev_ast_adc, ADC_PORT4, NONE, NONE,
	  dc_access, 2, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },

	{ SENSOR_NUM_2OU_P3V3_STBY_ADC_VOLT, sensor_dev_ast_adc, ADC_PORT5, NONE, NONE, stby_access,
	  2, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },

	{ SENSOR_NUM_2OU_P1V8_STBY_ADC_VOLT, sensor_dev_ast_adc, ADC_PORT11, NONE, NONE,
	  stby_access, 1, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },

	{ SENSOR_NUM_2OU_P1V2_STBY_ADC_VOLT, sensor_dev_ast_adc, ADC_PORT14, NONE, NONE,
	  stby_access, 1, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },

	//INA233 VOL
	{ SENSOR_NUM_2OU_E1S_SSD3_P12V_VOLT, sensor_dev_ina233, I2C_BUS3, INA233_E1S_3_ADDR,
	  INA233_VOLT_OFFSET, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[0] },

	{ SENSOR_NUM_2OU_E1S_SSD4_P12V_VOLT, sensor_dev_ina233, I2C_BUS3, INA233_E1S_4_ADDR,
	  INA233_VOLT_OFFSET, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[0] },

	//INA233 CURR
	{ SENSOR_NUM_2OU_E1S_SSD3_P12V_CURR, sensor_dev_ina233, I2C_BUS3, INA233_E1S_3_ADDR,
	  INA233_CURR_OFFSET, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[0] },

	{ SENSOR_NUM_2OU_E1S_SSD4_P12V_CURR, sensor_dev_ina233, I2C_BUS3, INA233_E1S_4_ADDR,
	  INA233_CURR_OFFSET, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[0] },

	//INA233 PWR
	{ SENSOR_NUM_2OU_E1S_SSD3_P12V_PWR, sensor_dev_ina233, I2C_BUS3, INA233_E1S_3_ADDR,
	  INA233_PWR_OFFSET, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[0] },

	{ SENSOR_NUM_2OU_E1S_SSD4_P12V_PWR, sensor_dev_ina233, I2C_BUS3, INA233_E1S_4_ADDR,
	  INA233_PWR_OFFSET, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &ina233_init_args[0] },
};

const int SENSOR_CONFIG_SIZE = ARRAY_SIZE(plat_sensor_config);

void load_sensor_config(void)
{
	pal_change_sensor_config_number();

	memcpy(sensor_config, plat_sensor_config, sizeof(plat_sensor_config));
	sensor_config_count = ARRAY_SIZE(plat_sensor_config);

	// Fix config table in different system/config
	pal_extend_sensor_config();
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

	switch (card_type) {
	case CARD_TYPE_OPA:

		sensor_count = ARRAY_SIZE(plat_expansion_A_sensor_config);
		for (int index = 0; index < sensor_count; index++) {
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
