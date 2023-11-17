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

#include <logging/log.h>
#include "pmbus.h"
#include "ast_adc.h"
#include "pdr.h"
#include "sensor.h"
#include "pldm_sensor.h"
#include "pldm_monitor.h"
#include "plat_apml.h"
#include "plat_hook.h"
#include "plat_i2c.h"
#include "plat_pldm_sensor.h"

LOG_MODULE_REGISTER(plat_pldm_sensor);

static struct pldm_sensor_thread pal_pldm_sensor_thread[MAX_SENSOR_THREAD_ID] = {
	// thread id, thread name
	{ ADC_SENSOR_THREAD_ID, "ADC_PLDM_SENSOR_THREAD" },
	{ VR_SENSOR_THREAD_ID, "VR_PLDM_SENSOR_THREAD" },
	{ MB_TEMP_SENSOR_THREAD_ID, "MB_TEMP_SENSOR_THREAD" },
	{ CPU_SENSOR_THREAD_ID, "CPU_TEMP_SENSOR_THREAD" },
};

pldm_sensor_info plat_pldm_sensor_adc_table[] = {
	{
		{
			// P12V stby Voltage
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x0018, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x0002, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;
			-4, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x04, //uint8_t sensor_data_size;
			1, //int32_t resolution;
			0, //int32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //int32_t update_interval;
			0x0001FA40, //uint32_t max_readable;
			0x0001AF40, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00020460, //uint32_t warning_high;
			0x0001A6A0, //uint32_t warning_low;
			0x00020970, //uint32_t critical_high;
			0x0001A250, //uint32_t critical_low;
			0x00022FE2, //uint32_t fatal_high;
			0x00018A2E, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_ast_adc,
			.port = ADC_PORT0,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.arg0 = 66,
			.arg1 = 10,
			.init_args = &ast_adc_init_args[0],
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// PVDD18 S5 Voltage
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x0019, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x0003, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;
			-3, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000000, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_ast_adc,
			.port = ADC_PORT1,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.arg0 = 1,
			.arg1 = 1,
			.init_args = &ast_adc_init_args[0],
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// P3V3 stby Voltage
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x001A, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x0004, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;
			-5, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x04, //uint8_t sensor_data_size;
			1, //int32_t resolution;
			0, //int32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_3S, //int32_t update_interval;
			0x00054984, //uint32_t max_readable;
			0x0004C89C, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00056496, //uint32_t warning_high;
			0x0004B01E, //uint32_t warning_low;
			0x00057279, //uint32_t critical_high;
			0x0004A3DF, //uint32_t critical_low;
			0x0005D750, //uint32_t fatal_high;
			0x00040740, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_ast_adc,
			.port = ADC_PORT2,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.arg0 = 2,
			.arg1 = 1,
			.init_args = &ast_adc_init_args[0],
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// P3V BAT Voltage
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x001B, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x0005, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;
			-4, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //int32_t update_interval;
			0x000084D0, //uint32_t max_readable;
			0x00006F54, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00008778, //uint32_t warning_high;
			0x00006D1A, //uint32_t warning_low;
			0x000088CC, //uint32_t critical_high;
			0x00006BFD, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_ast_adc,
			.port = ADC_PORT4,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.arg0 = 3,
			.arg1 = 1,
			.init_args = &ast_adc_init_args[0],
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// PVDD33 S5 Voltage
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x001C, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x0006, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;
			-3, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000000, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_ast_adc,
			.port = ADC_PORT5,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.arg0 = 2,
			.arg1 = 1,
			.init_args = &ast_adc_init_args[0],
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// P5V stby Voltage
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x001D, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x0007, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;
			-4, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x0000D6D8, //uint32_t max_readable;
			0x0000B98C, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x0000DB24, //uint32_t warning_high;
			0x0000B5D6, //uint32_t warning_low;
			0x0000DD4A, //uint32_t critical_high;
			0x0000B3FB, //uint32_t critical_low;
			0x0000E290, //uint32_t fatal_high;
			0x00009C40, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_ast_adc,
			.port = ADC_PORT6,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.arg0 = 711,
			.arg1 = 200,
			.init_args = &ast_adc_init_args[0],
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// P12V DIMM0 Voltage
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x001E, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x0008, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;
			-0x03, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x000035E8, //uint32_t max_readable;
			0x000027D8, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x000036FC, //uint32_t warning_high;
			0x0000270C, //uint32_t warning_low;
			0x00003786, //uint32_t critical_high;
			0x000026A6, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_ast_adc,
			.port = ADC_PORT7,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.arg0 = 66,
			.arg1 = 10,
			.init_args = &ast_adc_init_args[0],
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// P12V DIMM1 Voltage
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x001F, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x0009, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;
			-0x03, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x000035E8, //uint32_t max_readable;
			0x000027D8, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x000036FC, //uint32_t warning_high;
			0x0000270C, //uint32_t warning_low;
			0x00003786, //uint32_t critical_high;
			0x000026A6, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_ast_adc,
			.port = ADC_PORT8,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.arg0 = 66,
			.arg1 = 10,
			.init_args = &ast_adc_init_args[1],
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// P1V2 stby Voltage
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x0020, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x000A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;
			-4, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00003138, //uint32_t max_readable;
			0x00002C88, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00003234, //uint32_t warning_high;
			0x00002BA4, //uint32_t warning_low;
			0x000032B2, //uint32_t critical_high;
			0x00002B32, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_ast_adc,
			.port = ADC_PORT9,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.arg0 = 1,
			.arg1 = 1,
			.init_args = &ast_adc_init_args[1],
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// P1V8 stby Voltage
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x0021, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x000B, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;
			-4, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x000049D4, //uint32_t max_readable;
			0x000042CC, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00004B4E, //uint32_t warning_high;
			0x00004176, //uint32_t warning_low;
			0x00004C0B, //uint32_t critical_high;
			0x000040CB, //uint32_t critical_low;
			0x00005190, //uint32_t fatal_high;
			0x00003840, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_ast_adc,
			.port = ADC_PORT11,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.arg0 = 1,
			.arg1 = 1,
			.init_args = &ast_adc_init_args[1],
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// Slot detect Voltage
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x0022, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x000C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;
			-3, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000000, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_ast_adc,
			.port = ADC_PORT13,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.arg0 = 367,
			.arg1 = 267,
			.init_args = &ast_adc_init_args[1],
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// Sidecar detect stby Voltage
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x0023, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x000D, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;
			-3, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000000, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_ast_adc,
			.port = ADC_PORT15,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.arg0 = 1,
			.arg1 = 1,
			.init_args = &ast_adc_init_args[1],
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
};

pldm_sensor_info plat_pldm_sensor_vr_table[] = {
	{
		{
			// VR CPU0 Temperature
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x0013, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0001, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;
			0, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x000000FF, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000064, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x0000007D, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2856gut,
			.port = I2C_BUS4,
			.target_addr = ADDR_VR_CPU0,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// VR SOC Temperature
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x0014, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0002, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;
			0, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x000000FF, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000064, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x0000007D, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2856gut,
			.port = I2C_BUS4,
			.target_addr = ADDR_VR_SOC,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// VR CPU1 Temperature
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x0015, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0003, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;
			0, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000000, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2856gut,
			.port = I2C_BUS4,
			.target_addr = ADDR_VR_CPU1,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// VR PVDDIO Temperature
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x0016, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0004, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;
			0, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x000000FF, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000064, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x0000007D, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2856gut,
			.port = I2C_BUS4,
			.target_addr = ADDR_VR_PVDDIO,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// VR PVDD11 Temperature
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x0017, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0005, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;
			0, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x000000FF, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000064, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x0000007D, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2856gut,
			.port = I2C_BUS4,
			.target_addr = ADDR_VR_PVDD11,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// VR CPU0 Current
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x0024, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0006, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;
			0, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000000, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2856gut,
			.port = I2C_BUS4,
			.target_addr = ADDR_VR_CPU0,
			.offset = PMBUS_READ_IOUT,
			.access_checker = vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// VR SOC Current
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x0025, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0006, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;
			0, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000000, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2856gut,
			.port = I2C_BUS4,
			.target_addr = ADDR_VR_SOC,
			.offset = PMBUS_READ_IOUT,
			.access_checker = vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// VR CPU1 Current
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x0026, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0008, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;
			0, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000000, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2856gut,
			.port = I2C_BUS4,
			.target_addr = ADDR_VR_CPU1,
			.offset = PMBUS_READ_IOUT,
			.access_checker = vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// VR PVDDIO Current
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x0027, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0009, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;
			0, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000000, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2856gut,
			.port = I2C_BUS4,
			.target_addr = ADDR_VR_PVDDIO,
			.offset = PMBUS_READ_IOUT,
			.access_checker = vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// VR PVDD11 Current
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x0028, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x000A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;
			0, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000000, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2856gut,
			.port = I2C_BUS4,
			.target_addr = ADDR_VR_PVDD11,
			.offset = PMBUS_READ_IOUT,
			.access_checker = vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// VR CPU0 Power
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x0029, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x000B, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;
			0, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000000, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2856gut,
			.port = I2C_BUS4,
			.target_addr = ADDR_VR_CPU0,
			.offset = PMBUS_READ_POUT,
			.access_checker = vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// VR SOC Power
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x002A, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x000C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;
			0, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000000, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2856gut,
			.port = I2C_BUS4,
			.target_addr = ADDR_VR_SOC,
			.offset = PMBUS_READ_POUT,
			.access_checker = vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// VR CPU1 Power
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x002B, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x000D, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;
			0, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000000, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2856gut,
			.port = I2C_BUS4,
			.target_addr = ADDR_VR_CPU1,
			.offset = PMBUS_READ_POUT,
			.access_checker = vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// VR PVDDIO Power
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x002C, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x000E, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;
			0, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000000, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2856gut,
			.port = I2C_BUS4,
			.target_addr = ADDR_VR_PVDDIO,
			.offset = PMBUS_READ_POUT,
			.access_checker = vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// VR PVDD11 Power
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x002D, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x000F, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;
			0, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000000, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2856gut,
			.port = I2C_BUS4,
			.target_addr = ADDR_VR_PVDD11,
			.offset = PMBUS_READ_POUT,
			.access_checker = vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
};

pldm_sensor_info plat_pldm_sensor_mb_temp_table[] = {
	{
		{
			// Inlet Temperature
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x0001, //uint16_t sensor_id;
			0x0089, //uint16_t entity_type;
			0x0001, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;
			0, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x000000FF, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x0000003C, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000096, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_tmp75,
			.port = I2C_BUS1,
			.target_addr = ADDR_TMP75_INLET,
			.offset = OFFSET_TMP75_TEMP,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// Outlet Temperature
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x0002, //uint16_t sensor_id;
			0x0089, //uint16_t entity_type;
			0x0002, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;
			0, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x000000FF, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000050, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000096, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_tmp75,
			.port = I2C_BUS1,
			.target_addr = ADDR_TMP75_OUTLET,
			.offset = OFFSET_TMP75_TEMP,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// FIO Temperature
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x0003, //uint16_t sensor_id;
			0x0089, //uint16_t entity_type;
			0x0003, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;
			0, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x000000FF, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000028, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000096, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_tmp75,
			.port = I2C_BUS1,
			.target_addr = ADDR_TMP75_FIO,
			.offset = OFFSET_TMP75_TEMP,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
};

pldm_sensor_info plat_pldm_sensor_cpu_table[] = {

	{
		{
			// MB_CPU_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				0x02, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x0004, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0005, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;
			-3, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x04, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //int32_t state_transition_interval;
			5, //int32_t update_interval;
			0x00030D40, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00016378, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_amd_tsi,
			.port = I2C_BUS14,
			.target_addr = SB_TSI_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = post_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.post_sensor_read_hook = post_amd_tsi_read,
		},
	},
	{
		{
			// MB_CPU_PWR_W
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				0x02, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			0x002E, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x000B, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;
			-2, //int8_t unit_modifier;
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xFF, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //int32_t state_transition_interval;
			5, //int32_t update_interval;
			0x000FFFFF, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000000, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_apml_mailbox,
			.port = I2C_BUS14,
			.target_addr = SB_RMI_ADDR, //ADDR_APML
			.offset = SBRMI_MAILBOX_PKGPWR,
			.access_checker = post_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.init_args = &apml_mailbox_init_args[0],
		},
	},
};

PDR_sensor_auxiliary_names plat_pdr_sensor_aux_names_table[] = {
	{
		// MB_INLET_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0001,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_INLET_TEMP_C",
	},
	{
		// MB_OUTLET_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0002,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_OUTLET_TEMP_C",
	},
	{
		// MB_FIO_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0003,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_FIO_TEMP_C",
	},
	{
		// MB_CPU_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0004,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_CPU_TEMP_C",
	},
	{
		// MB_DIMM_A_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0005,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_DIMM_A_TEMP_C",
	},
	{
		// MB_DIMM_B_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0006,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_DIMM_B_TEMP_C",
	},
	{
		// MB_DIMM_C_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0007,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_DIMM_C_TEMP_C",
	},
	{
		// MB_DIMM_D_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0008,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_DIMM_D_TEMP_C",
	},
	{
		// MB_DIMM_E_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0009,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_DIMM_E_TEMP_C",
	},
	{
		// MB_DIMM_F_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x000A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_DIMM_F_TEMP_C",
	},
	{
		// MB_DIMM_G_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x000B,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_DIMM_G_TEMP_C",
	},
	{
		// MB_DIMM_H_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x000C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_DIMM_H_TEMP_C",
	},
	{
		// MB_DIMM_I_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x000D,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_DIMM_I_TEMP_C",
	},
	{
		// MB_DIMM_J_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x000E,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_DIMM_J_TEMP_C",
	},
	{
		// MB_DIMM_K_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x000F,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_DIMM_K_TEMP_C",
	},
	{
		// MB_DIMM_L_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0010,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_DIMM_L_TEMP_C",
	},
	{
		// MB_SSD_BOOT_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0011,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_SSD_BOOT_TEMP_C",
	},
	{
		// MB_SSD_DATA_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0012,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_SSD_DATA_TEMP_C",
	},
	{
		// MB_VR_CPU0_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0013,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_VR_CPU0_TEMP_C",
	},
	{
		// MB_VR_SOC_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0014,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_VR_SOC_TEMP_C",
	},
	{
		// MB_VR_CPU1_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0015,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_VR_CPU1_TEMP_C",
	},
	{
		// MB_VR_PVDDIO_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0016,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_VR_PVDDIO_TEMP_C",
	},
	{
		// MB_VR_PVDD11_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0017,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_VR_PVDD11_TEMP_C",
	},
	{
		// MB_ADC_P12V_STBY_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0018,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_ADC_P12V_STBY_VOLT_V",
	},
	{
		// MB_ADC_PVDD18_S5_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0019,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_ADC_PVDD18_S5_VOLT_V",
	},
	{
		// MB_ADC_P3V3_STBY_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x001A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_ADC_P3V3_STBY_VOLT_V",
	},
	{
		// MB_ADC_P3V_BAT_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x001B,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_ADC_P3V_BAT_VOLT_V",
	},
	{
		// MB_ADC_PVDD33_S5_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x001C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_ADC_PVDD33_S5_VOLT_V",
	},
	{
		// MB_ADC_P5V_STBY_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x001D,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_ADC_P5V_STBY_VOLT_V",
	},
	{
		// MB_ADC_P12V_DIMM_0_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x001E,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_ADC_P12V_DIMM_0_VOLT_V",
	},
	{
		// MB_ADC_P12V_DIMM_1_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x001F,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_ADC_P12V_DIMM_1_VOLT_V",
	},
	{
		// MB_ADC_P1V2_STBY_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0020,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_ADC_P1V2_STBY_VOLT_V",
	},
	{
		// MB_ADC_P1V8_STBY_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0021,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_ADC_P1V8_STBY_VOLT_V",
	},
	{
		// MB_ADC_SLOT_DETECT_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0022,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_ADC_SLOT_DETECT_VOLT_V",
	},
	{
		// MB_ADC_SIDECAR_DETECT_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0023,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_ADC_SIDECAR_DETECT_VOLT_V",
	},
	{
		// MB_VR_CPU0_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0024,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_VR_CPU0_CURR_A",
	},
	{
		// MB_VR_SOC_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0025,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_VR_SOC_CURR_A",
	},
	{
		// MB_VR_CPU1_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0026,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_VR_CPU1_CURR_A",
	},
	{
		// MB_VR_PVDDIO_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0027,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_VR_PVDDIO_CURR_A",
	},
	{
		// MB_VR_PVDD11_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0028,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_VR_PVDD11_CURR_A",
	},
	{
		// MB_VR_CPU0_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0029,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_VR_CPU0_PWR_W",
	},
	{
		// MB_VR_SOC_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x002A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_VR_SOC_PWR_W",
	},
	{
		// MB_VR_CPU1_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x002B,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_VR_CPU1_PWR_W",
	},
	{
		// MB_VR_PVDDIO_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x002C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_VR_PVDDIO_PWR_W",
	},
	{
		// MB_VR_PVDD11_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x002D,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_VR_PVDD11_PWR_W",
	},
	{
		// MB_CPU_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x002E,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_CPU_PWR_W",
	},
	{
		// MB_DIMM_A_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x002F,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_DIMM_A_PWR_W",
	},
	{
		// MB_DIMM_B_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0030,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_DIMM_B_PWR_W",
	},
	{
		// MB_DIMM_C_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0031,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_DIMM_C_PWR_W",
	},
	{
		// MB_DIMM_D_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0032,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_DIMM_D_PWR_W",
	},
	{
		// MB_DIMM_E_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0033,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_DIMM_E_PWR_W",
	},
	{
		// MB_DIMM_F_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0034,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_DIMM_F_PWR_W",
	},
	{
		// MB_DIMM_G_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0035,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_DIMM_G_PWR_W",
	},
	{
		// MB_DIMM_H_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0036,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_DIMM_H_PWR_W",
	},
	{
		// MB_DIMM_I_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0037,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_DIMM_I_PWR_W",
	},
	{
		// MB_DIMM_J_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0038,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_DIMM_J_PWR_W",
	},
	{
		// MB_DIMM_K_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0039,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_DIMM_K_PWR_W",
	},
	{
		// MB_DIMM_L_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x003A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_DIMM_L_PWR_W",
	},
};

uint32_t plat_get_pdr_size(uint8_t pdr_type)
{
	int total_size = 0, i = 0;

	switch (pdr_type) {
	case PLDM_NUMERIC_SENSOR_PDR:
		for (i = 0; i < MAX_SENSOR_THREAD_ID; i++) {
			total_size += plat_pldm_sensor_get_sensor_count(i);
		}
		break;
	case PLDM_SENSOR_AUXILIARY_NAMES_PDR:
		total_size = ARRAY_SIZE(plat_pdr_sensor_aux_names_table);
		break;
	default:
		break;
	}

	return total_size;
}

pldm_sensor_thread *plat_pldm_sensor_load_thread()
{
	return pal_pldm_sensor_thread;
}

pldm_sensor_info *plat_pldm_sensor_load(int thread_id)
{
	switch (thread_id) {
	case ADC_SENSOR_THREAD_ID:
		return plat_pldm_sensor_adc_table;
	case VR_SENSOR_THREAD_ID:
		return plat_pldm_sensor_vr_table;
	case MB_TEMP_SENSOR_THREAD_ID:
		return plat_pldm_sensor_mb_temp_table;
	case CPU_SENSOR_THREAD_ID:
		return plat_pldm_sensor_cpu_table;
	default:
		LOG_ERR("Unknow pldm sensor thread id %d", thread_id);
		return NULL;
	}
}

int plat_pldm_sensor_get_sensor_count(int thread_id)
{
	int count = 0;

	switch (thread_id) {
	case ADC_SENSOR_THREAD_ID:
		count = ARRAY_SIZE(plat_pldm_sensor_adc_table);
		break;
	case VR_SENSOR_THREAD_ID:
		count = ARRAY_SIZE(plat_pldm_sensor_vr_table);
		break;
	case MB_TEMP_SENSOR_THREAD_ID:
		count = ARRAY_SIZE(plat_pldm_sensor_mb_temp_table);
		break;
	case CPU_SENSOR_THREAD_ID:
		count = ARRAY_SIZE(plat_pldm_sensor_cpu_table);
		break;
	default:
		count = -1;
		LOG_ERR("Unknow pldm sensor thread id %d", thread_id);
		break;
	}

	return count;
}

void plat_pldm_sensor_get_pdr_numeric_sensor(int thread_id, int sensor_num,
					     PDR_numeric_sensor *numeric_sensor_table)
{
	switch (thread_id) {
	case ADC_SENSOR_THREAD_ID:
		memcpy(numeric_sensor_table,
		       &plat_pldm_sensor_adc_table[sensor_num].pdr_numeric_sensor,
		       sizeof(PDR_numeric_sensor));
		break;
	case VR_SENSOR_THREAD_ID:
		memcpy(numeric_sensor_table,
		       &plat_pldm_sensor_vr_table[sensor_num].pdr_numeric_sensor,
		       sizeof(PDR_numeric_sensor));
		break;
	case MB_TEMP_SENSOR_THREAD_ID:
		memcpy(numeric_sensor_table,
		       &plat_pldm_sensor_mb_temp_table[sensor_num].pdr_numeric_sensor,
		       sizeof(PDR_numeric_sensor));
		break;
	case CPU_SENSOR_THREAD_ID:
		memcpy(numeric_sensor_table,
		       &plat_pldm_sensor_cpu_table[sensor_num].pdr_numeric_sensor,
		       sizeof(PDR_numeric_sensor));
		break;
	default:
		LOG_ERR("Unknow pldm sensor thread id %d", thread_id);
		break;
	}
}

void plat_load_numeric_sensor_pdr_table(PDR_numeric_sensor *numeric_sensor_table)
{
	int thread_id = 0, sensor_num = 0;
	int max_sensor_num = 0, current_sensor_size = 0;

	for (thread_id = 0; thread_id < MAX_SENSOR_THREAD_ID; thread_id++) {
		max_sensor_num = plat_pldm_sensor_get_sensor_count(thread_id);
		for (sensor_num = 0; sensor_num < max_sensor_num; sensor_num++) {
			plat_pldm_sensor_get_pdr_numeric_sensor(
				thread_id, sensor_num, &numeric_sensor_table[current_sensor_size]);
			current_sensor_size++;
		}
	}
}

void plat_load_aux_sensor_names_pdr_table(PDR_sensor_auxiliary_names *aux_sensor_name_table)
{
	memcpy(aux_sensor_name_table, &plat_pdr_sensor_aux_names_table,
	       sizeof(plat_pdr_sensor_aux_names_table));
}
