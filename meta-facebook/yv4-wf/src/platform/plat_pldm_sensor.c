/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * youmay not use this file except in compliance with the License.
 * Youmay obtain a copy of the License at
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
#include "vistara.h"
#include "pldm_sensor.h"
#include "pldm_monitor.h"
#include "plat_i2c.h"
#include "plat_power_seq.h"
#include "plat_hook.h"
#include "plat_pldm_sensor.h"
#include "plat_isr.h"

LOG_MODULE_REGISTER(plat_pldm_sensor);

#define VR_DEVICE_UNKNOWN 0xFF

void plat_pldm_sensor_change_vr_dev();

static struct pldm_sensor_thread pal_pldm_sensor_thread[MAX_SENSOR_THREAD_ID] = {
	// thread id, thread name
	{ ADC_SENSOR_THREAD_ID, "ADC_PLDM_SENSOR_THREAD" },
	{ TMP_SENSOR_THREAD_ID, "TMP_PLDM_SENSOR_THREAD" },
	{ INA233_SENSOR_THREAD_ID, "INA233_PLDM_SENSOR_THREAD" },
	{ VR_SENSOR_THREAD_ID, "VR_PLDM_SENSOR_THREAD" },
	{ DIMM_SENSOR_THREAD_ID, "DIMM_PLDM_SENSOR_THREAD" },
	{ ADC_MONITOR_SENSOR_THREAD_ID, "ADC_MONITOR_PLDM_SENSOR_THREAD" },
};

pldm_sensor_info plat_pldm_sensor_adc_table[] = {
	{
		{
			// WF_ADC_P3V3_STBY_VOLT_V
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

			0x0022, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x0001, //uint16_t entity_instance_number;

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

			UPDATE_INTERVAL_1S, //int32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00057288, //uint32_t warning_high;
			0x0004b320, //uint32_t warning_low;
			0x000566d0, //uint32_t critical_high;
			0x0004a3df, //uint32_t critical_low;
			0x000617c4, //uint32_t fatal_high;
			0x00038658, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_ast_adc,
			.port = ADC_PORT10,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.arg0 = 2,
			.arg1 = 1,
			.init_args = &ast_adc_init_args[1],
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// WF_ADC_P3V3_E1S_0_VOLT_V
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

			0x0023, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x0002, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support;

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
			.port = ADC_PORT11,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.arg0 = 2,
			.arg1 = 1,
			.init_args = &ast_adc_init_args[1],
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// WF_ADC_P1V2_STBY_VOLT_V
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

			0x0024, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x0003, //uint16_t entity_instance_number;

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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x000032c8, //uint32_t warning_high;
			0x00002ba4, //uint32_t warning_low;
			0x00003200, //uint32_t critical_high;
			0x00002af8, //uint32_t critical_low;
			0x00003660, //uint32_t fatal_high;
			0x00002580, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_ast_adc,
			.port = ADC_PORT2,
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
			// WF_ADC_P1V2_ASIC1_VOLT_V
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

			0x0025, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x0004, //uint16_t entity_instance_number;

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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x000032c8, //uint32_t warning_high;
			0x00002ba4, //uint32_t warning_low;
			0x00003200, //uint32_t critical_high;
			0x00002af8, //uint32_t critical_low;
			0x00003660, //uint32_t fatal_high;
			0x00002580, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_ast_adc,
			.port = ADC_PORT3,
			.access_checker = dc_access,
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
			// WF_ADC_P1V8_ASIC1_VOLT_V
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

			0x0026, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x0005, //uint16_t entity_instance_number;

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

			UPDATE_INTERVAL_1S, //int32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x0002e64a, //uint32_t warning_high;
			0x00029580, //uint32_t warning_low;
			0x0002ee00, //uint32_t critical_high;
			0x00028c58, //uint32_t critical_low;
			0x00032fa0, //uint32_t fatal_high;
			0x00023280, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_ast_adc,
			.port = ADC_PORT4,
			.access_checker = dc_access,
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
			// WF_ADC_P0V75_ASIC1_VOLT_V
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

			0x002B, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x0006, //uint16_t entity_instance_number;

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

			UPDATE_INTERVAL_1S, //int32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00014438, //uint32_t warning_high;
			0x000105b8, //uint32_t warning_low;
			0x00014b96, //uint32_t critical_high;
			0x0000ffe7, //uint32_t critical_low;
			0x000161b6, //uint32_t fatal_high;
			0x0000e8a8, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_ast_adc,
			.port = ADC_PORT9,
			.access_checker = dc_access,
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
			// WF_ADC_PVPP_AB_ASIC1_VOLT_V
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

			0x002C, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x0007, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;

			0x05, //uint8_t base_unit;
			-2, //int8_t unit_modifier;

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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00000112, //uint32_t warning_high;
			0x000000e1, //uint32_t warning_low;
			0x00000118, //uint32_t critical_high;
			0x000000dc, //uint32_t critical_low;
			0x0000012c, //uint32_t fatal_high;
			0x000000c8, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_ast_adc,
			.port = ADC_PORT5,
			.access_checker = dc_access,
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
			// WF_ADC_PVPP_CD_ASIC1_VOLT_V
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

			0x002D, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x0008, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;

			0x05, //uint8_t base_unit;
			-2, //int8_t unit_modifier;

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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00000112, //uint32_t warning_high;
			0x000000e1, //uint32_t warning_low;
			0x00000118, //uint32_t critical_high;
			0x000000dc, //uint32_t critical_low;
			0x0000012c, //uint32_t fatal_high;
			0x000000c8, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_ast_adc,
			.port = ADC_PORT6,
			.access_checker = dc_access,
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
			// WF_ADC_PVTT_AB_ASIC1_VOLT_V
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
			0x0087, //uint16_t entity_type;
			0x0009, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00000294, //uint32_t warning_high;
			0x0000021c, //uint32_t warning_low;
			0x0000029e, //uint32_t critical_high;
			0x00000210, //uint32_t critical_low;
			0x000002d0, //uint32_t fatal_high;
			0x000001e0, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_ast_adc,
			.port = ADC_PORT7,
			.access_checker = dc_access,
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
			// WF_ADC_PVTT_CD_ASIC1_VOLT_V
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

			0x002F, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x000a, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00000294, //uint32_t warning_high;
			0x0000021c, //uint32_t warning_low;
			0x0000029e, //uint32_t critical_high;
			0x00000210, //uint32_t critical_low;
			0x000002d0, //uint32_t fatal_high;
			0x000001e0, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_ast_adc,
			.port = ADC_PORT8,
			.access_checker = dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.arg0 = 1,
			.arg1 = 1,
			.init_args = &ast_adc_init_args[1],
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
};

pldm_sensor_info plat_pldm_sensor_adc_monitor_table[] = {
	{
		{
			// WF_ADC_P1V2_ASIC2_VOLT_V
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

			0x0030, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x0001, //uint16_t entity_instance_number;

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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x000032c8, //uint32_t warning_high;
			0x00002ba4, //uint32_t warning_low;
			0x00003200, //uint32_t critical_high;
			0x00002af8, //uint32_t critical_low;
			0x00003660, //uint32_t fatal_high;
			0x00002580, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_max11617,
			.port = I2C_BUS6,
			.target_addr = ADDR_MAX11617,
			.offset = ADC_PORT4,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.init_args = &max11617_init_args[0],
		},
	},
	{
		{
			// WF_ADC_P1V8_ASIC2_VOLT_V
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

			0x0031, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x0002, //uint16_t entity_instance_number;

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

			UPDATE_INTERVAL_1S, //int32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x0002e64a, //uint32_t warning_high;
			0x00029580, //uint32_t warning_low;
			0x0002ee00, //uint32_t critical_high;
			0x00028c58, //uint32_t critical_low;
			0x00032fa0, //uint32_t fatal_high;
			0x00023280, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_max11617,
			.port = I2C_BUS6,
			.target_addr = ADDR_MAX11617,
			.offset = ADC_PORT5,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.init_args = &max11617_init_args[0],
		},
	},
	{
		{
			// WF_ADC_P0V75_ASIC2_VOLT_V
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

			0x0036, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x0003, //uint16_t entity_instance_number;

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

			UPDATE_INTERVAL_1S, //int32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00014438, //uint32_t warning_high;
			0x000105b8, //uint32_t warning_low;
			0x00014b96, //uint32_t critical_high;
			0x0000ffe7, //uint32_t critical_low;
			0x000161b6, //uint32_t fatal_high;
			0x0000e8a8, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_max11617,
			.port = I2C_BUS6,
			.target_addr = ADDR_MAX11617,
			.offset = ADC_PORT6,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.init_args = &max11617_init_args[0],
		},
	},
	{
		{
			// WF_ADC_PVPP_AB_ASIC2_VOLT_V
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

			0x0037, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x0004, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;

			0x05, //uint8_t base_unit;
			-2, //int8_t unit_modifier;

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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00000112, //uint32_t warning_high;
			0x000000e1, //uint32_t warning_low;
			0x00000118, //uint32_t critical_high;
			0x000000dc, //uint32_t critical_low;
			0x0000012c, //uint32_t fatal_high;
			0x000000c8, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_max11617,
			.port = I2C_BUS6,
			.target_addr = ADDR_MAX11617,
			.offset = ADC_PORT0,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.init_args = &max11617_init_args[0],
		},
	},
	{
		{
			// WF_ADC_PVPP_CD_ASIC2_VOLT_V
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

			0x0038, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x0005, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;

			0x05, //uint8_t base_unit;
			-2, //int8_t unit_modifier;

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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00000112, //uint32_t warning_high;
			0x000000e1, //uint32_t warning_low;
			0x00000118, //uint32_t critical_high;
			0x000000dc, //uint32_t critical_low;
			0x0000012c, //uint32_t fatal_high;
			0x000000c8, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_max11617,
			.port = I2C_BUS6,
			.target_addr = ADDR_MAX11617,
			.offset = ADC_PORT1,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.init_args = &max11617_init_args[0],
		},
	},
	{
		{
			// WF_ADC_PVTT_AB_ASIC2_VOLT_V
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

			0x0039, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x0006, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00000294, //uint32_t warning_high;
			0x0000021c, //uint32_t warning_low;
			0x0000029e, //uint32_t critical_high;
			0x00000210, //uint32_t critical_low;
			0x000002d0, //uint32_t fatal_high;
			0x000001e0, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_max11617,
			.port = I2C_BUS6,
			.target_addr = ADDR_MAX11617,
			.offset = ADC_PORT2,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.init_args = &max11617_init_args[0],
		},
	},
	{
		{
			// WF_ADC_PVTT_CD_ASIC2_VOLT_V
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

			0x003A, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x0006, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00000294, //uint32_t warning_high;
			0x0000021c, //uint32_t warning_low;
			0x0000029e, //uint32_t critical_high;
			0x00000210, //uint32_t critical_low;
			0x000002d0, //uint32_t fatal_high;
			0x000001e0, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_max11617,
			.port = I2C_BUS6,
			.target_addr = ADDR_MAX11617,
			.offset = ADC_PORT3,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.init_args = &max11617_init_args[0],
		},
	},
};

pldm_sensor_info plat_pldm_sensor_tmp_table[] = {
	{
		{
			// WF_1OU_BOARD_INLET_TEMP_C
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
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
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
			0x000000FF, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x14, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000032, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000096, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_tmp75,
			.port = I2C_BUS6,
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
			// WF_CXL1_CNTR_TEMP_C
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
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
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
			0x000000FF, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x04, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000055, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_tmp461,
			.port = I2C_BUS2,
			.target_addr = ADDR_TMP461_CXL1,
			.offset = OFFSET_TMP461_TEMP,
			.access_checker = dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// WF_CXL2_CNTR_TEMP_C
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
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
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
			0x000000FF, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x04, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000055, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_tmp461,
			.port = I2C_BUS4,
			.target_addr = ADDR_TMP461_CXL2,
			.offset = OFFSET_TMP461_TEMP,
			.access_checker = dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// WF_E1S_TEMP_C
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
			0x0089, //uint16_t entity_type;
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
			0x04, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000046, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_nvme,
			.port = I2C_BUS10,
			.target_addr = ADDR_NVME,
			.offset = OFFSET_NVME_TEMP,
			.access_checker = dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
};

pldm_sensor_info plat_pldm_sensor_ina233_table[] = {
	{
		{
			// WF_INA233_P12V_STBY_VOLT_V
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

			0x0020, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0001, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00003386, //uint32_t warning_high;
			0x00002A26, //uint32_t warning_low;
			0x00003408, //uint32_t critical_high;
			0x000029EA, //uint32_t critical_low;
			0x000037FD, //uint32_t fatal_high;
			0x00002756, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_ina233,
			.port = I2C_BUS6,
			.target_addr = ADDR_INA233_P12V_STBY,
			.offset = PMBUS_READ_VOUT,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.init_args = &ina233_init_args[0],
		},
	},
	{
		{
			// WF_INA233_P12V_E1S_0_L_VOLT_V
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

			0x0021, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0002, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00003386, //uint32_t warning_high;
			0x00002A26, //uint32_t warning_low;
			0x00003408, //uint32_t critical_high;
			0x000029EA, //uint32_t critical_low;
			0x000037FD, //uint32_t fatal_high;
			0x00002756, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_ina233,
			.port = I2C_BUS6,
			.target_addr = ADDR_INA233_E1S,
			.offset = PMBUS_READ_VOUT,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.init_args = &ina233_init_args[1],
		},
	},
	{
		{
			// WF_INA233_P12V_STBY_CURR_A
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

			0x0040, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0003, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;

			0x06, //uint8_t base_unit;
			-2, //int8_t unit_modifier;

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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x15, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00000421, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x0000042C, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000433, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_ina233,
			.port = I2C_BUS6,
			.target_addr = ADDR_INA233_P12V_STBY,
			.offset = PMBUS_READ_IOUT,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.init_args = &ina233_init_args[0],
		},
	},
	{
		{
			// WF_INA233_P12V_E1S_0_L_CURR_A
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

			0x0041, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0004, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;

			0x06, //uint8_t base_unit;
			-1, //int8_t unit_modifier;

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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x15, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00000015, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000016, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x0000001E, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_ina233,
			.port = I2C_BUS6,
			.target_addr = ADDR_INA233_E1S,
			.offset = PMBUS_READ_IOUT,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.init_args = &ina233_init_args[1],
		},
	},
	{
		{
			// WF_INA233_P12V_STBY_PWR_W
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

			0x0050, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0005, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;

			0x07, //uint8_t base_unit;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support;

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
			.type = sensor_dev_ina233,
			.port = I2C_BUS6,
			.target_addr = ADDR_INA233_P12V_STBY,
			.offset = PMBUS_READ_POUT,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.init_args = &ina233_init_args[0],
		},
	},
	{
		{
			// WF_INA233_P12V_E1S_0_L_PWR_W
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

			0x0051, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0006, //uint16_t entity_instance_number;

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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x15, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x000009F6, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000A0F, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000A28, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_ina233,
			.port = I2C_BUS6,
			.target_addr = ADDR_INA233_E1S,
			.offset = PMBUS_READ_POUT,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.init_args = &ina233_init_args[1],
		},
	},
};

pldm_sensor_info plat_pldm_sensor_vr_table[] = {
	{
		{
			// WF_VR_P0V8_ASIC1_TEMP_C
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

			0x0004, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0001, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x04, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000055, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS8,
			.target_addr = ADDR_VR_P0V8_ASIC1,
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
			// WF_VR_PVDDQ_CD_ASIC1_TEMP_C
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

			0x0005, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0002, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x04, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000055, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS8,
			.target_addr = ADDR_VR_PVDDQ_CD_ASIC1,
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
			// WF_VR_P0V85_ASIC1_TEMP_C
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

			0x0006, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0003, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x04, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000055, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS8,
			.target_addr = ADDR_VR_P0V85_ASIC1,
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
			// WF_VR_PVDDQ_AB_ASIC1_TEMP_C
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

			0x0007, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0004, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x04, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000055, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS8,
			.target_addr = ADDR_VR_PVDDQ_AB_ASIC1,
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
			// WF_VR_P0V8_ASIC2_TEMP_C
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

			0x0008, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0005, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x04, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000055, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS3,
			.target_addr = ADDR_VR_P0V8_ASIC2,
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
			// WF_VR_PVDDQ_CD_ASIC2_TEMP_C
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

			0x0009, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0006, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x04, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000055, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS3,
			.target_addr = ADDR_VR_PVDDQ_CD_ASIC2,
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
			// WF_VR_P0V85_ASIC2_TEMP_C
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

			0x000A, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0007, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x04, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000055, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS3,
			.target_addr = ADDR_VR_P0V85_ASIC2,
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
			// WF_VR_PVDDQ_AB_ASIC2_TEMP_C
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

			0x000B, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0008, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x04, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000055, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS3,
			.target_addr = ADDR_VR_PVDDQ_AB_ASIC2,
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
			// WF_VR_P0V8_ASIC1_VOLT_V
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
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00000370, //uint32_t warning_high;
			0x000002D0, //uint32_t warning_low;
			0x00000384, //uint32_t critical_high;
			0x000002C0, //uint32_t critical_low;
			0x000003C0, //uint32_t fatal_high;
			0x00000280, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS8,
			.target_addr = ADDR_VR_P0V8_ASIC1,
			.offset = PMBUS_READ_VOUT,
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
			// WF_VR_PVDDQ_CD_ASIC1_VOLT_V
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00003200, //uint32_t warning_high;
			0x00002BA4, //uint32_t warning_low;
			0x000032C8, //uint32_t critical_high;
			0x00002AF8, //uint32_t critical_low;
			0x00003660, //uint32_t fatal_high;
			0x00002580, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS8,
			.target_addr = ADDR_VR_PVDDQ_CD_ASIC1,
			.offset = PMBUS_READ_VOUT,
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
			// WF_VR_P0V85_ASIC1_VOLT_V
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
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;

			0x05, //uint8_t base_unit;
			-9, //int8_t unit_modifier;

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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x39387000, //uint32_t warning_high;
			0x2AEA5400, //uint32_t warning_low;
			0x3A855B46, //uint32_t critical_high;
			0x29F63000, //uint32_t critical_low;
			0x3E6C1D17, //uint32_t fatal_high;
			0x2625A000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS8,
			.target_addr = ADDR_VR_P0V85_ASIC1,
			.offset = PMBUS_READ_VOUT,
			.access_checker = vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
			.post_sensor_read_hook = post_p085v_voltage_read,
		},
	},
	{
		{
			// WF_VR_PVDDQ_AB_ASIC1_VOLT_V
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
			0x000000FF, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00003200, //uint32_t warning_high;
			0x00002BA4, //uint32_t warning_low;
			0x000032C8, //uint32_t critical_high;
			0x00002AF8, //uint32_t critical_low;
			0x00003660, //uint32_t fatal_high;
			0x00002580, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS8,
			.target_addr = ADDR_VR_PVDDQ_AB_ASIC1,
			.offset = PMBUS_READ_VOUT,
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
			// WF_VR_P0V8_ASIC2_VOLT_V
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

			0x0032, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x000D, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00000370, //uint32_t warning_high;
			0x000002D0, //uint32_t warning_low;
			0x00000384, //uint32_t critical_high;
			0x000002C0, //uint32_t critical_low;
			0x000003C0, //uint32_t fatal_high;
			0x00000280, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS3,
			.target_addr = ADDR_VR_P0V8_ASIC2,
			.offset = PMBUS_READ_VOUT,
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
			// WF_VR_PVDDQ_CD_ASIC2_VOLT_V
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

			0x0033, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x000E, //uint16_t entity_instance_number;

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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00003200, //uint32_t warning_high;
			0x00002BA4, //uint32_t warning_low;
			0x000032C8, //uint32_t critical_high;
			0x00002AF8, //uint32_t critical_low;
			0x00003660, //uint32_t fatal_high;
			0x00002580, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS3,
			.target_addr = ADDR_VR_PVDDQ_CD_ASIC2,
			.offset = PMBUS_READ_VOUT,
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
			// WF_VR_P0V85_ASIC2_VOLT_V
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

			0x0034, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x000F, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;

			0x05, //uint8_t base_unit;
			-9, //int8_t unit_modifier;

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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x39387000, //uint32_t warning_high;
			0x2AEA5400, //uint32_t warning_low;
			0x3A855B46, //uint32_t critical_high;
			0x29F63000, //uint32_t critical_low;
			0x3E6C1D17, //uint32_t fatal_high;
			0x2625A000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS3,
			.target_addr = ADDR_VR_P0V85_ASIC2,
			.offset = PMBUS_READ_VOUT,
			.access_checker = vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
			.post_sensor_read_hook = post_p085v_voltage_read,
		},
	},
	{
		{
			// WF_VR_PVDDQ_AB_ASIC2_VOLT_V
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

			0x0035, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0010, //uint16_t entity_instance_number;

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
			0x000000FF, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00003200, //uint32_t warning_high;
			0x00002BA4, //uint32_t warning_low;
			0x000032C8, //uint32_t critical_high;
			0x00002AF8, //uint32_t critical_low;
			0x00003660, //uint32_t fatal_high;
			0x00002580, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS3,
			.target_addr = ADDR_VR_PVDDQ_AB_ASIC2,
			.offset = PMBUS_READ_VOUT,
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
			// WF_VR_P0V8_ASIC1_CURR_A
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

			0x0042, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0011, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;

			0x06, //uint8_t base_unit;
			-2, //int8_t unit_modifier;

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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x15, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00000091, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000097, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x000000C8, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS8,
			.target_addr = ADDR_VR_P0V8_ASIC1,
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
			// WF_VR_PVDDQ_CD_ASIC1_CURR_A
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

			0x0043, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0012, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;

			0x06, //uint8_t base_unit;
			-1, //int8_t unit_modifier;

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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x15, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00000078, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x0000007E, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x000000A0, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS8,
			.target_addr = ADDR_VR_PVDDQ_CD_ASIC1,
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
			// WF_VR_P0V85_ASIC1_CURR_A
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

			0x0044, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0013, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;

			0x06, //uint8_t base_unit;
			-2, //int8_t unit_modifier;

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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x15, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x000002EC, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000310, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x000004B0, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS8,
			.target_addr = ADDR_VR_P0V85_ASIC1,
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
			// WF_VR_PVDDQ_AB_ASIC1_CURR_A
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

			0x0045, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0014, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;

			0x06, //uint8_t base_unit;
			-1, //int8_t unit_modifier;

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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x15, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00000078, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x0000007E, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x000000A0, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS8,
			.target_addr = ADDR_VR_PVDDQ_AB_ASIC1,
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
			// WF_VR_P0V8_ASIC2_CURR_A
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

			0x0046, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0015, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;

			0x06, //uint8_t base_unit;
			-2, //int8_t unit_modifier;

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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x15, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00000091, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000097, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x000000C8, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS3,
			.target_addr = ADDR_VR_P0V8_ASIC2,
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
			// WF_VR_PVDDQ_CD_ASIC2_CURR_A
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

			0x0047, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0016, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;

			0x06, //uint8_t base_unit;
			-1, //int8_t unit_modifier;

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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x15, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00000078, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x0000007E, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x000000A0, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS3,
			.target_addr = ADDR_VR_PVDDQ_CD_ASIC2,
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
			// WF_VR_P0V85_ASIC2_CURR_A
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

			0x0048, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0017, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;

			0x06, //uint8_t base_unit;
			-2, //int8_t unit_modifier;

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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x15, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x000002EC, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000310, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x000004B0, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS3,
			.target_addr = ADDR_VR_P0V85_ASIC2,
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
			// WF_VR_PVDDQ_AB_ASIC2_CURR_A
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

			0x0049, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0018, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;

			0x06, //uint8_t base_unit;
			-1, //int8_t unit_modifier;

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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x15, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00000078, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x0000007E, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x000000A0, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS3,
			.target_addr = ADDR_VR_PVDDQ_AB_ASIC2,
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
			// WF_VR_P0V8_ASIC1_PWR_W
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

			0x0052, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0019, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;

			0x07, //uint8_t base_unit;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x15, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x000004FC, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x0000054F, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000780, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS8,
			.target_addr = ADDR_VR_P0V8_ASIC1,
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
			// WF_VR_PVDDQ_CD_ASIC1_PWR_W
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

			0x0053, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x001A, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;

			0x07, //uint8_t base_unit;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x15, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00003C00, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00003FFC, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00005700, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS8,
			.target_addr = ADDR_VR_PVDDQ_CD_ASIC1,
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
			// WF_VR_P0V85_ASIC1_PWR_W
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

			0x0054, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x001B, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;

			0x07, //uint8_t base_unit;
			-8, //int8_t unit_modifier;

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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x15, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x2ACD0800, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x2DE1622E, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x4AE82221, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS8,
			.target_addr = ADDR_VR_P0V85_ASIC1,
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
			// WF_VR_PVDDQ_AB_ASIC1_PWR_W
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

			0x0055, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x001C, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;

			0x07, //uint8_t base_unit;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x15, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00003C00, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00003FFC, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00005700, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS8,
			.target_addr = ADDR_VR_PVDDQ_AB_ASIC1,
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
			// WF_VR_P0V8_ASIC2_PWR_W
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

			0x0056, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x001D, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;

			0x07, //uint8_t base_unit;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x15, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x000004FC, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x0000054F, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000780, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS3,
			.target_addr = ADDR_VR_P0V8_ASIC2,
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
			// WF_VR_PVDDQ_CD_ASIC2_PWR_W
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

			0x0057, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x001E, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;

			0x07, //uint8_t base_unit;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x15, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00003C00, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00003FFC, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00005700, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS3,
			.target_addr = ADDR_VR_PVDDQ_CD_ASIC2,
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
			// WF_VR_P0V85_ASIC2_PWR_W
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

			0x0058, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x001F, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;

			0x07, //uint8_t base_unit;
			-8, //int8_t unit_modifier;

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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x15, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x2ACD0800, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x2DE1622E, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x4AE82221, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS3,
			.target_addr = ADDR_VR_P0V85_ASIC2,
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
			// WF_VR_PVDDQ_AB_ASIC2_PWR_W
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

			0x0059, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0020, //uint16_t entity_instance_number;

			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;

			0x07, //uint8_t base_unit;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x15, //uint8_t range_field_support;

			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;

			0x00003C00, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00003FFC, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00005700, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS3,
			.target_addr = ADDR_VR_PVDDQ_AB_ASIC2,
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

pldm_sensor_info plat_pldm_sensor_dimm_table[] = {
	{
		{
			// WF_DIMM_ASIC1_DIMM_A_EVENT_TEMP_C
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
			SENSOR_ID_ASIC1_DIMM_A_TEMP, //uint16_t sensor_id;
			0x0089, //uint16_t entity_type;
			0x0001, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;
			-2, //int8_t unit_modifier;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x05, //uint8_t range_field_format;
			0x3E, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //int32_t warning_high;
			0x00001D4C, //int32_t warning_low;
			0x0000251C, //int32_t critical_high;
			0x00000FA0, //int32_t critical_low;
			0x000030D4, //int32_t fatal_high;
			-0x000007D0, //int32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_vistara,
			.port = CXL_ID_0,
			.target_addr = DIMMA_ID,
			.access_checker = cxl_ready_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// WF_DIMM_ASIC1_DIMM_B_EVENT_TEMP_C
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
			SENSOR_ID_ASIC1_DIMM_B_TEMP, //uint16_t sensor_id;
			0x0089, //uint16_t entity_type;
			0x0002, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;
			-2, //int8_t unit_modifier;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x05, //uint8_t range_field_format;
			0x3E, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //int32_t warning_high;
			0x00001D4C, //int32_t warning_low;
			0x0000251C, //int32_t critical_high;
			0x00000FA0, //int32_t critical_low;
			0x000030D4, //int32_t fatal_high;
			-0x000007D0, //int32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_vistara,
			.port = CXL_ID_0,
			.target_addr = DIMMB_ID,
			.access_checker = cxl_ready_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// WF_DIMM_ASIC1_DIMM_C_EVENT_TEMP_C
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
			SENSOR_ID_ASIC1_DIMM_C_TEMP, //uint16_t sensor_id;
			0x0089, //uint16_t entity_type;
			0x0003, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;
			-2, //int8_t unit_modifier;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x05, //uint8_t range_field_format;
			0x3E, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //int32_t warning_high;
			0x00001D4C, //int32_t warning_low;
			0x0000251C, //int32_t critical_high;
			0x00000FA0, //int32_t critical_low;
			0x000030D4, //int32_t fatal_high;
			-0x000007D0, //int32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_vistara,
			.port = CXL_ID_0,
			.target_addr = DIMMC_ID,
			.access_checker = cxl_ready_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// WF_DIMM_ASIC1_DIMM_D_EVENT_TEMP_C
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
			SENSOR_ID_ASIC1_DIMM_D_TEMP, //uint16_t sensor_id;
			0x0089, //uint16_t entity_type;
			0x0004, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;
			-2, //int8_t unit_modifier;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x05, //uint8_t range_field_format;
			0x3E, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //int32_t warning_high;
			0x00001D4C, //int32_t warning_low;
			0x0000251C, //int32_t critical_high;
			0x00000FA0, //int32_t critical_low;
			0x000030D4, //int32_t fatal_high;
			-0x000007D0, //int32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_vistara,
			.port = CXL_ID_0,
			.target_addr = DIMMD_ID,
			.access_checker = cxl_ready_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// WF_DIMM_ASIC2_DIMM_A_EVENT_TEMP_C
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
			SENSOR_ID_ASIC2_DIMM_A_TEMP, //uint16_t sensor_id;
			0x0089, //uint16_t entity_type;
			0x0005, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;
			-2, //int8_t unit_modifier;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x05, //uint8_t range_field_format;
			0x3E, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //int32_t warning_high;
			0x00001D4C, //int32_t warning_low;
			0x0000251C, //int32_t critical_high;
			0x00000FA0, //int32_t critical_low;
			0x000030D4, //int32_t fatal_high;
			-0x000007D0, //int32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_vistara,
			.port = CXL_ID_1,
			.target_addr = DIMMA_ID,
			.access_checker = cxl_ready_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// WF_DIMM_ASIC2_DIMM_B_EVENT_TEMP_C
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
			SENSOR_ID_ASIC2_DIMM_B_TEMP, //uint16_t sensor_id;
			0x0089, //uint16_t entity_type;
			0x0006, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;
			-2, //int8_t unit_modifier;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x05, //uint8_t range_field_format;
			0x3E, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //int32_t warning_high;
			0x00001D4C, //int32_t warning_low;
			0x0000251C, //int32_t critical_high;
			0x00000FA0, //int32_t critical_low;
			0x000030D4, //int32_t fatal_high;
			-0x000007D0, //int32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_vistara,
			.port = CXL_ID_1,
			.target_addr = DIMMB_ID,
			.access_checker = cxl_ready_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// WF_DIMM_ASIC2_DIMM_C_EVENT_TEMP_C
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
			SENSOR_ID_ASIC2_DIMM_C_TEMP, //uint16_t sensor_id;
			0x0089, //uint16_t entity_type;
			0x0007, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;
			-2, //int8_t unit_modifier;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x05, //uint8_t range_field_format;
			0x3E, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //int32_t warning_high;
			0x00001D4C, //int32_t warning_low;
			0x0000251C, //int32_t critical_high;
			0x00000FA0, //int32_t critical_low;
			0x000030D4, //int32_t fatal_high;
			-0x000007D0, //int32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_vistara,
			.port = CXL_ID_1,
			.target_addr = DIMMC_ID,
			.access_checker = cxl_ready_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// WF_DIMM_ASIC2_DIMM_D_EVENT_TEMP_C
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
			SENSOR_ID_ASIC2_DIMM_D_TEMP, //uint16_t sensor_id;
			0x0089, //uint16_t entity_type;
			0x0008, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;
			-2, //int8_t unit_modifier;
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
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x05, //uint8_t range_field_format;
			0x3E, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //int32_t warning_high;
			0x00001D4C, //int32_t warning_low;
			0x0000251C, //int32_t critical_high;
			0x00000FA0, //int32_t critical_low;
			0x000030D4, //int32_t fatal_high;
			-0x000007D0, //int32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_vistara,
			.port = CXL_ID_1,
			.target_addr = DIMMD_ID,
			.access_checker = cxl_ready_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
};

// clang-format off
PDR_sensor_auxiliary_names plat_pdr_sensor_aux_names_table[] = {
	{
		// WF_1OU_BOARD_INLET_TEMP_C
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
		.sensorName = u"WF_1OU_BOARD_INLET_TEMP_C",
	},
	{
		// WF_CXL1_CNTR_TEMP_C
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
		.sensorName = u"WF_CXL1_CNTR_TEMP_C",
	},
	{
		// WF_CXL2_CNTR_TEMP_C
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
		.sensorName = u"WF_CXL2_CNTR_TEMP_C",
	},
	{
		// WF_VR_P0V8_ASIC1_TEMP_C
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
		.sensorName = u"WF_VR_P0V8_ASIC1_TEMP_C",
	},
	{
		// WF_VR_PVDDQ_CD_ASIC1_TEMP_C
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
		.sensorName = u"WF_VR_PVDDQ_CD_ASIC1_TEMP_C",
	},
	{
		// WF_VR_P0V85_ASIC1_TEMP_C
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
		.sensorName = u"WF_VR_P0V85_ASIC1_TEMP_C",
	},
	{
		// WF_VR_PVDDQ_AB_ASIC1_TEMP_C
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
		.sensorName = u"WF_VR_PVDDQ_AB_ASIC1_TEMP_C",
	},
	{
		// WF_VR_P0V8_ASIC2_TEMP_C
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
		.sensorName = u"WF_VR_P0V8_ASIC2_TEMP_C",
	},
	{
		// WF_VR_PVDDQ_CD_ASIC2_TEMP_C
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
		.sensorName = u"WF_VR_PVDDQ_CD_ASIC2_TEMP_C",
	},
	{
		// WF_VR_P0V85_ASIC2_TEMP_C
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
		.sensorName = u"WF_VR_P0V85_ASIC2_TEMP_C",
	},
	{
		// WF_VR_PVDDQ_AB_ASIC2_TEMP_C
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
		.sensorName = u"WF_VR_PVDDQ_AB_ASIC2_TEMP_C",
	},
	{
		// WF_E1S_TEMP_C
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
		.sensorName = u"WF_E1S_TEMP_C",
	},
	{
		// WF_INA233_P12V_STBY_VOLT_V
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
		.sensorName = u"WF_INA233_P12V_STBY_VOLT_V",
	},
	{
		// WF_INA233_P12V_E1S_0_L_VOLT_V
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
		.sensorName = u"WF_INA233_P12V_E1S_0_L_VOLT_V",
	},
	{
		// WF_ADC_P3V3_STBY_VOLT_V
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
		.sensorName = u"WF_ADC_P3V3_STBY_VOLT_V",
	},
	{
		// WF_ADC_P3V3_E1S_0_VOLT_V
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
		.sensorName = u"WF_ADC_P3V3_E1S_0_VOLT_V",
	},
	{
		// WF_ADC_P1V2_STBY_VOLT_V
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
		.sensorName = u"WF_ADC_P1V2_STBY_VOLT_V",
	},
	{
		// WF_ADC_P1V2_ASIC1_VOLT_V
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
		.sensorName = u"WF_ADC_P1V2_ASIC1_VOLT_V",
	},
	{
		// WF_ADC_P1V8_ASIC1_VOLT_V
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
		.sensorName = u"WF_ADC_P1V8_ASIC1_VOLT_V",
	},
	{
		// WF_VR_P0V8_ASIC1_VOLT_V
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
		.sensorName = u"WF_VR_P0V8_ASIC1_VOLT_V",
	},
	{
		// WF_VR_PVDDQ_CD_ASIC1_VOLT_V
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
		.sensorName = u"WF_VR_PVDDQ_CD_ASIC1_VOLT_V",
	},
	{
		// WF_VR_P0V85_ASIC1_VOLT_V
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
		.sensorName = u"WF_VR_P0V85_ASIC1_VOLT_V",
	},
	{
		// WF_VR_PVDDQ_AB_ASIC1_VOLT_V
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
		.sensorName = u"WF_VR_PVDDQ_AB_ASIC1_VOLT_V",
	},
	{
		// WF_ADC_P0V75_ASIC1_VOLT_V
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
		.sensorName = u"WF_ADC_P0V75_ASIC1_VOLT_V",
	},
	{
		// WF_ADC_PVPP_AB_ASIC1_VOLT_V
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
		.sensorName = u"WF_ADC_PVPP_AB_ASIC1_VOLT_V",
	},
	{
		// WF_ADC_PVPP_CD_ASIC1_VOLT_V
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
		.sensorName = u"WF_ADC_PVPP_CD_ASIC1_VOLT_V",
	},
	{
		// WF_ADC_PVTT_AB_ASIC1_VOLT_V
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
		.sensorName = u"WF_ADC_PVTT_AB_ASIC1_VOLT_V",
	},
	{
		// WF_ADC_PVTT_CD_ASIC1_VOLT_V
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
		.sensorName = u"WF_ADC_PVTT_CD_ASIC1_VOLT_V",
	},
	{
		// WF_ADC_P1V2_ASIC2_VOLT_V
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
		.sensorName = u"WF_ADC_P1V2_ASIC2_VOLT_V",
	},
	{
		// WF_ADC_P1V8_ASIC2_VOLT_V
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
		.sensorName = u"WF_ADC_P1V8_ASIC2_VOLT_V",
	},
	{
		// WF_VR_P0V8_ASIC2_VOLT_V
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
		.sensorName = u"WF_VR_P0V8_ASIC2_VOLT_V",
	},
	{
		// WF_VR_PVDDQ_CD_ASIC2_VOLT_V
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
		.sensorName = u"WF_VR_PVDDQ_CD_ASIC2_VOLT_V",
	},
	{
		// WF_VR_P0V85_ASIC2_VOLT_V
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
		.sensorName = u"WF_VR_P0V85_ASIC2_VOLT_V",
	},
	{
		// WF_VR_PVDDQ_AB_ASIC2_VOLT_V
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
		.sensorName = u"WF_VR_PVDDQ_AB_ASIC2_VOLT_V",
	},
	{
		// WF_ADC_P0V75_ASIC2_VOLT_V
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
		.sensorName = u"WF_ADC_P0V75_ASIC2_VOLT_V",
	},
	{
		// WF_ADC_PVPP_AB_ASIC2_VOLT_V
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
		.sensorName = u"WF_ADC_PVPP_AB_ASIC2_VOLT_V ",
	},
	{
		// WF_ADC_PVPP_CD_ASIC2_VOLT_V
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
		.sensorName = u"WF_ADC_PVPP_CD_ASIC2_VOLT_V ",
	},
	{
		// WF_ADC_PVTT_AB_ASIC2_VOLT_V
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
		.sensorName = u"WF_ADC_PVTT_AB_ASIC2_VOLT_V ",
	},
	{
		// WF_ADC_PVTT_CD_ASIC2_VOLT_V
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
		.sensorName = u"WF_ADC_PVTT_CD_ASIC2_VOLT_V ",
	},
	{
		// WF_INA233_P12V_STBY_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0040,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"WF_INA233_P12V_STBY_CURR_A",
	},
	{
		// WF_INA233_P12V_E1S_0_L_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0041,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"WF_INA233_P12V_E1S_0_L_CURR_A",
	},
	{
		// WF_VR_P0V8_ASIC1_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0042,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"WF_VR_P0V8_ASIC1_CURR_A",
	},
	{
		// WF_VR_PVDDQ_CD_ASIC1_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0043,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"WF_VR_PVDDQ_CD_ASIC1_CURR_A",
	},
	{
		// WF_VR_P0V85_ASIC1_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0044,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"WF_VR_P0V85_ASIC1_CURR_A",
	},
	{
		// WF_VR_PVDDQ_AB_ASIC1_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0045,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"WF_VR_PVDDQ_AB_ASIC1_CURR_A",
	},
	{
		// WF_VR_P0V8_ASIC2_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0046,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"WF_VR_P0V8_ASIC2_CURR_A",
	},
	{
		// WF_VR_PVDDQ_CD_ASIC2_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0047,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"WF_VR_PVDDQ_CD_ASIC2_CURR_A",
	},
	{
		// WF_VR_P0V85_ASIC2_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0048,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"WF_VR_P0V85_ASIC2_CURR_A",
	},
	{
		// WF_VR_PVDDQ_AB_ASIC2_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0049,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"WF_VR_PVDDQ_AB_ASIC2_CURR_A",
	},
	{
		// WF_INA233_P12V_STBY_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0050,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"WF_INA233_P12V_STBY_PWR_W",
	},
	{
		// WF_INA233_P12V_E1S_0_L_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0051,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"WF_INA233_P12V_E1S_0_L_PWR_W",
	},
	{
		// WF_VR_P0V8_ASIC1_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0052,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"WF_VR_P0V8_ASIC1_PWR_W",
	},
	{
		// WF_VR_PVDDQ_CD_ASIC1_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0053,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"WF_VR_PVDDQ_CD_ASIC1_PWR_W",
	},
	{
		// WF_VR_P0V85_ASIC1_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0054,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"WF_VR_P0V85_ASIC1_PWR_W",
	},
	{
		// WF_VR_PVDDQ_AB_ASIC1_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0055,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"WF_VR_PVDDQ_AB_ASIC1_PWR_W",
	},
	{
		// WF_VR_P0V8_ASIC2_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0056,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"WF_VR_P0V8_ASIC2_PWR_W",
	},
	{
		// WF_VR_PVDDQ_CD_ASIC2_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0057,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"WF_VR_PVDDQ_CD_ASIC2_PWR_W",
	},
	{
		// WF_VR_P0V85_ASIC2_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0058,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"WF_VR_P0V85_ASIC2_PWR_W",
	},
	{
		// WF_VR_PVDDQ_AB_ASIC2_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0059,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"WF_VR_PVDDQ_AB_ASIC2_PWR_W",
	},
	{
		// WF_ASIC1_DIMM_A_TEMP_C 
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_ID_ASIC1_DIMM_A_TEMP,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"WF_ASIC1_DIMM_A_TEMP_C",
	},
	{
		// WF_ASIC1_DIMM_B_TEMP_C 
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_ID_ASIC1_DIMM_B_TEMP,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"WF_ASIC1_DIMM_B_TEMP_C",
	},
	{
		// WF_ASIC1_DIMM_C_TEMP_C 
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_ID_ASIC1_DIMM_C_TEMP,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"WF_ASIC1_DIMM_C_TEMP_C",
	},
	{
		// WF_ASIC1_DIMM_D_TEMP_C 
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_ID_ASIC1_DIMM_D_TEMP,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"WF_ASIC1_DIMM_D_TEMP_C",
	},
	{
		// WF_ASIC2_DIMM_A_TEMP_C 
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_ID_ASIC2_DIMM_A_TEMP,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"WF_ASIC2_DIMM_A_TEMP_C",
	},
	{
		// WF_ASIC2_DIMM_B_TEMP_C 
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_ID_ASIC2_DIMM_B_TEMP,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"WF_ASIC2_DIMM_B_TEMP_C",
	},
	{
		// WF_ASIC2_DIMM_C_TEMP_C 
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_ID_ASIC2_DIMM_C_TEMP,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"WF_ASIC2_DIMM_C_TEMP_C",
	},
	{
		// WF_ASIC2_DIMM_D_TEMP_C 
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_ID_ASIC2_DIMM_D_TEMP,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"WF_ASIC2_DIMM_D_TEMP_C",
	},
};
// clang-format on
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
	case TMP_SENSOR_THREAD_ID:
		return plat_pldm_sensor_tmp_table;
	case INA233_SENSOR_THREAD_ID:
		return plat_pldm_sensor_ina233_table;
	case VR_SENSOR_THREAD_ID:
		plat_pldm_sensor_change_vr_dev();
		return plat_pldm_sensor_vr_table;
	case DIMM_SENSOR_THREAD_ID:
		return plat_pldm_sensor_dimm_table;
	case ADC_MONITOR_SENSOR_THREAD_ID:
		return plat_pldm_sensor_adc_monitor_table;
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
	case TMP_SENSOR_THREAD_ID:
		count = ARRAY_SIZE(plat_pldm_sensor_tmp_table);
		break;
	case INA233_SENSOR_THREAD_ID:
		count = ARRAY_SIZE(plat_pldm_sensor_ina233_table);
		break;
	case VR_SENSOR_THREAD_ID:
		count = ARRAY_SIZE(plat_pldm_sensor_vr_table);
		break;
	case DIMM_SENSOR_THREAD_ID:
		count = ARRAY_SIZE(plat_pldm_sensor_dimm_table);
		break;
	case ADC_MONITOR_SENSOR_THREAD_ID:
		count = ARRAY_SIZE(plat_pldm_sensor_adc_monitor_table);
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
	case TMP_SENSOR_THREAD_ID:
		memcpy(numeric_sensor_table,
		       &plat_pldm_sensor_tmp_table[sensor_num].pdr_numeric_sensor,
		       sizeof(PDR_numeric_sensor));
		break;
	case INA233_SENSOR_THREAD_ID:
		memcpy(numeric_sensor_table,
		       &plat_pldm_sensor_ina233_table[sensor_num].pdr_numeric_sensor,
		       sizeof(PDR_numeric_sensor));
		break;
	case VR_SENSOR_THREAD_ID:
		memcpy(numeric_sensor_table,
		       &plat_pldm_sensor_vr_table[sensor_num].pdr_numeric_sensor,
		       sizeof(PDR_numeric_sensor));
		break;
	case DIMM_SENSOR_THREAD_ID:
		memcpy(numeric_sensor_table,
		       &plat_pldm_sensor_dimm_table[sensor_num].pdr_numeric_sensor,
		       sizeof(PDR_numeric_sensor));
		break;
	case ADC_MONITOR_SENSOR_THREAD_ID:
		memcpy(numeric_sensor_table,
		       &plat_pldm_sensor_adc_monitor_table[sensor_num].pdr_numeric_sensor,
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

void plat_pldm_sensor_change_vr_dev()
{
	uint8_t vr_dev = VR_DEVICE_UNKNOWN;
	if (plat_pldm_sensor_get_vr_dev(&vr_dev) != GET_VR_DEV_SUCCESS) {
		LOG_ERR("Unable to change the VR device due to its unknown status.");
		return;
	}

	for (int index = 0; index < plat_pldm_sensor_get_sensor_count(VR_SENSOR_THREAD_ID);
	     index++) {
		plat_pldm_sensor_vr_table[index].pldm_sensor_cfg.type = vr_dev;
	}
}

uint8_t plat_pldm_sensor_get_vr_dev(uint8_t *vr_dev)
{
	/*
	 * Get VR type from IOE3 P14
	 * High - MPS
	 * Low - IFX
	 */

	uint8_t reg = 0;
	if (get_ioe_value(ADDR_IOE3, TCA9555_INPUT_PORT_REG_1, &reg) == -1) {
		LOG_ERR("Failed to get the VR type.");
		*vr_dev = VR_DEVICE_UNKNOWN;
		return GET_VR_DEV_FAILED;
	}

	if (GETBIT(reg, IOE_P14)) {
		*vr_dev = sensor_dev_mp2971;
	} else {
		*vr_dev = sensor_dev_xdpe12284c;
	}

	return GET_VR_DEV_SUCCESS;
}
