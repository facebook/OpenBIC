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
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_pldm_sensor);

#define VR_DEVICE_UNKNOWN 0xFF
static K_MUTEX_DEFINE(cxl_dimm_mutex);

void plat_pldm_sensor_change_vr_dev();
void plat_pldm_sensor_change_adc_monitor_dev();
void plat_pldm_sensor_change_asic_tmp_dev();
void plat_pldm_sensor_change_ina233_dev();

float cxl_dimm_temp[MAX_CXL_ID][MAX_DIMM_ID] = { { -1, -1, -1, -1 }, { -1, -1, -1, -1 } };

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

			0x000566d0, //uint32_t warning_high;
			0x0004b320, //uint32_t warning_low;
			0x00057288, //uint32_t critical_high;
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

			0x000566d0, //uint32_t warning_high;
			0x0004b320, //uint32_t warning_low;
			0x00057288, //uint32_t critical_high;
			0x0004a3df, //uint32_t critical_low;
			0x000617c4, //uint32_t fatal_high;
			0x00038658, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_ast_adc,
			.port = ADC_PORT11,
			.access_checker = e1s_pwrgd_access,
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

			0x00003200, //uint32_t warning_high;
			0x00002ba4, //uint32_t warning_low;
			0x000032c8, //uint32_t critical_high;
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

			0x00003200, //uint32_t warning_high;
			0x00002ba4, //uint32_t warning_low;
			0x000032c8, //uint32_t critical_high;
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

			0x00003200, //uint32_t warning_high;
			0x00002ba4, //uint32_t warning_low;
			0x000032c8, //uint32_t critical_high;
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
			.access_checker = dc_access,
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
			.access_checker = dc_access,
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
			.access_checker = dc_access,
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
			.access_checker = dc_access,
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
			.access_checker = dc_access,
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
			.access_checker = dc_access,
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
			.access_checker = dc_access,
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
			0xC6, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;

			UPDATE_INTERVAL_1S, //int32_t update_interval;
			0x00000000, //uint32_t max_readable;
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
			0xC2, //uint8_t supported_thresholds;
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
			0x0000006E, //uint32_t fatal_high;
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
			0xC2, //uint8_t supported_thresholds;
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
			0x0000006E, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_tmp461,
			.port = I2C_BUS4,
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
			0xC2, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x04, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x0000004B, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000055, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_nvme,
			.port = I2C_BUS10,
			.target_addr = ADDR_NVME,
			.offset = OFFSET_NVME_TEMP,
			.access_checker = e1s_pwrgd_access,
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
			.access_checker = e1s_pwrgd_access,
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
			0xC7, //uint8_t supported_thresholds;
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
			0x00000436, //uint32_t fatal_high;
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
			0xC7, //uint8_t supported_thresholds;
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
			.access_checker = e1s_pwrgd_access,
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
			0xC7, //uint8_t supported_thresholds;
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

			0x00003675, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00003791, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00003C5B, //uint32_t fatal_high;
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
			0xC7, //uint8_t supported_thresholds;
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
			.access_checker = e1s_pwrgd_access,
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
			0xC2, //uint8_t supported_thresholds;
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
			0x00000064, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x0000007D, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS8,
			.target_addr = ADDR_VR_P0V8_ASIC1,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = cxl1_vr_access,
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
			0xC2, //uint8_t supported_thresholds;
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
			0x00000064, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x0000007D, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS8,
			.target_addr = ADDR_VR_PVDDQ_CD_ASIC1,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = cxl1_vr_access,
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
			0xC2, //uint8_t supported_thresholds;
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
			0x00000064, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x0000007D, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS8,
			.target_addr = ADDR_VR_P0V85_ASIC1,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = cxl1_vr_access,
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
			0xC2, //uint8_t supported_thresholds;
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
			0x00000064, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x0000007D, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS8,
			.target_addr = ADDR_VR_PVDDQ_AB_ASIC1,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = cxl1_vr_access,
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
			0xC2, //uint8_t supported_thresholds;
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
			0x00000064, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x0000007D, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS3,
			.target_addr = ADDR_VR_P0V8_ASIC2,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = cxl2_vr_access,
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
			0xC2, //uint8_t supported_thresholds;
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
			0x00000064, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x0000007D, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS3,
			.target_addr = ADDR_VR_PVDDQ_CD_ASIC2,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = cxl2_vr_access,
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
			0xC2, //uint8_t supported_thresholds;
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
			0x00000064, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x0000007D, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS3,
			.target_addr = ADDR_VR_P0V85_ASIC2,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = cxl2_vr_access,
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
			0xC2, //uint8_t supported_thresholds;
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
			0x00000064, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x0000007D, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS3,
			.target_addr = ADDR_VR_PVDDQ_AB_ASIC2,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = cxl2_vr_access,
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
			.access_checker = cxl1_vr_access,
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
			.access_checker = cxl1_vr_access,
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
			.access_checker = cxl1_vr_access,
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
			.target_addr = ADDR_VR_PVDDQ_AB_ASIC1,
			.offset = PMBUS_READ_VOUT,
			.access_checker = cxl1_vr_access,
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
			.access_checker = cxl2_vr_access,
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
			.access_checker = cxl2_vr_access,
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
			.access_checker = cxl2_vr_access,
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
			.target_addr = ADDR_VR_PVDDQ_AB_ASIC2,
			.offset = PMBUS_READ_VOUT,
			.access_checker = cxl2_vr_access,
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
			0xC7, //uint8_t supported_thresholds;
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

			0x0000012C, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000190, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x000001F4, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS8,
			.target_addr = ADDR_VR_P0V8_ASIC1,
			.offset = PMBUS_READ_IOUT,
			.access_checker = cxl1_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
			.post_sensor_read_hook = post_vr_read,
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
			0xC7, //uint8_t supported_thresholds;
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

			0x00000082, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000091, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x0000009B, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS8,
			.target_addr = ADDR_VR_PVDDQ_CD_ASIC1,
			.offset = PMBUS_READ_IOUT,
			.access_checker = cxl1_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
			.post_sensor_read_hook = post_vr_read,
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
			0xC7, //uint8_t supported_thresholds;
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

			0x00000320, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x000003E8, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000578, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS8,
			.target_addr = ADDR_VR_P0V85_ASIC1,
			.offset = PMBUS_READ_IOUT,
			.access_checker = cxl1_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
			.post_sensor_read_hook = post_vr_read,
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
			0xC7, //uint8_t supported_thresholds;
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

			0x00000082, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000091, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x0000009B, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS8,
			.target_addr = ADDR_VR_PVDDQ_AB_ASIC1,
			.offset = PMBUS_READ_IOUT,
			.access_checker = cxl1_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
			.post_sensor_read_hook = post_vr_read,
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
			0xC7, //uint8_t supported_thresholds;
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

			0x0000012C, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000190, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x000001F4, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS3,
			.target_addr = ADDR_VR_P0V8_ASIC2,
			.offset = PMBUS_READ_IOUT,
			.access_checker = cxl2_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
			.post_sensor_read_hook = post_vr_read,
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
			0xC7, //uint8_t supported_thresholds;
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

			0x00000082, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000091, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x0000009B, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS3,
			.target_addr = ADDR_VR_PVDDQ_CD_ASIC2,
			.offset = PMBUS_READ_IOUT,
			.access_checker = cxl2_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
			.post_sensor_read_hook = post_vr_read,
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
			0xC7, //uint8_t supported_thresholds;
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

			0x00000320, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x000003E8, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000578, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS3,
			.target_addr = ADDR_VR_P0V85_ASIC2,
			.offset = PMBUS_READ_IOUT,
			.access_checker = cxl2_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
			.post_sensor_read_hook = post_vr_read,
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
			0xC7, //uint8_t supported_thresholds;
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

			0x00000082, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000091, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x0000009B, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS3,
			.target_addr = ADDR_VR_PVDDQ_AB_ASIC2,
			.offset = PMBUS_READ_IOUT,
			.access_checker = cxl2_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
			.post_sensor_read_hook = post_vr_read,
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
			0xC7, //uint8_t supported_thresholds;
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

			0x00000BB8, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000FA0, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00001388, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS8,
			.target_addr = ADDR_VR_P0V8_ASIC1,
			.offset = PMBUS_READ_POUT,
			.access_checker = cxl1_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
			.post_sensor_read_hook = post_vr_read,
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
			0xC7, //uint8_t supported_thresholds;
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

			0x00004268, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00004A38, //uint32_t critical_high;
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
			.access_checker = cxl1_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
			.post_sensor_read_hook = post_vr_read,
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
			0xC7, //uint8_t supported_thresholds;
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

			0x2FAF0800, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x3B9ACA00, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x59682F00, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS8,
			.target_addr = ADDR_VR_P0V85_ASIC1,
			.offset = PMBUS_READ_POUT,
			.access_checker = cxl1_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
			.post_sensor_read_hook = post_vr_read,
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
			0xC7, //uint8_t supported_thresholds;
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

			0x00004268, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00004A38, //uint32_t critical_high;
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
			.access_checker = cxl1_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
			.post_sensor_read_hook = post_vr_read,
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
			0xC7, //uint8_t supported_thresholds;
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

			0x00000BB8, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000FA0, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00001388, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS3,
			.target_addr = ADDR_VR_P0V8_ASIC2,
			.offset = PMBUS_READ_POUT,
			.access_checker = cxl2_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
			.post_sensor_read_hook = post_vr_read,
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
			0xC7, //uint8_t supported_thresholds;
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

			0x00004268, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00004A38, //uint32_t critical_high;
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
			.access_checker = cxl2_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
			.post_sensor_read_hook = post_vr_read,
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
			0xC7, //uint8_t supported_thresholds;
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

			0x2FAF0800, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x3B9ACA00, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x59682F00, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_xdpe12284c,
			.port = I2C_BUS3,
			.target_addr = ADDR_VR_P0V85_ASIC2,
			.offset = PMBUS_READ_POUT,
			.access_checker = cxl2_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
			.post_sensor_read_hook = post_vr_read,
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
			0xC7, //uint8_t supported_thresholds;
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

			0x00004268, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00004A38, //uint32_t critical_high;
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
			.access_checker = cxl2_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
			.post_sensor_read_hook = post_vr_read,
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
			0xC2, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_10S, //int32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x05, //uint8_t range_field_format;
			0x04, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //int32_t warning_high;
			0x00000000, //int32_t warning_low;
			0x00002134, //int32_t critical_high;
			0x00000000, //int32_t critical_low;
			0x00000000, //int32_t fatal_high;
			0x00000000, //int32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_vistara,
			.port = CXL_ID_1,
			.target_addr = DIMMA_ID,
			.access_checker = cxl1_ready_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.init_args = &vistara_init_args[0],
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
			0xC2, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_10S, //int32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x05, //uint8_t range_field_format;
			0x04, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //int32_t warning_high;
			0x00000000, //int32_t warning_low;
			0x00002134, //int32_t critical_high;
			0x00000000, //int32_t critical_low;
			0x00000000, //int32_t fatal_high;
			0x00000000, //int32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_vistara,
			.port = CXL_ID_1,
			.target_addr = DIMMB_ID,
			.access_checker = cxl1_ready_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.init_args = &vistara_init_args[0],
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
			0xC2, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_10S, //int32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x05, //uint8_t range_field_format;
			0x04, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //int32_t warning_high;
			0x00000000, //int32_t warning_low;
			0x00002134, //int32_t critical_high;
			0x00000000, //int32_t critical_low;
			0x00000000, //int32_t fatal_high;
			0x00000000, //int32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_vistara,
			.port = CXL_ID_1,
			.target_addr = DIMMC_ID,
			.access_checker = cxl1_ready_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.init_args = &vistara_init_args[0],
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
			0xC2, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_10S, //int32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x05, //uint8_t range_field_format;
			0x04, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //int32_t warning_high;
			0x00000000, //int32_t warning_low;
			0x00002134, //int32_t critical_high;
			0x00000000, //int32_t critical_low;
			0x00000000, //int32_t fatal_high;
			0x00000000, //int32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_vistara,
			.port = CXL_ID_1,
			.target_addr = DIMMD_ID,
			.access_checker = cxl1_ready_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.init_args = &vistara_init_args[0],
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
			0xC2, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_10S, //int32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x05, //uint8_t range_field_format;
			0x04, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //int32_t warning_high;
			0x00000000, //int32_t warning_low;
			0x00002134, //int32_t critical_high;
			0x00000000, //int32_t critical_low;
			0x00000000, //int32_t fatal_high;
			0x00000000, //int32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_vistara,
			.port = CXL_ID_2,
			.target_addr = DIMMA_ID,
			.access_checker = cxl2_ready_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.init_args = &vistara_init_args[0],
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
			0xC2, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_10S, //int32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x05, //uint8_t range_field_format;
			0x04, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //int32_t warning_high;
			0x00000000, //int32_t warning_low;
			0x00002134, //int32_t critical_high;
			0x00000000, //int32_t critical_low;
			0x00000000, //int32_t fatal_high;
			0x00000000, //int32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_vistara,
			.port = CXL_ID_2,
			.target_addr = DIMMB_ID,
			.access_checker = cxl2_ready_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.init_args = &vistara_init_args[0],
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
			0xC2, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_10S, //int32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x05, //uint8_t range_field_format;
			0x04, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //int32_t warning_high;
			0x00000000, //int32_t warning_low;
			0x00002134, //int32_t critical_high;
			0x00000000, //int32_t critical_low;
			0x00000000, //int32_t fatal_high;
			0x00000000, //int32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_vistara,
			.port = CXL_ID_2,
			.target_addr = DIMMC_ID,
			.access_checker = cxl2_ready_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.init_args = &vistara_init_args[0],
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
			0xC2, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_10S, //int32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x05, //uint8_t range_field_format;
			0x04, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //int32_t warning_high;
			0x00000000, //int32_t warning_low;
			0x00002134, //int32_t critical_high;
			0x00000000, //int32_t critical_low;
			0x00000000, //int32_t fatal_high;
			0x00000000, //int32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_vistara,
			.port = CXL_ID_2,
			.target_addr = DIMMD_ID,
			.access_checker = cxl2_ready_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.init_args = &vistara_init_args[0],
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
		.sensorName = u"WF_INLET_TEMP_C",
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
		.sensorName = u"WF_VR_ASIC1_P0V8_TEMP_C",
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
		.sensorName = u"WF_VR_ASIC1_PVDDQ_CD_TEMP_C",
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
		.sensorName = u"WF_VR_ASIC1_P0V85_TEMP_C",
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
		.sensorName = u"WF_VR_ASIC1_PVDDQ_AB_TEMP_C",
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
		.sensorName = u"WF_VR_ASIC2_P0V8_TEMP_C",
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
		.sensorName = u"WF_VR_ASIC2_PVDDQ_CD_TEMP_C",
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
		.sensorName = u"WF_VR_ASIC2_P0V85_TEMP_C",
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
		.sensorName = u"WF_VR_ASIC2_PVDDQ_AB_TEMP_C",
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
		.sensorName = u"WF_PMON_P12V_STBY_VOLT_V",
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
		.sensorName = u"WF_PMON_E1S0_P12V_VOLT_V",
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
		.sensorName = u"WF_ADC_E1S0_P3V3_VOLT_V",
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
		.sensorName = u"WF_ADC_ASIC1_P1V2_VOLT_V",
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
		.sensorName = u"WF_ADC_ASIC1_P1V8_VOLT_V",
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
		.sensorName = u"WF_VR_ASIC1_P0V8_VOLT_V",
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
		.sensorName = u"WF_VR_ASIC1_PVDDQ_CD_VOLT_V",
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
		.sensorName = u"WF_VR_ASIC1_P0V85_VOLT_V",
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
		.sensorName = u"WF_VR_ASIC1_PVDDQ_AB_VOLT_V",
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
		.sensorName = u"WF_ADC_ASIC1_P0V75_VOLT_V",
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
		.sensorName = u"WF_ADC_ASIC1_PVPP_AB_VOLT_V",
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
		.sensorName = u"WF_ADC_ASIC1_PVPP_CD_VOLT_V",
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
		.sensorName = u"WF_ADC_ASIC1_PVTT_AB_VOLT_V",
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
		.sensorName = u"WF_ADC_ASIC1_PVTT_CD_VOLT_V",
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
		.sensorName = u"WF_ADC_ASIC2_P1V2_VOLT_V",
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
		.sensorName = u"WF_ADC_ASIC2_P1V8_VOLT_V",
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
		.sensorName = u"WF_VR_ASIC2_P0V8_VOLT_V",
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
		.sensorName = u"WF_VR_ASIC2_PVDDQ_CD_VOLT_V",
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
		.sensorName = u"WF_VR_ASIC2_P0V85_VOLT_V",
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
		.sensorName = u"WF_VR_ASIC2_PVDDQ_AB_VOLT_V",
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
		.sensorName = u"WF_ADC_ASIC2_P0V75_VOLT_V",
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
		.sensorName = u"WF_ADC_ASIC2_PVPP_AB_VOLT_V",
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
		.sensorName = u"WF_ADC_ASIC2_PVPP_CD_VOLT_V",
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
		.sensorName = u"WF_ADC_ASIC2_PVTT_AB_VOLT_V",
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
		.sensorName = u"WF_ADC_ASIC2_PVTT_CD_VOLT_V",
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
		.sensorName = u"WF_PMON_P12V_STBY_CURR_A",
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
		.sensorName = u"WF_PMON_E1S0_P12V_CURR_A",
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
		.sensorName = u"WF_VR_ASIC1_P0V8_CURR_A",
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
		.sensorName = u"WF_VR_ASIC1_PVDDQ_CD_CURR_A",
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
		.sensorName = u"WF_VR_ASIC1_P0V85_CURR_A",
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
		.sensorName = u"WF_VR_ASIC1_PVDDQ_AB_CURR_A",
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
		.sensorName = u"WF_VR_ASIC2_P0V8_CURR_A",
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
		.sensorName = u"WF_VR_ASIC2_PVDDQ_CD_CURR_A",
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
		.sensorName = u"WF_VR_ASIC2_P0V85_CURR_A",
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
		.sensorName = u"WF_VR_ASIC2_PVDDQ_AB_CURR_A",
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
		.sensorName = u"WF_PMON_P12V_STBY_PWR_W",
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
		.sensorName = u"WF_PMON_E1S0_P12V_PWR_W",
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
		.sensorName = u"WF_VR_ASIC1_P0V8_PWR_W",
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
		.sensorName = u"WF_VR_ASIC1_PVDDQ_CD_PWR_W",
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
		.sensorName = u"WF_VR_ASIC1_P0V85_PWR_W",
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
		.sensorName = u"WF_VR_ASIC1_PVDDQ_AB_PWR_W",
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
		.sensorName = u"WF_VR_ASIC2_P0V8_PWR_W",
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
		.sensorName = u"WF_VR_ASIC2_PVDDQ_CD_PWR_W",
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
		.sensorName = u"WF_VR_ASIC2_P0V85_PWR_W",
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
		.sensorName = u"WF_VR_ASIC2_PVDDQ_AB_PWR_W",
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
		.sensorName = u"WF_MCIO4_DIMM_A1_TEMP_C",
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
		.sensorName = u"WF_MCIO4_DIMM_A0_TEMP_C",
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
		.sensorName = u"WF_MCIO4_DIMM_B0_TEMP_C",
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
		.sensorName = u"WF_MCIO4_DIMM_B1_TEMP_C",
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
		.sensorName = u"WF_MCIO3_DIMM_A1_TEMP_C",
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
		.sensorName = u"WF_MCIO3_DIMM_A0_TEMP_C",
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
		.sensorName = u"WF_MCIO3_DIMM_B0_TEMP_C",
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
		.sensorName = u"WF_MCIO3_DIMM_B1_TEMP_C",
	},
};

PDR_entity_auxiliary_names plat_pdr_entity_aux_names_table[] = { {
	{
		.record_handle = 0x00000000,
		.PDR_header_version = 0x01,
		.PDR_type = PLDM_ENTITY_AUXILIARY_NAMES_PDR,
		.record_change_number = 0x0000,
		.data_length = 0x0000,
	},
	.entity_type = 0x0000,
	.entity_instance_number = 0x0001,
	.container_id = 0x0000,
	.shared_name_count = 0x0,
	.nameStringCount = 0x1,
	.nameLanguageTag = "en",
} };

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
	case PLDM_ENTITY_AUXILIARY_NAMES_PDR:
		total_size = ARRAY_SIZE(plat_pdr_entity_aux_names_table);
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
		plat_pldm_sensor_change_asic_tmp_dev();
		return plat_pldm_sensor_tmp_table;
	case INA233_SENSOR_THREAD_ID:
		plat_pldm_sensor_change_ina233_dev();
		return plat_pldm_sensor_ina233_table;
	case VR_SENSOR_THREAD_ID:
		plat_pldm_sensor_change_vr_dev();
		return plat_pldm_sensor_vr_table;
	case DIMM_SENSOR_THREAD_ID:
		return plat_pldm_sensor_dimm_table;
	case ADC_MONITOR_SENSOR_THREAD_ID:
		plat_pldm_sensor_change_adc_monitor_dev();
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

uint16_t plat_pdr_entity_aux_names_table_size = 0;

// Custom function to calculate the length of a char16_t string
size_t char16_strlen(const char16_t *str)
{
	const char16_t *s = str;
	while (*s)
		++s;
	return s - str;
}

// Custom function to copy a char16_t string
char16_t *char16_strcpy(char16_t *dest, const char16_t *src)
{
	char16_t *d = dest;
	while ((*d++ = *src++))
		;
	return dest;
}

void plat_init_entity_aux_names_pdr_table()
{
	// Base name
	const char16_t base_name[] = u"WAILUA_FALLS_SLOT_";

	// Calculate the length of the base name
	size_t base_len = char16_strlen(base_name);

	// Copy the base name to the entityName field
	char16_strcpy(plat_pdr_entity_aux_names_table[0].entityName, base_name);

	plat_pdr_entity_aux_names_table_size =
		sizeof(PDR_entity_auxiliary_names) + (base_len * sizeof(char16_t));
}

void plat_load_entity_aux_names_pdr_table(PDR_entity_auxiliary_names *entity_aux_name_table)
{
	memcpy(entity_aux_name_table, &plat_pdr_entity_aux_names_table,
	       plat_pdr_entity_aux_names_table_size);
}

void update_entity_name_with_eid(uint8_t eid)
{
	PDR_entity_auxiliary_names *table = get_entity_auxiliary_names_table();

	if (table == NULL) {
		return; // Handle the error case if the table is not initialized
	}

	// EID is 10-based
	uint8_t slot_id = eid / 10;
	// Create the base entity name
	char16_t base_name[] = u"WAILUA_FALLS_SLOT_";

	// Calculate the length of the base name and the eid
	size_t base_name_len =
		sizeof(base_name) / sizeof(char16_t) - 1; // -1 to exclude the null terminator
	size_t total_len = base_name_len + 2; // +2 for the slot ID digit and null terminator
	size_t slot_id_len = 1; // Assuming slot_id is a single digit

	// Copy the base name into the entity name field
	memcpy(table[0].entityName, base_name, base_name_len * sizeof(char16_t));

	// Append the eid to the entity name
	table[0].entityName[base_name_len] = (char16_t)(u'0' + slot_id); // Assuming slot_id is 0-9
	table[0].entityName[base_name_len + slot_id_len] = u'\0'; // Null-terminate the string

	plat_pdr_entity_aux_names_table_size =
		sizeof(PDR_entity_auxiliary_names) + (total_len * sizeof(char16_t));

	// Convert entity name to UTF16-BE
	for (int i = 0; table[0].entityName[i] != 0x0000; i++) {
		table[0].entityName[i] = sys_cpu_to_be16(table[0].entityName[i]);
	}
}

uint16_t plat_get_pdr_entity_aux_names_size()
{
	return plat_pdr_entity_aux_names_table_size;
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

void plat_pldm_sensor_change_adc_monitor_dev()
{
	uint8_t reg = 0;

	/*
	 * Get ADC type from IOE3 P15
	 * Low - MAXIM (main source)
	 * High - TI (2nd source)
	 */

	if (get_ioe_value(ADDR_IOE3, TCA9555_INPUT_PORT_REG_1, &reg) == -1) {
		LOG_ERR("Failed to get the ADC type.");
		return;
	}

	if (GETBIT(reg, IOE_P15)) {
		for (int index = 0;
		     index < plat_pldm_sensor_get_sensor_count(ADC_MONITOR_SENSOR_THREAD_ID);
		     index++) {
			plat_pldm_sensor_adc_monitor_table[index].pldm_sensor_cfg.target_addr =
				ADDR_ADC128D818;
			plat_pldm_sensor_adc_monitor_table[index].pldm_sensor_cfg.type =
				sensor_dev_adc128d818;
			plat_pldm_sensor_adc_monitor_table[index].pldm_sensor_cfg.init_args =
				&adc128d818_init_args[0];
			plat_pldm_sensor_adc_monitor_table[index]
				.pldm_sensor_cfg.post_sensor_read_hook = post_adc128d818_read;
		}
	}
}

void plat_pldm_sensor_change_asic_tmp_dev()
{
	if (get_board_revision() < BOARD_PVT) {
		for (int index = 0; index < plat_pldm_sensor_get_sensor_count(TMP_SENSOR_THREAD_ID);
		     index++) {
			if (plat_pldm_sensor_tmp_table[index].pldm_sensor_cfg.type ==
				    sensor_dev_tmp461 &&
			    plat_pldm_sensor_tmp_table[index].pldm_sensor_cfg.port == I2C_BUS4) {
				plat_pldm_sensor_tmp_table[index].pldm_sensor_cfg.target_addr =
					ADDR_TMP461_CXL2;
				break;
			}
		}
	}
}

void plat_pldm_sensor_change_ina233_dev()
{
	uint8_t reg = 0;

	/*
	 * Get remote sensor type from BOARD_ID2 (IOE3 P16)
	 * Low - INA233 (main source)
	 * High - RTQ6056 (2nd source)
	 */

	if (get_ioe_value(ADDR_IOE3, TCA9555_INPUT_PORT_REG_1, &reg) == -1) {
		LOG_ERR("Failed to get the remote sensor type from BOARD_ID2");
		return;
	}

	if (GETBIT(reg, IOE_P16)) {
		for (int index = 0;
		     index < plat_pldm_sensor_get_sensor_count(INA233_SENSOR_THREAD_ID); index++) {
			if (plat_pldm_sensor_ina233_table[index].pldm_sensor_cfg.type ==
			    sensor_dev_ina233) {
				plat_pldm_sensor_ina233_table[index].pldm_sensor_cfg.type =
					sensor_dev_rtq6056;

				if (plat_pldm_sensor_ina233_table[index]
					    .pldm_sensor_cfg.target_addr == ADDR_INA233_P12V_STBY) {
					plat_pldm_sensor_ina233_table[index]
						.pldm_sensor_cfg.init_args = &rtq6056_init_args[0];
				} else {
					plat_pldm_sensor_ina233_table[index]
						.pldm_sensor_cfg.init_args = &rtq6056_init_args[1];
				}
			}
		}
	}
}

float plat_get_dimm_cache(uint8_t cxl_id, uint8_t dimm_id)
{
	float result = -1.0;
	if (k_mutex_lock(&cxl_dimm_mutex, K_MSEC(CXL_DIMM_MUTEX_WAITING_TIME_MS))) {
		return result;
	}

	result = cxl_dimm_temp[cxl_id][dimm_id];

	k_mutex_unlock(&cxl_dimm_mutex);

	return result;
}

void plat_set_dimm_cache(uint8_t *resp_buf, uint8_t cxl_id, uint8_t status)
{
	if (k_mutex_lock(&cxl_dimm_mutex, K_MSEC(CXL_DIMM_MUTEX_WAITING_TIME_MS))) {
		cxl_dimm_temp[cxl_id][DIMMA_ID] = -1;
		cxl_dimm_temp[cxl_id][DIMMB_ID] = -1;
		cxl_dimm_temp[cxl_id][DIMMC_ID] = -1;
		cxl_dimm_temp[cxl_id][DIMMD_ID] = -1;
		return;
	}

	for (int i = 0; i < MAX_DIMM_ID; i++) {
		if (status == SENSOR_READ_SUCCESS) {
			read_ddr_temp_resp *ddr_temp = (read_ddr_temp_resp *)(resp_buf + i * 8);
			cxl_dimm_temp[cxl_id][i] = *((float *)&ddr_temp->dimm_temp);
			LOG_HEXDUMP_DBG(ddr_temp->dimm_temp, sizeof(float), "ddr temp");
		} else {
			cxl_dimm_temp[cxl_id][i] = -1;
		}
	}

	k_mutex_unlock(&cxl_dimm_mutex);
}
