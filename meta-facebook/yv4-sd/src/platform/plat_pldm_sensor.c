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
#include "ina233.h"
#include "sq52205.h"
#include "pt5161l.h"
#include "bcm85658.h"
#include "rtq6056.h"
#include "sensor.h"
#include "pldm_sensor.h"
#include "pldm_monitor.h"
#include "plat_apml.h"
#include "plat_hook.h"
#include "plat_i2c.h"
#include "plat_pldm_sensor.h"
#include "plat_dimm.h"
#include "plat_class.h"
#include "plat_gpio.h"

LOG_MODULE_REGISTER(plat_pldm_sensor);
static bool bootdrive_exist = false;
K_THREAD_STACK_DEFINE(monitor_prochot_sensor_stack, MONITOR_PROCHOT_SENSOR_STACK_SIZE);
struct k_thread monitor_prochot_sensor_thread;
k_tid_t monitor_prochot_sensor_tid;

#ifdef ENABLE_PLATFORM_PROVIDES_PLDM_SENSOR_STACKS
K_THREAD_STACK_DEFINE(plat_adc_sensor_stack, ADC_SENSOR_STACK_SIZE);
K_THREAD_STACK_DEFINE(plat_vr_sensor_stack, VR_SENSOR_STACK_SIZE);
K_THREAD_STACK_DEFINE(plat_mb_temp_sensor_stack, MB_TEMP_SENSOR_STACK_SIZE);
K_THREAD_STACK_DEFINE(plat_cpu_sensor_stack, CPU_SENSOR_STACK_SIZE);
K_THREAD_STACK_DEFINE(plat_ina233_sensor_stack, INA233_SENSOR_STACK_SIZE);
K_THREAD_STACK_DEFINE(plat_dimm_sensor_stack, DIMM_SENSOR_STACK_SIZE);

static k_thread_stack_t *pldm_sensor_poll_stacks[MAX_SENSOR_THREAD_ID] = {
	plat_adc_sensor_stack, plat_vr_sensor_stack,	 plat_mb_temp_sensor_stack,
	plat_cpu_sensor_stack, plat_ina233_sensor_stack, plat_dimm_sensor_stack
};

static const size_t pldm_sensor_stack_sizes[MAX_SENSOR_THREAD_ID] = {
	ADC_SENSOR_STACK_SIZE, VR_SENSOR_STACK_SIZE,	 MB_TEMP_SENSOR_STACK_SIZE,
	CPU_SENSOR_STACK_SIZE, INA233_SENSOR_STACK_SIZE, DIMM_SENSOR_STACK_SIZE
};
#endif

static struct pldm_sensor_thread pal_pldm_sensor_thread[MAX_SENSOR_THREAD_ID] = {
	// thread id, thread name
	{ ADC_SENSOR_THREAD_ID, "ADC_PLDM_SENSOR_THREAD" },
	{ VR_SENSOR_THREAD_ID, "VR_PLDM_SENSOR_THREAD" },
	{ MB_TEMP_SENSOR_THREAD_ID, "MB_TEMP_SENSOR_THREAD" },
	{ CPU_SENSOR_THREAD_ID, "CPU_TEMP_SENSOR_THREAD" },
	{ INA233_SENSOR_THREAD_ID, "INA233_PLDM_SENSOR_THREAD" },
	{ DIMM_SENSOR_THREAD_ID, "DIMM_SENSOR_THREAD" },
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
			0x0020, //uint16_t sensor_id;
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
			0x0021, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x0003, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
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
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xD2, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x000000B8, //uint32_t max_readable;
			0x000000B0, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x0C, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x000000C0, //uint32_t critical_high;
			0x000000AC, //uint32_t critical_low;
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
			0x0022, //uint16_t sensor_id;
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
			0x0005721F, //uint32_t critical_high;
			0x0004A3DF, //uint32_t critical_low;
			0x000617C4, //uint32_t fatal_high;
			0x00038658, //uint32_t fatal_low;
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
			0x0023, //uint16_t sensor_id;
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
			0xDB, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATA_INTERNAL_1HR, //int32_t update_interval;
			0x000084D0, //uint32_t max_readable;
			0x00006F54, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x0F, //uint8_t range_field_support;
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
			.pre_sensor_read_hook = pre_p3v_bat_read,
			.post_sensor_read_hook = post_p3v_bat_read,
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
			0x0024, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x0006, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
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
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xD2, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x0000015A, //uint32_t max_readable;
			0x00000139, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x0C, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x0000015B, //uint32_t critical_high;
			0x00000138, //uint32_t critical_low;
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
			0x0025, //uint16_t sensor_id;
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
			0x0000CD14, //uint32_t max_readable;
			0x0000B98C, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x0000D12E, //uint32_t warning_high;
			0x0000B5D6, //uint32_t warning_low;
			0x0000D33B, //uint32_t critical_high;
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
			0x0026, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x0008, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
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
			0x00000510, //uint32_t max_readable;
			0x00000450, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000528, //uint32_t warning_high;
			0x00000438, //uint32_t warning_low;
			0x0000053A, //uint32_t critical_high;
			0x0000042C, //uint32_t critical_low;
			0x0000059A, //uint32_t fatal_high;
			0x000003F0, //uint32_t fatal_low;
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
			0x0027, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x0009, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
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
			0x00000510, //uint32_t max_readable;
			0x00000450, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000528, //uint32_t warning_high;
			0x00000438, //uint32_t warning_low;
			0x0000053A, //uint32_t critical_high;
			0x0000042C, //uint32_t critical_low;
			0x0000059A, //uint32_t fatal_high;
			0x000003F0, //uint32_t fatal_low;
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
			0x0028, //uint16_t sensor_id;
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
			0xFB, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00003138, //uint32_t max_readable;
			0x00002C88, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x0F, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00003234, //uint32_t warning_high;
			0x00002BA4, //uint32_t warning_low;
			0x000032B2, //uint32_t critical_high;
			0x00002B32, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00002580, //uint32_t fatal_low;
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
			0x0029, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x000B, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
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
			0x0002D7BC, //uint32_t max_readable;
			0x0002A300, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x0002E64A, //uint32_t warning_high;
			0x00029580, //uint32_t warning_low;
			0x0002ED91, //uint32_t critical_high;
			0x00028EC0, //uint32_t critical_low;
			0x00032FA0, //uint32_t fatal_high;
			0x00023280, //uint32_t fatal_low;
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
			0x002A, //uint16_t sensor_id;
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
			0x00, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
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
			// PVDD11 S3 Voltage
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
			0x0087, //uint16_t entity_type;
			0x000D, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
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
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xD2, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000074, //uint32_t max_readable;
			0x00000069, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x0C, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000078, //uint32_t critical_high;
			0x00000066, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_ast_adc,
			.port = ADC_PORT14,
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
			0x002C, //uint16_t sensor_id;
			0x0087, //uint16_t entity_type;
			0x000E, //uint16_t entity_instance_number;
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
			0x00, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
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
			0xC6, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x14, //uint8_t range_field_support;
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
			0xC6, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x14, //uint8_t range_field_support;
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
			0xC6, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x14, //uint8_t range_field_support;
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
			0xC6, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x14, //uint8_t range_field_support;
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
			0xC6, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x14, //uint8_t range_field_support;
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
			// VR CPU0 Voltage
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
			0x0006, //uint16_t entity_instance_number;
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
			0xD2, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x0C, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x000042CC, //uint32_t critical_high;
			0x00000E74, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2856gut,
			.port = I2C_BUS4,
			.target_addr = ADDR_VR_CPU0,
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
			// VR SOC Voltage
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
			0x002E, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
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
			0xD2, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x0C, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00003520, //uint32_t critical_high;
			0x0000170C, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2856gut,
			.port = I2C_BUS4,
			.target_addr = ADDR_VR_SOC,
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
			// VR CPU1 Voltage
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
			0x002F, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0008, //uint16_t entity_instance_number;
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
			0xD2, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x0C, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x000042CC, //uint32_t critical_high;
			0x00000E74, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2856gut,
			.port = I2C_BUS4,
			.target_addr = ADDR_VR_CPU1,
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
			// VR PVDDIO Voltage
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
			0x0030, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0009, //uint16_t entity_instance_number;
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
			0xD2, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x0C, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00003264, //uint32_t critical_high;
			0x00001FA4, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2856gut,
			.port = I2C_BUS4,
			.target_addr = ADDR_VR_PVDDIO,
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
			// VR PVDD11 Voltage
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
			0x0031, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
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
			0xD2, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x0C, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00002DB4, //uint32_t critical_high;
			0x000028A0, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2856gut,
			.port = I2C_BUS4,
			.target_addr = ADDR_VR_PVDD11,
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
			0x0040, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x000B, //uint16_t entity_instance_number;
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
			0x000000E6, //uint32_t critical_high;
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
			.post_sensor_read_hook = post_vr_read,
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
			0x0041, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x000C, //uint16_t entity_instance_number;
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
			0x00000082, //uint32_t critical_high;
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
			.post_sensor_read_hook = post_vr_read,
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
			0x0042, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x000D, //uint16_t entity_instance_number;
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
			0x000000E6, //uint32_t critical_high;
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
			.post_sensor_read_hook = post_vr_read,
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
			0x0043, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x000E, //uint16_t entity_instance_number;
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
			0x0000008C, //uint32_t critical_high;
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
			.post_sensor_read_hook = post_vr_read,
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
			0x0044, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x000F, //uint16_t entity_instance_number;
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
			0x00000050, //uint32_t critical_high;
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
			.post_sensor_read_hook = post_vr_read,
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
			0x0050, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0010, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;
			-1, //int8_t unit_modifier;
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
			0x00000F5D, //uint32_t critical_high;
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
			.post_sensor_read_hook = post_vr_read,
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
			0x0051, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0011, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;
			-1, //int8_t unit_modifier;
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
			0x000006E8, //uint32_t critical_high;
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
			.post_sensor_read_hook = post_vr_read,
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
			0x0052, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0012, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;
			-1, //int8_t unit_modifier;
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
			0x00000F5D, //uint32_t critical_high;
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
			.post_sensor_read_hook = post_vr_read,
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
			0x0053, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0013, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;
			-1, //int8_t unit_modifier;
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
			0x0000070E, //uint32_t critical_high;
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
			.post_sensor_read_hook = post_vr_read,
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
			0x0054, //uint16_t sensor_id;
			0x007C, //uint16_t entity_type;
			0x0014, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;
			-1, //int8_t unit_modifier;
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
			0x000003A8, //uint32_t critical_high;
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
			.post_sensor_read_hook = post_vr_read,
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
			0xC6, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x14, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000037, //uint32_t critical_high;
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
			0xC6, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x14, //uint8_t range_field_support;
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
			0x00, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
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
	{
		{
			// MB_SSD_BOOT_TEMP_C
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
			0x0011, //uint16_t sensor_id;
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
			0x5, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xC6, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x14, //uint8_t range_field_support;
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
			.port = I2C_BUS1,
			.target_addr = ADDR_NVME,
			.offset = OFFSET_NVME_TEMP,
			.access_checker = bootdrive_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// MB_SSD_DATA_TEMP_C
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
			0x0012, //uint16_t sensor_id;
			0x0089, //uint16_t entity_type;
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
			0xC6, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x14, //uint8_t range_field_support;
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
			.port = I2C_BUS6,
			.target_addr = ADDR_NVME,
			.offset = OFFSET_NVME_TEMP,
			.access_checker = post_access,
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
			0xC6, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //int32_t state_transition_interval;
			5, //int32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x04, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00015F90, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00017318, //uint32_t fatal_high;
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
			// MB_SOC_PACKAGE_PWR_W
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
			NUM_SOC_PACKAGE_PWR, //uint16_t sensor_id;
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
			0x00, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //int32_t state_transition_interval;
			5, //int32_t update_interval;
			0x00000BDC, //uint32_t max_readable;
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

pldm_sensor_info plat_pldm_sensor_ina233_table[] = {
	{
		{
			// MB_INA233_x8_RTM_CURR_A
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
			SENSOR_NUM_MB_INA233_X8_RTM_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0001, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;
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
			0xC3, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;

			UPDATE_INTERVAL_1S, //int32_t update_interval;
			0x0001750C, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x05, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x000187B3, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00019A5A, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_ina233,
			.port = I2C_BUS6,
			.target_addr = ADDR_X8_INA233,
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
			// MB_INA233_x8_RTM_PWR_W
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
			SENSOR_NUM_MB_INA233_X8_RTM_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0003, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			PDR_SENSOR_USEINIT_PDR, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;
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
			0xC7, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //int32_t update_interval;
			0x000FBCE8, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x15, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00100D7C, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x001035C6, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00105E10, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_ina233,
			.port = I2C_BUS6,
			.target_addr = ADDR_X8_INA233,
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
			// MB_INA233_E1S_Boot_VOLT_V
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
			0x0000, //uint16_t entity_type; //Need to check
			0x0005, //uint16_t entity_instance_number;
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
			0x00003390, //uint32_t max_readable;
			0x00002A30, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00003480, //uint32_t warning_high;
			0x00002940, //uint32_t warning_low;
			0x000034F8, //uint32_t critical_high;
			0x000028C8, //uint32_t critical_low;
			0x000037FD, //uint32_t fatal_high;
			0x0000276B, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_ina233,
			.port = I2C_BUS1,
			.target_addr = ADDR_E1S_BOOT_INA233,
			.offset = PMBUS_READ_VOUT,
			.access_checker = bootdrive_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.init_args = &ina233_init_args[2],
		},
	},
	{
		{
			// MB_INA233_E1S_Data_VOLT_V
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
			SENSOR_NUM_MB_INA233_E1S_DATA_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
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
			0x00003390, //uint32_t max_readable;
			0x00002A30, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0xFF, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00003480, //uint32_t warning_high;
			0x00002940, //uint32_t warning_low;
			0x000034F8, //uint32_t critical_high;
			0x000028C8, //uint32_t critical_low;
			0x000037FD, //uint32_t fatal_high;
			0x0000276B, //uint32_t fatal_low;

		},
		.update_time = 0,
		{
			.type = sensor_dev_ina233,
			.port = I2C_BUS6,
			.target_addr = ADDR_E1S_DATA_INA233,
			.offset = PMBUS_READ_VOUT,
			.access_checker = post_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.init_args = &ina233_init_args[3],
		},
	},
	{
		{
			// MB_INA233_E1S_Boot_CURR_A
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
			0x0000, //uint16_t entity_type; //Need to check
			0x0007, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;
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
			0xC7, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x0000413C, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x15, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x0000447F, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x000047C2, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x000055F0, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_ina233,
			.port = I2C_BUS1,
			.target_addr = ADDR_E1S_BOOT_INA233,
			.offset = PMBUS_READ_IOUT,
			.access_checker = bootdrive_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.init_args = &ina233_init_args[2],
		},
	},
	{
		{
			// MB_INA233_E1S_Data_CURR_A
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
			SENSOR_NUM_MB_INA233_E1S_DATA_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0008, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
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
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0xC7, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000014, //uint32_t max_readable;
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
			.target_addr = ADDR_E1S_DATA_INA233,
			.offset = PMBUS_READ_IOUT,
			.access_checker = post_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.init_args = &ina233_init_args[3],
		},
	},
	{
		{
			// MB_INA233_E1S_Boot_PWR_W
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
			0x0064, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0009, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;
			-1, //int8_t unit_modifier;
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
			0xC7, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x000000C8, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x15, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x000000CC, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x000000CE, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x000000D0, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.type = sensor_dev_ina233,
			.port = I2C_BUS1,
			.target_addr = ADDR_E1S_BOOT_INA233,
			.offset = PMBUS_READ_POUT,
			.access_checker = bootdrive_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.init_args = &ina233_init_args[2],
		},
	},
	{
		{
			// MB_INA233_E1S_Data_PWR_W
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
			SENSOR_NUM_MB_INA233_E1S_DATA_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x000A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
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
			0xC7, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x000009C4, //uint32_t max_readable;
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
			.target_addr = ADDR_E1S_DATA_INA233,
			.offset = PMBUS_READ_POUT,
			.access_checker = post_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.init_args = &ina233_init_args[3],
		},
	},
	{
		{
			// x8 retimer Temperature
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
			0x0089, //uint16_t entity_type;
			0x0006, //uint16_t entity_instance_number;
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
			0xC6, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x14, //uint8_t range_field_support;
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
			.type = sensor_dev_pt5161l,
			.port = I2C_BUS6,
			.target_addr = ADDR_X8_RETIMER,
			.offset = PT5161L_TEMP_OFFSET,
			.access_checker = post_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_retimer_read,
			.init_args = &pt5161l_init_args[0],
		},
	},
};

pldm_sensor_info plat_pldm_sensor_dimm_table[] = {
	{
		{
			// DIMMA Temp
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
			NUM_DIMM_A_TEMP, //uint16_t sensor_id;
			0x8E, //uint16_t entity_type;
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
			0xC6, //uint8_t supported_thresholds;
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
			0x00000055, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x0000005F, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = NUM_DIMM_A_TEMP,
			.type = sensor_dev_i3c_dimm,
			.port = I3C_BUS3,
			.target_addr = DIMM_SPD_A_G_ADDR,
			.offset = DIMM_SPD_TEMP,
			.access_checker = post_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_dimm_i3c_read,
		},
	},
	{
		{
			// DIMMB Temp
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
			NUM_DIMM_B_TEMP, //uint16_t sensor_id;
			0x8E, //uint16_t entity_type;
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
			0xC6, //uint8_t supported_thresholds;
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
			0x00000055, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x0000005F, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = NUM_DIMM_B_TEMP,
			.type = sensor_dev_i3c_dimm,
			.port = I3C_BUS3,
			.target_addr = DIMM_SPD_B_H_ADDR,
			.offset = DIMM_SPD_TEMP,
			.access_checker = post_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_dimm_i3c_read,
		},
	},
	{
		{
			// DIMMC Temp
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
			NUM_DIMM_C_TEMP, //uint16_t sensor_id;
			0x8E, //uint16_t entity_type;
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
			0xC6, //uint8_t supported_thresholds;
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
			0x00000055, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x0000005F, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = NUM_DIMM_C_TEMP,
			.type = sensor_dev_i3c_dimm,
			.port = I3C_BUS3,
			.target_addr = DIMM_SPD_C_I_ADDR,
			.offset = DIMM_SPD_TEMP,
			.access_checker = post_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_dimm_i3c_read,
		},
	},
	{
		{
			// DIMMD Temp
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
			NUM_DIMM_D_TEMP, //uint16_t sensor_id;
			0x8E, //uint16_t entity_type;
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
			0xC6, //uint8_t supported_thresholds;
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
			0x00000055, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x0000005F, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = NUM_DIMM_D_TEMP,
			.type = sensor_dev_i3c_dimm,
			.port = I3C_BUS3,
			.target_addr = DIMM_SPD_D_J_ADDR,
			.offset = DIMM_SPD_TEMP,
			.access_checker = post_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_dimm_i3c_read,
		},
	},
	{
		{
			// DIMME Temp
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
			NUM_DIMM_E_TEMP, //uint16_t sensor_id;
			0x8E, //uint16_t entity_type;
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
			0xC6, //uint8_t supported_thresholds;
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
			0x00000055, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x0000005F, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = NUM_DIMM_E_TEMP,
			.type = sensor_dev_i3c_dimm,
			.port = I3C_BUS3,
			.target_addr = DIMM_SPD_E_K_ADDR,
			.offset = DIMM_SPD_TEMP,
			.access_checker = post_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_dimm_i3c_read,
		},
	},
	{
		{
			// DIMMF Temp
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
			NUM_DIMM_F_TEMP, //uint16_t sensor_id;
			0x8E, //uint16_t entity_type;
			0x0006, //uint16_t entity_instance_number;
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
			0xC6, //uint8_t supported_thresholds;
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
			0x00000055, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x0000005F, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = NUM_DIMM_F_TEMP,
			.type = sensor_dev_i3c_dimm,
			.port = I3C_BUS3,
			.target_addr = DIMM_SPD_F_L_ADDR,
			.offset = DIMM_SPD_TEMP,
			.access_checker = post_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_dimm_i3c_read,
		},
	},
	{
		{
			// DIMMG Temp
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
			NUM_DIMM_G_TEMP, //uint16_t sensor_id;
			0x8E, //uint16_t entity_type;
			0x0007, //uint16_t entity_instance_number;
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
			0xC6, //uint8_t supported_thresholds;
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
			0x00000055, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x0000005F, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = NUM_DIMM_G_TEMP,
			.type = sensor_dev_i3c_dimm,
			.port = I3C_BUS3,
			.target_addr = DIMM_SPD_A_G_ADDR,
			.offset = DIMM_SPD_TEMP,
			.access_checker = post_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_dimm_i3c_read,
		},
	},
	{
		{
			// DIMMH Temp
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
			NUM_DIMM_H_TEMP, //uint16_t sensor_id;
			0x8E, //uint16_t entity_type;
			0x0008, //uint16_t entity_instance_number;
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
			0xC6, //uint8_t supported_thresholds;
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
			0x00000055, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x0000005F, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = NUM_DIMM_H_TEMP,
			.type = sensor_dev_i3c_dimm,
			.port = I3C_BUS3,
			.target_addr = DIMM_SPD_B_H_ADDR,
			.offset = DIMM_SPD_TEMP,
			.access_checker = post_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_dimm_i3c_read,
		},
	},
	{
		{
			// DIMMI Temp
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
			NUM_DIMM_I_TEMP, //uint16_t sensor_id;
			0x8E, //uint16_t entity_type;
			0x0009, //uint16_t entity_instance_number;
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
			0xC6, //uint8_t supported_thresholds;
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
			0x00000055, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x0000005F, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = NUM_DIMM_I_TEMP,
			.type = sensor_dev_i3c_dimm,
			.port = I3C_BUS3,
			.target_addr = DIMM_SPD_C_I_ADDR,
			.offset = DIMM_SPD_TEMP,
			.access_checker = post_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_dimm_i3c_read,
		},
	},
	{
		{
			// DIMMJ Temp
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
			NUM_DIMM_J_TEMP, //uint16_t sensor_id;
			0x8E, //uint16_t entity_type;
			0x000A, //uint16_t entity_instance_number;
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
			0xC6, //uint8_t supported_thresholds;
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
			0x00000055, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x0000005F, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = NUM_DIMM_J_TEMP,
			.type = sensor_dev_i3c_dimm,
			.port = I3C_BUS3,
			.target_addr = DIMM_SPD_D_J_ADDR,
			.offset = DIMM_SPD_TEMP,
			.access_checker = post_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_dimm_i3c_read,
		},
	},
	{
		{
			// DIMMK Temp
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
			NUM_DIMM_K_TEMP, //uint16_t sensor_id;
			0x8E, //uint16_t entity_type;
			0x000B, //uint16_t entity_instance_number;
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
			0xC6, //uint8_t supported_thresholds;
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
			0x00000055, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x0000005F, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = NUM_DIMM_K_TEMP,
			.type = sensor_dev_i3c_dimm,
			.port = I3C_BUS3,
			.target_addr = DIMM_SPD_E_K_ADDR,
			.offset = DIMM_SPD_TEMP,
			.access_checker = post_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_dimm_i3c_read,
		},
	},
	{
		{
			// DIMML Temp
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
			NUM_DIMM_L_TEMP, //uint16_t sensor_id;
			0x8E, //uint16_t entity_type;
			0x000C, //uint16_t entity_instance_number;
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
			0xC6, //uint8_t supported_thresholds;
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
			0x00000055, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x0000005F, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = NUM_DIMM_L_TEMP,
			.type = sensor_dev_i3c_dimm,
			.port = I3C_BUS3,
			.target_addr = DIMM_SPD_F_L_ADDR,
			.offset = DIMM_SPD_TEMP,
			.access_checker = post_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_dimm_i3c_read,
		},
	},
	{
		{
			// DIMMA_PMIC_PWR_W
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
			NUM_DIMM_A_PMIC_PWR, //uint16_t sensor_id;
			0x8E, //uint16_t entity_type;
			0x000D, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
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
			0xC3, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000BDC, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x05, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000C1A, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000C37, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = NUM_DIMM_A_PMIC_PWR,
			.type = sensor_dev_i3c_dimm,
			.port = I3C_BUS3,
			.target_addr = DIMM_PMIC_A_G_ADDR,
			.offset = DIMM_PMIC_SWA_PWR,
			.access_checker = post_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_dimm_i3c_read,
		},
	},
	{
		{
			// DIMMB_PMIC_PWR_W
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
			NUM_DIMM_B_PMIC_PWR, //uint16_t sensor_id;
			0x8E, //uint16_t entity_type;
			0x000E, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
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
			0xC3, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000BDC, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x05, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000C1A, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000C37, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = NUM_DIMM_B_PMIC_PWR,
			.type = sensor_dev_i3c_dimm,
			.port = I3C_BUS3,
			.target_addr = DIMM_PMIC_B_H_ADDR,
			.offset = DIMM_PMIC_SWA_PWR,
			.access_checker = post_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_dimm_i3c_read,
		},
	},
	{
		{
			// DIMMC_PMIC_PWR_W
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
			NUM_DIMM_C_PMIC_PWR, //uint16_t sensor_id;
			0x8E, //uint16_t entity_type;
			0x000F, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
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
			0xC3, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000BDC, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x05, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000C1A, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000C37, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = NUM_DIMM_C_PMIC_PWR,
			.type = sensor_dev_i3c_dimm,
			.port = I3C_BUS3,
			.target_addr = DIMM_PMIC_C_I_ADDR,
			.offset = DIMM_PMIC_SWA_PWR,
			.access_checker = post_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_dimm_i3c_read,
		},
	},
	{
		{
			// DIMMD_PMIC_PWR_W
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
			NUM_DIMM_D_PMIC_PWR, //uint16_t sensor_id;
			0x8E, //uint16_t entity_type;
			0x0010, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
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
			0xC3, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000BDC, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x05, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000C1A, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000C37, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = NUM_DIMM_D_PMIC_PWR,
			.type = sensor_dev_i3c_dimm,
			.port = I3C_BUS3,
			.target_addr = DIMM_PMIC_D_J_ADDR,
			.offset = DIMM_PMIC_SWA_PWR,
			.access_checker = post_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_dimm_i3c_read,
		},
	},
	{
		{
			// DIMME_PMIC_PWR_W
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
			NUM_DIMM_E_PMIC_PWR, //uint16_t sensor_id;
			0x8E, //uint16_t entity_type;
			0x0011, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
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
			0xC3, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000BDC, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x05, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000C1A, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000C37, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = NUM_DIMM_E_PMIC_PWR,
			.type = sensor_dev_i3c_dimm,
			.port = I3C_BUS3,
			.target_addr = DIMM_PMIC_E_K_ADDR,
			.offset = DIMM_PMIC_SWA_PWR,
			.access_checker = post_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_dimm_i3c_read,
		},
	},
	{
		{
			// DIMMF_PMIC_PWR_W
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
			NUM_DIMM_F_PMIC_PWR, //uint16_t sensor_id;
			0x8E, //uint16_t entity_type;
			0x0012, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
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
			0xC3, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000BDC, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x05, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000C1A, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000C37, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = NUM_DIMM_F_PMIC_PWR,
			.type = sensor_dev_i3c_dimm,
			.port = I3C_BUS3,
			.target_addr = DIMM_PMIC_F_L_ADDR,
			.offset = DIMM_PMIC_SWA_PWR,
			.access_checker = post_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_dimm_i3c_read,
		},
	},
	{
		{
			// DIMMG_PMIC_PWR_W
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
			NUM_DIMM_G_PMIC_PWR, //uint16_t sensor_id;
			0x8E, //uint16_t entity_type;
			0x0013, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
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
			0xC3, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000BDC, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x05, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000C1A, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000C37, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = NUM_DIMM_G_PMIC_PWR,
			.type = sensor_dev_i3c_dimm,
			.port = I3C_BUS3,
			.target_addr = DIMM_PMIC_A_G_ADDR,
			.offset = DIMM_PMIC_SWA_PWR,
			.access_checker = post_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_dimm_i3c_read,
		},
	},
	{
		{
			// DIMMH_PMIC_PWR_W
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
			NUM_DIMM_H_PMIC_PWR, //uint16_t sensor_id;
			0x8E, //uint16_t entity_type;
			0x0014, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
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
			0xC3, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000BDC, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x05, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000C1A, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000C37, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = NUM_DIMM_H_PMIC_PWR,
			.type = sensor_dev_i3c_dimm,
			.port = I3C_BUS3,
			.target_addr = DIMM_PMIC_B_H_ADDR,
			.offset = DIMM_PMIC_SWA_PWR,
			.access_checker = post_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_dimm_i3c_read,
		},
	},
	{
		{
			// DIMMI_PMIC_PWR_W
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
			NUM_DIMM_I_PMIC_PWR, //uint16_t sensor_id;
			0x8E, //uint16_t entity_type;
			0x0015, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
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
			0xC3, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000BDC, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x05, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000C1A, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000C37, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = NUM_DIMM_I_PMIC_PWR,
			.type = sensor_dev_i3c_dimm,
			.port = I3C_BUS3,
			.target_addr = DIMM_PMIC_C_I_ADDR,
			.offset = DIMM_PMIC_SWA_PWR,
			.access_checker = post_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_dimm_i3c_read,
		},
	},
	{
		{
			// DIMMJ_PMIC_PWR_W
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
			NUM_DIMM_J_PMIC_PWR, //uint16_t sensor_id;
			0x8E, //uint16_t entity_type;
			0x0016, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
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
			0xC3, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000BDC, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x05, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000C1A, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000C37, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = NUM_DIMM_J_PMIC_PWR,
			.type = sensor_dev_i3c_dimm,
			.port = I3C_BUS3,
			.target_addr = DIMM_PMIC_D_J_ADDR,
			.offset = DIMM_PMIC_SWA_PWR,
			.access_checker = post_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_dimm_i3c_read,
		},
	},
	{
		{
			// DIMMK_PMIC_PWR_W
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
			NUM_DIMM_K_PMIC_PWR, //uint16_t sensor_id;
			0x8E, //uint16_t entity_type;
			0x0017, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
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
			0xC3, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000BDC, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x05, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000C1A, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000C37, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = NUM_DIMM_K_PMIC_PWR,
			.type = sensor_dev_i3c_dimm,
			.port = I3C_BUS3,
			.target_addr = DIMM_PMIC_E_K_ADDR,
			.offset = DIMM_PMIC_SWA_PWR,
			.access_checker = post_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_dimm_i3c_read,
		},
	},
	{
		{
			// DIMML_PMIC_PWR_W
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
			NUM_DIMM_L_PMIC_PWR, //uint16_t sensor_id;
			0x8E, //uint16_t entity_type;
			0x0018, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
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
			0xC3, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000BDC, //uint32_t max_readable;
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x05, //uint8_t range_field_support;
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000C1A, //uint32_t warning_high;
			0x00000000, //uint32_t warning_low;
			0x00000C37, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = NUM_DIMM_L_PMIC_PWR,
			.type = sensor_dev_i3c_dimm,
			.port = I3C_BUS3,
			.target_addr = DIMM_PMIC_F_L_ADDR,
			.offset = DIMM_PMIC_SWA_PWR,
			.access_checker = post_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_dimm_i3c_read,
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
		.sensor_id = NUM_DIMM_A_TEMP,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"DIMM_A0_TEMP_C",
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
		.sensor_id = NUM_DIMM_B_TEMP,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"DIMM_A1_TEMP_C",
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
		.sensor_id = NUM_DIMM_C_TEMP,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"DIMM_A2_TEMP_C",
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
		.sensor_id = NUM_DIMM_D_TEMP,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"DIMM_A3_TEMP_C",
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
		.sensor_id = NUM_DIMM_E_TEMP,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"DIMM_A4_TEMP_C",
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
		.sensor_id = NUM_DIMM_F_TEMP,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"DIMM_A5_TEMP_C",
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
		.sensor_id = NUM_DIMM_G_TEMP,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"DIMM_A6_TEMP_C",
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
		.sensor_id = NUM_DIMM_H_TEMP,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"DIMM_A7_TEMP_C",
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
		.sensor_id = NUM_DIMM_I_TEMP,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"DIMM_A8_TEMP_C",
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
		.sensor_id = NUM_DIMM_J_TEMP,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"DIMM_A9_TEMP_C",
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
		.sensor_id = NUM_DIMM_K_TEMP,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"DIMM_A10_TEMP_C",
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
		.sensor_id = NUM_DIMM_L_TEMP,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"DIMM_A11_TEMP_C",
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
		// MB_X8_RETIMER_TEMP_C
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
		.sensorName = u"MB_X8_RTM_TEMP_C",
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
		.sensor_id = 0x0020,
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
		.sensor_id = 0x0021,
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
		.sensor_id = 0x0022,
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
		.sensor_id = 0x0023,
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
		.sensor_id = 0x0024,
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
		.sensor_id = 0x0025,
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
		.sensor_id = 0x0026,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_ADC_DIMM0_P12V_VOLT_V",
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
		.sensor_id = 0x0027,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_ADC_DIMM1_P12V_VOLT_V",
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
		.sensor_id = 0x0028,
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
		.sensor_id = 0x0029,
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
		.sensor_id = 0x002A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_ADC_SLOT_DETECT_VOLT_V",
	},
	{
		// MB_ADC_PVDD11_S3_VOLT_V
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
		.sensorName = u"MB_ADC_PVDD11_S3_VOLT_V",
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
		.sensor_id = 0x002C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_ADC_SIDECAR_DETECT_VOLT_V",
	},
	{
		// MB_VR_CPU0_VOLT_V
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
		.sensorName = u"MB_VR_CPU0_VOLT_V",
	},
	{
		// MB_VR_SOC_VOLT_V
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
		.sensorName = u"MB_VR_SOC_VOLT_V",
	},
	{
		// MB_VR_CPU1_VOLT_V
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
		.sensorName = u"MB_VR_CPU1_VOLT_V",
	},
	{
		// MB_VR_PVDDIO_VOLT_V
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
		.sensorName = u"MB_VR_PVDDIO_VOLT_V",
	},
	{
		// MB_VR_PVDD11_VOLT_V
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
		.sensorName = u"MB_VR_PVDD11_VOLT_V",
	},
	{
		// MB_INA233_E1S_Boot_VOLT_V
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
		.sensorName = u"MB_PMON_E1S_BOOT_VOLT_V",
	},
	{
		// MB_INA233_E1S_Data_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_MB_INA233_E1S_DATA_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_PMON_E1S_DATA_VOLT_V",
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
		.sensor_id = 0x0040,
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
		.sensor_id = 0x0041,
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
		.sensor_id = 0x0042,
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
		.sensor_id = 0x0043,
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
		.sensor_id = 0x0044,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_VR_PVDD11_CURR_A",
	},
	{
		// MB_INA233_x8_RTM_CURR_A
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
		.sensorName = u"MB_PMON_X8_RTM_CURR_A",
	},
	{
		// MB_INA233_E1S_Boot_CURR_A
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
		.sensorName = u"MB_PMON_E1S_BOOT_CURR_A",
	},
	{
		// MB_INA233_E1S_Data_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_MB_INA233_E1S_DATA_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_PMON_E1S_DATA_CURR_A",
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
		.sensor_id = 0x0050,
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
		.sensor_id = 0x0051,
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
		.sensor_id = 0x0052,
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
		.sensor_id = 0x0053,
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
		.sensor_id = 0x0054,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_VR_PVDD11_PWR_W",
	},
	{
		// MB_SOC_PACKAGE_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = NUM_SOC_PACKAGE_PWR,
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
		.sensor_id = NUM_DIMM_A_PMIC_PWR,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"DIMM_A0_PWR_W",
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
		.sensor_id = NUM_DIMM_B_PMIC_PWR,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"DIMM_A1_PWR_W",
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
		.sensor_id = NUM_DIMM_C_PMIC_PWR,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"DIMM_A2_PWR_W",
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
		.sensor_id = NUM_DIMM_D_PMIC_PWR,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"DIMM_A3_PWR_W",
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
		.sensor_id = NUM_DIMM_E_PMIC_PWR,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"DIMM_A4_PWR_W",
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
		.sensor_id = NUM_DIMM_F_PMIC_PWR,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"DIMM_A5_PWR_W",
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
		.sensor_id = NUM_DIMM_G_PMIC_PWR,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"DIMM_A6_PWR_W",
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
		.sensor_id = NUM_DIMM_H_PMIC_PWR,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"DIMM_A7_PWR_W",
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
		.sensor_id = NUM_DIMM_I_PMIC_PWR,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"DIMM_A8_PWR_W",
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
		.sensor_id = NUM_DIMM_J_PMIC_PWR,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"DIMM_A9_PWR_W",
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
		.sensor_id = NUM_DIMM_K_PMIC_PWR,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"DIMM_A10_PWR_W",
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
		.sensor_id = NUM_DIMM_L_PMIC_PWR,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"DIMM_A11_PWR_W",
	},
	{
		// MB_INA233_x8_RTM_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0062,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_PMON_X8_RTM_PWR_W",
	},
	{
		// MB_INA233_E1S_Boot_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = 0x0064,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_PMON_E1S_BOOT_PWR_W",
	},
	{
		// MB_INA233_E1S_Data_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_MB_INA233_E1S_DATA_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"MB_PMON_E1S_DATA_PWR_W",
	}
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

uint16_t total_disable_sensors_count = 0;

uint16_t *disable_sensors_id_table = NULL;

void plat_init_pldm_sensor_table()
{
	// Disable/Change sensors according to different config
	plat_pldm_sensor_change_ssd_dev();
	plat_pldm_sensor_change_retimer_dev();
	plat_pldm_sensor_change_vr_dev();
	plat_pldm_sensor_change_cpu_bus();
	plat_pldm_sensor_change_ina_dev();
	plat_pldm_sensor_change_dimm_dev();

	// Initialize sensors that needs to be disabled
	plat_init_pldm_disabled_sensors();
}

void plat_pldm_disable_sensor(sensor_cfg *cfg)
{
	CHECK_NULL_ARG(cfg);

	cfg->cache_status = PLDM_SENSOR_DISABLED;
	total_disable_sensors_count++;
}

void plat_init_pldm_disabled_sensors()
{
	int count = 0;
	uint16_t thread_id = 0;
	uint16_t sensor_id = 0;
	uint16_t current_sensor_count = 0;
	pldm_sensor_info *pldm_sensor_table = NULL;

	if (total_disable_sensors_count == 0) {
		return;
	}

	disable_sensors_id_table =
		(uint16_t *)malloc(total_disable_sensors_count * sizeof(uint16_t));
	if (disable_sensors_id_table == NULL) {
		total_disable_sensors_count = 0;
		LOG_ERR("Failed to allocate disable_sensors_id_table, revise sensors count to 0");
		return;
	}

	for (thread_id = 0; thread_id < MAX_SENSOR_THREAD_ID; ++thread_id) {
		pldm_sensor_table = plat_pldm_sensor_load(thread_id);
		if (pldm_sensor_table == NULL) {
			LOG_ERR("Failed to get pldm sensor table, thread id: 0x%x", thread_id);
			continue;
		}

		count = plat_pldm_sensor_get_sensor_count(thread_id);
		if (count < 0) {
			LOG_ERR("Failed to get pldm sensor table count, thread id: 0x%x",
				thread_id);
			continue;
		}

		for (sensor_id = 0; sensor_id < count; ++sensor_id) {
			if (pldm_sensor_table[sensor_id].pldm_sensor_cfg.cache_status ==
			    PLDM_SENSOR_DISABLED) {
				if (current_sensor_count >= total_disable_sensors_count) {
					LOG_ERR("Record disabled sensor id exceeded table size, total size: 0x%x, current thread id: 0x%x, sensor id: 0x%x",
						total_disable_sensors_count, thread_id,
						pldm_sensor_table[sensor_id]
							.pdr_numeric_sensor.sensor_id);
					continue;
				}

				disable_sensors_id_table[current_sensor_count] =
					pldm_sensor_table[sensor_id].pdr_numeric_sensor.sensor_id;
				current_sensor_count++;
			}
		}
	}
}

uint16_t plat_get_disabled_sensor_count()
{
	return total_disable_sensors_count;
}

uint32_t plat_get_pdr_size(uint8_t pdr_type)
{
	int total_size = 0, i = 0;

	switch (pdr_type) {
	case PLDM_NUMERIC_SENSOR_PDR:
		for (i = 0; i < MAX_SENSOR_THREAD_ID; i++) {
			total_size += plat_pldm_sensor_get_sensor_count(i);
		}
		total_size -= total_disable_sensors_count;
		break;
	case PLDM_SENSOR_AUXILIARY_NAMES_PDR:
		total_size = ARRAY_SIZE(plat_pdr_sensor_aux_names_table);
		total_size -= total_disable_sensors_count;
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
	case VR_SENSOR_THREAD_ID:
		return plat_pldm_sensor_vr_table;
	case MB_TEMP_SENSOR_THREAD_ID:
		return plat_pldm_sensor_mb_temp_table;
	case CPU_SENSOR_THREAD_ID:
		return plat_pldm_sensor_cpu_table;
	case INA233_SENSOR_THREAD_ID:
		return plat_pldm_sensor_ina233_table;
	case DIMM_SENSOR_THREAD_ID:
		return plat_pldm_sensor_dimm_table;
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
	case INA233_SENSOR_THREAD_ID:
		count = ARRAY_SIZE(plat_pldm_sensor_ina233_table);
		break;
	case DIMM_SENSOR_THREAD_ID:
		count = ARRAY_SIZE(plat_pldm_sensor_dimm_table);
		break;
	default:
		count = -1;
		LOG_ERR("Unknow pldm sensor thread id %d", thread_id);
		break;
	}

	return count;
}

void plat_load_numeric_sensor_pdr_table(PDR_numeric_sensor *numeric_sensor_table)
{
	int thread_id = 0, sensor_num = 0;
	int max_sensor_num = 0, current_sensor_size = 0;
	uint32_t total_size = plat_get_pdr_size(PLDM_NUMERIC_SENSOR_PDR);
	pldm_sensor_info *pdr_table = NULL;

	for (thread_id = 0; thread_id < MAX_SENSOR_THREAD_ID; thread_id++) {
		pdr_table = plat_pldm_sensor_load(thread_id);
		if (pdr_table == NULL) {
			LOG_ERR("Failed to get pdr table, thread id: 0x%x", thread_id);
			continue;
		}

		max_sensor_num = plat_pldm_sensor_get_sensor_count(thread_id);
		if (max_sensor_num < 0) {
			LOG_ERR("Failed to get sensor count, thread id: 0x%x", thread_id);
			continue;
		}

		for (sensor_num = 0; sensor_num < max_sensor_num; sensor_num++) {
			if (pdr_table[sensor_num].pldm_sensor_cfg.cache_status !=
			    PLDM_SENSOR_DISABLED) {
				if (current_sensor_size >= total_size) {
					LOG_ERR("Load numeric sensor pdr exceeded table size, total size: 0x%x, current thread: 0x%x, sensor id: 0x%x",
						total_size, thread_id,
						pdr_table[sensor_num].pdr_numeric_sensor.sensor_id);
					continue;
				}

				memcpy(&numeric_sensor_table[current_sensor_size],
				       &pdr_table[sensor_num].pdr_numeric_sensor,
				       sizeof(PDR_numeric_sensor));
				current_sensor_size++;
			}
		}
	}
}

void plat_load_aux_sensor_names_pdr_table(PDR_sensor_auxiliary_names *aux_sensor_name_table)
{
	int index = 0;
	int disable_sensor_id = 0;
	int current_sensor_size = 0;
	uint32_t total_size = plat_get_pdr_size(PLDM_SENSOR_AUXILIARY_NAMES_PDR);

	for (index = 0; index < ARRAY_SIZE(plat_pdr_sensor_aux_names_table); ++index) {
		for (disable_sensor_id = 0; disable_sensor_id < total_disable_sensors_count;
		     ++disable_sensor_id) {
			if (plat_pdr_sensor_aux_names_table[index].sensor_id ==
			    disable_sensors_id_table[disable_sensor_id]) {
				break;
			}
		}

		if (disable_sensor_id >= total_disable_sensors_count) {
			// AUX sensor name isn't disabled sensor
			if (current_sensor_size >= total_size) {
				LOG_ERR("Load aux sensor names exceeded table size, current sensors size: 0x%x, sensor id: 0x%x",
					current_sensor_size,
					plat_pdr_sensor_aux_names_table[index].sensor_id);
				continue;
			}

			memcpy(&aux_sensor_name_table[current_sensor_size],
			       &plat_pdr_sensor_aux_names_table[index],
			       sizeof(PDR_sensor_auxiliary_names));
			current_sensor_size++;
		}
	}
}

#ifdef ENABLE_PLATFORM_PROVIDES_PLDM_SENSOR_STACKS
k_thread_stack_t *plat_get_pldm_sensor_stack(int thread_id)
{
	if (thread_id >= 0 && thread_id < MAX_SENSOR_THREAD_ID) {
		return pldm_sensor_poll_stacks[thread_id];
	}
	return NULL;
}

size_t plat_get_pldm_sensor_stack_size(int thread_id)
{
	if (thread_id >= 0 && thread_id < MAX_SENSOR_THREAD_ID) {
		return pldm_sensor_stack_sizes[thread_id];
	}
	return PLDM_SENSOR_POLL_STACK_SIZE;
}
#endif

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

// Custom function to concatenate a char16_t character to a string
char16_t *char16_strcat_char(char16_t *dest, char16_t ch)
{
	size_t len = char16_strlen(dest);
	dest[len] = ch;
	dest[len + 1] = u'\0';
	return dest;
}

void plat_init_entity_aux_names_pdr_table()
{
	// Base name
	const char16_t base_name[] = u"SENTINEL_DOME_SLOT_";

	// Get slot ID
	uint8_t slot_id = get_slot_id();

	// Calculate the length of the base name
	size_t base_len = char16_strlen(base_name);

	// Calculate the required length for the final string (base name + 1 digit + null terminator)
	size_t total_len = base_len + 2; // +2 for the slot ID digit and null terminator

	// Ensure the final length does not exceed MAX_AUX_SENSOR_NAME_LEN
	if (total_len > MAX_AUX_SENSOR_NAME_LEN) {
		total_len = MAX_AUX_SENSOR_NAME_LEN;
	}

	// Create a buffer for the full name
	char16_t full_name[MAX_AUX_SENSOR_NAME_LEN] = { 0 };

	// Copy base name to full name, with length limit
	char16_strcpy(full_name, base_name);

	// Append slot ID as a character, ensuring it fits within the buffer
	if (base_len + 1 < MAX_AUX_SENSOR_NAME_LEN) {
		char16_strcat_char(full_name, u'0' + slot_id);
	}

	// Now copy the full name to the entityName field of your structure
	char16_strcpy(plat_pdr_entity_aux_names_table[0].entityName, full_name);

	plat_pdr_entity_aux_names_table_size =
		sizeof(PDR_entity_auxiliary_names) + (total_len * sizeof(char16_t));
}

void plat_load_entity_aux_names_pdr_table(PDR_entity_auxiliary_names *entity_aux_name_table)
{
	memcpy(entity_aux_name_table, &plat_pdr_entity_aux_names_table,
	       plat_pdr_entity_aux_names_table_size);
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
void plat_pldm_sensor_change_ssd_dev()
{
	uint16_t disable_ina233_sensors_id[] = { SENSOR_NUM_MB_INA233_E1S_DATA_VOLT_V,
						 SENSOR_NUM_MB_INA233_E1S_DATA_CURR_A,
						 SENSOR_NUM_MB_INA233_E1S_DATA_PWR_W };
	uint8_t blade_conf = BLADE_CONFIG_UNKNOWN;
	if (get_blade_config(&blade_conf) == false) {
		LOG_ERR("Unable to change the ssd device due to its unknown blade config.");
		return;
	}

	// T1C system doesn't have E1.S data drive and WF, and only have 10 DIMMs.
	if (blade_conf == BLADE_CONFIG_T1C) {
		for (int index = 0;
		     index < plat_pldm_sensor_get_sensor_count(MB_TEMP_SENSOR_THREAD_ID); index++) {
			if (plat_pldm_sensor_mb_temp_table[index].pldm_sensor_cfg.port ==
				    I2C_BUS6 &&
			    plat_pldm_sensor_mb_temp_table[index].pldm_sensor_cfg.target_addr ==
				    ADDR_NVME) {
				plat_pldm_disable_sensor(
					&plat_pldm_sensor_mb_temp_table[index].pldm_sensor_cfg);
			}
		}

		for (int disable_id = 0; disable_id < ARRAY_SIZE(disable_ina233_sensors_id);
		     ++disable_id) {
			for (int index = 0;
			     index < plat_pldm_sensor_get_sensor_count(INA233_SENSOR_THREAD_ID);
			     index++) {
				if (disable_ina233_sensors_id[disable_id] ==
				    plat_pldm_sensor_ina233_table[index]
					    .pdr_numeric_sensor.sensor_id) {
					plat_pldm_disable_sensor(
						&plat_pldm_sensor_ina233_table[index]
							 .pldm_sensor_cfg);
					break;
				}
			}
		}
	}
}
void plat_pldm_sensor_change_cpu_bus()
{
	uint8_t board_rev = BOARD_REV_EVT;

	if (get_board_rev(&board_rev) == false) {
		LOG_ERR("Failed to get board revision.");
		return;
	}

	if (board_rev > BOARD_REV_EVT) {
		// For DVT and later, the hardware design was changed to I2C_BUS10
		for (int i = 0; i < plat_pldm_sensor_get_sensor_count(CPU_SENSOR_THREAD_ID); i++) {
			plat_pldm_sensor_cpu_table[i].pldm_sensor_cfg.port = I2C_BUS10;
		}
	}

	return;
}

void plat_pldm_disable_ina233_sensor_with_no_retimer()
{
	int index = 0;
	int disable_id = 0;
	int max_sensor_count = plat_pldm_sensor_get_sensor_count(INA233_SENSOR_THREAD_ID);
	uint16_t disable_ina233_sensors_id[] = { SENSOR_NUM_MB_INA233_X8_RTM_CURR_A,
						 SENSOR_NUM_MB_INA233_X8_RTM_PWR_W };

	if (max_sensor_count < 0) {
		LOG_ERR("Failed to get ina233 sensors count");
		return;
	}

	for (disable_id = 0; disable_id < ARRAY_SIZE(disable_ina233_sensors_id); ++disable_id) {
		for (index = 0; index < max_sensor_count; ++index) {
			if (disable_ina233_sensors_id[disable_id] ==
			    plat_pldm_sensor_ina233_table[index].pdr_numeric_sensor.sensor_id) {
				plat_pldm_disable_sensor(
					&plat_pldm_sensor_ina233_table[index].pldm_sensor_cfg);
				break;
			}
		}
	}
}

void plat_pldm_sensor_change_retimer_dev()
{
	uint8_t retimer_type = get_retimer_type();

	for (int index = 0; index < plat_pldm_sensor_get_sensor_count(INA233_SENSOR_THREAD_ID);
	     index++) {
		if (plat_pldm_sensor_ina233_table[index].pldm_sensor_cfg.type ==
		    sensor_dev_pt5161l) {
			switch (retimer_type) {
			case RETIMER_TYPE_ASTERALABS:
				// Default setting in pldm sensor table
				return;
			case RETIMER_TYPE_NO_RETIMER:
				// No need to get sensor reading from retimer
				plat_pldm_disable_sensor(
					&plat_pldm_sensor_ina233_table[index].pldm_sensor_cfg);
				break;
			case RETIMER_TYPE_KANDOU:
				// TODO: Currently, disable sensor reading until support kandou
				plat_pldm_disable_sensor(
					&plat_pldm_sensor_ina233_table[index].pldm_sensor_cfg);
				break;
			case RETIMER_TYPE_BROADCOM:
				plat_pldm_sensor_ina233_table[index].pldm_sensor_cfg.type =
					sensor_dev_bcm85658;
				plat_pldm_sensor_ina233_table[index].pldm_sensor_cfg.offset =
					BCM85658_TEMP_OFFSET;
				plat_pldm_sensor_ina233_table[index].pldm_sensor_cfg.init_args =
					NULL;
				plat_pldm_sensor_ina233_table[index]
					.pldm_sensor_cfg.pre_sensor_read_hook = NULL;
				break;
			default:
				LOG_ERR("Failed to change the Retimer device due to unknown vendor.");
				break;
			}
		}
	}

	if (retimer_type == RETIMER_TYPE_NO_RETIMER) {
		plat_pldm_disable_ina233_sensor_with_no_retimer();
	}
}

void plat_pldm_sensor_change_ina_dev()
{
	uint8_t ina_dev = plat_pldm_sensor_get_ina_dev();
	for (int index = 0; index < plat_pldm_sensor_get_sensor_count(INA233_SENSOR_THREAD_ID);
	     index++) {
		sensor_cfg *cfg = &plat_pldm_sensor_ina233_table[index].pldm_sensor_cfg;
		if (cfg->port == I2C_BUS6 && cfg->target_addr == ADDR_X8_INA233) {
			cfg->type = ina_dev;
			if (ina_dev == sensor_dev_rtq6056) {
				cfg->init_args = &rtq6056_init_args[0];
			} else if (ina_dev == sensor_dev_sq52205) {
				cfg->init_args = &sq52205_init_args[0];
			}
		} else if (cfg->port == I2C_BUS1 && cfg->target_addr == ADDR_E1S_BOOT_INA233) {
			cfg->type = ina_dev;
			if (ina_dev == sensor_dev_rtq6056) {
				cfg->init_args = &rtq6056_init_args[2];
			} else if (ina_dev == sensor_dev_sq52205) {
				cfg->init_args = &sq52205_init_args[2];
			}
		} else if (cfg->port == I2C_BUS6 && cfg->target_addr == ADDR_E1S_DATA_INA233) {
			cfg->type = ina_dev;
			if (ina_dev == sensor_dev_rtq6056) {
				cfg->init_args = &rtq6056_init_args[3];
			} else if (ina_dev == sensor_dev_sq52205) {
				cfg->init_args = &sq52205_init_args[3];
			}
		}

		if (ina_dev == sensor_dev_sq52205) {
			switch (cfg->offset) {
			case PMBUS_READ_VOUT:
				cfg->offset = SQ52205_READ_VOL_OFFSET;
				break;
			case PMBUS_READ_IOUT:
				cfg->offset = SQ52205_READ_CUR_OFFSET;
				break;
			case PMBUS_READ_POUT:
				cfg->offset = SQ52205_READ_PWR_OFFSET;
				break;
			default:
				LOG_ERR("UNKNOWN power monitor offset %x change table failed",
					cfg->offset);
				continue;
			}
		}
	}
}

void plat_pldm_sensor_change_dimm_dev()
{
	int index = 0;
	int disable_id = 0;
	int max_sensor_count = plat_pldm_sensor_get_sensor_count(DIMM_SENSOR_THREAD_ID);
	if (max_sensor_count < 0) {
		LOG_ERR("Failed to get dimm sensors count");
		return;
	}

	uint8_t blade_conf = BLADE_CONFIG_UNKNOWN;
	uint16_t t1c_disable_dimm_sensors_id[] = { NUM_DIMM_F_TEMP, NUM_DIMM_L_TEMP,
						   NUM_DIMM_F_PMIC_PWR, NUM_DIMM_L_PMIC_PWR };

	if (get_blade_config(&blade_conf) == false) {
		LOG_ERR("Unable to change the ssd device due to its unknown blade config.");
		return;
	}

	// T1C system doesn't have E1.S data drive and WF, and only have 10 DIMMs.
	if (blade_conf == BLADE_CONFIG_T1C) {
		for (disable_id = 0; disable_id < ARRAY_SIZE(t1c_disable_dimm_sensors_id);
		     ++disable_id) {
			for (index = 0; index < max_sensor_count; ++index) {
				if (t1c_disable_dimm_sensors_id[disable_id] ==
				    plat_pldm_sensor_dimm_table[index].pdr_numeric_sensor.sensor_id) {
					plat_pldm_disable_sensor(
						&plat_pldm_sensor_dimm_table[index].pldm_sensor_cfg);
					break;
				}
			}
		}
	}
}

uint8_t plat_pldm_sensor_get_vr_dev(uint8_t *vr_dev)
{
	/*
	 * GPIO VR_TYPE_0 and VR_TYPE_1 are used to determine the VR type.
	 * 
	 * VR_TYPE[1:0]
	 * 00 - MPS
	 * 10 - RNS
	 * 01 - IFX
	 * 11 - TI
	 */

	int high_byte = gpio_get(VR_TYPE_1);
	int low_byte = gpio_get(VR_TYPE_0);

	if (high_byte == HIGH_ACTIVE) {
		if (low_byte == HIGH_ACTIVE) {
			*vr_dev = sensor_dev_tps53689;
		} else if (low_byte == HIGH_INACTIVE) {
			*vr_dev = sensor_dev_raa229621;
		} else {
			goto error_exit;
		}
	} else if (high_byte == HIGH_INACTIVE) {
		if (low_byte == HIGH_ACTIVE) {
			*vr_dev = sensor_dev_xdpe19283b;
		} else if (low_byte == HIGH_INACTIVE) {
			*vr_dev = sensor_dev_mp2856gut;
		} else {
			goto error_exit;
		}
	} else {
		goto error_exit;
	}
	return GET_VR_DEV_SUCCESS;

error_exit:
	LOG_ERR("Unable to get VR device due to unknown VR_TYPE_1:%d VR_TYPE_0:%d", high_byte,
		low_byte);
	*vr_dev = VR_DEVICE_UNKNOWN;
	return GET_VR_DEV_FAILED;
}

void plat_pldm_sensor_clear_vr_fault(uint8_t vr_addr, uint8_t vr_bus, uint8_t page_cnt)
{
	// Clear VR fault by command code 03h - CLEAR_FAULTS
	uint8_t retry = 5;
	int ret = 0;
	I2C_MSG msg = { 0 };
	msg.bus = vr_bus;
	msg.target_addr = vr_addr;

	for (int page = 0; page < page_cnt; ++page) {
		/* set page for power rail */
		msg.tx_len = 2;
		msg.data[0] = PMBUS_PAGE;
		msg.data[1] = page;
		if (i2c_master_write(&msg, retry)) {
			LOG_ERR("Set page failed, bus: 0x%x, addr: 0x%x, page: %d", msg.bus,
				msg.target_addr, page);
			continue;
		}

		/* write CLEAR_FAULTS */
		msg.tx_len = 1;
		msg.data[0] = PMBUS_CLEAR_FAULTS;
		ret = i2c_master_write(&msg, retry);
		if (ret != 0) {
			LOG_ERR("Clear faults failed, bus: 0x%x, addr: 0x%x", msg.bus,
				msg.target_addr);
			continue;
		}
	}
}

uint8_t plat_pldm_sensor_get_ina_dev()
{
	int retry = 5;
	I2C_MSG msg = { 0 };

	// read boot drive to check
	// check INA233 device
	msg.bus = I2C_BUS1;
	msg.target_addr = ADDR_E1S_BOOT_INA233;
	msg.tx_len = 1;
	msg.rx_len = 3;
	msg.data[0] = PMBUS_MFR_ID;
	if (i2c_master_read(&msg, retry) != 0) {
		LOG_ERR("Failed to get MFR ID from boot INA233");
	} else {
		if (memcmp(msg.data, INA233_DEVICE_ID, sizeof(INA233_DEVICE_ID)) == 0) {
			LOG_INF("use INA233");
			return sensor_dev_ina233;
		}
	}

	// check RTQ6056 device
	msg.bus = I2C_BUS1;
	msg.target_addr = ADDR_E1S_BOOT_INA233;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = RTQ6056_MFR_ID_REG;
	if (i2c_master_read(&msg, retry) != 0) {
		LOG_ERR("Failed to get MFR ID from boot RTQ6056");
	} else {
		if (memcmp(msg.data, RTQ6056_DEVICE_ID, sizeof(RTQ6056_DEVICE_ID)) == 0) {
			LOG_INF("use RQT6056");
			return sensor_dev_rtq6056;
		}
	}

	// since sq52205 does not support MFR_ID, we set it as default
	LOG_INF("use SQ52205");
	return sensor_dev_sq52205;
}

void set_bootdrive_exist_status()
{
	int retry = 5;
	I2C_MSG msg = { 0 };

	// read CPLD to check if bootdrive exist
	msg.bus = CPLD_IO_I2C_BUS;
	msg.target_addr = CPLD_IO_I2C_ADDR;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = OFFSET_CARD_PRSNT;

	if (i2c_master_read(&msg, retry) != 0) {
		LOG_ERR("Failed to get BootDrive PRSNT from CPLD IOE");
		return;
	} else {
		bootdrive_exist = !(msg.data[0] & (1 << 6));
		LOG_ERR("Return Value is %x, Bootdrive %s", msg.data[0],
			bootdrive_exist ? "exist" : "does not exist");
	}
}

bool get_bootdrive_exist_status()
{
	return bootdrive_exist;
}

bool bootdrive_access(uint8_t sensor_num)
{
	// If the bootdrice exist, wait for post_completed then start to monitor
	if (get_bootdrive_exist_status()) {
		return post_access(sensor_num);
	}

	// If the bootdrive does not exist, let the sensor do not stock in init status
	return true;
}

void start_monitor_prochot_sensor_thread()
{
	LOG_INF("Start thread to monitor PROCHOT sensor");
	monitor_prochot_sensor_tid =
		k_thread_create(&monitor_prochot_sensor_thread, monitor_prochot_sensor_stack,
				K_THREAD_STACK_SIZEOF(monitor_prochot_sensor_stack),
				monitor_prochot_sensor_handler, NULL, NULL, NULL, K_PRIO_PREEMPT(1),
				0, K_NO_WAIT);

	k_thread_name_set(&monitor_prochot_sensor_thread, "monitor_prochot_sensor_thread");
}

const prochot_sensor_info prochot_sensor_table[PROCHOT_SENSOR_TABLE_LEN] = {
	// { sensor_id, event_bit }
	{ NUM_DIMM_A_TEMP, BIT(0) }, // DIMM_A0_TEMP_C
	{ NUM_DIMM_B_TEMP, BIT(0) }, // DIMM_A1_TEMP_C
	{ NUM_DIMM_C_TEMP, BIT(1) }, // DIMM_A2_TEMP_C
	{ NUM_DIMM_D_TEMP, BIT(1) }, // DIMM_A3_TEMP_C
	{ NUM_DIMM_E_TEMP, BIT(2) }, // DIMM_A4_TEMP_C
	{ NUM_DIMM_F_TEMP, BIT(2) }, // DIMM_A5_TEMP_C
	{ NUM_DIMM_G_TEMP, BIT(3) }, // DIMM_A6_TEMP_C
	{ NUM_DIMM_H_TEMP, BIT(3) }, // DIMM_A7_TEMP_C
	{ NUM_DIMM_I_TEMP, BIT(4) }, // DIMM_A8_TEMP_C
	{ NUM_DIMM_J_TEMP, BIT(4) }, // DIMM_A9_TEMP_C
	{ NUM_DIMM_K_TEMP, BIT(5) }, // DIMM_A10_TEMP_C
	{ NUM_DIMM_L_TEMP, BIT(5) }, // DIMM_A11_TEMP_C
	{ 0x0004, BIT(6) }, // MB_CPU_TEMP_C
	{ 0x0013, BIT(7) }, // MB_VR_CPU0_TEMP_C
	{ 0x0015, BIT(8) }, // MB_VR_CPU1_TEMP_C
	{ 0x0017, BIT(9) }, // MB_VR_PVDD11_TEMP_C
	{ 0x0016, BIT(10) }, // MB_VR_PVDDIO_TEMP_C
	{ 0x0014, BIT(11) }, // MB_VR_SOC_TEMP_C
};

void monitor_prochot_sensor_handler()
{
	PDR_numeric_sensor sensor_pdr;
	uint16_t sensor_id;
	float critical_high;
	float sensor_reading = 0, decimal = 0, resolution = 0, offset = 0;
	int cache_reading = 0;
	int8_t unit_modifier = 0;
	int16_t integer = 0;
	uint8_t sensor_operational_state = PLDM_SENSOR_STATUSUNKOWN;
	static bool event_status[PROCHOT_SENSOR_TABLE_LEN] = { 0 };
	uint16_t event_bit = 0;
	static uint16_t event_bit_history = 0;

	while (1) {
		k_msleep(MONITOR_PROCHOT_SENSOR_TIME_MS);

		// Check DC on
		if (gpio_get(RST_PLTRST_BIC_N) == GPIO_LOW) {
			continue;
		}
		// Check sensor poll enable
		if (get_sensor_poll_enable_flag() == false) {
			continue;
		}

		for (int i = 0; i < PROCHOT_SENSOR_TABLE_LEN; i++) {
			sensor_id = prochot_sensor_table[i].sensor_id;
			sensor_reading = 0;
			sensor_operational_state = PLDM_SENSOR_STATUSUNKOWN;

			if (pldm_sensor_get_info_via_sensor_id(sensor_id, &resolution, &offset,
							       &unit_modifier, &cache_reading,
							       &sensor_operational_state) != 0) {
				LOG_ERR("sensor id: 0x%04x could not be found in pldm_sensor_list",
					sensor_id);
				// Keep previous status
				continue;
			}
			if (sensor_operational_state != PLDM_SENSOR_ENABLED) {
				// Keep previous status
				continue;
			}

			// Convert two byte integer, two byte decimal sensor format to float
			integer = cache_reading & 0xffff;
			decimal = (float)(cache_reading >> 16) / 1000.0;

			if (integer >= 0) {
				sensor_reading = (float)integer + decimal;
			} else {
				sensor_reading = (float)integer - decimal;
			}
			// Filter abnormal reading
			if (sensor_reading > 255) {
				LOG_ERR("sensor id: 0x%04x, reading = %d is abnormal (over 255)",
					sensor_id, (int)sensor_reading);
				// Keep previous status
				continue;
			}

			if (get_pdr_with_sensor_id(sensor_id, &sensor_pdr) != 0) {
				LOG_ERR("sensor id: 0x%4x get pdr failed", sensor_id);
				// Keep previous status
				continue;
			}

			// Sensor support upperThresholdCritical
			if ((sensor_pdr.supported_thresholds & BIT(1)) == 0) {
				LOG_ERR("sensor id: 0x%04x unsupported upper critical threshold",
					sensor_id);
				// Keep previous status
				continue;
			}
			critical_high =
				sensor_pdr.critical_high * power(10, sensor_pdr.unit_modifier);

			if (sensor_reading > critical_high) {
				event_status[i] = true;
			} else {
				event_status[i] = false;
			}
		}

		event_bit = 0;
		for (size_t i = 0; i < PROCHOT_SENSOR_TABLE_LEN; i++) {
			if (event_status[i]) {
				event_bit |= prochot_sensor_table[i].event_bit;
			}
		}

		if (event_bit != event_bit_history) {
			event_bit_history = event_bit;

			if (event_bit != 0) {
				gpio_set(FM_BMC_DEBUG_ENABLE_R_N, GPIO_LOW);
				LOG_INF("Set FM_BMC_DEBUG_ENABLE_R_N to low");
			} else {
				gpio_set(FM_BMC_DEBUG_ENABLE_R_N, GPIO_HIGH);
				LOG_INF("Set FM_BMC_DEBUG_ENABLE_R_N to high");
			}

			struct pldm_addsel_data sel_msg = { 0 };

			sel_msg.assert_type = (event_bit != 0) ? EVENT_ASSERTED : EVENT_DEASSERTED;
			sel_msg.event_type = PROCHOT_TRIGGERED_BY_SENSOR_UCR;
			sel_msg.event_data_2 = event_bit >> 8;
			sel_msg.event_data_3 = event_bit & 0xFF;

			if (PLDM_SUCCESS != send_event_log_to_bmc(sel_msg)) {
				LOG_ERR("Failed to send sensor UCR triggered Prochot SEL, event data: 0x%02x 0x%02x 0x%02x",
					sel_msg.event_data_1, sel_msg.event_data_2,
					sel_msg.event_data_3);
			} else {
				LOG_INF("Send sensor UCR triggered Prochot SEL, event data: 0x%02x 0x%02x 0x%02x",
					sel_msg.event_data_1, sel_msg.event_data_2,
					sel_msg.event_data_3);
			}
		}
	}
}
