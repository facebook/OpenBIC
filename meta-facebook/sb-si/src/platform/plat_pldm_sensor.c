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
#include "ads7830.h"
#include "pex90144.h"
#include "pdr.h"
#include "tmp431.h"
#include "sensor.h"
#include "pldm_sensor.h"
#include "pldm_monitor.h"
#include "plat_hook.h"
#include "plat_i2c.h"
#include "plat_pldm_sensor.h"
#include "plat_class.h"
#include "plat_gpio.h"
#include "emc1413.h"

LOG_MODULE_REGISTER(plat_pldm_sensor);

static bool plat_sensor_polling_enable_flag = true;
static bool plat_sensor_adc_polling_enable_flag = true;
static bool plat_sensor_temp_polling_enable_flag = true;
static bool plat_sensor_vr_polling_enable_flag = true;

void find_vr_addr_by_sensor_id(uint8_t sensor_id, uint8_t *vr_addr);
typedef struct plat_sensor_vr_extend_info {
	uint16_t sensor_id;
	uint8_t target_rns_addr; // ISL69260 or RAA228238
	void *mps_vr_init_args;
	void *rns_vr_init_args;
} plat_sensor_vr_extend_info;

typedef struct plat_sensor_tmp_extend_info {
	uint16_t sensor_id;
	uint8_t target_emc1413_addr;
	uint16_t offset;
} plat_sensor_tmp_extend_info;

plat_sensor_vr_extend_info plat_sensor_vr_extend_table[] = {
	{ SENSOR_NUM_VR_ASIC_P0V895_PEX_TEMP_C, VR_ASIC_P0V895_PEX_MP2971_ADDR },
	{ SENSOR_NUM_VR_ASIC_P0V895_PEX_VOLT_V, VR_ASIC_P0V895_PEX_MP2971_ADDR },
	{ SENSOR_NUM_VR_ASIC_P0V895_PEX_CURR_A, VR_ASIC_P0V895_PEX_MP2971_ADDR },
	{ SENSOR_NUM_VR_ASIC_P0V895_PEX_PWR_W, VR_ASIC_P0V895_PEX_MP2971_ADDR },

	{ SENSOR_NUM_VR_ASIC_P0V825_A0_TEMP_C, VR_ASIC_P0V825_A0_MP2971_ADDR },
	{ SENSOR_NUM_VR_ASIC_P0V825_A0_VOLT_V, VR_ASIC_P0V825_A0_MP2971_ADDR },
	{ SENSOR_NUM_VR_ASIC_P0V825_A0_CURR_A, VR_ASIC_P0V825_A0_MP2971_ADDR },
	{ SENSOR_NUM_VR_ASIC_P0V825_A0_PWR_W, VR_ASIC_P0V825_A0_MP2971_ADDR },

	{ SENSOR_NUM_VR_ASIC_P0V825_A1_TEMP_C, VR_ASIC_P0V825_A1_MP2971_ADDR },
	{ SENSOR_NUM_VR_ASIC_P0V825_A1_VOLT_V, VR_ASIC_P0V825_A1_MP2971_ADDR },
	{ SENSOR_NUM_VR_ASIC_P0V825_A1_CURR_A, VR_ASIC_P0V825_A1_MP2971_ADDR },
	{ SENSOR_NUM_VR_ASIC_P0V825_A1_PWR_W, VR_ASIC_P0V825_A1_MP2971_ADDR },

	{ SENSOR_NUM_VR_ASIC_P0V825_A2_TEMP_C, VR_ASIC_P0V825_A2_MP2971_ADDR },
	{ SENSOR_NUM_VR_ASIC_P0V825_A2_VOLT_V, VR_ASIC_P0V825_A2_MP2971_ADDR },
	{ SENSOR_NUM_VR_ASIC_P0V825_A2_CURR_A, VR_ASIC_P0V825_A2_MP2971_ADDR },
	{ SENSOR_NUM_VR_ASIC_P0V825_A2_PWR_W, VR_ASIC_P0V825_A2_MP2971_ADDR },

};

plat_sensor_tmp_extend_info plat_sensor_tmp_extend_table[] = {};

static struct pldm_sensor_thread pal_pldm_sensor_thread[MAX_SENSOR_THREAD_ID] = {
	// thread id, thread name
	{ VR_SENSOR_THREAD_ID, "VR_PLDM_SENSOR_THREAD" },
	{ TEMP_SENSOR_THREAD_ID, "TEMP_SENSOR_THREAD" },
	{ ADC_SENSOR_THREAD_ID, "ADC_SENSOR_THREAD" },
};

pldm_sensor_info plat_pldm_sensor_temp_table[] = {
	{
		{
			// SENSOR_NUM_THERMAL_SENSOR_1_TEMP_C
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
			SENSOR_NUM_THERMAL_SENSOR_1_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type;
			0x0001, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
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
			.num = SENSOR_NUM_THERMAL_SENSOR_1_TEMP_C,
			.type = sensor_dev_tmp75,
			.port = I2C_BUS1,
			.target_addr = THERMAL_SENSOR_1_ADDR,
			.offset = TMP75_TEMP_OFFSET,
			.access_checker = is_temp_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// THERMAL_SENSOR_2_TEMP_C
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
			SENSOR_NUM_THERMAL_SENSOR_2_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type;
			0x0002, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
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
			.num = SENSOR_NUM_THERMAL_SENSOR_2_TEMP_C,
			.type = sensor_dev_tmp75,
			.port = I2C_BUS1,
			.target_addr = THERMAL_SENSOR_2_ADDR,
			.offset = TMP75_TEMP_OFFSET,
			.access_checker = is_temp_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// PCIE_SWITCH_TEMP_C
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
			SENSOR_NUM_PCIE_SWITCH_PEX90144_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type;
			0x0002, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
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
			UP_THRESHOLD_CRIT, //uint8_t supported_thresholds;
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
			100000, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_PCIE_SWITCH_PEX90144_TEMP_C,
			.type = sensor_dev_pex90144,
			.port = I2C_BUS4,
			.target_addr = PCIE_SWITCH_ADDR,
			.offset = PEX_TEMP,
			.access_checker = is_pcie_switch_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.init_args = &pex_sensor_init_args[0],
		},
	},
};

pldm_sensor_info plat_pldm_sensor_vr_table[] = {
	{
		{
			// VR_ASIC_P0V895_PEX_TEMP_C
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
			SENSOR_NUM_VR_ASIC_P0V895_PEX_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type;
			0x0003, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
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
			UP_THRESHOLD_CRIT, //uint8_t supported_thresholds;
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
			100000, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_VR_ASIC_P0V895_PEX_TEMP_C,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS1,
			.target_addr = VR_ASIC_P0V895_PEX_MP2971_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V895 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V895 * 2],
		},
	},
	{
		{
			// VR_ASIC_P0V895_PEX_VOLT_V
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
			SENSOR_NUM_VR_ASIC_P0V895_PEX_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type;
			0x0004, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
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
			UP_THRESHOLD_CRIT | LOW_THRESHOLD_CRIT, //uint8_t supported_thresholds;
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
			920, //uint32_t critical_high;
			877, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_VR_ASIC_P0V895_PEX_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS1,
			.target_addr = VR_ASIC_P0V895_PEX_MP2971_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V895 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V895 * 2],
		},
	},
	{
		{
			// VR_ASIC_P0V895_PEX_CURR_A
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
			SENSOR_NUM_VR_ASIC_P0V895_PEX_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type;
			0x0005, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
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
			UP_THRESHOLD_CRIT, //uint8_t supported_thresholds;
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
			96800, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_VR_ASIC_P0V895_PEX_CURR_A,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS1,
			.target_addr = VR_ASIC_P0V895_PEX_MP2971_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V895 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V895 * 2],
		},
	},
	{
		{
			// VR_ASIC_P0V895_PEX_PWR_W
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
			SENSOR_NUM_VR_ASIC_P0V895_PEX_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type;
			0x0006, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
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
			UP_THRESHOLD_CRIT, //uint8_t supported_thresholds;
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
			89056, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_VR_ASIC_P0V895_PEX_PWR_W,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS1,
			.target_addr = VR_ASIC_P0V895_PEX_MP2971_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V895 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V895 * 2],
		},
	},
	{
		{
			// VR_ASIC_P0V825_A0_TEMP_C
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
			SENSOR_NUM_VR_ASIC_P0V895_PEX_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type;
			0x0007, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
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
			UP_THRESHOLD_CRIT, //uint8_t supported_thresholds;
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
			100000, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_VR_ASIC_P0V825_A0_TEMP_C,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS1,
			.target_addr = VR_ASIC_P0V825_A0_MP2971_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V895 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V895 * 2 + 1],
		},
	},
	{
		{
			// VR_ASIC_P0V825_A0_VOLT_V
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
			SENSOR_NUM_VR_ASIC_P0V825_A0_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type;
			0x0008, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
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
			UP_THRESHOLD_CRIT | LOW_THRESHOLD_CRIT, //uint8_t supported_thresholds;
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
			842, //uint32_t critical_high;
			775, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_VR_ASIC_P0V825_A0_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS1,
			.target_addr = VR_ASIC_P0V825_A0_MP2971_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V895 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V895 * 2 + 1],
		},
	},
	{
		{
			// VR_ASIC_P0V825_A0_CURR_A
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
			SENSOR_NUM_VR_ASIC_P0V825_A0_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type;
			0x0009, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
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
			UP_THRESHOLD_CRIT, //uint8_t supported_thresholds;
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
			19800, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_VR_ASIC_P0V825_A0_CURR_A,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS1,
			.target_addr = VR_ASIC_P0V825_A0_MP2971_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V895 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V895 * 2 + 1],
		},
	},
	{
		{
			// VR_ASIC_P0V825_A0_PWR_W
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
			SENSOR_NUM_VR_ASIC_P0V825_A0_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type;
			0x000A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
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
			UP_THRESHOLD_CRIT, //uint8_t supported_thresholds;
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
			16335, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_VR_ASIC_P0V825_A0_PWR_W,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS1,
			.target_addr = VR_ASIC_P0V825_A0_MP2971_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V895 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V895 * 2 + 1],
		},
	},
	{
		{
			// VR_ASIC_P0V825_A1_TEMP_C
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
			SENSOR_NUM_VR_ASIC_P0V825_A1_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type;
			0x000B, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
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
			UP_THRESHOLD_CRIT, //uint8_t supported_thresholds;
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
			100000, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_VR_ASIC_P0V825_A1_TEMP_C,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS1,
			.target_addr = VR_ASIC_P0V825_A1_MP2971_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V825 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V825 * 2],
		},
	},
	{
		{
			// VR_ASIC_P0V825_A1_VOLT_V
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
			SENSOR_NUM_VR_ASIC_P0V825_A1_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type;
			0x000C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
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
			UP_THRESHOLD_CRIT | LOW_THRESHOLD_CRIT, //uint8_t supported_thresholds;
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
			842, //uint32_t critical_high;
			775, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_VR_ASIC_P0V825_A1_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS1,
			.target_addr = VR_ASIC_P0V825_A1_MP2971_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V825 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V825 * 2],
		},
	},
	{
		{
			// VR_ASIC_P0V825_A1_CURR_A
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
			SENSOR_NUM_VR_ASIC_P0V825_A1_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type;
			0x000D, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
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
			UP_THRESHOLD_CRIT, //uint8_t supported_thresholds;
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
			19800, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_VR_ASIC_P0V825_A1_CURR_A,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS1,
			.target_addr = VR_ASIC_P0V825_A1_MP2971_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V825 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V825 * 2],
		},
	},
	{
		{
			// VR_ASIC_P0V825_A1_PWR_W
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
			SENSOR_NUM_VR_ASIC_P0V825_A1_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type;
			0x000E, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
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
			UP_THRESHOLD_CRIT, //uint8_t supported_thresholds;
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
			16335, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_VR_ASIC_P0V825_A1_PWR_W,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS1,
			.target_addr = VR_ASIC_P0V825_A1_MP2971_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V825 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V825 * 2],
		},
	},
	{
		{
			// VR_ASIC_P0V825_A2_TEMP_C
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
			SENSOR_NUM_VR_ASIC_P0V825_A2_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type;
			0x000F, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
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
			UP_THRESHOLD_CRIT, //uint8_t supported_thresholds;
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
			100000, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_VR_ASIC_P0V825_A2_TEMP_C,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS1,
			.target_addr = VR_ASIC_P0V825_A2_MP2971_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V825 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V825 * 2 + 1],
		},
	},
	{
		{
			// VR_ASIC_P0V825_A2_VOLT_V
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
			SENSOR_NUM_VR_ASIC_P0V825_A2_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type;
			0x0010, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
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
			UP_THRESHOLD_CRIT | LOW_THRESHOLD_CRIT, //uint8_t supported_thresholds;
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
			842, //uint32_t critical_high;
			775, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_VR_ASIC_P0V825_A2_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS1,
			.target_addr = VR_ASIC_P0V825_A2_MP2971_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V825 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V825 * 2 + 1],
		},
	},
	{
		{
			// VR_ASIC_P0V825_A2_CURR_A
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
			SENSOR_NUM_VR_ASIC_P0V825_A2_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type;
			0x0011, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
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
			UP_THRESHOLD_CRIT, //uint8_t supported_thresholds;
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
			19800, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_VR_ASIC_P0V825_A2_CURR_A,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS1,
			.target_addr = VR_ASIC_P0V825_A2_MP2971_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V825 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V825 * 2 + 1],
		},
	},
	{
		{
			// VR_ASIC_P0V825_A2_PWR_W
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
			SENSOR_NUM_VR_ASIC_P0V825_A2_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type;
			0x0012, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init;
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
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
			UP_THRESHOLD_CRIT, //uint8_t supported_thresholds;
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
			16335, //uint32_t critical_high;
			0x00000000, //uint32_t critical_low;
			0x00000000, //uint32_t fatal_high;
			0x00000000, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_VR_ASIC_P0V825_A2_PWR_W,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS1,
			.target_addr = VR_ASIC_P0V825_A2_MP2971_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V825 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_P0V825 * 2 + 1],
		},
	},
};

pldm_sensor_info plat_pldm_sensor_adc_table[] = {
	{
		{
			// ADC_P12V_SCALED
			/*** PDR common header ***/
			{
				0x00000000, // uint32_t record_handle
				0x01, // uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, // uint8_t PDR_type
				0x0000, // uint16_t record_change_number
				0x0000, // uint16_t data_length
			},

			/*** numeric sensor format ***/
			0x0000, // uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ADC_P12V_SCALED_VOLT_V, // uint16_t sensor_id;
			0x0000, // uint16_t entity_type;
			0x0013, // uint16_t entity_instance_number;
			0x0000, // uint16_t container_id;
			0x00, // uint8_t sensor_init;
			0x01, // uint8_t sensor_auxiliary_names_pdr;
			0x02, // uint8_t base_unit;  // unit
			-3, // int8_t unit_modifier;
			0x00, // uint8_t rate_unit;
			0x00, // uint8_t base_oem_unit_handle;
			0x00, // uint8_t aux_unit;
			0x00, // int8_t aux_unit_modifier;
			0x00, // uint8_t auxrate_unit;
			0x00, // uint8_t rel;
			0x00, // uint8_t aux_oem_unit_handle;
			0x00, // uint8_t is_linear;
			0x4, // uint8_t sensor_data_size;
			1, // real32_t resolution;
			0, // real32_t offset;
			0x0000, // uint16_t accuracy;
			0x00, // uint8_t plus_tolerance;
			0x00, // uint8_t minus_tolerance;
			0x00000000, // uint32_t hysteresis;
			UP_THRESHOLD_CRIT | LOW_THRESHOLD_CRIT, // uint8_t supported_thresholds;
			0x00, // uint8_t threshold_and_hysteresis_volatility;
			0, // real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, // real32_t update_interval;
			0x00000000, // uint32_t max_readable;
			0x00000000, // uint32_t min_readable;
			0x04, // uint8_t range_field_format;
			0x00, // uint8_t range_field_support;
			0x00000000, // uint32_t nominal_value;
			0x00000000, // uint32_t normal_max;
			0x00000000, // uint32_t normal_min;
			0x00000000, // uint32_t warning_high;
			0x00000000, // uint32_t warning_low;
			12960, // uint32_t critical_high;
			11040, // uint32_t critical_low;
			0x00000000, // uint32_t fatal_high;
			0x00000000, // uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ADC_P12V_SCALED_VOLT_V,
			.type = sensor_dev_ads7830,
			.port = I2C_BUS1,
			.target_addr = ADS7830_I2C_ADDR,
			.offset = ADC_CH0,
			.access_checker = is_adc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.arg0 = 158, // R1 = 15.8kΩ
			.arg1 = 18, // R2 = 1.8kΩ
		},
	},
	{
		{
			// ADC_P5V_STBY_SCALED
			/*** PDR common header ***/
			{
				0x00000000, // uint32_t record_handle
				0x01, // uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, // uint8_t PDR_type
				0x0000, // uint16_t record_change_number
				0x0000, // uint16_t data_length
			},

			/*** numeric sensor format ***/
			0x0000, // uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ADC_P5V_STBY_SCALED_VOLT_V, // uint16_t sensor_id;
			0x0000, // uint16_t entity_type;
			0x0014, // uint16_t entity_instance_number;
			0x0000, // uint16_t container_id;
			0x00, // uint8_t sensor_init;
			0x01, // uint8_t sensor_auxiliary_names_pdr;
			0x02, // uint8_t base_unit;  // unit
			-3, // int8_t unit_modifier;
			0x00, // uint8_t rate_unit;
			0x00, // uint8_t base_oem_unit_handle;
			0x00, // uint8_t aux_unit;
			0x00, // int8_t aux_unit_modifier;
			0x00, // uint8_t auxrate_unit;
			0x00, // uint8_t rel;
			0x00, // uint8_t aux_oem_unit_handle;
			0x00, // uint8_t is_linear;
			0x4, // uint8_t sensor_data_size;
			1, // real32_t resolution;
			0, // real32_t offset;
			0x0000, // uint16_t accuracy;
			0x00, // uint8_t plus_tolerance;
			0x00, // uint8_t minus_tolerance;
			0x00000000, // uint32_t hysteresis;
			UP_THRESHOLD_CRIT | LOW_THRESHOLD_CRIT, // uint8_t supported_thresholds;
			0x00, // uint8_t threshold_and_hysteresis_volatility;
			0, // real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, // real32_t update_interval;
			0x00000000, // uint32_t max_readable;
			0x00000000, // uint32_t min_readable;
			0x04, // uint8_t range_field_format;
			0x00, // uint8_t range_field_support;
			0x00000000, // uint32_t nominal_value;
			0x00000000, // uint32_t normal_max;
			0x00000000, // uint32_t normal_min;
			0x00000000, // uint32_t warning_high;
			0x00000000, // uint32_t warning_low;
			5250, // uint32_t critical_high;
			4750, // uint32_t critical_low;
			0x00000000, // uint32_t fatal_high;
			0x00000000, // uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ADC_P5V_STBY_SCALED_VOLT_V,
			.type = sensor_dev_ads7830,
			.port = I2C_BUS1,
			.target_addr = ADS7830_I2C_ADDR,
			.offset = ADC_CH1,
			.access_checker = is_adc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.arg0 = 536,
			.arg1 = 180,
		},
	},
	{
		{
			// ADC_P3V3_AUX_SCALED
			/*** PDR common header ***/
			{
				0x00000000, // uint32_t record_handle
				0x01, // uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, // uint8_t PDR_type
				0x0000, // uint16_t record_change_number
				0x0000, // uint16_t data_length
			},

			/*** numeric sensor format ***/
			0x0000, // uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ADC_P3V3_AUX_SCALED_VOLT_V, // uint16_t sensor_id;
			0x0000, // uint16_t entity_type;
			0x0015, // uint16_t entity_instance_number;
			0x0000, // uint16_t container_id;
			0x00, // uint8_t sensor_init;
			0x01, // uint8_t sensor_auxiliary_names_pdr;
			0x02, // uint8_t base_unit;  // unit
			-3, // int8_t unit_modifier;
			0x00, // uint8_t rate_unit;
			0x00, // uint8_t base_oem_unit_handle;
			0x00, // uint8_t aux_unit;
			0x00, // int8_t aux_unit_modifier;
			0x00, // uint8_t auxrate_unit;
			0x00, // uint8_t rel;
			0x00, // uint8_t aux_oem_unit_handle;
			0x00, // uint8_t is_linear;
			0x4, // uint8_t sensor_data_size;
			1, // real32_t resolution;
			0, // real32_t offset;
			0x0000, // uint16_t accuracy;
			0x00, // uint8_t plus_tolerance;
			0x00, // uint8_t minus_tolerance;
			0x00000000, // uint32_t hysteresis;
			UP_THRESHOLD_CRIT | LOW_THRESHOLD_CRIT, // uint8_t supported_thresholds;
			0x00, // uint8_t threshold_and_hysteresis_volatility;
			0, // real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, // real32_t update_interval;
			0x00000000, // uint32_t max_readable;
			0x00000000, // uint32_t min_readable;
			0x04, // uint8_t range_field_format;
			0x00, // uint8_t range_field_support;
			0x00000000, // uint32_t nominal_value;
			0x00000000, // uint32_t normal_max;
			0x00000000, // uint32_t normal_min;
			0x00000000, // uint32_t warning_high;
			0x00000000, // uint32_t warning_low;
			3465, // uint32_t critical_high;
			3135, // uint32_t critical_low;
			0x00000000, // uint32_t fatal_high;
			0x00000000, // uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ADC_P3V3_AUX_SCALED_VOLT_V,
			.type = sensor_dev_ads7830,
			.port = I2C_BUS1,
			.target_addr = ADS7830_I2C_ADDR,
			.offset = ADC_CH3,
			.access_checker = is_adc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.arg0 = 287,
			.arg1 = 180,
		},
	},
	{
		{
			// ADC_P1V5_PEX_SCALED
			/*** PDR common header ***/
			{
				0x00000000, // uint32_t record_handle
				0x01, // uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, // uint8_t PDR_type
				0x0000, // uint16_t record_change_number
				0x0000, // uint16_t data_length
			},

			/*** numeric sensor format ***/
			0x0000, // uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ADC_P1V5_PEX_SCALED_VOLT_V, // uint16_t sensor_id;
			0x0000, // uint16_t entity_type;
			0x0016, // uint16_t entity_instance_number;
			0x0000, // uint16_t container_id;
			0x00, // uint8_t sensor_init;
			0x01, // uint8_t sensor_auxiliary_names_pdr;
			0x02, // uint8_t base_unit;  // unit
			-3, // int8_t unit_modifier;
			0x00, // uint8_t rate_unit;
			0x00, // uint8_t base_oem_unit_handle;
			0x00, // uint8_t aux_unit;
			0x00, // int8_t aux_unit_modifier;
			0x00, // uint8_t auxrate_unit;
			0x00, // uint8_t rel;
			0x00, // uint8_t aux_oem_unit_handle;
			0x00, // uint8_t is_linear;
			0x4, // uint8_t sensor_data_size;
			1, // real32_t resolution;
			0, // real32_t offset;
			0x0000, // uint16_t accuracy;
			0x00, // uint8_t plus_tolerance;
			0x00, // uint8_t minus_tolerance;
			0x00000000, // uint32_t hysteresis;
			UP_THRESHOLD_CRIT | LOW_THRESHOLD_CRIT, // uint8_t supported_thresholds;
			0x00, // uint8_t threshold_and_hysteresis_volatility;
			0, // real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, // real32_t update_interval;
			0x00000000, // uint32_t max_readable;
			0x00000000, // uint32_t min_readable;
			0x04, // uint8_t range_field_format;
			0x00, // uint8_t range_field_support;
			0x00000000, // uint32_t nominal_value;
			0x00000000, // uint32_t normal_max;
			0x00000000, // uint32_t normal_min;
			0x00000000, // uint32_t warning_high;
			0x00000000, // uint32_t warning_low;
			1575, // uint32_t critical_high;
			1425, // uint32_t critical_low;
			0x00000000, // uint32_t fatal_high;
			0x00000000, // uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ADC_P1V5_PEX_SCALED_VOLT_V,
			.type = sensor_dev_ads7830,
			.port = I2C_BUS1,
			.target_addr = ADS7830_I2C_ADDR,
			.offset = ADC_CH4,
			.access_checker = is_adc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.arg0 = 0,
			.arg1 = 1,
		},
	},
	{
		{
			// ADC_P1V2_PEX_SCALED
			/*** PDR common header ***/
			{
				0x00000000, // uint32_t record_handle
				0x01, // uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, // uint8_t PDR_type
				0x0000, // uint16_t record_change_number
				0x0000, // uint16_t data_length
			},

			/*** numeric sensor format ***/
			0x0000, // uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ADC_P1V2_PEX_SCALED_VOLT_V, // uint16_t sensor_id;
			0x0000, // uint16_t entity_type;
			0x0017, // uint16_t entity_instance_number;
			0x0000, // uint16_t container_id;
			0x00, // uint8_t sensor_init;
			0x01, // uint8_t sensor_auxiliary_names_pdr;
			0x02, // uint8_t base_unit;  // unit
			-3, // int8_t unit_modifier;
			0x00, // uint8_t rate_unit;
			0x00, // uint8_t base_oem_unit_handle;
			0x00, // uint8_t aux_unit;
			0x00, // int8_t aux_unit_modifier;
			0x00, // uint8_t auxrate_unit;
			0x00, // uint8_t rel;
			0x00, // uint8_t aux_oem_unit_handle;
			0x00, // uint8_t is_linear;
			0x4, // uint8_t sensor_data_size;
			1, // real32_t resolution;
			0, // real32_t offset;
			0x0000, // uint16_t accuracy;
			0x00, // uint8_t plus_tolerance;
			0x00, // uint8_t minus_tolerance;
			0x00000000, // uint32_t hysteresis;
			UP_THRESHOLD_CRIT | LOW_THRESHOLD_CRIT, // uint8_t supported_thresholds;
			0x00, // uint8_t threshold_and_hysteresis_volatility;
			0, // real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, // real32_t update_interval;
			0x00000000, // uint32_t max_readable;
			0x00000000, // uint32_t min_readable;
			0x04, // uint8_t range_field_format;
			0x00, // uint8_t range_field_support;
			0x00000000, // uint32_t nominal_value;
			0x00000000, // uint32_t normal_max;
			0x00000000, // uint32_t normal_min;
			0x00000000, // uint32_t warning_high;
			0x00000000, // uint32_t warning_low;
			1236, // uint32_t critical_high;
			1164, // uint32_t critical_low;
			0x00000000, // uint32_t fatal_high;
			0x00000000, // uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ADC_P1V2_PEX_SCALED_VOLT_V,
			.type = sensor_dev_ads7830,
			.port = I2C_BUS1,
			.target_addr = ADS7830_I2C_ADDR,
			.offset = ADC_CH5,
			.access_checker = is_adc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.arg0 = 0,
			.arg1 = 1,
		},
	},
	{
		{
			// ADC_P1V8_PEX_SCALED
			/*** PDR common header ***/
			{
				0x00000000, // uint32_t record_handle
				0x01, // uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, // uint8_t PDR_type
				0x0000, // uint16_t record_change_number
				0x0000, // uint16_t data_length
			},

			/*** numeric sensor format ***/
			0x0000, // uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ADC_P1V8_PEX_SCALED_VOLT_V, // uint16_t sensor_id;
			0x0000, // uint16_t entity_type;
			0x0018, // uint16_t entity_instance_number;
			0x0000, // uint16_t container_id;
			0x00, // uint8_t sensor_init;
			0x01, // uint8_t sensor_auxiliary_names_pdr;
			0x02, // uint8_t base_unit;  // unit
			-3, // int8_t unit_modifier;
			0x00, // uint8_t rate_unit;
			0x00, // uint8_t base_oem_unit_handle;
			0x00, // uint8_t aux_unit;
			0x00, // int8_t aux_unit_modifier;
			0x00, // uint8_t auxrate_unit;
			0x00, // uint8_t rel;
			0x00, // uint8_t aux_oem_unit_handle;
			0x00, // uint8_t is_linear;
			0x4, // uint8_t sensor_data_size;
			1, // real32_t resolution;
			0, // real32_t offset;
			0x0000, // uint16_t accuracy;
			0x00, // uint8_t plus_tolerance;
			0x00, // uint8_t minus_tolerance;
			0x00000000, // uint32_t hysteresis;
			UP_THRESHOLD_CRIT | LOW_THRESHOLD_CRIT, // uint8_t supported_thresholds;
			0x00, // uint8_t threshold_and_hysteresis_volatility;
			0, // real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, // real32_t update_interval;
			0x00000000, // uint32_t max_readable;
			0x00000000, // uint32_t min_readable;
			0x04, // uint8_t range_field_format;
			0x00, // uint8_t range_field_support;
			0x00000000, // uint32_t nominal_value;
			0x00000000, // uint32_t normal_max;
			0x00000000, // uint32_t normal_min;
			0x00000000, // uint32_t warning_high;
			0x00000000, // uint32_t warning_low;
			1890, // uint32_t critical_high;
			1710, // uint32_t critical_low;
			0x00000000, // uint32_t fatal_high;
			0x00000000, // uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ADC_P1V8_PEX_SCALED_VOLT_V,
			.type = sensor_dev_ads7830,
			.port = I2C_BUS1,
			.target_addr = ADS7830_I2C_ADDR,
			.offset = ADC_CH6,
			.access_checker = is_adc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.arg0 = 0,
			.arg1 = 1,
		},
	},
};

PDR_sensor_auxiliary_names plat_pdr_sensor_aux_names_table[] = {
	{

		// SENSOR_NUM_THERMAL_SENSOR_1_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_THERMAL_SENSOR_1_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"THERMAL_SENSOR_1_TEMP_C",
	},
	{

		// SENSOR_NUM_THERMAL_SENSOR_2_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_THERMAL_SENSOR_2_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"THERMAL_SENSOR_2_TEMP_C",
	},
	{

		// SENSOR_NUM_PCIE_SWITCH_PEX90144_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_PCIE_SWITCH_PEX90144_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"PCIE_SWITCH_TEMP_C",
	},
	{

		// VR_ASIC_P0V895_PEX_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_VR_ASIC_P0V895_PEX_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"VR_ASIC_P0V895_PEX_TEMP_C",
	},
	{

		// VR_ASIC_P0V895_PEX_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_VR_ASIC_P0V895_PEX_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"VR_ASIC_P0V895_PEX_VOLT_V",
	},
	{

		// VR_ASIC_P0V895_PEX_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_VR_ASIC_P0V895_PEX_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"VR_ASIC_P0V895_PEX_CURR_A",
	},
	{

		// VR_ASIC_P0V895_PEX_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_VR_ASIC_P0V895_PEX_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"VR_ASIC_P0V895_PEX_PWR_W",
	},
	{

		// VR_ASIC_P0V825_A0_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_VR_ASIC_P0V825_A0_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"VR_ASIC_P0V825_A0_TEMP_C",
	},
	{

		// VR_ASIC_P0V825_A0_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_VR_ASIC_P0V825_A0_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"VR_ASIC_P0V825_A0_VOLT_V",
	},
	{

		// VR_ASIC_P0V825_A0_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_VR_ASIC_P0V825_A0_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"VR_ASIC_P0V825_A0_CURR_A",
	},
	{

		// VR_ASIC_P0V825_A0_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_VR_ASIC_P0V825_A0_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"VR_ASIC_P0V825_A0_PWR_W",
	},
	{

		// VR_ASIC_P0V825_A1_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_VR_ASIC_P0V825_A1_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"VR_ASIC_P0V825_A1_TEMP_C",
	},
	{

		// VR_ASIC_P0V825_A1_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_VR_ASIC_P0V825_A1_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"VR_ASIC_P0V825_A1_VOLT_V",
	},
	{

		// VR_ASIC_P0V825_A1_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_VR_ASIC_P0V825_A1_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"VR_ASIC_P0V825_A1_CURR_A",
	},
	{

		// VR_ASIC_P0V825_A1_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_VR_ASIC_P0V825_A1_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"VR_ASIC_P0V825_A1_PWR_W",
	},
	{

		// VR_ASIC_P0V825_A2_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_VR_ASIC_P0V825_A2_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"VR_ASIC_P0V825_A2_TEMP_C",
	},
	{

		// VR_ASIC_P0V825_A2_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_VR_ASIC_P0V825_A2_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"VR_ASIC_P0V825_A2_VOLT_V",
	},
	{

		// VR_ASIC_P0V825_A2_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_VR_ASIC_P0V825_A2_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"VR_ASIC_P0V825_A2_CURR_A",
	},
	{

		// VR_ASIC_P0V825_A2_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_VR_ASIC_P0V825_A2_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"VR_ASIC_P0V825_A2_PWR_W",
	},
	{

		// ADC_P12V_SCALED
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ADC_P12V_SCALED_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ADC_P12V_SCALED_VOLT_V",
	},
	{

		// ADC_P5V_STBY_SCALED
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ADC_P5V_STBY_SCALED_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ADC_P5V_STBY_SCALED_VOLT_V",
	},
	{

		// ADC_P3V3_AUX_SCALED
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ADC_P3V3_AUX_SCALED_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ADC_P3V3_AUX_SCALED_VOLT_V",
	},
	{

		// ADC_P1V5_PEX_SCALED
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ADC_P1V5_PEX_SCALED_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ADC_P1V5_PEX_SCALED_VOLT_V",
	},
	{

		// ADC_P1V2_PEX_SCALED
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ADC_P1V2_PEX_SCALED_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ADC_P1V2_PEX_SCALED_VOLT_V",
	},
	{

		// ADC_P1V8_PEX_SCALED
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ADC_P1V8_PEX_SCALED_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ADC_P1V8_PEX_SCALED_VOLT_V",
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
	case VR_SENSOR_THREAD_ID:

		return plat_pldm_sensor_vr_table;
	case TEMP_SENSOR_THREAD_ID:

		return plat_pldm_sensor_temp_table;
	case ADC_SENSOR_THREAD_ID:

		return plat_pldm_sensor_adc_table;
	default:
		LOG_ERR("Unknow pldm sensor thread id %d", thread_id);
		return NULL;
	}
}

int plat_pldm_sensor_get_sensor_count(int thread_id)
{
	int count = 0;

	switch (thread_id) {
	case VR_SENSOR_THREAD_ID:
		count = ARRAY_SIZE(plat_pldm_sensor_vr_table);
		break;
	case TEMP_SENSOR_THREAD_ID:
		count = ARRAY_SIZE(plat_pldm_sensor_temp_table);
		break;
	case ADC_SENSOR_THREAD_ID:
		count = ARRAY_SIZE(plat_pldm_sensor_adc_table);
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
	case VR_SENSOR_THREAD_ID:
		memcpy(numeric_sensor_table,
		       &plat_pldm_sensor_vr_table[sensor_num].pdr_numeric_sensor,
		       sizeof(PDR_numeric_sensor));
		break;
	case TEMP_SENSOR_THREAD_ID:
		memcpy(numeric_sensor_table,
		       &plat_pldm_sensor_temp_table[sensor_num].pdr_numeric_sensor,
		       sizeof(PDR_numeric_sensor));
		break;
	case ADC_SENSOR_THREAD_ID:
		memcpy(numeric_sensor_table,
		       &plat_pldm_sensor_adc_table[sensor_num].pdr_numeric_sensor,
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
	const char16_t base_name[] = u"SI_SLOT_";

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

void find_vr_addr_by_sensor_id(uint8_t sensor_id, uint8_t *vr_addr)
{
	for (int index = 0; index < ARRAY_SIZE(plat_sensor_vr_extend_table); index++) {
		if (plat_sensor_vr_extend_table[index].sensor_id == sensor_id) {
			*vr_addr = plat_sensor_vr_extend_table[index].target_rns_addr;
			return;
		}
	}
}

void find_tmp_addr_and_offset_by_sensor_id(uint8_t sensor_id, uint8_t *tmp_addr,
					   uint16_t *tmp_offset)
{
	for (int index = 0; index < ARRAY_SIZE(plat_sensor_tmp_extend_table); index++) {
		if (plat_sensor_tmp_extend_table[index].sensor_id == sensor_id) {
			*tmp_addr = plat_sensor_tmp_extend_table[index].target_emc1413_addr;
			*tmp_offset = plat_sensor_tmp_extend_table[index].offset;
			return;
		}
	}
}

void set_plat_sensor_polling_enable_flag(bool value)
{
	plat_sensor_polling_enable_flag = value;
}

void set_plat_sensor_temp_polling_enable_flag(bool value)
{
	plat_sensor_temp_polling_enable_flag = value;
}

void set_plat_sensor_vr_polling_enable_flag(bool value)
{
	plat_sensor_vr_polling_enable_flag = value;
}

void set_plat_sensor_adc_polling_enable_flag(bool value)
{
	plat_sensor_adc_polling_enable_flag = value;
}

bool get_plat_sensor_polling_enable_flag()
{
	return plat_sensor_polling_enable_flag;
}

bool get_plat_sensor_adc_polling_enable_flag()
{
	return plat_sensor_adc_polling_enable_flag;
}

bool get_plat_sensor_temp_polling_enable_flag()
{
	return plat_sensor_temp_polling_enable_flag;
}

bool get_plat_sensor_vr_polling_enable_flag()
{
	return plat_sensor_vr_polling_enable_flag;
}

bool is_adc_access(uint8_t sensor_num)
{
	if (!get_plat_sensor_adc_polling_enable_flag() || !get_plat_sensor_polling_enable_flag()) {
		LOG_DBG("Polling disabled: ADC polling enable flag=%d, general polling enable flag=%d",
			get_plat_sensor_adc_polling_enable_flag(),
			get_plat_sensor_polling_enable_flag());
		return false;
	}

	I2C_MSG msg = { 0 };
	uint8_t retry = 3;

	// Step 1: Write command byte to select CH0
	msg.bus = I2C_BUS1;
	msg.target_addr = ADS7830_I2C_ADDR;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] =
		0x8C; // Command byte for CH0 (single-ended, with power-down between conversions)

	// Step 2: Read ADC data (1 byte)
	if (i2c_master_read(&msg, retry) != 0) {
		LOG_ERR("ADS7830 read failed at addr 0x%x, bus %d (sensor_num=0x%x)",
			ADS7830_I2C_ADDR, I2C_BUS1, sensor_num);
		return false;
	}

	// You can log the value if needed:
	LOG_DBG("ADS7830 read CH0 success: value = %d (sensor_num=0x%x)", msg.data[0], sensor_num);

	return true;
}

bool is_temp_access(uint8_t cfg_idx)
{
	return (get_plat_sensor_temp_polling_enable_flag() &&
		get_plat_sensor_polling_enable_flag());
}

bool is_pcie_switch_access(uint8_t cfg_idx)
{
	return (get_plat_sensor_temp_polling_enable_flag() &&
		get_plat_sensor_polling_enable_flag());
}

bool is_vr_access(uint8_t sensor_num)
{
	return (get_plat_sensor_vr_polling_enable_flag() && get_plat_sensor_polling_enable_flag());
}

bool get_sensor_info_by_sensor_id(uint8_t sensor_id, uint8_t *vr_bus, uint8_t *vr_addr,
				  uint8_t *sensor_dev)
{
	CHECK_NULL_ARG_WITH_RETURN(vr_bus, false);
	CHECK_NULL_ARG_WITH_RETURN(vr_addr, false);
	CHECK_NULL_ARG_WITH_RETURN(sensor_dev, false);

	int pldm_sensor_count = 0;

	if (sensor_id >= SENSOR_NUM_THERMAL_SENSOR_1_TEMP_C &&
	    sensor_id <= SENSOR_NUM_PCIE_SWITCH_PEX90144_TEMP_C) {
		pldm_sensor_count = plat_pldm_sensor_get_sensor_count(TEMP_SENSOR_THREAD_ID);
		for (int index = 0; index < pldm_sensor_count; index++) {
			if (plat_pldm_sensor_temp_table[index].pldm_sensor_cfg.num == sensor_id) {
				*vr_addr = plat_pldm_sensor_temp_table[index]
						   .pldm_sensor_cfg.target_addr;
				*vr_bus = plat_pldm_sensor_temp_table[index].pldm_sensor_cfg.port;
				*sensor_dev =
					plat_pldm_sensor_temp_table[index].pldm_sensor_cfg.type;
				return true;
			}
		}
	} else if (sensor_id >= SENSOR_NUM_VR_ASIC_P0V895_PEX_TEMP_C &&
		   sensor_id <= SENSOR_NUM_VR_ASIC_P0V825_A2_PWR_W) {
		pldm_sensor_count = plat_pldm_sensor_get_sensor_count(VR_SENSOR_THREAD_ID);
		for (int index = 0; index < pldm_sensor_count; index++) {
			if (plat_pldm_sensor_vr_table[index].pldm_sensor_cfg.num == sensor_id) {
				*vr_addr =
					plat_pldm_sensor_vr_table[index].pldm_sensor_cfg.target_addr;
				*vr_bus = plat_pldm_sensor_vr_table[index].pldm_sensor_cfg.port;
				*sensor_dev = plat_pldm_sensor_vr_table[index].pldm_sensor_cfg.type;
				return true;
			}
		}
	} else if (sensor_id >= SENSOR_NUM_ADC_P12V_SCALED_VOLT_V &&
		   sensor_id <= SENSOR_NUM_ADC_P1V8_PEX_SCALED_VOLT_V) {
		pldm_sensor_count = plat_pldm_sensor_get_sensor_count(ADC_SENSOR_THREAD_ID);
		for (int index = 0; index < pldm_sensor_count; index++) {
			if (plat_pldm_sensor_adc_table[index].pldm_sensor_cfg.num == sensor_id) {
				*vr_addr = plat_pldm_sensor_adc_table[index]
						   .pldm_sensor_cfg.target_addr;
				*vr_bus = plat_pldm_sensor_adc_table[index].pldm_sensor_cfg.port;
				*sensor_dev =
					plat_pldm_sensor_adc_table[index].pldm_sensor_cfg.type;
				return true;
			}
		}
	}

	return false;
}

sensor_cfg *get_sensor_cfg_by_sensor_id(uint8_t sensor_id)
{
	int pldm_sensor_count = 0;

	if (sensor_id >= SENSOR_NUM_THERMAL_SENSOR_1_TEMP_C &&
	    sensor_id <= SENSOR_NUM_PCIE_SWITCH_PEX90144_TEMP_C) {
		pldm_sensor_count = plat_pldm_sensor_get_sensor_count(TEMP_SENSOR_THREAD_ID);
		for (int index = 0; index < pldm_sensor_count; index++) {
			if (plat_pldm_sensor_temp_table[index].pldm_sensor_cfg.num == sensor_id) {
				return &plat_pldm_sensor_temp_table[index].pldm_sensor_cfg;
			}
		}
	} else if (sensor_id >= SENSOR_NUM_VR_ASIC_P0V895_PEX_TEMP_C &&
		   sensor_id <= SENSOR_NUM_VR_ASIC_P0V825_A2_PWR_W) {
		pldm_sensor_count = plat_pldm_sensor_get_sensor_count(VR_SENSOR_THREAD_ID);
		for (int index = 0; index < pldm_sensor_count; index++) {
			if (plat_pldm_sensor_vr_table[index].pldm_sensor_cfg.num == sensor_id) {
				return &plat_pldm_sensor_vr_table[index].pldm_sensor_cfg;
			}
		}
	} else if (sensor_id >= SENSOR_NUM_ADC_P12V_SCALED_VOLT_V &&
		   sensor_id <= SENSOR_NUM_ADC_P1V8_PEX_SCALED_VOLT_V) {
		pldm_sensor_count = plat_pldm_sensor_get_sensor_count(ADC_SENSOR_THREAD_ID);
		for (int index = 0; index < pldm_sensor_count; index++) {
			if (plat_pldm_sensor_adc_table[index].pldm_sensor_cfg.num == sensor_id) {
				return &plat_pldm_sensor_adc_table[index].pldm_sensor_cfg;
			}
		}
	}

	return NULL;
}
