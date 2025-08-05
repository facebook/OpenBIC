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
#include "sensor.h"
#include "tmp431.h"
#include "pldm_sensor.h"
#include "plat_hook.h"
#include "plat_i2c.h"
#include "plat_util.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_pldm_sensor);

static bool plat_sensor_polling_enable_flag = true;
static bool plat_sensor_ubc_polling_enable_flag = true;
static bool plat_sensor_temp_polling_enable_flag = true;
static bool plat_sensor_vr_polling_enable_flag = true;

static struct pldm_sensor_thread pal_pldm_sensor_thread[MAX_SENSOR_THREAD_ID] = {
	// thread id, thread name
	{ TEMP_SENSOR_THREAD_ID, "TEMP_SENSOR_THREAD" },
	{ VR_SENSOR_THREAD_ID, "VR_PLDM_SENSOR_THREAD" },
	{ QUICK_VR_SENSOR_THREAD_ID, "QUICK_VR_PLDM_SENSOR_THREAD", 10 },
	{ UBC_SENSOR_THREAD_ID, "UBC_PLDM_SENSOR_THREAD" },
};

extern vr_pre_proc_arg vr_pre_read_args[];

static bool is_quick_vr_sensor(uint8_t sensor_num)
{
	switch (sensor_num) {
	case SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_VOLT_V:
	case SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_CURR_A:
	case SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_PWR_W:
	case SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_VOLT_V:
	case SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_CURR_A:
	case SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_PWR_W:
	case SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_VOLT_V:
	case SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_CURR_A:
	case SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_PWR_W:
		return true;
	}
	return false;
}

typedef struct {
	uint8_t orig_addr;
	uint8_t rns_addr;
} addr_map_t;

static const addr_map_t addr_map_table[] = {
	{ ASIC_P0V85_MEDHA0_VDD_ADDR, ASIC_P0V85_MEDHA0_VDD_RNS_ADDR },
	{ ASIC_P0V85_MEDHA1_VDD_ADDR, ASIC_P0V85_MEDHA1_VDD_RNS_ADDR },
	{ ASIC_P0V9_OWL_E_TRVDD_ADDR, ASIC_P0V9_OWL_E_TRVDD_RNS_ADDR },
	{ ASIC_P0V75_OWL_E_TRVDD_ADDR, ASIC_P0V75_OWL_E_TRVDD_RNS_ADDR },
	{ ASIC_P0V75_MAX_M_VDD_ADDR, ASIC_P0V75_MAX_M_VDD_RNS_ADDR },
	{ ASIC_P0V75_VDDPHY_HBM1357_ADDR, ASIC_P0V75_VDDPHY_HBM1357_RNS_ADDR },
	{ ASIC_P0V75_OWL_E_VDD_ADDR, ASIC_P0V75_OWL_E_VDD_RNS_ADDR },
	{ ASIC_P0V4_VDDQL_HBM1357_ADDR, ASIC_P0V4_VDDQL_HBM1357_RNS_ADDR },
	{ ASIC_P1V1_VDDQC_HBM1357_ADDR, ASIC_P1V1_VDDQC_HBM1357_RNS_ADDR },
	{ ASIC_P1V8_VPP_HBM1357_ADDR, ASIC_P1V8_VPP_HBM1357_RNS_ADDR },
	{ ASIC_P0V75_MAX_N_VDD_ADDR, ASIC_P0V75_MAX_N_VDD_RNS_ADDR },
	{ ASIC_P0V8_HAMSA_AVDD_PCIE_ADDR, ASIC_P0V8_HAMSA_AVDD_PCIE_RNS_ADDR },
	{ ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_ADDR, ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_RNS_ADDR },
	{ ASIC_P0V85_HAMSA_VDD_ADDR, ASIC_P0V85_HAMSA_VDD_RNS_ADDR },
	{ ASIC_P1V1_VDDQC_HBM0246_ADDR, ASIC_P1V1_VDDQC_HBM0246_RNS_ADDR },
	{ ASIC_P1V8_VPP_HBM0246_ADDR, ASIC_P1V8_VPP_HBM0246_RNS_ADDR },
	{ ASIC_P0V4_VDDQL_HBM0246_ADDR, ASIC_P0V4_VDDQL_HBM0246_RNS_ADDR },
	{ ASIC_P0V75_VDDPHY_HBM0246_ADDR, ASIC_P0V75_VDDPHY_HBM0246_RNS_ADDR },
	{ ASIC_P0V75_OWL_W_VDD_ADDR, ASIC_P0V75_OWL_W_VDD_RNS_ADDR },
	{ ASIC_P0V75_MAX_S_VDD_ADDR, ASIC_P0V75_MAX_S_VDD_RNS_ADDR },
	{ ASIC_P0V9_OWL_W_TRVDD_ADDR, ASIC_P0V9_OWL_W_TRVDD_RNS_ADDR },
	{ ASIC_P0V75_OWL_W_TRVDD_ADDR, ASIC_P0V75_OWL_W_TRVDD_RNS_ADDR },
};

uint8_t convert_addr_to_rns(uint8_t addr)
{
	for (int i = 0; i < ARRAY_SIZE(addr_map_table); i++) {
		if (addr_map_table[i].orig_addr == addr) {
			return addr_map_table[i].rns_addr;
		}
	}
	return addr;
}

uint8_t check_sensor_type(uint8_t sensor_num)
{
	if (sensor_num == 0 || sensor_num >= SENSOR_NUM_NUMBERS)
		return MAX_SENSOR_THREAD_ID;

	if (sensor_num <= SENSOR_NUM_ASIC_HAMSA_LS_TEMP_C)
		return TEMP_SENSOR_THREAD_ID;

	if (sensor_num <= SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_PWR_W)
		return is_quick_vr_sensor(sensor_num) ? QUICK_VR_SENSOR_THREAD_ID :
							VR_SENSOR_THREAD_ID;

	if (sensor_num <= SENSOR_NUM_UBC2_P52V_INPUT_VOLT_V)
		return UBC_SENSOR_THREAD_ID;

	return MAX_SENSOR_THREAD_ID;
}

pldm_sensor_info plat_pldm_sensor_temp_table[] = {
	{
		{
			// TOP_INLET
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
			SENSOR_NUM_TOP_INLET_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_TOP_INLET_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			60000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_TOP_INLET_TEMP_C,
			.type = sensor_dev_tmp75,
			.port = I2C_BUS1,
			.target_addr = TOP_INLET_ADDR,
			.offset = TMP75_TEMP_OFFSET,
			.access_checker = is_temp_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// BOT_INLET
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
			SENSOR_NUM_BOT_INLET_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_BOT_INLET_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			60000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_BOT_INLET_TEMP_C,
			.type = sensor_dev_tmp75,
			.port = I2C_BUS1,
			.target_addr = BOT_INLET_ADDR,
			.offset = TMP75_TEMP_OFFSET,
			.access_checker = is_temp_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// BOT_OUTLET
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
			SENSOR_NUM_BOT_OUTLET_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_BOT_OUTLET_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			80000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_BOT_OUTLET_TEMP_C,
			.type = sensor_dev_tmp75,
			.port = I2C_BUS1,
			.target_addr = BOT_OUTLET_ADDR,
			.offset = TMP75_TEMP_OFFSET,
			.access_checker = is_temp_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// ASIC_MEDHA0_SENSOR0
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
			SENSOR_NUM_ASIC_MEDHA0_SENSOR0_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_MEDHA0_SENSOR0_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			95000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_MEDHA0_SENSOR0_TEMP_C,
			.type = sensor_dev_tmp431,
			.port = I2C_BUS3,
			.target_addr = ASIC_MEDHA0_SENSOR0_ADDR,
			.offset = TMP432_REMOTE_TEMPERATRUE_1,
			.access_checker = is_temp_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// ASIC_MEDHA0_SENSOR1
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
			SENSOR_NUM_ASIC_MEDHA0_SENSOR1_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_MEDHA0_SENSOR1_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			95000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_MEDHA0_SENSOR1_TEMP_C,
			.type = sensor_dev_tmp431,
			.port = I2C_BUS3,
			.target_addr = ASIC_MEDHA0_SENSOR1_ADDR,
			.offset = TMP432_REMOTE_TEMPERATRUE_2,
			.access_checker = is_temp_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// ASIC_OWL_W
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
			SENSOR_NUM_ASIC_OWL_W_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_OWL_W_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			95000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_OWL_W_TEMP_C,
			.type = sensor_dev_tmp431,
			.port = I2C_BUS3,
			.target_addr = ASIC_OWL_W_ADDR,
			.offset = TMP432_REMOTE_TEMPERATRUE_1,
			.access_checker = is_temp_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// ASIC_OWL_E
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
			SENSOR_NUM_ASIC_OWL_E_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_OWL_E_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			95000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_OWL_E_TEMP_C,
			.type = sensor_dev_tmp431,
			.port = I2C_BUS3,
			.target_addr = ASIC_OWL_E_ADDR,
			.offset = TMP432_REMOTE_TEMPERATRUE_2,
			.access_checker = is_temp_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// ASIC_MEDHA1_SENSOR0
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
			SENSOR_NUM_ASIC_MEDHA1_SENSOR0_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_MEDHA1_SENSOR0_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			95000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_MEDHA1_SENSOR0_TEMP_C,
			.type = sensor_dev_tmp431,
			.port = I2C_BUS2,
			.target_addr = ASIC_MEDHA1_SENSOR0_ADDR,
			.offset = TMP432_REMOTE_TEMPERATRUE_1,
			.access_checker = is_temp_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// ASIC_MEDHA1_SENSOR1
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
			SENSOR_NUM_ASIC_MEDHA1_SENSOR1_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_MEDHA1_SENSOR1_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			95000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_MEDHA1_SENSOR1_TEMP_C,
			.type = sensor_dev_tmp431,
			.port = I2C_BUS2,
			.target_addr = ASIC_MEDHA1_SENSOR1_ADDR,
			.offset = TMP432_REMOTE_TEMPERATRUE_2,
			.access_checker = is_temp_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// ASIC_HAMSA_CRM
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
			SENSOR_NUM_ASIC_HAMSA_CRM_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_HAMSA_CRM_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			95000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_HAMSA_CRM_TEMP_C,
			.type = sensor_dev_tmp431,
			.port = I2C_BUS2,
			.target_addr = ASIC_HAMSA_CRM_ADDR,
			.offset = TMP432_REMOTE_TEMPERATRUE_1,
			.access_checker = is_temp_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// ASIC_HAMSA_LS
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
			SENSOR_NUM_ASIC_HAMSA_LS_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_HAMSA_LS_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			95000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_HAMSA_LS_TEMP_C,
			.type = sensor_dev_tmp431,
			.port = I2C_BUS2,
			.target_addr = ASIC_HAMSA_LS_ADDR,
			.offset = TMP432_REMOTE_TEMPERATRUE_2,
			.access_checker = is_temp_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
};

pldm_sensor_info plat_pldm_sensor_vr_table[] = {
	{
		{
			// ASIC_P0V85_MEDHA0_VDD_TEMP_C
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
			SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			100000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_TEMP_C,
			.type = sensor_dev_mp29816a,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V85_MEDHA0_VDD_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_2 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_2 * 2],
		},
	},
	{
		{
			// ASIC_P0V85_MEDHA1_VDD_TEMP_C
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
			SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			100000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_TEMP_C,
			.type = sensor_dev_mp29816a,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V85_MEDHA1_VDD_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_1 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_1 * 2],
		},
	},
	{
		{
			// ASIC_P0V9_OWL_E_TRVDD_TEMP_C
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
			SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			100000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_TEMP_C,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V9_OWL_E_TRVDD_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_3 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_3 * 2],
		},
	},
	{
		{
			// ASIC_P0V9_OWL_E_TRVDD_VOLT_V
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
			SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_VOLT_V, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			3571, //uint32_t critical_high;
			3041, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V9_OWL_E_TRVDD_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_3 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_3 * 2],
		},
	},
	{
		{
			// ASIC_P0V9_OWL_E_TRVDD_CURR_A
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
			SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_CURR_A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_CURR_A,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V9_OWL_E_TRVDD_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_3 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_3 * 2],
		},
	},
	{
		{
			// ASIC_P0V9_OWL_E_TRVDD_PWR_W
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
			SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_PWR_W, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_PWR_W,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V9_OWL_E_TRVDD_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_3 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_3 * 2],
		},
	},
	{
		{
			// ASIC_P0V75_OWL_E_TRVDD_TEMP_C
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
			SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			100000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_TEMP_C,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V75_OWL_E_TRVDD_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_3 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_3 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V75_OWL_E_TRVDD_VOLT_V
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
			SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_VOLT_V, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			3571, //uint32_t critical_high;
			3041, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V75_OWL_E_TRVDD_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_3 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_3 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V75_OWL_E_TRVDD_CURR_A
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
			SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_CURR_A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_CURR_A,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V75_OWL_E_TRVDD_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_3 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_3 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V75_OWL_E_TRVDD_PWR_W
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
			SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_PWR_W, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_PWR_W,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V75_OWL_E_TRVDD_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_3 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_3 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V75_OWL_E_VDD_TEMP_C
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
			SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			100000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_TEMP_C,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V75_OWL_E_VDD_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_5 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_5 * 2],
		},
	},
	{
		{
			// ASIC_P0V75_OWL_E_VDD_VOLT_V
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
			SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_VOLT_V, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			3571, //uint32_t critical_high;
			3041, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V75_OWL_E_VDD_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_5 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_5 * 2],
		},
	},
	{
		{
			// ASIC_P0V75_OWL_E_VDD_CURR_A
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
			SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_CURR_A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_CURR_A,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V75_OWL_E_VDD_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_5 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_5 * 2],
		},
	},
	{
		{
			// ASIC_P0V75_OWL_E_VDD_PWR_W
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
			SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_PWR_W, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_PWR_W,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V75_OWL_E_VDD_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_5 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_5 * 2],
		},
	},
	{
		{
			// ASIC_P0V9_OWL_W_TRVDD_TEMP_C
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
			SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			100000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_TEMP_C,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V9_OWL_W_TRVDD_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2],
		},
	},
	{
		{
			// ASIC_P0V9_OWL_W_TRVDD_VOLT_V
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
			SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_VOLT_V, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			3571, //uint32_t critical_high;
			3041, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V9_OWL_W_TRVDD_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2],
		},
	},
	{
		{
			// ASIC_P0V9_OWL_W_TRVDD_CURR_A
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
			SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_CURR_A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_CURR_A,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V9_OWL_W_TRVDD_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2],
		},
	},
	{
		{
			// ASIC_P0V9_OWL_W_TRVDD_PWR_W
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
			SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_PWR_W, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_PWR_W,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V9_OWL_W_TRVDD_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2],
		},
	},
	{
		{
			// ASIC_P0V75_OWL_W_TRVDD_TEMP_C
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
			SENSOR_NUM_ASIC_P0V75_OWL_W_TRVDD_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_OWL_W_TRVDD_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			100000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_OWL_W_TRVDD_TEMP_C,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V75_OWL_W_TRVDD_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V75_OWL_W_TRVDD_VOLT_V
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
			SENSOR_NUM_ASIC_P0V75_OWL_W_TRVDD_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_OWL_W_TRVDD_VOLT_V, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			3571, //uint32_t critical_high;
			3041, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_OWL_W_TRVDD_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V75_OWL_W_TRVDD_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V75_OWL_W_TRVDD_CURR_A
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
			SENSOR_NUM_ASIC_P0V75_OWL_W_TRVDD_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_OWL_W_TRVDD_CURR_A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_OWL_W_TRVDD_CURR_A,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V75_OWL_W_TRVDD_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V75_OWL_W_TRVDD_PWR_W
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
			SENSOR_NUM_ASIC_P0V75_OWL_W_TRVDD_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_OWL_W_TRVDD_PWR_W, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_OWL_W_TRVDD_PWR_W,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V75_OWL_W_TRVDD_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V75_OWL_W_VDD_TEMP_C
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
			SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			100000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_TEMP_C,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V75_OWL_W_VDD_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2],
		},
	},
	{
		{
			// ASIC_P0V75_OWL_W_VDD_VOLT_V
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
			SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_VOLT_V, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			3571, //uint32_t critical_high;
			3041, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V75_OWL_W_VDD_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2],
		},
	},
	{
		{
			// ASIC_P0V75_OWL_W_VDD_CURR_A
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
			SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_CURR_A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_CURR_A,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V75_OWL_W_VDD_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2],
		},
	},
	{
		{
			// ASIC_P0V75_OWL_W_VDD_PWR_W
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
			SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_PWR_W, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_PWR_W,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V75_OWL_W_VDD_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2],
		},
	},
	{
		{
			// ASIC_P0V75_MAX_M_VDD_TEMP_C
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
			SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			100000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_TEMP_C,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V75_MAX_M_VDD_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_4 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_4 * 2],
		},
	},
	{
		{
			// ASIC_P0V75_MAX_M_VDD_VOLT_V
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
			SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_VOLT_V, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			3571, //uint32_t critical_high;
			3041, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V75_MAX_M_VDD_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_4 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_4 * 2],
		},
	},
	{
		{
			// ASIC_P0V75_MAX_M_VDD_CURR_A
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
			SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_CURR_A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_CURR_A,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V75_MAX_M_VDD_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_4 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_4 * 2],
		},
	},
	{
		{
			// ASIC_P0V75_MAX_M_VDD_PWR_W
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
			SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_PWR_W, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_PWR_W,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V75_MAX_M_VDD_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_4 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_4 * 2],
		},
	},
	{
		{
			// ASIC_P0V75_MAX_N_VDD_TEMP_C
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
			SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			100000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_TEMP_C,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V75_MAX_N_VDD_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_7 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_7 * 2],
		},
	},
	{
		{
			// ASIC_P0V75_MAX_N_VDD_VOLT_V
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
			SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_VOLT_V, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			3571, //uint32_t critical_high;
			3041, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V75_MAX_N_VDD_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_7 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_7 * 2],
		},
	},
	{
		{
			// ASIC_P0V75_MAX_N_VDD_CURR_A
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
			SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_CURR_A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_CURR_A,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V75_MAX_N_VDD_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_7 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_7 * 2],
		},
	},
	{
		{
			// ASIC_P0V75_MAX_N_VDD_PWR_W
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
			SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_PWR_W, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_PWR_W,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V75_MAX_N_VDD_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_7 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_7 * 2],
		},
	},
	{
		{
			// ASIC_P0V75_MAX_S_VDD_TEMP_C
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
			SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			100000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_TEMP_C,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V75_MAX_S_VDD_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V75_MAX_S_VDD_VOLT_V
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
			SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_VOLT_V, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			3571, //uint32_t critical_high;
			3041, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V75_MAX_S_VDD_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V75_MAX_S_VDD_CURR_A
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
			SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_CURR_A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_CURR_A,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V75_MAX_S_VDD_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V75_MAX_S_VDD_PWR_W
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
			SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_PWR_W, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_PWR_W,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V75_MAX_S_VDD_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V8_HAMSA_AVDD_PCIE_TEMP_C
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
			SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			100000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_TEMP_C,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V8_HAMSA_AVDD_PCIE_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_7 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_7 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V8_HAMSA_AVDD_PCIE_VOLT_V
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
			SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_VOLT_V, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			3571, //uint32_t critical_high;
			3041, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V8_HAMSA_AVDD_PCIE_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_7 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_7 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V8_HAMSA_AVDD_PCIE_CURR_A
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
			SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_CURR_A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_CURR_A,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V8_HAMSA_AVDD_PCIE_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_7 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_7 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V8_HAMSA_AVDD_PCIE_PWR_W
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
			SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_PWR_W, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_PWR_W,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V8_HAMSA_AVDD_PCIE_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_7 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_7 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_TEMP_C
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
			SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			100000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_TEMP_C,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2],
		},
	},
	{
		{
			// ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_VOLT_V
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
			SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_VOLT_V, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			3571, //uint32_t critical_high;
			3041, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2],
		},
	},
	{
		{
			// ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_CURR_A
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
			SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_CURR_A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_CURR_A,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2],
		},
	},
	{
		{
			// ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_PWR_W
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
			SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_PWR_W, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_PWR_W,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2],
		},
	},
	{
		{
			// ASIC_P0V85_HAMSA_VDD_TEMP_C
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
			SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			100000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_TEMP_C,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V85_HAMSA_VDD_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V75_VDDPHY_HBM0246_TEMP_C
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
			SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			100000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_TEMP_C,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V75_VDDPHY_HBM0246_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_10 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_10 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V75_VDDPHY_HBM0246_VOLT_V
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
			SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_VOLT_V, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			3571, //uint32_t critical_high;
			3041, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V75_VDDPHY_HBM0246_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_10 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_10 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V75_VDDPHY_HBM0246_CURR_A
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
			SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_CURR_A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_CURR_A,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V75_VDDPHY_HBM0246_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_10 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_10 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V75_VDDPHY_HBM0246_PWR_W
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
			SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_PWR_W, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_PWR_W,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V75_VDDPHY_HBM0246_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_10 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_10 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V4_VDDQL_HBM0246_TEMP_C
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
			SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			100000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_TEMP_C,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V4_VDDQL_HBM0246_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_10 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_10 * 2],
		},
	},
	{
		{
			// ASIC_P0V4_VDDQL_HBM0246_VOLT_V
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
			SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_VOLT_V, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			3571, //uint32_t critical_high;
			3041, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V4_VDDQL_HBM0246_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_10 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_10 * 2],
		},
	},
	{
		{
			// ASIC_P0V4_VDDQL_HBM0246_CURR_A
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
			SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_CURR_A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_CURR_A,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V4_VDDQL_HBM0246_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_10 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_10 * 2],
		},
	},
	{
		{
			// ASIC_P0V4_VDDQL_HBM0246_PWR_W
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
			SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_PWR_W, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_PWR_W,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V4_VDDQL_HBM0246_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_10 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_10 * 2],
		},
	},
	{
		{
			// ASIC_P1V1_VDDQC_HBM0246_TEMP_C
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
			SENSOR_NUM_ASIC_P1V1_VDDQC_HBM0246_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V1_VDDQC_HBM0246_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			100000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V1_VDDQC_HBM0246_TEMP_C,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P1V1_VDDQC_HBM0246_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_9 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_9 * 2],
		},
	},
	{
		{
			// ASIC_P1V1_VDDQC_HBM0246_VOLT_V
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
			SENSOR_NUM_ASIC_P1V1_VDDQC_HBM0246_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V1_VDDQC_HBM0246_VOLT_V, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			3571, //uint32_t critical_high;
			3041, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V1_VDDQC_HBM0246_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P1V1_VDDQC_HBM0246_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_9 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_9 * 2],
		},
	},
	{
		{
			// ASIC_P1V1_VDDQC_HBM0246_CURR_A
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
			SENSOR_NUM_ASIC_P1V1_VDDQC_HBM0246_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V1_VDDQC_HBM0246_CURR_A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V1_VDDQC_HBM0246_CURR_A,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P1V1_VDDQC_HBM0246_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_9 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_9 * 2],
		},
	},
	{
		{
			// ASIC_P1V1_VDDQC_HBM0246_PWR_W
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
			SENSOR_NUM_ASIC_P1V1_VDDQC_HBM0246_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V1_VDDQC_HBM0246_PWR_W, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V1_VDDQC_HBM0246_PWR_W,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P1V1_VDDQC_HBM0246_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_9 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_9 * 2],
		},
	},
	{
		{
			// ASIC_P1V8_VPP_HBM0246_TEMP_C
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
			SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			100000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_TEMP_C,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P1V8_VPP_HBM0246_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_9 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_9 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P1V8_VPP_HBM0246_VOLT_V
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
			SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_VOLT_V, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			3571, //uint32_t critical_high;
			3041, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P1V8_VPP_HBM0246_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_9 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_9 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P1V8_VPP_HBM0246_CURR_A
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
			SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_CURR_A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_CURR_A,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P1V8_VPP_HBM0246_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_9 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_9 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P1V8_VPP_HBM0246_PWR_W
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
			SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_PWR_W, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_PWR_W,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P1V8_VPP_HBM0246_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_9 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_9 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V75_VDDPHY_HBM1357_TEMP_C
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
			SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			100000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_TEMP_C,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V75_VDDPHY_HBM1357_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_4 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_4 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V75_VDDPHY_HBM1357_VOLT_V
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
			SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_VOLT_V, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			3571, //uint32_t critical_high;
			3041, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V75_VDDPHY_HBM1357_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_4 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_4 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V75_VDDPHY_HBM1357_CURR_A
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
			SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_CURR_A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_CURR_A,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V75_VDDPHY_HBM1357_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_4 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_4 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V75_VDDPHY_HBM1357_PWR_W
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
			SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_PWR_W, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_PWR_W,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V75_VDDPHY_HBM1357_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_4 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_4 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V4_VDDQL_HBM1357_TEMP_C
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
			SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			100000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_TEMP_C,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V4_VDDQL_HBM1357_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_5 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_5 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V4_VDDQL_HBM1357_VOLT_V
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
			SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_VOLT_V, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			3571, //uint32_t critical_high;
			3041, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V4_VDDQL_HBM1357_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_5 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_5 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V4_VDDQL_HBM1357_CURR_A
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
			SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_CURR_A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_CURR_A,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V4_VDDQL_HBM1357_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_5 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_5 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V4_VDDQL_HBM1357_PWR_W
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
			SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_PWR_W, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_PWR_W,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V4_VDDQL_HBM1357_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_5 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_5 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P1V1_VDDQC_HBM1357_TEMP_C
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
			SENSOR_NUM_ASIC_P1V1_VDDQC_HBM1357_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V1_VDDQC_HBM1357_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			100000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V1_VDDQC_HBM1357_TEMP_C,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P1V1_VDDQC_HBM1357_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_6 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_6 * 2],
		},
	},
	{
		{
			// ASIC_P1V1_VDDQC_HBM1357_VOLT_V
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
			SENSOR_NUM_ASIC_P1V1_VDDQC_HBM1357_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V1_VDDQC_HBM1357_VOLT_V, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			3571, //uint32_t critical_high;
			3041, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V1_VDDQC_HBM1357_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P1V1_VDDQC_HBM1357_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_6 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_6 * 2],
		},
	},
	{
		{
			// ASIC_P1V1_VDDQC_HBM1357_CURR_A
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
			SENSOR_NUM_ASIC_P1V1_VDDQC_HBM1357_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V1_VDDQC_HBM1357_CURR_A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V1_VDDQC_HBM1357_CURR_A,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P1V1_VDDQC_HBM1357_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_6 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_6 * 2],
		},
	},
	{
		{
			// ASIC_P1V1_VDDQC_HBM1357_PWR_W
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
			SENSOR_NUM_ASIC_P1V1_VDDQC_HBM1357_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V1_VDDQC_HBM1357_PWR_W, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V1_VDDQC_HBM1357_PWR_W,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P1V1_VDDQC_HBM1357_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_6 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_6 * 2],
		},
	},
	{
		{
			// ASIC_P1V8_VPP_HBM1357_TEMP_C
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
			SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			100000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_TEMP_C,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P1V8_VPP_HBM1357_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_6 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_6 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P1V8_VPP_HBM1357_VOLT_V
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
			SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_VOLT_V, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			3571, //uint32_t critical_high;
			3041, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P1V8_VPP_HBM1357_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_6 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_6 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P1V8_VPP_HBM1357_CURR_A
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
			SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_CURR_A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_CURR_A,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P1V8_VPP_HBM1357_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_6 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_6 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P1V8_VPP_HBM1357_PWR_W
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
			SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_PWR_W, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_PWR_W,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P1V8_VPP_HBM1357_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_6 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_6 * 2 + 1],
		},
	},
};

pldm_sensor_info plat_pldm_sensor_quick_vr_table[] = {
	{
		{
			// ASIC_P0V85_MEDHA0_VDD_VOLT_V
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
			SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_VOLT_V, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			3571, //uint32_t critical_high;
			3041, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_VOLT_V,
			.type = sensor_dev_mp29816a,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V85_MEDHA0_VDD_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_2 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_2 * 2],
		},
	},
	{
		{
			// ASIC_P0V85_MEDHA0_VDD_CURR_A
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
			SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_CURR_A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_CURR_A,
			.type = sensor_dev_mp29816a,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V85_MEDHA0_VDD_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_2 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_2 * 2],
		},
	},
	{
		{
			// ASIC_P0V85_MEDHA0_VDD_PWR_W
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
			SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_PWR_W, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_PWR_W,
			.type = sensor_dev_mp29816a,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V85_MEDHA0_VDD_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_2 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_2 * 2],
		},
	},
	{
		{
			// ASIC_P0V85_MEDHA1_VDD_VOLT_V
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
			SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_VOLT_V, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			3571, //uint32_t critical_high;
			3041, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_VOLT_V,
			.type = sensor_dev_mp29816a,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V85_MEDHA1_VDD_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_1 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_1 * 2],
		},
	},
	{
		{
			// ASIC_P0V85_MEDHA1_VDD_CURR_A
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
			SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_CURR_A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_CURR_A,
			.type = sensor_dev_mp29816a,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V85_MEDHA1_VDD_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_1 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_1 * 2],
		},
	},
	{
		{
			// ASIC_P0V85_MEDHA1_VDD_PWR_W
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
			SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_PWR_W, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_PWR_W,
			.type = sensor_dev_mp29816a,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V85_MEDHA1_VDD_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_1 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_1 * 2],
		},
	},
	{
		{
			// ASIC_P0V85_HAMSA_VDD_VOLT_V
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
			SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_VOLT_V, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			3571, //uint32_t critical_high;
			3041, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V85_HAMSA_VDD_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V85_HAMSA_VDD_CURR_A
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
			SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_CURR_A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_CURR_A,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V85_HAMSA_VDD_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V85_HAMSA_VDD_PWR_W
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
			SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_PWR_W, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_PWR_W,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V85_HAMSA_VDD_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2 + 1],
		},
	},
};

pldm_sensor_info plat_pldm_sensor_ubc_table[] = {
	{
		{
			// UBC1_P12V_TEMP_C
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
			SENSOR_NUM_UBC1_P12V_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_UBC1_P12V_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			100000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_UBC1_P12V_TEMP_C,
			.type = sensor_dev_u50su4p180pmdafc,
			.port = I2C_BUS10,
			.target_addr = UBC1_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_ubc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// UBC1_P12V_VOLT_V
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
			SENSOR_NUM_UBC1_P12V_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_UBC1_P12V_VOLT_V, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			15000, //uint32_t critical_high;
			9500, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_UBC1_P12V_VOLT_V,
			.type = sensor_dev_u50su4p180pmdafc,
			.port = I2C_BUS10,
			.target_addr = UBC1_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_ubc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// UBC1_P12V_CURR_A
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
			SENSOR_NUM_UBC1_P12V_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_UBC1_P12V_CURR_A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_UBC1_P12V_CURR_A,
			.type = sensor_dev_u50su4p180pmdafc,
			.port = I2C_BUS10,
			.target_addr = UBC1_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_ubc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// UBC1_P12V_PWR_W
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
			SENSOR_NUM_UBC1_P12V_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_UBC1_P12V_PWR_W, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_UBC1_P12V_PWR_W,
			.type = sensor_dev_u50su4p180pmdafc,
			.port = I2C_BUS10,
			.target_addr = UBC1_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_ubc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// UBC1_P52V_INPUT_VOLT_V
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
			SENSOR_NUM_UBC1_P52V_INPUT_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_UBC1_P52V_INPUT_VOLT_V, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			60000, //uint32_t critical_high;
			38000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_UBC1_P52V_INPUT_VOLT_V,
			.type = sensor_dev_u50su4p180pmdafc,
			.port = I2C_BUS10,
			.target_addr = UBC1_ADDR,
			.offset = PMBUS_READ_VIN,
			.access_checker = is_ubc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// UBC2_P12V_TEMP_C
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
			SENSOR_NUM_UBC2_P12V_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_UBC2_P12V_TEMP_C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			100000, //uint32_t critical_high;
			5000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_UBC2_P12V_TEMP_C,
			.type = sensor_dev_u50su4p180pmdafc,
			.port = I2C_BUS10,
			.target_addr = UBC2_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_ubc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// UBC2_P12V_VOLT_V
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
			SENSOR_NUM_UBC2_P12V_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_UBC2_P12V_VOLT_V, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			15000, //uint32_t critical_high;
			9500, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_UBC2_P12V_VOLT_V,
			.type = sensor_dev_u50su4p180pmdafc,
			.port = I2C_BUS10,
			.target_addr = UBC2_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_ubc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// UBC2_P12V_CURR_A
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
			SENSOR_NUM_UBC2_P12V_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_UBC2_P12V_CURR_A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_UBC2_P12V_CURR_A,
			.type = sensor_dev_u50su4p180pmdafc,
			.port = I2C_BUS10,
			.target_addr = UBC2_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_ubc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// UBC2_P12V_PWR_W
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
			SENSOR_NUM_UBC2_P12V_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_UBC2_P12V_PWR_W, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0, //uint8_t supported_thresholds;
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			0, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_UBC2_P12V_PWR_W,
			.type = sensor_dev_u50su4p180pmdafc,
			.port = I2C_BUS10,
			.target_addr = UBC2_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_ubc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// UBC2_P52V_INPUT_VOLT_V
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
			SENSOR_NUM_UBC2_P52V_INPUT_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_UBC2_P52V_INPUT_VOLT_V, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			-3, //int8_t unit_modifier; //Need to check
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
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			60000, //uint32_t critical_high;
			38000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_UBC2_P52V_INPUT_VOLT_V,
			.type = sensor_dev_u50su4p180pmdafc,
			.port = I2C_BUS10,
			.target_addr = UBC2_ADDR,
			.offset = PMBUS_READ_VIN,
			.access_checker = is_ubc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
};

PDR_sensor_auxiliary_names plat_pdr_sensor_aux_names_table[] = {
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_TOP_INLET_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_TOP_INLET_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_BOT_INLET_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_BOT_INLET_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_BOT_OUTLET_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_BOT_OUTLET_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_MEDHA0_SENSOR0_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_MEDHA0_SENSOR0_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_MEDHA0_SENSOR1_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_MEDHA0_SENSOR1_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_OWL_W_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_OWL_W_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_OWL_E_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_OWL_E_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_MEDHA1_SENSOR0_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_MEDHA1_SENSOR0_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_MEDHA1_SENSOR1_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_MEDHA1_SENSOR1_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_HAMSA_CRM_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_HAMSA_CRM_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_HAMSA_LS_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_HAMSA_LS_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_VOLT_V",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_CURR_A",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_PWR_W",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_VOLT_V",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_CURR_A",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_PWR_W",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_VOLT_V",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_CURR_A",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_PWR_W",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_VOLT_V",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_CURR_A",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_PWR_W",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_VOLT_V",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_CURR_A",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_PWR_W",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_VOLT_V",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_CURR_A",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_PWR_W",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_OWL_W_TRVDD_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_OWL_W_TRVDD_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_OWL_W_TRVDD_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_OWL_W_TRVDD_VOLT_V",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_OWL_W_TRVDD_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_OWL_W_TRVDD_CURR_A",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_OWL_W_TRVDD_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_OWL_W_TRVDD_PWR_W",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_VOLT_V",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_CURR_A",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_PWR_W",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_VOLT_V",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_CURR_A",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_PWR_W",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_VOLT_V",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_CURR_A",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_PWR_W",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_VOLT_V",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_CURR_A",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_PWR_W",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_VOLT_V",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_CURR_A",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_PWR_W",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_VOLT_V",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_CURR_A",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_PWR_W",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_VOLT_V",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_CURR_A",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_PWR_W",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_VOLT_V",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_CURR_A",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_PWR_W",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_VOLT_V",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_CURR_A",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_PWR_W",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P1V1_VDDQC_HBM0246_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P1V1_VDDQC_HBM0246_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P1V1_VDDQC_HBM0246_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P1V1_VDDQC_HBM0246_VOLT_V",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P1V1_VDDQC_HBM0246_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P1V1_VDDQC_HBM0246_CURR_A",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P1V1_VDDQC_HBM0246_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P1V1_VDDQC_HBM0246_PWR_W",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_VOLT_V",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_CURR_A",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_PWR_W",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_VOLT_V",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_CURR_A",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_PWR_W",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_VOLT_V",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_CURR_A",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_PWR_W",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P1V1_VDDQC_HBM1357_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P1V1_VDDQC_HBM1357_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P1V1_VDDQC_HBM1357_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P1V1_VDDQC_HBM1357_VOLT_V",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P1V1_VDDQC_HBM1357_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P1V1_VDDQC_HBM1357_CURR_A",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P1V1_VDDQC_HBM1357_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P1V1_VDDQC_HBM1357_PWR_W",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_VOLT_V",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_CURR_A",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_PWR_W",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_UBC1_P12V_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_UBC1_P12V_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_UBC1_P12V_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_UBC1_P12V_VOLT_V",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_UBC1_P12V_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_UBC1_P12V_CURR_A",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_UBC1_P12V_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_UBC1_P12V_PWR_W",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_UBC1_P52V_INPUT_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_UBC1_P52V_INPUT_VOLT_V",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_UBC2_P12V_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_UBC2_P12V_TEMP_C",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_UBC2_P12V_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_UBC2_P12V_VOLT_V",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_UBC2_P12V_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_UBC2_P12V_CURR_A",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_UBC2_P12V_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_UBC2_P12V_PWR_W",
	},
	{
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_UBC2_P52V_INPUT_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"SENSOR_NUM_UBC2_P52V_INPUT_VOLT_V",
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
	case UBC_SENSOR_THREAD_ID:
		return plat_pldm_sensor_ubc_table;
	case VR_SENSOR_THREAD_ID:
		return plat_pldm_sensor_vr_table;
	case QUICK_VR_SENSOR_THREAD_ID:
		return plat_pldm_sensor_quick_vr_table;
	case TEMP_SENSOR_THREAD_ID:
		return plat_pldm_sensor_temp_table;
	default:
		LOG_ERR("Unknow pldm sensor thread id %d", thread_id);
		return NULL;
	}
}

int plat_pldm_sensor_get_sensor_count(int thread_id)
{
	int count = 0;

	switch (thread_id) {
	case TEMP_SENSOR_THREAD_ID:
		count = ARRAY_SIZE(plat_pldm_sensor_temp_table);
		break;
	case VR_SENSOR_THREAD_ID:
		count = ARRAY_SIZE(plat_pldm_sensor_vr_table);
		break;
	case QUICK_VR_SENSOR_THREAD_ID:
		count = ARRAY_SIZE(plat_pldm_sensor_quick_vr_table);
		break;
	case UBC_SENSOR_THREAD_ID:
		count = ARRAY_SIZE(plat_pldm_sensor_ubc_table);
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
	case TEMP_SENSOR_THREAD_ID:
		memcpy(numeric_sensor_table,
		       &plat_pldm_sensor_temp_table[sensor_num].pdr_numeric_sensor,
		       sizeof(PDR_numeric_sensor));
		break;
	case VR_SENSOR_THREAD_ID:
		memcpy(numeric_sensor_table,
		       &plat_pldm_sensor_vr_table[sensor_num].pdr_numeric_sensor,
		       sizeof(PDR_numeric_sensor));
		break;
	case QUICK_VR_SENSOR_THREAD_ID:
		memcpy(numeric_sensor_table,
		       &plat_pldm_sensor_quick_vr_table[sensor_num].pdr_numeric_sensor,
		       sizeof(PDR_numeric_sensor));
		break;
	case UBC_SENSOR_THREAD_ID:
		memcpy(numeric_sensor_table,
		       &plat_pldm_sensor_ubc_table[sensor_num].pdr_numeric_sensor,
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
	const char16_t base_name[] = u"RB";

	// Get slot ID
	uint8_t slot_id = get_mmc_slot() + 1;

	// Calculate the length of the base name
	size_t base_len = char16_strlen(base_name);

	// Calculate the required length for the final string (base name + null terminator)
	size_t total_len = base_len + 1; // +1 for the null terminator

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

sensor_cfg *get_sensor_cfg_by_sensor_id(uint8_t sensor_id)
{
	uint8_t type = check_sensor_type(sensor_id);
	CHECK_ARG_WITH_RETURN(type == MAX_SENSOR_THREAD_ID, NULL);

	pldm_sensor_info *table = plat_pldm_sensor_load(type);
	CHECK_NULL_ARG_WITH_RETURN(table, NULL);

	int count = plat_pldm_sensor_get_sensor_count(type);
	CHECK_ARG_WITH_RETURN(count < 0, NULL);

	for (uint8_t i = 0; i < count; i++) {
		if (table[i].pldm_sensor_cfg.num == sensor_id)
			return &table[i].pldm_sensor_cfg;
	}

	return NULL;
}

bool get_raw_data_from_sensor_id(uint8_t sensor_id, uint8_t offset, uint8_t *val, uint8_t len)
{
	CHECK_NULL_ARG_WITH_RETURN(val, false);

	sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);

	bool ret = true;

	memset(val, 0, len);

	if ((cfg->pre_sensor_read_hook)) {
		if ((cfg->pre_sensor_read_hook)(cfg, cfg->pre_sensor_read_args) == false) {
			LOG_DBG("%d read raw val pre hook fail!", sensor_id);
			return false;
		}
	}

	if (!plat_i2c_read(cfg->port, cfg->target_addr, offset, val, len)) {
		LOG_DBG("%d read raw value fail!", sensor_id);
		ret = false;
		goto err;
	}

err:
	if ((cfg->post_sensor_read_hook)) {
		if ((cfg->post_sensor_read_hook)(cfg, cfg->post_sensor_read_args, 0) == false) {
			LOG_DBG("%d read raw value post hook fail!", sensor_id);
			return false;
		}
	}

	return ret;
}

void change_sensor_cfg(uint8_t vr_module)
{
	if (vr_module == VR_MODULE_MPS)
		return;

	for (uint8_t i = VR_SENSOR_THREAD_ID; i <= QUICK_VR_SENSOR_THREAD_ID; i++) {
		pldm_sensor_info *table = plat_pldm_sensor_load(i);
		if (table == NULL)
			return;

		int count = plat_pldm_sensor_get_sensor_count(i);
		if (count < 0)
			return;

		for (uint8_t j = 0; j < count; j++) {
			table[j].pldm_sensor_cfg.type = sensor_dev_raa228249;
			table[j].pldm_sensor_cfg.target_addr =
				convert_addr_to_rns(table[j].pldm_sensor_cfg.target_addr);
		}
	}
}

bool is_dc_access(uint8_t sensor_num)
{
	return is_mb_dc_on();
}

void set_plat_sensor_polling_enable_flag(bool value)
{
	plat_sensor_polling_enable_flag = value;
}

void set_plat_sensor_ubc_polling_enable_flag(bool value)
{
	plat_sensor_ubc_polling_enable_flag = value;
}

void set_plat_sensor_temp_polling_enable_flag(bool value)
{
	plat_sensor_temp_polling_enable_flag = value;
}

void set_plat_sensor_vr_polling_enable_flag(bool value)
{
	plat_sensor_vr_polling_enable_flag = value;
}

bool get_plat_sensor_polling_enable_flag()
{
	return plat_sensor_polling_enable_flag;
}

bool get_plat_sensor_ubc_polling_enable_flag()
{
	return plat_sensor_ubc_polling_enable_flag;
}

bool get_plat_sensor_temp_polling_enable_flag()
{
	return plat_sensor_temp_polling_enable_flag;
}

bool get_plat_sensor_vr_polling_enable_flag()
{
	return plat_sensor_vr_polling_enable_flag;
}

bool is_ubc_access(uint8_t sensor_num)
{
	return (is_dc_access(sensor_num) && get_plat_sensor_ubc_polling_enable_flag() &&
		get_plat_sensor_polling_enable_flag());
}

bool is_temp_access(uint8_t cfg_idx)
{
	return (get_plat_sensor_temp_polling_enable_flag() &&
		get_plat_sensor_polling_enable_flag());
}

bool is_vr_access(uint8_t sensor_num)
{
	return (is_dc_access(sensor_num) && get_plat_sensor_vr_polling_enable_flag() &&
		get_plat_sensor_polling_enable_flag());
}
