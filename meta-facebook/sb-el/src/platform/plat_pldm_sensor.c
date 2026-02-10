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

static bool plat_sensor_polling_enable_flag = false;
static bool plat_sensor_ubc_polling_enable_flag = true;
static bool plat_sensor_temp_polling_enable_flag = true;
static bool plat_sensor_vr_polling_enable_flag = true;
static uint8_t plat_sensor_one_step_power_enable_flag = 0;

static uint32_t quick_vr_poll_interval = QUICK_POLL_INTERVAL;
static bool is_quick_vr_poll_changed = false;

static struct pldm_sensor_thread pal_pldm_sensor_thread[MAX_SENSOR_THREAD_ID] = {
	// thread id, thread name
	{ TEMP_SENSOR_THREAD_ID, "TEMP_SENSOR_THREAD" },
	{ VR_SENSOR_THREAD_ID, "VR_PLDM_SENSOR_THREAD" },
	{ QUICK_VR_SENSOR_THREAD_ID, "QUICK_VR_PLDM_SENSOR_THREAD", QUICK_POLL_INTERVAL, true },
	{ UBC_SENSOR_THREAD_ID, "UBC_PLDM_SENSOR_THREAD" },
};

extern vr_pre_proc_arg vr_pre_read_args[];
extern mpc12109_init_arg mpc12109_init_args[];
uint8_t ioe_init_flag = 0;
static bool is_quick_vr_sensor(uint8_t sensor_num)
{
	switch (sensor_num) {
	case SENSOR_NUM_ASIC_P0V75_NUWA0_VDD_VOLT_V:
	case SENSOR_NUM_ASIC_P0V75_NUWA0_VDD_PWR_W:
	case SENSOR_NUM_ASIC_P0V75_NUWA1_VDD_VOLT_V:
	case SENSOR_NUM_ASIC_P0V75_NUWA1_VDD_PWR_W:
		return true;
	}
	return false;
}

typedef struct {
	uint8_t orig_addr; // old MPS
	uint8_t rns_addr; // old rns
} addr_map_t;

static const addr_map_t addr_map_table[] = {
	{ ASIC_P0V75_NUWA0_VDD_ADDR, ASIC_P0V75_NUWA0_VDD_RNS_ADDR},
	{ ASIC_P0V75_NUWA1_VDD_ADDR, ASIC_P0V75_NUWA1_VDD_RNS_ADDR},
	{ ASIC_P0V9_OWL_E_TRVDD_ADDR, ASIC_P0V9_OWL_E_TRVDD_RNS_ADDR},
	{ ASIC_P0V75_OWL_E_TRVDD_ADDR, ASIC_P0V75_OWL_E_TRVDD_RNS_ADDR},
	{ ASIC_P0V75_MAX_M_VDD_ADDR, ASIC_P0V75_MAX_M_VDD_RNS_ADDR},
	{ ASIC_P0V75_VDDPHY_HBM1357_ADDR, ASIC_P0V75_VDDPHY_HBM1357_RNS_ADDR},
	{ ASIC_P0V75_OWL_E_VDD_ADDR, ASIC_P0V75_OWL_E_VDD_RNS_ADDR},
	{ ASIC_P0V4_VDDQL_HBM1357_ADDR, ASIC_P0V4_VDDQL_HBM1357_RNS_ADDR},
	{ ASIC_P1V05_VDDQC_HBM1357_ADDR, ASIC_P1V05_VDDQC_HBM1357_RNS_ADDR},
	{ ASIC_P1V8_VPP_HBM1357_ADDR, ASIC_P1V8_VPP_HBM1357_RNS_ADDR},
	{ ASIC_P0V9_VDDQ_HBM1357_ADDR, ASIC_P0V9_VDDQ_HBM1357_RNS_ADDR},
	{ ASIC_P0V85_HAMSA_VDD_ADDR, ASIC_P0V85_HAMSA_VDD_RNS_ADDR},
	{ ASIC_P0V75_MAX_N_VDD_ADDR, ASIC_P0V75_MAX_N_VDD_RNS_ADDR},
	{ ASIC_P0V8_HAMSA_AVDD_PCIE_ADDR, ASIC_P0V8_HAMSA_AVDD_PCIE_RNS_ADDR},
	{ ASIC_P0V9_VDDQ_HBM0246_ADDR, ASIC_P0V9_VDDQ_HBM0246_RNS_ADDR},
	{ ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_ADDR, ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_RNS_ADDR},
	{ ASIC_P1V05_VDDQC_HBM0246_ADDR, ASIC_P1V05_VDDQC_HBM0246_RNS_ADDR},
	{ ASIC_P1V8_VPP_HBM0246_ADDR, ASIC_P1V8_VPP_HBM0246_RNS_ADDR},
	{ ASIC_P0V4_VDDQL_HBM0246_ADDR, ASIC_P0V4_VDDQL_HBM0246_RNS_ADDR},
	{ ASIC_P0V75_VDDPHY_HBM0246_ADDR, ASIC_P0V75_VDDPHY_HBM0246_RNS_ADDR},
	{ ASIC_P0V75_OWL_W_VDD_ADDR, ASIC_P0V75_OWL_W_VDD_RNS_ADDR},
	{ ASIC_P0V75_MAX_S_VDD_ADDR, ASIC_P0V75_MAX_S_VDD_RNS_ADDR},
	{ ASIC_P0V9_OWL_W_TRVDD_ADDR, ASIC_P0V9_OWL_W_TRVDD_RNS_ADDR},
	{ ASIC_P0V75_OWL_W_TRVDD_ADDR, ASIC_P0V75_OWL_W_TRVDD_RNS_ADDR},
};

uint8_t convert_vr_addr(uint8_t addr, uint8_t vr_change_mode)
{
	for (int i = 0; i < ARRAY_SIZE(addr_map_table); i++) {
		if (addr_map_table[i].orig_addr == addr) {
			if (vr_change_mode == OLD_RNS)
				return addr_map_table[i].rns_addr;
			else if (vr_change_mode == OLD_MPS)
				LOG_DBG("don't need to change VR address");
			else
				LOG_ERR("vr_change_mode: 0x%x error", vr_change_mode);
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
			65000, //uint32_t critical_high;
			0, //uint32_t critical_low;
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
			.post_sensor_read_hook = post_common_sensor_read,
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
			65000, //uint32_t critical_high;
			0, //uint32_t critical_low;
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
			.post_sensor_read_hook = post_common_sensor_read,
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
			100000, //uint32_t critical_high;
			0, //uint32_t critical_low;
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
			.post_sensor_read_hook = post_common_sensor_read,
		},
	},
	{
		{
			// ASIC_NUWA0_SENSOR0
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_NUWA0_SENSOR0_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_NUWA0_SENSOR0_TEMP_C, //uint16_t entity_instance_number;
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
			95000, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_NUWA0_SENSOR0_TEMP_C,
			.type = sensor_dev_tmp431,
			.port = I2C_BUS3,
			.target_addr = ASIC_NUWA0_SENSOR0_ADDR,
			.offset = TMP432_REMOTE_TEMPERATRUE_1,
			.access_checker = is_temp_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.post_sensor_read_hook = post_common_sensor_read,
		},
	},
	{
		{
			// ASIC_NUWA0_SENSOR1
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_NUWA0_SENSOR1_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_NUWA0_SENSOR1_TEMP_C, //uint16_t entity_instance_number;
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
			95000, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_NUWA0_SENSOR1_TEMP_C,
			.type = sensor_dev_tmp431,
			.port = I2C_BUS3,
			.target_addr = ASIC_NUWA0_SENSOR1_ADDR,
			.offset = TMP432_REMOTE_TEMPERATRUE_2,
			.access_checker = is_temp_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.post_sensor_read_hook = post_common_sensor_read,
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
			95000, //uint32_t critical_high;
			0, //uint32_t critical_low;
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
			.post_sensor_read_hook = post_common_sensor_read,
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
			95000, //uint32_t critical_high;
			0, //uint32_t critical_low;
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
			.post_sensor_read_hook = post_common_sensor_read,
		},
	},
	{
		{
			// ASIC_NUWA1_SENSOR0
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_NUWA1_SENSOR0_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_NUWA1_SENSOR0_TEMP_C, //uint16_t entity_instance_number;
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
			95000, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_NUWA1_SENSOR0_TEMP_C,
			.type = sensor_dev_tmp431,
			.port = I2C_BUS2,
			.target_addr = ASIC_NUWA1_SENSOR0_ADDR,
			.offset = TMP432_REMOTE_TEMPERATRUE_1,
			.access_checker = is_temp_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.post_sensor_read_hook = post_common_sensor_read,
		},
	},
	{
		{
			// ASIC_NUWA1_SENSOR1
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_NUWA1_SENSOR1_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_NUWA1_SENSOR1_TEMP_C, //uint16_t entity_instance_number;
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
			95000, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_NUWA1_SENSOR1_TEMP_C,
			.type = sensor_dev_tmp431,
			.port = I2C_BUS2,
			.target_addr = ASIC_NUWA1_SENSOR1_ADDR,
			.offset = TMP432_REMOTE_TEMPERATRUE_2,
			.access_checker = is_temp_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.post_sensor_read_hook = post_common_sensor_read,
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
			95000, //uint32_t critical_high;
			0, //uint32_t critical_low;
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
			.post_sensor_read_hook = post_common_sensor_read,
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
			95000, //uint32_t critical_high;
			0, //uint32_t critical_low;
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
			.post_sensor_read_hook = post_common_sensor_read,
		},
	},
};

pldm_sensor_info plat_pldm_sensor_vr_table[] = {
	{
		{
			// ASIC_P0V75_NUWA0_VDD_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V75_NUWA0_VDD_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_NUWA0_VDD_TEMP_C, //uint16_t entity_instance_number;
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
			110000, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_NUWA0_VDD_TEMP_C,
			.type = sensor_dev_mp29816a,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V75_NUWA0_VDD_ADDR,
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
			// ASIC_P0V75_NUWA0_VDD_CURR_A
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V75_NUWA0_VDD_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_NUWA0_VDD_CURR_A, //uint16_t entity_instance_number;
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
			1450000, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_NUWA0_VDD_CURR_A,
			.type = sensor_dev_mp29816a,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V75_NUWA0_VDD_ADDR,
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
			// SENSOR_NUM_ASIC_P0V75_NUWA0_VDD_INPUT_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V75_NUWA0_VDD_INPUT_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_NUWA0_VDD_INPUT_VOLT_V, //uint16_t entity_instance_number;
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
			16000, //uint32_t critical_high;
			7000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_NUWA0_VDD_INPUT_VOLT_V,
			.type = sensor_dev_mp29816a,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V75_NUWA0_VDD_ADDR,
			.offset = PMBUS_READ_VIN,
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
			// ASIC_P0V75_NUWA1_VDD_CURR_A
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V75_NUWA1_VDD_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_NUWA1_VDD_CURR_A, //uint16_t entity_instance_number;
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
			1450000, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_NUWA1_VDD_CURR_A,
			.type = sensor_dev_mp29816a,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V75_NUWA1_VDD_ADDR,
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
			// ASIC_P0V75_NUWA1_VDD_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V75_NUWA1_VDD_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_NUWA1_VDD_TEMP_C, //uint16_t entity_instance_number;
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
			110000, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_NUWA1_VDD_TEMP_C,
			.type = sensor_dev_mp29816a,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V75_NUWA1_VDD_ADDR,
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
			// SENSOR_NUM_ASIC_P0V75_NUWA1_VDD_INPUT_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V75_NUWA1_VDD_INPUT_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_NUWA1_VDD_INPUT_VOLT_V, //uint16_t entity_instance_number;
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
			16000, //uint32_t critical_high;
			7000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_NUWA1_VDD_INPUT_VOLT_V,
			.type = sensor_dev_mp29816a,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V75_NUWA1_VDD_ADDR,
			.offset = PMBUS_READ_VIN,
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
			110000, //uint32_t critical_high;
			0, //uint32_t critical_low;
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
			954, //uint32_t critical_high;
			846, //uint32_t critical_low;
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
			15000, //uint32_t critical_high;
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
			13500, //uint32_t critical_high;
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
			// ASIC_P0V9_OWL_E_TRVDD_INPUT_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_INPUT_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_INPUT_VOLT_V, //uint16_t entity_instance_number;
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
			16000, //uint32_t critical_high;
			7000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_INPUT_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V9_OWL_E_TRVDD_ADDR,
			.offset = PMBUS_READ_VIN,
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
			110000, //uint32_t critical_high;
			0, //uint32_t critical_low;
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
			795, //uint32_t critical_high;
			705, //uint32_t critical_low;
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
			9000, //uint32_t critical_high;
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
			6750, //uint32_t critical_high;
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
			// ASIC_P0V75_OWL_E_TRVDD_INPUT_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_INPUT_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_INPUT_VOLT_V, //uint16_t entity_instance_number;
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
			16000, //uint32_t critical_high;
			7000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_INPUT_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V75_OWL_E_TRVDD_ADDR,
			.offset = PMBUS_READ_VIN,
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
			110000, //uint32_t critical_high;
			0, //uint32_t critical_low;
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
			810, //uint32_t critical_high;
			690, //uint32_t critical_low;
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
			150000, //uint32_t critical_high;
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
			112500, //uint32_t critical_high;
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
			// ASIC_P0V75_OWL_E_VDD_INPUT_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_INPUT_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_INPUT_VOLT_V, //uint16_t entity_instance_number;
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
			16000, //uint32_t critical_high;
			7000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_INPUT_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V75_OWL_E_VDD_ADDR,
			.offset = PMBUS_READ_VIN,
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
			110000, //uint32_t critical_high;
			0, //uint32_t critical_low;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_13 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_13 * 2],
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
			954, //uint32_t critical_high;
			846, //uint32_t critical_low;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_13 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_13 * 2],
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
			15000, //uint32_t critical_high;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_13 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_13 * 2],
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
			13500, //uint32_t critical_high;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_13 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_13 * 2],
		},
	},
	{
		{
			// ASIC_P0V9_OWL_W_TRVDD_INPUT_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_INPUT_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_INPUT_VOLT_V, //uint16_t entity_instance_number;
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
			16000, //uint32_t critical_high;
			7000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_INPUT_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V9_OWL_W_TRVDD_ADDR,
			.offset = PMBUS_READ_VIN,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_13 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_13 * 2],
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
			110000, //uint32_t critical_high;
			0, //uint32_t critical_low;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_13 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_13 * 2 + 1],
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
			795, //uint32_t critical_high;
			705, //uint32_t critical_low;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_13 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_13 * 2 + 1],
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
			9000, //uint32_t critical_high;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_13 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_13 * 2 + 1],
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
			6750, //uint32_t critical_high;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_13 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_13 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V75_OWL_W_TRVDD_INPUT_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V75_OWL_W_TRVDD_INPUT_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_OWL_W_TRVDD_INPUT_VOLT_V, //uint16_t entity_instance_number;
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
			16000, //uint32_t critical_high;
			7000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_OWL_W_TRVDD_INPUT_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V75_OWL_W_TRVDD_ADDR,
			.offset = PMBUS_READ_VIN,
			.access_checker = is_vr_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_13 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_13 * 2],
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
			110000, //uint32_t critical_high;
			0, //uint32_t critical_low;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2],
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
			810, //uint32_t critical_high;
			690, //uint32_t critical_low;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2],
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
			150000, //uint32_t critical_high;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2],
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
			112500, //uint32_t critical_high;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2],
		},
	},
	{
		{
			// ASIC_P0V75_OWL_W_VDD_INPUT_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_INPUT_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_INPUT_VOLT_V, //uint16_t entity_instance_number;
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
			16000, //uint32_t critical_high;
			7000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_INPUT_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V75_OWL_W_VDD_ADDR,
			.offset = PMBUS_READ_VIN,
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
			110000, //uint32_t critical_high;
			0, //uint32_t critical_low;
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
			848, //uint32_t critical_high;
			690, //uint32_t critical_low;
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
			57000, //uint32_t critical_high;
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
			42750, //uint32_t critical_high;
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
			// ASIC_P0V75_MAX_M_VDD_INPUT_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_INPUT_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_INPUT_VOLT_V, //uint16_t entity_instance_number;
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
			16000, //uint32_t critical_high;
			7000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_INPUT_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V75_MAX_M_VDD_ADDR,
			.offset = PMBUS_READ_VIN,
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
			110000, //uint32_t critical_high;
			0, //uint32_t critical_low;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2],
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
			848, //uint32_t critical_high;
			690, //uint32_t critical_low;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2],
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
			57000, //uint32_t critical_high;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2],
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
			42750, //uint32_t critical_high;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2],
		},
	},
	{
		{
			// ASIC_P0V75_MAX_N_VDD_INPUT_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_INPUT_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_INPUT_VOLT_V, //uint16_t entity_instance_number;
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
			16000, //uint32_t critical_high;
			7000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_INPUT_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V75_MAX_N_VDD_ADDR,
			.offset = PMBUS_READ_VIN,
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
			110000, //uint32_t critical_high;
			0, //uint32_t critical_low;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2 + 1],
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
			848, //uint32_t critical_high;
			690, //uint32_t critical_low;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2 + 1],
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
			48000, //uint32_t critical_high;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2 + 1],
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
			36000, //uint32_t critical_high;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_12 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V75_MAX_S_VDD_INPUT_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_INPUT_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_INPUT_VOLT_V, //uint16_t entity_instance_number;
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
			16000, //uint32_t critical_high;
			7000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_INPUT_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V75_MAX_S_VDD_ADDR,
			.offset = PMBUS_READ_VIN,
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
			110000, //uint32_t critical_high;
			0, //uint32_t critical_low;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2 + 1],
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
			848, //uint32_t critical_high;
			752, //uint32_t critical_low;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2 + 1],
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
			40000, //uint32_t critical_high;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2 + 1],
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
			32000, //uint32_t critical_high;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_8 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V8_HAMSA_AVDD_PCIE_INPUT_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_INPUT_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_INPUT_VOLT_V, //uint16_t entity_instance_number;
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
			16000, //uint32_t critical_high;
			7000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_INPUT_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V8_HAMSA_AVDD_PCIE_ADDR,
			.offset = PMBUS_READ_VIN,
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
			110000, //uint32_t critical_high;
			0, //uint32_t critical_low;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_9 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_9 * 2 + 1],
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
			1272, //uint32_t critical_high;
			1128, //uint32_t critical_low;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_9 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_9 * 2 + 1],
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
			9000, //uint32_t critical_high;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_9 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_9 * 2 + 1],
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
			10800, //uint32_t critical_high;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_9 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_9 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_INPUT_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_INPUT_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_INPUT_VOLT_V, //uint16_t entity_instance_number;
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
			16000, //uint32_t critical_high;
			7000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_INPUT_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_ADDR,
			.offset = PMBUS_READ_VIN,
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
			110000, //uint32_t critical_high;
			0, //uint32_t critical_low;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_7 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_7 * 2 + 1],
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
			918, //uint32_t critical_high;
			782, //uint32_t critical_low;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_7 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_7 * 2 + 1],
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
			75000, //uint32_t critical_high;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_7 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_7 * 2 + 1],
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
			63750, //uint32_t critical_high;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_7 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_7 * 2 + 1],
		},
	},
	{
		{
			// SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_INPUT_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_INPUT_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_INPUT_VOLT_V, //uint16_t entity_instance_number;
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
			16000, //uint32_t critical_high;
			7000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_INPUT_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V85_HAMSA_VDD_ADDR,
			.offset = PMBUS_READ_VIN,
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
			110000, //uint32_t critical_high;
			0, //uint32_t critical_low;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2 + 1],
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
			795, //uint32_t critical_high;
			705, //uint32_t critical_low;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2 + 1],
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
			113000, //uint32_t critical_high;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2 + 1],
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
			84750, //uint32_t critical_high;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P0V75_VDDPHY_HBM0246_INPUT_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_INPUT_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_INPUT_VOLT_V, //uint16_t entity_instance_number;
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
			16000, //uint32_t critical_high;
			7000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_INPUT_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V75_VDDPHY_HBM0246_ADDR,
			.offset = PMBUS_READ_VIN,
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
			110000, //uint32_t critical_high;
			0, //uint32_t critical_low;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2],
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
			440, //uint32_t critical_high;
			380, //uint32_t critical_low;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2],
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
			31000, //uint32_t critical_high;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2],
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
			12400, //uint32_t critical_high;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_11 * 2],
		},
	},
	{
		{
			// ASIC_P0V4_VDDQL_HBM0246_INPUT_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_INPUT_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_INPUT_VOLT_V, //uint16_t entity_instance_number;
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
			16000, //uint32_t critical_high;
			7000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_INPUT_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V4_VDDQL_HBM0246_ADDR,
			.offset = PMBUS_READ_VIN,
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
			// ASIC_P1V05_VDDQC_HBM0246_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P1V05_VDDQC_HBM0246_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V05_VDDQC_HBM0246_TEMP_C, //uint16_t entity_instance_number;
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
			110000, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V05_VDDQC_HBM0246_TEMP_C,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P1V05_VDDQC_HBM0246_ADDR,
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
			// ASIC_P1V05_VDDQC_HBM0246_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P1V05_VDDQC_HBM0246_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V05_VDDQC_HBM0246_VOLT_V, //uint16_t entity_instance_number;
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
			1177, //uint32_t critical_high;
			1067, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V05_VDDQC_HBM0246_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P1V05_VDDQC_HBM0246_ADDR,
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
			// ASIC_P1V05_VDDQC_HBM0246_CURR_A
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P1V05_VDDQC_HBM0246_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V05_VDDQC_HBM0246_CURR_A, //uint16_t entity_instance_number;
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
			270000, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V05_VDDQC_HBM0246_CURR_A,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P1V05_VDDQC_HBM0246_ADDR,
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
			// ASIC_P1V05_VDDQC_HBM0246_PWR_W
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P1V05_VDDQC_HBM0246_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V05_VDDQC_HBM0246_PWR_W, //uint16_t entity_instance_number;
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
			297000, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V05_VDDQC_HBM0246_PWR_W,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P1V05_VDDQC_HBM0246_ADDR,
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
			// ASIC_P1V05_VDDQC_HBM0246_INPUT_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P1V05_VDDQC_HBM0246_INPUT_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V05_VDDQC_HBM0246_INPUT_VOLT_V, //uint16_t entity_instance_number;
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
			16000, //uint32_t critical_high;
			7000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V05_VDDQC_HBM0246_INPUT_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P1V05_VDDQC_HBM0246_ADDR,
			.offset = PMBUS_READ_VIN,
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
			// ASIC_P1V05_VDDQC_HBM0246_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V9_VDDQ_HBM0246_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V9_VDDQ_HBM0246_TEMP_C, //uint16_t entity_instance_number;
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
			110000, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V9_VDDQ_HBM0246_TEMP_C,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V9_VDDQ_HBM0246_ADDR,
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
			// ASIC_P1V05_VDDQC_HBM0246_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V9_VDDQ_HBM0246_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V9_VDDQ_HBM0246_VOLT_V, //uint16_t entity_instance_number;
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
			1177, //uint32_t critical_high;
			1067, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V9_VDDQ_HBM0246_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V9_VDDQ_HBM0246_ADDR,
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
			// ASIC_P1V05_VDDQC_HBM0246_CURR_A
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V9_VDDQ_HBM0246_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V9_VDDQ_HBM0246_CURR_A, //uint16_t entity_instance_number;
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
			270000, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V9_VDDQ_HBM0246_CURR_A,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V9_VDDQ_HBM0246_ADDR,
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
			// ASIC_P1V05_VDDQC_HBM0246_PWR_W
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V9_VDDQ_HBM0246_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V9_VDDQ_HBM0246_PWR_W, //uint16_t entity_instance_number;
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
			297000, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V9_VDDQ_HBM0246_PWR_W,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V9_VDDQ_HBM0246_ADDR,
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
			// ASIC_P1V05_VDDQC_HBM0246_INPUT_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V9_VDDQ_HBM0246_INPUT_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V9_VDDQ_HBM0246_INPUT_VOLT_V, //uint16_t entity_instance_number;
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
			16000, //uint32_t critical_high;
			7000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V9_VDDQ_HBM0246_INPUT_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V9_VDDQ_HBM0246_ADDR,
			.offset = PMBUS_READ_VIN,
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
			110000, //uint32_t critical_high;
			0, //uint32_t critical_low;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_10 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_10 * 2 + 1],
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
			1950, //uint32_t critical_high;
			1746, //uint32_t critical_low;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_10 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_10 * 2 + 1],
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
			13000, //uint32_t critical_high;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_10 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_10 * 2 + 1],
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
			23400, //uint32_t critical_high;
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
			.pre_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_10 * 2 + 1],
			.post_sensor_read_hook = post_vr_read,
			.post_sensor_read_args = &vr_pre_read_args[VR_INDEX_E_10 * 2 + 1],
		},
	},
	{
		{
			// ASIC_P1V8_VPP_HBM0246_INPUT_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_INPUT_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_INPUT_VOLT_V, //uint16_t entity_instance_number;
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
			16000, //uint32_t critical_high;
			7000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_INPUT_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = ASIC_P1V8_VPP_HBM0246_ADDR,
			.offset = PMBUS_READ_VIN,
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
			110000, //uint32_t critical_high;
			0, //uint32_t critical_low;
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
			795, //uint32_t critical_high;
			705, //uint32_t critical_low;
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
			113000, //uint32_t critical_high;
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
			84750, //uint32_t critical_high;
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
			// ASIC_P0V75_VDDPHY_HBM1357_INPUT_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_INPUT_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_INPUT_VOLT_V, //uint16_t entity_instance_number;
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
			16000, //uint32_t critical_high;
			7000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_INPUT_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V75_VDDPHY_HBM1357_ADDR,
			.offset = PMBUS_READ_VIN,
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
			110000, //uint32_t critical_high;
			0, //uint32_t critical_low;
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
			440, //uint32_t critical_high;
			380, //uint32_t critical_low;
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
			31000, //uint32_t critical_high;
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
			12400, //uint32_t critical_high;
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
			// ASIC_P0V4_VDDQL_HBM1357_INPUT_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_INPUT_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_INPUT_VOLT_V, //uint16_t entity_instance_number;
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
			16000, //uint32_t critical_high;
			7000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_INPUT_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V4_VDDQL_HBM1357_ADDR,
			.offset = PMBUS_READ_VIN,
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
			// ASIC_P1V05_VDDQC_HBM1357_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P1V05_VDDQC_HBM1357_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V05_VDDQC_HBM1357_TEMP_C, //uint16_t entity_instance_number;
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
			110000, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V05_VDDQC_HBM1357_TEMP_C,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P1V05_VDDQC_HBM1357_ADDR,
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
			// ASIC_P1V05_VDDQC_HBM1357_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P1V05_VDDQC_HBM1357_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V05_VDDQC_HBM1357_VOLT_V, //uint16_t entity_instance_number;
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
			1177, //uint32_t critical_high;
			1067, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V05_VDDQC_HBM1357_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P1V05_VDDQC_HBM1357_ADDR,
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
			// ASIC_P1V05_VDDQC_HBM1357_CURR_A
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P1V05_VDDQC_HBM1357_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V05_VDDQC_HBM1357_CURR_A, //uint16_t entity_instance_number;
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
			270000, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V05_VDDQC_HBM1357_CURR_A,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P1V05_VDDQC_HBM1357_ADDR,
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
			// ASIC_P1V05_VDDQC_HBM1357_PWR_W
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P1V05_VDDQC_HBM1357_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V05_VDDQC_HBM1357_PWR_W, //uint16_t entity_instance_number;
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
			297000, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V05_VDDQC_HBM1357_PWR_W,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P1V05_VDDQC_HBM1357_ADDR,
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
			// ASIC_P1V05_VDDQC_HBM1357_INPUT_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P1V05_VDDQC_HBM1357_INPUT_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V05_VDDQC_HBM1357_INPUT_VOLT_V, //uint16_t entity_instance_number;
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
			16000, //uint32_t critical_high;
			7000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V05_VDDQC_HBM1357_INPUT_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P1V05_VDDQC_HBM1357_ADDR,
			.offset = PMBUS_READ_VIN,
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
			// ASIC_P1V05_VDDQC_HBM1357_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V9_VDDQ_HBM1357_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V9_VDDQ_HBM1357_TEMP_C, //uint16_t entity_instance_number;
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
			110000, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V9_VDDQ_HBM1357_TEMP_C,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V9_VDDQ_HBM1357_ADDR,
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
			// ASIC_P1V05_VDDQC_HBM1357_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V9_VDDQ_HBM1357_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V9_VDDQ_HBM1357_VOLT_V, //uint16_t entity_instance_number;
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
			1177, //uint32_t critical_high;
			1067, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V9_VDDQ_HBM1357_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V9_VDDQ_HBM1357_ADDR,
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
			// ASIC_P1V05_VDDQC_HBM1357_CURR_A
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V9_VDDQ_HBM1357_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V9_VDDQ_HBM1357_CURR_A, //uint16_t entity_instance_number;
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
			270000, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V9_VDDQ_HBM1357_CURR_A,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V9_VDDQ_HBM1357_ADDR,
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
			// ASIC_P1V05_VDDQC_HBM1357_PWR_W
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V9_VDDQ_HBM1357_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V9_VDDQ_HBM1357_PWR_W, //uint16_t entity_instance_number;
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
			297000, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V9_VDDQ_HBM1357_PWR_W,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V9_VDDQ_HBM1357_ADDR,
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
			// ASIC_P1V05_VDDQC_HBM1357_INPUT_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V9_VDDQ_HBM1357_INPUT_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V9_VDDQ_HBM1357_INPUT_VOLT_V, //uint16_t entity_instance_number;
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
			16000, //uint32_t critical_high;
			7000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V9_VDDQ_HBM1357_INPUT_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V9_VDDQ_HBM1357_ADDR,
			.offset = PMBUS_READ_VIN,
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
			110000, //uint32_t critical_high;
			0, //uint32_t critical_low;
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
			1950, //uint32_t critical_high;
			1746, //uint32_t critical_low;
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
			13000, //uint32_t critical_high;
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
			23400, //uint32_t critical_high;
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
	{
		{
			// ASIC_P1V8_VPP_HBM1357_INPUT_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_INPUT_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_INPUT_VOLT_V, //uint16_t entity_instance_number;
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
			16000, //uint32_t critical_high;
			7000, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_INPUT_VOLT_V,
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = ASIC_P1V8_VPP_HBM1357_ADDR,
			.offset = PMBUS_READ_VIN,
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
};

pldm_sensor_info plat_pldm_sensor_quick_vr_table[] = {
	{
		{
			// ASIC_P0V75_NUWA0_VDD_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V75_NUWA0_VDD_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_NUWA0_VDD_VOLT_V, //uint16_t entity_instance_number;
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
			918, //uint32_t critical_high;
			782, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_NUWA0_VDD_VOLT_V,
			.type = sensor_dev_mp29816a,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V75_NUWA0_VDD_ADDR,
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
			// ASIC_P0V75_NUWA0_VDD_PWR_W
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V75_NUWA0_VDD_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_NUWA0_VDD_PWR_W, //uint16_t entity_instance_number;
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
			0, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			1232500, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_NUWA0_VDD_PWR_W,
			.type = sensor_dev_mp29816a,
			.port = I2C_BUS3,
			.target_addr = ASIC_P0V75_NUWA0_VDD_ADDR,
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
			// ASIC_P0V75_NUWA1_VDD_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V75_NUWA1_VDD_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_NUWA1_VDD_VOLT_V, //uint16_t entity_instance_number;
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
			918, //uint32_t critical_high;
			782, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_NUWA1_VDD_VOLT_V,
			.type = sensor_dev_mp29816a,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V75_NUWA1_VDD_ADDR,
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
			// ASIC_P0V75_NUWA1_VDD_PWR_W
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ASIC_P0V75_NUWA1_VDD_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			SENSOR_NUM_ASIC_P0V75_NUWA1_VDD_PWR_W, //uint16_t entity_instance_number;
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
			0, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0, //uint32_t warning_high;
			0, //uint32_t warning_low;
			1232500, //uint32_t critical_high;
			0, //uint32_t critical_low;
			0, //uint32_t fatal_high;
			0, //uint32_t fatal_low;
		},
		.update_time = 0,
		{
			.num = SENSOR_NUM_ASIC_P0V75_NUWA1_VDD_PWR_W,
			.type = sensor_dev_mp29816a,
			.port = I2C_BUS2,
			.target_addr = ASIC_P0V75_NUWA1_VDD_ADDR,
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
			120000, //uint32_t critical_high;
			0, //uint32_t critical_low;
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
			.post_sensor_read_hook = post_common_sensor_read,
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
			13770, //uint32_t critical_high;
			11730, //uint32_t critical_low;
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
			.post_sensor_read_hook = post_common_sensor_read,
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
			180000, //uint32_t critical_high;
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
			.post_sensor_read_hook = post_common_sensor_read,
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
			2295000, //uint32_t critical_high;
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
			.post_sensor_read_hook = post_common_sensor_read,
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
			55557, //uint32_t critical_high;
			45620, //uint32_t critical_low;
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
			.post_sensor_read_hook = post_common_sensor_read,
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
			120000, //uint32_t critical_high;
			0, //uint32_t critical_low;
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
			.post_sensor_read_hook = post_common_sensor_read,
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
			13770, //uint32_t critical_high;
			11730, //uint32_t critical_low;
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
			.post_sensor_read_hook = post_common_sensor_read,
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
			180000, //uint32_t critical_high;
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
			.post_sensor_read_hook = post_common_sensor_read,
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
			2295000, //uint32_t critical_high;
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
			.post_sensor_read_hook = post_common_sensor_read,
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
			55557, //uint32_t critical_high;
			45620, //uint32_t critical_low;
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
			.post_sensor_read_hook = post_common_sensor_read,
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
		.sensorName = u"TOP_INLET_TEMP_C",
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
		.sensorName = u"BOT_INLET_TEMP_C",
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
		.sensorName = u"BOT_OUTLET_TEMP_C",
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
		.sensor_id = SENSOR_NUM_ASIC_NUWA0_SENSOR0_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_NUWA0_SENSOR0_TEMP_C",
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
		.sensor_id = SENSOR_NUM_ASIC_NUWA0_SENSOR1_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_NUWA0_SENSOR1_TEMP_C",
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
		.sensorName = u"ASIC_OWL_W_TEMP_C",
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
		.sensorName = u"ASIC_OWL_E_TEMP_C",
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
		.sensor_id = SENSOR_NUM_ASIC_NUWA1_SENSOR0_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_NUWA1_SENSOR0_TEMP_C",
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
		.sensor_id = SENSOR_NUM_ASIC_NUWA1_SENSOR1_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_NUWA1_SENSOR1_TEMP_C",
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
		.sensorName = u"ASIC_HAMSA_CRM_TEMP_C",
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
		.sensorName = u"ASIC_HAMSA_LS_TEMP_C",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V75_NUWA0_VDD_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V75_NUWA0_VDD_TEMP_C",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V75_NUWA0_VDD_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V75_NUWA0_VDD_VOLT_V",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V75_NUWA0_VDD_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V75_NUWA0_VDD_CURR_A",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V75_NUWA0_VDD_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V75_NUWA0_VDD_PWR_W",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V75_NUWA0_VDD_INPUT_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V75_NUWA0_VDD_INPUT_VOLT_V",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V75_NUWA1_VDD_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V75_NUWA1_VDD_TEMP_C",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V75_NUWA1_VDD_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V75_NUWA1_VDD_VOLT_V",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V75_NUWA1_VDD_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V75_NUWA1_VDD_CURR_A",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V75_NUWA1_VDD_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V75_NUWA1_VDD_PWR_W",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V75_NUWA1_VDD_INPUT_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V75_NUWA1_VDD_INPUT_VOLT_V",
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
		.sensorName = u"ASIC_P0V9_OWL_E_TRVDD_TEMP_C",
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
		.sensorName = u"ASIC_P0V9_OWL_E_TRVDD_VOLT_V",
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
		.sensorName = u"ASIC_P0V9_OWL_E_TRVDD_CURR_A",
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
		.sensorName = u"ASIC_P0V9_OWL_E_TRVDD_PWR_W",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_INPUT_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V9_OWL_E_TRVDD_INPUT_VOLT_V",
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
		.sensorName = u"ASIC_P0V75_OWL_E_TRVDD_TEMP_C",
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
		.sensorName = u"ASIC_P0V75_OWL_E_TRVDD_VOLT_V",
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
		.sensorName = u"ASIC_P0V75_OWL_E_TRVDD_CURR_A",
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
		.sensorName = u"ASIC_P0V75_OWL_E_TRVDD_PWR_W",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_INPUT_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V75_OWL_E_TRVDD_INPUT_VOLT_V",
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
		.sensorName = u"ASIC_P0V75_OWL_E_VDD_TEMP_C",
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
		.sensorName = u"ASIC_P0V75_OWL_E_VDD_VOLT_V",
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
		.sensorName = u"ASIC_P0V75_OWL_E_VDD_CURR_A",
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
		.sensorName = u"ASIC_P0V75_OWL_E_VDD_PWR_W",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_INPUT_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V75_OWL_E_VDD_INPUT_VOLT_V",
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
		.sensorName = u"ASIC_P0V9_OWL_W_TRVDD_TEMP_C",
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
		.sensorName = u"ASIC_P0V9_OWL_W_TRVDD_VOLT_V",
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
		.sensorName = u"ASIC_P0V9_OWL_W_TRVDD_CURR_A",
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
		.sensorName = u"ASIC_P0V9_OWL_W_TRVDD_PWR_W",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_INPUT_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V9_OWL_W_TRVDD_INPUT_VOLT_V",
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
		.sensorName = u"ASIC_P0V75_OWL_W_TRVDD_TEMP_C",
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
		.sensorName = u"ASIC_P0V75_OWL_W_TRVDD_VOLT_V",
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
		.sensorName = u"ASIC_P0V75_OWL_W_TRVDD_CURR_A",
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
		.sensorName = u"ASIC_P0V75_OWL_W_TRVDD_PWR_W",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V75_OWL_W_TRVDD_INPUT_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V75_OWL_W_TRVDD_INPUT_VOLT_V",
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
		.sensorName = u"ASIC_P0V75_OWL_W_VDD_TEMP_C",
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
		.sensorName = u"ASIC_P0V75_OWL_W_VDD_VOLT_V",
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
		.sensorName = u"ASIC_P0V75_OWL_W_VDD_CURR_A",
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
		.sensorName = u"ASIC_P0V75_OWL_W_VDD_PWR_W",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_INPUT_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V75_OWL_W_VDD_INPUT_VOLT_V",
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
		.sensorName = u"ASIC_P0V75_MAX_M_VDD_TEMP_C",
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
		.sensorName = u"ASIC_P0V75_MAX_M_VDD_VOLT_V",
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
		.sensorName = u"ASIC_P0V75_MAX_M_VDD_CURR_A",
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
		.sensorName = u"ASIC_P0V75_MAX_M_VDD_PWR_W",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_INPUT_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V75_MAX_M_VDD_INPUT_VOLT_V",
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
		.sensorName = u"ASIC_P0V75_MAX_N_VDD_TEMP_C",
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
		.sensorName = u"ASIC_P0V75_MAX_N_VDD_VOLT_V",
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
		.sensorName = u"ASIC_P0V75_MAX_N_VDD_CURR_A",
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
		.sensorName = u"ASIC_P0V75_MAX_N_VDD_PWR_W",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_INPUT_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V75_MAX_N_VDD_INPUT_VOLT_V",
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
		.sensorName = u"ASIC_P0V75_MAX_S_VDD_TEMP_C",
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
		.sensorName = u"ASIC_P0V75_MAX_S_VDD_VOLT_V",
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
		.sensorName = u"ASIC_P0V75_MAX_S_VDD_CURR_A",
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
		.sensorName = u"ASIC_P0V75_MAX_S_VDD_PWR_W",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_INPUT_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V75_MAX_S_VDD_INPUT_VOLT_V",
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
		.sensorName = u"ASIC_P0V8_HAMSA_AVDD_PCIE_TEMP_C",
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
		.sensorName = u"ASIC_P0V8_HAMSA_AVDD_PCIE_VOLT_V",
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
		.sensorName = u"ASIC_P0V8_HAMSA_AVDD_PCIE_CURR_A",
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
		.sensorName = u"ASIC_P0V8_HAMSA_AVDD_PCIE_PWR_W",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_INPUT_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V8_HAMSA_AVDD_PCIE_INPUT_VOLT_V",
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
		.sensorName = u"ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_TEMP_C",
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
		.sensorName = u"ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_VOLT_V",
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
		.sensorName = u"ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_CURR_A",
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
		.sensorName = u"ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_PWR_W",
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
		.sensor_id = SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_INPUT_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_INPUT_VOLT_V",
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
		.sensorName = u"ASIC_P0V85_HAMSA_VDD_TEMP_C",
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
		.sensorName = u"ASIC_P0V85_HAMSA_VDD_VOLT_V",
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
		.sensorName = u"ASIC_P0V85_HAMSA_VDD_CURR_A",
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
		.sensorName = u"ASIC_P0V85_HAMSA_VDD_PWR_W",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_INPUT_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V85_HAMSA_VDD_INPUT_VOLT_V",
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
		.sensorName = u"ASIC_P0V75_VDDPHY_HBM0246_TEMP_C",
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
		.sensorName = u"ASIC_P0V75_VDDPHY_HBM0246_VOLT_V",
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
		.sensorName = u"ASIC_P0V75_VDDPHY_HBM0246_CURR_A",
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
		.sensorName = u"ASIC_P0V75_VDDPHY_HBM0246_PWR_W",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_INPUT_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V75_VDDPHY_HBM0246_INPUT_VOLT_V",
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
		.sensorName = u"ASIC_P0V4_VDDQL_HBM0246_TEMP_C",
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
		.sensorName = u"ASIC_P0V4_VDDQL_HBM0246_VOLT_V",
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
		.sensorName = u"ASIC_P0V4_VDDQL_HBM0246_CURR_A",
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
		.sensorName = u"ASIC_P0V4_VDDQL_HBM0246_PWR_W",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_INPUT_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V4_VDDQL_HBM0246_INPUT_VOLT_V",
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
		.sensor_id = SENSOR_NUM_ASIC_P1V05_VDDQC_HBM0246_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P1V05_VDDQC_HBM0246_TEMP_C",
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
		.sensor_id = SENSOR_NUM_ASIC_P1V05_VDDQC_HBM0246_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P1V05_VDDQC_HBM0246_VOLT_V",
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
		.sensor_id = SENSOR_NUM_ASIC_P1V05_VDDQC_HBM0246_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P1V05_VDDQC_HBM0246_CURR_A",
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
		.sensor_id = SENSOR_NUM_ASIC_P1V05_VDDQC_HBM0246_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P1V05_VDDQC_HBM0246_PWR_W",
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
		.sensor_id = SENSOR_NUM_ASIC_P1V05_VDDQC_HBM0246_INPUT_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P1V05_VDDQC_HBM0246_INPUT_VOLT_V",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V9_VDDQ_HBM0246_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V9_VDDQ_HBM0246_TEMP_C",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V9_VDDQ_HBM0246_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V9_VDDQ_HBM0246_VOLT_V",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V9_VDDQ_HBM0246_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V9_VDDQ_HBM0246_CURR_A",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V9_VDDQ_HBM0246_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V9_VDDQ_HBM0246_PWR_W",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V9_VDDQ_HBM0246_INPUT_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V9_VDDQ_HBM0246_INPUT_VOLT_V",
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
		.sensorName = u"ASIC_P1V8_VPP_HBM0246_TEMP_C",
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
		.sensorName = u"ASIC_P1V8_VPP_HBM0246_VOLT_V",
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
		.sensorName = u"ASIC_P1V8_VPP_HBM0246_CURR_A",
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
		.sensorName = u"ASIC_P1V8_VPP_HBM0246_PWR_W",
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
		.sensor_id = SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_INPUT_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P1V8_VPP_HBM0246_INPUT_VOLT_V",
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
		.sensorName = u"ASIC_P0V75_VDDPHY_HBM1357_TEMP_C",
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
		.sensorName = u"ASIC_P0V75_VDDPHY_HBM1357_VOLT_V",
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
		.sensorName = u"ASIC_P0V75_VDDPHY_HBM1357_CURR_A",
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
		.sensorName = u"ASIC_P0V75_VDDPHY_HBM1357_PWR_W",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_INPUT_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V75_VDDPHY_HBM1357_INPUT_VOLT_V",
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
		.sensorName = u"ASIC_P0V4_VDDQL_HBM1357_TEMP_C",
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
		.sensorName = u"ASIC_P0V4_VDDQL_HBM1357_VOLT_V",
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
		.sensorName = u"ASIC_P0V4_VDDQL_HBM1357_CURR_A",
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
		.sensorName = u"ASIC_P0V4_VDDQL_HBM1357_PWR_W",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_INPUT_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V4_VDDQL_HBM1357_INPUT_VOLT_V",
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
		.sensor_id = SENSOR_NUM_ASIC_P1V05_VDDQC_HBM1357_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P1V05_VDDQC_HBM1357_TEMP_C",
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
		.sensor_id = SENSOR_NUM_ASIC_P1V05_VDDQC_HBM1357_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P1V05_VDDQC_HBM1357_VOLT_V",
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
		.sensor_id = SENSOR_NUM_ASIC_P1V05_VDDQC_HBM1357_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P1V05_VDDQC_HBM1357_CURR_A",
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
		.sensor_id = SENSOR_NUM_ASIC_P1V05_VDDQC_HBM1357_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P1V05_VDDQC_HBM1357_PWR_W",
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
		.sensor_id = SENSOR_NUM_ASIC_P1V05_VDDQC_HBM1357_INPUT_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P1V05_VDDQC_HBM1357_INPUT_VOLT_V",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V9_VDDQ_HBM1357_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V9_VDDQ_HBM1357_TEMP_C",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V9_VDDQ_HBM1357_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V9_VDDQ_HBM1357_VOLT_V",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V9_VDDQ_HBM1357_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V9_VDDQ_HBM1357_CURR_A",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V9_VDDQ_HBM1357_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V9_VDDQ_HBM1357_PWR_W",
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
		.sensor_id = SENSOR_NUM_ASIC_P0V9_VDDQ_HBM1357_INPUT_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P0V9_VDDQ_HBM1357_INPUT_VOLT_V",
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
		.sensorName = u"ASIC_P1V8_VPP_HBM1357_TEMP_C",
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
		.sensorName = u"ASIC_P1V8_VPP_HBM1357_VOLT_V",
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
		.sensorName = u"ASIC_P1V8_VPP_HBM1357_CURR_A",
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
		.sensorName = u"ASIC_P1V8_VPP_HBM1357_PWR_W",
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
		.sensor_id = SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_INPUT_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"ASIC_P1V8_VPP_HBM1357_INPUT_VOLT_V",
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
		.sensorName = u"UBC1_P12V_TEMP_C",
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
		.sensorName = u"UBC1_P12V_VOLT_V",
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
		.sensorName = u"UBC1_P12V_CURR_A",
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
		.sensorName = u"UBC1_P12V_PWR_W",
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
		.sensorName = u"UBC1_P52V_INPUT_VOLT_V",
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
		.sensorName = u"UBC2_P12V_TEMP_C",
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
		.sensorName = u"UBC2_P12V_VOLT_V",
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
		.sensorName = u"UBC2_P12V_CURR_A",
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
		.sensorName = u"UBC2_P12V_PWR_W",
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
		.sensorName = u"UBC2_P52V_INPUT_VOLT_V",
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
	const char16_t base_name[] = u"EL_";

	// Get slot ID
	uint8_t slot_id = get_mmc_slot();

	// Calculate the length of the base name
	size_t base_len = char16_strlen(base_name);

	// Calculate the required length for the final string (base name + null terminator)
	size_t total_len = base_len + 2; // +1 for the null terminator

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
		if ((cfg->post_sensor_read_hook)(cfg, cfg->post_sensor_read_args, 0) == false &&
		    cfg->cache_status != SENSOR_OPEN_CIRCUIT) {
			LOG_DBG("%d read raw value post hook fail! %x", sensor_id,
				cfg->cache_status);
			return false;
		}
	}

	return ret;
}

PDR_numeric_sensor *get_pdr_numeric_sensor_by_sensor_id(uint8_t sensor_id)
{
	uint8_t type = check_sensor_type(sensor_id);
	CHECK_ARG_WITH_RETURN(type == MAX_SENSOR_THREAD_ID, NULL);

	pldm_sensor_info *table = plat_pldm_sensor_load(type);
	CHECK_NULL_ARG_WITH_RETURN(table, NULL);

	int count = plat_pldm_sensor_get_sensor_count(type);
	CHECK_ARG_WITH_RETURN(count < 0, NULL);

	for (uint8_t i = 0; i < count; i++) {
		if (table[i].pldm_sensor_cfg.num == sensor_id)
			return &table[i].pdr_numeric_sensor;
	}

	return NULL;
}

#define SENSOR_CFG_NO_CHANGE 0xFF

void change_sensor_cfg(uint8_t asic_board_id, uint8_t vr_module, uint8_t ubc_module,
		       uint8_t board_rev_id)
{
	// uint8_t ubc1_change_flag = 0;
	uint8_t vr_change_mode = OLD_MPS;
	/*
	When changing the address version, you first need to check the board type (EVB or Electra), and then check the board revision ID.
	FAB2 corresponds to EVT1B
	FAB3 corresponds to EVT2
	There are two VR vendors, and their settings are different:
	RNS
	MPS
	For the two UBC chips, both need to be changed on FAB2.
	*/

	LOG_INF("asic_board_id: %d, board_rev_id: %d, vr_module: %d", asic_board_id, board_rev_id,
		vr_module);
	// VR check version
	switch (asic_board_id) {
	case ASIC_BOARD_ID_ELECTRA:
		if (vr_module == VR_MODULE_RNS) {
			LOG_WRN("change VR address to old RNS");
			vr_change_mode = OLD_RNS;
		}
		// default is old MPS so do nothing
		break;
	default:
		break;
	}
	LOG_INF("vr change mode: 0x%x", vr_change_mode);
	// vr sensor
	for (uint8_t i = VR_SENSOR_THREAD_ID; i <= QUICK_VR_SENSOR_THREAD_ID; i++) {
		if (vr_change_mode == OLD_MPS)
			continue;

		pldm_sensor_info *table = plat_pldm_sensor_load(i);
		if (table == NULL)
			return;

		int count = plat_pldm_sensor_get_sensor_count(i);
		if (count < 0)
			return;
		// change VR address
		for (uint8_t j = 0; j < count; j++) {
			if (vr_change_mode == OLD_RNS)
				table[j].pldm_sensor_cfg.type = sensor_dev_raa228249;

			table[j].pldm_sensor_cfg.target_addr = convert_vr_addr(
				table[j].pldm_sensor_cfg.target_addr, vr_change_mode);
			LOG_INF("change VR sensors 0x%x address to 0x%x",
				table[j].pldm_sensor_cfg.num, table[j].pldm_sensor_cfg.target_addr);
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

void set_plat_sensor_one_step_enable_flag(uint8_t value)
{
	plat_sensor_one_step_power_enable_flag = value;
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

uint8_t get_plat_sensor_one_step_enable_flag()
{
	return plat_sensor_one_step_power_enable_flag;
}

bool is_ubc_access(uint8_t sensor_num)
{
	if (get_plat_sensor_one_step_enable_flag() == ONE_STEP_POWER_MAGIC_NUMBER) {
		return (get_plat_sensor_ubc_polling_enable_flag() &&
			get_plat_sensor_polling_enable_flag());
	} else {
		return (is_dc_access(sensor_num) && get_plat_sensor_ubc_polling_enable_flag() &&
			get_plat_sensor_polling_enable_flag());
	}
}

bool is_temp_access(uint8_t cfg_idx)
{
	return (get_plat_sensor_temp_polling_enable_flag() &&
		get_plat_sensor_polling_enable_flag());
}

bool is_vr_access(uint8_t sensor_num)
{
	if (get_plat_sensor_one_step_enable_flag() == ONE_STEP_POWER_MAGIC_NUMBER) {
		return (get_plat_sensor_vr_polling_enable_flag() &&
			get_plat_sensor_polling_enable_flag());

	} else {
		return (is_dc_access(sensor_num) && get_plat_sensor_vr_polling_enable_flag() &&
			get_plat_sensor_polling_enable_flag());
	}
}

uint32_t plat_pldm_sensor_get_quick_vr_poll_interval()
{
	return quick_vr_poll_interval;
}

void plat_pldm_sensor_set_quick_vr_poll_interval(uint32_t value)
{
	quick_vr_poll_interval = value;
	is_quick_vr_poll_changed = true;
}

void plat_pldm_sensor_change_poll_interval(int thread_id, uint32_t *poll_interval_ms)
{
	CHECK_NULL_ARG(poll_interval_ms);

	if (thread_id == QUICK_VR_SENSOR_THREAD_ID) {
		if (is_quick_vr_poll_changed) {
			*poll_interval_ms = quick_vr_poll_interval;
			is_quick_vr_poll_changed = false;
		}
	}

	return;
}

void set_ioe_value(uint8_t ioe_addr, uint8_t ioe_reg, uint8_t value)
{
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };
	int ret = 0;
	msg.bus = I2C_BUS1;
	msg.target_addr = ioe_addr;
	msg.tx_len = 2;
	msg.data[0] = ioe_reg;
	msg.data[1] = value;

	ret = i2c_master_write(&msg, retry);

	if (ret != 0) {
		LOG_ERR("Failed to write IOE(0x%02X). The register is 0x%02X.", ioe_addr, ioe_reg);
		k_msleep(3000);
	}
}
int get_ioe_value(uint8_t ioe_addr, uint8_t ioe_reg, uint8_t *value)
{
	int ret = 0;
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	msg.bus = I2C_BUS1;
	msg.target_addr = ioe_addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = ioe_reg;

	ret = i2c_master_read(&msg, retry);

	if (ret != 0) {
		LOG_ERR("Failed to read IOE(0x%02X). The register is 0x%02X.", ioe_addr, ioe_reg);
		k_msleep(3000);
		return -1;
	}

	*value = msg.data[0];

	return 0;
}

uint8_t get_ioe_init_flag()
{
	return ioe_init_flag;
}

void set_ioe_init_flag(uint8_t flag)
{
	ioe_init_flag = flag;
}

struct k_thread quick_sensor_poll;
K_KERNEL_STACK_MEMBER(quick_sensor_poll_stack, 1024);
k_tid_t quick_sensor_tid;

void init_U200051_IO()
{
	LOG_INF("init U200051 IO expander");
	// only bit6 is input (1)
	set_ioe_value(U200051_IO_ADDR, CONFIG, 0x40);
	// io5,io7 default output 1
	set_ioe_value(U200051_IO_ADDR, OUTPUT_PORT, 0x80);
}

/* quick sensor */
void quick_sensor_poll_handler(void *arug0, void *arug1, void *arug2)
{
	k_msleep(1000); // delay 1 second to wait for drivers ready before start sensor polling
	int quick_sensor_poll_interval_ms = 30;
	int ret = 0;
	uint8_t leak_2_value = 0;
	uint8_t set_io7_value = 0;
	uint8_t log_show_flag = 0;
	while (1) {
		//check dc on/off and polling enable/disable
		if (is_mb_dc_on() == false || !get_plat_sensor_polling_enable_flag()) {
			//dc is off, sleep 1 second
			k_msleep(1000);
			continue;
		}

		if (!get_ioe_init_flag()) {
			LOG_INF("U200051 IO expander need init");
			init_U200051_IO();
			set_ioe_init_flag(1);
		}

		k_msleep(quick_sensor_poll_interval_ms);
		// read mux U200051 IO_6, if change means leak detected set io_7 to 1
		ret = get_ioe_value(U200051_IO_ADDR, INPUT_PORT, &leak_2_value);

		if (ret != 0) {
			LOG_ERR("Failed to read IOE(0x%02X). The register is 0x%02X.",
				U200051_IO_ADDR, OUTPUT_PORT);
			continue;
		}

		if (leak_2_value & 0x40) {
			if (log_show_flag == 0) {
				LOG_WRN("leak_2 detected");
				get_ioe_value(U200051_IO_ADDR, OUTPUT_PORT, &set_io7_value);
				//inverse bit-7
				set_io7_value ^= 0x80;
				set_ioe_value(U200051_IO_ADDR, OUTPUT_PORT, set_io7_value);
				log_show_flag = 1;
			}
		} else {
			log_show_flag = 0;
		}
	}
}

void quick_sensor_poll_init()
{
	quick_sensor_tid = k_thread_create(&quick_sensor_poll, quick_sensor_poll_stack,
					   K_THREAD_STACK_SIZEOF(quick_sensor_poll_stack),
					   quick_sensor_poll_handler, NULL, NULL, NULL,
					   CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&quick_sensor_poll, "quick_sensor_poll");
	return;
}
