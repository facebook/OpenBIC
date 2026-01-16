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

#include <kernel.h>
#include <stdlib.h>
#include <logging/log.h>
#include <libutil.h>
#include "plat_sensor_table.h"
#include "fru.h"
#include "plat_fru.h"
#include "plat_i2c.h"
#include "plat_log.h"
#include "plat_cpld.h"
#include "plat_hook.h"
#include "plat_class.h"
#include "plat_pldm_sensor.h"

LOG_MODULE_REGISTER(plat_log);

#define LOG_MAX_INDEX 0x0FFF // recount when log index > 0x0FFF
#define LOG_MAX_NUM 100 // total log amount: 100
#define FRU_LOG_START 0x0000 // log offset: 0KB
#define EEPROM_MAX_WRITE_TIME 5 // the BR24G512 eeprom max write time is 3.5 ms
#define CPLD_VR_VENDOR_TYPE_REG 0x1C
#define ERROR_CODE_TYPE_SHIFT 13
#define SENSOR_NUMBER_DONT_CARE 0xFF

static plat_err_log_mapping err_log_data[LOG_MAX_NUM];
static uint16_t err_code_caches[200]; //extend if error code types > 200
static uint16_t next_log_position = 0; // Next position to write in the eeprom, 1-based, defaut 0
static uint16_t next_index = 0; // Next global index to use for logs, 1-based, defaut 0
static uint8_t log_num; // Number of logs in EEPROM

typedef struct _vr_device_match_sensor_num {
	uint8_t index;
	uint8_t sensor_num_1;
} vr_device_match_sensor_num;

vr_device_match_sensor_num vr_error_fault_table[] = {
	// follow VR_PWR_LOG_DEVICE_INDEX_E
	// //pwr fault reg 1
	{ PWRGD_OWL_E_TRVDD0P9_R_FAULT, SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_TEMP_C },
	{ PWRGD_OWL_W_TRVDD0P9_R_FAULT, SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_TEMP_C },
	{ PWRGD_OWL_E_TRVDD0P75_R_FAULT, SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_TEMP_C },
	{ PWRGD_OWL_W_TRVDD0P75_R_FAULT, SENSOR_NUM_ASIC_P0V75_OWL_W_TRVDD_TEMP_C },
	{ PWRGD_HAMSA_AVDD_PCIE_R_FAULT, SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_TEMP_C },
	{ PWRGD_HAMSA_VDDHRXTX_PCIE_R_FAULT, SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_TEMP_C },
	{ PWRGD_P4V2_R_FAULT, 0 }, // TODO
	{ PWRGD_P0V75_AVDD_HCSL_R_FAULT, 0 }, // TODO
	//pwr fault reg 2
	{ PWRGD_MEDHA1_VDD_FAULT, SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_TEMP_C },
	{ PWRGD_MEDHA0_VDD_FAULT, SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_TEMP_C },
	{ PWRGD_OWL_E_VDD_R_FAULT, SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_TEMP_C },
	{ PWRGD_OWL_W_VDD_R_FAULT, SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_TEMP_C },
	{ PWRGD_HAMSA_VDD_R_FAULT, SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_TEMP_C },
	{ PWRGD_MAX_S_VDD_R_FAULT, SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_TEMP_C },
	{ PWRGD_MAX_M_VDD_R_FAULT, SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_TEMP_C },
	{ PWRGD_MAX_N_VDD_R_FAULT, SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_TEMP_C },
	//pwr fault reg 3
	{ PWRGD_VDDQL_HBM0_HBM2_HBM4_HBM6_R_FAULT, SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_TEMP_C },
	{ PWRGD_VDDQC_HBM0_HBM2_HBM4_HBM6_R_FAULT, SENSOR_NUM_ASIC_P1V1_VDDQC_HBM0246_TEMP_C },
	{ PWRGD_VPP_HBM0_HBM2_HBM4_HBM6_R_FAULT, SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_TEMP_C },
	{ PWRGD_VDDPHY_HBM0_HBM2_HBM4_HBM6_R_FAULT, SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_TEMP_C },
	{ PWRGD_VDDQL_HBM1_HBM3_HBM5_HBM7_R_FAULT, SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_TEMP_C },
	{ PWRGD_VDDQC_HBM1_HBM3_HBM5_HBM7_R_FAULT, SENSOR_NUM_ASIC_P1V1_VDDQC_HBM1357_TEMP_C },
	{ PWRGD_VPP_HBM1_HBM3_HBM5_HBM7_R_FAULT, SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_TEMP_C },
	{ PWRGD_VDDPHY_HBM1_HBM3_HBM5_HBM7_R_FAULT, SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_TEMP_C },
	//pwr fault reg 4
	{ PWRGD_PLL_VDDA15_HBM0_HBM2_FAULT, 0 }, // TODO
	{ PWRGD_PLL_VDDA15_HBM1_HBM3_FAULT, 0 }, // TODO
	{ PWRGD_PLL_VDDA15_HBM4_HBM6_FAULT, 0 }, // TODO
	{ PWRGD_PLL_VDDA15_HBM5_HBM7_FAULT, 0 }, // TODO
	{ PWRGD_P0V9_OWL_E_PVDD_FAULT, 0 }, // TODO
	{ PWRGD_P0V9_OWL_W_PVDD_FAULT, 0 }, // TODO
	{ PWRGD_P1V5_E_RVDD_FAULT, 0 }, // TODO
	{ PWRGD_P1V5_W_RVDD_FAULT, 0 }, // TODO
	//pwr fault reg 5
	{ P12V_UBC_PWRGD_FAULT, 0 }, // TODO
	{ PWRGD_P5V_R_FAULT, 0 }, // TODO
	{ PWRGD_P3V3_R_FAULT, 0 }, // TODO
	{ PWRGD_P1V8_R_FAULT, 0 }, // TODO
	{ PWRGD_LDO_IN_1V2_R_FAULT, 0 }, // TODO
	{ PWRGD_P1V5_PLL_VDDA_OWL_FAULT, 0 }, // TODO
	{ PWRGD_P1V5_PLL_VDDA_SOC_FAULT, 0 }, // TODO
	{ PWRGD_PVDD1P5_FAULT, 0 }, // TODO
	// don't care of sensor num
	{ VR_ERR_DEVICE_DONT_CARE, SENSOR_NUMBER_DONT_CARE },
};

typedef struct _vr_error_callback_info_ {
	uint8_t cpld_offset;
	uint8_t bit_mapping_vr_sensor_num[8];
} vr_error_callback_info;

vr_error_callback_info vr_error_callback_info_table[] = {
	// cpld_offset, reading mask, bit_mapping_vr_sensor_num
	{ VR_POWER_FAULT_1_REG,
	  {
		  PWRGD_P0V75_AVDD_HCSL_R_FAULT, // bit0
		  PWRGD_P4V2_R_FAULT, // bit1
		  PWRGD_HAMSA_VDDHRXTX_PCIE_R_FAULT, // bit2
		  PWRGD_HAMSA_AVDD_PCIE_R_FAULT, // bit3
		  PWRGD_OWL_W_TRVDD0P75_R_FAULT, // bit4
		  PWRGD_OWL_E_TRVDD0P75_R_FAULT, // bit5
		  PWRGD_OWL_W_TRVDD0P9_R_FAULT, // bit6
		  PWRGD_OWL_E_TRVDD0P9_R_FAULT // bit7
	  } },
	{ VR_POWER_FAULT_2_REG,
	  {
		  PWRGD_MAX_N_VDD_R_FAULT, // bit0
		  PWRGD_MAX_M_VDD_R_FAULT, // bit1
		  PWRGD_MAX_S_VDD_R_FAULT, // bit2
		  PWRGD_HAMSA_VDD_R_FAULT, // bit3
		  PWRGD_OWL_W_VDD_R_FAULT, // bit4
		  PWRGD_OWL_E_VDD_R_FAULT, // bit5
		  PWRGD_MEDHA0_VDD_FAULT, // bit6
		  PWRGD_MEDHA1_VDD_FAULT // bit7
	  } },
	{ VR_POWER_FAULT_3_REG,
	  {
		  PWRGD_VDDPHY_HBM1_HBM3_HBM5_HBM7_R_FAULT, // bit0
		  PWRGD_VPP_HBM1_HBM3_HBM5_HBM7_R_FAULT, // bit1
		  PWRGD_VDDQC_HBM1_HBM3_HBM5_HBM7_R_FAULT, // bit2
		  PWRGD_VDDQL_HBM1_HBM3_HBM5_HBM7_R_FAULT, // bit3
		  PWRGD_VDDPHY_HBM0_HBM2_HBM4_HBM6_R_FAULT, // bit4
		  PWRGD_VPP_HBM0_HBM2_HBM4_HBM6_R_FAULT, // bit5
		  PWRGD_VDDQC_HBM0_HBM2_HBM4_HBM6_R_FAULT, // bit6
		  PWRGD_VDDQL_HBM0_HBM2_HBM4_HBM6_R_FAULT // bit7
	  } },
	{ VR_POWER_FAULT_4_REG,
	  {
		  PWRGD_P1V5_W_RVDD_FAULT, // bit0
		  PWRGD_P1V5_E_RVDD_FAULT, // bit1
		  PWRGD_P0V9_OWL_W_PVDD_FAULT, // bit2
		  PWRGD_P0V9_OWL_E_PVDD_FAULT, // bit3
		  PWRGD_PLL_VDDA15_HBM5_HBM7_FAULT, // bit4
		  PWRGD_PLL_VDDA15_HBM4_HBM6_FAULT, // bit5
		  PWRGD_PLL_VDDA15_HBM1_HBM3_FAULT, // bit6
		  PWRGD_PLL_VDDA15_HBM0_HBM2_FAULT // bit7
	  } }, // to_do not sure
	{ VR_POWER_FAULT_5_REG,
	  {
		  PWRGD_PVDD1P5_FAULT, // bit0
		  PWRGD_P1V5_PLL_VDDA_SOC_FAULT, // bit1
		  PWRGD_P1V5_PLL_VDDA_OWL_FAULT, // bit2
		  PWRGD_LDO_IN_1V2_R_FAULT, // bit3
		  PWRGD_P1V8_R_FAULT, // bit4
		  PWRGD_P3V3_R_FAULT, // bit5
		  PWRGD_P5V_R_FAULT, // bit6
		  P12V_UBC_PWRGD_FAULT // bit7
	  } }, // to_do not sure
	{ VR_SMBUS_ALERT_EVENT_LOG_REG,
	  { 0x00, VR_ERR_DEVICE_DONT_CARE, VR_ERR_DEVICE_DONT_CARE, VR_ERR_DEVICE_DONT_CARE,
	    VR_ERR_DEVICE_DONT_CARE, VR_ERR_DEVICE_DONT_CARE, VR_ERR_DEVICE_DONT_CARE,
	    VR_ERR_DEVICE_DONT_CARE } },

};
typedef struct vr_smbus_alrt_sensor_map {
	uint8_t bit_number;
	uint8_t vr_rail_1_page_0;
	uint8_t vr_rail_1_page_1;
	uint8_t vr_rail_2_page_0;
	uint8_t vr_rail_2_page_1;
} vr_smbus_alrt_sensor_map;

vr_smbus_alrt_sensor_map vr_smbus_alrt_sensor_map_table[] = {
	{ 0 },
	{ 1, VR_RAIL_E_ASIC_P0V75_MAX_N_VDD, VR_RAIL_E_ASIC_P0V8_HAMSA_AVDD_PCIE,
	  VR_RAIL_E_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE, VR_RAIL_E_ASIC_P0V85_HAMSA_VDD },
	{ 2, VR_RAIL_E_ASIC_P1V1_VDDQC_HBM0246, VR_RAIL_E_ASIC_P1V8_VPP_HBM0246,
	  VR_RAIL_E_ASIC_P0V4_VDDQL_HBM0246, VR_RAIL_E_ASIC_P0V75_VDDPHY_HBM0246 },
	{ 3, VR_RAIL_E_ASIC_P0V75_MAX_M_VDD, VR_RAIL_E_ASIC_P0V75_VDDPHY_HBM1357,
	  VR_RAIL_E_ASIC_P1V1_VDDQC_HBM1357, VR_RAIL_E_ASIC_P1V8_VPP_HBM1357 },
	{
		4,
		VR_RAIL_E_ASIC_P0V9_OWL_W_TRVDD,
		VR_RAIL_E_ASIC_P0V75_OWL_W_TRVDD,
		VR_RAIL_E_ASIC_P0V75_OWL_W_VDD,
		VR_RAIL_E_ASIC_P0V75_MAX_S_VDD,
	},
	{ 5, VR_RAIL_E_ASIC_P0V9_OWL_E_TRVDD, VR_RAIL_E_ASIC_P0V75_OWL_E_TRVDD,
	  VR_RAIL_E_ASIC_P0V75_OWL_E_VDD, VR_RAIL_E_ASIC_P0V4_VDDQL_HBM1357 },
	{ 6, VR_RAIL_E_ASIC_P0V85_MEDHA1_VDD },
	{ 7, VR_RAIL_E_ASIC_P0V85_MEDHA0_VDD },
};

void plat_log_read(uint8_t *log_data, uint8_t cmd_size, uint16_t order)
{
	CHECK_NULL_ARG(log_data);

	// Calculate the target log position based on next_log_position
	uint16_t zero_base_log_position =
		((next_log_position - 1) + LOG_MAX_NUM - order) % LOG_MAX_NUM;

	uint16_t eeprom_address =
		FRU_LOG_START + zero_base_log_position * sizeof(plat_err_log_mapping);

	LOG_DBG("order: %d, log_position: %d, eeprom_address: 0x%X", order,
		(zero_base_log_position + 1),
		eeprom_address); //remove after all log function is ready

	plat_err_log_mapping log_entry;

	if (!plat_eeprom_read(eeprom_address, (uint8_t *)&log_entry,
			      sizeof(plat_err_log_mapping))) {
		LOG_ERR("Failed to read log from EEPROM at position %d (address: 0x%X)", order,
			eeprom_address);
		memset(log_data, 0x00, cmd_size);
		return;
	}

	memcpy(log_data, &log_entry, cmd_size);

	const plat_err_log_mapping *p = (plat_err_log_mapping *)log_data;

	LOG_HEXDUMP_DBG(log_data, cmd_size, "plat_log_read_before");

	if (p->index == 0xFFFF) {
		memset(log_data, 0x00, cmd_size);
	}

	LOG_HEXDUMP_DBG(log_data, cmd_size, "plat_log_read_after");
}

// Clear logs from memory and EEPROM with error handling
void plat_clear_log()
{
	memset(err_log_data, 0xFF, sizeof(err_log_data));
	memset(err_code_caches, 0, sizeof(err_code_caches));

	for (uint8_t i = 0; i < LOG_MAX_NUM; i++) {
		if (!plat_eeprom_write(FRU_LOG_START + sizeof(plat_err_log_mapping) * i,
				       (uint8_t *)err_log_data, sizeof(plat_err_log_mapping))) {
			LOG_ERR("Clear EEPROM Log failed at index %d", i);
		}
		k_msleep(EEPROM_MAX_WRITE_TIME);
	}
	log_num = 0;
	next_index = 0;
}

bool vr_fault_get_error_data(uint8_t sensor_id, uint8_t *data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	// vr status word
	return get_raw_data_from_sensor_id(sensor_id, 0x79, data, 2);
}

bool get_multi_vr_status(uint8_t alrt_index, uint8_t *data)
{
	const vr_smbus_alrt_sensor_map *entry = &vr_smbus_alrt_sensor_map_table[alrt_index];
	uint16_t vr_data;
	uint8_t *ptr = data;

	// Collect all VR rails from this table entry
	uint8_t vr_list[4] = {
		entry->vr_rail_1_page_0,
		entry->vr_rail_1_page_1,
		entry->vr_rail_2_page_0,
		entry->vr_rail_2_page_1,
	};

	for (int i = 0; i < 4; i++) {
		if (vr_list[i] == 0)
			continue; // Skip empty entries

		if (!plat_get_vr_status(vr_list[i], VR_STAUS_E_STATUS_WORD, &vr_data)) {
			LOG_ERR("SMBus alert: Failed to get VR[%d] status word", vr_list[i]);
			return false;
		}
		LOG_INF("VR[%d] status word: 0x%x", vr_list[i], vr_data);
		// Write each uint16_t value into the output buffer (little-endian)
		*ptr++ = (uint8_t)(vr_data & 0xFF);
		*ptr++ = (uint8_t)((vr_data >> 8) & 0xFF);
	}

	// print VR data
	for (int i = 0; i < 8; i++)
		LOG_DBG("data[%d]: %02X ", i, data[i]);

	return true;
}

bool get_error_data(uint16_t error_code, uint8_t *data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);
	//  temperature error code
	uint8_t trigger_case = (error_code >> 13) & 0x07;
	if (trigger_case == TEMPERATURE_TRIGGER_CAUSE) {
		uint8_t temperature_sensoor_num = error_code & 0xFF;
		LOG_WRN("trigger_case: 0x%x, temperature_sensoor_num: 0x%x", trigger_case,
			temperature_sensoor_num);
		if (!get_raw_data_from_sensor_id(temperature_sensoor_num, 0x02, data, 1)) {
			LOG_ERR("Failed to get temperature data");
			return false;
		};
		// save sensor num to data and keep raw data
		data[1] = temperature_sensoor_num;
		LOG_INF("Temperature status: 0x%x, sensor num: 0x%x", data[0], data[1]);
		return true;
	}

	// Extract CPLD offset and bit position from the error code
	uint8_t cpld_offset = error_code & 0xFF;
	uint8_t bit_position = (error_code >> 8) & 0x07;
	LOG_WRN("cpld_offset: 0x%x, bit_position: 0x%x", cpld_offset, bit_position);

	// Initialize sensor number
	uint8_t sensor_num = 0x00;
	uint8_t device_id = 0x00;
	uint8_t smbus_alrt_index = bit_position;

	// Find the device_id associated with the error code
	for (size_t i = 0; i < ARRAY_SIZE(vr_error_callback_info_table); i++) {
		if (vr_error_callback_info_table[i].cpld_offset == cpld_offset) {
			device_id = vr_error_callback_info_table[i]
					    .bit_mapping_vr_sensor_num[bit_position];
			LOG_DBG("offset: 0x%x found", cpld_offset);
			LOG_DBG("device_id: 0x%x found", device_id);
			break;
		}
	}

	if (device_id == 0x00) {
		LOG_WRN("No valid device_id for error_code: 0x%x", error_code);
		return false;
	}

	// Find the sensor number associated with the device_id
	for (size_t i = 0; i < ARRAY_SIZE(vr_error_fault_table); i++) {
		if (vr_error_fault_table[i].index == device_id) {
			sensor_num = vr_error_fault_table[i].sensor_num_1;
			break;
		}
	}

	// If no valid sensor number is found, skip further data retrieval
	if (sensor_num == 0x00) {
		LOG_ERR("No valid sensor_num for error_code: 0x%x", error_code);
		return false;
	}

	if (sensor_num == SENSOR_NUMBER_DONT_CARE)
		LOG_WRN("error_code: 0x%x, no need to get sensor_num", error_code);

	if (cpld_offset == VR_SMBUS_ALERT_EVENT_LOG_REG) {
		// smbalrt status some bits will include 2 different VRs(each VR has 2 pages so total 8 Bytes)
		// Handle VR_FAULT_ASSERT errors and retrieve VR-specific data
		if (smbus_alrt_index < ARRAY_SIZE(vr_smbus_alrt_sensor_map_table)) {
			if (!get_multi_vr_status(smbus_alrt_index, data)) {
				LOG_ERR("Failed to retrieve VR error data for smbus alrt index: 0x%x",
					smbus_alrt_index);
				return false;
			}
		} else {
			LOG_ERR("smbus alrt index: 0x%x out of range", smbus_alrt_index);
			return false;
		}
	} else {
		// Handle VR_FAULT_ASSERT errors and retrieve VR-specific data
		if (!vr_fault_get_error_data(sensor_num, data)) {
			LOG_ERR("Failed to retrieve VR fault data for sensor_num: 0x%x",
				sensor_num);
			return false;
		}
	}

	return true;
}

// Handle error log events and record them if necessary
void error_log_event(uint16_t error_code, bool log_status)
{
	bool log_todo = false;
	// if error_code is not temperature error
	if (((error_code >> 13) & 0x07) != TEMPERATURE_TRIGGER_CAUSE) {
		// Check if the error_code is already logged
		for (uint8_t i = 1; i < ARRAY_SIZE(err_code_caches); i++) {
			if (err_code_caches[i] == error_code) {
				if (log_status == LOG_ASSERT) {
					log_todo = false; // Duplicate error, no need to log again
					LOG_INF("Duplicate error_code: 0x%x, log_status: %d",
						error_code, log_status);
					return;
				} else if (log_status == LOG_DEASSERT) {
					log_todo = true; // The error needs to be cleared
					err_code_caches[i] =
						0; // Remove the error code from the cache
					LOG_INF("Duplicate error_code: 0x%x, log_status: %d",
						error_code, log_status);
					return;
				}
			}
		}
	}

	// If the error_code is new and it's a LOG_ASSERT, add it to the cache
	if (!log_todo && (log_status == LOG_ASSERT)) {
		for (uint8_t i = 1; i < ARRAY_SIZE(err_code_caches); i++) {
			if (err_code_caches[i] == 0) {
				err_code_caches[i] =
					error_code; // Add the new error code to the cache
				log_todo = true;
				break;
			}
		}
	}

	// If no action is needed, exit the function
	if (!log_todo) {
		LOG_INF("Duplicate or no log needed for error_code: 0x%x", error_code);
		return;
	}

	uint16_t fru_count = next_log_position;

	// Update the log entry's index
	err_log_data[fru_count].index = next_index;
	next_index = (next_index % LOG_MAX_INDEX) + 1;

	// Update log error code and timestamp
	err_log_data[fru_count].err_code = error_code;
	err_log_data[fru_count].sys_time = k_uptime_get();

	if (!get_error_data(error_code, err_log_data[fru_count].error_data)) {
		// Clear error data if no valid data is found
		memset(err_log_data[fru_count].error_data, 0,
		       sizeof(err_log_data[fru_count].error_data));
	}

	if (!plat_read_cpld(CPLD_REGISTER_1ST_PART_START_OFFSET, err_log_data[fru_count].cpld_dump,
			    CPLD_REGISTER_1ST_PART_NUM)) {
		LOG_ERR("Failed to dump 1st part CPLD data");
	}

	//dump err_log_data for debug
	LOG_HEXDUMP_DBG(&err_log_data[fru_count], sizeof(plat_err_log_mapping), "err_log_data");

	// 1 base fru_count, write_address is 0 base
	uint16_t write_address = FRU_LOG_START + (fru_count - 1) * sizeof(plat_err_log_mapping);

	// Write log to EEPROM with error handling
	if (!plat_eeprom_write(write_address, (uint8_t *)&err_log_data[fru_count],
			       sizeof(plat_err_log_mapping))) {
		LOG_ERR("Write Log failed with Error code: %02x", error_code);
	} else {
		k_msleep(EEPROM_MAX_WRITE_TIME); // wait 5ms to write EEPROM
	}

	// Update the next log position
	next_log_position = (fru_count % LOG_MAX_NUM) + 1;
	log_num++;

	if (log_num > LOG_MAX_NUM) {
		log_num = LOG_MAX_NUM;
	}
}

void reset_error_log_event(uint8_t err_type)
{
	// Remove and DEASSERT error logs starting with the err_type
	for (uint8_t i = 1; i < ARRAY_SIZE(err_code_caches); i++) {
		uint16_t error_code = err_code_caches[i];
		uint8_t code_type = error_code >> ERROR_CODE_TYPE_SHIFT;
		if (code_type == err_type) {
			LOG_DBG("DEASSERT");
			error_log_event(error_code, LOG_DEASSERT);
			err_code_caches[i] = 0;
		}
	}
}

uint8_t plat_log_get_num(void)
{
	return log_num;
}

void find_last_log_position()
{
	uint16_t max_index = 0; // Highest valid index found
	uint16_t last_position = 0; // Position of the highest valid index
	bool all_empty = true; // Flag to detect if all entries are empty
	plat_err_log_mapping log_entry;

	for (uint16_t i = 0; i < LOG_MAX_NUM; i++) {
		uint16_t eeprom_address = FRU_LOG_START + i * sizeof(plat_err_log_mapping);

		if (!plat_eeprom_read(eeprom_address, (uint8_t *)&log_entry,
				      sizeof(plat_err_log_mapping))) {
			LOG_ERR("Failed to read log at position %d (address: 0x%X)", i,
				eeprom_address);
			continue;
		}

		// Check if the entry is valid
		if (log_entry.index != 0xFFFF && log_entry.index <= LOG_MAX_INDEX) {
			all_empty = false; // At least one entry is valid
			log_num++;
			if (log_entry.index > max_index) {
				max_index = log_entry.index; // Update max index
				last_position = i + 1; // Update last position, 1 base
			}
		}
	}

	// All entries are empty
	if (all_empty) {
		LOG_INF("All entries are empty. Initializing next_log_position and next_index to 1.");
		next_log_position = 1;
		next_index = 1;
		return;
	}

	next_log_position = (last_position % LOG_MAX_NUM) + 1;
	next_index = (max_index % LOG_MAX_INDEX) + 1;
	LOG_INF("Next log position: %d, next index: %d", next_log_position, next_index);
}

// Load logs from EEPROM into memory during initialization
void init_load_eeprom_log(void)
{
	memset(err_log_data, 0xFF, sizeof(err_log_data));
	uint16_t log_len = sizeof(plat_err_log_mapping);
	for (uint8_t i = 0; i < LOG_MAX_NUM; i++) {
		if (!plat_eeprom_read(FRU_LOG_START + i * log_len, (uint8_t *)&err_log_data[i],
				      log_len)) {
			LOG_ERR("READ Event %d failed from EEPROM", i + 1);
		}
	}

	// Determine the next log position
	find_last_log_position();
}

bool check_temp_status_bit(uint8_t bit_num)
{
	const vr_smbus_alrt_sensor_map *entry = &vr_smbus_alrt_sensor_map_table[bit_num];
	uint16_t status_word;
	uint8_t vr_sensor_num_list[4] = {
		entry->vr_rail_1_page_0,
		entry->vr_rail_1_page_1,
		entry->vr_rail_2_page_0,
		entry->vr_rail_2_page_1,
	};
	for (int i = 0; i < 4; i++) {
		if (vr_sensor_num_list[i] == 0)
			continue; // Skip empty entries

		if (!plat_get_vr_status(vr_sensor_num_list[i], VR_STAUS_E_STATUS_WORD,
					&status_word)) {
			LOG_ERR("SMBus alert: Failed to get VR[%d] status word",
				vr_sensor_num_list[i]);
		}

		// check bit-2 is 1 or not
		if ((status_word >> 2) & 0x01)
			return false;
	}

	return true;
}