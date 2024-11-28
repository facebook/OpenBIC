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
#include "plat_event.h"
#include "plat_hook.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_log);

#define LOG_MAX_INDEX 0x0FFF // recount when log index > 0x0FFF
#define LOG_MAX_NUM 100 // total log amount: 100
#define AEGIS_FRU_LOG_START 0x0000 // log offset: 0KB
#define AEGIS_CPLD_REGISTER_START_OFFSET 0x00
#define AEGIS_CPLD_REGISTER_MAX_OFFSET 0x3C
#define EEPROM_MAX_WRITE_TIME 5 // the BR24G512 eeprom max write time is 3.5 ms
#define AEGIS_CPLD_ADDR (0x4C >> 1)
#define I2C_BUS_CPLD I2C_BUS5
#define AEGIS_CPLD_VR_VENDOR_TYPE_REG 0x1C

static plat_err_log_mapping err_log_data[LOG_MAX_NUM];
static uint16_t err_code_caches[200]; //extend if error code types > 200
static uint16_t next_log_position = 0; // Next position to write in the eeprom, 1-based, defaut 0
static uint16_t next_index = 0; // Next global index to use for logs, 1-based, defaut 0

enum VR_UBC_INDEX_E {
	UBC_1 = 1,
	UBC_2,
	VR_1,
	VR_2,
	VR_3,
	VR_4,
	VR_5,
	VR_6,
	VR_7,
	VR_8,
	VR_9,
	VR_10,
	VR_11,
	VR_MAX,
};

typedef struct _vr_ubc_device_table_ {
	uint8_t index;
	uint8_t sensor_num_1;
	uint8_t sensor_num_2;
} vr_ubc_device_table;

vr_ubc_device_table vr_device_table[] = {
	{ UBC_1, SENSOR_NUM_UBC_1_TEMP_C },
	{ UBC_2, SENSOR_NUM_UBC_2_TEMP_C },
	{ VR_1, SENSOR_NUM_OSFP_P3V3_TEMP_C },
	{ VR_2, SENSOR_NUM_CPU_P0V85_PVDD_TEMP_C },
	{ VR_3, SENSOR_NUM_CPU_P0V75_PVDD_CH_N_TEMP_C, SENSOR_NUM_CPU_P0V75_MAX_PHY_N_TEMP_C },
	{ VR_4, SENSOR_NUM_CPU_P0V75_PVDD_CH_S_TEMP_C, SENSOR_NUM_CPU_P0V75_MAX_PHY_S_TEMP_C },
	{ VR_5, SENSOR_NUM_CPU_P0V75_TRVDD_ZONEA_TEMP_C, SENSOR_NUM_CPU_P1V8_VPP_HBM0_2_4_TEMP_C },
	{ VR_6, SENSOR_NUM_CPU_P0V75_TRVDD_ZONEB_TEMP_C,
	  SENSOR_NUM_CPU_P0V4_VDDQL_HBM0_2_4_TEMP_C },
	{ VR_7, SENSOR_NUM_CPU_P1V1_VDDC_HBM0_2_4_TEMP_C,
	  SENSOR_NUM_CPU_P0V75_VDDPHY_HBM0_2_4_TEMP_C },
	{ VR_8, SENSOR_NUM_CPU_P0V9_TRVDD_ZONEA_TEMP_C, SENSOR_NUM_CPU_P1V8_VPP_HBM1_3_5_TEMP_C },
	{ VR_9, SENSOR_NUM_CPU_P0V9_TRVDD_ZONEB_TEMP_C, SENSOR_NUM_CPU_P0V4_VDDQL_HBM1_3_5_TEMP_C },
	{ VR_10, SENSOR_NUM_CPU_P1V1_VDDC_HBM1_3_5_TEMP_C,
	  SENSOR_NUM_CPU_P0V75_VDDPHY_HBM1_3_5_TEMP_C },
	{ VR_11, SENSOR_NUM_CPU_P0V8_VDDA_PCIE_TEMP_C, SENSOR_NUM_CPU_P1V2_VDDHTX_PCIE_TEMP_C },
};

typedef struct _vr_error_callback_info_ {
	uint8_t cpld_offset;
	uint8_t vr_status_word_access_map;
	uint8_t bit_mapping_vr_sensor_num[8];
} vr_error_callback_info;

vr_error_callback_info vr_error_callback_info_table[] = {
	{ VR_POWER_FAULT_1_REG, 0x7E, { 0x00, VR_5, VR_6, UBC_2, UBC_1, VR_4, VR_3, 0x00 } },
	{ VR_POWER_FAULT_2_REG, 0xDF, { VR_10, VR_7, VR_8, VR_5, VR_11, 0x00, VR_8, VR_9 } },
	{ VR_POWER_FAULT_3_REG, 0xD7, { VR_4, VR_3, VR_10, 0x00, VR_7, 0x00, VR_9, VR_6 } },
	{ VR_POWER_FAULT_4_REG, 0x80, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, VR_2 } },
	{ VR_POWER_FAULT_5_REG, 0x48, { 0x00, 0x00, 0x00, VR_1, 0x00, 0x00, VR_11, 0x00 } },
	{ VR_SMBUS_ALERT_1_REG, 0xFF, { VR_1, VR_10, VR_7, VR_8, VR_9, VR_2, VR_4, VR_3 } },
	{ VR_SMBUS_ALERT_2_REG, 0xF8, { 0x00, 0x00, 0x00, VR_11, VR_5, VR_6, UBC_1, UBC_2 } },
	{ ASIC_OC_WARN_REG, 0x00, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
	{ SYSTEM_ALERT_FAULT_REG, 0x00, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
	{ VR_HOT_FAULT_1_REG, 0xFF, { VR_10, VR_7, VR_6, VR_4, VR_5, VR_3, VR_9, VR_8 } },
	{ VR_HOT_FAULT_2_REG, 0xC0, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, VR_11, VR_1 } },
	{ TEMPERATURE_IC_OVERT_FAULT_REG, 0x00, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
	{ VR_POWER_INPUT_FAULT_1_REG, 0xFF, { VR_10, VR_7, VR_6, VR_4, VR_5, VR_3, VR_9, VR_8 } },
	{ VR_POWER_INPUT_FAULT_2_REG, 0xC0, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, VR_11, VR_1 } },
	{ LEAK_DETCTION_REG, 0x00, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
};

void plat_log_read(uint8_t *log_data, uint8_t cmd_size, uint16_t order)
{
	CHECK_NULL_ARG(log_data);

	// Calculate the target log position based on next_log_position
	uint16_t zero_base_log_position =
		((next_log_position - 1) + LOG_MAX_NUM - order) % LOG_MAX_NUM;

	uint16_t eeprom_address =
		AEGIS_FRU_LOG_START + zero_base_log_position * sizeof(plat_err_log_mapping);

	LOG_INF("order: %d, log_position: %d, eeprom_address: 0x%X", order,
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

	plat_err_log_mapping *p = (plat_err_log_mapping *)log_data;

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
		if (!plat_eeprom_write(AEGIS_FRU_LOG_START + sizeof(plat_err_log_mapping) * i,
				       (uint8_t *)err_log_data, sizeof(plat_err_log_mapping))) {
			LOG_ERR("Clear EEPROM Log failed at index %d", i);
		}
		k_msleep(EEPROM_MAX_WRITE_TIME); // the eeprom max write time is 10 ms
	}
}

bool plat_dump_cpld(uint8_t offset, uint8_t length, uint8_t *data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = I2C_BUS_CPLD;
	i2c_msg.target_addr = AEGIS_CPLD_ADDR;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = length;
	i2c_msg.data[0] = offset;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read CPLD register 0x%02X", offset);
		return false;
	}

	memcpy(data, i2c_msg.data, length);
	return true;
}

bool get_vr_status_word(uint8_t bus, uint8_t addr, uint8_t *vr_status_word)
{
	CHECK_NULL_ARG_WITH_RETURN(vr_status_word, false);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 2;
	i2c_msg.data[0] = 0x79;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read VR status word");
		return false;
	}

	memcpy(vr_status_word, i2c_msg.data, 2);
	return true;
}

bool vr_fault_get_error_data(uint8_t sensor_id, uint8_t device_id, uint8_t *data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	bool ret = false;

	uint8_t bus;
	uint8_t addr;
	uint8_t sensor_dev;
	uint8_t vr_status_word[2] = { 0 };

	if (!get_sensor_info_by_sensor_id(sensor_id, &bus, &addr, &sensor_dev)) {
		LOG_ERR("Failed to find VR address and bus");
		return false;
	}

	struct k_mutex *p_mutex;

	if (device_id >= VR_1) {
		p_mutex = (struct k_mutex *)vr_mutex_get(device_id - 3); //mapping to VR_INDEX_E
		LOG_DBG("vr device_id %d, mutex %p", device_id, p_mutex);

		if (!p_mutex) {
			LOG_ERR("vr device_id %d, mutex is NULL", device_id);
			return false;
		}

		if (k_mutex_lock(p_mutex, K_MSEC(VR_MUTEX_LOCK_TIMEOUT_MS))) {
			LOG_ERR("vr device_id %d, mutex %p lock fail", device_id, p_mutex);
			return false;
		}
	}

	if (!get_vr_status_word(bus, addr, vr_status_word)) {
		LOG_ERR("Failed to get VR status word, sensor_id: 0x%x , bus: 0x%x, addr: 0x%x",
			sensor_id, bus, addr);
		goto err;
	}

	LOG_DBG("vr_fault_get_error_data VR status word: 0x%x 0x%x", vr_status_word[0],
		vr_status_word[1]);
	memcpy(data, vr_status_word, sizeof(vr_status_word));
	ret = true;

err:
	LOG_DBG("vr device_id %d, mutex %p unlock", device_id, p_mutex);
	if (device_id >= VR_1) {
		if (k_mutex_unlock(p_mutex))
			LOG_ERR("vr device_id %d, mutex %p unlock fail", device_id, p_mutex);
	}
	return ret;
}

bool get_error_data(uint16_t error_code, uint8_t *data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	// Extract CPLD offset and bit position from the error code
	uint8_t cpld_offset = error_code & 0xFF;
	uint8_t bit_position = (error_code >> 8) & 0x07;
	LOG_DBG("cpld_offset: 0x%x, bit_position: 0x%x", cpld_offset, bit_position);

	// Initialize sensor number
	uint8_t sensor_num = 0x00;
	uint8_t device_id = 0x00;

	// Find the device_id associated with the error code
	for (size_t i = 0; i < ARRAY_SIZE(vr_error_callback_info_table); i++) {
		if (vr_error_callback_info_table[i].cpld_offset == cpld_offset) {
			device_id = vr_error_callback_info_table[i]
					    .bit_mapping_vr_sensor_num[bit_position];
			break;
		}
	}

	if (device_id == 0x00) {
		LOG_DBG("No valid device_id for error_code: 0x%x", error_code);
		return false;
	}

	// Aegis BD dosn't have OSFP 3V3 VR
	if ((device_id == VR_1) && (get_board_type() == MINERVA_AEGIS_BD)) {
		LOG_DBG("Skip the check for OSFP 3V3 VR");
		return false;
	}

	// Find the sensor number associated with the device_id
	for (size_t i = 0; i < ARRAY_SIZE(vr_device_table); i++) {
		if (vr_device_table[i].index == device_id) {
			sensor_num = vr_device_table[i].sensor_num_1;
			break;
		}
	}

	// If no valid sensor number is found, skip further data retrieval
	if (sensor_num == 0x00) {
		LOG_DBG("No valid sensor_num for error_code: 0x%x", error_code);
		return false;
	}

	// Handle VR_FAULT_ASSERT errors and retrieve VR-specific data
	if (!vr_fault_get_error_data(sensor_num, device_id, data)) {
		LOG_ERR("Failed to retrieve VR fault data for sensor_num: 0x%x", sensor_num);
		return false;
	}

	return true;
}

// Handle error log events and record them if necessary
void error_log_event(uint16_t error_code, bool log_status)
{
	bool log_todo = false;
	uint8_t dump_data[AEGIS_CPLD_REGISTER_MAX_OFFSET - AEGIS_CPLD_REGISTER_START_OFFSET + 1];

	// Check if the error_code is already logged
	for (uint8_t i = 0; i < ARRAY_SIZE(err_code_caches); i++) {
		if (err_code_caches[i] == error_code) {
			if (log_status == LOG_ASSERT) {
				log_todo = false; // Duplicate error, no need to log again
				break;
			} else if (log_status == LOG_DEASSERT) {
				log_todo = true; // The error needs to be cleared
				err_code_caches[i] = 0; // Remove the error code from the cache
				break;
			}
		}
	}

	// If the error_code is new and it's a LOG_ASSERT, add it to the cache
	if (!log_todo && log_status == LOG_ASSERT) {
		for (uint8_t i = 0; i < ARRAY_SIZE(err_code_caches); i++) {
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

	// Record error log if necessary
	if (log_todo) {
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

		// Dump CPLD data and store it in cpld_dump
		if (plat_dump_cpld(
			    AEGIS_CPLD_REGISTER_START_OFFSET,
			    (AEGIS_CPLD_REGISTER_MAX_OFFSET - AEGIS_CPLD_REGISTER_START_OFFSET + 1),
			    dump_data)) {
			memcpy(err_log_data[fru_count].cpld_dump, dump_data, sizeof(dump_data));
		} else {
			LOG_ERR("Failed to dump CPLD data");
		}

		//dump err_log_data for debug
		LOG_HEXDUMP_DBG(&err_log_data[fru_count], sizeof(plat_err_log_mapping),
				"err_log_data");

		// 1 base fru_count, write_address is 0 base
		uint16_t write_address =
			AEGIS_FRU_LOG_START + (fru_count - 1) * sizeof(plat_err_log_mapping);

		// Write log to EEPROM with error handling
		if (!plat_eeprom_write(write_address, (uint8_t *)&err_log_data[fru_count],
				       sizeof(plat_err_log_mapping))) {
			LOG_ERR("Write Log failed with Error code: %02x", error_code);
		} else {
			k_msleep(EEPROM_MAX_WRITE_TIME); // wait 5ms to write EEPROM
		}

		// Update the next log position
		next_log_position = (fru_count % LOG_MAX_NUM) + 1;
	}
}

void find_last_log_position()
{
	uint16_t max_index = 0; // Highest valid index found
	uint16_t last_position = 0; // Position of the highest valid index
	bool all_empty = true; // Flag to detect if all entries are empty
	plat_err_log_mapping log_entry;

	for (uint16_t i = 0; i < LOG_MAX_NUM; i++) {
		uint16_t eeprom_address = AEGIS_FRU_LOG_START + i * sizeof(plat_err_log_mapping);

		if (!plat_eeprom_read(eeprom_address, (uint8_t *)&log_entry,
				      sizeof(plat_err_log_mapping))) {
			LOG_ERR("Failed to read log at position %d (address: 0x%X)", i,
				eeprom_address);
			continue;
		}

		// Check if the entry is valid
		if (log_entry.index != 0xFFFF && log_entry.index <= LOG_MAX_INDEX) {
			all_empty = false; // At least one entry is valid
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
		if (!plat_eeprom_read(AEGIS_FRU_LOG_START + i * log_len,
				      (uint8_t *)&err_log_data[i], log_len)) {
			LOG_ERR("READ Event %d failed from EEPROM", i + 1);
		}
	}

	// Determine the next log position
	find_last_log_position();
}
