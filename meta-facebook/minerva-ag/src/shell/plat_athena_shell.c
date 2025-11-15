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

#include <stdlib.h>
#include <shell/shell.h>
#include <string.h>
#include <limits.h>
#include <stdio.h>
#include "hal_i2c.h"
#include "plat_i2c.h"
#include "sideband_commands_athena_open.h"
// sideband_commands_athena_open.h SHA256: 52BC573B28B62D8577AFF25995C8F08FB2048E986BAFC53C6A4FAEA157930245
// Header file corresponding to Athena Boot-1 v4.13.1

// HBM data offset constants
#define HBM0_DATA_OFFSET BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG__OFFSET
#define HBM1_DATA_OFFSET BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG__OFFSET
#define HBM2_DATA_OFFSET BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG__OFFSET
#define HBM3_DATA_OFFSET BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG__OFFSET
#define HBM4_DATA_OFFSET BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG__OFFSET
#define HBM5_DATA_OFFSET BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG__OFFSET

#define HBM_CHANNEL_DATA_SIZE 8 // 8 bytes per channel
#define HBM_MAX_CHANNELS 16
#define HBM_MAX_HBMS 6

// OWL VTMON constants
#define OWL_MAX_COUNT 2 // owl[0] and owl[1]
#define VTMON_MAX_COUNT 2 // vtmon[0] and vtmon[1]

// HBM temperature constants
#define HBM_TEMP_MAX_COUNT 6 // hbm[0] to hbm[5]

// Efuse constants
#define EFUSE_MAX_TARGETS 3 // athena, owl_0, owl_1

// Boot status constants
#define CMD_STATUS_FLAGS_NOT_READY_OFFSET BRCMLIB_SB_STATUS_FLAGS_W_OFF
#define CMD_STATUS_FLAGS_NOT_READY_BIT 6

// Temperature monitoring constants
#define TEMP_VAL_MIN 0x00 // 0 °C
#define TEMP_VAL_MAX 0x7E // 126 °C
#define TEMP_VAL_OVER127 0x7F // 127 °C or higher

#define TEMP_VAL_NO_DATA 0x80 // No temperature data or data is older than 5 seconds
#define TEMP_VAL_SENSOR_FAIL 0x81 // Temperature sensor failure

#define TEMP_VAL_RESERVED_START 0x82 // Reserved range start
#define TEMP_VAL_RESERVED_END 0xC3 // Reserved range end

#define TEMP_VAL_BELOW_MINUS60 0xC4 // -60 °C or lower

#define TEMP_VAL_TWOS_COMPLEMENT 0xC5 // -1 °C to -59 °C (two's complement representation)
#define TEMP_VAL_TWOS_COMPLEMENT_END 0xFF

// Monitoring details base offset from the header file
#define MONITORING_DETAILS_BASE_OFFSET 0xD07C4000

#define ATHENA_NVME_MI_INDIRECT_ACCESS_SETTING_OFFSET 0xD3
#define ATHENA_INDIRECT_ACCESS_ALL_REGION 0xFF
#define ATHENA_INDIRECT_ACCESS_MONITORING_REGION 0x03

#define ATHENA_INDIRECT_ACCESS_SETTING_LENGTH 0x06
#define ATHENA_INDIRECT_ACCESS_SETTING_FLUSH_NO_INCR 0x10
#define ATHENA_INDIRECT_ACCESS_SETTING_PEC_BYTE 0xFF //Fixed to 0xFF

// I2C configuration for HBM
#define ATHENA_I2C_BUS I2C_BUS6
#define ATHENA_I2C_ADDR_WRITE 0x6A // Write address for offset
#define ATHENA_I2C_ADDR_READ 0x6C // Read address for data

// Maximum read length per transaction
#define ATHENA_MAX_READ_LENGTH 0xFF

// Efuse offset lookup table for the three targets
typedef struct {
	const char *name;
	uint16_t fab_designation_offset;
	uint16_t lot_designation_offset;
	uint16_t lot_number_offset;
	uint16_t wafer_number_offset;
	uint16_t wafer_region_offset;
	uint16_t internal1_offset;
	uint16_t oscillator_count_offset;
	uint16_t internal2_offset;
	uint16_t overclocking_offset;
	uint16_t m7serial_offset;
	uint8_t total_size; // Total bytes to read
} efuse_target_offsets_t;

static const efuse_target_offsets_t efuse_targets[EFUSE_MAX_TARGETS] = {
	// athena target
	{ "athena", BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_FAB_DESIGNATION__OFFSET,
	  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_LOT_DESIGNATION__OFFSET,
	  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_LOT_NUMBER__OFFSET,
	  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_WAFER_NUMBER__OFFSET,
	  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_WAFER_REGION__OFFSET,
	  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_INTERNAL1__OFFSET,
	  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_OSCILLATOR_COUNT__OFFSET,
	  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_INTERNAL2__OFFSET,
	  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_OVERCLOCKING__OFFSET,
	  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_M7SERIAL__OFFSET,
	  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA__BYTES },
	// owl_0 target
	{ "owl_0", BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_FAB_DESIGNATION__OFFSET,
	  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_LOT_DESIGNATION__OFFSET,
	  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_LOT_NUMBER__OFFSET,
	  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_WAFER_NUMBER__OFFSET,
	  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_WAFER_REGION__OFFSET,
	  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_INTERNAL1__OFFSET,
	  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_OSCILLATOR_COUNT__OFFSET,
	  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_INTERNAL2__OFFSET,
	  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_OVERCLOCKING__OFFSET,
	  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_M7SERIAL__OFFSET,
	  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0__BYTES },
	// owl_1 target
	{ "owl_1", BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_FAB_DESIGNATION__OFFSET,
	  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_LOT_DESIGNATION__OFFSET,
	  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_LOT_NUMBER__OFFSET,
	  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_WAFER_NUMBER__OFFSET,
	  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_WAFER_REGION__OFFSET,
	  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_INTERNAL1__OFFSET,
	  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_OSCILLATOR_COUNT__OFFSET,
	  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_INTERNAL2__OFFSET,
	  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_OVERCLOCKING__OFFSET,
	  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_M7SERIAL__OFFSET,
	  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1__BYTES }
};

/**
 * @brief VTMON data structure containing all monitoring values
 */
struct vtmon_data {
	int32_t v3p3;
	int32_t v1p8;
	int32_t v1p0[6]; // 6 v1p0 values
	int32_t temp[8]; // 8 temperature values
	int32_t diode;
	int32_t vctat;
	int32_t error;
};

/**
 * @brief Temperature monitoring data structure
 */
struct temp_monitor_data {
	uint8_t north_temp_local;
	uint8_t north_temp_remote[8];
	uint8_t south_temp_local;
	uint8_t south_temp_remote[8];
};

/**
 * @brief HBM temperature data structure
 */
struct hbm_temp_data {
	uint8_t max_temp_current;
	uint8_t junction_temperature;
	uint8_t channel_temps[64]; // 64 channel temperature values
	int8_t min_temp_across_channels; // Minimum temperature across all channels
	int8_t max_temp_across_channels; // Maximum temperature across all channels
};

/**
 * @brief Efuse data structure containing all efuse information
 */
struct efuse_data {
	char fab_designation;
	char lot_designation;
	char lot_number[4]; // 4-character lot number
	uint8_t wafer_number;
	uint16_t wafer_region;
	uint8_t internal1;
	uint16_t oscillator_count;
	uint64_t internal2;
	uint16_t overclocking; // MHz unit, 0 = unknown
	uint8_t m7serial[15]; // 15-byte M7 serial
};

/**
 * @brief Generic read function for Athena device
 * @param region Region code to access (e.g., 0xFF for all, 0x03 for monitoring)
 * @param final_offset 32-bit final offset address (base_offset + offset)
 * @param length Number of bytes to read (1-64)
 * @param data Buffer to store read data
 * @return 0 on success, negative error code on failure
 */
static int athena_generic_read(uint8_t region, uint32_t final_offset, uint8_t length, uint8_t *data)
{
	if (!data || length == 0 || length > ATHENA_MAX_READ_LENGTH) {
		return -1; // Invalid parameters
	}

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;

	// Setup I2C message for writing offset address
	i2c_msg.bus = ATHENA_I2C_BUS;
	i2c_msg.target_addr = ATHENA_I2C_ADDR_WRITE;
	i2c_msg.tx_len = 9;
	i2c_msg.rx_len = 0;

	// Pack the command and offset address
	i2c_msg.data[0] = ATHENA_NVME_MI_INDIRECT_ACCESS_SETTING_OFFSET;
	i2c_msg.data[1] = ATHENA_INDIRECT_ACCESS_SETTING_LENGTH;
	i2c_msg.data[2] = (final_offset >> 0) & 0xFF; // LSB
	i2c_msg.data[3] = (final_offset >> 8) & 0xFF;
	i2c_msg.data[4] = (final_offset >> 16) & 0xFF;
	i2c_msg.data[5] = (final_offset >> 24) & 0xFF; // MSB
	i2c_msg.data[6] = region;
	i2c_msg.data[7] = ATHENA_INDIRECT_ACCESS_SETTING_FLUSH_NO_INCR;
	i2c_msg.data[8] = ATHENA_INDIRECT_ACCESS_SETTING_PEC_BYTE;

	// Write offset address to device
	if (i2c_master_write(&i2c_msg, retry)) {
		return -2; // Write offset failed
	}

	// Setup I2C message for reading data
	i2c_msg.bus = ATHENA_I2C_BUS;
	i2c_msg.target_addr = ATHENA_I2C_ADDR_READ;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = length;
	i2c_msg.data[0] = 0x00; // Register offset

	// Read the data
	if (i2c_master_read(&i2c_msg, retry)) {
		return -3; // Read data failed
	}

	// Copy data to output buffer
	memcpy(data, i2c_msg.data, length);

	return 0;
}

/**
 * @brief Read OWL VTMON data from a specific OWL and VTMON
 * @param owl OWL number (0-1)
 * @param vtmon VTMON number (0-1)
 * @param data Buffer to store the VTMON data structure
 * @return 0 on success, negative error code on failure
 */
static int read_owl_vtmon_data(uint8_t owl, uint8_t vtmon, struct vtmon_data *data)
{
	if (owl >= OWL_MAX_COUNT || vtmon >= VTMON_MAX_COUNT || !data) {
		return -1;
	}

	uint32_t base_offset = MONITORING_DETAILS_BASE_OFFSET;
	uint32_t vtmon_offset;
	int ret;
	uint8_t temp_buffer[76]; // Buffer for the entire VTMON structure (76 bytes)

	// Determine the base offset for the specific OWL and VTMON
	if (owl == 0) {
		if (vtmon == 0) {
			vtmon_offset = BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0__OFFSET;
		} else {
			vtmon_offset = BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1__OFFSET;
		}
	} else {
		if (vtmon == 0) {
			vtmon_offset = BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0__OFFSET;
		} else {
			vtmon_offset = BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1__OFFSET;
		}
	}

	// Read the entire VTMON structure at once (76 bytes)
	uint32_t final_address = base_offset + vtmon_offset;
	ret = athena_generic_read(ATHENA_INDIRECT_ACCESS_ALL_REGION, final_address, 76,
				  temp_buffer);
	if (ret != 0) {
		return ret;
	}

	// Parse the data from the buffer
	// v3p3 (offset 0, 4 bytes)
	memcpy(&data->v3p3, &temp_buffer[0], 4);

	// v1p8 (offset 4, 4 bytes)
	memcpy(&data->v1p8, &temp_buffer[4], 4);

	// v1p0 array (offset 8, 6*4=24 bytes)
	memcpy(data->v1p0, &temp_buffer[8], 24);

	// temp array (offset 32, 8*4=32 bytes)
	memcpy(data->temp, &temp_buffer[32], 32);

	// diode (offset 64, 4 bytes)
	memcpy(&data->diode, &temp_buffer[64], 4);

	// vctat (offset 68, 4 bytes)
	memcpy(&data->vctat, &temp_buffer[68], 4);

	// error (offset 72, 4 bytes)
	memcpy(&data->error, &temp_buffer[72], 4);

	return 0;
}

/**
 * @brief Read temperature monitoring data
 * @param data Buffer to store the temperature monitoring data structure
 * @return 0 on success, negative error code on failure
 */
static int read_temp_monitor_data(struct temp_monitor_data *data)
{
	if (!data) {
		return -1;
	}

	uint32_t base_offset = MONITORING_DETAILS_BASE_OFFSET;
	int ret;

	// Read north temp_local (1 byte)
	ret = athena_generic_read(
		ATHENA_INDIRECT_ACCESS_ALL_REGION,
		base_offset + BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_TEMP_LOCAL__OFFSET, 1,
		&data->north_temp_local);
	if (ret != 0)
		return ret;

	// Read north temp_remote array (8 bytes)
	ret = athena_generic_read(
		ATHENA_INDIRECT_ACCESS_ALL_REGION,
		base_offset + BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_TEMP_REMOTE__OFFSET, 8,
		data->north_temp_remote);
	if (ret != 0)
		return ret;

	// Read south temp_local (1 byte)
	ret = athena_generic_read(
		ATHENA_INDIRECT_ACCESS_ALL_REGION,
		base_offset + BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_TEMP_LOCAL__OFFSET, 1,
		&data->south_temp_local);
	if (ret != 0)
		return ret;

	// Read south temp_remote array (8 bytes)
	ret = athena_generic_read(
		ATHENA_INDIRECT_ACCESS_ALL_REGION,
		base_offset + BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_TEMP_REMOTE__OFFSET, 8,
		data->south_temp_remote);
	if (ret != 0)
		return ret;

	return 0;
}

/**
 * @brief Read HBM temperature data from a specific HBM
 * @param hbm HBM number (0-5)
 * @param data Buffer to store the HBM temperature data structure
 * @return 0 on success, negative error code on failure
 */
static int read_hbm_temp_data(uint8_t hbm, struct hbm_temp_data *data)
{
	if (hbm >= HBM_TEMP_MAX_COUNT || !data) {
		return -1;
	}

	uint32_t base_offset = MONITORING_DETAILS_BASE_OFFSET;
	int ret;

	// Define offset lookup table for max_temp_current
	static const uint16_t max_temp_current_offsets[HBM_TEMP_MAX_COUNT] = {
		BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__OFFSET, // 5053
		BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__OFFSET, // 6347
		BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__OFFSET, // 7641
		BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__OFFSET, // 8935
		BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__OFFSET, // 10229
		BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__OFFSET // 11523
	};

	// Define offset lookup table for junction_temperature
	static const uint16_t junction_temperature_offsets[HBM_TEMP_MAX_COUNT] = {
		BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__OFFSET, // 5054
		BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__OFFSET, // 6348
		BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__OFFSET, // 7642
		BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__OFFSET, // 8936
		BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__OFFSET, // 10230
		BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__OFFSET // 11524
	};

	// Define offset lookup table for channel_temps (64 channels)
	static const uint16_t channel_temps_offsets[HBM_TEMP_MAX_COUNT] = {
		BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__OFFSET, // 5057
		BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__OFFSET, // 6351
		BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__OFFSET, // 7645
		BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__OFFSET, // 8939
		BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__OFFSET, // 10233
		BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__OFFSET // 11527
	};

	// Read max_temp_current (1 byte)
	ret = athena_generic_read(ATHENA_INDIRECT_ACCESS_ALL_REGION,
				  base_offset + max_temp_current_offsets[hbm], 1,
				  &data->max_temp_current);
	if (ret != 0)
		return ret;

	// Read junction_temperature (1 byte)
	ret = athena_generic_read(ATHENA_INDIRECT_ACCESS_ALL_REGION,
				  base_offset + junction_temperature_offsets[hbm], 1,
				  &data->junction_temperature);
	if (ret != 0)
		return ret;

	// Read channel_temps (64 bytes)
	ret = athena_generic_read(ATHENA_INDIRECT_ACCESS_ALL_REGION,
				  base_offset + channel_temps_offsets[hbm], 64,
				  data->channel_temps);
	if (ret != 0)
		return ret;

	// Calculate min and max temperature across all channels
	int8_t min_temp = (int8_t)data->channel_temps[0];
	int8_t max_temp = (int8_t)data->channel_temps[0];

	for (int i = 1; i < 64; i++) {
		int8_t temp = (int8_t)data->channel_temps[i];
		if (temp < min_temp) {
			min_temp = temp;
		}
		if (temp > max_temp) {
			max_temp = temp;
		}
	}

	data->min_temp_across_channels = min_temp;
	data->max_temp_across_channels = max_temp;

	return 0;
}

/**
 * @brief Read Athena boot status to check if boot1 is loaded
 * @param is_boot1_loaded Pointer to store the boot1 load status (true if loaded, false if not)
 * @return 0 on success, negative error code on failure
 */
static int read_athena_boot_status(bool *is_boot1_loaded)
{
	if (!is_boot1_loaded) {
		return -1;
	}

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;

	// Setup I2C message for reading the status flags
	i2c_msg.bus = ATHENA_I2C_BUS;
	i2c_msg.target_addr = ATHENA_I2C_ADDR_WRITE;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = CMD_STATUS_FLAGS_NOT_READY_OFFSET;

	// Read the status register
	if (i2c_master_read(&i2c_msg, retry)) {
		return -2; // Read failed
	}

	// Check bit 6 of the status register
	// Bit 6 = 1: device is not ready (boot1 not loaded)
	// Bit 6 = 0: device is ready (boot1 loaded)
	uint8_t status_byte = i2c_msg.data[0];
	bool not_ready = (status_byte >> CMD_STATUS_FLAGS_NOT_READY_BIT) & 0x01;

	*is_boot1_loaded = !not_ready; // Invert the logic: not_ready=0 means boot1 is loaded

	return 0;
}

/**
 * @brief Read Athena ASIC version
 * @param asic_version Pointer to store the ASIC version value
 * @return 0 on success, negative error code on failure
 */
static int read_athena_asic_version(uint8_t *asic_version)
{
	if (!asic_version) {
		return -1;
	}

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;

	// Setup I2C message for reading the ASIC version
	i2c_msg.bus = ATHENA_I2C_BUS;
	i2c_msg.target_addr = ATHENA_I2C_ADDR_WRITE;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = BRCMLIB_SB_VERSIONS_ASIC_W_OFF;

	// Read the ASIC version register
	if (i2c_master_read(&i2c_msg, retry)) {
		return -2;
	}

	*asic_version = i2c_msg.data[0];
	return 0;
}

/**
 * @brief Read Athena M7 version
 * @param major Pointer to store the M7 major version
 * @param minor Pointer to store the M7 minor version
 * @param optional Pointer to store the M7 optional version
 * @return 0 on success, negative error code on failure
 */
static int read_athena_m7_version(uint8_t *major, uint8_t *minor, uint8_t *optional)
{
	if (!major || !minor || !optional) {
		return -1;
	}

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;

	// Read M7 major version
	i2c_msg.bus = ATHENA_I2C_BUS;
	i2c_msg.target_addr = ATHENA_I2C_ADDR_WRITE;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = BRCMLIB_SB_VERSIONS_M7_MAJOR_W_OFF;

	if (i2c_master_read(&i2c_msg, retry)) {
		return -2;
	}
	*major = i2c_msg.data[0];

	// Read M7 minor version
	i2c_msg.data[0] = BRCMLIB_SB_VERSIONS_M7_MINOR_W_OFF;
	if (i2c_master_read(&i2c_msg, retry)) {
		return -2;
	}
	*minor = i2c_msg.data[0];

	// Read M7 optional version
	i2c_msg.data[0] = BRCMLIB_SB_VERSIONS_M7_OPTIONAL_W_OFF;
	if (i2c_master_read(&i2c_msg, retry)) {
		return -2;
	}
	*optional = i2c_msg.data[0];

	return 0;
}

/**
 * @brief Read Athena patch version
 * @param patch_major Pointer to store the patch major version
 * @return 0 on success, negative error code on failure
 */
static int read_athena_patch_version(uint8_t *patch_major)
{
	if (!patch_major) {
		return -1;
	}

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;

	// Setup I2C message for reading the patch version
	i2c_msg.bus = ATHENA_I2C_BUS;
	i2c_msg.target_addr = ATHENA_I2C_ADDR_WRITE;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = BRCMLIB_SB_VERSIONS_PATCH_MAJOR_W_OFF;

	// Read the patch version register
	if (i2c_master_read(&i2c_msg, retry)) {
		return -2;
	}

	*patch_major = i2c_msg.data[0];
	return 0;
}

/**
 * @brief Read Athena current temperature
 * @param current_temp Pointer to store the current temperature
 * @return 0 on success, negative error code on failure
 */
static int read_athena_current_temp(uint8_t *current_temp)
{
	if (!current_temp) {
		return -1;
	}

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;

	// Setup I2C message for reading the current temperature
	i2c_msg.bus = ATHENA_I2C_BUS;
	i2c_msg.target_addr = ATHENA_I2C_ADDR_WRITE;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = BRCMLIB_SB_STATUS_TEMP_W_OFF;

	// Read the current temperature register
	if (i2c_master_read(&i2c_msg, retry)) {
		return -2;
	}

	*current_temp = i2c_msg.data[0];
	return 0;
}

/**
 * @brief Read Athena max ASIC temperature
 * @param max_asic_temp Pointer to store the max ASIC temperature
 * @return 0 on success, negative error code on failure
 */
static int read_athena_max_asic_temp(uint8_t *max_asic_temp)
{
	if (!max_asic_temp) {
		return -1;
	}

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;

	// Setup I2C message for reading the max ASIC temperature
	i2c_msg.bus = ATHENA_I2C_BUS;
	i2c_msg.target_addr = ATHENA_I2C_ADDR_WRITE;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = BRCMLIB_SB_ERROR_REPORT_MAX_ASIC_TEMP_W_OFF;

	// Read the max ASIC temperature register
	if (i2c_master_read(&i2c_msg, retry)) {
		return -2;
	}

	*max_asic_temp = i2c_msg.data[0];
	return 0;
}

/**
 * @brief Print raw data in hex format with ASCII representation
 * @param shell Shell instance
 * @param offset Starting offset address
 * @param data Raw data bytes
 * @param length Number of bytes
 */
static void print_hex_data(const struct shell *shell, uint32_t offset, uint8_t *data,
			   uint8_t length)
{
	shell_print(shell, "Address: 0x%08X (%u bytes)", offset, length);
	shell_print(shell, "");

	for (uint16_t i = 0; i < length; i += 16) {
		// Print address
		shell_fprintf(shell, SHELL_NORMAL, "%08X: ", offset + i);

		// Print hex values
		for (uint8_t j = 0; j < 16 && (i + j) < length; j++) {
			shell_fprintf(shell, SHELL_NORMAL, "%02X ", data[i + j]);
		}

		// Pad with spaces if less than 16 bytes
		uint8_t bytes_in_line = (length - i > 16) ? 16 : (length - i);
		for (uint8_t j = bytes_in_line; j < 16; j++) {
			shell_fprintf(shell, SHELL_NORMAL, "   ");
		}

		// Print ASCII representation
		shell_fprintf(shell, SHELL_NORMAL, " |");
		for (uint8_t j = 0; j < 16 && (i + j) < length; j++) {
			uint8_t c = data[i + j];
			if (c >= 32 && c <= 126) {
				shell_fprintf(shell, SHELL_NORMAL, "%c", c);
			} else {
				shell_fprintf(shell, SHELL_NORMAL, ".");
			}
		}
		shell_fprintf(shell, SHELL_NORMAL, "|\n");
	}
	shell_print(shell, "");
}

/**
 * @brief Print a single VTMON value in simplified format
 * @param shell Shell instance
 * @param label Value label
 * @param raw_data Raw bytes (4 bytes)
 * @param value_type Type of value: 'V' for voltage, 'T' for temperature, 'N' for normal
 */
static void print_vtmon_value(const struct shell *shell, const char *label, uint8_t *raw_data,
			      char value_type)
{
	// Little endian interpretation
	int32_t le_value = (int32_t)(raw_data[0] | (raw_data[1] << 8) | (raw_data[2] << 16) |
				     (raw_data[3] << 24));

	// Check if it's an invalid value
	bool is_invalid = (le_value == INT32_MIN);

	if (is_invalid) {
		shell_fprintf(shell, SHELL_NORMAL, "  %-12s raw: %02X %02X %02X %02X -> %-12s\n",
			      label, raw_data[0], raw_data[1], raw_data[2], raw_data[3],
			      "(invalid)");
	} else {
		if (value_type == 'V') {
			// Voltage values divided by 1000 with V unit
			float voltage = (float)le_value / 1000.0f;
			shell_fprintf(shell, SHELL_NORMAL,
				      "  %-12s raw: %02X %02X %02X %02X -> %-12.3fV\n", label,
				      raw_data[0], raw_data[1], raw_data[2], raw_data[3], voltage);
		} else if (value_type == 'T') {
			// Temperature values divided by 1000 with °C unit
			float temp = (float)le_value / 1000.0f;
			shell_fprintf(shell, SHELL_NORMAL,
				      "  %-12s raw: %02X %02X %02X %02X -> %-12.3f°C\n", label,
				      raw_data[0], raw_data[1], raw_data[2], raw_data[3], temp);
		} else {
			// Normal values, just display the number
			shell_fprintf(shell, SHELL_NORMAL,
				      "  %-12s raw: %02X %02X %02X %02X -> %-12d\n", label,
				      raw_data[0], raw_data[1], raw_data[2], raw_data[3], le_value);
		}
	}
}

/**
 * @brief Print OWL VTMON data in a formatted way with raw data
 * @param shell Shell instance
 * @param owl OWL number
 * @param vtmon VTMON number
 * @param data VTMON data structure
 */
static void print_owl_vtmon_data(const struct shell *shell, uint8_t owl, uint8_t vtmon,
				 struct vtmon_data *data)
{
	shell_print(shell, "OWL %d VTMON %d:", owl, vtmon);

	// Print v3p3
	print_vtmon_value(shell, "v3p3      ", (uint8_t *)&data->v3p3, 'V');

	// Print v1p8
	print_vtmon_value(shell, "v1p8      ", (uint8_t *)&data->v1p8, 'V');

	// Print v1p0 array
	for (int i = 0; i < 6; i++) {
		char label[16];
		snprintf(label, sizeof(label), "v1p0[%d]   ", i);
		print_vtmon_value(shell, label, (uint8_t *)&data->v1p0[i], 'V');
	}

	// Print temp array
	for (int i = 0; i < 8; i++) {
		char label[16];
		snprintf(label, sizeof(label), "temp[%d]   ", i);
		print_vtmon_value(shell, label, (uint8_t *)&data->temp[i], 'T');
	}

	// Print diode
	print_vtmon_value(shell, "diode     ", (uint8_t *)&data->diode, 'T');

	// Print vctat
	print_vtmon_value(shell, "vctat     ", (uint8_t *)&data->vctat, 'T');

	// Print error
	print_vtmon_value(shell, "error     ", (uint8_t *)&data->error, 'N');

	shell_print(shell, "");
}

/**
 * @brief Parse and interpret temperature value
 * @param shell Shell instance
 * @param label Temperature sensor label
 * @param temp_val Raw temperature value (1 byte)
 */
static void print_temperature_value(const struct shell *shell, const char *label, uint8_t temp_val)
{
	shell_fprintf(shell, SHELL_NORMAL, "  %s raw: 0x%02X -> ", label, temp_val);

	if (temp_val >= TEMP_VAL_MIN && temp_val <= TEMP_VAL_MAX) {
		// 00h–7Eh: Temperature is measured in degrees Celsius (0 to 126 °C)
		shell_fprintf(shell, SHELL_NORMAL, "%d°C\n", temp_val);
	} else if (temp_val == TEMP_VAL_OVER127) {
		// 7Fh: 127 °C or higher
		shell_fprintf(shell, SHELL_NORMAL, "127°C or higher\n");
	} else if (temp_val == TEMP_VAL_NO_DATA) {
		// 80h: No temperature data or temperature data is more than 5 seconds old
		shell_fprintf(shell, SHELL_NORMAL,
			      "No temperature data or data is older than 5 seconds\n");
	} else if (temp_val == TEMP_VAL_SENSOR_FAIL) {
		// 81h: Temperature sensor failure
		shell_fprintf(shell, SHELL_NORMAL, "Temperature sensor failure\n");
	} else if (temp_val >= TEMP_VAL_RESERVED_START && temp_val <= TEMP_VAL_RESERVED_END) {
		// 82h–C3h: Reserved
		shell_fprintf(shell, SHELL_NORMAL, "Reserved\n");
	} else if (temp_val == TEMP_VAL_BELOW_MINUS60) {
		// C4h: Temperature is -60 °C or lower
		shell_fprintf(shell, SHELL_NORMAL, "-60°C or lower\n");
	} else if (temp_val >= TEMP_VAL_TWOS_COMPLEMENT &&
		   temp_val <= TEMP_VAL_TWOS_COMPLEMENT_END) {
		// C5h–FFh: Temperature measured in degrees Celsius, represented in two's complement (-1 to -59 °C)
		int8_t signed_temp = (int8_t)temp_val;
		shell_fprintf(shell, SHELL_NORMAL, "%d°C (two's complement)\n", signed_temp);
	} else {
		shell_fprintf(shell, SHELL_NORMAL, "Unknown/Invalid\n");
	}
}

/**
 * @brief Print temperature monitoring data in a formatted way
 * @param shell Shell instance
 * @param data Temperature monitoring data structure
 */
static void print_temp_monitor_data(const struct shell *shell, struct temp_monitor_data *data)
{
	shell_print(shell, "Temperature Monitor Data:");
	shell_print(shell, "");

	// Print North temperature sensors
	shell_print(shell, "North:");
	print_temperature_value(shell, "temp_local    ", data->north_temp_local);
	for (int i = 0; i < 8; i++) {
		char label[32];
		snprintf(label, sizeof(label), "temp_remote[%d]", i);
		print_temperature_value(shell, label, data->north_temp_remote[i]);
	}

	shell_print(shell, "");

	// Print South temperature sensors
	shell_print(shell, "South:");
	print_temperature_value(shell, "temp_local    ", data->south_temp_local);
	for (int i = 0; i < 8; i++) {
		char label[32];
		snprintf(label, sizeof(label), "temp_remote[%d]", i);
		print_temperature_value(shell, label, data->south_temp_remote[i]);
	}

	shell_print(shell, "");
}

/**
 * @brief Print HBM temperature data in a formatted way
 * @param shell Shell instance
 * @param hbm HBM number
 * @param data HBM temperature data structure
 */
static void print_hbm_temp_data(const struct shell *shell, uint8_t hbm, struct hbm_temp_data *data)
{
	shell_print(shell, "HBM %d Temperature:", hbm);

	// Print max_temp_current with raw hex and decimal conversion
	shell_fprintf(shell, SHELL_NORMAL, "  %-20s raw: %02X -> %-12d°C\n", "max_temp_current",
		      data->max_temp_current, (int8_t)data->max_temp_current);

	// Print junction_temperature with raw hex and decimal conversion
	shell_fprintf(shell, SHELL_NORMAL, "  %-20s raw: %02X -> %-12d°C\n", "junction_temperature",
		      data->junction_temperature, (int8_t)data->junction_temperature);

	// Print min and max temperature across all channels
	shell_fprintf(shell, SHELL_NORMAL, "  %-20s raw: %02X -> %-12d°C\n", "min_temp_all_chan",
		      (uint8_t)data->min_temp_across_channels, data->min_temp_across_channels);
	shell_fprintf(shell, SHELL_NORMAL, "  %-20s raw: %02X -> %-12d°C\n", "max_temp_all_chan",
		      (uint8_t)data->max_temp_across_channels, data->max_temp_across_channels);

	// Print all 64 channel temperatures with raw values
	shell_print(shell, "  Channel temperatures (64 channels):");
	for (int i = 0; i < 64; i++) {
		// Print 8 temperatures per line for better readability
		if (i % 8 == 0) {
			shell_fprintf(shell, SHELL_NORMAL, "    Ch%02d-%02d:", i, i + 7);
		}
		shell_fprintf(shell, SHELL_NORMAL, " %3d°C", (int8_t)data->channel_temps[i]);
		if (i % 8 == 7) {
			shell_fprintf(shell, SHELL_NORMAL, "\n");
		}
	}

	shell_print(shell, "");
}

/**
 * @brief Read efuse data from specified target
 * @param target_idx Target index (0=athena, 1=owl_0, 2=owl_1)
 * @param efuse_data Pointer to efuse data structure
 * @return 0 on success, negative error code on failure
 */
int read_efuse_data(int target_idx, struct efuse_data *efuse_data)
{
	if (target_idx < 0 || target_idx >= EFUSE_MAX_TARGETS || !efuse_data) {
		return -1;
	}

	const efuse_target_offsets_t *target = &efuse_targets[target_idx];
	uint8_t buffer[64]; // Buffer to hold raw data for parsing
	uint32_t base_offset = MONITORING_DETAILS_BASE_OFFSET;
	int ret;

	// Read fab_designation (1 byte)
	ret = athena_generic_read(ATHENA_INDIRECT_ACCESS_ALL_REGION,
				  base_offset + target->fab_designation_offset, 1, buffer);
	if (ret != 0) {
		return -2;
	}
	efuse_data->fab_designation = (char)buffer[0];

	// Read lot_designation (1 byte)
	ret = athena_generic_read(ATHENA_INDIRECT_ACCESS_ALL_REGION,
				  base_offset + target->lot_designation_offset, 1, buffer);
	if (ret != 0) {
		return -2;
	}
	efuse_data->lot_designation = (char)buffer[0];

	// Read lot_number (4 bytes)
	ret = athena_generic_read(ATHENA_INDIRECT_ACCESS_ALL_REGION,
				  base_offset + target->lot_number_offset, 4, buffer);
	if (ret != 0) {
		return -2;
	}
	for (int i = 0; i < 4; i++) {
		efuse_data->lot_number[i] = (char)buffer[i];
	}

	// Read wafer_number (1 byte)
	ret = athena_generic_read(ATHENA_INDIRECT_ACCESS_ALL_REGION,
				  base_offset + target->wafer_number_offset, 1, buffer);
	if (ret != 0) {
		return -2;
	}
	efuse_data->wafer_number = buffer[0];

	// Read wafer_region (2 bytes)
	ret = athena_generic_read(ATHENA_INDIRECT_ACCESS_ALL_REGION,
				  base_offset + target->wafer_region_offset, 2, buffer);
	if (ret != 0) {
		return -2;
	}
	efuse_data->wafer_region = (uint16_t)(buffer[0] | (buffer[1] << 8));

	// Read internal1 (1 byte)
	ret = athena_generic_read(ATHENA_INDIRECT_ACCESS_ALL_REGION,
				  base_offset + target->internal1_offset, 1, buffer);
	if (ret != 0) {
		return -2;
	}
	efuse_data->internal1 = buffer[0];

	// Read oscillator_count (2 bytes)
	ret = athena_generic_read(ATHENA_INDIRECT_ACCESS_ALL_REGION,
				  base_offset + target->oscillator_count_offset, 2, buffer);
	if (ret != 0) {
		return -2;
	}
	efuse_data->oscillator_count = (uint16_t)(buffer[0] | (buffer[1] << 8));

	// Read internal2 (8 bytes)
	ret = athena_generic_read(ATHENA_INDIRECT_ACCESS_ALL_REGION,
				  base_offset + target->internal2_offset, 8, buffer);
	if (ret != 0) {
		return -2;
	}
	efuse_data->internal2 = 0;
	for (int i = 0; i < 8; i++) {
		efuse_data->internal2 |= ((uint64_t)buffer[i]) << (i * 8);
	}

	// Read overclocking (2 bytes)
	ret = athena_generic_read(ATHENA_INDIRECT_ACCESS_ALL_REGION,
				  base_offset + target->overclocking_offset, 2, buffer);
	if (ret != 0) {
		return -2;
	}
	efuse_data->overclocking = (uint16_t)(buffer[0] | (buffer[1] << 8));

	// Read m7serial (15 bytes)
	ret = athena_generic_read(ATHENA_INDIRECT_ACCESS_ALL_REGION,
				  base_offset + target->m7serial_offset, 15, buffer);
	if (ret != 0) {
		return -2;
	}
	for (int i = 0; i < 15; i++) {
		efuse_data->m7serial[i] = buffer[i];
	}

	return 0;
}

/**
 * @brief Print efuse field with raw hex and interpreted value
 * @param shell Shell instance
 * @param field_name Field name to display
 * @param raw_data Pointer to raw data
 * @param size Size of raw data in bytes
 * @param format_type Type of formatting: 'c' for ASCII char, 's' for ASCII string, 'd' for decimal, 'M' for MHz
 * @param value Optional decimal value for non-ASCII fields
 */
void print_efuse_field(const struct shell *shell, const char *field_name, const void *raw_data,
		       size_t size, char format_type, uint64_t value)
{
	const uint8_t *data = (const uint8_t *)raw_data;

	// Print field name and raw hex
	shell_fprintf(shell, SHELL_NORMAL, "  %-18s", field_name);

	// Print raw hex data
	for (size_t i = 0; i < size; i++) {
		shell_fprintf(shell, SHELL_NORMAL, "%02X", data[i]);
		if (i < size - 1) {
			shell_fprintf(shell, SHELL_NORMAL, " ");
		}
	}

	// Add appropriate spacing based on size
	for (size_t i = size; i < 8; i++) {
		shell_fprintf(shell, SHELL_NORMAL, "   "); // 3 spaces per missing byte
	}

	// Print interpreted value based on format type
	shell_fprintf(shell, SHELL_NORMAL, " = ");
	switch (format_type) {
	case 'c': // Single ASCII character
		if (data[0] >= 32 && data[0] <= 126) { // Printable ASCII
			shell_fprintf(shell, SHELL_NORMAL, "'%c'", data[0]);
		} else {
			shell_fprintf(shell, SHELL_NORMAL, "'.'");
		}
		break;
	case 's': // ASCII string
		shell_fprintf(shell, SHELL_NORMAL, "\"");
		for (size_t i = 0; i < size; i++) {
			if (data[i] >= 32 && data[i] <= 126) { // Printable ASCII
				shell_fprintf(shell, SHELL_NORMAL, "%c", data[i]);
			} else {
				shell_fprintf(shell, SHELL_NORMAL, ".");
			}
		}
		shell_fprintf(shell, SHELL_NORMAL, "\"");
		break;
	case 'd': // Decimal
		shell_fprintf(shell, SHELL_NORMAL, "%llu", value);
		break;
	case 'M': // MHz unit
		if (value == 0) {
			shell_fprintf(shell, SHELL_NORMAL, "unknown");
		} else {
			shell_fprintf(shell, SHELL_NORMAL, "%llu MHz", value);
		}
		break;
	default:
		shell_fprintf(shell, SHELL_NORMAL, "???");
		break;
	}

	shell_fprintf(shell, SHELL_NORMAL, "\n");
}

/**
 * @brief Print efuse data with formatted output
 * @param shell Shell instance
 * @param target_name Target name
 * @param efuse_data Efuse data structure
 */
void print_efuse_data(const struct shell *shell, const char *target_name,
		      const struct efuse_data *efuse_data)
{
	shell_print(shell, "");
	shell_print(shell, "Efuse data for %s:", target_name);
	shell_print(shell, "");

	// Print each field with appropriate formatting
	print_efuse_field(shell, "fab_designation:", &efuse_data->fab_designation, 1, 'c', 0);
	print_efuse_field(shell, "lot_designation:", &efuse_data->lot_designation, 1, 'c', 0);
	print_efuse_field(shell, "lot_number:", efuse_data->lot_number, 4, 's', 0);
	print_efuse_field(shell, "wafer_number:", &efuse_data->wafer_number, 1, 'd',
			  efuse_data->wafer_number);
	print_efuse_field(shell, "wafer_region:", &efuse_data->wafer_region, 2, 'd',
			  efuse_data->wafer_region);
	print_efuse_field(shell, "internal1:", &efuse_data->internal1, 1, 'd',
			  efuse_data->internal1);
	print_efuse_field(shell, "oscillator_count:", &efuse_data->oscillator_count, 2, 'd',
			  efuse_data->oscillator_count);
	print_efuse_field(shell, "internal2:", &efuse_data->internal2, 8, 'd',
			  efuse_data->internal2);
	print_efuse_field(shell, "overclocking:", &efuse_data->overclocking, 2, 'M',
			  efuse_data->overclocking);
	print_efuse_field(shell, "m7serial:", efuse_data->m7serial, 15, 's', 0);

	shell_print(shell, "");
}

/**
 * @brief Read OWL max die temperature from a specific OWL
 * @param owl OWL number (0-1)
 * @param max_die_temp Pointer to store the max die temperature (int32_t)
 * @return 0 on success, negative error code on failure
 */
static int read_owl_max_die_temp(uint8_t owl, int32_t *max_die_temp)
{
	if (owl >= OWL_MAX_COUNT || !max_die_temp) {
		return -1;
	}

	uint32_t base_offset = MONITORING_DETAILS_BASE_OFFSET;
	uint32_t max_die_temp_offset;
	int ret;
	uint8_t temp_buffer[4]; // 4 bytes for int32_t

	// Determine the offset for the specific OWL
	if (owl == 0) {
		max_die_temp_offset = BRCMLIB__MONITORING_DETAILS_OWL__0_MAX_DIE_TEMP__OFFSET;
	} else {
		max_die_temp_offset = BRCMLIB__MONITORING_DETAILS_OWL__1_MAX_DIE_TEMP__OFFSET;
	}

	// Read the max die temperature (4 bytes)
	uint32_t final_address = base_offset + max_die_temp_offset;
	ret = athena_generic_read(ATHENA_INDIRECT_ACCESS_ALL_REGION, final_address, 4, temp_buffer);
	if (ret != 0) {
		return ret;
	}

	// Parse the data as little-endian int32_t
	*max_die_temp = (int32_t)(temp_buffer[0] | (temp_buffer[1] << 8) | (temp_buffer[2] << 16) |
				  (temp_buffer[3] << 24));

	return 0;
}

/**
 * @brief Read HBM all max temperature
 * @param max_temp_all_chan Pointer to store the max temperature across all channels (int8_t)
 * @return 0 on success, negative error code on failure
 */
static int read_hbm_all_max_temp(int8_t *max_temp_all_chan)
{
	if (!max_temp_all_chan) {
		return -1;
	}

	uint32_t base_offset = MONITORING_DETAILS_BASE_OFFSET;
	int ret;
	uint8_t temp_buffer[1]; // 1 byte

	// Read the max temperature across all channels (1 byte)
	uint32_t final_address =
		base_offset + BRCMLIB__MONITORING_DETAILS_HBM_ALL_MAX_TEMP_ALL_CHAN__OFFSET;
	ret = athena_generic_read(ATHENA_INDIRECT_ACCESS_ALL_REGION, final_address, 1, temp_buffer);
	if (ret != 0) {
		return ret;
	}

	// Convert to signed int8_t
	*max_temp_all_chan = (int8_t)temp_buffer[0];

	return 0;
}

/**
 * @brief Read HBM all min temperature in SID2
 * @param min_temp_sid2 Pointer to store the min temperature in SID2 (int8_t)
 * @return 0 on success, negative error code on failure
 */
static int read_hbm_all_min_temp_sid2(int8_t *min_temp_sid2)
{
	if (!min_temp_sid2) {
		return -1;
	}

	uint32_t base_offset = MONITORING_DETAILS_BASE_OFFSET;
	int ret;
	uint8_t temp_buffer[1]; // 1 byte

	// Read the min temperature in SID2 across all channels (1 byte)
	uint32_t final_address =
		base_offset + BRCMLIB__MONITORING_DETAILS_HBM_ALL_MIN_TEMP_ALL_CHAN_SID2__OFFSET;
	ret = athena_generic_read(ATHENA_INDIRECT_ACCESS_ALL_REGION, final_address, 1, temp_buffer);
	if (ret != 0) {
		return ret;
	}

	// Convert to signed int8_t
	*min_temp_sid2 = (int8_t)temp_buffer[0];

	return 0;
}

/**
 * @brief Parse parameter with strict format validation (generic function)
 * @param arg Input argument string (expected format: "prefix_number")
 * @param prefix Expected prefix string (e.g., "hbm_", "owl_", "vtmon_")
 * @param max_value Maximum allowed value for the number part (exclusive)
 * @param parsed_num Pointer to store the parsed number
 * @return 0 on success, -1 on failure
 */
static int parse_prefixed_param(const char *arg, const char *prefix, uint8_t max_value,
				uint8_t *parsed_num)
{
	if (!arg || !prefix || !parsed_num) {
		return -1;
	}

	size_t prefix_len = strlen(prefix);

	// Check if the string starts with the expected prefix
	if (strncmp(arg, prefix, prefix_len) != 0) {
		return -1;
	}

	// Check if there's actually a number part after the prefix
	if (strlen(arg) <= prefix_len) {
		return -1;
	}

	// Extract the number part after the prefix
	char *endptr;
	long num = strtol(arg + prefix_len, &endptr, 10);

	// Check if the entire remaining string was consumed and is a valid number
	if (*endptr != '\0') {
		return -1; // Invalid characters after number
	}

	// Check if the number is within valid range
	if (num < 0 || num >= max_value) {
		return -1; // Number out of range
	}

	*parsed_num = (uint8_t)num;
	return 0;
}

void athena_read_cmd(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 5) {
		shell_error(shell, "Usage: athena read <region> <base_offset> <offset> <length>");
		shell_error(shell, "");
		shell_error(shell, "Parameters:");
		shell_error(
			shell,
			"  region      : Region code (hex, e.g., 0xFF for all, 0x03 for monitoring)");
		shell_error(shell, "  base_offset : Base offset address (hex or decimal)");
		shell_error(shell, "  offset      : Additional offset (hex or decimal)");
		shell_error(shell, "  length      : Number of bytes to read (1-%d)",
			    ATHENA_MAX_READ_LENGTH);
		shell_error(shell, "");
		shell_error(shell, "Final address = base_offset + offset");
		shell_error(shell, "");
		shell_error(shell, "Examples:");
		shell_error(
			shell,
			"  athena read 0xFF 0xD07C4000 0xEED 8    # Read 8 bytes from 0xD07C4EED");
		shell_error(shell,
			    "  athena read 0x03 0x1000 0x100 16       # Read 16 bytes from 0x1100");
		shell_error(
			shell,
			"  athena read 255 268435456 3821 32     # Same as first example (decimal)");
		return;
	}

	// Parse region (support both hex and decimal)
	char *endptr;
	uint8_t region = (uint8_t)strtoul(argv[1], &endptr, 0);
	if (*endptr != '\0') {
		shell_error(shell, "Invalid region value: %s", argv[1]);
		return;
	}

	// Parse base_offset (support both hex and decimal)
	uint32_t base_offset = (uint32_t)strtoul(argv[2], &endptr, 0);
	if (*endptr != '\0') {
		shell_error(shell, "Invalid base_offset value: %s", argv[2]);
		return;
	}

	// Parse offset (support both hex and decimal)
	uint32_t offset = (uint32_t)strtoul(argv[3], &endptr, 0);
	if (*endptr != '\0') {
		shell_error(shell, "Invalid offset value: %s", argv[3]);
		return;
	}

	// Parse length
	uint8_t length = (uint8_t)strtoul(argv[4], &endptr, 0);
	if (*endptr != '\0' || length == 0 || length > ATHENA_MAX_READ_LENGTH) {
		shell_error(shell, "Invalid length. Must be 1-%d", ATHENA_MAX_READ_LENGTH);
		return;
	}

	// Calculate final address
	uint32_t final_offset = base_offset + offset;

	// Allocate buffer for data
	uint8_t data[ATHENA_MAX_READ_LENGTH];

	// Execute the read
	int ret = athena_generic_read(region, final_offset, length, data);

	if (ret == 0) {
		print_hex_data(shell, final_offset, data, length);
	} else {
		const char *error_msg;
		switch (ret) {
		case -1:
			error_msg = "Invalid parameters";
			break;
		case -2:
			error_msg = "Failed to write offset address";
			break;
		case -3:
			error_msg = "Failed to read data";
			break;
		default:
			error_msg = "Unknown error";
			break;
		}
		shell_error(shell, "Failed to read from region 0x%02X offset 0x%08X: %s", region,
			    final_offset, error_msg);
	}
}

void athena_owl_vtmon_cmd(const struct shell *shell, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(shell, "Usage: athena owl_vtmon all");
		shell_error(shell, "       athena owl_vtmon <owl_0~1> <vtmon_0~1>");
		shell_error(shell, "       athena owl_vtmon <owl_0~1> max_die_temp");
		return;
	}

	if (strcmp(argv[1], "all") == 0) {
		// Read all OWL VTMON data
		shell_print(shell, "Reading all OWL VTMON data...");

		for (uint8_t owl = 0; owl < OWL_MAX_COUNT; owl++) {
			for (uint8_t vtmon = 0; vtmon < VTMON_MAX_COUNT; vtmon++) {
				struct vtmon_data data;
				int ret = read_owl_vtmon_data(owl, vtmon, &data);

				if (ret == 0) {
					print_owl_vtmon_data(shell, owl, vtmon, &data);
				} else {
					shell_error(shell,
						    "Failed to read OWL%d VTMON%d (error: %d)", owl,
						    vtmon, ret);
				}
			}

			// Also read max die temperature for each OWL
			int32_t max_die_temp;
			int ret = read_owl_max_die_temp(owl, &max_die_temp);
			if (ret == 0) {
				float temp = (float)max_die_temp / 1000.0f;
				char label[32];
				snprintf(label, sizeof(label), "OWL%d max_die_temp", owl);
				uint8_t *raw_bytes = (uint8_t *)&max_die_temp;
				shell_fprintf(shell, SHELL_NORMAL,
					      "  %-16s raw: %02X %02X %02X %02X -> %-12.3f°C\n\n",
					      label, raw_bytes[0], raw_bytes[1], raw_bytes[2],
					      raw_bytes[3], temp);
			} else {
				shell_error(shell, "Failed to read OWL%d max die temp (error: %d)",
					    owl, ret);
			}
		}
	} else if (argc == 3 && strcmp(argv[2], "max_die_temp") == 0) {
		// Read specific OWL max die temperature
		uint8_t owl;
		if (parse_prefixed_param(argv[1], "owl_", OWL_MAX_COUNT, &owl) != 0) {
			shell_error(shell, "Usage: athena owl_vtmon all");
			shell_error(shell, "       athena owl_vtmon <owl_0~1> <vtmon_0~1>");
			shell_error(shell, "       athena owl_vtmon <owl_0~1> max_die_temp");
			return;
		}

		shell_print(shell, "Reading OWL %d max die temperature...", owl);

		int32_t max_die_temp;
		int ret = read_owl_max_die_temp(owl, &max_die_temp);

		if (ret == 0) {
			float temp = (float)max_die_temp / 1000.0f;
			shell_print(shell, "");
			char label[32];
			snprintf(label, sizeof(label), "OWL%d_max_die_temp", owl);
			uint8_t *raw_bytes = (uint8_t *)&max_die_temp;
			shell_fprintf(shell, SHELL_NORMAL,
				      "  %-16s raw: %02X %02X %02X %02X -> %-12.3f°C\n", label,
				      raw_bytes[0], raw_bytes[1], raw_bytes[2], raw_bytes[3], temp);
			shell_print(shell, "");
		} else {
			const char *error_msg;
			switch (ret) {
			case -1:
				error_msg = "Invalid parameters";
				break;
			case -2:
				error_msg = "Failed to write offset address";
				break;
			case -3:
				error_msg = "Failed to read data";
				break;
			default:
				error_msg = "Unknown error";
				break;
			}
			shell_error(shell, "Failed to read OWL%d max die temp: %s", owl, error_msg);
		}
	} else {
		// Read specific OWL and VTMON
		if (argc != 3) {
			shell_error(shell, "Usage: athena owl_vtmon <owl_0~1> <vtmon_0~1>");
			shell_error(shell, "       athena owl_vtmon <owl_0~1> max_die_temp");
			return;
		}

		// Parse OWL parameter with strict format validation
		uint8_t owl;
		if (parse_prefixed_param(argv[1], "owl_", OWL_MAX_COUNT, &owl) != 0) {
			shell_error(shell, "Usage: athena owl_vtmon <owl_0~1> <vtmon_0~1>");
			shell_error(shell, "       athena owl_vtmon <owl_0~1> max_die_temp");
			return;
		}

		// Parse VTMON parameter with strict format validation
		uint8_t vtmon;
		if (parse_prefixed_param(argv[2], "vtmon_", VTMON_MAX_COUNT, &vtmon) != 0) {
			shell_error(shell, "Usage: athena owl_vtmon <owl_0~1> <vtmon_0~1>");
			shell_error(shell, "       athena owl_vtmon <owl_0~1> max_die_temp");
			return;
		}

		// Read the data
		struct vtmon_data data;
		int ret = read_owl_vtmon_data(owl, vtmon, &data);

		if (ret == 0) {
			print_owl_vtmon_data(shell, owl, vtmon, &data);
		} else {
			const char *error_msg;
			switch (ret) {
			case -1:
				error_msg = "Invalid parameters";
				break;
			case -2:
				error_msg = "Failed to write offset address";
				break;
			case -3:
				error_msg = "Failed to read data";
				break;
			default:
				error_msg = "Unknown error";
				break;
			}
			shell_error(shell, "Failed to read OWL%d VTMON%d: %s", owl, vtmon,
				    error_msg);
		}
	}
}

void athena_temp_monitor_cmd(const struct shell *shell, size_t argc, char **argv)
{
	if (argc > 1) {
		shell_error(shell, "Usage: athena temp_monitor");
		return;
	}

	// Read temperature monitoring data
	shell_print(shell, "Reading temperature monitoring data...\n");

	struct temp_monitor_data data;
	int ret = read_temp_monitor_data(&data);

	if (ret == 0) {
		print_temp_monitor_data(shell, &data);
	} else {
		const char *error_msg;
		switch (ret) {
		case -1:
			error_msg = "Invalid parameters";
			break;
		case -2:
			error_msg = "Failed to write offset address";
			break;
		case -3:
			error_msg = "Failed to read data";
			break;
		default:
			error_msg = "Unknown error";
			break;
		}
		shell_error(shell, "Failed to read temperature monitoring data: %s", error_msg);
	}
}

void athena_hbm_temp_cmd(const struct shell *shell, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(shell, "Usage: athena hbm_temp all");
		shell_error(shell, "       athena hbm_temp <hbm_0~5>");
		shell_error(shell, "       athena hbm_temp max_temp_current");
		shell_error(shell, "       athena hbm_temp min_temp_in_sid2");
		return;
	}

	if (strcmp(argv[1], "all") == 0) {
		// Read all HBM temperature data
		shell_print(shell, "Reading all HBM temperature data...");

		for (uint8_t hbm = 0; hbm < HBM_TEMP_MAX_COUNT; hbm++) {
			struct hbm_temp_data data;
			int ret = read_hbm_temp_data(hbm, &data);

			if (ret == 0) {
				print_hbm_temp_data(shell, hbm, &data);
			} else {
				shell_error(shell, "Failed to read HBM%d temperature (error: %d)",
					    hbm, ret);
			}
		}

		// Also read HBM all max temperature
		int8_t max_temp_all_chan;
		int ret = read_hbm_all_max_temp(&max_temp_all_chan);
		if (ret == 0) {
			shell_fprintf(shell, SHELL_NORMAL, "%-20s raw: %02X -> %-12d°C\n",
				      "HBM_all_max_temp", (uint8_t)max_temp_all_chan,
				      max_temp_all_chan);
		} else {
			shell_error(shell, "Failed to read HBM all max temp (error: %d)", ret);
		}

		// Also read HBM all min temperature in SID2
		int8_t min_temp_sid2;
		ret = read_hbm_all_min_temp_sid2(&min_temp_sid2);
		if (ret == 0) {
			shell_fprintf(shell, SHELL_NORMAL, "%-20s raw: %02X -> %-12d°C\n",
				      "HBM_all_min_temp_sid2", (uint8_t)min_temp_sid2,
				      min_temp_sid2);
		} else {
			shell_error(shell, "Failed to read HBM all min temp SID2 (error: %d)", ret);
		}

	} else if (strcmp(argv[1], "max_temp_current") == 0) {
		// Read HBM all max temperature across all channels
		shell_print(shell, "Reading HBM all max temperature across all channels...");

		int8_t max_temp_all_chan;
		int ret = read_hbm_all_max_temp(&max_temp_all_chan);

		if (ret == 0) {
			shell_print(shell, "");
			shell_fprintf(shell, SHELL_NORMAL, "  %-20s raw: %02X -> %-12d°C\n",
				      "HBM_all_max_temp", (uint8_t)max_temp_all_chan,
				      max_temp_all_chan);
			shell_print(shell, "");
		} else {
			const char *error_msg;
			switch (ret) {
			case -1:
				error_msg = "Invalid parameters";
				break;
			case -2:
				error_msg = "Failed to write offset address";
				break;
			case -3:
				error_msg = "Failed to read data";
				break;
			default:
				error_msg = "Unknown error";
				break;
			}
			shell_error(shell, "Failed to read HBM all max temp: %s", error_msg);
		}

	} else if (strcmp(argv[1], "min_temp_in_sid2") == 0) {
		// Read HBM all min temperature in SID2
		shell_print(shell, "Reading HBM all min temperature in SID2...");

		int8_t min_temp_sid2;
		int ret = read_hbm_all_min_temp_sid2(&min_temp_sid2);

		if (ret == 0) {
			shell_print(shell, "");
			shell_fprintf(shell, SHELL_NORMAL, "  %-20s raw: %02X -> %-12d°C\n",
				      "HBM_all_min_temp_sid2", (uint8_t)min_temp_sid2,
				      min_temp_sid2);
			shell_print(shell, "");
		} else {
			const char *error_msg;
			switch (ret) {
			case -1:
				error_msg = "Invalid parameters";
				break;
			case -2:
				error_msg = "Failed to write offset address";
				break;
			case -3:
				error_msg = "Failed to read data";
				break;
			default:
				error_msg = "Unknown error";
				break;
			}
			shell_error(shell, "Failed to read HBM all min temp SID2: %s", error_msg);
		}

	} else {
		// Read specific HBM temperature
		if (argc != 2) {
			shell_error(shell, "Usage: athena hbm_temp <hbm_0~5>");
			return;
		}

		// Parse HBM parameter with strict format validation
		uint8_t hbm;
		if (parse_prefixed_param(argv[1], "hbm_", HBM_TEMP_MAX_COUNT, &hbm) != 0) {
			shell_error(shell, "Usage: athena hbm_temp all");
			shell_error(shell, "       athena hbm_temp <hbm_0~5>");
			shell_error(shell, "       athena hbm_temp max_temp_current");
			shell_error(shell, "       athena hbm_temp min_temp_in_sid2");
			return;
		}

		// Read the data
		struct hbm_temp_data data;
		int ret = read_hbm_temp_data(hbm, &data);

		if (ret == 0) {
			print_hbm_temp_data(shell, hbm, &data);
		} else {
			const char *error_msg;
			switch (ret) {
			case -1:
				error_msg = "Invalid parameters";
				break;
			case -2:
				error_msg = "Failed to write offset address";
				break;
			case -3:
				error_msg = "Failed to read data";
				break;
			default:
				error_msg = "Unknown error";
				break;
			}
			shell_error(shell, "Failed to read HBM%d temperature: %s", hbm, error_msg);
		}
	}
}

void athena_boot_status_cmd(const struct shell *shell, size_t argc, char **argv)
{
	if (argc > 1) {
		shell_error(shell, "Usage: athena boot_status");
		return;
	}

	// Read boot status
	shell_print(shell, "Checking Athena boot status...");

	bool is_boot1_loaded;
	int ret = read_athena_boot_status(&is_boot1_loaded);

	if (ret == 0) {
		shell_print(shell, "");
		shell_fprintf(shell, SHELL_NORMAL, "  Boot1 Status:  %s\n",
			      is_boot1_loaded ? "Loaded" : "Not Loaded");
		shell_print(shell, "");
	} else {
		const char *error_msg;
		switch (ret) {
		case -1:
			error_msg = "Invalid parameters";
			break;
		case -2:
			error_msg = "Failed to read status register";
			break;
		default:
			error_msg = "Unknown error";
			break;
		}
		shell_error(shell, "Failed to read boot status: %s", error_msg);
	}
}

void athena_efuse_cmd(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(shell, "Usage: athena efuse all");
		shell_error(shell, "       athena efuse athena");
		shell_error(shell, "       athena efuse owl_0");
		shell_error(shell, "       athena efuse owl_1");
		return;
	}

	if (strcmp(argv[1], "all") == 0) {
		// Read all efuse targets
		shell_print(shell, "Reading all efuse data...");

		const char *target_names[EFUSE_MAX_TARGETS] = { "athena", "owl_0", "owl_1" };

		for (int target_idx = 0; target_idx < EFUSE_MAX_TARGETS; target_idx++) {
			struct efuse_data data;
			int ret = read_efuse_data(target_idx, &data);

			if (ret == 0) {
				print_efuse_data(shell, target_names[target_idx], &data);
			} else {
				shell_error(shell, "Failed to read efuse data for %s (error: %d)",
					    target_names[target_idx], ret);
			}
		}
	} else {
		// Read specific efuse target
		int target_idx = -1;
		if (strcmp(argv[1], "athena") == 0) {
			target_idx = 0;
		} else if (strcmp(argv[1], "owl_0") == 0) {
			target_idx = 1;
		} else if (strcmp(argv[1], "owl_1") == 0) {
			target_idx = 2;
		} else {
			shell_error(shell, "Invalid target. Use: all, athena, owl_0, or owl_1");
			return;
		}

		// Read the efuse data
		shell_print(shell, "Reading efuse data for %s...", argv[1]);

		struct efuse_data data;
		int ret = read_efuse_data(target_idx, &data);

		if (ret == 0) {
			print_efuse_data(shell, argv[1], &data);
		} else {
			const char *error_msg;
			switch (ret) {
			case -1:
				error_msg = "Invalid parameters";
				break;
			case -2:
				error_msg = "Failed to read efuse data";
				break;
			default:
				error_msg = "Unknown error";
				break;
			}
			shell_error(shell, "Failed to read efuse data for %s: %s", argv[1],
				    error_msg);
		}
	}
}

/**
 * @brief Print ASIC version with formatting
 * @param shell Shell instance
 * @param verbose Whether to print detailed messages and formatting
 * @return 0 on success, negative error code on failure
 */
static int print_asic_version(const struct shell *shell, bool verbose)
{
	uint8_t asic_version;
	int ret = read_athena_asic_version(&asic_version);

	if (ret == 0) {
		const char *version_str;
		if (asic_version == 0x01) {
			version_str = "A0";
		} else if (asic_version == 0x02) {
			version_str = "B0";
		} else {
			version_str = "not supported";
		}

		if (verbose) {
			shell_print(shell, "");
			shell_fprintf(shell, SHELL_NORMAL, "  ASIC Version:  %s (raw = 0x%02X)\n",
				      version_str, asic_version);
			shell_print(shell, "");
		} else {
			shell_fprintf(shell, SHELL_NORMAL, "  ASIC Version:   %s (raw = 0x%02X)\n",
				      version_str, asic_version);
		}
	} else if (verbose) {
		const char *error_msg;
		switch (ret) {
		case -1:
			error_msg = "Invalid parameters";
			break;
		case -2:
			error_msg = "Failed to read ASIC version register";
			break;
		default:
			error_msg = "Unknown error";
			break;
		}
		shell_error(shell, "Failed to read ASIC version: %s", error_msg);
	} else {
		shell_fprintf(shell, SHELL_NORMAL, "  ASIC Version:   Failed to read\n");
	}

	return ret;
}

/**
 * @brief Print M7 version with formatting
 * @param shell Shell instance
 * @param verbose Whether to print detailed messages and formatting
 * @return 0 on success, negative error code on failure
 */
static int print_m7_version(const struct shell *shell, bool verbose)
{
	uint8_t major, minor, optional;
	int ret = read_athena_m7_version(&major, &minor, &optional);

	if (ret == 0) {
		if (verbose) {
			shell_print(shell, "");
			shell_fprintf(shell, SHELL_NORMAL, "  M7 Version:  v%d.%d.%d\n", major,
				      minor, optional);
			shell_print(shell, "");
		} else {
			shell_fprintf(shell, SHELL_NORMAL, "  M7 Version:     v%d.%d.%d\n", major,
				      minor, optional);
		}
	} else if (verbose) {
		const char *error_msg;
		switch (ret) {
		case -1:
			error_msg = "Invalid parameters";
			break;
		case -2:
			error_msg = "Failed to read M7 version registers";
			break;
		default:
			error_msg = "Unknown error";
			break;
		}
		shell_error(shell, "Failed to read M7 version: %s", error_msg);
	} else {
		shell_fprintf(shell, SHELL_NORMAL, "  M7 Version:     Failed to read\n");
	}

	return ret;
}

/**
 * @brief Print patch version with formatting
 * @param shell Shell instance
 * @param verbose Whether to print detailed messages and formatting
 * @return 0 on success, negative error code on failure
 */
static int print_patch_version(const struct shell *shell, bool verbose)
{
	uint8_t patch_major;
	int ret = read_athena_patch_version(&patch_major);

	if (ret == 0) {
		if (verbose) {
			shell_print(shell, "");
			shell_fprintf(shell, SHELL_NORMAL, "  Patch Version:  %d\n", patch_major);
			shell_print(shell, "");
		} else {
			shell_fprintf(shell, SHELL_NORMAL, "  Patch Version:  %d\n", patch_major);
		}
	} else if (verbose) {
		const char *error_msg;
		switch (ret) {
		case -1:
			error_msg = "Invalid parameters";
			break;
		case -2:
			error_msg = "Failed to read patch version register";
			break;
		default:
			error_msg = "Unknown error";
			break;
		}
		shell_error(shell, "Failed to read patch version: %s", error_msg);
	} else {
		shell_fprintf(shell, SHELL_NORMAL, "  Patch Version:  Failed to read\n");
	}

	return ret;
}

void athena_version_cmd(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(shell, "Usage: athena version all");
		shell_error(shell, "       athena version asic");
		shell_error(shell, "       athena version m7");
		shell_error(shell, "       athena version patch");
		return;
	}

	if (strcmp(argv[1], "all") == 0) {
		// Read all version information
		shell_print(shell, "Reading all Athena version information...");
		shell_print(shell, "");

		// Use helper functions with compact formatting
		print_asic_version(shell, false);
		print_m7_version(shell, false);
		print_patch_version(shell, false);

		shell_print(shell, "");
	} else if (strcmp(argv[1], "asic") == 0) {
		// Read ASIC version with verbose output
		shell_print(shell, "Reading Athena ASIC version...");
		print_asic_version(shell, true);
	} else if (strcmp(argv[1], "m7") == 0) {
		// Read M7 version with verbose output
		shell_print(shell, "Reading Athena M7 version...");
		print_m7_version(shell, true);
	} else if (strcmp(argv[1], "patch") == 0) {
		// Read patch version with verbose output
		shell_print(shell, "Reading Athena patch version...");
		print_patch_version(shell, true);
	} else {
		shell_error(shell, "Invalid version type. Use: all, asic, m7, or patch");
	}
}

void athena_current_temp_cmd(const struct shell *shell, size_t argc, char **argv)
{
	if (argc > 1) {
		shell_error(shell, "Usage: athena current_temp");
		return;
	}

	shell_print(shell, "Reading Athena current temperature...");

	uint8_t current_temp;
	int ret = read_athena_current_temp(&current_temp);

	if (ret == 0) {
		shell_print(shell, "");
		shell_fprintf(shell, SHELL_NORMAL, "  %-20s raw: %02X -> %d°C\n", "current_temp",
			      current_temp, current_temp);
		shell_print(shell, "");
	} else {
		const char *error_msg;
		switch (ret) {
		case -1:
			error_msg = "Invalid parameters";
			break;
		case -2:
			error_msg = "Failed to read current temperature register";
			break;
		default:
			error_msg = "Unknown error";
			break;
		}
		shell_error(shell, "Failed to read current temperature: %s", error_msg);
	}
}

void athena_max_asic_temp_cmd(const struct shell *shell, size_t argc, char **argv)
{
	if (argc > 1) {
		shell_error(shell, "Usage: athena max_asic_temp");
		return;
	}

	shell_print(shell, "Reading Athena max ASIC temperature...");

	uint8_t max_asic_temp;
	int ret = read_athena_max_asic_temp(&max_asic_temp);

	if (ret == 0) {
		shell_print(shell, "");
		shell_fprintf(shell, SHELL_NORMAL, "  %-20s raw: %02X -> %d°C\n", "max_asic_temp",
			      max_asic_temp, max_asic_temp);
		shell_print(shell, "");
	} else {
		const char *error_msg;
		switch (ret) {
		case -1:
			error_msg = "Invalid parameters";
			break;
		case -2:
			error_msg = "Failed to read max ASIC temperature register";
			break;
		default:
			error_msg = "Unknown error";
			break;
		}
		shell_error(shell, "Failed to read max ASIC temperature: %s", error_msg);
	}
}

void athena_read_all_cmd(const struct shell *shell, size_t argc, char **argv)
{
	if (argc > 1) {
		shell_error(shell, "Usage: athena read_all");
		return;
	}

	shell_print(shell, "=== Athena Dump ===");

	// 1. Boot Status
	shell_print(shell, "\n[1/8] System Boot Status");
	shell_print(shell, "=====================");
	athena_boot_status_cmd(shell, 1, (char *[]){ "boot_status" });

	// 2. HBM Temperature (all)
	shell_print(shell, "\n[2/8] HBM Temperature Readings");
	shell_print(shell, "============================");
	athena_hbm_temp_cmd(shell, 2, (char *[]){ "hbm_temp", "all" });

	// 3. OWL VTMON (all)
	shell_print(shell, "\n[3/8] OWL Voltage & Temperature Readings");
	shell_print(shell, "=======================================");
	athena_owl_vtmon_cmd(shell, 2, (char *[]){ "owl_vtmon", "all" });

	// 4. Temperature Monitor
	shell_print(shell, "\n[4/8] System Temperature Readings");
	shell_print(shell, "============================");
	athena_temp_monitor_cmd(shell, 1, (char *[]){ "temp_monitor" });

	// 5. Current Temperature
	shell_print(shell, "\n[5/8] Current Temperature Readings");
	shell_print(shell, "============================");
	athena_current_temp_cmd(shell, 1, (char *[]){ "current_temp" });

	// 6. Max ASIC Temperature
	shell_print(shell, "\n[6/8] MAX ASIC Temperature History");
	shell_print(shell, "================================");
	athena_max_asic_temp_cmd(shell, 1, (char *[]){ "max_asic_temp" });

	// 7. Efuse Data (all)
	shell_print(shell, "\n[7/8] Efuse data");
	shell_print(shell, "=================================");
	athena_efuse_cmd(shell, 2, (char *[]){ "efuse", "all" });

	// 8. Version Information (all)
	shell_print(shell, "\n[8/8] Firmware & Hardware Versions");
	shell_print(shell, "===============================");
	athena_version_cmd(shell, 2, (char *[]){ "version", "all" });

	shell_print(shell, "\n=== Athena Dump Complete ===");
}

/**
 * @brief Display help information for all Athena commands
 * @param shell Shell instance
 * @param argc Argument count
 * @param argv Argument values
 */
void athena_help_cmd(const struct shell *shell, size_t argc, char **argv)
{
	shell_error(shell, "Usage: athena help");
	shell_error(shell, "       athena read_all");
	shell_error(shell, "       athena read <region> <base_offset> <offset> <length>");
	shell_error(shell, "       athena boot_status");
	shell_error(shell, "       athena current_temp");
	shell_error(shell, "       athena max_asic_temp");
	shell_error(shell, "       athena temp_monitor");
	shell_error(shell, "       athena hbm_temp all");
	shell_error(shell, "       athena hbm_temp <hbm_0~5>");
	shell_error(shell, "       athena hbm_temp max_temp_current");
	shell_error(shell, "       athena hbm_temp min_temp_in_sid2");
	shell_error(shell, "       athena owl_vtmon all");
	shell_error(shell, "       athena owl_vtmon <owl_0~1> <vtmon_0~1>");
	shell_error(shell, "       athena owl_vtmon <owl_0~1> max_die_temp");
	shell_error(shell, "       athena efuse all");
	shell_error(shell, "       athena efuse athena");
	shell_error(shell, "       athena efuse owl_0");
	shell_error(shell, "       athena efuse owl_1");
	shell_error(shell, "       athena version all");
	shell_error(shell, "       athena version asic");
	shell_error(shell, "       athena version m7");
	shell_error(shell, "       athena version patch");
}

/* Sub-command Level 1 of athena commands */
SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_athena_cmds, SHELL_CMD(read, NULL, "low-level read command", athena_read_cmd),
	SHELL_CMD(owl_vtmon, NULL, "read OWL VTMON data command", athena_owl_vtmon_cmd),
	SHELL_CMD(temp_monitor, NULL, "read temperature monitoring data", athena_temp_monitor_cmd),
	SHELL_CMD(hbm_temp, NULL, "read HBM temperature data", athena_hbm_temp_cmd),
	SHELL_CMD(boot_status, NULL, "check Athena boot status", athena_boot_status_cmd),
	SHELL_CMD(efuse, NULL, "read efuse data", athena_efuse_cmd),
	SHELL_CMD(version, NULL, "read Athena version data", athena_version_cmd),
	SHELL_CMD(current_temp, NULL, "read Athena current temperature", athena_current_temp_cmd),
	SHELL_CMD(max_asic_temp, NULL, "read Athena max ASIC temperature",
		  athena_max_asic_temp_cmd),
	SHELL_CMD(read_all, NULL, "read all Athena system data", athena_read_all_cmd),
	SHELL_CMD(help, NULL, "display help information for Athena commands", athena_help_cmd),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(athena, &sub_athena_cmds, "athena low-level commands", NULL);
