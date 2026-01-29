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

#ifndef PLAT_I2C_SLAVE_H
#define PLAT_I2C_SLAVE_H

#include <drivers/i2c.h>
#include "hal_i2c_target.h"

#define TARGET_ENABLE 1
#define TARGET_DISABLE 0

// i2c target register
#define SENSOR_INIT_DATA_0_REG 0x00
#define SENSOR_INIT_DATA_1_REG 0x01
#define SENSOR_READING_0_REG 0x02
#define SENSOR_READING_1_REG 0x03
#define SENSOR_READING_2_REG 0x04
#define SENSOR_READING_3_REG 0x05
#define INVENTORY_IDS_REG 0x06
#define STRAP_CAPABILTITY_REG 0x08
#define WRITE_STRAP_PIN_VALUE_REG 0x09
#define I2C_BRIDGE_COMMAND_REG 0x40
#define I2C_BRIDGE_COMMAND_STATUS_REG 0x41
#define I2C_BRIDGE_COMMAND_RESPONSE_REG 0x42

#define FRU_BOARD_PART_NUMBER_REG 0x60
#define FRU_BOARD_SERIAL_NUMBER_REG 0x61
#define FRU_BOARD_PRODUCT_NAME_REG 0x62
#define FRU_BOARD_CUSTOM_DATA_1_REG 0x63
#define FRU_BOARD_CUSTOM_DATA_2_REG 0x64
#define FRU_BOARD_CUSTOM_DATA_3_REG 0x65
#define FRU_BOARD_CUSTOM_DATA_4_REG 0x66
#define FRU_BOARD_CUSTOM_DATA_5_REG 0x67
#define FRU_BOARD_CUSTOM_DATA_6_REG 0x68
#define FRU_BOARD_CUSTOM_DATA_7_REG 0x69
#define FRU_BOARD_CUSTOM_DATA_8_REG 0x6A
#define FRU_BOARD_CUSTOM_DATA_9_REG 0x6B
#define FRU_BOARD_CUSTOM_DATA_10_REG 0x6C

#define FRU_PRODUCT_NAME_REG 0x70
#define FRU_PRODUCT_PART_NUMBER_REG 0x71
#define FRU_PRODUCT_PART_VERSION_REG 0x72
#define FRU_PRODUCT_SERIAL_NUMBER_REG 0x73
#define FRU_PRODUCT_ASSET_TAG_REG 0x74
#define FRU_PRODUCT_CUSTOM_DATA_1_REG 0x75
#define FRU_PRODUCT_CUSTOM_DATA_2_REG 0x76

#define CONTROL_VOL_VR_ASIC_P0V75_VDDPHY_HBM0246_REG 0x80
#define CONTROL_VOL_VR_ASIC_P0V75_VDDPHY_HBM1357_REG 0x81
#define CONTROL_VOL_VR_ASIC_P1V1_VDDQC_HBM0246_REG 0x82
#define CONTROL_VOL_VR_ASIC_P1V1_VDDQC_HBM1357_REG 0x83
#define CONTROL_VOL_VR_ASIC_P0V4_VDDQL_HBM0246_REG 0x84
#define CONTROL_VOL_VR_ASIC_P0V4_VDDQL_HBM1357_REG 0x85
#define CONTROL_VOL_VR_ASIC_P1V8_VPP_HBM0246_REG 0x86
#define CONTROL_VOL_VR_ASIC_P1V8_VPP_HBM1357_REG 0x87
#define CONTROL_VOL_VR_ASIC_P0V85_MEDHA0_VDD_REG 0x88
#define CONTROL_VOL_VR_ASIC_P0V85_MEDHA1_VDD_REG 0x89
#define LEVEL_1_PWR_ALERT_THRESHOLD_TIME_REG 0x90
#define LEVEL_2_PWR_ALERT_THRESHOLD_TIME_REG 0x91
#define LEVEL_3_PWR_ALERT_THRESHOLD_TIME_REG 0x92
#define VR_POWER_READING_REG 0x93
#define MEDHA_SENSOR_VALUE_REG 0x94
#define POWER_CAPPING_METHOD_REG 0x95
#define MEDHA_POWER_SOURCE_REG 0x96
#define POLLING_RATE_TELEMETRY_REG 0x97
#define TRAY_INFO_REG 0x98

#define SET_SENSOR_POLLING_COMMAND_REG 0xF0

typedef struct _i2c_target_command_mapping {
	uint16_t reg; // register address
	uint8_t addr;
	uint8_t (*wr_fn)(struct _i2c_target_command_mapping *);
	uint8_t (*rd_fn)(struct _i2c_target_command_mapping *);
	uint8_t *data;
} i2c_target_command_mapping;
typedef struct __attribute__((__packed__)) {
	uint16_t carrier_board_id; //  MTIA Gen1 - Aegis=0x00
	uint32_t bic_fw_version;
	uint32_t cpld_fw_version;
} plat_inventory_ids;
// size = sizeof(plat_inventory_ids)
typedef struct __attribute__((__packed__)) {
	uint8_t device_type;
	uint8_t register_layout_version;
	uint8_t num_idx; //Number of PDR indexes in this register
	uint8_t reserved_1;

	uint16_t sbi; //Sensor base index (SBI= 248*R) used in this register's array
	uint16_t max_pdr_idx; //Max PDR sensor index exported in total (MAX_PDRIDX)

	uint8_t sensor_r_len[]; //sensor[0] reading length ~ sensor[247] reading length
} plat_sensor_init_data;
typedef struct __attribute__((__packed__)) {
	uint8_t sensor_index_offset; // Sensor index offset (e.g. PDR sensor index offset)
	uint32_t sensor_value; // Sensor value (4 bytes)
} sensor_entry;
typedef struct __attribute__((__packed__)) {
	uint8_t device_type; // Device type (Aegis = 0x01, Rainbow = 0x02)
	uint8_t register_layout_version; // Register layout version (e.g. VERSION_1 = 0x01)
	uint16_t sensor_base_index; // Sensor base index (SBI)
	uint8_t max_sbi_off; // Max sensor base index offset in this register (0 <= MAX_SBI_OFF <= 49)
	// The following is a flexible array of sensor entries.
	// The number of entries is (max_sbi_off + 1)
	sensor_entry sensor_entries[];
} plat_sensor_reading;

typedef struct _plat_sensor_reading_data_ {
	uint8_t sensor_number;
	uint8_t data[4];
} plat_sensor_reading_data;

enum VR_INFO_TYPE {
	VR_THRESHOLD,
	VR_TIME_WINDOW,
	VR_INFO_TYPE_MAX,
};

enum VR_PWR_CONTROLLER {
	MEDHA0,
	MEDHA1,
	VR_PWR_CONTROLLER_MAX,
};

enum VR_ALERT_LEVEL {
	VR_ALERT_LEVEL_1,
	VR_ALERT_LEVEL_2,
	VR_ALERT_LEVEL_3,
	VR_ALERT_MAX,
};
typedef struct {
	uint8_t threshold_lsb;
	uint8_t threshold_msb;
	uint8_t time_window_lsb;
	uint8_t time_window_msb;
	uint8_t read_data_lsb;
	uint8_t read_data_msb;
} vr_level_t;

typedef struct {
	vr_level_t level[VR_ALERT_MAX];
} vr_controller_t;

// size = sizeof(plat_sensor_reading) + num_sensors * sizeof(SensorEntry);
// num_sensors = max_sbi_off + 1
typedef struct _telemetry_info_ telemetry_info;
typedef struct _telemetry_info_ {
	uint8_t telemetry_offset;
	uint16_t data_size;
	bool (*telemetry_table_init)(telemetry_info *, uint8_t *);
} telemetry_info;
typedef struct __attribute__((__packed__)) {
	uint8_t strap_set_index;
	uint8_t strap_set_value;
	uint8_t strap_set_type;
	// b7 No Drive-Input, b6 PushPull-Output, b5 OpenDrain-Output, b4-b3: Reserved,
	// b2: DriveType(0:OpenDrain,1:PushPull), b1: Direction(0:output,1:input), b0: Reserved
} strap_entry;
typedef struct __attribute__((__packed__)) {
	uint8_t strap_data_length;
	strap_entry strap_set_format[];
} plat_strap_capability;
typedef struct __attribute__((__packed__)) {
	struct k_work work;
	uint8_t bus; // 0 base (0,1,2,3,4,..)
	uint8_t addr; // 7 bit addr
	uint8_t read_len;
	uint8_t write_len; // include data[0]
	uint8_t data[]; // data[0]: offset
} plat_i2c_bridge_command_config;
typedef struct __attribute__((__packed__)) {
	uint8_t data_status;
} plat_i2c_bridge_command_status;
typedef struct __attribute__((__packed__)) {
	uint8_t data_length;
	uint8_t response_data[];
} plat_i2c_bridge_command_response_data;
typedef enum i2c_bridge_command_error {
	I2C_BRIDGE_COMMAND_SUCCESS = 0,
	I2C_BRIDGE_COMMAND_IN_PROCESS,
	I2C_BRIDGE_COMMAND_FAILURE,
} i2c_bridge_command_error;
typedef struct voltage_rail_mapping_sensor {
	uint8_t control_vol_reg;
	uint8_t vr_rail_e;
} voltage_rail_mapping_sensor;
typedef struct __attribute__((__packed__)) {
	struct k_work work;
	uint8_t rail;
	uint16_t set_value;
} plat_control_voltage;
typedef struct __attribute__((__packed__)) {
	struct k_work work;
	uint8_t lv;
	uint8_t in_data[8];
} plat_power_capping_threshold_time_t;
typedef struct __attribute__((__packed__)) {
	struct k_work work;
	uint8_t set_value;
} plat_power_capping_method_t;
typedef struct __attribute__((__packed__)) {
	struct k_work work;
	uint8_t set_value;
} plat_control_sensor_polling;
void plat_telemetry_table_init(void);
void update_sensor_reading_by_sensor_number(uint8_t sensor_number);
void update_strap_capability_table(void);
int get_cached_sensor_reading_by_sensor_number(uint8_t sensor_number);
#endif
