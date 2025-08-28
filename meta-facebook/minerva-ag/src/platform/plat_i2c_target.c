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

/*
  NAME: I2C TARGET INIT
  FILE: plat_i2c_target.c
  DESCRIPTION: Provide i2c target EN/CFG table "I2C_TARGET_EN_TABLE[]/I2C_TARGET_CFG_TABLE[]" for init target config.
  AUTHOR: MouchenHung
  DATE/VERSION: 2021.11.26 - v1.1
  Note: 
    (1) "plat_i2c_target.h" is included by "hal_i2c_target.h"
*/

#include <zephyr.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "libutil.h"
#include "plat_i2c_target.h"
#include "plat_i2c.h"
#include <logging/log.h>
#include "plat_gpio.h"
#include "plat_pldm_sensor.h"
#include "pldm_sensor.h"
#include "plat_version.h"
#include "plat_isr.h"
#include "plat_hook.h"
#include "plat_fru.h"

#define DEVICE_TYPE 0x01
#define REGISTER_LAYOUT_VERSION 0x01
#define AEGIS_CARRIER_BOARD_ID 0x0000
#define AEGIS_CPLD_ADDR (0x4C >> 1)
#define DATA_TABLE_LENGTH_1 1
#define DATA_TABLE_LENGTH_2 2
#define DATA_TABLE_LENGTH_4 4
#define DATA_TABLE_LENGTH_7 7
#define DATA_TABLE_LENGTH_13 13
#define SENSOR_INIT_PDR_INDEX_MAX 248 // PDR indexe is on 0 base
#define SENSOR_READING_PDR_INDEX_MAX 50
#define PLAT_MASTER_WRITE_STACK_SIZE 1024
#define STRAP_SET_TYPE 0x44 // 01000100
#define TELEMETRY_MSG_MAX_LENGTH 32

static bool command_reply_data_handle(void *arg);
void set_bootstrap_element_handler();
K_WORK_DEFINE(set_bootstrap_element_work, set_bootstrap_element_handler);

K_THREAD_STACK_DEFINE(plat_master_write_stack, PLAT_MASTER_WRITE_STACK_SIZE);
struct k_thread plat_master_write_thread;
k_tid_t plat_master_write_tid;

typedef struct __attribute__((__packed__)) {
	uint8_t device_type;
	uint8_t register_layout_version;
	uint8_t num_idx; //Number of PDR indexes in this register
	uint8_t reserved_1;

	uint16_t sbi; //Sensor base index (SBI= 248*R) used in this register's array
	uint16_t max_pdr_idx; //Max PDR sensor index exported in total (MAX_PDRIDX)

	uint8_t sensor_r_len[]; //sensor[0] reading length ~ sensor[247] reading length
} plat_sensor_init_data;
// size = sizeof(plat_sensor_init_data) + num_idx

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
// size = sizeof(plat_sensor_reading) + num_sensors * sizeof(SensorEntry);
// num_sensors = max_sbi_off + 1

typedef struct __attribute__((__packed__)) {
	uint16_t carrier_board_id; //  MTIA Gen1 - Aegis=0x00
	uint32_t bic_fw_version;
	uint32_t cpld_fw_version;
} plat_inventory_ids;
// size = sizeof(plat_inventory_ids)

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
	uint8_t data_length;
	uint8_t fru_data[];
} plat_fru_data;

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

typedef struct __attribute__((__packed__)) {
	struct k_work work;
	uint8_t set_value;
} plat_control_sensor_polling;

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
	uint8_t power_capping_reg;
	uint8_t sensor_id;
	uint16_t sensor_value; // Sensor value (2 bytes)
} plat_power_capping;

typedef struct __attribute__((__packed__)) {
	struct k_work work;
	uint16_t set_value_HC;
	uint16_t set_value_LC;
} plat_power_capping_set;

static uint8_t bootstrap_pin;
static uint8_t user_setting_level;

LOG_MODULE_REGISTER(plat_i2c_target);
/* I2C target init-enable table */
const bool I2C_TARGET_ENABLE_TABLE[MAX_TARGET_NUM] = {
	TARGET_DISABLE, TARGET_DISABLE, TARGET_DISABLE, TARGET_ENABLE,
	TARGET_DISABLE, TARGET_DISABLE, TARGET_ENABLE,	TARGET_DISABLE,
	TARGET_DISABLE, TARGET_DISABLE, TARGET_DISABLE, TARGET_DISABLE,
};

/* I2C target init-config table */
const struct _i2c_target_config I2C_TARGET_CONFIG_TABLE[MAX_TARGET_NUM] = {
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0x40, 0xA },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0x40, 0xA, command_reply_data_handle },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
};

// clang-format off
voltage_rail_mapping_sensor voltage_rail_mapping_table[] = {
	{ CONTROL_VOL_Minerva_Aegis_VR_ASIC_P0V75_VDDPHY_HBM0_HBM2_HBM4_REG, VR_RAIL_E_P0V75_VDDPHY_HBM0_HBM2_HBM4 },
	{ CONTROL_VOL_Minerva_Aegis_VR_ASIC_P0V75_VDDPHY_HBM1_HBM3_HBM5_REG, VR_RAIL_E_P0V75_VDDPHY_HBM1_HBM3_HBM5 },
	{ CONTROL_VOL_Minerva_Aegis_VR_ASIC_P1V1_VDDC_HBM0_HBM2_HBM4_REG, VR_RAIL_E_P1V1_VDDC_HBM0_HBM2_HBM4 },
	{ CONTROL_VOL_Minerva_Aegis_VR_ASIC_P1V1_VDDC_HBM1_HBM3_HBM5_REG, VR_RAIL_E_P1V1_VDDC_HBM1_HBM3_HBM5 },
	{ CONTROL_VOL_Minerva_Aegis_VR_ASIC_P0V4_VDDQL_HBM0_HBM2_HBM4_REG, VR_RAIL_E_P0V4_VDDQL_HBM0_HBM2_HBM4 },
	{ CONTROL_VOL_Minerva_Aegis_VR_ASIC_P0V4_VDDQL_HBM1_HBM3_HBM5_REG, VR_RAIL_E_P0V4_VDDQL_HBM1_HBM3_HBM5 },
	{ CONTROL_VOL_Minerva_Aegis_VR_ASIC_P1V8_VPP_HBM0_HBM2_HBM4_REG, VR_RAIL_E_P1V8_VPP_HBM0_HBM2_HBM4 },
	{ CONTROL_VOL_Minerva_Aegis_VR_ASIC_P1V8_VPP_HBM1_HBM3_HBM5_REG, VR_RAIL_E_P1V8_VPP_HBM1_HBM3_HBM5 },
};

plat_power_capping plat_power_capping_table[] = {
	{ POWER_CAPPING_GET_VR_ASIC_P0V85_PVDD_PWR_W_REG, VR_ASIC_P0V85_PVDD_PWR_W, 0x0000 },
	{ POWER_CAPPING_GET_VR_ASIC_P0V4_VDDQL_HBM0_HBM2_HBM4_PWR_W_REG, VR_ASIC_P0V4_VDDQL_HBM0_HBM2_HBM4_PWR_W, 0x0000 },
	{ POWER_CAPPING_GET_VR_ASIC_P0V75_VDDPHY_HBM0_HBM2_HBM4_PWR_W_REG, VR_ASIC_P0V75_VDDPHY_HBM0_HBM2_HBM4_PWR_W, 0x0000 },
	{ POWER_CAPPING_GET_VR_ASIC_P0V4_VDDQL_HBM1_HBM3_HBM5_PWR_W_REG, VR_ASIC_P0V4_VDDQL_HBM1_HBM3_HBM5_PWR_W, 0x0000 },
	{ POWER_CAPPING_GET_VR_ASIC_P0V75_VDDPHY_HBM1_HBM3_HBM5_PWR_W_REG, VR_ASIC_P0V75_VDDPHY_HBM1_HBM3_HBM5_PWR_W, 0x0000 },
	{ POWER_CAPPING_GET_VR_ASIC_P0V75_MAX_PHY_N_PWR_W_REG, VR_ASIC_P0V75_MAX_PHY_N_PWR_W, 0x0000 },
	{ POWER_CAPPING_GET_VR_ASIC_P0V75_MAX_PHY_S_PWR_W_REG, VR_ASIC_P0V75_MAX_PHY_S_PWR_W, 0x0000 },
};
// clang-format on

plat_sensor_init_data *sensor_init_data_table[DATA_TABLE_LENGTH_2] = { NULL };
plat_sensor_reading *sensor_reading_table[DATA_TABLE_LENGTH_4] = { NULL };
plat_inventory_ids *inventory_ids_table[DATA_TABLE_LENGTH_1] = { NULL };
plat_strap_capability *strap_capability_table[DATA_TABLE_LENGTH_1] = { NULL };
plat_fru_data *fru_board_data_table[DATA_TABLE_LENGTH_13] = { NULL };
plat_fru_data *fru_product_data_table[DATA_TABLE_LENGTH_7] = { NULL };
plat_i2c_bridge_command_status *i2c_bridge_command_status_table[DATA_TABLE_LENGTH_1] = { NULL };
plat_i2c_bridge_command_response_data
	*i2c_bridge_command_response_data_table[DATA_TABLE_LENGTH_1] = { NULL };

void *allocate_table(void **buffer, size_t buffer_size)
{
	if (*buffer) {
		free(*buffer);
		*buffer = NULL;
	}

	*buffer = malloc(buffer_size);
	if (!*buffer) {
		LOG_ERR("Memory allocation failed!");
		return NULL;
	}
	return *buffer;
}

uint8_t get_vr_rail_by_control_vol_reg(uint8_t control_vol_reg)
{
	for (int i = 0; i < ARRAY_SIZE(voltage_rail_mapping_table); i++) {
		if (voltage_rail_mapping_table[i].control_vol_reg == control_vol_reg) {
			return voltage_rail_mapping_table[i].vr_rail_e;
		}
	}
	return VR_RAIL_E_MAX;
}

void set_control_voltage_handler(struct k_work *work_item)
{
	const plat_control_voltage *sensor_data =
		CONTAINER_OF(work_item, plat_control_voltage, work);
	uint8_t rail = sensor_data->rail;
	uint16_t millivolt = sensor_data->set_value;
	LOG_DBG("Setting rail %x to %d mV", rail, millivolt);

	plat_set_vout_command(rail, &millivolt, false, false);
}

void set_power_capping_handler(struct k_work *work_item)
{
	const plat_power_capping_set *sensor_data =
		CONTAINER_OF(work_item, plat_power_capping_set, work);

	uint16_t set_value_HC = sensor_data->set_value_HC;
	uint16_t set_value_LC = sensor_data->set_value_LC;
	plat_set_power_capping_command(POWER_CAPPING_INDEX_HC, &set_value_HC);
	plat_set_power_capping_command(POWER_CAPPING_INDEX_LC, &set_value_LC);
	// LOG_DBG("Power capping set HC: %d, LC: %d", set_value_HC, set_value_LC);
}

bool get_fru_info_element(telemetry_info *telemetry_info, char **fru_element,
			  uint8_t *fru_element_size)
{
	CHECK_NULL_ARG_WITH_RETURN(telemetry_info, false);

	FRU_INFO *plat_fru_info = get_fru_info();
	if (!plat_fru_info)
		return false;

	switch (telemetry_info->telemetry_offset) {
	case FRU_BOARD_PART_NUMBER_REG:
		*fru_element = plat_fru_info->board.board_part_number;
		break;
	case FRU_BOARD_SERIAL_NUMBER_REG:
		*fru_element = plat_fru_info->board.board_serial;
		break;
	case FRU_BOARD_PRODUCT_NAME_REG:
		*fru_element = plat_fru_info->board.board_product;
		break;
	case FRU_BOARD_CUSTOM_DATA_1_REG:
		*fru_element = plat_fru_info->board.board_custom_data[0];
		break;
	case FRU_BOARD_CUSTOM_DATA_2_REG:
		*fru_element = plat_fru_info->board.board_custom_data[1];
		break;
	case FRU_BOARD_CUSTOM_DATA_3_REG:
		*fru_element = plat_fru_info->board.board_custom_data[2];
		break;
	case FRU_BOARD_CUSTOM_DATA_4_REG:
		*fru_element = plat_fru_info->board.board_custom_data[3];
		break;
	case FRU_BOARD_CUSTOM_DATA_5_REG:
		*fru_element = plat_fru_info->board.board_custom_data[4];
		break;
	case FRU_BOARD_CUSTOM_DATA_6_REG:
		*fru_element = plat_fru_info->board.board_custom_data[5];
		break;
	case FRU_BOARD_CUSTOM_DATA_7_REG:
		*fru_element = plat_fru_info->board.board_custom_data[6];
		break;
	case FRU_BOARD_CUSTOM_DATA_8_REG:
		*fru_element = plat_fru_info->board.board_custom_data[7];
		break;
	case FRU_BOARD_CUSTOM_DATA_9_REG:
		*fru_element = plat_fru_info->board.board_custom_data[8];
		break;
	case FRU_BOARD_CUSTOM_DATA_10_REG:
		*fru_element = plat_fru_info->board.board_custom_data[9];
		break;
	case FRU_PRODUCT_NAME_REG:
		*fru_element = plat_fru_info->product.product_name;
		break;
	case FRU_PRODUCT_PART_NUMBER_REG:
		*fru_element = plat_fru_info->product.product_part_number;
		break;
	case FRU_PRODUCT_PART_VERSION_REG:
		*fru_element = plat_fru_info->product.product_version;
		break;
	case FRU_PRODUCT_SERIAL_NUMBER_REG:
		*fru_element = plat_fru_info->product.product_serial;
		break;
	case FRU_PRODUCT_ASSET_TAG_REG:
		*fru_element = plat_fru_info->product.product_asset_tag;
		break;
	case FRU_PRODUCT_CUSTOM_DATA_1_REG:
		*fru_element = plat_fru_info->product.product_custom_data[0];
		break;
	case FRU_PRODUCT_CUSTOM_DATA_2_REG:
		*fru_element = plat_fru_info->product.product_custom_data[1];
		break;
	default:
		LOG_ERR("Unknown reg offset: 0x%02x", telemetry_info->telemetry_offset);
		break;
	}
	if (*fru_element) {
		*fru_element_size = (uint8_t)strlen(*fru_element);
	} else {
		*fru_element_size = 0;
	}
	return true;
}

bool set_bootstrap_element(uint8_t bootstrap_pin, uint8_t user_setting_level)
{
	uint8_t change_setting_value;
	uint8_t drive_index_level = user_setting_level;
	bootstrap_mapping_register bootstrap_item;
	if (!set_bootstrap_table_and_user_settings(bootstrap_pin, &change_setting_value,
						   drive_index_level, false, false)) {
		LOG_ERR("set bootstrap_table[%02x]:%d failed", bootstrap_pin, drive_index_level);
		return false;
	}
	if (!find_bootstrap_by_rail(bootstrap_pin, &bootstrap_item)) {
		LOG_ERR("Can't find bootstrap_item by bootstrap_pin index: 0x%02x", bootstrap_pin);
		return false;
	}
	// LOG_DBG("set bootstrap_table[%2x]=%x, cpld_offsets 0x%02x change_setting_value 0x%02x",
	// 	bootstrap_pin, drive_index_level, bootstrap_item.cpld_offsets,
	// 	change_setting_value);
	if (!plat_i2c_write(I2C_BUS5, AEGIS_CPLD_ADDR, bootstrap_item.cpld_offsets,
			    &change_setting_value, 1)) {
		LOG_ERR("Can't write bootstrap[0x%02x]=%x, cpld_offsets 0x%02x change_setting_value 0x%02x",
			bootstrap_pin, user_setting_level, bootstrap_item.cpld_offsets,
			change_setting_value);
		return false;
	}
	return true;
}

void set_bootstrap_element_handler()
{
	if (bootstrap_pin >= STRAP_INDEX_MAX) {
		LOG_ERR("bootstrap_pin[%02x] is out of range", bootstrap_pin);
		return;
	}
	if (!set_bootstrap_element(bootstrap_pin, user_setting_level)) {
		LOG_ERR("set_bootstrap_element fail");
		return;
	}
}

void i2c_bridge_command_handler(struct k_work *work_item)
{
	const plat_i2c_bridge_command_config *sensor_data_config =
		CONTAINER_OF(work_item, plat_i2c_bridge_command_config, work);

	int response_data_len = sensor_data_config->read_len;

	size_t table_size_41 = sizeof(plat_i2c_bridge_command_status);
	plat_i2c_bridge_command_status *sensor_data_status =
		allocate_table((void **)&i2c_bridge_command_status_table[0], table_size_41);
	if (!sensor_data_status)
		return;

	size_t table_size_42 =
		sizeof(plat_i2c_bridge_command_response_data) + response_data_len * sizeof(uint8_t);
	plat_i2c_bridge_command_response_data *sensor_data_response =
		allocate_table((void **)&i2c_bridge_command_response_data_table[0], table_size_42);
	if (!sensor_data_response)
		return;

	sensor_data_status->data_status = I2C_BRIDGE_COMMAND_IN_PROCESS;
	sensor_data_response->data_length = 0x00;

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = sensor_data_config->bus;
	i2c_msg.target_addr = sensor_data_config->addr;
	i2c_msg.tx_len = sensor_data_config->write_len;
	i2c_msg.rx_len = sensor_data_config->read_len;
	memcpy(&i2c_msg.data, sensor_data_config->data, sensor_data_config->write_len);
	if (response_data_len == 0) {
		if (i2c_master_write(&i2c_msg, retry)) {
			LOG_ERR("Failed to write reg, bus: %d, addr: 0x%x, tx_len: 0x%x",
				i2c_msg.bus, i2c_msg.target_addr, i2c_msg.tx_len);
			sensor_data_status->data_status = I2C_BRIDGE_COMMAND_FAILURE;
			return;
		}
		sensor_data_status->data_status = I2C_BRIDGE_COMMAND_SUCCESS;
		sensor_data_response->data_length = response_data_len;
	} else {
		if (i2c_master_read(&i2c_msg, retry)) {
			LOG_ERR("Failed to read reg, bus: %d, addr: 0x%x, tx_len: 0x%x",
				i2c_msg.bus, i2c_msg.target_addr, i2c_msg.tx_len);
			sensor_data_status->data_status = I2C_BRIDGE_COMMAND_FAILURE;
			return;
		}
		sensor_data_status->data_status = I2C_BRIDGE_COMMAND_SUCCESS;
		sensor_data_response->data_length = response_data_len;
		memcpy(sensor_data_response->response_data, i2c_msg.data, response_data_len);
	}
}

void set_sensor_polling_handler(struct k_work *work_item)
{
	const plat_control_sensor_polling *sensor_data =
		CONTAINER_OF(work_item, plat_control_sensor_polling, work);

	int value = sensor_data->set_value;
	if (!(value == 0 || value == 1)) {
		LOG_ERR("set sensor_polling:%x is out of range", value);
		return;
	}
	set_plat_sensor_polling_enable_flag(value);
}

bool initialize_sensor_data(telemetry_info *telemetry_info, uint8_t *buffer_size)
{
	CHECK_NULL_ARG_WITH_RETURN(telemetry_info, false);

	int table_index = telemetry_info->telemetry_offset - SENSOR_INIT_DATA_0_REG;
	if (table_index < 0 || table_index >= DATA_TABLE_LENGTH_2)
		return false;

	// Calculate num_idx
	int num_idx = (PLAT_SENSOR_NUM_MAX - 1) - (SENSOR_INIT_PDR_INDEX_MAX * table_index);
	num_idx = (num_idx > 0) ?
			  ((num_idx > SENSOR_INIT_PDR_INDEX_MAX) ? SENSOR_INIT_PDR_INDEX_MAX :
								   num_idx) :
			  0;

	// Calculate the memory size
	size_t table_size = sizeof(plat_sensor_init_data) + num_idx * sizeof(uint8_t);
	plat_sensor_init_data *sensor_data =
		allocate_table((void **)&sensor_init_data_table[table_index], table_size);
	if (!sensor_data)
		return false;

	sensor_data->device_type = DEVICE_TYPE;
	sensor_data->register_layout_version = REGISTER_LAYOUT_VERSION;
	sensor_data->num_idx = num_idx;
	sensor_data->reserved_1 = 0xFF;
	sensor_data->sbi = table_index * SENSOR_INIT_PDR_INDEX_MAX;
	sensor_data->max_pdr_idx = (table_index == 0x00) ? PLAT_SENSOR_NUM_MAX - 2 : 0xFFFF;
	memset(sensor_data->sensor_r_len, 4, num_idx * sizeof(uint8_t));

	*buffer_size = (uint8_t)table_size;
	return true;
}

bool initialize_sensor_reading(telemetry_info *telemetry_info, uint8_t *buffer_size)
{
	CHECK_NULL_ARG_WITH_RETURN(telemetry_info, false);

	int table_index = telemetry_info->telemetry_offset - SENSOR_READING_0_REG;
	if (table_index < 0 || table_index >= DATA_TABLE_LENGTH_4)
		return false;

	int num_idx = (PLAT_SENSOR_NUM_MAX - 1) - (SENSOR_READING_PDR_INDEX_MAX * table_index);
	num_idx = (num_idx > 0) ?
			  ((num_idx > SENSOR_READING_PDR_INDEX_MAX) ? SENSOR_READING_PDR_INDEX_MAX :
								      num_idx) :
			  0;

	size_t table_size = sizeof(plat_sensor_reading) + num_idx * sizeof(sensor_entry);
	plat_sensor_reading *sensor_data =
		allocate_table((void **)&sensor_reading_table[table_index], table_size);
	if (!sensor_data)
		return false;

	sensor_data->device_type = DEVICE_TYPE;
	sensor_data->register_layout_version = REGISTER_LAYOUT_VERSION;
	sensor_data->sensor_base_index = table_index * SENSOR_READING_PDR_INDEX_MAX;
	sensor_data->max_sbi_off = (num_idx > 0) ? num_idx - 1 : 0;
	for (int i = 0; i < num_idx; i++) {
		sensor_data->sensor_entries[i].sensor_index_offset =
			i; // sensor_index_offset range: 0~49
		sensor_data->sensor_entries[i].sensor_value = 0x00000000;
	}

	*buffer_size = (uint8_t)table_size;
	return true;
}

bool initialize_inventory_ids(telemetry_info *telemetry_info, uint8_t *buffer_size)
{
	CHECK_NULL_ARG_WITH_RETURN(telemetry_info, false);

	int table_index = telemetry_info->telemetry_offset - INVENTORY_IDS_REG;
	if (table_index < 0 || table_index >= DATA_TABLE_LENGTH_1)
		return false;

	size_t table_size = sizeof(plat_inventory_ids);
	plat_inventory_ids *sensor_data =
		allocate_table((void **)&inventory_ids_table[table_index], table_size);
	if (!sensor_data)
		return false;

	uint8_t data[4] = { 0 };
	uint32_t bic_version = 0;
	uint32_t cpld_version = 0;
	if (!plat_i2c_read(I2C_BUS5, AEGIS_CPLD_ADDR, 0x44, data, 4)) {
		LOG_ERR("Failed to read cpld version from cpld");
		return false;
	}
	cpld_version = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
	bic_version =
		(BIC_FW_YEAR_MSB << 24) | (BIC_FW_YEAR_LSB << 16) | (BIC_FW_WEEK << 8) | BIC_FW_VER;

	sensor_data->carrier_board_id = AEGIS_CARRIER_BOARD_ID;
	sensor_data->bic_fw_version = bic_version;
	sensor_data->cpld_fw_version = cpld_version;

	*buffer_size = (uint8_t)table_size;
	return true;
}

bool initialize_strap_capability(telemetry_info *telemetry_info, uint8_t *buffer_size)
{
	CHECK_NULL_ARG_WITH_RETURN(telemetry_info, false);

	int table_index = telemetry_info->telemetry_offset - STRAP_CAPABILTITY_REG;
	if (table_index < 0 || table_index >= DATA_TABLE_LENGTH_1)
		return false;

	int num_idx = STRAP_INDEX_MAX;

	size_t table_size = sizeof(plat_strap_capability) + num_idx * sizeof(strap_entry);
	plat_strap_capability *sensor_data =
		allocate_table((void **)&strap_capability_table[table_index], table_size);
	if (!sensor_data)
		return false;

	sensor_data->strap_data_length = STRAP_INDEX_MAX * 3; // strap_entry data length
	for (int i = 0; i < STRAP_INDEX_MAX; i++) {
		sensor_data->strap_set_format[i].strap_set_index = i;
		sensor_data->strap_set_format[i].strap_set_type = STRAP_SET_TYPE;
		int drive_level = 0;
		if (!get_bootstrap_change_drive_level(i, &drive_level)) {
			LOG_ERR("Can't get_bootstrap_change_drive_level by index: %x", i);
			continue;
		}
		if (drive_level < 0) {
			LOG_ERR("Invalid drive_level: %x", drive_level);
			continue;
		}
		sensor_data->strap_set_format[i].strap_set_value = drive_level;
	}

	*buffer_size = (uint8_t)table_size;
	return true;
}

bool initialize_fru_board_data(telemetry_info *telemetry_info, uint8_t *buffer_size)
{
	CHECK_NULL_ARG_WITH_RETURN(telemetry_info, false);

	int table_index = telemetry_info->telemetry_offset - FRU_BOARD_PART_NUMBER_REG;
	if (table_index < 0 || table_index >= DATA_TABLE_LENGTH_13)
		return false;

	char *fru_string = NULL;
	uint8_t fru_length = 0;
	if (!get_fru_info_element(telemetry_info, &fru_string, &fru_length)) {
		LOG_ERR("Failed to retrieve FRU Element");
	}

	size_t table_size = sizeof(plat_fru_data) + fru_length;
	plat_fru_data *sensor_data =
		allocate_table((void **)&fru_board_data_table[table_index], table_size);
	if (!sensor_data)
		return false;

	sensor_data->data_length = fru_length;
	memcpy(sensor_data->fru_data, fru_string, fru_length);

	*buffer_size = (uint8_t)table_size;
	return true;
}

bool initialize_fru_product_data(telemetry_info *telemetry_info, uint8_t *buffer_size)
{
	CHECK_NULL_ARG_WITH_RETURN(telemetry_info, false);

	int table_index = telemetry_info->telemetry_offset - FRU_PRODUCT_NAME_REG;
	if (table_index < 0 || table_index >= DATA_TABLE_LENGTH_7)
		return false;

	char *fru_string = NULL;
	uint8_t fru_length = 0;
	if (!get_fru_info_element(telemetry_info, &fru_string, &fru_length)) {
		LOG_ERR("Failed to retrieve FRU Element");
	}

	size_t table_size = sizeof(plat_fru_data) + fru_length;
	plat_fru_data *sensor_data =
		allocate_table((void **)&fru_product_data_table[table_index], table_size);
	if (!sensor_data)
		return false;

	sensor_data->data_length = fru_length;
	memcpy(sensor_data->fru_data, fru_string, fru_length);

	*buffer_size = (uint8_t)table_size;
	return true;
}

void update_sensor_reading_table()
{
	for (int table_index = 0; table_index < DATA_TABLE_LENGTH_4; table_index++) {
		if (!sensor_reading_table[table_index])
			continue;

		plat_sensor_reading *sensor_data = sensor_reading_table[table_index];
		int num_idx = sensor_data->max_sbi_off + 1;

		for (int i = 0; i < num_idx; i++) {
			int sensor_number =
				sensor_data->sensor_base_index + i + 1; // sensor number is 1 base
			uint8_t status = SENSOR_UNAVAILABLE;
			int reading = 0;
			uint8_t sensor_operational_state = PLDM_SENSOR_STATUSUNKOWN;

			status = pldm_sensor_get_reading_from_cache(sensor_number, &reading,
								    &sensor_operational_state);
			sensor_data->sensor_entries[i].sensor_value =
				(status == SENSOR_READ_SUCCESS) ? reading : 0xFFFFFFFF;
		}
	}
}

void update_strap_capability_table()
{
	if (!strap_capability_table[0])
		return;

	plat_strap_capability *sensor_data = strap_capability_table[0];

	for (int i = 0; i < STRAP_INDEX_MAX; i++) {
		int drive_level = 0;
		if (!get_bootstrap_change_drive_level(i, &drive_level)) {
			LOG_ERR("Can't get_bootstrap_change_drive_level by index: %x", i);
			continue;
		}
		if (drive_level < 0) {
			LOG_ERR("Invalid drive_level: %x", drive_level);
			continue;
		}
		sensor_data->strap_set_format[i].strap_set_value = drive_level;
	}
}

void update_plat_power_capping_table()
{
	for (int i = 0; i < ARRAY_SIZE(plat_power_capping_table); i++) {
		int sensor_number = plat_power_capping_table[i].sensor_id;
		uint8_t status = SENSOR_UNAVAILABLE;
		int reading = 0; // unit: mW
		uint8_t sensor_operational_state = PLDM_SENSOR_STATUSUNKOWN;
		status = pldm_sensor_get_reading_from_cache(sensor_number, &reading,
							    &sensor_operational_state);
		uint16_t transfer_reading;

		if (status == SENSOR_READ_SUCCESS) {
			float watt = reading / 1000.0f;
			if (watt < 0) {
				transfer_reading = 0xFFFF;
			} else {
				transfer_reading = (uint16_t)watt;
			}
		} else {
			transfer_reading = 0xFFFF;
		}
		plat_power_capping_table[i].sensor_value = transfer_reading;
	}
}

// clang-format off
telemetry_info telemetry_info_table[] = {
	{ SENSOR_INIT_DATA_0_REG, 0x00, .telemetry_table_init = initialize_sensor_data },
	{ SENSOR_INIT_DATA_1_REG, 0x00, .telemetry_table_init = initialize_sensor_data },
	{ SENSOR_READING_0_REG, 0x00, .telemetry_table_init = initialize_sensor_reading },
	{ SENSOR_READING_1_REG, 0x00, .telemetry_table_init = initialize_sensor_reading },
	{ SENSOR_READING_2_REG, 0x00, .telemetry_table_init = initialize_sensor_reading },
	{ SENSOR_READING_3_REG, 0x00, .telemetry_table_init = initialize_sensor_reading },
	{ INVENTORY_IDS_REG, 0x00, .telemetry_table_init = initialize_inventory_ids },
	{ STRAP_CAPABILTITY_REG, 0x00, .telemetry_table_init = initialize_strap_capability },
	{ WRITE_STRAP_PIN_VALUE_REG },
	{ I2C_BRIDGE_COMMAND_REG },
	{ I2C_BRIDGE_COMMAND_STATUS_REG },
	{ I2C_BRIDGE_COMMAND_RESPONSE_REG },
	{ FRU_BOARD_PART_NUMBER_REG, 0x00, .telemetry_table_init = initialize_fru_board_data },
	{ FRU_BOARD_SERIAL_NUMBER_REG, 0x00, .telemetry_table_init = initialize_fru_board_data },
	{ FRU_BOARD_PRODUCT_NAME_REG, 0x00, .telemetry_table_init = initialize_fru_board_data },
	{ FRU_BOARD_CUSTOM_DATA_1_REG, 0x00, .telemetry_table_init = initialize_fru_board_data },
	{ FRU_BOARD_CUSTOM_DATA_2_REG, 0x00, .telemetry_table_init = initialize_fru_board_data },
	{ FRU_BOARD_CUSTOM_DATA_3_REG, 0x00, .telemetry_table_init = initialize_fru_board_data },
	{ FRU_BOARD_CUSTOM_DATA_4_REG, 0x00, .telemetry_table_init = initialize_fru_board_data },
	{ FRU_BOARD_CUSTOM_DATA_5_REG, 0x00, .telemetry_table_init = initialize_fru_board_data },
	{ FRU_BOARD_CUSTOM_DATA_6_REG, 0x00, .telemetry_table_init = initialize_fru_board_data },
	{ FRU_BOARD_CUSTOM_DATA_7_REG, 0x00, .telemetry_table_init = initialize_fru_board_data },
	{ FRU_BOARD_CUSTOM_DATA_8_REG, 0x00, .telemetry_table_init = initialize_fru_board_data },
	{ FRU_BOARD_CUSTOM_DATA_9_REG, 0x00, .telemetry_table_init = initialize_fru_board_data },
	{ FRU_BOARD_CUSTOM_DATA_10_REG, 0x00, .telemetry_table_init = initialize_fru_board_data },
	{ FRU_PRODUCT_NAME_REG, 0x00, .telemetry_table_init = initialize_fru_product_data },
	{ FRU_PRODUCT_PART_NUMBER_REG, 0x00, .telemetry_table_init = initialize_fru_product_data },
	{ FRU_PRODUCT_PART_VERSION_REG, 0x00, .telemetry_table_init = initialize_fru_product_data },
	{ FRU_PRODUCT_SERIAL_NUMBER_REG, 0x00, .telemetry_table_init = initialize_fru_product_data },
	{ FRU_PRODUCT_ASSET_TAG_REG, 0x00, .telemetry_table_init = initialize_fru_product_data },
	{ FRU_PRODUCT_CUSTOM_DATA_1_REG, 0x00, .telemetry_table_init = initialize_fru_product_data },
	{ FRU_PRODUCT_CUSTOM_DATA_2_REG, 0x00, .telemetry_table_init = initialize_fru_product_data },
	{ POWER_CAPPING_GET_VR_ASIC_P0V85_PVDD_PWR_W_REG, 0x00},
	{ POWER_CAPPING_GET_VR_ASIC_P0V4_VDDQL_HBM0_HBM2_HBM4_PWR_W_REG, 0x00 },
	{ POWER_CAPPING_GET_VR_ASIC_P0V75_VDDPHY_HBM0_HBM2_HBM4_PWR_W_REG, 0x00 },
	{ POWER_CAPPING_GET_VR_ASIC_P0V4_VDDQL_HBM1_HBM3_HBM5_PWR_W_REG, 0x00 },
	{ POWER_CAPPING_GET_VR_ASIC_P0V75_VDDPHY_HBM1_HBM3_HBM5_PWR_W_REG },
	{ POWER_CAPPING_GET_VR_ASIC_P0V75_MAX_PHY_N_PWR_W_REG, 0x00 },
	{ POWER_CAPPING_GET_VR_ASIC_P0V75_MAX_PHY_S_PWR_W_REG, 0x00 },
};
// clang-format on

static bool command_reply_data_handle(void *arg)
{
	struct i2c_target_data *data = (struct i2c_target_data *)arg;

	/*TODO: put board telemetry here*/

	/* Only check fisrt byte from received data */
	if (data->wr_buffer_idx >= 1) {
		if (data->wr_buffer_idx == 1) {
			uint8_t reg_offset = data->target_wr_msg.msg[0];
			size_t struct_size = 0;
			for (int i = 0; i < ARRAY_SIZE(telemetry_info_table); i++) {
				if (telemetry_info_table[i].telemetry_offset == reg_offset) {
					struct_size = telemetry_info_table[i].data_size;
					break;
				}
			}
			// Make sure the target buffer is not exceeded when reading
			if (struct_size > sizeof(data->target_rd_msg.msg)) {
				struct_size = sizeof(data->target_rd_msg.msg);
			}
			switch (reg_offset) {
			case SENSOR_INIT_DATA_0_REG:
			case SENSOR_INIT_DATA_1_REG: {
				data->target_rd_msg.msg_length = struct_size;
				memcpy(data->target_rd_msg.msg,
				       sensor_init_data_table[reg_offset - SENSOR_INIT_DATA_0_REG],
				       struct_size);
			} break;
			case SENSOR_READING_0_REG:
			case SENSOR_READING_1_REG:
			case SENSOR_READING_2_REG:
			case SENSOR_READING_3_REG: {
				data->target_rd_msg.msg_length = struct_size;
				memcpy(data->target_rd_msg.msg,
				       sensor_reading_table[reg_offset - SENSOR_READING_0_REG],
				       struct_size);
			} break;
			case INVENTORY_IDS_REG: {
				data->target_rd_msg.msg_length = struct_size;
				memcpy(data->target_rd_msg.msg,
				       inventory_ids_table[reg_offset - INVENTORY_IDS_REG],
				       struct_size);
			} break;
			case STRAP_CAPABILTITY_REG: {
				data->target_rd_msg.msg_length = struct_size;
				memcpy(data->target_rd_msg.msg,
				       strap_capability_table[reg_offset - STRAP_CAPABILTITY_REG],
				       struct_size);
			} break;
			case FRU_BOARD_PART_NUMBER_REG:
			case FRU_BOARD_SERIAL_NUMBER_REG:
			case FRU_BOARD_PRODUCT_NAME_REG:
			case FRU_BOARD_CUSTOM_DATA_1_REG:
			case FRU_BOARD_CUSTOM_DATA_2_REG:
			case FRU_BOARD_CUSTOM_DATA_3_REG:
			case FRU_BOARD_CUSTOM_DATA_4_REG:
			case FRU_BOARD_CUSTOM_DATA_5_REG:
			case FRU_BOARD_CUSTOM_DATA_6_REG:
			case FRU_BOARD_CUSTOM_DATA_7_REG:
			case FRU_BOARD_CUSTOM_DATA_8_REG:
			case FRU_BOARD_CUSTOM_DATA_9_REG:
			case FRU_BOARD_CUSTOM_DATA_10_REG: {
				data->target_rd_msg.msg_length = struct_size;
				memcpy(data->target_rd_msg.msg,
				       fru_board_data_table[reg_offset - FRU_BOARD_PART_NUMBER_REG],
				       struct_size);
			} break;
			case FRU_PRODUCT_NAME_REG:
			case FRU_PRODUCT_PART_NUMBER_REG:
			case FRU_PRODUCT_PART_VERSION_REG:
			case FRU_PRODUCT_SERIAL_NUMBER_REG:
			case FRU_PRODUCT_ASSET_TAG_REG:
			case FRU_PRODUCT_CUSTOM_DATA_1_REG:
			case FRU_PRODUCT_CUSTOM_DATA_2_REG: {
				data->target_rd_msg.msg_length = struct_size;
				memcpy(data->target_rd_msg.msg,
				       fru_product_data_table[reg_offset - FRU_PRODUCT_NAME_REG],
				       struct_size);
			} break;
			case I2C_BRIDGE_COMMAND_STATUS_REG: {
				data->target_rd_msg.msg_length = 1;
				data->target_rd_msg.msg[0] =
					i2c_bridge_command_status_table[0] ?
						i2c_bridge_command_status_table[0]->data_status :
						I2C_BRIDGE_COMMAND_FAILURE;
			} break;
			case I2C_BRIDGE_COMMAND_RESPONSE_REG: {
				if (i2c_bridge_command_response_data_table[0]) {
					struct_size = i2c_bridge_command_response_data_table[0]
							      ->data_length +
						      1;
					data->target_rd_msg.msg_length = struct_size;
					memcpy(data->target_rd_msg.msg,
					       i2c_bridge_command_response_data_table[0],
					       struct_size);
				} else {
					data->target_rd_msg.msg_length = 1;
					data->target_rd_msg.msg[0] = 0xFF;
				}
			} break;
			case CONTROL_VOL_Minerva_Aegis_VR_ASIC_P0V75_VDDPHY_HBM0_HBM2_HBM4_REG:
			case CONTROL_VOL_Minerva_Aegis_VR_ASIC_P0V75_VDDPHY_HBM1_HBM3_HBM5_REG:
			case CONTROL_VOL_Minerva_Aegis_VR_ASIC_P1V1_VDDC_HBM0_HBM2_HBM4_REG:
			case CONTROL_VOL_Minerva_Aegis_VR_ASIC_P1V1_VDDC_HBM1_HBM3_HBM5_REG:
			case CONTROL_VOL_Minerva_Aegis_VR_ASIC_P0V4_VDDQL_HBM0_HBM2_HBM4_REG:
			case CONTROL_VOL_Minerva_Aegis_VR_ASIC_P0V4_VDDQL_HBM1_HBM3_HBM5_REG:
			case CONTROL_VOL_Minerva_Aegis_VR_ASIC_P1V8_VPP_HBM0_HBM2_HBM4_REG:
			case CONTROL_VOL_Minerva_Aegis_VR_ASIC_P1V8_VPP_HBM1_HBM3_HBM5_REG: {
				uint8_t rail = get_vr_rail_by_control_vol_reg(reg_offset);
				uint16_t vout = 0xFFFF;
				if (!voltage_command_setting_get(rail, &vout)) {
					LOG_ERR("Can't voltage_command setting_get by rail: 0x%02x",
						rail);
				}
				memcpy(data->target_rd_msg.msg, &vout, sizeof(vout));
				data->target_rd_msg.msg_length = 2;
			} break;
			case POWER_CAPPING_GET_VR_ASIC_P0V85_PVDD_PWR_W_REG:
			case POWER_CAPPING_GET_VR_ASIC_P0V4_VDDQL_HBM0_HBM2_HBM4_PWR_W_REG:
			case POWER_CAPPING_GET_VR_ASIC_P0V75_VDDPHY_HBM0_HBM2_HBM4_PWR_W_REG:
			case POWER_CAPPING_GET_VR_ASIC_P0V4_VDDQL_HBM1_HBM3_HBM5_PWR_W_REG:
			case POWER_CAPPING_GET_VR_ASIC_P0V75_VDDPHY_HBM1_HBM3_HBM5_PWR_W_REG:
			case POWER_CAPPING_GET_VR_ASIC_P0V75_MAX_PHY_N_PWR_W_REG:
			case POWER_CAPPING_GET_VR_ASIC_P0V75_MAX_PHY_S_PWR_W_REG: {
				uint16_t power =
					plat_power_capping_table
						[reg_offset -
						 POWER_CAPPING_GET_VR_ASIC_P0V85_PVDD_PWR_W_REG]
							.sensor_value;
				memcpy(data->target_rd_msg.msg, &power, sizeof(power));
				data->target_rd_msg.msg_length = sizeof(power);
			} break;
			default:
				LOG_ERR("Unknown reg offset: 0x%02x", reg_offset);
				data->target_rd_msg.msg_length = 1;
				data->target_rd_msg.msg[0] = 0xFF;
				break;
			}
		} else if (data->wr_buffer_idx == 2) {
			uint8_t reg_offset = data->target_wr_msg.msg[0];
			switch (reg_offset) {
			case WRITE_STRAP_PIN_VALUE_REG: {
				int rail = data->target_wr_msg.msg[1];
				int drive_level = -1;
				if (!get_bootstrap_change_drive_level(rail, &drive_level)) {
					LOG_ERR("Can't get_bootstrap_change_drive_level by rail index: %x",
						rail);
					data->target_rd_msg.msg[0] = 0xFF;
				} else {
					data->target_rd_msg.msg[0] = drive_level;
				}
				data->target_rd_msg.msg_length = 1;
			} break;
			default:
				LOG_ERR("Unknown reg offset: 0x%02x", reg_offset);
				data->target_rd_msg.msg_length = 1;
				data->target_rd_msg.msg[0] = 0xFF;
				break;
			}
		} else {
			LOG_ERR("Received data length: 0x%02x", data->wr_buffer_idx);
			data->target_rd_msg.msg_length = 1;
			data->target_rd_msg.msg[0] = 0xFF;
		}
	}
	return false;
}

void plat_master_write_thread_handler()
{
	int rc = 0;
	while (1) {
		uint8_t rdata[MAX_I2C_TARGET_BUFF] = { 0 };
		uint16_t rlen = 0;
		rc = i2c_target_read(I2C_BUS7, rdata, sizeof(rdata), &rlen);
		if (rc) {
			LOG_ERR("i2c_target_read fail, ret %d", rc);
			continue;
		}
		// LOG_DBG("rlen = %d", rlen);
		// LOG_HEXDUMP_DBG(rdata, rlen, "");
		if (rlen < 1) {
			LOG_ERR("Received data too short");
			continue;
		}

		uint8_t reg_offset = rdata[0];
		switch (reg_offset) {
		case WRITE_STRAP_PIN_VALUE_REG: {
			if (rlen != 3) {
				LOG_ERR("Invalid length for offset: 0x%02x", reg_offset);
				break;
			}
			bootstrap_pin = rdata[1];
			user_setting_level = rdata[2];
			k_work_submit(&set_bootstrap_element_work);
		} break;
		case I2C_BRIDGE_COMMAND_REG: {
			if (rlen < 5) {
				LOG_ERR("Invalid length for offset: 0x%02x", reg_offset);
				break;
			}
			size_t payload_len = rlen - 4;
			size_t struct_size = sizeof(plat_i2c_bridge_command_config) + payload_len;
			plat_i2c_bridge_command_config *sensor_data = malloc(struct_size);
			if (!sensor_data) {
				LOG_ERR("Memory allocation failed!");
				break;
			}
			sensor_data->bus = rdata[1];
			sensor_data->addr = rdata[2];
			sensor_data->read_len = rdata[3];
			sensor_data->write_len = payload_len;
			memcpy(sensor_data->data, &rdata[4], payload_len);
			k_work_init(&sensor_data->work, i2c_bridge_command_handler);
			k_work_submit(&sensor_data->work);
		} break;
		case CONTROL_VOL_Minerva_Aegis_VR_ASIC_P0V75_VDDPHY_HBM0_HBM2_HBM4_REG:
		case CONTROL_VOL_Minerva_Aegis_VR_ASIC_P0V75_VDDPHY_HBM1_HBM3_HBM5_REG:
		case CONTROL_VOL_Minerva_Aegis_VR_ASIC_P1V1_VDDC_HBM0_HBM2_HBM4_REG:
		case CONTROL_VOL_Minerva_Aegis_VR_ASIC_P1V1_VDDC_HBM1_HBM3_HBM5_REG:
		case CONTROL_VOL_Minerva_Aegis_VR_ASIC_P0V4_VDDQL_HBM0_HBM2_HBM4_REG:
		case CONTROL_VOL_Minerva_Aegis_VR_ASIC_P0V4_VDDQL_HBM1_HBM3_HBM5_REG:
		case CONTROL_VOL_Minerva_Aegis_VR_ASIC_P1V8_VPP_HBM0_HBM2_HBM4_REG:
		case CONTROL_VOL_Minerva_Aegis_VR_ASIC_P1V8_VPP_HBM1_HBM3_HBM5_REG: {
			if (rlen != 3) {
				LOG_ERR("Invalid length for offset: 0x%02x", reg_offset);
				break;
			}
			plat_control_voltage *sensor_data = malloc(sizeof(plat_control_voltage));
			if (!sensor_data) {
				LOG_ERR("Memory allocation failed!");
				break;
			}
			sensor_data->rail = get_vr_rail_by_control_vol_reg(reg_offset);
			sensor_data->set_value = rdata[1] | (rdata[2] << 8);
			k_work_init(&sensor_data->work, set_control_voltage_handler);
			k_work_submit(&sensor_data->work);
		} break;
		case POWER_CAPPING_SET_VALUE_REG: {
			if (rlen != 5) {
				LOG_ERR("Invalid length for offset: 0x%02x", reg_offset);
				break;
			}
			plat_power_capping_set *sensor_data =
				malloc(sizeof(plat_power_capping_set));
			if (!sensor_data) {
				LOG_ERR("Memory allocation failed!");
				break;
			}
			sensor_data->set_value_HC = rdata[1] | (rdata[2] << 8);
			sensor_data->set_value_LC = rdata[3] | (rdata[4] << 8);
			k_work_init(&sensor_data->work, set_power_capping_handler);
			k_work_submit(&sensor_data->work);
		} break;
		case SET_SENSOR_POLLING_COMMAND_REG: {
			if (rlen != 8) {
				LOG_ERR("Invalid length for offset: 0x%02x", reg_offset);
				break;
			}
			uint8_t expected_signature[] = {
				0x4D, 0x54, 0x49, 0x41, 0x56, 0x31
			}; // ascii to hex: "MTIAV1"
			if (memcmp(&rdata[1], expected_signature, sizeof(expected_signature)) !=
			    0) {
				LOG_HEXDUMP_DBG(rdata, rlen, "rdata:");
				LOG_ERR("Wrong command for set sensor_polling");
				break;
			}
			plat_control_sensor_polling *sensor_data =
				malloc(sizeof(plat_control_sensor_polling));
			if (!sensor_data) {
				LOG_ERR("Memory allocation failed!");
				break;
			}
			sensor_data->set_value = rdata[7];
			k_work_init(&sensor_data->work, set_sensor_polling_handler);
			k_work_submit(&sensor_data->work);
		} break;
		default:
			LOG_ERR("Unknown reg offset: 0x%02x", reg_offset);
			break;
		}
	}
}

void plat_master_write_thread_init()
{
	plat_master_write_tid = k_thread_create(&plat_master_write_thread, plat_master_write_stack,
						K_THREAD_STACK_SIZEOF(plat_master_write_stack),
						plat_master_write_thread_handler, NULL, NULL, NULL,
						CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&plat_master_write_thread, "plat_master_write_thread");
}

void plat_telemetry_table_init(void)
{
	uint8_t buffer_size = 0;
	for (int i = 0; i < ARRAY_SIZE(telemetry_info_table); i++) {
		if (telemetry_info_table[i].telemetry_table_init) {
			bool success = telemetry_info_table[i].telemetry_table_init(
				&telemetry_info_table[i], &buffer_size);
			if (!success) {
				LOG_ERR("initialize sensor data at offset 0x%02X",
					telemetry_info_table[i].telemetry_offset);
			}
			telemetry_info_table[i].data_size = buffer_size;
		}
	}
	plat_master_write_thread_init();
}
