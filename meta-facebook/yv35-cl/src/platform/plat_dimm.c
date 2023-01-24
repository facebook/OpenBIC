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

#include "plat_dimm.h"

#include <stdio.h>
#include <stdlib.h>
#include <zephyr.h>
#include <logging/log.h>
#include <errno.h>
#include "sensor.h"
#include "libutil.h"
#include "sensor.h"
#include "power_status.h"
#include "pmic.h"
#include "plat_class.h"
#include "plat_i2c.h"
#include "plat_sensor_table.h"

LOG_MODULE_REGISTER(plat_dimm);

K_THREAD_STACK_DEFINE(get_dimm_info_stack, GET_DIMM_INFO_STACK_SIZE);
struct k_thread get_dimm_info_thread;
k_tid_t get_dimm_info_tid;

struct k_mutex i3c_dimm_mux_mutex;

uint8_t pmic_i3c_addr_list[MAX_COUNT_DIMM / 2] = { PMIC_A0_A4_ADDR, PMIC_A2_A6_ADDR,
						   PMIC_A3_A7_ADDR };
uint8_t spd_i3c_addr_list[MAX_COUNT_DIMM / 2] = { DIMM_SPD_A0_A4_ADDR, DIMM_SPD_A2_A6_ADDR,
						  DIMM_SPD_A3_A7_ADDR };

dimm_info dimm_data[MAX_COUNT_DIMM];

static bool is_dimm_data_init = false;

void start_get_dimm_info_thread()
{
	LOG_INF("Start thread to get dimm information");

	get_dimm_info_tid =
		k_thread_create(&get_dimm_info_thread, get_dimm_info_stack,
				K_THREAD_STACK_SIZEOF(get_dimm_info_stack), get_dimm_info_handler,
				NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&get_dimm_info_thread, "get_dimm_info_thread");
}

void init_i3c_dimm_data()
{
	I3C_MSG i3c_msg = { 0 };
	int i = 0, ret = 0;

	// Clear DIMM data
	memset(dimm_data, 0, sizeof(dimm_data));

	i3c_msg.bus = I3C_BUS4;

	// Init DIMM present status
	for (i = 0; i < MAX_COUNT_DIMM; i++) {
		// Read SPD vender to check dimm present
		switch_i3c_dimm_mux(I3C_MUX_TO_BIC, i / (MAX_COUNT_DIMM / 2));
		all_brocast_ccc(&i3c_msg);

		i3c_msg.target_addr = spd_i3c_addr_list[i % (MAX_COUNT_DIMM / 2)];
		i3c_msg.tx_len = 1;
		i3c_msg.rx_len = 1;
		i3c_msg.data[0] = 0x00;

		ret = i3c_transfer(&i3c_msg);
		if (ret == -EIO) {
			dimm_data[i].is_present = false;
			clear_unaccessible_dimm_data(i);
		} else {
			dimm_data[i].is_present = true;
		}
	}

	is_dimm_data_init = true;
}

void get_dimm_info_handler()
{
	I3C_MSG i3c_msg = { 0 };
	int i = 0;

	i3c_msg.bus = I3C_BUS4;

	// Attach DIMM SPD addr and PMIC addr
	for (i = 0; i < (MAX_COUNT_DIMM / 2); i++) {
		i3c_msg.target_addr = pmic_i3c_addr_list[i];
		i3c_attach(&i3c_msg);
		i3c_msg.target_addr = spd_i3c_addr_list[i];
		i3c_attach(&i3c_msg);
	}

	// Init mutex
	if (k_mutex_init(&i3c_dimm_mux_mutex)) {
		LOG_ERR("i3c_dimm_mux_mutex mutex init fail");
	}

	// Switch I3C mux to BIC when host post complete but BIC reset
	if (get_post_status()) {
		if (k_mutex_lock(&i3c_dimm_mux_mutex, K_MSEC(I3C_DIMM_MUX_MUTEX_TIMEOUT_MS))) {
			LOG_ERR("Failed to lock I3C dimm MUX");
		}

		switch_i3c_dimm_mux(I3C_MUX_TO_BIC, DIMM_MUX_TO_DIMM_A0A1A3);

		if (k_mutex_unlock(&i3c_dimm_mux_mutex)) {
			LOG_ERR("Failed to unlock I3C dimm MUX");
		}
	}

	while (1) {
		int dimm_id = 0, ret = 0;

		// Avoid to get wrong thus only monitor after post complete
		if (!get_post_status() || !is_i3c_mux_to_bic()) {
			k_msleep(GET_DIMM_INFO_TIME_MS);
			continue;
		}

		if (!is_dimm_data_init) {
			init_i3c_dimm_data();
		}

		if (k_mutex_lock(&i3c_dimm_mux_mutex, K_MSEC(I3C_DIMM_MUX_MUTEX_TIMEOUT_MS))) {
			LOG_ERR("Failed to lock I3C dimm MUX");
			k_msleep(GET_DIMM_INFO_TIME_MS);
			continue;
		}

		for (dimm_id = 0; dimm_id < MAX_COUNT_DIMM; dimm_id++) {
			// Read all dimm relate information
			// DIMM temp: 2 byte, PMIC error: 7 byte, PMIC power: 1 byte
			if (!dimm_data[dimm_id].is_present) {
				continue;
			}

			i3c_msg.bus = I3C_BUS4;
			ret = switch_i3c_dimm_mux(I3C_MUX_TO_BIC, dimm_id / (MAX_COUNT_DIMM / 2));
			if (ret != 0) {
				clear_unaccessible_dimm_data(dimm_id);
				continue;
			}

			// I3C_CCC_RSTDAA: Reset dynamic address assignment
			// I3C_CCC_SETAASA: Set all addresses to static address
			ret = all_brocast_ccc(&i3c_msg);
			if (ret != 0) {
				clear_unaccessible_dimm_data(dimm_id);
				continue;
			}

			// Double check before read each DIMM info
			if (!get_post_status()) {
				break;
			}

			// Read DIMM SPD temperature
			i3c_msg.target_addr = spd_i3c_addr_list[dimm_id % (MAX_COUNT_DIMM / 2)];
			i3c_msg.tx_len = 1;
			i3c_msg.rx_len = MAX_LEN_I3C_GET_SPD_TEMP;
			i3c_msg.data[0] = DIMM_SPD_TEMP;

			ret = i3c_spd_reg_read(&i3c_msg, false);
			if (ret != 0) {
				clear_unaccessible_dimm_data(dimm_id);
				LOG_ERR("Failed to read DIMM %d SPD temperature via I3C, ret%d",
					dimm_id, ret);
			} else {
				memcpy(&dimm_data[dimm_id].spd_temp_data, &i3c_msg.data,
				       sizeof(dimm_data[dimm_id].spd_temp_data));
			}

			// Double check before read each DIMM info
			if (!get_post_status()) {
				break;
			}

			// Read DIMM PMIC power
			i3c_msg.target_addr = pmic_i3c_addr_list[dimm_id % (MAX_COUNT_DIMM / 2)];
			i3c_msg.tx_len = 1;
			i3c_msg.rx_len = MAX_LEN_I3C_GET_PMIC_PWR;
			i3c_msg.data[0] = DIMM_PMIC_SWA_PWR;

			ret = i3c_transfer(&i3c_msg);
			if (ret != 0) {
				clear_unaccessible_dimm_data(dimm_id);
				LOG_ERR("Failed to read DIMM %d PMIC error via I3C, ret%d", dimm_id,
					ret);
				continue;
			} else {
				memcpy(&dimm_data[dimm_id].pmic_pwr_data, &i3c_msg.data,
				       sizeof(dimm_data[dimm_id].pmic_pwr_data));
			}

			// Double check before read each DIMM info
			if (!get_post_status()) {
				break;
			}

			// Read DIMM PMIC error
			i3c_msg.target_addr = pmic_i3c_addr_list[dimm_id % (MAX_COUNT_DIMM / 2)];
			i3c_msg.tx_len = 1;
			i3c_msg.rx_len = MAX_LEN_I3C_GET_PMIC_ERR;
			i3c_msg.data[0] = PMIC_POR_ERROR_LOG_ADDR_VAL;

			ret = i3c_transfer(&i3c_msg);
			if (ret != 0) {
				clear_unaccessible_dimm_data(dimm_id);
				LOG_ERR("Failed to read DIMM %d PMIC error via I3C, ret%d", dimm_id,
					ret);
				continue;
			} else {
				memcpy(&dimm_data[dimm_id].pmic_error_data, &i3c_msg.data,
				       sizeof(dimm_data[dimm_id].pmic_error_data));
			}
		}

		if (k_mutex_unlock(&i3c_dimm_mux_mutex)) {
			LOG_ERR("Failed to unlock I3C dimm MUX");
		}

		k_msleep(GET_DIMM_INFO_TIME_MS);
	}
}

int switch_i3c_dimm_mux(uint8_t i3c_mux_position, uint8_t dimm_mux_position)
{
	I2C_MSG i2c_msg = { 0 };
	int ret = 0, retry = 3;

	i2c_msg.bus = I2C_BUS1;
	i2c_msg.target_addr = CPLD_ADDR;
	i2c_msg.tx_len = 2;
	i2c_msg.rx_len = 0;
	i2c_msg.data[0] = DIMM_I3C_MUX_CONTROL_OFFSET;

	// BIT 0: PD_SPD1_REMOTE_EN
	// 0: switch I3C mux to CPU 1: switch I3C mux to BIC
	// BIC 1: PD_SPD2_REMOTE_EN
	// 0: switch DIMM mux to A0~A3 1: switch DIMM mux to A4~A7
	i2c_msg.data[1] = (dimm_mux_position << 1) | i3c_mux_position;

	ret = i2c_master_write(&i2c_msg, retry);
	if (ret != 0) {
		LOG_ERR("Failed to switch I3C MUX: 0x%x, ret=%d", i3c_mux_position, ret);
	}

	return ret;
}

bool is_i3c_mux_to_bic()
{
	I2C_MSG i2c_msg = { 0 };
	int ret = 0, retry = 3;

	i2c_msg.bus = I2C_BUS1;
	i2c_msg.target_addr = CPLD_ADDR;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = DIMM_I3C_MUX_CONTROL_OFFSET;

	ret = i2c_master_read(&i2c_msg, retry);
	if (ret != 0) {
		LOG_ERR("Failed to read I3C MUX status, ret=%d", ret);
		return false;
	}

	if (GETBIT(i2c_msg.data[0], 0) == I3C_MUX_TO_BIC) {
		return true;
	} else {
		return false;
	}
}

bool is_dimm_present(uint8_t dimm_id)
{
	return dimm_data[dimm_id].is_present;
}

bool is_dimm_init()
{
	return is_dimm_data_init;
}

int all_brocast_ccc(I3C_MSG *i3c_msg)
{
	CHECK_NULL_ARG_WITH_RETURN(i3c_msg, -1);

	int ret = 0;

	ret = i3c_brocast_ccc(i3c_msg, I3C_CCC_RSTDAA, I3C_BROADCAST_ADDR);
	if (ret != 0) {
		return ret;
	}

	ret = i3c_brocast_ccc(i3c_msg, I3C_CCC_SETAASA, I3C_BROADCAST_ADDR);
	if (ret != 0) {
		return ret;
	}

	return ret;
}

int get_pmic_error_raw_data(int dimm_index, uint8_t *data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, -1);

	int i = 0, fail_count = 0;

	for (i = 0; i < sizeof(dimm_data[dimm_index].pmic_error_data); i++) {
		if (dimm_data[dimm_index].pmic_error_data[i] == SENSOR_FAIL) {
			fail_count++;
		}
	}

	// PMIC error data read failed
	if (fail_count == sizeof(dimm_data[dimm_index].pmic_error_data)) {
		return -1;
	}

	memcpy(data, &dimm_data[dimm_index].pmic_error_data,
	       sizeof(dimm_data[dimm_index].pmic_error_data));

	return 0;
}

void get_pmic_power_raw_data(int dimm_index, uint8_t *data)
{
	CHECK_NULL_ARG(data);

	memcpy(data, &dimm_data[dimm_index].pmic_pwr_data,
	       sizeof(dimm_data[dimm_index].pmic_pwr_data));
}

void get_spd_temp_raw_data(int dimm_index, uint8_t *data)
{
	CHECK_NULL_ARG(data);

	memcpy(data, &dimm_data[dimm_index].spd_temp_data,
	       sizeof(dimm_data[dimm_index].spd_temp_data));
}

uint8_t sensor_num_map_dimm_id(uint8_t sensor_num)
{
	uint8_t dimm_id = DIMM_ID_UNKNOWN;

	switch (sensor_num) {
	case SENSOR_NUM_PWR_DIMMA0_PMIC:
	case SENSOR_NUM_TEMP_DIMM_A0:
		dimm_id = DIMM_ID_A0;
		break;
	case SENSOR_NUM_PWR_DIMMA2_PMIC:
	case SENSOR_NUM_TEMP_DIMM_A2:
		dimm_id = DIMM_ID_A2;
		break;
	case SENSOR_NUM_PWR_DIMMA3_PMIC:
	case SENSOR_NUM_TEMP_DIMM_A3:
		dimm_id = DIMM_ID_A3;
		break;
	case SENSOR_NUM_PWR_DIMMA4_PMIC:
	case SENSOR_NUM_TEMP_DIMM_A4:
		dimm_id = DIMM_ID_A4;
		break;
	case SENSOR_NUM_PWR_DIMMA6_PMIC:
	case SENSOR_NUM_TEMP_DIMM_A6:
		dimm_id = DIMM_ID_A6;
		break;
	case SENSOR_NUM_PWR_DIMMA7_PMIC:
	case SENSOR_NUM_TEMP_DIMM_A7:
		dimm_id = DIMM_ID_A7;
		break;
	default:
		dimm_id = DIMM_ID_UNKNOWN;
		break;
	}

	return dimm_id;
}

int pal_get_pmic_pwr(uint8_t sensor_num, uint8_t *data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, -1);

	uint8_t dimm_id = DIMM_ID_UNKNOWN;

	dimm_id = sensor_num_map_dimm_id(sensor_num);
	if (dimm_id == DIMM_ID_UNKNOWN) {
		return -1;
	}

	get_pmic_power_raw_data(dimm_id, (uint8_t *)data);

	// If sensor data is SENSOR_FAIL, return failed
	if (data[0] == SENSOR_FAIL) {
		return -1;
	}

	return 0;
}

int pal_get_spd_temp(uint8_t sensor_num, uint8_t *data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, -1);

	uint8_t dimm_id = DIMM_ID_UNKNOWN;

	dimm_id = sensor_num_map_dimm_id(sensor_num);
	if (dimm_id == DIMM_ID_UNKNOWN) {
		return -1;
	}

	get_spd_temp_raw_data(dimm_id, (uint8_t *)data);

	// If sensor data is SENSOR_FAIL, return failed
	if (data[0] == SENSOR_FAIL) {
		return -1;
	}

	return 0;
}

void clear_unaccessible_dimm_data(uint8_t dimm_id)
{
	memset(dimm_data[dimm_id].pmic_error_data, SENSOR_FAIL,
	       sizeof(dimm_data[dimm_id].pmic_error_data));
	memset(dimm_data[dimm_id].pmic_pwr_data, SENSOR_FAIL,
	       sizeof(dimm_data[dimm_id].pmic_pwr_data));
	memset(dimm_data[dimm_id].spd_temp_data, SENSOR_FAIL,
	       sizeof(dimm_data[dimm_id].spd_temp_data));
}
