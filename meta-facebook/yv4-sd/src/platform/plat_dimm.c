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

#include <stdio.h>
#include <stdlib.h>
#include <zephyr.h>
#include <logging/log.h>
#include <errno.h>

#include "sensor.h"
#include "power_status.h"

#include "plat_dimm.h"
#include "plat_i2c.h"
#include "plat_i3c.h"

LOG_MODULE_REGISTER(plat_dimm);

K_THREAD_STACK_DEFINE(get_dimm_info_stack, GET_DIMM_INFO_STACK_SIZE);
struct k_thread get_dimm_info_thread;
struct k_mutex i3c_dimm_mutex;
k_tid_t get_dimm_info_tid;

dimm_info dimm_data[DIMM_ID_MAX];

bool is_dimm_checked_presnt = false;
bool is_cpld_support_i3c_mux_check = false;
int init_dimm_laps = 0;

uint8_t spd_i3c_addr_list[] = { DIMM_SPD_A_G_ADDR, DIMM_SPD_B_H_ADDR, DIMM_SPD_C_I_ADDR,
				DIMM_SPD_D_J_ADDR, DIMM_SPD_E_K_ADDR, DIMM_SPD_F_L_ADDR };

uint8_t pmic_i3c_addr_list[] = { DIMM_PMIC_A_G_ADDR, DIMM_PMIC_B_H_ADDR, DIMM_PMIC_C_I_ADDR,
				 DIMM_PMIC_D_J_ADDR, DIMM_PMIC_E_K_ADDR, DIMM_PMIC_F_L_ADDR };

void start_get_dimm_info_thread()
{
	LOG_INF("Start thread to get dimm information");

	get_dimm_info_tid =
		k_thread_create(&get_dimm_info_thread, get_dimm_info_stack,
				K_THREAD_STACK_SIZEOF(get_dimm_info_stack), get_dimm_info_handler,
				NULL, NULL, NULL, K_PRIO_PREEMPT(1), 0, K_NO_WAIT);
	k_thread_name_set(&get_dimm_info_thread, "get_dimm_info_thread");
}

static int ddr5_spd_read_bytes(uint8_t i2c_bus, uint8_t spd_addr, uint8_t *out)
{
	uint8_t keep_present = out[SPD_OEM_PRESENT];
	uint8_t keep_status = out[SPD_OEM_STATUS];
	memset(out, 0, SPD_RAW_LEN);
	out[SPD_OEM_PRESENT] = keep_present;
	out[SPD_OEM_STATUS] = keep_status;
	typedef struct {
		uint16_t raw_off;
		uint16_t comp_off;
		uint16_t len;
	} seg_t;
	static const seg_t segs[] = {
		{ 0x0002, SPD_TYPE, 1 },
		{ 0x0004, SPD_SZ_B4, 1 },
		{ 0x0006, SPD_SZ_B6, 1 },
		{ 0x0014, SPD_SPEED_L, 2 },
		{ 0x00C6, SPD_PMICVEN_L, 2 },
		{ 0x00EA, SPD_SZ_BEA, 1 },
		{ 0x00EB, SPD_SZ_BEB, 1 },
		{ 0x00F0, SPD_REGVEN_L, 2 },
		{ 0x0200, SPD_MFG_ID_L, 2 },
		{ 0x0202, SPD_MFG_LOC, 1 },
		{ 0x0203, SPD_MFG_YY, 2 },
		{ 0x0205, SPD_SN_OFF, SPD_SN_LEN }, /* 4 */
		{ 0x0209, SPD_PN_OFF, SPD_PN_LEN }, /* 30 */
	};

	uint16_t offset = 0x0000;
	uint16_t end_excl = 0x0227;
	size_t len = (size_t)(end_excl - offset);

	while (len) {
		size_t chunk = (len > 16) ? 16 : len;

		I2C_MSG spd_msg;
		memset(&spd_msg, 0, sizeof(I2C_MSG));
		spd_msg.bus = i2c_bus;
		spd_msg.target_addr = spd_addr;
		spd_msg.tx_len = 2;
		spd_msg.rx_len = chunk;

		// set MR11[3] = 1b for 2-bytes addressing (offset) mode
		uint16_t enc_offset = ((offset & 0x780) << 1) | (0x80 | (offset & 0x7F));
		spd_msg.data[0] = enc_offset & 0xFF;
		spd_msg.data[1] = enc_offset >> 8;

		if (i2c_master_read(&spd_msg, 1) != 0) {
			return -1;
		}

		for (size_t i = 0; i < chunk; ++i) {
			uint16_t cur = offset + (uint16_t)i;
			for (size_t s = 0; s < (sizeof(segs) / sizeof(segs[0])); ++s) {
				uint16_t so = segs[s].raw_off;
				uint16_t sl = segs[s].len;
				if (cur >= so && cur < (uint16_t)(so + sl)) {
					out[segs[s].comp_off + (cur - so)] = spd_msg.data[i];
					break;
				}
			}
		}

		offset += (uint16_t)chunk;
		len -= chunk;
	}

	return 0;
}

void get_dimm_info_handler()
{
	I2C_MSG msg = { 0 };

	// Init mutex
	if (k_mutex_init(&i3c_dimm_mutex)) {
		LOG_ERR("i3c_dimm_mux_mutex mutex init fail");
	}

	while (1) {
		int ret = 0;
		uint8_t dimm_id;
		uint8_t dimm_mux_status = 0;

		// Check sensor poll enable
		if (get_sensor_poll_enable_flag() == false) {
			k_msleep(GET_DIMM_INFO_TIME_MS);
			continue;
		}

		// Avoid to get wrong thus only monitor after post complete
		if (!get_post_status()) {
			k_msleep(GET_DIMM_INFO_TIME_MS);
			continue;
		}

		ret = check_i3c_dimm_mux(&dimm_mux_status);
		if (ret != 0) {
			// Failed to get i3c dimm mux status from cpld
			continue;
		}
		if (dimm_mux_status & (1 << I3C_MUX_STATUS_ENABLE_FUNCTION_CHECK)) {
			is_cpld_support_i3c_mux_check = true;
			if (!(dimm_mux_status & (1 << I3C_MUX_STATUS_PD_SPD_1_REMOTE_EN))) {
				// spd1 mux set to cpu now
				continue;
			}
		}

		if (is_dimm_checked_presnt == false) {
			if (init_dimm_prsnt_status() < 0) {
				k_msleep(GET_DIMM_INFO_TIME_MS);
				continue;
			}
		}

		if (k_mutex_lock(&i3c_dimm_mutex, K_MSEC(I3C_DIMM_MUTEX_TIMEOUT_MS))) {
			LOG_ERR("Failed to lock I3C dimm MUX");
			k_msleep(GET_DIMM_INFO_TIME_MS);
			continue;
		}

		for (dimm_id = 0; dimm_id < DIMM_ID_MAX; dimm_id++) {
			if (dimm_data[dimm_id].is_present != DIMM_PRSNT) {
				continue;
			}

			uint8_t i3c_ctrl_mux_data = (dimm_id / (DIMM_ID_MAX / 2)) ?
							    I3C_MUX_BIC_TO_DIMMG_TO_L :
							    I3C_MUX_BIC_TO_DIMMA_TO_F;

			if (is_cpld_support_i3c_mux_check) {
				ret = check_i3c_dimm_mux(&dimm_mux_status);
				if (ret != 0) {
					continue;
				}
				if (!(dimm_mux_status & (1 << I3C_MUX_STATUS_PD_SPD_1_REMOTE_EN))) {
					// spd1 mux set to cpu now
					continue;
				} else {
					if (((dimm_mux_status & I3C_MUX_STATUS_SPD_MASK) >> 5) !=
					    i3c_ctrl_mux_data) {
						ret = switch_i3c_dimm_mux(i3c_ctrl_mux_data);
						if (ret != 0) {
							clear_unaccessible_dimm_data(dimm_id);
							continue;
						}
						ret = check_i3c_dimm_mux(&dimm_mux_status);
						if (ret != 0) {
							continue;
						}
						if (((dimm_mux_status & I3C_MUX_STATUS_SPD_MASK) >>
						     5) != i3c_ctrl_mux_data) {
							continue;
						}
					}
				}
			} else {
				ret = switch_i3c_dimm_mux(i3c_ctrl_mux_data);
				if (ret != 0) {
					clear_unaccessible_dimm_data(dimm_id);
					continue;
				}
			}

			// When OS reboot, we need to switch it back to CPU
			if (!get_post_status()) {
				switch_i3c_dimm_mux(I3C_MUX_CPU_TO_DIMM);
				break;
			}

			if (!dimm_data[dimm_id].is_spd_raw_ready) {
				const uint8_t spd_addr =
					spd_i3c_addr_list[dimm_id % (DIMM_ID_MAX / 2)];

				I2C_MSG msg;
				memset(&msg, 0, sizeof(I2C_MSG));
				msg.bus = I2C_BUS13;
				msg.target_addr = spd_addr;
				msg.tx_len = 2;
				msg.rx_len = 0;
				msg.data[0] = 0x0B;
				msg.data[1] = 0x08;

				uint8_t *out = dimm_data[dimm_id].spd_raw_data;

				out[SPD_OEM_STATUS] &= ~SPD_OEM_STATUS_SPD_READY;

				if (i2c_master_write(&msg, 1) == 0) {
					if (ddr5_spd_read_bytes(I2C_BUS13, spd_addr, out) == 0) {
						out[SPD_OEM_STATUS] |= SPD_OEM_STATUS_SPD_READY;
						dimm_data[dimm_id].is_spd_raw_ready = true;
						LOG_INF("DIMM %d: SPD compact cache ready (0x50 bytes)",
							dimm_id);
					} else {
						LOG_DBG("DIMM %d: SPD compact cache read failed",
							dimm_id);
					}
				} else {
					LOG_DBG("DIMM %d: MR11 setting failed", dimm_id);
				}
			}

			memset(&msg, 0, sizeof(I2C_MSG));
			msg.bus = I2C_BUS13;
			msg.target_addr = spd_i3c_addr_list[dimm_id % (DIMM_ID_MAX / 2)];
			msg.tx_len = 1;
			msg.rx_len = MAX_LEN_I3C_GET_SPD_TEMP;
			msg.data[0] = DIMM_SPD_TEMP;

			if (i2c_master_read(&msg, 3)) {
				clear_unaccessible_dimm_data(dimm_id);
				LOG_ERR("Failed to read DIMM %d SPD temperature", dimm_id);
			} else {
				memcpy(&dimm_data[dimm_id].spd_temp_data, &msg.data,
				       sizeof(dimm_data[dimm_id].spd_temp_data));
			}

			memset(&msg, 0, sizeof(I2C_MSG));
			msg.bus = I2C_BUS13;
			msg.target_addr = pmic_i3c_addr_list[dimm_id % (DIMM_ID_MAX / 2)];
			msg.tx_len = 1;
			msg.rx_len = MAX_LEN_I3C_GET_PMIC_PWR;
			msg.data[0] = DIMM_PMIC_SWA_PWR;

			if (i2c_master_read(&msg, 3)) {
				clear_unaccessible_dimm_data(dimm_id);
				LOG_ERR("Failed to read DIMM %d SPD pwr", dimm_id);
				continue;
			} else {
				memcpy(&dimm_data[dimm_id].pmic_pwr_data, &msg.data,
				       sizeof(dimm_data[dimm_id].pmic_pwr_data));
			}
		}

		if (k_mutex_unlock(&i3c_dimm_mutex)) {
			LOG_ERR("Failed to unlock I3C dimm MUX");
		}

		k_msleep(GET_DIMM_INFO_TIME_MS);
	}
}

uint8_t sensor_num_map_dimm_id(uint8_t sensor_num)
{
	uint8_t dimm_id = DIMM_ID_UNKNOWN;

	switch (sensor_num) {
	case NUM_DIMM_A_TEMP:
	case NUM_DIMM_A_PMIC_PWR:
		dimm_id = DIMM_ID_A;
		break;
	case NUM_DIMM_B_TEMP:
	case NUM_DIMM_B_PMIC_PWR:
		dimm_id = DIMM_ID_B;
		break;
	case NUM_DIMM_C_TEMP:
	case NUM_DIMM_C_PMIC_PWR:
		dimm_id = DIMM_ID_C;
		break;
	case NUM_DIMM_D_TEMP:
	case NUM_DIMM_D_PMIC_PWR:
		dimm_id = DIMM_ID_D;
		break;
	case NUM_DIMM_E_TEMP:
	case NUM_DIMM_E_PMIC_PWR:
		dimm_id = DIMM_ID_E;
		break;
	case NUM_DIMM_F_TEMP:
	case NUM_DIMM_F_PMIC_PWR:
		dimm_id = DIMM_ID_F;
		break;
	case NUM_DIMM_G_TEMP:
	case NUM_DIMM_G_PMIC_PWR:
		dimm_id = DIMM_ID_G;
		break;
	case NUM_DIMM_H_TEMP:
	case NUM_DIMM_H_PMIC_PWR:
		dimm_id = DIMM_ID_H;
		break;
	case NUM_DIMM_I_TEMP:
	case NUM_DIMM_I_PMIC_PWR:
		dimm_id = DIMM_ID_I;
		break;
	case NUM_DIMM_J_TEMP:
	case NUM_DIMM_J_PMIC_PWR:
		dimm_id = DIMM_ID_J;
		break;
	case NUM_DIMM_K_TEMP:
	case NUM_DIMM_K_PMIC_PWR:
		dimm_id = DIMM_ID_K;
		break;
	case NUM_DIMM_L_TEMP:
	case NUM_DIMM_L_PMIC_PWR:
		dimm_id = DIMM_ID_L;
		break;
	default:
		break;
	}

	return dimm_id;
}

void get_spd_temp_raw_data(int dimm_index, uint8_t *data)
{
	CHECK_NULL_ARG(data);

	memcpy(data, &dimm_data[dimm_index].spd_temp_data,
	       sizeof(dimm_data[dimm_index].spd_temp_data));
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

void get_pmic_power_raw_data(int dimm_index, uint8_t *data)
{
	CHECK_NULL_ARG(data);

	memcpy(data, &dimm_data[dimm_index].pmic_pwr_data,
	       sizeof(dimm_data[dimm_index].pmic_pwr_data));
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

void clear_unaccessible_dimm_data(uint8_t dimm_id)
{
	memset(dimm_data[dimm_id].spd_temp_data, SENSOR_FAIL,
	       sizeof(dimm_data[dimm_id].spd_temp_data));
	memset(dimm_data[dimm_id].pmic_pwr_data, SENSOR_FAIL,
	       sizeof(dimm_data[dimm_id].pmic_pwr_data));
}

bool get_spd_raw_ready(uint8_t dimm_id)
{
	if (dimm_id < DIMM_ID_MAX)
		return dimm_data[dimm_id].is_spd_raw_ready;
	return false;
}

int plat_get_spd_raw(uint8_t dimm_id, uint8_t **buf_out, bool *ready_out)
{
	if (dimm_id >= DIMM_ID_MAX || buf_out == NULL || ready_out == NULL) {
		return -1;
	}
	if (dimm_data[dimm_id].is_present == DIMM_NOT_PRSNT) {
		return -2;
	}
	*buf_out = dimm_data[dimm_id].spd_raw_data;
	*ready_out = dimm_data[dimm_id].is_spd_raw_ready;
	return 0;
}

int switch_i3c_dimm_mux(uint8_t i3c_ctrl_mux_data)
{
	I2C_MSG i2c_msg = { 0 };
	int ret = 0, retry = 3;

	i2c_msg.bus = I2C_BUS5;
	i2c_msg.target_addr = CPLD_ADDR;
	i2c_msg.tx_len = 2;
	i2c_msg.rx_len = 0;
	i2c_msg.data[0] = DIMM_I3C_MUX_CONTROL_OFFSET;

	i2c_msg.data[1] = i3c_ctrl_mux_data;

	ret = i2c_master_write(&i2c_msg, retry);
	if (ret != 0) {
		LOG_ERR("Failed to switch I3C MUX: 0x%x, ret= %d", i3c_ctrl_mux_data, ret);
	}

	return ret;
}

int check_i3c_dimm_mux(uint8_t *status_data)
{
	I2C_MSG i2c_msg = { 0 };
	int ret = 0, retry = 3;
	*status_data = 0;

	i2c_msg.bus = I2C_BUS5;
	i2c_msg.target_addr = CPLD_ADDR;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = DIMM_I3C_MUX_STATUS_OFFSET;

	ret = i2c_master_read(&i2c_msg, retry);
	if (ret != 0) {
		LOG_ERR("Failed to get I3C MUX status from CPLD: 0x%x, ret= %d",
			DIMM_I3C_MUX_STATUS_OFFSET, ret);
	} else {
		*status_data = i2c_msg.data[0];
	}

	return ret;
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

int init_dimm_prsnt_status()
{
	I2C_MSG msg = { 0 };
	int ret = 0;

	int is_dimm_not_present = 0;
	int retry = 10;

	// Clear DIMM data
	memset(dimm_data, 0, sizeof(dimm_data));

	if (k_mutex_lock(&i3c_dimm_mutex, K_MSEC(I3C_DIMM_MUTEX_TIMEOUT_MS))) {
		LOG_ERR("Failed to lock I3C dimm MUX");
		return -1;
	}

	// Init DIMM present status
	for (uint8_t dimm_id = 0; dimm_id < DIMM_ID_MAX; dimm_id++) {
		uint8_t i3c_ctrl_mux_data = (dimm_id / (DIMM_ID_MAX / 2)) ?
						    I3C_MUX_BIC_TO_DIMMG_TO_L :
						    I3C_MUX_BIC_TO_DIMMA_TO_F;
		dimm_data[dimm_id].is_spd_raw_ready = false;
		memset(dimm_data[dimm_id].spd_raw_data, 0, SPD_RAW_LEN);
		ret = switch_i3c_dimm_mux(i3c_ctrl_mux_data);
		if (ret != 0) {
			clear_unaccessible_dimm_data(dimm_id);
			LOG_DBG("[%s]DIMM ID 0x%02x is not present", __func__, dimm_id);
			dimm_data[dimm_id].is_present = DIMM_NOT_PRSNT;
			dimm_data[dimm_id].is_spd_raw_ready = false;
			dimm_data[dimm_id].spd_raw_data[SPD_OEM_PRESENT] = 0;
			dimm_data[dimm_id].spd_raw_data[SPD_OEM_STATUS] &=
				~SPD_OEM_STATUS_SPD_READY;
			is_dimm_not_present = 1;
			continue;
		}

		memset(&msg, 0, sizeof(I2C_MSG));
		msg.bus = I2C_BUS13;
		msg.target_addr = spd_i3c_addr_list[dimm_id % (DIMM_ID_MAX / 2)];
		msg.tx_len = 1;
		msg.rx_len = 1;
		msg.data[0] = 0x00;

		if (i2c_master_read(&msg, 3)) {
			clear_unaccessible_dimm_data(dimm_id);
			LOG_INF("[%s]DIMM ID 0x%02x is not present", __func__, dimm_id);
			dimm_data[dimm_id].is_present = DIMM_NOT_PRSNT;
			dimm_data[dimm_id].is_spd_raw_ready = false;
			is_dimm_not_present = 1;
		} else {
			LOG_DBG("[%s]DIMM ID 0x%02x is present", __func__, dimm_id);
			dimm_data[dimm_id].is_present = DIMM_PRSNT;
			dimm_data[dimm_id].is_spd_raw_ready = false;
			dimm_data[dimm_id].spd_raw_data[SPD_OEM_PRESENT] = 1;
			dimm_data[dimm_id].spd_raw_data[SPD_OEM_STATUS] &=
				~SPD_OEM_STATUS_SPD_READY;
		}
	}

	if (k_mutex_unlock(&i3c_dimm_mutex)) {
		LOG_ERR("Failed to unlock I3C dimm MUX");
	}

	if (is_dimm_not_present) {
		if (init_dimm_laps < retry) {
			init_dimm_laps++;
			LOG_ERR("Some DIMMs are not present. Retry %d\n", init_dimm_laps);
			return -1;
		} else {
			LOG_ERR("Some DIMMs are not present. Retry reach max.");
		}
	}

	is_dimm_checked_presnt = true;
	return 0;
}

uint8_t get_dimm_present(uint8_t dimm_id)
{
	if (dimm_id == DIMM_ID_UNKNOWN) {
		LOG_ERR("Failed to get DIMM's present. DIMM ID: 0x%02x", dimm_id);
		return DIMM_NOT_PRSNT;
	}
	return dimm_data[dimm_id].is_present;
}
