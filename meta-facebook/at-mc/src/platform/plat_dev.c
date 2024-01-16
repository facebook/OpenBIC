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
#include "sensor.h"
#include "pmbus.h"
#include "libutil.h"
#include "hal_i2c.h"
#include "plat_dev.h"
#include "util_pmbus.h"
#include "plat_isr.h"
#include "plat_fru.h"
#include "ioexp_tca9555.h"
#include "common_i2c_mux.h"
#include "plat_sensor_table.h"
#include <logging/log.h>
#include "plat_def.h"
#include "pm8702.h"
#include "cci.h"
#include "plat_mctp.h"
#include "plat_hook.h"
#include "plat_class.h"
#include "hal_gpio.h"
#include "plat_gpio.h"
#include "power_status.h"
#include "plat_pldm_monitor.h"
#include "pldm_state_set.h"

LOG_MODULE_REGISTER(plat_dev);

#define CXL_IOEXP_CONFIG_REG_DEFAULT_VAL 0xFF

#define CXL_IOEXP_U14_OUTPUT_0_REG_VAL 0xFF
#define CXL_IOEXP_U14_OUTPUT_1_REG_VAL 0xFF
#define CXL_IOEXP_U14_CONFIG_0_REG_VAL 0xFF
#define CXL_IOEXP_U14_CONFIG_1_REG_VAL 0xFF

#define CXL_IOEXP_U15_OUTPUT_0_REG_VAL 0x3B
#define CXL_IOEXP_U15_OUTPUT_1_REG_VAL 0xFF
#define CXL_IOEXP_U15_CONFIG_0_REG_VAL 0x21
#define CXL_IOEXP_U15_CONFIG_1_REG_VAL 0xFE

#define CXL_IOEXP_U16_OUTPUT_0_REG_VAL 0xFF
#define CXL_IOEXP_U16_OUTPUT_1_REG_VAL 0xFF
#define CXL_IOEXP_U16_CONFIG_0_REG_VAL 0x00
#define CXL_IOEXP_U16_CONFIG_1_REG_VAL 0x00

#define CXL_IOEXP_U17_OUTPUT_0_REG_VAL 0xFF
#define CXL_IOEXP_U17_OUTPUT_1_REG_VAL 0xFF
#define CXL_IOEXP_U17_CONFIG_0_REG_VAL 0xFF
#define CXL_IOEXP_U17_CONFIG_1_REG_VAL 0xFF

#define PM8702_DEFAULT_SENSOR_NUM SENSOR_NUM_TEMP_CXL_DIMMA

#define CXL_CARD_VR_COUNT 3

#define MONITOR_SSD_POWER_FAULT_STACK_SIZE 1536
#define MONITOR_SSD_POWER_FAULT_DELAY_MS 2000
K_THREAD_STACK_DEFINE(monitor_ssd_power_fault_thread, MONITOR_SSD_POWER_FAULT_STACK_SIZE);
struct k_thread ssd_power_fault_thread_handler;
k_tid_t ssd_power_fault_tid;

pm8702_dev_info pm8702_table[] = {
	{ .is_init = false }, { .is_init = false }, { .is_init = false }, { .is_init = false },
	{ .is_init = false }, { .is_init = false }, { .is_init = false }, { .is_init = false },
};

cxl_vr_fw_info cxl_vr_info_table[] = {
	{ .is_init = false }, { .is_init = false }, { .is_init = false }, { .is_init = false },
	{ .is_init = false }, { .is_init = false }, { .is_init = false }, { .is_init = false },
	{ .is_init = false }, { .is_init = false }, { .is_init = false }, { .is_init = false },
	{ .is_init = false }, { .is_init = false }, { .is_init = false }, { .is_init = false },
	{ .is_init = false }, { .is_init = false }, { .is_init = false }, { .is_init = false },
	{ .is_init = false }, { .is_init = false }, { .is_init = false }, { .is_init = false },
};

static bool ssd_power_fault[SSD_COUNT] = {
	false,
	false,
	false,
	false,
};

void clear_ssd_power_fault_flag()
{
	uint8_t index = 0;
	for (index = 0; index < ARRAY_SIZE(ssd_power_fault); ++index) {
		ssd_power_fault[index] = false;
	}
}

void clear_cxl_card_cache_value(uint8_t cxl_id)
{
	if (cxl_id >= ARRAY_SIZE(pm8702_table)) {
		LOG_ERR("Fail to clear CXL card cache by invalid cxl id: 0x%x", cxl_id);
		return;
	}

	uint8_t index = 0;
	uint8_t offset = 0;

	pm8702_table[cxl_id].is_init = false;
	memset(&pm8702_table[cxl_id].dev_info, 0, sizeof(cci_fw_info_resp));

	for (index = 0; index < CXL_CARD_VR_COUNT; ++index) {
		offset = cxl_id * CXL_CARD_VR_COUNT + index;
		memset(&cxl_vr_info_table[offset], 0, sizeof(cxl_vr_fw_info));
	}
}

void cxl_mb_status_init(uint8_t cxl_id)
{
	/** Initial mb reset pin status by checking the IO expander on CXL module **/
	int ret = 0;
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };
	mux_config meb_mux = { 0 };
	mux_config cxl_mux = { 0 };

	/** MEB mux for cxl channels **/
	meb_mux.bus = MEB_CXL_BUS;
	meb_mux.target_addr = CXL_FRU_MUX0_ADDR;
	meb_mux.channel = cxl_work_item[cxl_id].cxl_channel;

	/** CXL mux for sensor channels **/
	cxl_mux.bus = MEB_CXL_BUS;
	cxl_mux.target_addr = CXL_FRU_MUX1_ADDR;
	cxl_mux.channel = CXL_IOEXP_MUX_CHANNEL;

	/** Mutex lock bus **/
	struct k_mutex *meb_mutex = get_i2c_mux_mutex(meb_mux.bus);
	if (k_mutex_lock(meb_mutex, K_MSEC(MUTEX_LOCK_INTERVAL_MS))) {
		LOG_ERR("mutex locked failed bus%u", meb_mux.bus);
		return;
	}

	/** Enable mux channel **/
	if (set_mux_channel(meb_mux, MUTEX_LOCK_ENABLE) == false) {
		k_mutex_unlock(meb_mutex);
		return;
	}

	if (set_mux_channel(cxl_mux, MUTEX_LOCK_ENABLE) == false) {
		k_mutex_unlock(meb_mutex);
		return;
	}

	/** Read cxl U15 ioexp input port0 status **/
	msg.bus = MEB_CXL_BUS;
	msg.target_addr = CXL_IOEXP_U15_ADDR;
	msg.rx_len = 1;
	msg.tx_len = 1;
	msg.data[0] = TCA9555_INPUT_PORT_REG_0;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Unable to read ioexp bus: %u addr: 0x%02x", msg.bus, msg.target_addr);
		k_mutex_unlock(meb_mutex);
		return;
	}

	if ((msg.data[0] & CXL_IOEXP_MB_RESET_BIT) != 0) {
		/** CXL mux for cxl channels **/
		cxl_mux.bus = MEB_CXL_BUS;
		cxl_mux.target_addr = CXL_FRU_MUX1_ADDR;
		cxl_mux.channel = CXL_CONTROLLER_MUX_CHANNEL;

		if (set_mux_channel(cxl_mux, MUTEX_LOCK_ENABLE) == false) {
			k_mutex_unlock(meb_mutex);
			return;
		}

		if (get_set_cxl_endpoint(cxl_id, MCTP_EID_CXL) != true) {
			LOG_ERR("Fail to set eid, cxl id: 0x%x", cxl_id);
		} else {
			if (pm8702_table[cxl_id].is_init != true) {
				ret = pal_init_pm8702_info(cxl_id);
				if (ret != true) {
					LOG_ERR("Initial cxl id: 0x%x info fail", cxl_id);
				}
			}
		}
	}

	k_mutex_unlock(meb_mutex);
}

bool cxl_single_ioexp_alert_reset(uint8_t ioexp_name, bool is_mutex)
{
	int ret = 0;
	uint8_t retry = 5;
	uint8_t ioexp_addr = 0;
	I2C_MSG msg = { 0 };

	switch (ioexp_name) {
	case IOEXP_U14:
		ioexp_addr = CXL_IOEXP_U14_ADDR;
		break;
	case IOEXP_U15:
		ioexp_addr = CXL_IOEXP_U15_ADDR;
		break;
	case IOEXP_U16:
		ioexp_addr = CXL_IOEXP_U16_ADDR;
		break;
	case IOEXP_U17:
		ioexp_addr = CXL_IOEXP_U17_ADDR;
		break;
	default:
		LOG_ERR("CXL ioexp name: 0x%x is invalid", ioexp_name);
		return false;
	}

	/** Read cxl ioexp input port 0 status **/
	msg.bus = MEB_CXL_BUS;
	msg.target_addr = ioexp_addr;
	msg.rx_len = 1;
	msg.tx_len = 1;
	msg.data[0] = TCA9555_INPUT_PORT_REG_0;

	if (is_mutex) {
		ret = i2c_master_read(&msg, retry);
	} else {
		ret = i2c_master_read_without_mutex(&msg, retry);
	}

	if (ret != 0) {
		LOG_ERR("Unable to read ioexp bus: %u addr: 0x%02x", msg.bus, msg.target_addr);
		return false;
	}

	/** Read cxl ioexp input port 1 status **/
	memset(&msg, 0, sizeof(I2C_MSG));
	msg.bus = MEB_CXL_BUS;
	msg.target_addr = ioexp_addr;
	msg.rx_len = 1;
	msg.tx_len = 1;
	msg.data[0] = TCA9555_INPUT_PORT_REG_1;

	if (is_mutex) {
		ret = i2c_master_read(&msg, retry);
	} else {
		ret = i2c_master_read_without_mutex(&msg, retry);
	}

	if (ret != 0) {
		LOG_ERR("Unable to read ioexp bus: %u addr: 0x%02x", msg.bus, msg.target_addr);
		return false;
	}

	return true;
}

bool cxl_single_ioexp_config_init(uint8_t ioexp_name)
{
	int ret = 0;
	uint8_t retry = 5;
	uint8_t tx_len = 2;
	uint8_t ioexp_addr = 0;
	uint8_t output_0_value = 0;
	uint8_t output_1_value = 0;
	uint8_t config_0_value = 0;
	uint8_t config_1_value = 0;
	uint8_t data[tx_len];
	I2C_MSG msg = { 0 };
	memset(data, 0, sizeof(uint8_t) * tx_len);

	switch (ioexp_name) {
	case IOEXP_U14:
		ioexp_addr = CXL_IOEXP_U14_ADDR;
		output_0_value = CXL_IOEXP_U14_OUTPUT_0_REG_VAL;
		output_1_value = CXL_IOEXP_U14_OUTPUT_1_REG_VAL;
		config_0_value = CXL_IOEXP_U14_CONFIG_0_REG_VAL;
		config_1_value = CXL_IOEXP_U14_CONFIG_1_REG_VAL;
		break;
	case IOEXP_U15:
		ioexp_addr = CXL_IOEXP_U15_ADDR;
		output_0_value = CXL_IOEXP_U15_OUTPUT_0_REG_VAL;
		output_1_value = CXL_IOEXP_U15_OUTPUT_1_REG_VAL;
		config_0_value = CXL_IOEXP_U15_CONFIG_0_REG_VAL;
		config_1_value = CXL_IOEXP_U15_CONFIG_1_REG_VAL;
		break;
	case IOEXP_U16:
		ioexp_addr = CXL_IOEXP_U16_ADDR;
		output_0_value = CXL_IOEXP_U16_OUTPUT_0_REG_VAL;
		output_1_value = CXL_IOEXP_U16_OUTPUT_1_REG_VAL;
		config_0_value = CXL_IOEXP_U16_CONFIG_0_REG_VAL;
		config_1_value = CXL_IOEXP_U16_CONFIG_1_REG_VAL;
		break;
	case IOEXP_U17:
		ioexp_addr = CXL_IOEXP_U17_ADDR;
		output_0_value = CXL_IOEXP_U17_OUTPUT_0_REG_VAL;
		output_1_value = CXL_IOEXP_U17_OUTPUT_1_REG_VAL;
		config_0_value = CXL_IOEXP_U17_CONFIG_0_REG_VAL;
		config_1_value = CXL_IOEXP_U17_CONFIG_1_REG_VAL;
		break;
	default:
		LOG_ERR("CXL ioexp name: 0x%x is invalid", ioexp_name);
		return false;
	}

	/** Write cxl ioexp output 0 register **/
	data[0] = TCA9555_OUTPUT_PORT_REG_0;
	data[1] = output_0_value;
	msg = construct_i2c_message(MEB_CXL_BUS, ioexp_addr, tx_len, data, 0);

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Unable to write ioexp output 0 register bus: %u addr: 0x%02x", msg.bus,
			msg.target_addr);
		return false;
	}

	/** Write cxl ioexp output 1 register **/
	data[0] = TCA9555_OUTPUT_PORT_REG_1;
	data[1] = output_1_value;
	msg = construct_i2c_message(MEB_CXL_BUS, ioexp_addr, tx_len, data, 0);

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Unable to write ioexp output 1 register bus: %u addr: 0x%02x", msg.bus,
			msg.target_addr);
		return false;
	}

	/** Write cxl ioexp config 0 register **/
	data[0] = TCA9555_CONFIG_REG_0;
	data[1] = config_0_value;
	msg = construct_i2c_message(MEB_CXL_BUS, ioexp_addr, tx_len, data, 0);

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Unable to write ioexp config 0 register bus: %u addr: 0x%02x", msg.bus,
			msg.target_addr);
		return false;
	}

	/** Write cxl ioexp config 1 register **/
	memset(&msg, 0, sizeof(I2C_MSG));
	data[0] = TCA9555_CONFIG_REG_1;
	data[1] = config_1_value;
	msg = construct_i2c_message(MEB_CXL_BUS, ioexp_addr, tx_len, data, 0);

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Unable to write ioexp config 1 register bus: %u addr: 0x%02x", msg.bus,
			msg.target_addr);
		return false;
	}

	return true;
}

int cxl_ioexp_init(uint8_t cxl_channel)
{
	bool ret = false;
	mux_config meb_mux = { 0 };
	mux_config cxl_mux = { 0 };

	/** MEB mux for cxl channels **/
	meb_mux.bus = MEB_CXL_BUS;
	meb_mux.target_addr = CXL_FRU_MUX0_ADDR;
	meb_mux.channel = cxl_channel;

	/** CXL mux for sensor channels **/
	cxl_mux.bus = MEB_CXL_BUS;
	cxl_mux.target_addr = CXL_FRU_MUX1_ADDR;
	cxl_mux.channel = CXL_IOEXP_MUX_CHANNEL;

	/** Mutex lock bus **/
	struct k_mutex *meb_mutex = get_i2c_mux_mutex(meb_mux.bus);
	if (k_mutex_lock(meb_mutex, K_MSEC(MUTEX_LOCK_INTERVAL_MS))) {
		LOG_ERR("mutex locked failed bus%u", meb_mux.bus);
		return -1;
	}

	/** Enable mux channel **/
	ret = set_mux_channel(meb_mux, MUTEX_LOCK_ENABLE);
	if (ret == false) {
		k_mutex_unlock(meb_mutex);
		return -1;
	}

	ret = set_mux_channel(cxl_mux, MUTEX_LOCK_ENABLE);
	if (ret == false) {
		k_mutex_unlock(meb_mutex);
		return -1;
	}

	/** ALL ioexp config register initial **/
	cxl_single_ioexp_config_init(IOEXP_U14);
	cxl_single_ioexp_config_init(IOEXP_U15);
	cxl_single_ioexp_config_init(IOEXP_U16);
	cxl_single_ioexp_config_init(IOEXP_U17);

	/** ALL ioexp initial **/
	cxl_single_ioexp_alert_reset(IOEXP_U14, MUTEX_LOCK_ENABLE);
	cxl_single_ioexp_alert_reset(IOEXP_U15, MUTEX_LOCK_ENABLE);
	cxl_single_ioexp_alert_reset(IOEXP_U16, MUTEX_LOCK_ENABLE);
	cxl_single_ioexp_alert_reset(IOEXP_U17, MUTEX_LOCK_ENABLE);

	/** mutex unlock bus **/
	k_mutex_unlock(meb_mutex);

	return 0;
}

void init_cxl_card_ioexp(uint8_t cxl_id)
{
	int ret = 0;
	uint8_t retry_count = 0;
	uint8_t gpio_alert_pin = 0;

	ret = get_cxl_ioexp_alert_pin(cxl_id, &gpio_alert_pin);
	if (ret != 0) {
		LOG_ERR("Fail to get gpio alert pin, cxl id: 0x%x", cxl_id);
		return;
	}

	for (retry_count = 0; retry_count < CXL_IOEXP_INIT_RETRY_COUNT; ++retry_count) {
		if (gpio_get(gpio_alert_pin) != HIGH_ACTIVE) {
			ret = cxl_ioexp_init(cxl_work_item[cxl_id].cxl_channel);
			if (ret != 0) {
				LOG_ERR("cxl: 0x%x ioexp initial fail", cxl_id);
			}
		} else {
			return;
		}
	}
}

uint8_t pal_pm8702_read(uint8_t card_id, sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_UNSPECIFIED_ERROR;
	}
	uint8_t port = cfg->port;
	uint8_t address = cfg->target_addr;
	uint8_t pm8702_access = cfg->offset;

	mctp *mctp_inst = NULL;
	mctp_ext_params ext_params = { 0 };
	sensor_val *sval = (sensor_val *)reading;
	if (get_mctp_info_by_eid(port, &mctp_inst, &ext_params) == false) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, SENSOR_UNSPECIFIED_ERROR);

	switch (pm8702_access) {
	case chip_temp:
		if (cci_get_chip_temp(mctp_inst, ext_params, &sval->integer) == false) {
			return SENSOR_NOT_ACCESSIBLE;
		}
		sval->fraction = 0;
		break;
	case dimm_temp:
		if (pm8702_get_dimm_temp(mctp_inst, ext_params, address, &sval->integer,
					 &sval->fraction) == false) {
			return SENSOR_NOT_ACCESSIBLE;
		}
		break;
	default:
		LOG_ERR("Invalid access offset %d", pm8702_access);
		return SENSOR_PARAMETER_NOT_VALID;
	}

	return SENSOR_READ_SUCCESS;
}

bool pal_init_pm8702_info(uint8_t cxl_id)
{
	bool ret = false;
	uint8_t *req_buf = NULL;
	uint8_t req_len = GET_FW_INFO_REQ_PL_LEN;
	uint8_t resp_len = sizeof(cci_fw_info_resp);
	uint8_t resp_buf[resp_len];
	memset(resp_buf, 0, sizeof(uint8_t) * resp_len);

	ret = pal_pm8702_command_handler(cxl_id, CCI_GET_FW_INFO, req_buf, req_len, resp_buf,
					 &resp_len);
	if (ret != true) {
		LOG_ERR("Fail to get cxl id: 0x%x fw version", cxl_id);
		return false;
	}

	memcpy(&pm8702_table[cxl_id].dev_info, resp_buf, resp_len);
	pm8702_table[cxl_id].is_init = true;
	return true;
}

bool pal_get_pm8702_hbo_status(uint8_t cxl_id, uint8_t *resp_buf, uint8_t *resp_len)
{
	CHECK_NULL_ARG_WITH_RETURN(resp_buf, false);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, false);

	bool ret = false;
	uint8_t *req_buf = NULL;
	uint8_t req_len = HBO_STATUS_REQ_PL_LEN;

	ret = pal_pm8702_command_handler(cxl_id, PM8702_HBO_STATUS, req_buf, req_len, resp_buf,
					 resp_len);
	if (ret != true) {
		LOG_ERR("Fail to get cxl id: 0x%x HBO status", cxl_id);
	}

	return ret;
}

bool pal_pm8702_transfer_fw(uint8_t cxl_id, uint8_t *req_buf, int req_len)
{
	CHECK_NULL_ARG_WITH_RETURN(req_buf, false);

	bool ret = false;
	uint8_t resp_buf[TRANSFER_FW_RESP_PL_LEN];
	uint8_t resp_len = 0;

	ret = pal_pm8702_command_handler(cxl_id, PM8702_HBO_TRANSFER_FW, req_buf, req_len, resp_buf,
					 &resp_len);
	if (ret != true) {
		LOG_ERR("Fail to transfer cxl id: 0x%x firmware", cxl_id);
	}

	return ret;
}

bool pal_set_pm8702_active_slot(uint8_t cxl_id, uint8_t *req_buf, int req_len)
{
	CHECK_NULL_ARG_WITH_RETURN(req_buf, false);

	bool ret = false;
	uint8_t resp_buf[ACTIVATE_FW_RESP_PL_LEN];
	uint8_t resp_len = 0;

	ret = pal_pm8702_command_handler(cxl_id, PM8702_HBO_ACTIVATE_FW, req_buf, req_len, resp_buf,
					 &resp_len);
	if (ret != true) {
		LOG_ERR("Fail to activate cxl id: 0x%x slot firmware", cxl_id);
	}

	return ret;
}

bool pal_pm8702_command_handler(uint8_t cxl_id, uint16_t opcode, uint8_t *data_buf, int data_len,
				uint8_t *response, uint8_t *response_len)
{
	if (data_len != 0) {
		CHECK_NULL_ARG_WITH_RETURN(data_buf, false);
	}

	CHECK_NULL_ARG_WITH_RETURN(response, false);
	CHECK_NULL_ARG_WITH_RETURN(response_len, false);

	bool ret = false;
	uint8_t sensor_num = PM8702_DEFAULT_SENSOR_NUM;
	mctp *mctp_inst = NULL;
	mctp_ext_params ext_params = { 0 };

	if (is_cxl_access(cxl_id) != true) {
		LOG_ERR("Cxl id: 0x%x PM8702 can't access", cxl_id);
		return false;
	}

	if (get_mctp_info_by_eid(MCTP_EID_CXL, &mctp_inst, &ext_params) == false) {
		LOG_ERR("Fail to get mctp info via eid: 0x%x", MCTP_EID_CXL);
		return false;
	}

	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, false);

	ret = pre_cxl_switch_mux(sensor_num, (void *)&cxl_id);
	if (ret != true) {
		LOG_ERR("Pre switch mux fail, sensor num: 0x%x, cxl id: 0x%x", sensor_num, cxl_id);
		return false;
	}

	ret = pm8702_cmd_handler(mctp_inst, ext_params, opcode, data_buf, data_len, response,
				 response_len);
	if (ret != true) {
		post_cxl_switch_mux(sensor_num, (void *)&cxl_id);
		return false;
	}

	ret = post_cxl_switch_mux(sensor_num, (void *)&cxl_id);
	if (ret != true) {
		LOG_ERR("Post switch mux fail, sensor num: 0x%x, cxl id: 0x%x", sensor_num, cxl_id);
	}

	return true;
}

void monitor_ssd_power_fault()
{
	while (1) {
		uint8_t index = 0;
		uint8_t retry = 3;
		uint8_t status = 0;
		uint8_t bit_count = 8;
		I2C_MSG i2c_msg = { 0 };

		i2c_msg.bus = CPLD_BUS;
		i2c_msg.target_addr = CPLD_ADDR;
		i2c_msg.tx_len = 1;
		i2c_msg.rx_len = 1;
		i2c_msg.data[0] = CPLD_SSD_POWER_FAULT_REG;
		if (i2c_master_read(&i2c_msg, retry)) {
			LOG_ERR("Get SSD power fault register value fail");
		} else {
			for (index = 0; index < bit_count; ++index) {
				if (ssd_power_fault[index / 2] != true) {
					if (i2c_msg.data[0] & BIT(index)) {
						status =
							((index % 2) ?
								 PLDM_STATE_SET_OEM_DEVICE_12V_AUX_POWER_FAULT :
								 PLDM_STATE_SET_OEM_DEVICE_3V3_AUX_FAULT);
						plat_send_ssd_power_fault_event(index / 2, status);
						ssd_power_fault[index / 2] = true;
					}
				}
			}
		}

		k_msleep(MONITOR_SSD_POWER_FAULT_DELAY_MS);
	}
}

void init_ssd_power_fault_thread()
{
	if (ssd_power_fault_tid != NULL &&
	    (strcmp(k_thread_state_str(ssd_power_fault_tid), "dead") != 0) &&
	    (strcmp(k_thread_state_str(ssd_power_fault_tid), "unknown") != 0)) {
		return;
	}
	ssd_power_fault_tid =
		k_thread_create(&ssd_power_fault_thread_handler, monitor_ssd_power_fault_thread,
				K_THREAD_STACK_SIZEOF(monitor_ssd_power_fault_thread),
				monitor_ssd_power_fault, NULL, NULL, NULL,
				CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&ssd_power_fault_thread_handler, "ssd_power_fault_thread");
}

void abort_ssd_power_fault_thread()
{
	if (ssd_power_fault_tid != NULL &&
	    (strcmp(k_thread_state_str(ssd_power_fault_tid), "dead") != 0) &&
	    (strcmp(k_thread_state_str(ssd_power_fault_tid), "unknown") != 0)) {
		k_thread_abort(ssd_power_fault_tid);
	}
}

void init_ssd_power_fault_work()
{
	if (get_DC_status()) {
		init_ssd_power_fault_thread();
	} else {
		abort_ssd_power_fault_thread();
	}
}
