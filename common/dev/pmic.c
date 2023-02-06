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
#include "ipmb.h"
#include "ipmi.h"
#include "pmic.h"
#include "sensor.h"
#include "libutil.h"
#include "plat_ipmb.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(pmic);

uint8_t *compose_memory_write_read_req(uint8_t smbus_identifier, uint8_t smbus_address,
				       uint32_t addr_value, uint8_t *write_data, uint8_t write_len)
{
	if ((write_len != 0) && (write_data == NULL)) {
		LOG_ERR("Input parameter write_data is NULL");
		return NULL;
	}

	static memory_write_read_req pmic_req;
	pmic_req.intel_id = INTEL_ID;
	pmic_req.smbus_identifier = smbus_identifier;
	pmic_req.smbus_address = smbus_address;
	pmic_req.addr_size = PMIC_ADDR_SIZE;
	pmic_req.addr_value = addr_value;
	pmic_req.data_len = PMIC_DATA_LEN;

	memset(pmic_req.write_data, 0, MAX_MEMORY_DATA * sizeof(uint8_t));
	memcpy(pmic_req.write_data, write_data, write_len);

	return (uint8_t *)&pmic_req;
}

int pmic_ipmb_transfer(int *total_pmic_power, uint8_t seq_source, uint8_t netFn, uint8_t command,
		       uint8_t source_inft, uint8_t target_inft, uint16_t data_len, uint8_t *data)
{
	// Need input pointer to record total pmic power if command is memory read command
	if ((command == CMD_SMBUS_READ_MEMORY) && (total_pmic_power == NULL)) {
		LOG_ERR("Input total pmic power is NULL  cmd: 0x%x", command);
		return -1;
	}

	// No need input pointer to record total pmic power if command is memory write/cold reset command
	if ((command == CMD_SMBUS_WRITE_MEMORY || command == CMD_APP_COLD_RESET) &&
	    (total_pmic_power != NULL)) {
		LOG_ERR("Input total pmic power is not NULL  cmd: 0x%x", command);
		return -1;
	}

	// Input data pointer not match with data length
	if ((data_len != 0) && (data == NULL)) {
		LOG_ERR("Input data pointer/length in invaild  data_len: %d",
		       data_len);
		return -1;
	}

	ipmi_msg *pmic_msg = (ipmi_msg *)malloc(sizeof(ipmi_msg));
	if (pmic_msg == NULL) {
		LOG_ERR("Failed to allocate memory");
		return -1;
	}

	memset(pmic_msg, 0, sizeof(ipmi_msg));
	*pmic_msg = construct_ipmi_message(seq_source, netFn, command, source_inft, target_inft,
					   data_len, data);
	ipmb_error ret = ipmb_read(pmic_msg, IPMB_inf_index_map[pmic_msg->InF_target]);

	if ((ret != IPMB_ERROR_SUCCESS) || (pmic_msg->completion_code != CC_SUCCESS)) {
		LOG_ERR("Failed to send pmic_command ret: 0x%x CC: 0x%x", ret,
		       pmic_msg->completion_code);
		SAFE_FREE(pmic_msg);
		return -1;
	}

	if (pmic_msg->data_len < 4) {
		LOG_DBG("pmic res data_len: 0x%x", pmic_msg->data_len);
		SAFE_FREE(pmic_msg);
		return -1;
	}

	if (total_pmic_power != NULL) {
		*total_pmic_power = pmic_msg->data[3] * PMIC_TOTAL_POWER_MW;
	}

	SAFE_FREE(pmic_msg);
	return 0;
}

uint8_t pmic_read(uint8_t sensor_num, int *reading)
{
	if (reading == NULL) {
		LOG_ERR("Input parameter reading is NULL");
		return SENSOR_UNSPECIFIED_ERROR;
	}

	if ((sensor_num > SENSOR_NUM_MAX) ||
	    (sensor_config[sensor_config_index_map[sensor_num]].init_args == NULL)) {
		LOG_ERR("Sensor 0x%x input parameter is invaild", sensor_num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	pmic_init_arg *pmic_arg = sensor_config[sensor_config_index_map[sensor_num]].init_args;
	if (pmic_arg->is_init == false) {
		LOG_ERR("Device isn't initialized");
		return SENSOR_NOT_FOUND;
	}
#if MAX_IPMB_IDX
	int total_pmic_power = 0, ret = 0;
	uint8_t seq_source = 0xFF;
	uint8_t *compose_memory_write_read_msg = compose_memory_write_read_req(
		pmic_arg->smbus_bus_identifier, pmic_arg->smbus_addr, PMIC_SWA_ADDR_VAL, NULL, 0);
	if (compose_memory_write_read_msg == NULL) {
		LOG_ERR("Compose memory write read msg is NULL");
		return SENSOR_UNSPECIFIED_ERROR;
	}

	ret = pmic_ipmb_transfer(&total_pmic_power, seq_source, NETFN_NM_REQ, CMD_SMBUS_READ_MEMORY,
				 SELF, ME_IPMB, PMIC_READ_DATA_LEN, compose_memory_write_read_msg);
	if (ret != 0) {
		LOG_ERR("PMIC ipmb transfer error");
		return SENSOR_FAIL_TO_ACCESS;
	}

	memset(reading, 0, sizeof(int));
	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (total_pmic_power / 1000) & 0xFFFF;
	sval->fraction = (total_pmic_power % 1000) & 0xFFFF;

	return SENSOR_READ_SUCCESS;
#else
	return SENSOR_FAIL_TO_ACCESS;
#endif
}

uint8_t pmic_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		LOG_ERR("Input sensor number 0x%x is invaild", sensor_num);
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}
	sensor_config[sensor_config_index_map[sensor_num]].read = pmic_read;
	pmic_init_arg *init_arg = sensor_config[sensor_config_index_map[sensor_num]].init_args;
	init_arg->is_init = true;
	return SENSOR_INIT_SUCCESS;
}

__weak int pal_set_pmic_error_flag(uint8_t dimm_id, uint8_t error_type)
{
	return -1;
}
