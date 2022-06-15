#include <stdio.h>
#include <stdlib.h>
#include "ipmb.h"
#include "ipmi.h"
#include "pmic.h"
#include "sensor.h"
#include "libutil.h"
#include "plat_ipmb.h"
uint8_t *compose_memory_write_read_req(uint8_t smbus_identifier, uint8_t smbus_address,
				       uint32_t addr_value, uint8_t *write_data, uint8_t write_len)
{
	if ((write_len != 0) && (write_data == NULL)) {
		printf("[%s] input parameter write_data is NULL\n", __func__);
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
		printf("[%s] input total pmic power is NULL  cmd: 0x%x\n", __func__, command);
		return -1;
	}

	// No need input pointer to record total pmic power if command is memory write/cold reset command
	if ((command == CMD_SMBUS_WRITE_MEMORY || command == CMD_APP_COLD_RESET) &&
	    (total_pmic_power != NULL)) {
		printf("[%s] input total pmic power is not NULL  cmd: 0x%x\n", __func__, command);
		return -1;
	}

	// Input data pointer not match with data length
	if ((data_len != 0) && (data == NULL)) {
		printf("[%s] input data pointer/length in invaild  data_len: %d\n", __func__,
		       data_len);
		return -1;
	}

	ipmi_msg *pmic_msg = (ipmi_msg *)malloc(sizeof(ipmi_msg));
	if (pmic_msg == NULL) {
		printf("[%s] Failed to allocate memory\n", __func__);
		return -1;
	}

	memset(pmic_msg, 0, sizeof(ipmi_msg));
	*pmic_msg = construct_ipmi_message(seq_source, netFn, command, source_inft, target_inft,
					   data_len, data);
	ipmb_error ret = ipmb_read(pmic_msg, IPMB_inf_index_map[pmic_msg->InF_target]);

	if ((ret != IPMB_ERROR_SUCCESS) || (pmic_msg->completion_code != CC_SUCCESS)) {
		printf("[%s] Failed to send pmic_command ret: 0x%x CC: 0x%x\n", __func__, ret,
		       pmic_msg->completion_code);
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
		printf("[%s] input parameter reading is NULL\n", __func__);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	if ((sensor_num > SENSOR_NUM_MAX) ||
	    (sensor_config[sensor_config_index_map[sensor_num]].init_args == NULL)) {
		printf("[%s] sensor 0x%x input parameter is invaild\n", __func__, sensor_num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	pmic_init_arg *pmic_arg = sensor_config[sensor_config_index_map[sensor_num]].init_args;
	if (pmic_arg->is_init == false) {
		printf("[%s] device isn't initialized\n", __func__);
		return SENSOR_NOT_FOUND;
	}
#if MAX_IPMB_IDX
	int total_pmic_power = 0, ret = 0;
	uint8_t seq_source = 0xFF;
	uint8_t *compose_memory_write_read_msg = compose_memory_write_read_req(
		pmic_arg->smbus_bus_identifier, pmic_arg->smbus_addr, PMIC_SWA_ADDR_VAL, NULL, 0);
	if (compose_memory_write_read_msg == NULL) {
		printf("[%s] compose memory write read msg is NULL\n", __func__);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	ret = pmic_ipmb_transfer(&total_pmic_power, seq_source, NETFN_NM_REQ, CMD_SMBUS_READ_MEMORY,
				 SELF, ME_IPMB, PMIC_READ_DATA_LEN, compose_memory_write_read_msg);
	if (ret != 0) {
		printf("[%s] PMIC ipmb transfer error\n", __func__);
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
		printf("[%s] input sensor number 0x%x is invaild\n", __func__, sensor_num);
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}
	sensor_config[sensor_config_index_map[sensor_num]].read = pmic_read;
	pmic_init_arg *init_arg = sensor_config[sensor_config_index_map[sensor_num]].init_args;
	init_arg->is_init = true;
	return SENSOR_INIT_SUCCESS;
}
