#include <stdlib.h>
#include "storage_handler.h"
#include "plat_fru.h"
#include "plat_ipmb.h"
#include "fru.h"
#include "sdr.h"

__weak void STORAGE_GET_FRUID_INFO(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	uint8_t fruid;
	uint16_t fru_size;

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	fruid = msg->data[0];

	// check if FRU is defined
	if (fruid >= MAX_FRU_ID) {
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	fru_size = find_FRU_size(fruid);
	// indicate ID not found
	if (fru_size == 0xFFFF) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	msg->data[0] = fru_size & 0xFF; // lsb
	msg->data[1] = (fru_size >> 8) & 0xFF; // msb
	msg->data[2] = get_FRU_access(fruid); // access type
	msg->data_len = 3;
	msg->completion_code = CC_SUCCESS;

	return;
}

__weak void STORAGE_READ_FRUID_DATA(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	uint8_t status;
	EEPROM_ENTRY fru_entry;

	if (msg->data_len != 4) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	fru_entry.config.dev_id = msg->data[0];
	fru_entry.offset = (msg->data[2] << 8) | msg->data[1];
	fru_entry.data_len = msg->data[3];

	// According to IPMI, messages are limited to 32 bytes
	if (fru_entry.data_len > 32) {
		msg->completion_code = CC_LENGTH_EXCEEDED;
		return;
	}

	status = FRU_read(&fru_entry);

	msg->data_len = fru_entry.data_len + 1;
	msg->data[0] = fru_entry.data_len;
	memcpy(&msg->data[1], &fru_entry.data[0], fru_entry.data_len);

	switch (status) {
	case FRU_READ_SUCCESS:
		msg->completion_code = CC_SUCCESS;
		break;
	case FRU_INVALID_ID:
		msg->completion_code = CC_INVALID_PARAM;
		break;
	case FRU_OUT_OF_RANGE:
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		break;
	case FRU_FAIL_TO_ACCESS:
		msg->completion_code = CC_FRU_DEV_BUSY;
		break;
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}

	return;
}

__weak void STORAGE_WRITE_FRUID_DATA(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	uint8_t status;
	EEPROM_ENTRY fru_entry;

	if (msg->data_len < 4) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	fru_entry.config.dev_id = msg->data[0];
	fru_entry.offset = (msg->data[2] << 8) | msg->data[1];
	fru_entry.data_len = msg->data_len - 3; // skip id and offset
	if (fru_entry.data_len > 32) { // According to IPMI, messages are limited to 32 bytes
		msg->completion_code = CC_LENGTH_EXCEEDED;
		return;
	}
	memcpy(&fru_entry.data[0], &msg->data[3], fru_entry.data_len);

	msg->data[0] = msg->data_len - 3;
	msg->data_len = 1;
	status = FRU_write(&fru_entry);

	switch (status) {
	case FRU_WRITE_SUCCESS:
		msg->completion_code = CC_SUCCESS;
		break;
	case FRU_INVALID_ID:
		msg->completion_code = CC_INVALID_PARAM;
		break;
	case FRU_OUT_OF_RANGE:
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		break;
	case FRU_FAIL_TO_ACCESS:
		msg->completion_code = CC_FRU_DEV_BUSY;
		break;
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}

	return;
}

__weak void STORAGE_RSV_SDR(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	uint16_t RSV_ID;
	uint8_t rsv_table_index = RSV_TABLE_INDEX_0;

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	if (msg->InF_source == SLOT3_BIC) {
		rsv_table_index = RSV_TABLE_INDEX_1;
	}

	RSV_ID = SDR_get_RSV_ID(rsv_table_index);
	msg->data[0] = RSV_ID & 0xFF;
	msg->data[1] = (RSV_ID >> 8) & 0xFF;
	msg->data_len = 2;
	msg->completion_code = CC_SUCCESS;

	return;
}

__weak void STORAGE_GET_SDR(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	uint16_t next_record_ID;
	uint16_t rsv_ID, record_ID;
	uint8_t offset, req_len;
	uint8_t *table_ptr;
	uint8_t rsv_table_index = RSV_TABLE_INDEX_0;

	// Config D: slot1 and slot3 need to use different reservation id
	if (msg->InF_source == SLOT3_BIC) {
		rsv_table_index = RSV_TABLE_INDEX_1;
	}

	rsv_ID = (msg->data[1] << 8) | msg->data[0];
	record_ID = (msg->data[3] << 8) | msg->data[2];
	offset = msg->data[4];
	req_len = msg->data[5];

	if (msg->data_len != 6) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	// If SDR_RSV_ID_check gets false is represent reservation id is incorrect
	if (SDR_RSV_ID_check(rsv_ID, rsv_table_index) == false) {
		msg->completion_code = CC_INVALID_RESERVATION;
		return;
	}

	if (!SDR_check_record_ID(record_ID)) {
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	// request length + next record ID should not over IPMB data limit
	if ((req_len + 2) > IPMI_DATA_MAX_LENGTH) {
		msg->completion_code = CC_LENGTH_EXCEEDED;
		return;
	}

	if ((offset + req_len) > IPMI_SDR_HEADER_LEN + full_sdr_table[record_ID].record_len) {
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		return;
	}

	next_record_ID = SDR_get_record_ID(record_ID);
	msg->data[0] = next_record_ID & 0xFF;
	msg->data[1] = (next_record_ID >> 8) & 0xFF;

	table_ptr = (uint8_t *)&full_sdr_table[record_ID];
	memcpy(&msg->data[2], (table_ptr + offset), req_len);

	msg->data_len = req_len + 2; // return next record ID + sdr data
	msg->completion_code = CC_SUCCESS;

	return;
}

__weak void STORAGE_ADD_SEL(ipmi_msg *msg)
{
	if (msg == NULL) {
		printf("%s failed due to parameter is NULL\n", __func__);
		return;
	}
#if MAX_IPMB_IDX
	if (msg->data_len != 16) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	ipmb_error status;
	ipmi_msg *add_sel_msg;

	add_sel_msg = (ipmi_msg *)malloc(sizeof(ipmi_msg));
	if (add_sel_msg == NULL) {
		printf("%s malloc fail\n", __func__);
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	memset(add_sel_msg, 0, sizeof(ipmi_msg));
	add_sel_msg->InF_source = SELF;
	add_sel_msg->InF_target = BMC_IPMB;
	add_sel_msg->netfn = NETFN_STORAGE_REQ;
	add_sel_msg->cmd = CMD_STORAGE_ADD_SEL;
	add_sel_msg->data_len = msg->data_len;
	memcpy(add_sel_msg->data, msg->data, add_sel_msg->data_len);

	status = ipmb_read(add_sel_msg, IPMB_inf_index_map[add_sel_msg->InF_target]);
	free(add_sel_msg);

	msg->data_len = 0;
	if (status == IPMB_ERROR_FAILURE) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		printf("%s Fail to post msg to txqueue for addsel\n", __func__);
		return;
	} else if (status == IPMB_ERROR_GET_MESSAGE_QUEUE) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		printf("%s No response from bmc for addsel\n", __func__);
		return;
	}

	msg->completion_code = CC_SUCCESS;
#else
	msg->completion_code = CC_UNSPECIFIED_ERROR;
#endif
	return;
}

void IPMI_Storage_handler(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	switch (msg->cmd) {
	case CMD_STORAGE_GET_FRUID_INFO:
		STORAGE_GET_FRUID_INFO(msg);
		break;
	case CMD_STORAGE_READ_FRUID_DATA:
		STORAGE_READ_FRUID_DATA(msg);
		break;
	case CMD_STORAGE_WRITE_FRUID_DATA:
		STORAGE_WRITE_FRUID_DATA(msg);
		break;
	case CMD_STORAGE_RSV_SDR:
		STORAGE_RSV_SDR(msg);
		break;
	case CMD_STORAGE_GET_SDR:
		STORAGE_GET_SDR(msg);
		break;
	case CMD_STORAGE_ADD_SEL:
		STORAGE_ADD_SEL(msg);
		break;
	default:
		printf("invalid Storage msg netfn: %x, cmd: %x\n", msg->netfn, msg->cmd);
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_CMD;
		break;
	}
	return;
}
