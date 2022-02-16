#include <zephyr.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "plat_i2c.h"
#include "sensor.h"
#include "ipmi.h"
#include "plat_ipmi.h"
#include "hal_gpio.h"
#include "plat_gpio.h"
#include "ipmi_def.h"
#include "guid.h"
#include "plat_guid.h"
#include "fru.h"
#include "plat_fru.h"
#include "sensor_def.h"
#include "util_spi.h"

bool pal_is_not_return_cmd(uint8_t netfn, uint8_t cmd)
{
	if ((netfn == NETFN_OEM_1S_REQ) && (cmd == CMD_OEM_1S_MSG_OUT)) {
		return 1;
	} else if ((netfn == NETFN_OEM_1S_REQ) && (cmd == CMD_OEM_1S_MSG_IN)) {
		return 1;
	}

	// Reserve for future commands

	return 0;
}

void pal_APP_GET_DEVICE_ID(ipmi_msg *msg)
{
	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	msg->data[0] = DEVICE_ID;
	msg->data[1] = DEVICE_REVISION;
	msg->data[2] = FIRMWARE_REVISION_1;
	msg->data[3] = FIRMWARE_REVISION_2;
	msg->data[4] = IPMI_VERSION;
	msg->data[5] = ADDITIONAL_DEVICE_SUPPORT;
	msg->data[6] = (WW_IANA_ID & 0xFF);
	msg->data[7] = (WW_IANA_ID >> 8) & 0xFF;
	msg->data[8] = (WW_IANA_ID >> 16) & 0xFF;
	msg->data[9] = (PRODUCT_ID & 0xFF);
	msg->data[10] = (PRODUCT_ID >> 8) & 0xFF;
	msg->data[11] = (AUXILIARY_FW_REVISION >> 24) & 0xFF;
	msg->data[12] = (AUXILIARY_FW_REVISION >> 16) & 0xFF;
	msg->data[13] = (AUXILIARY_FW_REVISION >> 8) & 0xFF;
	msg->data[14] = (AUXILIARY_FW_REVISION & 0xFF);
	msg->data_len = 15;
	msg->completion_code = CC_SUCCESS;

	return;
}

void pal_APP_GET_SELFTEST_RESULTS(ipmi_msg *msg)
{
	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t test_result = 0;
	// CannotAccessSel
	test_result = (test_result | GET_TEST_RESULT) << 1;
	// CannotAccessSdrr
	test_result = (test_result | is_SDR_not_init) << 1;
	// CannotAccessFru
	// Get common header
	uint8_t i, status, checksum = 0;
	EEPROM_ENTRY fru_entry;

	fru_entry.config.dev_id = 0;
	fru_entry.offset = 0;
	fru_entry.data_len = 8;
	status = FRU_read(&fru_entry);
	for (i = 0; i < fru_entry.data_len; i++) {
		checksum += fru_entry.data[i];
	}

	if (checksum == 0) {
		test_result = (test_result | 0) << 1;
	} else {
		test_result = (test_result | 1) << 1;
	}
	// IpmbLinesDead
	test_result = (test_result | GET_TEST_RESULT) << 1;
	// SdrrEmpty
	test_result = (test_result | is_SDR_not_init) << 1;
	// InternalCorrupt
	if (checksum == 0) {
		test_result = (test_result | 0) << 1;
	} else {
		test_result = (test_result | 1) << 1;
	}

	// UpdateFWCorrupt
	test_result = (test_result | GET_TEST_RESULT) << 1;
	// OpFWCorrupt
	test_result = test_result | GET_TEST_RESULT;

	msg->data[0] = test_result == 0x00 ? 0x55 : 0x57;
	msg->data[1] = test_result == 0x00 ? 0x00 : test_result;
	msg->data_len = 2;
	msg->completion_code = CC_SUCCESS;

	return;
}

void pal_APP_MASTER_WRITE_READ(ipmi_msg *msg)
{
	uint8_t retry = 3;
	uint8_t bus_7bit;
	I2C_MSG i2c_msg;

	if (msg->data_len < 4) { // at least include bus, addr, rx_len, offset
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	bus_7bit = ((msg->data[0] - 1) >> 1); // should ignore bit0, all bus public
	if (bus_7bit >= I2C_BUS_NUM) {
		printk("Accessing invalid bus with IPMI master write read\n");
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		return;
	}

	i2c_msg.bus = i2c_bus_to_index[bus_7bit];
	i2c_msg.slave_addr = (msg->data[1] >> 1); // 8 bit address to 7 bit
	i2c_msg.rx_len = msg->data[2];
	i2c_msg.tx_len = msg->data_len - 3;

	if (i2c_msg.tx_len == 0) {
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	memcpy(&i2c_msg.data[0], &msg->data[3], i2c_msg.tx_len);
	msg->data_len = i2c_msg.rx_len;

	if (i2c_msg.rx_len == 0) {
		if (!i2c_master_write(&i2c_msg, retry)) {
			msg->completion_code = CC_SUCCESS;
		} else {
			msg->completion_code = CC_I2C_BUS_ERROR;
		}
	} else {
		if (!i2c_master_read(&i2c_msg, retry)) {
			memcpy(&msg->data[0], &i2c_msg.data, i2c_msg.rx_len);
			msg->completion_code = CC_SUCCESS;
		} else {
			msg->completion_code = CC_I2C_BUS_ERROR;
		}
	}

	return;
}

void pal_STORAGE_GET_FRUID_INFO(ipmi_msg *msg)
{
	uint8_t fruid;
	uint16_t fru_size;

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	fruid = msg->data[0];

	if (fruid >= MAX_FRU_ID) { // check if FRU is defined
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	fru_size = find_FRU_size(fruid);
	if (fru_size == 0xFFFF) { // indicate ID not found
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

void pal_STORAGE_READ_FRUID_DATA(ipmi_msg *msg)
{
	uint8_t status;
	EEPROM_ENTRY fru_entry;

	if (msg->data_len != 4) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	fru_entry.config.dev_id = msg->data[0];
	fru_entry.offset = (msg->data[2] << 8) | msg->data[1];
	fru_entry.data_len = msg->data[3];

	if (fru_entry.data_len > 32) { // According to IPMI, messages are limited to 32 bytes
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

void pal_STORAGE_WRITE_FRUID_DATA(ipmi_msg *msg)
{
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

void pal_STORAGE_RSV_SDR(ipmi_msg *msg)
{
	uint16_t RSV_ID;

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	RSV_ID = SDR_get_RSV_ID();
	msg->data[0] = RSV_ID & 0xFF;
	msg->data[1] = (RSV_ID >> 8) & 0xFF;
	msg->data_len = 2;
	msg->completion_code = CC_SUCCESS;

	return;
}

void pal_STORAGE_GET_SDR(ipmi_msg *msg)
{
	uint16_t next_record_ID;
	uint16_t rsv_ID, record_ID;
	uint8_t offset, req_len;
	uint8_t *table_ptr;

	rsv_ID = (msg->data[1] << 8) | msg->data[0];
	record_ID = (msg->data[3] << 8) | msg->data[2];
	offset = msg->data[4];
	req_len = msg->data[5];

	if (msg->data_len != 6) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	if (!SDR_RSV_ID_check(rsv_ID)) {
		msg->completion_code = CC_INVALID_RESERVATION;
		return;
	}

	if (!SDR_check_record_ID(record_ID)) {
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	if ((req_len + 2) >
	    IPMI_DATA_MAX_LENGTH) { // request length + next record ID should not over IPMB data limit
		msg->completion_code = CC_LENGTH_EXCEEDED;
		return;
	}

	if ((offset + req_len) > IPMI_SDR_HEADER_LEN + full_sensor_table[record_ID].record_len) {
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		return;
	}

	next_record_ID = SDR_get_record_ID(record_ID);
	msg->data[0] = next_record_ID & 0xFF;
	msg->data[1] = (next_record_ID >> 8) & 0xFF;

	table_ptr = (uint8_t *)&full_sensor_table[record_ID];
	memcpy(&msg->data[2], (table_ptr + offset), req_len);

	msg->data_len = req_len + 2; // return next record ID + sdr data
	msg->completion_code = CC_SUCCESS;

	return;
}

void pal_SENSOR_GET_SENSOR_READING(ipmi_msg *msg)
{
	uint8_t status, snr_num;
	int reading;

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	if (!enable_sensor_poll) {
		printf("Reading sensor cache while sensor polling disable\n");
		msg->completion_code = CC_CAN_NOT_RESPOND;
		return;
	}

	snr_num = msg->data[0];
	status = get_sensor_reading(
		snr_num, &reading,
		get_from_cache); // Fix to get_from_cache. As need real time reading, use OEM command to get_from_sensor.

	switch (status) {
	case SNR_READ_SUCCESS:
		msg->data[0] = reading;
		// SDR sensor initialization bit6 enable scan, bit5 enable event
		// retunr data 1 bit 7 set to 0 to disable all event msg. bit 6 set to 0 disable sensor scan
		msg->data[1] =
			((full_sensor_table[SnrNum_SDR_map[snr_num]].sensor_init & 0x60) << 1);
		msg->data[2] =
			0xc0; // fix to threshold deassert status, BMC will compare with UCR/UNR itself
		msg->data_len = 3;
		msg->completion_code = CC_SUCCESS;
		break;
	case SNR_FAIL_TO_ACCESS:
		msg->completion_code = CC_NODE_BUSY; // transection error
		break;
	case SNR_NOT_ACCESSIBLE:
		msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE; // DC off
		break;
	case SNR_NOT_FOUND:
		msg->completion_code = CC_INVALID_DATA_FIELD; // request sensor number not found
		break;
	case SNR_UNSPECIFIED_ERROR:
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR; // unknown error
		break;
	}
	return;
}

void pal_OEM_GET_MB_INDEX(ipmi_msg *msg)
{
	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	if (msg->InF_source == SLOT1_BIC_IFs) {
		msg->data[0] = 0x01;
	} else if (msg->InF_source == SLOT3_BIC_IFs) {
		msg->data[0] = 0x03;
	} else {
		msg->data[0] = 0xFF;
		return;
	}

	msg->data_len = 1;
	msg->completion_code = CC_SUCCESS;
	return;
}

void pal_OEM_1S_MSG_OUT(ipmi_msg *msg)
{
	uint8_t target_IF;
	ipmb_error status;
	ipmi_msg *bridge_msg;

	if (msg->completion_code != CC_INVALID_IANA) {
		msg->completion_code = CC_SUCCESS;
	}

	if (msg->data_len <= 2) { // Should input target, netfn, cmd
		msg->completion_code = CC_INVALID_LENGTH;
	}

	target_IF = msg->data[0];

	if ((IPMB_config_table[IPMB_inf_index_map[target_IF]].Inf == Reserve_IFs) ||
	    (IPMB_config_table[IPMB_inf_index_map[target_IF]].EnStatus ==
	     Disable)) { // Bridge to invalid or disabled interface
		printf("OEM_MSG_OUT: Invalid bridge interface: %x\n", target_IF);
		msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
	}

	if (msg->completion_code == CC_SUCCESS) { // only send to target while msg is valid
		bridge_msg = (ipmi_msg *)malloc(sizeof(ipmi_msg));
		if (bridge_msg == NULL) {
			msg->completion_code = CC_OUT_OF_SPACE;
		} else {
			memset(bridge_msg, 0, sizeof(ipmi_msg));

			if (DEBUG_IPMI) {
				printf("bridge targetIf %x, len %d, netfn %x, cmd %x\n", target_IF,
				       msg->data_len, msg->data[1] >> 2, msg->data[2]);
			}

			bridge_msg->data_len = msg->data_len - 3;
			bridge_msg->seq_source = msg->seq_source;
			bridge_msg->InF_target = msg->data[0];
			bridge_msg->InF_source = msg->InF_source;
			bridge_msg->netfn = msg->data[1] >> 2;
			bridge_msg->cmd = msg->data[2];

			if (bridge_msg->data_len != 0) {
				memcpy(&bridge_msg->data[0], &msg->data[3],
				       bridge_msg->data_len * sizeof(msg->data[0]));
			}

			status = ipmb_send_request(bridge_msg, IPMB_inf_index_map[target_IF]);

			if (status != ipmb_error_success) {
				printf("OEM_MSG_OUT send IPMB req fail status: %x", status);
				msg->completion_code = CC_BRIDGE_MSG_ERR;
			}
			free(bridge_msg);
		}
	}

	if (msg->completion_code !=
	    CC_SUCCESS) { // Return to source while data is invalid or sending req to Tx task fail

		msg->data_len = 0;
		status = ipmb_send_response(msg, IPMB_inf_index_map[msg->InF_source]);

		if (status != ipmb_error_success) {
			printf("OEM_MSG_OUT send IPMB resp fail status: %x", status);
		}
	}

	return;
}

void pal_OEM_1S_GET_GPIO(ipmi_msg *msg)
{
	if (msg->data_len != 0) { // only input enable status
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t eight_bit_value = 0, gpio_value, gpio_cnt, data_len;
	gpio_cnt = gpio_ind_to_num_table_cnt +
		   (8 - (gpio_ind_to_num_table_cnt %
			 8)); // Bump up the gpio_ind_to_num_table_cnt to multiple of 8.
	data_len = gpio_cnt / 8;
	msg->data_len = data_len;
	for (uint8_t i = 0; i < gpio_cnt; i++) {
		gpio_value =
			(i >= gpio_ind_to_num_table_cnt) ? 0 : gpio_get(gpio_ind_to_num_table[i]);

		switch (i % 8) {
		case 0:
			eight_bit_value = gpio_value;
			break;
		case 1:
			eight_bit_value = eight_bit_value | (gpio_value << 1);
			break;
		case 2:
			eight_bit_value = eight_bit_value | (gpio_value << 2);
			break;
		case 3:
			eight_bit_value = eight_bit_value | (gpio_value << 3);
			break;
		case 4:
			eight_bit_value = eight_bit_value | (gpio_value << 4);
			break;
		case 5:
			eight_bit_value = eight_bit_value | (gpio_value << 5);
			break;
		case 6:
			eight_bit_value = eight_bit_value | (gpio_value << 6);
			break;
		case 7:
			eight_bit_value = eight_bit_value | (gpio_value << 7);
			break;
		}

		msg->data[i / 8] = eight_bit_value;
	}

	msg->completion_code = CC_SUCCESS;
	return;
}

void pal_OEM_1S_GET_SET_GPIO(ipmi_msg *msg)
{
	uint8_t completion_code = CC_INVALID_LENGTH;
	uint8_t gpio_num = gpio_ind_to_num_table[msg->data[1]];

	do {
		if (msg->data[0] == 0) { // Get GPIO output status
			if (msg->data_len != 2) {
				break;
			}
			msg->data[0] = gpio_num;
			msg->data[1] = gpio_get(gpio_num);
			completion_code = CC_SUCCESS;

		} else if (msg->data[0] == 1) { // Set GPIO output status
			if (msg->data_len != 3) {
				break;
			}
			msg->data[0] = gpio_num;
			gpio_conf(gpio_num, GPIO_OUTPUT);
			gpio_set(gpio_num, msg->data[2]);
			msg->data[1] = gpio_get(gpio_num);
			completion_code = CC_SUCCESS;

		} else if (msg->data[0] == 2) { // Get GPIO direction status
			if (msg->data_len != 2) {
				break;
			}
			msg->data[0] = gpio_num;
			completion_code = CC_NOT_SUPP_IN_CURR_STATE;

		} else if (msg->data[0] == 3) { // Set GPIO direction status
			if (msg->data_len != 3) {
				break;
			}
			if (msg->data[2]) {
				gpio_conf(gpio_num, GPIO_OUTPUT);
			} else {
				gpio_conf(gpio_num, GPIO_INPUT);
			}
			msg->data[0] = gpio_num;
			msg->data[1] = msg->data[2];
			completion_code = CC_SUCCESS;
		}
	} while (0);

	msg->data_len = 2; // Return GPIO number, status
	msg->completion_code = completion_code;
	return;
}

void pal_OEM_1S_FW_UPDATE(ipmi_msg *msg)
{
	/*********************************
 * buf 0:   target, 0x80 indicate last byte
 * buf 1~4: offset, 1 lsb
 * buf 5~6: length, 5 lsb
 * buf 7~N: data
 ***********************************/
	if (msg->data_len < 8) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t target = msg->data[0], status = 0;
	uint32_t offset =
		((msg->data[4] << 24) | (msg->data[3] << 16) | (msg->data[2] << 8) | msg->data[1]);
	uint16_t length = ((msg->data[6] << 8) | msg->data[5]);

	if ((length == 0) || (length != msg->data_len - 7)) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	if ((target == BIC_UPDATE) || (target == (BIC_UPDATE | UPDATE_EN))) {
		if (offset > 0x50000) { // Expect BIC firmware size not bigger than 320k
			msg->completion_code = CC_PARAM_OUT_OF_RANGE;
			return;
		}
		status = fw_update(offset, length, &msg->data[7], (target & UPDATE_EN),
				   devspi_fmc_cs0);
	}

	msg->data_len = 0;

	switch (status) {
	case fwupdate_success:
		msg->completion_code = CC_SUCCESS;
		break;
	case fwupdate_out_of_heap:
		msg->completion_code = CC_LENGTH_EXCEEDED;
		break;
	case fwupdate_over_length:
		msg->completion_code = CC_OUT_OF_SPACE;
		break;
	case fwupdate_repeated_updated:
		msg->completion_code = CC_INVALID_DATA_FIELD;
		break;
	case fwupdate_update_fail:
		msg->completion_code = CC_TIMEOUT;
		break;
	case fwupdate_error_offset:
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		break;
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}
	if (status != fwupdate_success) {
		printf("spi fw cc: %x\n", msg->completion_code);
	}

	return;
}

void pal_OEM_1S_I2C_DEV_SCAN(ipmi_msg *msg)
{
	if (msg->data[0] == 0x9C && msg->data[1] == 0x9C && msg->data[2] == 0x00) {
		while (1)
			; // hold firmware for debug only
	}

	if (msg->data_len != 1) { // only input scan bus
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t bus = i2c_bus_to_index[msg->data[0]];

	i2c_scan(bus, &msg->data[0], &msg->data_len);

	msg->completion_code = CC_SUCCESS;
	return;
}

void pal_OEM_1S_GET_FW_VERSION(ipmi_msg *msg)
{
	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t component = 0;

	component = msg->data[0];
	switch (component) {
	case CPNT_CPLD:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	case CPNT_BIC:
		msg->data[0] = BIC_FW_YEAR_MSB;
		msg->data[1] = BIC_FW_YEAR_LSB;
		msg->data[2] = BIC_FW_WEEK;
		msg->data[3] = BIC_FW_VER;
		msg->data[4] = BIC_FW_platform_0;
		msg->data[5] = BIC_FW_platform_1;
		msg->data[6] = BIC_FW_platform_2;
		msg->data_len = 7;
		msg->completion_code = CC_SUCCESS;
		break;
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
	}
}

void pal_OEM_1S_12V_CYCLE_SLOT(ipmi_msg *msg)
{
	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t retry = 3, isolator_num;
	I2C_MSG i2c_msg;

	i2c_msg.bus = CPLD_IO_I2C_BUS;
	i2c_msg.slave_addr = CPLD_IO_I2C_ADDR;
	i2c_msg.tx_len = 2;

	if (msg->InF_source == SLOT1_BIC_IFs) {
		isolator_num = FM_BIC_SLOT1_ISOLATED_EN_R;
		i2c_msg.data[0] = CPLD_IO_REG_OFS_HSC_EN_SLOT1; // offset

	} else if (msg->InF_source == SLOT3_BIC_IFs) {
		isolator_num = FM_BIC_SLOT3_ISOLATED_EN_R;
		i2c_msg.data[0] = CPLD_IO_REG_OFS_HSC_EN_SLOT3; // offset
	}

	// Before slot 12V off, disable isolator
	i2c_msg.data[1] = 0x00;
	gpio_set(isolator_num, GPIO_LOW);
	if (i2c_master_write(&i2c_msg, retry) < 0) {
		printk("slot off fail\n");
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	k_msleep(2000);

	// After slot 12V on, enable isolator
	i2c_msg.data[1] = 0x01;
	if (i2c_master_write(&i2c_msg, retry) < 0) {
		printk("slot on fail\n");
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}
	gpio_set(isolator_num, GPIO_HIGH);

	msg->completion_code = CC_SUCCESS;
	return;
}
