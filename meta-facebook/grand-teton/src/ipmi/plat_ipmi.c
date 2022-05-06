#include "plat_ipmi.h"

#include <stdio.h>
#include <stdlib.h>

#include "libutil.h"
#include "ipmi.h"
#include "plat_ipmb.h"
#include "plat_gpio.h"
#include "plat_fru.h"
#include "hal_jtag.h"
#include "eeprom.h"
#include "fru.h"
#include "sdr.h"
#include "app_handler.h"
#include "util_spi.h"
#include <drivers/spi_nor.h>
#include <drivers/flash.h>

static bool spi_isInitialized = false;

bool add_sel_evt_record(addsel_msg_t *sel_msg)
{
	ipmb_error status;
	ipmi_msg *msg;
	uint8_t system_event_record = 0x02; // IPMI spec definition
	uint8_t evt_msg_version = 0x04; // IPMI spec definition
	static uint16_t record_id = 0x1;

	// According to IPMI spec, record id 0h and FFFFh is reserved for special usage
	if ((record_id == 0) || (record_id == 0xFFFF)) {
		record_id = 0x1;
	}

	msg = (ipmi_msg *)malloc(sizeof(ipmi_msg));
	if (msg == NULL) {
		printf("add_sel_evt_record malloc fail\n");
		return false;
	}
	memset(msg, 0, sizeof(ipmi_msg));

	msg->data_len = 16;
	msg->InF_source = SELF;
	msg->InF_target = BMC_IPMB;
	msg->netfn = NETFN_STORAGE_REQ;
	msg->cmd = CMD_STORAGE_ADD_SEL;

	msg->data[0] = (record_id & 0xFF); // record id byte 0, lsb
	msg->data[1] = ((record_id >> 8) & 0xFF); // record id byte 1
	msg->data[2] = system_event_record; // record type
	msg->data[3] = 0x00; // timestamp, bmc would fill up for bic
	msg->data[4] = 0x00; // timestamp, bmc would fill up for bic
	msg->data[5] = 0x00; // timestamp, bmc would fill up for bic
	msg->data[6] = 0x00; // timestamp, bmc would fill up for bic
	msg->data[7] = (SELF_I2C_ADDRESS << 1); // generator id
	msg->data[8] = 0x00; // generator id
	msg->data[9] = evt_msg_version; // event message format version
	msg->data[10] = sel_msg->sensor_type; // sensor type, TBD
	msg->data[11] = sel_msg->sensor_number; // sensor number
	msg->data[12] = sel_msg->event_type; // event dir/event type
	msg->data[13] = sel_msg->event_data1; // sensor data 1
	msg->data[14] = sel_msg->event_data2; // sensor data 2
	msg->data[15] = sel_msg->event_data3; // sensor data 3
	record_id++;

	status = ipmb_read(msg, IPMB_inf_index_map[msg->InF_target]);
	SAFE_FREE(msg);
	if (status == IPMB_ERROR_FAILURE) {
		printf("Fail to post msg to txqueue for addsel\n");
		return false;
	} else if (status == IPMB_ERROR_GET_MESSAGE_QUEUE) {
		printf("No response from bmc for addsel\n");
		return false;
	}

	return true;
}
void OEM_1S_PEX_FLASH_READ(ipmi_msg *msg)
{
	if (!msg) {
		printf("<error> OEM_1S_PEX_FLASH_READ: parameter msg is NULL\n");
		return;
	}

	if (msg->data_len != 4) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	if (msg->data[0] > 3 || msg->data[0] < 0) {
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		return;
	}

	static const uint8_t flash_sel_pin[4] = { BIC_SEL_FLASH_SW0, BIC_SEL_FLASH_SW1,
						  BIC_SEL_FLASH_SW2, BIC_SEL_FLASH_SW3 };
	uint8_t read_len = msg->data[3];
	uint32_t addr = msg->data[1] | (msg->data[2] << 8);
	const struct device *flash_dev;

	if (read_len > 64) {
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		return;
	}
	/* pull high select pin to active flash and other should keep low*/
	for (int i = 0; i < ARRAY_SIZE(flash_sel_pin); i++) {
		if (msg->data[0] == i)
			gpio_set(flash_sel_pin[i], GPIO_HIGH);
		else
			gpio_set(flash_sel_pin[i], GPIO_LOW);
	}

	flash_dev = device_get_binding("spi1_cs0");
	if (!flash_dev) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}
	if (!spi_isInitialized) {
		/* Due to the SPI in this project has mux so call this function to re-init*/
		if (spi_nor_re_init(flash_dev)) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}
		spi_isInitialized = true;
	}

	if (flash_read(flash_dev, addr, &msg->data[0], read_len) != 0) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	msg->data_len = read_len;
	msg->completion_code = CC_SUCCESS;

	return;
}

void OEM_1S_GET_FPGA_USER_CODE(ipmi_msg *msg)
{
	if (!msg) {
		printf("<error> OEM_1S_GET_FPGA_USER_CODE: parameter msg is NULL\n");
		return;
	}

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t buffer[4] = { 0 };
	uint8_t ir_value = 0xc0;
	uint8_t dr_value = 0x00;
	const struct device *jtag_dev;

	jtag_dev = device_get_binding("JTAG0");

	if (!jtag_dev) {
		printf("JTAG device not found\n");
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	gpio_set(JTAG_BIC_EN, GPIO_HIGH);

	if (jtag_tap_set(jtag_dev, TAP_RESET)) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	k_msleep(10);

	if (jtag_ir_scan(jtag_dev, 8, &ir_value, buffer, TAP_IDLE)) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	if (jtag_dr_scan(jtag_dev, 32, &dr_value, buffer, TAP_IDLE)) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	gpio_set(JTAG_BIC_EN, GPIO_LOW);

	memcpy(msg->data, buffer, 4);
	msg->data_len = 4;
	msg->completion_code = CC_SUCCESS;
}

void APP_GET_SELFTEST_RESULTS(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	EEPROM_ENTRY fru_entry;
	fru_entry.config.dev_id = 0;
	fru_entry.offset = 0;
	fru_entry.data_len = 8;
	fru_entry.config.have_mux = true;
	fru_entry.config.mux_addr = SWB_FRU_MUX_ADDR;
	fru_entry.config.mux_channel = SWB_FRU_MUX_CHAN;
	FRU_read(&fru_entry);
	uint8_t checksum = 0;
	for (uint8_t i = 0; i < fru_entry.data_len; i++) {
		checksum += fru_entry.data[i];
	}

	SELF_TEST_RESULT res;

	res.result.opFwCorrupt = 0;
	res.result.updateFwCorrupt = 0;
	res.result.sdrRepoEmpty = is_sdr_not_init;
	res.result.ipmbLinesDead = 0;

	if (checksum == 0) {
		res.result.cannotAccessBmcFruDev = 0;
		res.result.internalCorrupt = 0;
	} else {
		res.result.cannotAccessBmcFruDev = 1;
		res.result.internalCorrupt = 1;
	}

	res.result.cannotAccessSdrRepo = is_sdr_not_init;
	res.result.cannotAccessSelDev = 0;

	memcpy(&msg->data[1], &res.result, 1);
	// 55h = No error, 57h = Corrupted or inaccessible data or devices
	msg->data[0] = (msg->data[1] == 0x00) ? 0x55 : 0x57;
	msg->data_len = 2;
	msg->completion_code = CC_SUCCESS;

	return;
}