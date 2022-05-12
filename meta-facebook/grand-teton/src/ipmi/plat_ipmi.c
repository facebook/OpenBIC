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

void OEM_1S_FW_UPDATE(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	/*********************************
	* Request Data
	*
	* Byte   0: [6:0] fw update target, [7] indicate last packet
	* Byte 1-4: offset, lsb first
	* Byte 5-6: length, lsb first
	* Byte 7-N: data
	***********************************/
	if (msg->data_len < 8) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t target = msg->data[0];
	uint8_t status = -1;
	uint32_t offset =
		((msg->data[4] << 24) | (msg->data[3] << 16) | (msg->data[2] << 8) | msg->data[1]);
	uint16_t length = ((msg->data[6] << 8) | msg->data[5]);

	if ((length == 0) || (length != msg->data_len - 7)) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	if ((target == SWB_BIC_UPDATE) || (target == (SWB_BIC_UPDATE | IS_SECTOR_END_MASK))) {
		// Expect BIC firmware size not bigger than 320k
		if (offset > BIC_UPDATE_MAX_OFFSET) {
			msg->completion_code = CC_PARAM_OUT_OF_RANGE;
			return;
		}
		status = fw_update(offset, length, &msg->data[7], (target & IS_SECTOR_END_MASK),
				   DEVSPI_FMC_CS0);
	} else if (((target & WITHOUT_SENCTOR_END_MASK) == PEX0_UPDATE) ||
		   ((target & WITHOUT_SENCTOR_END_MASK) == PEX1_UPDATE) ||
		   ((target & WITHOUT_SENCTOR_END_MASK) == PEX2_UPDATE) ||
		   ((target & WITHOUT_SENCTOR_END_MASK) == PEX3_UPDATE)) {
		uint8_t flash_sel_pin[4] = { BIC_SEL_FLASH_SW0, BIC_SEL_FLASH_SW1,
					     BIC_SEL_FLASH_SW2, BIC_SEL_FLASH_SW3 };
		uint8_t flash_sel_base = BIC_SEL_FLASH_SW0 - PEX0_UPDATE;
		uint8_t current_sel_pin = 0xFF;

		/* change mux to pex flash */
		for (int i = 0; i < ARRAY_SIZE(flash_sel_pin); i++) {
			if (flash_sel_base + (target & WITHOUT_SENCTOR_END_MASK) ==
			    flash_sel_pin[i]) {
				current_sel_pin = flash_sel_pin[i];
				gpio_set(current_sel_pin, GPIO_HIGH);
			} else {
				gpio_set(flash_sel_pin[i], GPIO_LOW);
			}
		}
		status = fw_update(offset, length, &msg->data[7], (target & IS_SECTOR_END_MASK),
				   DEVSPI_SPI1_CS0);

		if (current_sel_pin != 0xFF) {
			/* change mux to default */
			gpio_set(current_sel_pin, GPIO_LOW);
		}
	} else {
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	msg->data_len = 0;

	switch (status) {
	case FWUPDATE_SUCCESS:
		msg->completion_code = CC_SUCCESS;
		break;
	case FWUPDATE_OUT_OF_HEAP:
		msg->completion_code = CC_LENGTH_EXCEEDED;
		break;
	case FWUPDATE_OVER_LENGTH:
		msg->completion_code = CC_OUT_OF_SPACE;
		break;
	case FWUPDATE_REPEATED_UPDATED:
		msg->completion_code = CC_INVALID_DATA_FIELD;
		break;
	case FWUPDATE_UPDATE_FAIL:
		msg->completion_code = CC_TIMEOUT;
		break;
	case FWUPDATE_ERROR_OFFSET:
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		break;
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}
	if (status != FWUPDATE_SUCCESS) {
		printf("firmware (0x%02X) update failed cc: %x\n", target, msg->completion_code);
	}

	return;
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
	fru_entry.config.mux_present = true;
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