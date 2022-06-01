#include <stdio.h>
#include "eeprom.h"
#include "hal_i2c.h"
#include <string.h>

bool eeprom_mux_check(EEPROM_ENTRY *entry)
{
	if (entry == NULL) {
		return false;
	}
	I2C_MSG msg;
	uint8_t retry = 5;
	if (entry->config.mux_present) {
		msg.bus = entry->config.port;
		msg.target_addr = entry->config.mux_addr;
		msg.tx_len = 1;
		msg.data[0] = (1 << (entry->config.mux_channel));
		return ((i2c_master_write(&msg, retry) == 0) ? true : false);
	} else {
		return true;
	}
}

bool eeprom_write(EEPROM_ENTRY *entry)
{
	if (entry == NULL) {
		return false;
	}
	I2C_MSG msg;
	uint8_t retry = 5;
	uint8_t i;

	for (i = 0; i < retry; i++) {
		/* Check if there have a MUX before EEPROM and access it to change channel first */
		if (eeprom_mux_check(entry) == false)
			continue;

		msg.bus = entry->config.port;
		msg.target_addr = entry->config.target_addr;
		msg.tx_len = entry->data_len + 2; // write 2 byte offset to EEPROM
		msg.data[0] =
			((entry->config.start_offset + entry->offset) >> 8) & 0xFF; // offset msb
		msg.data[1] = (entry->config.start_offset + entry->offset) & 0xFF; // offset lsb
		memcpy(&msg.data[2], &entry->data[0], entry->data_len);

		if (i2c_master_write(&msg, retry) == 0)
			break;
	}

	return ((i == retry) ? false : true);
}

bool eeprom_read(EEPROM_ENTRY *entry)
{
	if (entry == NULL) {
		return false;
	}

	I2C_MSG msg;
	uint8_t retry = 5;
	uint8_t i;

	for (i = 0; i < retry; i++) {
		/* Check if there have a MUX before EEPROM and access it to change channel first */
		if (eeprom_mux_check(entry) == false)
			continue;

		msg.bus = entry->config.port;
		msg.target_addr = entry->config.target_addr;
		msg.tx_len = 2; // write 2 byte offset to EEPROM
		msg.rx_len = entry->data_len;
		msg.data[0] =
			((entry->config.start_offset + entry->offset) >> 8) & 0xFF; // offset msb
		msg.data[1] = (entry->config.start_offset + entry->offset) & 0xFF; // offset lsb

		if (i2c_master_read(&msg, retry) == 0) {
			entry->data_len = msg.rx_len;
			memcpy(&entry->data, &msg.data, msg.rx_len);
			break;
		}
	}

	return ((i == retry) ? false : true);
}
