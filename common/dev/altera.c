#include <zephyr.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "altera.h"
#include "util_spi.h"
#include "hal_i2c.h"

#define MAX_RETRY 3

static altera_max10_attr altera_max10_config;

__weak int pal_load_altera_max10_attr(altera_max10_attr *altera_max10_config)
{
	return -1;
}

int change_word_to_byte(uint8_t *output, int intput)
{
	if (output == NULL) {
		return -1;
	}

	output[0] = (intput >> 24) & 0xff;
	output[1] = (intput >> 16) & 0xff;
	output[2] = (intput >> 8) & 0xff;
	output[3] = (intput >> 0) & 0xff;

	return 0;
}

int get_register_via_i2c(int reg, int *val)
{
	if (val == NULL) {
		return -1;
	}

	int ret = 0;
	I2C_MSG i2c_msg;
	uint8_t data[4] = { 0 };
	uint8_t data_len = sizeof(data), res_len = 4;

	ret = change_word_to_byte(&data[0], reg);
	if (ret < 0) {
		return ret;
	}

	i2c_msg.bus = altera_max10_config.bus;
	i2c_msg.target_addr = altera_max10_config.slave_addr;
	memcpy(&i2c_msg.data[0], &data[0], data_len);
	i2c_msg.tx_len = data_len;
	i2c_msg.rx_len = res_len;

	ret = i2c_master_read(&i2c_msg, MAX_RETRY);
	if (ret != 0) {
		printf("%s() read register fails after retry %d times. ret=%d \n", __func__,
		       MAX_RETRY, ret);
		return ret;
	}

	*val = (i2c_msg.data[0] | (i2c_msg.data[1] << 8) | (i2c_msg.data[2] << 16) |
		(i2c_msg.data[3] << 24));
	return ret;
}

int max10_reg_read(int address)
{
	int ret = 0, data = 0;

	ret = get_register_via_i2c(address, &data);
	if (ret != 0) {
		printf("%s() Cannot read 0x%x data %d\n", __func__, address, data);
	}

	return data;
}

int max10_status_read(void)
{
	return max10_reg_read(ON_CHIP_FLASH_IP_CSR_STATUS_REG);
}

int set_register_via_i2c(int reg, int val)
{
	int ret = 0;
	uint8_t data[8] = { 0 };
	uint8_t data_len = sizeof(data);
	I2C_MSG i2c_msg;

	ret = change_word_to_byte(&data[0], reg);
	if (ret < 0) {
		return ret;
	}

	ret = change_word_to_byte(&data[4], val);
	if (ret < 0) {
		return ret;
	}

	i2c_msg.bus = altera_max10_config.bus;
	i2c_msg.target_addr = altera_max10_config.slave_addr;
	memcpy(&i2c_msg.data[0], &data[0], data_len);
	i2c_msg.tx_len = data_len;
	i2c_msg.rx_len = 0x0;

	ret = i2c_master_write(&i2c_msg, MAX_RETRY);
	if (ret != 0) {
		printf("%s() write register fails after retry %d times. ret=%d \n", __func__,
		       MAX_RETRY, ret);
	}
	return ret;
}

int max10_reg_write(int address, int data)
{
	int ret = 0;
	ret = set_register_via_i2c(address, data);

	k_usleep(100);
	return ret;
}

int max10_write_flash_data(int address, int data)
{
	return max10_reg_write(ON_CHIP_FLASH_IP_DATA_REG + address, data);
}

int cpld_altera_max10_fw_update(uint32_t offset, uint16_t msg_len, uint8_t *msg)
{
	uint32_t buffer_offset = 0;
	int addr = 0, byte = 0, data = 0;
	int ret = 0;
	int receive_buffer[4]; // for received data
	int retry = MAX_RETRY;
	uint8_t status = 0;
	bool is_done = false;

	if (msg == NULL) {
		return FWUPDATE_UPDATE_FAIL;
	}

	if (offset == 0) {
		ret = pal_load_altera_max10_attr(&altera_max10_config);
		if (ret < 0) {
			return FWUPDATE_UPDATE_FAIL;
		}
	}

	buffer_offset = 0;
	for (addr = (altera_max10_config.update_start_addr + offset);
	     addr < (altera_max10_config.update_start_addr + offset + msg_len); addr += 4) {
		// Get 4 bytes
		receive_buffer[0] = msg[buffer_offset + 0];
		receive_buffer[1] = msg[buffer_offset + 1];
		receive_buffer[2] = msg[buffer_offset + 2];
		receive_buffer[3] = msg[buffer_offset + 3];

		// Swap LSB with MSB before write into CFM
		for (byte = 0; byte < 4; byte++) {
			receive_buffer[byte] = (((receive_buffer[byte] & 0xaa) >> 1) |
						((receive_buffer[byte] & 0x55) << 1));
			receive_buffer[byte] = (((receive_buffer[byte] & 0xcc) >> 2) |
						((receive_buffer[byte] & 0x33) << 2));
			receive_buffer[byte] = (((receive_buffer[byte] & 0xf0) >> 4) |
						((receive_buffer[byte] & 0x0f) << 4));
		}

		// Combine 4 bytes to 1 word before write operation
		data = (receive_buffer[3] << 24) | (receive_buffer[2] << 16) |
		       (receive_buffer[1] << 8) | (receive_buffer[0]);
		ret = max10_write_flash_data(addr, data);
		if (ret != 0) {
			printf("[CPLD] write flash data failed\n");
			return FWUPDATE_UPDATE_FAIL;
		}

		// Check write successful
		is_done = false;
		retry = MAX_RETRY;
		do {
			status = max10_status_read();
			status &= STATUS_BIT_MASK;

			if ((status & WRITE_SUCCESS) == WRITE_SUCCESS) {
				is_done = true;

			} else {
				printf("status: %x retry...\n", status);
				k_usleep(100);
				retry--;
			}

		} while ((is_done == false) && (retry > 0));

		if (retry == 0) {
			return FWUPDATE_UPDATE_FAIL;
		}

		buffer_offset += 4;
	}

	return FWUPDATE_SUCCESS;
}
