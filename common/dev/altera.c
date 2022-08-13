#include "altera.h"
#include "hal_i2c.h"
#include "util_spi.h"
#include <logging/log.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <zephyr.h>

#define MAX_RETRY 3
#define CHECK_ALTERA_STATUS_DELAY_US 100

LOG_MODULE_REGISTER(ALTERA_LOG);

// Change bit 1010 1010b(0xaa) and 0101 0101b (0x55)
// e.g. 0123 4567 -> 1032 5476
// Change bit 1100 1100b(0xcc) and 0011 0011b(0x33)
// e.g. 1032 5476 -> 3210 7654
// Change bit 1111 0000b(0xf0) and 0000 1111b(0x0f)
// e.g. 3210 7654 -> 7654 3210
#define SWAP_LSB_TO_MSB(x)                                                                         \
	x = (((x & 0xaa) >> 1) | ((x & 0x55) << 1));                                               \
	x = (((x & 0xcc) >> 2) | ((x & 0x33) << 2));                                               \
	x = (((x & 0xf0) >> 4) | ((x & 0x0f) << 4));

static altera_max10_attr altera_max10_config;

__weak int pal_load_altera_max10_attr(altera_max10_attr *altera_max10_config)
{
	return -1;
}

int change_word_to_byte(uint8_t *output, int intput)
{
	if (output == NULL) {
		LOG_WRN("Output passed in as NULL.\n");
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
		LOG_WRN("val passed in as NULL.\n");
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
	i2c_msg.target_addr = altera_max10_config.target_addr;
	memcpy(&i2c_msg.data[0], &data[0], data_len);
	i2c_msg.tx_len = data_len;
	i2c_msg.rx_len = res_len;

	ret = i2c_master_read(&i2c_msg, MAX_RETRY);
	if (ret != 0) {
		LOG_ERR("Read register fails after retry %d times. ret=%d \n", MAX_RETRY, ret);
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
		LOG_ERR("%s() Cannot read 0x%x data %d\n", __func__, address, data);
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
	i2c_msg.target_addr = altera_max10_config.target_addr;
	memcpy(&i2c_msg.data[0], &data[0], data_len);
	i2c_msg.tx_len = data_len;
	i2c_msg.rx_len = 0x0;

	ret = i2c_master_write(&i2c_msg, MAX_RETRY);
	if (ret != 0) {
		LOG_ERR("write register fails after retry %d times. ret=%d \n", MAX_RETRY, ret);
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
		LOG_WRN("msg passed in as NULL.\n");
		return FWUPDATE_UPDATE_FAIL;
	}

	if (offset == 0) {
		ret = pal_load_altera_max10_attr(&altera_max10_config);
		if (ret < 0) {
			LOG_ERR("Failed to load max10 attribute.");
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
			SWAP_LSB_TO_MSB(receive_buffer[byte]);
		}

		// Combine 4 bytes to 1 word before write operation
		data = (receive_buffer[3] << 24) | (receive_buffer[2] << 16) |
		       (receive_buffer[1] << 8) | (receive_buffer[0]);
		ret = max10_write_flash_data(addr, data);
		if (ret != 0) {
			LOG_ERR("[CPLD] write flash data failed\n");
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
				LOG_DBG("status: %x retry...\n", status);
				k_usleep(CHECK_ALTERA_STATUS_DELAY_US);
				retry--;
			}

		} while ((is_done == false) && (retry > 0));

		if (retry == 0) {
			LOG_ERR("Attempted %d retries, giving up!\n", MAX_RETRY);
			return FWUPDATE_UPDATE_FAIL;
		}

		buffer_offset += 4;
	}

	return FWUPDATE_SUCCESS;
}
