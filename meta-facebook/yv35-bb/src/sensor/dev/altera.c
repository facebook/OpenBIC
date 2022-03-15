#include "fw_update.h"
#include "altera.h"

#define MAX_RETRY 3

void change_word_to_byte(uint8_t *output, int intput)
{
	output[0] = (intput >> 24) & 0xff;
	output[1] = (intput >> 16) & 0xff;
	output[2] = (intput >> 8) & 0xff;
	output[3] = (intput >> 0) & 0xff;
}

int get_register_via_i2c(int reg, int *val)
{
	int ret = 0;
	I2C_MSG i2c_msg;
	uint8_t data[4] = { 0 };
	uint8_t data_len = sizeof(data), res_len = 4;

	change_word_to_byte(&data[0], reg);

	i2c_msg.bus = CPLD_UPDATE_I2C_BUS;
	i2c_msg.slave_addr = CPLD_UPDATE_ADDR;
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

	change_word_to_byte(&data[0], reg);
	change_word_to_byte(&data[4], val);

	i2c_msg.bus = CPLD_UPDATE_I2C_BUS;
	i2c_msg.slave_addr = CPLD_UPDATE_ADDR;
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

void change_msg_to_buffer(uint8_t *buffer, int msg_len, uint8_t *msg)
{
	int i = 0;

	for (i = 0; i < msg_len; i++) {
		*buffer = msg[i];
		buffer++;
	}
}

int cpld_altera_fw_update(uint32_t offset, uint16_t msg_len, uint8_t *msg)
{
	uint32_t buffer_offset = 0;
	int addr = 0, byte = 0, data = 0;
	int ret = 0;
	int receive_buffer[4]; // for received data
	int retry = MAX_RETRY;
	uint8_t status = 0;
	bool is_done = false;

	buffer_offset = 0;
	for (addr = (CFM_START_ADDR + offset); addr < (CFM_START_ADDR + offset + msg_len);
	     addr += 4) {
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
			return fwupdate_update_fail;
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
			return fwupdate_update_fail;
		}

		buffer_offset += 4;
	}

	return fwupdate_success;
}
