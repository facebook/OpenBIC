#include "libutil.h"
#include <stdlib.h>
#include <string.h>

/*
 * @brief Construct an IPMI message buffer.
 *
 * @param seq_source The I2C bus ID.
 * @param netFn The net function.
 * @param cmd The command code.
 * @param source_inft the source bridge interface.
 * @param target_inft the target bridge interface.
 * @param data_len the amount of valid bytes in #data buffer.
 * @param data the data buffer.
 *
 * @retval The IPMI message buffer.
 */
ipmi_msg construct_ipmi_message(uint8_t seq_source, uint8_t netFn, uint8_t command,
				uint8_t source_inft, uint8_t target_inft, uint16_t data_len,
				uint8_t *data)
{
	ipmi_msg ipmi_message;
	ipmi_message.seq_source = seq_source;
	ipmi_message.netfn = netFn;
	ipmi_message.cmd = command;
	ipmi_message.InF_source = source_inft;
	ipmi_message.InF_target = target_inft;
	ipmi_message.data_len = data_len;
	if ((data_len != 0) && (data != NULL)) {
		memcpy(ipmi_message.data, data, data_len);
	}
	return ipmi_message;
}

/*
 * @brief Construct an I2C message buffer.
 *
 * @param bus_id The I2C bus ID.
 * @param address Target device address(7-bit address).
 * @param tx_len the data length that transmits to target device.
 * @param data the data buffer that ready to transmit to target.
 * @param rx_len the data length that receive the response from target device.
 *
 * @retval The I2C message buffer.
 */
I2C_MSG construct_i2c_message(uint8_t bus_id, uint8_t address, uint8_t tx_len, uint8_t *data,
			      uint8_t rx_len)
{
	I2C_MSG i2c_msg;
	i2c_msg.bus = bus_id;
	i2c_msg.target_addr = address;
	i2c_msg.tx_len = tx_len;
	memcpy(i2c_msg.data, data, tx_len);
	i2c_msg.rx_len = rx_len;
	return i2c_msg;
}

void reverse_array(uint8_t arr[], uint8_t size)
{
	uint8_t i;
	for (i = 0; i < size / 2; i++) {
		uint8_t temp = arr[i];
		arr[i] = arr[size - 1 - i];
		arr[size - 1 - i] = temp;
	}
}
