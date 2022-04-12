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
