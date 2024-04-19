#include <string.h>
#include <stdlib.h>
#include "plat_version.h"
#include "plat_modbus.h"
#include "modbus_server.h"
#include "util_spi.h"

#define UPADTE_FW_DATA_LENGTH_MIN 3 // contain 2 regs(offeset)+ 1 reg(length) at least

uint8_t modbus_get_fw_reversion(modbus_command_mapping *cmd)
{
	uint16_t byte_val = FIRMWARE_REVISION_1 << 8 | FIRMWARE_REVISION_2;
	memcpy(&cmd->data, &byte_val, sizeof(uint16_t) * cmd->size);
	return MODBUS_EXC_NONE;
}

uint8_t modbus_fw_download(modbus_command_mapping *cmd)
{
	uint32_t offset = cmd->data[0] << 16 | cmd->data[1]; // offset
	uint16_t msg_len = cmd->data[2] & 0x7FFF; // length
	uint8_t flag = (cmd->data[2] & (1 << 15)) ? (1 << 7) : 0;

	if (cmd->data_len < UPADTE_FW_DATA_LENGTH_MIN)
		return MODBUS_EXC_ILLEGAL_DATA_VAL;

	if (msg_len != ((cmd->data_len - UPADTE_FW_DATA_LENGTH_MIN) * 2))
		return MODBUS_EXC_ILLEGAL_DATA_VAL;

	return fw_update(offset, msg_len, (uint8_t *) &cmd->data[UPADTE_FW_DATA_LENGTH_MIN], flag, DEVSPI_FMC_CS0);
}