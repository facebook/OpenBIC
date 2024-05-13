#include <string.h>
#include <stdlib.h>
#include "plat_version.h"
#include "plat_modbus.h"
#include "plat_util.h"
#include "modbus_server.h"
#include <logging/log.h>
#include "util_spi.h"
#include "libutil.h"

LOG_MODULE_REGISTER(plat_fw_update);

#define UPADTE_FW_DATA_LENGTH_MIN 3 // contain 2 regs(offeset)+ 1 reg(length) at least

uint8_t modbus_get_fw_reversion(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	uint16_t byte_val[4] = { BIC_FW_YEAR_MSB_ASCII, BIC_FW_YEAR_LSB_ASCII, BIC_FW_WEEK_ASCII,
				 BIC_FW_VER_ASCII };
	memcpy(cmd->data, &byte_val[0], sizeof(uint16_t) * cmd->cmd_size);

	regs_reverse(cmd->data_len, cmd->data);

	return MODBUS_EXC_NONE;
}

uint8_t modbus_fw_download(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	uint32_t offset = cmd->data[0] << 16 | cmd->data[1]; // offset
	uint16_t msg_len = cmd->data[2] & 0x7FFF; // length
	uint8_t flag = (cmd->data[2] & BIT(15)) ? (SECTOR_END_FLAG | NO_RESET_FLAG) : 0;

	if (cmd->data_len <= UPADTE_FW_DATA_LENGTH_MIN)
		return MODBUS_EXC_ILLEGAL_DATA_VAL;

	if (msg_len != ((cmd->data_len - UPADTE_FW_DATA_LENGTH_MIN) * 2))
		return MODBUS_EXC_ILLEGAL_DATA_VAL;

	regs_reverse(cmd->data_len, cmd->data);

	return fw_update(offset, msg_len, (uint8_t *)&cmd->data[UPADTE_FW_DATA_LENGTH_MIN], flag,
			 DEVSPI_FMC_CS0);
}