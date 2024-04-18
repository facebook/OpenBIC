#include <string.h>
#include "plat_version.h"
#include "plat_modbus.h"
#include "modbus_server.h"
#include "util_spi.h"

uint8_t modbus_get_fw_reversion(modbus_command_mapping *cmd)
{
	uint16_t byte_val = FIRMWARE_REVISION_1 << 8 | FIRMWARE_REVISION_2;
	memcpy(&cmd->data, &byte_val, sizeof(uint16_t) * cmd->size);
	return MODBUS_EXC_NONE;
}

uint8_t modbus_fw_download(modbus_command_mapping *cmd)
{

    static uint32_t offset;
    static uint16_t msg_len; 
    static uint8_t *msg_buf= NULL; 
    static uint8_t flag;  

    if (cmd->offset < 3) {
        return MODBUS_EXC_ILLEGAL_DATA_VAL;
    } else if (cmd->offset >= 3) {
        if ((cmd->data[2] >> 8) == 0x10) //flag
            flag = SECTOR_END_FLAG;
        else if ((cmd->data[2] >> 8) > 0)
            return MODBUS_EXC_ILLEGAL_DATA_VAL;
            
        msg_len = cmd->data[2] & 0x00FF; // length
        offset = cmd->data[0] << 16 | cmd->data[1]; // offset
 
        if (msg_len != (cmd->offset) * 2 || (msg_len != 68 && msg_len != (cmd->size)* 2 && !flag)) 
            return MODBUS_EXC_ILLEGAL_DATA_VAL;
        else {
            memcpy(&msg_buf[(cmd->offset) * 2], &cmd->data, msg_len);
        }

    }

    return fw_update(offset, msg_len, msg_buf, flag, DEVSPI_FMC_CS0);
}