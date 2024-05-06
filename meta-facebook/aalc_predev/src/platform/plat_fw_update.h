#include <stdint.h>
#include "modbus_server.h"

uint8_t modbus_get_fw_reversion(modbus_command_mapping *cmd);
uint8_t modbus_fw_download(modbus_command_mapping *cmd);
uint8_t all_fan_full_duty();