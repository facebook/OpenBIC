#ifndef FLASH_SHELL_H
#define FLASH_SHELL_H

#include <stdlib.h>
#include <shell/shell.h>
#include <drivers/spi_nor.h>
#include <drivers/flash.h>

#define SPI_DEVICE_PREFIX "spi"
#define MAX_SFDP_BUFF_SIZE 256

void cmd_flash_re_init(const struct shell *shell, size_t argc, char **argv);
void cmd_flash_sfdp_read(const struct shell *shell, size_t argc, char **argv);
void device_spi_name_get(size_t idx, struct shell_static_entry *entry);

SHELL_DYNAMIC_CMD_CREATE(spi_device_name, device_spi_name_get);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_flash_cmds,
			       SHELL_CMD(re_init, &spi_device_name, "Re-init spi config",
					 cmd_flash_re_init),
			       SHELL_CMD(sfpd_read, &spi_device_name, "SFPD read",
					 cmd_flash_sfdp_read),
			       SHELL_SUBCMD_SET_END);

#endif
