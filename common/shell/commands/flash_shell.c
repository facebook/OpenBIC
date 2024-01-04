/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "flash_shell.h"
#include <devicetree.h>
#include <device.h>
#include <stdio.h>
#include "libutil.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(flash_shell);

/*
    Command FLASH
*/
void cmd_flash_re_init(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_warn(shell, "Help: platform flash re_init <spi_device>");
		return;
	}

	const struct device *flash_dev;
	flash_dev = device_get_binding(argv[1]);

	if (!flash_dev) {
		shell_error(shell, "Can't find any binding device with label %s", argv[1]);
	}

	if (spi_nor_re_init(flash_dev)) {
		shell_error(shell, "%s re-init failed!", argv[1]);
		return;
	}

	shell_print(shell, "%s re-init success!", argv[1]);
	return;
}

void cmd_flash_sfdp_read(const struct shell *shell, size_t argc, char **argv)
{
	if (argc < 2 || argc > 4) {
		shell_warn(
			shell,
			"Help: platform flash sfdp_read <spi_device> <offset(optional, default 0h)> <read_bytes(optional, default_max 256)>");
		return;
	}

	int read_bytes = MAX_SFDP_BUFF_SIZE;
	int offset = 0;
	if (argc == 4) {
		offset = strtol(argv[2], NULL, 16);
		read_bytes = strtol(argv[3], NULL, 10);
		if (read_bytes > MAX_SFDP_BUFF_SIZE)
			read_bytes = MAX_SFDP_BUFF_SIZE;
	} else if (argc == 3) {
		offset = strtol(argv[2], NULL, 10);
	}

	const struct device *flash_dev;
	flash_dev = device_get_binding(argv[1]);

	if (!flash_dev) {
		shell_error(shell, "Can't find any binding device with label %s", argv[1]);
	}

	if (!device_is_ready(flash_dev)) {
		shell_error(shell, "%s: device not ready", flash_dev->name);
		return;
	}

	uint8_t *raw = malloc(read_bytes * sizeof(uint8_t));
	SHELL_CHECK_NULL_ARG(raw);
	memset(raw, 0, read_bytes);

	int ret = flash_sfdp_read(flash_dev, offset, raw, read_bytes);
	if (ret) {
		shell_error(shell, "Failed to read flash sfdp with ret: %d", ret);
		goto exit;
	}

	printf("sfdp raw read from ofst %xh with %d bytes:", offset, read_bytes);
	if ((offset % 4)) {
		printf("\n[%-3x] ", 0);
		for (int i = 0; i < (offset % 4); i++) {
			printf("   ");
		}
	}
	for (int i = 0; i < read_bytes; i++) {
		if (!((offset + i) % 4)) {
			printf("\n[%-3x] ", (offset + i));
		}
		printf("%.2x ", raw[i]);
	}
	printf("\n");

exit:
	SAFE_FREE(raw);
	return;
}

/* Flash sub command */
void device_spi_name_get(size_t idx, struct shell_static_entry *entry)
{
	const struct device *dev = shell_device_lookup(idx, SPI_DEVICE_PREFIX);

	if (entry == NULL) {
		LOG_ERR("entry is NULL");
		return;
	}

	entry->syntax = (dev != NULL) ? dev->name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}
