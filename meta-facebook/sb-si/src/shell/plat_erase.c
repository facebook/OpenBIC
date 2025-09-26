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

#include <shell/shell.h>
#include <stdlib.h>
#include <stdio.h>
#include <drivers/flash.h>
#include <sys/reboot.h>

void plat_cmd_erase(const struct shell *shell, size_t argc, char **argv)
{
	off_t offset = (off_t)0x00;
	size_t size = (size_t)0x1000;

	const struct device *dev;
	dev = device_get_binding("spi_spim0_cs0");
	if (!dev) {
		shell_warn(shell, "Fail to bind device spi_spim0_cs0.\n");
		return;
	}

	int ret;
	ret = flash_erase(dev, offset, size);
	if (ret) {
		shell_warn(shell, "flash_erase failed (err:%d).\n", ret);
		return;
	}
	sys_reboot(0);
}

SHELL_CMD_REGISTER(erase, NULL, "erase flash and reboot", plat_cmd_erase);
