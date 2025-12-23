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

#include <stdlib.h>
#include <shell/shell.h>
#include "plat_adc.h"

static int cmd_adc_debug(const struct shell *shell, size_t argc, char **argv)
{
	if (argc < 4) {
		shell_print(
			shell,
			"Usage: adc_debug [0:read|1:write] [0:ads7066|1:ad4058] [0:meadha0|1:meadha1] [reg] [value]");
		return 0;
	}

	uint8_t w_r = strtoul(argv[1], NULL, 10); // 0: read, 1: write
	uint8_t adc_type = strtoul(argv[2], NULL, 10); // 0: ads7066, 1: ad4058
	uint8_t meadha_id = strtoul(argv[3], NULL, 16); // 0: meadha0, 1: meadha1
	uint8_t reg = strtoul(argv[4], NULL, 16);
	uint8_t write_value = strtoul(argv[5], NULL, 16);
	// follow shell input
	if (adc_type == 0) {
		if (w_r == 0) {
			uint8_t value = 0;
			int ret = ads7066_read_reg(reg, meadha_id, &value);
			if (ret < 0) {
				shell_error(shell, "read reg fail (err=%d)", ret);
				return -EINVAL;
			}
		} else if (w_r == 1) {
			int ret = ads7066_write_reg(reg, write_value, meadha_id);
			if (ret < 0) {
				shell_error(shell, "write reg fail (err=%d)", ret);
				return -EINVAL;
			}
		}
	} else if (adc_type == 1) {
		if (w_r == 0) {
			uint8_t value = 0;
			int ret = ad4058_read_reg(reg, meadha_id, &value);
			if (ret < 0) {
				shell_error(shell, "read reg fail (err=%d)", ret);
				return -EINVAL;
			}
		} else if (w_r == 1) {
			int ret = ad4058_write_reg(reg, write_value, meadha_id);
			if (ret < 0) {
				shell_error(shell, "write reg fail (err=%d)", ret);
				return -EINVAL;
			}
		}
	}

	else {
		shell_error(shell, "adc type error");
		return -EINVAL;
	}
	return 0;
}

SHELL_CMD_REGISTER(
	adc_debug, NULL,
	"ADC debug tool\n"
	"Usage: adc_debug [0:read|1:write] [0:ads7066|1:ad4058] [0:meadha0|1:meadha1] [reg] [value]",
	cmd_adc_debug);
