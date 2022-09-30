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

#include "power_shell.h"
#include "power_status.h"
#include "util_sys.h"
#include "hal_gpio.h"
#include <stdio.h>
#include <zephyr.h>
#include <string.h>

static struct device_arr device_info[MAX_DEVICE_CNT] = {
	[DEVICE_HOST] = { ENABLE, "HOST" },
	[DEVICE_BMC] = { DISABLE, "BMC" },
};

int check_pwr_ctl_valid(const struct shell *shell, uint8_t ctl_opt, uint8_t pwr_state)
{
	if (pwr_state == POWER_ON && ctl_opt == POWER_CTL_ON) {
		shell_warn(shell, "Power is already on");
		return 1;
	} else if (pwr_state == POWER_OFF && ctl_opt == POWER_CTL_OFF) {
		shell_warn(shell, "Power is already off");
		return 1;
	} else if (pwr_state == POWER_OFF && ctl_opt == POWER_CTL_RESET) {
		shell_warn(shell, "Power is off, can't reset");
		return 1;
	}
	return 0;
}

void cmd_power_status(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 1) {
		shell_warn(shell, "Help: platform power status");
		return;
	}

	shell_print(shell, "------------------------------------");
	for (int i = 0; i < MAX_DEVICE_CNT; i++) {
		if (device_info[i].enable == DISABLE) {
			shell_print(shell, "[%s]", device_info[i].name);
			shell_print(shell, "* status: non-support", device_info[i].name);
			continue;
		}
		switch (i) {
		case DEVICE_HOST:
			shell_print(shell, "[%s]", device_info[i].name);
			shell_print(shell, "* status:      %s", get_DC_status() ? "on" : "off");
			shell_print(shell, "* cpu status:  %s", CPU_power_good() ? "on" : "off");
			shell_print(shell, "* post status: %s", get_post_status() ? "on" : "off");
			break;

		case DEVICE_BMC:
			/* TODO */
			break;

		default:
			break;
		}
	}
	shell_print(shell, "------------------------------------");
}

void cmd_power_control(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_warn(shell, "Help: platform power control <device> <status>");
		return;
	}

	int dev_idx = -1;
	if (!strcmp(argv[1], HOST_KEYWORD)) {
		dev_idx = DEVICE_HOST;
	} else if (!strcmp(argv[1], BMC_KEYWORD)) {
		dev_idx = DEVICE_BMC;
	} else {
		shell_warn(shell, "Uknown device type %s", argv[1]);
		return;
	}

	if (device_info[dev_idx].enable == DISABLE) {
		shell_warn(shell, "%s power control is not support", device_info[dev_idx].name);
		return;
	}

	int control_opt = -1;
	if (!strcmp(argv[2], POWER_ON_KEYWORD)) {
		control_opt = POWER_CTL_ON;
	} else if (!strcmp(argv[2], POWER_OFF_KEYWORD)) {
		control_opt = POWER_CTL_OFF;
	} else if (!strcmp(argv[2], POWER_RESET_KEYWORD)) {
		control_opt = POWER_CTL_RESET;
	} else {
		shell_warn(shell, "Uknown status control type %s", argv[2]);
		return;
	}

	switch (dev_idx) {
	case DEVICE_HOST:
		if (control_opt != POWER_CTL_ON && control_opt != POWER_CTL_OFF &&
		    control_opt != POWER_CTL_RESET) {
			shell_warn(shell, "Selected device not support %s", argv[2]);
			return;
		}

		if (check_pwr_ctl_valid(shell, control_opt, get_DC_status()))
			return;

		int ret = pal_host_power_control(control_opt);
		if (ret == 0)
			shell_print(shell, "%s power %s successed!", device_info[DEVICE_HOST].name,
				    argv[2]);
		else if (ret == -1)
			shell_error(shell, "%s power %s not support in this project!",
				    device_info[DEVICE_HOST].name, argv[2]);
		else
			shell_error(shell, "%s power %s failed!", device_info[DEVICE_HOST].name,
				    argv[2]);
		break;

	case DEVICE_BMC:
		/* TODO */
		break;

	default:
		break;
	}
}
