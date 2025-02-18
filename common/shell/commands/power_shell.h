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

#ifndef POWER_SHELL_H
#define POWER_SHELL_H

#include <stdlib.h>
#include <shell/shell.h>
#include <plat_def.h>

struct device_arr {
	uint8_t enable;
	char *name;
};

enum device_id {
	DEVICE_HOST,
	DEVICE_BMC,
	MAX_DEVICE_COUNT,
};

#ifdef SHELL_PWR_SEQ

#define MAX_PWR_ON_RECORD 10
#define MAX_PWR_OFF_RECORD 10

enum SHELL_POWER_ON_STAGE {
	SHELL_BOARD_POWER_ON_STAGE0 = 0x00,
	SHELL_BOARD_POWER_ON_STAGE1,
	SHELL_BOARD_POWER_ON_STAGE2,
	SHELL_RETIMER_POWER_ON_STAGE0,
	SHELL_RETIMER_POWER_ON_STAGE1,
	SHELL_RETIMER_POWER_ON_STAGE2,
	SHELL_E1S_POWER_ON_STAGE0,
	SHELL_E1S_POWER_ON_STAGE1,
	SHELL_E1S_POWER_ON_STAGE2,
	SHELL_E1S_POWER_ON_STAGE3,
	SHELL_POWER_ON_NONE = 0xFF,
};

enum SHELL_POWER_OFF_STAGE {
	SHELL_E1S_POWER_OFF_STAGE0 = 0x00,
	SHELL_E1S_POWER_OFF_STAGE1,
	SHELL_E1S_POWER_OFF_STAGE2,
	SHELL_E1S_POWER_OFF_STAGE3,
	SHELL_RETIMER_POWER_OFF_STAGE0,
	SHELL_RETIMER_POWER_OFF_STAGE1,
	SHELL_RETIMER_POWER_OFF_STAGE2,
	SHELL_BOARD_POWER_OFF_STAGE0,
	SHELL_BOARD_POWER_OFF_STAGE1,
	SHELL_BOARD_POWER_OFF_STAGE2,
	SHELL_POWER_OFF_NONE = 0xFF,
};

extern uint8_t power_on_stage[10];
extern uint8_t power_off_stage[10];

void cmd_power_sequence(const struct shell *shell, size_t argc, char **argv);
void power_on_sequence_check(const struct shell *shell, uint8_t stage);
void power_off_sequence_check(const struct shell *shell, uint8_t stage);

#endif

void cmd_power_status(const struct shell *shell, size_t argc, char **argv);
void cmd_power_control(const struct shell *shell, size_t argc, char **argv);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_power_cmds,
			       SHELL_CMD(status, NULL, "List power status", cmd_power_status),
			       SHELL_CMD(control, NULL, "Power control", cmd_power_control),
#ifdef SHELL_PWR_SEQ
			       SHELL_CMD(sequence, NULL, "Power Sequence status",
					 cmd_power_sequence),
#endif
			       SHELL_SUBCMD_SET_END);

#endif
