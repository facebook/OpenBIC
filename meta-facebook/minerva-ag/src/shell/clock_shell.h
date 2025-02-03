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

#ifndef CLOCK_SHELL_H
#define CLOCK_SHELL_H

#include <shell/shell.h>

enum CLOCK_COMPONENT {
	CLKGEN_312M,
	AEGIS_CLKBUF_100M_U471,
	AEGIS_CLKBUF_100M_U519,
	CLKGEN_100M,
	CLK_COMPONENT_MAX
};

typedef struct clock_compnt_mapping {
	uint8_t clock_name_index;
	uint8_t addr;
	uint8_t bus;
	uint8_t *clock_name;
} clock_compnt_mapping;

void cmd_set_clock(const struct shell *shell, size_t argc, char **argv);
void cmd_get_clock(const struct shell *shell, size_t argc, char **argv);
void cmd_get_clock_status(const struct shell *shell, size_t argc, char **argv);
void cmd_clear_clock_status(const struct shell *shell, size_t argc, char **argv);

#endif
