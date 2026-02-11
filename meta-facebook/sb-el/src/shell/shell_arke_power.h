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
#ifndef PLAT_ARKE_POWER_H
#define PLAT_ARKE_POWER_H
typedef struct pwr_clock_compnt_mapping {
	uint8_t clock_name_index;
	uint8_t addr;
	uint8_t bus;
	uint8_t *clock_name;
} pwr_clock_compnt_mapping;

enum PWR_CLOCK_COMPONENT {
	CLK_BUF_100M_U85,
	CLK_BUF_100M_U690,
	CLK_BUF_100M_U88,
	CLK_GEN_100M_U86,
	CLK_COMPONENT_MAX
};

#define CLK_BUF_U85_ADDR (0xCE >> 1)
#define CLK_BUF_U690_ADDR (0xD8 >> 1)
#define CLK_BUF_U88_ADDR (0xDE >> 1)
#define CLK_GEN_100M_U86_ADDR 0x9
#define CLK_BUF_100M_WRITE_LOCK_CLEAR_LOS_EVENT_OFFSET 0x27
#define CLK_GEN_LOSMON_EVENT_OFFSET 0x5a
#define CLK_BUF_100M_BYTE_COUNT 0x7

void clear_clock_status(const struct shell *shell, uint8_t clock_index);
#endif
