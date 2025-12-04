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
#include <string.h>
#include <logging/log.h>
#include "plat_i2c.h"
#include "plat_cpld.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(vqps, LOG_LEVEL_DBG);

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#define VQPS_STATUS_CPLD_OFFSET 0x13
#define EVB_VQPS_CPLD_OFFSET 0xA7

#define MEDHA0_VQPS_TOP_EN_BIT 4
#define MEDHA1_VQPS_TOP_EN_BIT 3
#define MEDHA0_VQPS_U_EN_BIT 2
#define MEDHA1_VQPS_U_EN_BIT 1
#define HAMSA_VQPS_EFUSE_USER_BIT 0

/* Board-dependent VQPS enable bits (EVB / RAINBOW) */
#define MEDHA0_VQPS_W_EN_BIT 3
#define MEDHA1_VQPS_W_EN_BIT 2
#define MEDHA0_VQPS_E_EN_BIT 1
#define MEDHA1_VQPS_E_EN_BIT 0

#define VQPS_SET_BIT(orig, bit) ((uint8_t)((orig) | (1u << (bit))))
#define VQPS_CLR_BIT(orig, bit) ((uint8_t)((orig) & ~(1u << (bit))))

static const cpld_pin_map_t vqps_list[] = {
	{ "MEDHA0_VQPS_TOP_EN", MEDHA0_VQPS_TOP_EN_BIT, VQPS_STATUS_CPLD_OFFSET },
	{ "MEDHA1_VQPS_TOP_EN", MEDHA1_VQPS_TOP_EN_BIT, VQPS_STATUS_CPLD_OFFSET },
	{ "MEDHA0_VQPS_U_EN", MEDHA0_VQPS_U_EN_BIT, VQPS_STATUS_CPLD_OFFSET },
	{ "MEDHA1_VQPS_U_EN", MEDHA1_VQPS_U_EN_BIT, VQPS_STATUS_CPLD_OFFSET },
	{ "HAMSA_VQPS_EFUSE_USER", HAMSA_VQPS_EFUSE_USER_BIT, VQPS_STATUS_CPLD_OFFSET },
	{ "MEDHA0_VQPS_W_EN", MEDHA0_VQPS_W_EN_BIT, 0 },
	{ "MEDHA1_VQPS_W_EN", MEDHA1_VQPS_W_EN_BIT, 0 },
	{ "MEDHA0_VQPS_E_EN", MEDHA0_VQPS_E_EN_BIT, 0 },
	{ "MEDHA1_VQPS_E_EN", MEDHA1_VQPS_E_EN_BIT, 0 },
};

static const cpld_pin_map_t *get_vqps_item(const char *name)
{
	for (int i = 0; i < ARRAY_SIZE(vqps_list); i++) {
		if (!strcmp(name, vqps_list[i].name)) {
			return &vqps_list[i];
		}
	}
	return NULL;
}

static int cmd_vqps_get(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 1) {
		shell_print(shell, "Usage:");
		shell_print(shell, "  vqps get all");
		LOG_DBG("invalid argc in get: %d", (int)argc);
		return -1;
	}

	uint8_t board_id = get_asic_board_id();
	uint8_t reg_status = 0;
	uint8_t reg_en = 0;

	if (!plat_read_cpld(VQPS_STATUS_CPLD_OFFSET, &reg_status, 1)) {
		LOG_DBG("plat_read_cpld failed: offset=0x%02x", VQPS_STATUS_CPLD_OFFSET);
		shell_error(shell, "read VQPS status CPLD failed");
		return -1;
	}

	shell_print(shell, "VQPS status raw [0x%02X] = 0x%02X", VQPS_STATUS_CPLD_OFFSET,
		    reg_status);

	if (board_id == ASIC_BOARD_ID_EVB) {
		if (!plat_read_cpld(EVB_VQPS_CPLD_OFFSET, &reg_en, 1)) {
			LOG_DBG("plat_read_cpld failed: offset=0x%02x", EVB_VQPS_CPLD_OFFSET);
			shell_error(shell, "read VQPS platform CPLD failed");
			return -1;
		}
		shell_print(shell, "VQPS platform raw [0x%02X] = 0x%02X", EVB_VQPS_CPLD_OFFSET,
			    reg_en);
	} else {
		shell_print(shell, "VQPS platform register not supported on this board (0x%02X)",
			    board_id);
	}

	for (int i = 0; i < 5; i++) {
		const cpld_pin_map_t *item = &vqps_list[i];
		uint8_t bit_val = (reg_status >> item->bit) & 0x1;
		shell_print(shell, "%s : %d", item->name, bit_val);
	}

	if (board_id == ASIC_BOARD_ID_EVB) {
		for (int i = 5; i < ARRAY_SIZE(vqps_list); i++) {
			const cpld_pin_map_t *item = &vqps_list[i];
			uint8_t bit_val = (reg_en >> item->bit) & 0x1;
			shell_print(shell, "%s : %d", item->name, bit_val);
		}
	}

	return 0;
}

static int cmd_vqps_set(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_print(shell, "Usage: vqps set %s <0|1>", argv[0]);
		return -1;
	}

	const char *name = argv[1];
	long set_val = strtol(argv[2], NULL, 10);

	if (set_val != 0 && set_val != 1) {
		shell_error(shell, "Value must be 0 or 1");
		LOG_DBG("invalid value in set: %ld", set_val);
		return -1;
	}

	const cpld_pin_map_t *item = get_vqps_item(name);
	if (!item) {
		shell_error(shell, "Unknown name: %s", name);
		LOG_DBG("unknown name in set: %s", name);
		return -1;
	}

	uint8_t board_id = get_asic_board_id();
	ptrdiff_t idx = item - vqps_list;
	uint8_t offset = 0;

	if (idx < 5) {
		/* Status bits always map to 0x13 */
		offset = VQPS_STATUS_CPLD_OFFSET;
	} else {
		/* Enable bits only supported on EVB */
		if (board_id != ASIC_BOARD_ID_EVB) {
			shell_error(shell, "%s is not supported on this board (0x%02X)", name,
				    board_id);
			return -1;
		}
		offset = EVB_VQPS_CPLD_OFFSET;
	}

	uint8_t reg_val = 0;

	if (!plat_read_cpld(offset, &reg_val, 1)) {
		LOG_DBG("plat_read_cpld failed: offset=0x%02x", offset);
		shell_error(shell, "read CPLD failed");
		return -1;
	}

	if (set_val == 1) {
		reg_val = VQPS_SET_BIT(reg_val, item->bit);
	} else {
		reg_val = VQPS_CLR_BIT(reg_val, item->bit);
	}

	if (!plat_write_cpld(offset, &reg_val)) {
		LOG_DBG("plat_write_cpld failed: offset=0x%02x val=0x%02x", offset, reg_val);
		shell_error(shell, "write CPLD failed");
		return -1;
	}

	shell_print(shell, "set %s to %ld done", name, set_val);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(get_subcmds, SHELL_CMD(all, NULL, "vqps get all", cmd_vqps_get),
			       SHELL_SUBCMD_SET_END);

static void vqps_dynamic_get_name(size_t idx, struct shell_static_entry *entry)
{
	uint8_t board_id = get_asic_board_id();

	size_t max_cnt = ARRAY_SIZE(vqps_list);
	if (board_id != ASIC_BOARD_ID_EVB) {
		max_cnt = 5;
	}

	if (idx >= max_cnt) {
		entry->syntax = NULL;
		entry->handler = NULL;
		entry->subcmd = NULL;
		entry->help = NULL;
		return;
	}

	entry->syntax = vqps_list[idx].name;
	entry->handler = NULL;
	entry->subcmd = NULL;
	entry->help = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(set_dynamic, vqps_dynamic_get_name);

SHELL_STATIC_SUBCMD_SET_CREATE(vqps_subcmds, SHELL_CMD(get, &get_subcmds, "vqps get", NULL),
			       SHELL_CMD(set, &set_dynamic, "vqps set <NAME> <0|1>", cmd_vqps_set),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(vqps, &vqps_subcmds, "VQPS control via CPLD", NULL);
