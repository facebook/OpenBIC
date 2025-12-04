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
#include "plat_ioexp.h"

LOG_MODULE_REGISTER(power_capping_control, LOG_LEVEL_DBG);

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#define POWER_CAPPING_LV1_CPLD_OFFSET 0xA6
#define POWER_CAPPING_LV2_LV3_CPLD_OFFSET 0x25
#define VR_HOT_EVB_IOEXP_OUTPUT_OFFSET TCA6424A_OUTPUT_PORT_0

/* Bit mapping */
#define MEDHA0_PWR_CAP_LV1_BIT 1
#define MEDHA1_PWR_CAP_LV1_BIT 0
#define MEDHA0_PWR_CAP_LV2_BIT 7
#define MEDHA1_PWR_CAP_LV2_BIT 6
#define MEDHA0_PWR_CAP_LV3_BIT 5
#define MEDHA1_PWR_CAP_LV3_BIT 4
#define VR_HOT_RAINBOW_BIT 0
#define VR_HOT_EVB_BIT  HAMSA_MFIO19

#define POWER_CAPPING_SET_BIT(orig, bit) ((uint8_t)((orig) | (1u << (bit))))
#define POWER_CAPPING_CLR_BIT(orig, bit) ((uint8_t)((orig) & ~(1u << (bit))))

static const cpld_pin_map_t pwr_cap_list[] = {
	{ "MEDHA0_PWR_CAP_LV1_CPLD", MEDHA0_PWR_CAP_LV1_BIT, POWER_CAPPING_LV1_CPLD_OFFSET },
	{ "MEDHA1_PWR_CAP_LV1_CPLD", MEDHA1_PWR_CAP_LV1_BIT, POWER_CAPPING_LV1_CPLD_OFFSET },
	{ "MEDHA0_PWR_CAP_LV2_CPLD", MEDHA0_PWR_CAP_LV2_BIT, POWER_CAPPING_LV2_LV3_CPLD_OFFSET },
	{ "MEDHA1_PWR_CAP_LV2_CPLD", MEDHA1_PWR_CAP_LV2_BIT, POWER_CAPPING_LV2_LV3_CPLD_OFFSET },
	{ "MEDHA0_PWR_CAP_LV3_CPLD", MEDHA0_PWR_CAP_LV3_BIT, POWER_CAPPING_LV2_LV3_CPLD_OFFSET },
	{ "MEDHA1_PWR_CAP_LV3_CPLD", MEDHA1_PWR_CAP_LV3_BIT, POWER_CAPPING_LV2_LV3_CPLD_OFFSET },
	{ "VR_HOT", VR_HOT_RAINBOW_BIT, ASIC_VR_HOT_SWITCH },
};

static const cpld_pin_map_t *get_power_capping_item(const char *name)
{
	for (int i = 0; i < ARRAY_SIZE(pwr_cap_list); i++) {
		if (!strcmp(name, pwr_cap_list[i].name)) {
			return &pwr_cap_list[i];
		}
	}
	return NULL;
}

static int cmd_power_capping_get(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_print(shell, "Usage:");
		shell_print(shell, "  power_capping_control get all");
		LOG_DBG("invalid argc in get: %d", (int)argc);
		return -1;
	}

	uint8_t board_id = get_asic_board_id();

	for (int i = 0; i < ARRAY_SIZE(pwr_cap_list); i++) {
		const cpld_pin_map_t *item = &pwr_cap_list[i];
		uint8_t bit_val = 0;

		if (!strcmp(item->name, "VR_HOT") && (board_id == ASIC_BOARD_ID_EVB)) {
			uint8_t io_val = 0;

			if (!tca6424a_i2c_read(VR_HOT_EVB_IOEXP_OUTPUT_OFFSET, &io_val, 1)) {
				LOG_DBG("tca6424a_i2c_read VR_HOT failed: offset=0x%02x",
					VR_HOT_EVB_IOEXP_OUTPUT_OFFSET);
				shell_error(shell, "read VR_HOT via IO exp failed");
				return -1;
			}

			bit_val = (io_val >> VR_HOT_EVB_BIT) & 0x1;

		} else {
			uint8_t reg_val = 0;

			if (!plat_read_cpld(item->offset, &reg_val, 1)) {
				LOG_DBG("plat_read_cpld failed: offset=0x%02x", item->offset);
				shell_error(shell, "read CPLD failed");
				return -1;
			}

			bit_val = (reg_val >> item->bit) & 0x1;
		}

		shell_print(shell, "%s : %d", item->name, bit_val);
	}

	return 0;
}

static int cmd_power_capping_set(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_print(shell, "Usage: power_capping_control set <NAME> <0|1>");
		LOG_DBG("invalid argc in set: %d", (int)argc);
		return -1;
	}

	uint8_t board_id = get_asic_board_id();
	const char *name = argv[1];
	long set_val = strtol(argv[2], NULL, 10);

	if (set_val != 0 && set_val != 1) {
		shell_error(shell, "Value must be 0 or 1");
		LOG_DBG("invalid value in set: %ld", set_val);
		return -1;
	}

	const cpld_pin_map_t *item = get_power_capping_item(name);
	if (!item) {
		shell_error(shell, "Unknown name: %s", name);
		LOG_DBG("unknown name in set: %s", name);
		return -1;
	}

	if (!strcmp(name, "VR_HOT") && (board_id == ASIC_BOARD_ID_EVB)) {
		if (!tca6424a_i2c_write_bit(VR_HOT_EVB_IOEXP_OUTPUT_OFFSET, VR_HOT_EVB_BIT,
					    (uint8_t)set_val)) {
			shell_error(shell, "write VR_HOT via IO exp failed");
			LOG_DBG("tca6424a_i2c_write_bit failed: offset=0x%02x bit=%d val=%ld",
				VR_HOT_EVB_IOEXP_OUTPUT_OFFSET, VR_HOT_EVB_BIT, set_val);
			return -1;
		}

		shell_print(shell, "set %s to %ld done (EVB IO exp)", name, set_val);
		return 0;
	}

	uint8_t reg_val = 0;

	if (!plat_read_cpld(item->offset, &reg_val, 1)) {
		LOG_DBG("plat_read_cpld failed: offset=0x%02x", item->offset);
		shell_error(shell, "read CPLD failed");
		return -1;
	}

	if (set_val == 1) {
		reg_val = POWER_CAPPING_SET_BIT(reg_val, item->bit);
	} else {
		reg_val = POWER_CAPPING_CLR_BIT(reg_val, item->bit);
	}

	if (!plat_write_cpld(item->offset, &reg_val)) {
		LOG_DBG("plat_write_cpld failed: offset=0x%02x val=0x%02x", item->offset, reg_val);
		shell_error(shell, "write CPLD failed");
		return -1;
	}

	shell_print(shell, "set %s to %ld done", name, set_val);
	return 0;
}

static void power_capping_dynamic_get_name(size_t idx, struct shell_static_entry *entry)
{
	entry->syntax = NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;

	if (idx < ARRAY_SIZE(pwr_cap_list)) {
		entry->syntax = pwr_cap_list[idx].name;
	}
}

SHELL_DYNAMIC_CMD_CREATE(set_dynamic, power_capping_dynamic_get_name);

SHELL_STATIC_SUBCMD_SET_CREATE(get_subcmds,
			       SHELL_CMD(all, NULL, "power_capping_control get all", NULL),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(power_capping_subcmds,
			       SHELL_CMD(get, &get_subcmds, "power_capping_control get all",
					 cmd_power_capping_get),
			       SHELL_CMD(set, &set_dynamic,
					 "power_capping_control set <NAME> <0|1>",
					 cmd_power_capping_set),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(power_capping_control, &power_capping_subcmds, "Power capping control via CPLD",
		   NULL);
