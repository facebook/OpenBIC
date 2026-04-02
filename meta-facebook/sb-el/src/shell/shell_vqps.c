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
#include <stddef.h>
#include <logging/log.h>

#include "plat_cpld.h"
#include "plat_class.h"
#include "plat_ioexp.h"

LOG_MODULE_REGISTER(vqps);

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

/* Non-EVB VQPS status bits in CPLD 0x13 */
#define VQPS_STATUS_CPLD_OFFSET 0x13
/* EVB VQPS status bits moved to CPLD 0xA7 */
#define EVB_VQPS_CPLD_OFFSET 0xA7

/* VQPS status bits (same bit positions, different CPLD offset by board) */
#define NUWA0_VQPS_TOP_EN_BIT 4
#define NUWA1_VQPS_TOP_EN_BIT 3
#define NUWA0_VQPS_U_EN_BIT 2
#define NUWA1_VQPS_U_EN_BIT 1
#define HAMSA_VQPS_EFUSE_USER_BIT 0

/* EVB VQPS enable bits via TCA6424A OUTPUT_PORT_2 */
#define NUWA1_VQPS_W_EN_BIT 4
#define NUWA0_VQPS_W_EN_BIT 3
#define NUWA1_VQPS_E_EN_BIT 2
#define NUWA0_VQPS_E_EN_BIT 1

/* EVB P1V8 OWL EW VQPS enable via U200070 (PCA6554APW) OUTPUT_PORT bit7 */
#define EVB_P1V8_OWL_EW_VQPS_EN_BIT 7

#define VQPS_SET_BIT(orig, bit) ((uint8_t)((orig) | (1u << (bit))))
#define VQPS_CLR_BIT(orig, bit) ((uint8_t)((orig) & ~(1u << (bit))))

/*
 * vqps_list layout:
 * 0..4 : status bits (CPLD offset depends on board)
 * 5..8 : EVB-only, TCA6424A OUTPUT_PORT_2 bits
 * 9    : EVB-only, U200070 (PCA6554APW) OUTPUT_PORT bit7
 */
static const cpld_pin_map_t vqps_list[] = {
	{ "NUWA0_VQPS_TOP_EN", NUWA0_VQPS_TOP_EN_BIT, 0 },
	{ "NUWA1_VQPS_TOP_EN", NUWA1_VQPS_TOP_EN_BIT, 0 },
	{ "NUWA0_VQPS_U_EN", NUWA0_VQPS_U_EN_BIT, 0 },
	{ "NUWA1_VQPS_U_EN", NUWA1_VQPS_U_EN_BIT, 0 },
	{ "HAMSA_VQPS_EFUSE_USER", HAMSA_VQPS_EFUSE_USER_BIT, 0 },

	/* EVB-only, via TCA6424A OUTPUT_PORT_2 */
	{ "NUWA0_VQPS_W_EN", NUWA0_VQPS_W_EN_BIT, 0 },
	{ "NUWA1_VQPS_W_EN", NUWA1_VQPS_W_EN_BIT, 0 },
	{ "NUWA0_VQPS_E_EN", NUWA0_VQPS_E_EN_BIT, 0 },
	{ "NUWA1_VQPS_E_EN", NUWA1_VQPS_E_EN_BIT, 0 },

	/* EVB-only, via U200070 (PCA6554APW) OUTPUT_PORT bit7 */
	{ "P1V8_OWL_EW_VQPS_EN", 0, 0 },
};

static uint8_t get_vqps_status_offset(void)
{
	// return (get_asic_board_id() == ASIC_BOARD_ID_EVB) ? EVB_VQPS_CPLD_OFFSET :
	// 						    VQPS_STATUS_CPLD_OFFSET;
	return VQPS_STATUS_CPLD_OFFSET;
}

static const cpld_pin_map_t *get_vqps_item(const char *name)
{
	for (size_t i = 0; i < ARRAY_SIZE(vqps_list); i++) {
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
		return -1;
	}

	uint8_t board_id = get_asic_board_id();
	uint8_t status_off = get_vqps_status_offset();

	/* 0..4: status bits from CPLD (offset depends on board) */
	uint8_t reg_status = 0;
	if (!plat_read_cpld(status_off, &reg_status, 1)) {
		shell_error(shell, "read CPLD 0x%02X failed", status_off);
		return -1;
	}

	for (int i = 0; i < 5; i++) {
		const cpld_pin_map_t *item = &vqps_list[i];
		uint8_t bit_val = (reg_status >> item->bit) & 0x1;
		shell_print(shell, "%s : %d", item->name, bit_val);
	}

	/* EVB-only items */
	if (board_id != ASIC_BOARD_ID_EVB) {
		return 0;
	}

	/* 5..8: TCA6424A OUTPUT_PORT_2 */
	uint8_t tca_p2 = 0;
	if (!tca6424a_i2c_read(TCA6424A_OUTPUT_PORT_2, &tca_p2, 1)) {
		shell_error(shell, "read TCA6424A OUTPUT_PORT_2 failed");
		return -1;
	}

	for (int i = 5; i <= 8; i++) {
		const cpld_pin_map_t *item = &vqps_list[i];
		uint8_t bit_val = (tca_p2 >> item->bit) & 0x1;
		shell_print(shell, "%s : %d", item->name, bit_val);
	}

	/* 9: U200070 (PCA6554APW) OUTPUT_PORT bit7 */
	uint8_t out = 0;
	if (get_pca6554apw_ioe_value(U200070_IO_I2C_BUS, U200070_IO_ADDR, OUTPUT_PORT, &out) != 0) {
		shell_error(shell, "read U200070 OUTPUT_PORT failed");
		return -1;
	}
	shell_print(shell, "%s : %d", "P1V8_OWL_EW_VQPS_EN",
		    (out >> EVB_P1V8_OWL_EW_VQPS_EN_BIT) & 0x1);

	return 0;
}

static int cmd_vqps_set(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_print(shell, "Usage: vqps set <NAME> <0|1>");
		return -1;
	}

	const char *name = argv[1];
	long set_val = strtol(argv[2], NULL, 10);
	if (set_val != 0 && set_val != 1) {
		shell_error(shell, "Value must be 0 or 1");
		return -1;
	}

	const cpld_pin_map_t *item = get_vqps_item(name);
	if (!item) {
		shell_error(shell, "Unknown name: %s", name);
		return -1;
	}

	uint8_t board_id = get_asic_board_id();
	uint8_t status_off = get_vqps_status_offset();
	ptrdiff_t idx = item - vqps_list;

	/* EVB-only: P1V8_OWL_EW_VQPS_EN via U200070 */
	if (!strcmp(name, "P1V8_OWL_EW_VQPS_EN")) {
		if (board_id != ASIC_BOARD_ID_EVB) {
			shell_error(shell, "%s is EVB-only, board_id=0x%02X", name, board_id);
			return -1;
		}

		uint8_t out = 0;
		if (get_pca6554apw_ioe_value(U200070_IO_I2C_BUS, U200070_IO_ADDR, OUTPUT_PORT,
					     &out) != 0) {
			shell_error(shell, "read U200070 OUTPUT_PORT failed");
			return -1;
		}

		if (set_val) {
			out |= BIT(EVB_P1V8_OWL_EW_VQPS_EN_BIT);
		} else {
			out &= ~BIT(EVB_P1V8_OWL_EW_VQPS_EN_BIT);
		}

		set_pca6554apw_ioe_value(U200070_IO_I2C_BUS, U200070_IO_ADDR, OUTPUT_PORT, out);
		shell_print(shell, "set %s to %ld done (U200070)", name, set_val);
		return 0;
	}

	/* 0..4 : status bits -> CPLD offset depends on board */
	if (idx >= 0 && idx < 5) {
		uint8_t reg_val = 0;

		if (!plat_read_cpld(status_off, &reg_val, 1)) {
			shell_error(shell, "read CPLD 0x%02X failed", status_off);
			return -1;
		}

		if (set_val) {
			reg_val = VQPS_SET_BIT(reg_val, item->bit);
		} else {
			reg_val = VQPS_CLR_BIT(reg_val, item->bit);
		}

		if (!plat_write_cpld(status_off, &reg_val)) {
			shell_error(shell, "write CPLD 0x%02X failed", status_off);
			return -1;
		}

		shell_print(shell, "set %s to %ld done (CPLD 0x%02X)", name, set_val, status_off);
		return 0;
	}

	/* 5..8 : EVB-only, via TCA6424A OUTPUT_PORT_2 bits */
	if (idx >= 5 && idx <= 8) {
		if (board_id != ASIC_BOARD_ID_EVB) {
			shell_error(shell, "%s is EVB-only, board_id=0x%02X", name, board_id);
			return -1;
		}

		if (!tca6424a_i2c_write_bit(TCA6424A_OUTPUT_PORT_2, item->bit, (uint8_t)set_val)) {
			shell_error(shell, "write TCA6424A OUTPUT_PORT_2 bit %d failed", item->bit);
			return -1;
		}

		shell_print(shell, "set %s to %ld done (TCA6424A port2)", name, set_val);
		return 0;
	}

	shell_error(shell, "Unhandled VQPS item: %s (idx=%d)", name, (int)idx);
	return -1;
}

SHELL_STATIC_SUBCMD_SET_CREATE(get_subcmds, SHELL_CMD(all, NULL, "vqps get all", cmd_vqps_get),
			       SHELL_SUBCMD_SET_END);

static void vqps_dynamic_get_name(size_t idx, struct shell_static_entry *entry)
{
	uint8_t board_id = get_asic_board_id();

	size_t max_cnt = ARRAY_SIZE(vqps_list);
	if (board_id != ASIC_BOARD_ID_EVB) {
		max_cnt = 5; /* only status items */
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

SHELL_CMD_REGISTER(vqps, &vqps_subcmds, "VQPS control via CPLD/TCA6424A/U200070", NULL);