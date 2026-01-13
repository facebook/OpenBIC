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

#include <zephyr.h>
#include <stdlib.h>
#include <shell/shell.h>
#include "plat_cpld.h"
#include "mp2971.h"
#include "mp29816a.h"
#include "raa228249.h"
#include "plat_pldm_fw_update.h"
#include "plat_pldm_sensor.h"
#include "plat_hook.h"
#include "plat_class.h"
#include "plat_i2c.h"
#include "shell_iris_power.h"

LOG_MODULE_REGISTER(shell_fw_version);

#define ASIC_VERSION_BYTE 0x68
#define I2C_MAX_RETRY 3

bool temp_polling_flag = false;

typedef struct {
	uint8_t id;
	const char *name;
	uint8_t bus;
	uint8_t addr;
} asic_item_t;

static const asic_item_t asic_list[BOOT0_MAX] = {
	{ BOOT0_HAMSA, "HAMSA", I2C_BUS12, 0x32 },
	{ BOOT0_MEDHA0, "MEDHA0", I2C_BUS12, 0x32 },
	{ BOOT0_MEDHA1, "MEDHA1", I2C_BUS12, 0x32 },
};

void cmd_get_fw_version_vr(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 1) {
		shell_warn(shell, "Help: test get_fw_version vr");
		return;
	}
	// get polling flag and save it
	temp_polling_flag = get_plat_sensor_polling_enable_flag();
	/* Stop sensor polling */
	set_plat_sensor_polling_enable_flag(false);

	shell_print(shell, "comp_id |                rail name               |version |remain");
	for (int i = COMPNT_VR_1; i <= COMPNT_VR_3V3; i++) {
		uint8_t sensor_id = 0;
		char sensor_name[MAX_AUX_SENSOR_NAME_LEN] = { 0 };

		if (check_p3v3_p5v_pwrgd() == false) {
			shell_warn(shell,
				   "PWRGD_P3V3_R and PWRGD_P5V_R is not on, skip get VR version");
			continue;
		}

		if (i == COMPNT_HAMSA || i == COMPNT_MEDHA0 || i == COMPNT_MEDHA1)
			continue;

		if (i == COMPNT_VR_3V3 && (get_asic_board_id() != ASIC_BOARD_ID_EVB))
			continue;

		if (!find_sensor_id_and_name_by_firmware_comp_id(i, &sensor_id, sensor_name)) {
			LOG_ERR("Can't find sensor id and name by comp id: 0x%x", i);
			continue;
		}

		sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);
		if (cfg == NULL)
			continue;

		uint32_t version = 0;
		uint16_t remain = 0xFFFF;
		switch (cfg->type) {
		case sensor_dev_mp2971:
			if (!mp2971_get_checksum(cfg->port, cfg->target_addr, &version)) {
				shell_print(shell, "The VR MPS2971 version reading failed");
				continue;
			}
			break;
		case sensor_dev_mp29816a:
			if (!mp29816a_get_fw_version(cfg->port, cfg->target_addr, &version)) {
				shell_print(shell, "The VR MPS29816a version reading failed");
				continue;
			}
			break;
		case sensor_dev_raa228249:
			if (!raa228249_get_crc(cfg->port, cfg->target_addr, &version)) {
				shell_print(shell, "The VR RAA228249 version reading failed");
				continue;
			}
			if (raa228249_get_remaining_wr(cfg->port, cfg->target_addr,
						       (uint8_t *)&remain) < 0) {
				shell_print(shell, "The VR RAA228249 remaining reading failed");
				continue;
			}
			break;
		default:
			shell_print(shell, "Unsupport VR type(%d)", i);
			return;
		}

		if (remain != 0xFFFF) {
			remain = (uint8_t)((remain % 10) | (remain / 10 << 4));
		}

		if (cfg->type == sensor_dev_mp2891 || cfg->type == sensor_dev_mp29816a)
			shell_print(shell, "%-8x|%-40s|    %04x|%04x", i, sensor_name, version,
				    remain);
		else if (cfg->type == sensor_dev_isl69259 || cfg->type == sensor_dev_raa228238 ||
			 cfg->type == sensor_dev_raa228249 || cfg->type == sensor_dev_mp2971)
			shell_print(shell, "%-8x|%-40s|%08x|%04x", i, sensor_name, version, remain);
		else
			shell_print(shell, "not support sensor_dev: %d", cfg->type);
	}

	/* set back to the polling flag */
	set_plat_sensor_polling_enable_flag(temp_polling_flag);

	return;
}

void cmd_get_fw_version_cpld(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t data[4] = { 0 };
	uint32_t version = 0;

	if (!plat_read_cpld(CPLD_OFFSET_USERCODE, data, 4)) {
		shell_warn(shell, "cpld read 0x%02x fail", CPLD_OFFSET_USERCODE);
		return;
	}

	version = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
	shell_print(shell, "The cpld version: %08x", version);
	return;
}

void cmd_get_fw_version_asic(const struct shell *shell, size_t argc, char **argv)
{
	I2C_MSG i2c_msg = { .bus = asic_list[BOOT0_HAMSA].bus,
			    .target_addr = asic_list[BOOT0_HAMSA].addr };
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 11;
	i2c_msg.data[0] = ASIC_VERSION_BYTE;
	if (i2c_master_read(&i2c_msg, I2C_MAX_RETRY)) {
		shell_warn(shell, "Can't get boot0, boot1 version from ASIC");
		return;
	}
	shell_print(shell, " boot1 VER from asic: %02d.%02d.%02d", i2c_msg.data[2], i2c_msg.data[3],
		    i2c_msg.data[4]);
	shell_print(shell, " boot0 VER from asic: %02d.%02d.%02d", i2c_msg.data[9], i2c_msg.data[8],
		    i2c_msg.data[7]);
	uint32_t tmp_version_boot0 = i2c_msg.data[9] << 16 | i2c_msg.data[8] << 8 | i2c_msg.data[7];
	// if boot0 version is not 0, update temp data
	if (tmp_version_boot0) {
		// update temp data
		shell_print(shell, "update boot0 version read from asic");
		for (int i = 0; i < BOOT0_MAX; i++) {
			update_temp_boot0_version(tmp_version_boot0, i);
		}
	}
	// get temp version from temp array
	for (int i = 0; i < BOOT0_MAX; i++) {
		uint32_t version_tmp = plat_get_image_version(i);
		unsigned char tmp_bytes[4];
		tmp_bytes[0] = (version_tmp >> 24) & 0xFF;
		tmp_bytes[1] = (version_tmp >> 16) & 0xFF;
		tmp_bytes[2] = (version_tmp >> 8) & 0xFF;
		tmp_bytes[3] = version_tmp & 0xFF;
		shell_print(shell, " boot0 VER from temp array: %02d.%02d.%02d.%02d", tmp_bytes[0],
			    tmp_bytes[1], tmp_bytes[2], tmp_bytes[3]);
	}
	return;
}

void cmd_get_fw_version_asic_from_flash(const struct shell *shell, size_t argc, char **argv)
{
	// want to show like VER : 01.06.00 | CRC32 : e9e2b0ba
	uint32_t version = 0;
	uint32_t crc32 = 0;

	plat_set_cpld_reset_reg(RESET_CPLD_OFF);

	if (plat_get_image_crc_checksum_from_flash(COMPNT_HAMSA, &version, &crc32)) {
		shell_print(shell, "HAMSA boot0 VER : %08x | CRC32 : %08x", version, crc32);
	} else {
		shell_warn(shell, "HAMSA boot0 VER and CRC reading failed");
	}

	if (plat_get_image_crc_checksum_from_flash(COMPNT_MEDHA0, &version, &crc32)) {
		shell_print(shell, "MEDHA0 boot0 VER : %08x | CRC32 : %08x", version, crc32);
	} else {
		shell_warn(shell, "MEDHA0 boot0 VER and CRC reading failed");
	}

	if (plat_get_image_crc_checksum_from_flash(COMPNT_MEDHA1, &version, &crc32)) {
		shell_print(shell, "MEDHA1 boot0 VER : %08x | CRC32 : %08x", version, crc32);
	} else {
		shell_warn(shell, "MEDHA1 boot0 VER and CRC reading failed");
	}

	plat_set_cpld_reset_reg(RESET_CPLD_ON);

	return;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_get_fw_version_cmd, SHELL_CMD(vr, NULL, "get fw version vr", cmd_get_fw_version_vr),
	SHELL_CMD(cpld, NULL, "get fw version cpld", cmd_get_fw_version_cpld),
	SHELL_CMD(asic, NULL, "get fw version asic", cmd_get_fw_version_asic),
	SHELL_CMD(force_read_asic, NULL, "get fw version asic from flash",
		  cmd_get_fw_version_asic_from_flash),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(get_fw_version, &sub_get_fw_version_cmd, "get fw version command", NULL);
