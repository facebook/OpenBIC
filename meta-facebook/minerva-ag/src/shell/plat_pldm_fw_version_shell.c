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
#include "sensor.h"
#include "plat_class.h"
#include "plat_pldm_sensor.h"
#include "plat_pldm_fw_update.h"
#include "mp2971.h"
#include "mp2891.h"
#include "raa229621.h"
#include "raa228249.h"
#include "pdr.h"
#include "mp29816a.h"
#include "plat_pldm_sensor.h"
#include "plat_isr.h"
#include "plat_i2c.h"

#define AEGIS_CPLD_ADDR (0x4C >> 1)

LOG_MODULE_REGISTER(plat_pldm_fw_version_shell);

void cmd_get_fw_version_vr(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 1) {
		shell_warn(shell, "Help: test get_fw_version vr");
		return;
	}

	/* Stop sensor polling */
	set_plat_sensor_polling_enable_flag(false);

	shell_print(shell, "comp_id |                rail name               |version |remain");
	for (int i = 0; i < get_aegis_vr_compnt_mapping_sensor_table_count(); i++) {
		if ((get_board_type() == MINERVA_AEGIS_BD) && (i == 0))
			continue; // skip osfp p3v3 on AEGIS BD
		uint8_t comp_identifier = i + 1;
		uint8_t bus = 0;
		uint8_t addr = 0;
		uint8_t sensor_id = 0;
		uint8_t sensor_dev = 0;
		char sensor_name[MAX_AUX_SENSOR_NAME_LEN] = { 0 };
		if (!find_sensor_id_and_name_by_firmware_comp_id(comp_identifier, &sensor_id,
								 sensor_name)) {
			shell_print(shell, "Can't find sensor id and name by comp id: 0x%x",
				    comp_identifier);
			continue;
		}
		if (!get_sensor_info_by_sensor_id(sensor_id, &bus, &addr, &sensor_dev)) {
			shell_print(shell, "Can't find vr addr and bus by sensor id: 0x%x",
				    sensor_id);
			continue;
		}
		uint8_t type = get_vr_type();
		uint32_t version = 0;
		uint16_t remain = 0xFFFF;
		switch (sensor_dev) {
		case sensor_dev_isl69259:
			if (!raa229621_get_crc(bus, addr, &version)) {
				shell_print(shell, "The VR ISL69260 version reading failed");
				continue;
			}
			if (raa229621_get_remaining_wr(bus, addr, (uint8_t *)&remain) < 0) {
				shell_print(shell, "The VR ISL69260 remaining reading failed");
				continue;
			}
			break;
		case sensor_dev_raa228238:
			if (!raa229621_get_crc(bus, addr, &version)) {
				shell_print(shell, "The VR RAA228238 version reading failed");
				continue;
			}
			if (raa229621_get_remaining_wr(bus, addr, (uint8_t *)&remain) < 0) {
				shell_print(shell, "The VR RAA228238 remaining reading failed");
				continue;
			}
			break;
		case sensor_dev_mp2971:
			if (!mp2971_get_checksum(bus, addr, &version)) {
				shell_print(shell, "The VR MPS2971 version reading failed");
				continue;
			}
			break;
		case sensor_dev_mp2891:
			if (!mp2891_get_fw_version(bus, addr, &version)) {
				shell_print(shell, "The VR MPS2891 version reading failed");
				continue;
			}
			break;
		case sensor_dev_mp29816a:
			if (!mp29816a_get_fw_version(bus, addr, &version)) {
				shell_print(shell, "The VR MPS29816a version reading failed");
				continue;
			}
			break;
		case sensor_dev_raa228249:
			if (!raa228249_get_crc(bus, addr, &version)) {
				shell_print(shell, "The VR RAA228249 version reading failed");
				continue;
			}
			if (raa228249_get_remaining_wr(bus, addr, (uint8_t *)&remain) < 0) {
				shell_print(shell, "The VR RAA228249 remaining reading failed");
				continue;
			}
			break;
		default:
			shell_print(shell, "Unsupport VR type(%d)", type);
			return;
		}

		if (remain != 0xFFFF) {
			remain = (uint8_t)((remain % 10) | (remain / 10 << 4));
		}

		if (sensor_dev == sensor_dev_mp2891 || sensor_dev == sensor_dev_mp29816a)
			shell_print(shell, "%-8x|%-50s|    %04x|%04x", comp_identifier, sensor_name,
				    version, remain);
		else if (sensor_dev == sensor_dev_isl69259 || sensor_dev == sensor_dev_raa228238 ||
			 sensor_dev == sensor_dev_raa228249 || sensor_dev == sensor_dev_mp2971)
			shell_print(shell, "%-8x|%-50s|%08x|%04x", comp_identifier, sensor_name,
				    version, remain);
		else
			shell_print(shell, "not support sensor_dev: %d", sensor_dev);
	}

	/* Start sensor polling */
	set_plat_sensor_polling_enable_flag(true);

	return;
}

void cmd_get_fw_version_cpld(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 1) {
		shell_warn(shell, "Help: test get_fw_version cpld");
		return;
	}

	uint8_t data[4] = { 0 };
	uint32_t version = 0;
	if (!plat_i2c_read(I2C_BUS5, AEGIS_CPLD_ADDR, 0x44, data, 4)) {
		LOG_ERR("Failed to read cpld version from cpld");
		return;
	}
	version = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
	shell_print(shell, "The cpld version: %08x", version);
	return;
}
