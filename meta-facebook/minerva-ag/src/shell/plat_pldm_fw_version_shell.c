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
#include "pdr.h"

LOG_MODULE_REGISTER(plat_pldm_fw_version_shell);

void cmd_get_fw_version_vr(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 1) {
		shell_warn(shell, "Help: test get_fw_version vr");
		return;
	}

	/* Stop sensor polling */
	disable_sensor_poll();

	shell_print(shell, "comp_id |              sensor_name               |version |remain");
	for (int i = 0; i < get_aegis_compnt_mapping_sensor_table_count(); i++) {
		uint8_t comp_identifier = i + 1;
		uint8_t bus = 0;
		uint8_t addr = 0;
		uint8_t sensor_id = 0;
		uint8_t sensor_dev = 0;
		char sensor_name[MAX_AUX_SENSOR_NAME_LEN] = { 0 };
		find_sensor_id_and_name_by_firmware_comp_id(comp_identifier, &sensor_id,
							    sensor_name);
		find_vr_addr_and_bus_and_sensor_dev_by_sensor_id(sensor_id, &bus, &addr,
								 &sensor_dev);
		uint8_t type = get_vr_type();
		uint32_t version = 0;
		uint16_t remain = 0xFFFF;
		switch (type) {
		case VR_RNS_ISL69260_RAA228238: {
			if (sensor_dev == sensor_dev_isl69259) {
				if (!raa229621_get_crc(bus, addr, &version)) {
					shell_print(shell,
						    "The VR ISL69260 version reading failed");
					return;
				}
				if (raa229621_get_remaining_wr(bus, addr, (uint8_t *)&remain) < 0) {
					shell_print(shell,
						    "The VR ISL69260 remaining reading failed");
					return;
				}
			} else if (sensor_dev == sensor_dev_raa228238) {
				if (!raa229621_get_crc(bus, addr, &version)) {
					shell_print(shell,
						    "The VR RAA228238 version reading failed");
					return;
				}
				if (raa229621_get_remaining_wr(bus, addr, (uint8_t *)&remain) < 0) {
					shell_print(shell,
						    "The VR RAA228238 remaining reading failed");
					return;
				}
			}
			break;
		}
		case VR_MPS_MP2971_MP2891: {
			if (sensor_dev == sensor_dev_mp2971) {
				if (!mp2971_get_checksum(bus, addr, &version)) {
					shell_print(shell, "The VR MPS2971 version reading failed");
					return;
				}
			} else if (sensor_dev == sensor_dev_mp2891) {
				if (!mp2891_get_fw_version(bus, addr, &version)) {
					shell_print(shell, "The VR MPS289x version reading failed");
					return;
				}
			}
			break;
		}
		default:
			shell_print(shell, "Unsupport VR type(%x)", type);
			return;
		}

		if (remain != 0xFFFF) {
			remain = (uint8_t)((remain % 10) | (remain / 10 << 4));
		}

		if (sensor_dev == sensor_dev_mp2891) {
			shell_print(shell, "%-8x|%-40s|    %04x|%04x", comp_identifier, sensor_name,
				    version, remain);
		} else {
			shell_print(shell, "%-8x|%-40s|%08x|%04x", comp_identifier, sensor_name,
				    version, remain);
		}
	}

	/* Start sensor polling */
	enable_sensor_poll();
	return;
}
