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

#include <stdlib.h>
#include <shell/shell.h>
#include <string.h>
#include <limits.h>
#include <stdio.h>
#include "hal_i2c.h"
#include "plat_i2c.h"

#define ASIC_I2C_BUS I2C_BUS12
#define ASIC_I2C_ADDR 0x32
#define I2C_MAX_RETRY 3

//asic reg define
#define ASIC_REG_STATUS_REG 0x00
#define ASIC_MONITOR_HBM_TEMP_REG 0x8F // Max of 8 HBM temperature sensors in history
#define ASIC_MONITOR_TEMP_REG 0x70
#define ASIC_VERSION_REG 0x68
//asic reg len
#define ASIC_STATUS_REG_LEN 8
#define ASIC_VERSION_REG_LEN 11
#define ASIC_MONITOR_TEMP_REG_LEN 10
#define ASIC_MONITOR_HBM_TEMP_REG_LEN 10

int asic_read_cmd(const struct shell *shell, uint8_t reg, uint8_t *data, uint8_t len)
{
	I2C_MSG i2c_msg = {
		.bus = ASIC_I2C_BUS,
		.target_addr = ASIC_I2C_ADDR,
	};
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = len;
	i2c_msg.data[0] = reg;
	if (i2c_master_read(&i2c_msg, I2C_MAX_RETRY)) {
		shell_error(shell, "Can't get data from ASIC, reg: 0x%02x", reg);
		return -1;
	}
	memcpy(data, i2c_msg.data, len);
	return 0;
}
void asic_boot_status_cmd(const struct shell *shell)
{
	/*
	BYTE[1]
	bit6: driver_not_ready – Bit is set to ‘1’ 
	when the device(boot1 firmware) is not capable of processing management commands,. 
	If cleared to ‘0’, 
	then the firmware is initialised and ready to respond to management commands. 
	*/

	uint8_t status_data[ASIC_STATUS_REG_LEN] = { 0 };
	if (asic_read_cmd(shell, ASIC_REG_STATUS_REG, (uint8_t *)status_data,
			  ASIC_STATUS_REG_LEN) != 0) {
		shell_warn(shell, "Can't get status data from ASIC");
		return;
	}
	shell_print(shell, "Boot status from asic: 0x%02x", status_data[1]);
	if (status_data[1] & 0x40) {
		shell_print(shell, "Asic is not ready...");
		return;
	}

	shell_print(shell, "Asic is ready !");
}

void asic_version_cmd(const struct shell *shell)
{
	uint8_t version_data[ASIC_VERSION_REG_LEN] = { 0 };
	if (asic_read_cmd(shell, ASIC_VERSION_REG, (uint8_t *)version_data, ASIC_VERSION_REG_LEN) !=
	    0) {
		shell_warn(shell, "Can't get version data from ASIC");
		return;
	}

	shell_print(shell, " boot1 VER from asic: %02d.%02d.%02d", version_data[2], version_data[3],
		    version_data[4]);
	shell_print(shell, " boot0 VER from asic: %02d.%02d.%02d", version_data[9], version_data[8],
		    version_data[7]);
}

void max_asic_temp_history_cmd(const struct shell *shell)
{
	/*
	MONITOR_HBM_TEMP_REG byte format:
	0	length
	1	hamsa_remote_temp
	2	cip0_remote_temp
	3	cip1_remote_temp
	4	owle_remote_temp
	5	owlw_remote_temp
	6	max_asic_temp
	7:8	reserved
	9	pec
	*/
	uint8_t temp_data[ASIC_MONITOR_TEMP_REG_LEN] = { 0 };
	if (asic_read_cmd(shell, ASIC_MONITOR_TEMP_REG, (uint8_t *)temp_data,
			  ASIC_MONITOR_TEMP_REG_LEN) != 0) {
		shell_warn(shell, "Can't get max asic temp data from ASIC, reg: 0x%02x",
			   ASIC_MONITOR_TEMP_REG);
		return;
	}
	shell_print(shell, "  %-20s raw: 0x%02X -> %d degC", "hamsa_remote_temp", temp_data[1],
		    temp_data[1]);
	shell_print(shell, "  %-20s raw: 0x%02X -> %d degC", "cip0_remote_temp", temp_data[2],
		    temp_data[2]);
	shell_print(shell, "  %-20s raw: 0x%02X -> %d degC", "cip1_remote_temp", temp_data[3],
		    temp_data[3]);
	shell_print(shell, "  %-20s raw: 0x%02X -> %d degC", "owle_remote_temp", temp_data[4],
		    temp_data[4]);
	shell_print(shell, "  %-20s raw: 0x%02X -> %d degC", "owlw_remote_temp", temp_data[5],
		    temp_data[5]);
	shell_print(shell, "  %-20s raw: 0x%02X -> %d degC", "max_asic_temp", temp_data[6],
		    temp_data[6]);
	/*
	ASIC_MONITOR_HBM_TEMP_REG byte format:
	0	length
	1	cip0_hbm0_remote_temp
	2	cip0_hbm1_remote_temp
	3	cip0_hbm2_remote_temp
	4	cip0_hbm3_remote_temp
	5	cip1_hbm0_remote_temp
	6	cip1_hbm1_remote_temp
	7	cip1_hbm2_remote_temp
	8	cip1_hbm3_remote_temp
	9	pec
	*/
	uint8_t hbm_temp_data[ASIC_MONITOR_HBM_TEMP_REG_LEN] = { 0 };
	if (asic_read_cmd(shell, ASIC_MONITOR_HBM_TEMP_REG, (uint8_t *)hbm_temp_data,
			  ASIC_MONITOR_HBM_TEMP_REG_LEN) != 0) {
		shell_warn(shell, "Can't get max asic temp data from ASIC, reg: 0x%02x",
			   ASIC_MONITOR_HBM_TEMP_REG);
		return;
	}
	shell_print(shell, "  %-20s raw: 0x%02X -> %d degC", "cip0_hbm0_remote_temp",
		    hbm_temp_data[1], hbm_temp_data[1]);
	shell_print(shell, "  %-20s raw: 0x%02X -> %d degC", "cip0_hbm1_remote_temp",
		    hbm_temp_data[2], hbm_temp_data[2]);
	shell_print(shell, "  %-20s raw: 0x%02X -> %d degC", "cip0_hbm2_remote_temp",
		    hbm_temp_data[3], hbm_temp_data[3]);
	shell_print(shell, "  %-20s raw: 0x%02X -> %d degC", "cip0_hbm3_remote_temp",
		    hbm_temp_data[4], hbm_temp_data[4]);
	shell_print(shell, "  %-20s raw: 0x%02X -> %d degC", "cip1_hbm0_remote_temp",
		    hbm_temp_data[5], hbm_temp_data[5]);
	shell_print(shell, "  %-20s raw: 0x%02X -> %d degC", "cip1_hbm1_remote_temp",
		    hbm_temp_data[6], hbm_temp_data[6]);
	shell_print(shell, "  %-20s raw: 0x%02X -> %d degC", "cip1_hbm2_remote_temp",
		    hbm_temp_data[7], hbm_temp_data[7]);
	shell_print(shell, "  %-20s raw: 0x%02X -> %d degC", "cip1_hbm3_remote_temp",
		    hbm_temp_data[8], hbm_temp_data[8]);
}

void asic_read_all_cmd(const struct shell *shell, size_t argc, char **argv)
{
	if (argc > 1) {
		shell_error(shell, "Usage: asic read_all");
		return;
	}

	shell_print(shell, "=== Asic Dump ===");
	// 1. Boot Status
	// ASIC_REG_STATUS_REG
	shell_print(shell, "\n[1/3] System Boot Status");
	shell_print(shell, "============================================");
	asic_boot_status_cmd(shell);
	// 2. Max ASIC Temperature (all)
	// ASIC_MONITOR_HBM_TEMP_REG
	shell_print(shell, "\n[2/3] MAX ASIC Temperature History");
	shell_print(shell, "============================================");
	max_asic_temp_history_cmd(shell);
	// 3. Version Information (all)
	shell_print(shell, "\n[3/3] Firmware & Hardware Versions");
	shell_print(shell, "============================================");
	asic_version_cmd(shell);

	shell_print(shell, "\n=== Asic Dump Complete ===");
}

/**
 * @brief Display help information for all Asic commands
 * @param shell Shell instance
 * @param argc Argument count
 * @param argv Argument values
 */
void asic_help_cmd(const struct shell *shell, size_t argc, char **argv)
{
	shell_info(shell, "Usage: Asic help");
	shell_info(shell, "       Asic read_all");
}

/* Sub-command Level 1 of  commands */
SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_asic_cmds, SHELL_CMD(read_all, NULL, "read all Asic system data", asic_read_all_cmd),
	SHELL_CMD(help, NULL, "display help information for Asic commands", asic_help_cmd),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(rb_asic, &sub_asic_cmds, "Rainbow Asic low-level commands", NULL);
