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
#include <stdio.h>
#include "plat_log.h"
#include "plat_fru.h"
#include "plat_cpld.h"
#include "plat_user_setting.h"

typedef struct {
	uint8_t cpld_offset;
	const char *reg_name;
	const char *bit_name[8];
} cpld_bit_name_table_t;

const cpld_bit_name_table_t cpld_bit_name_table[] = {
	{ VR_POWER_FAULT_1_REG,
	  "VR Power Fault   (1:Power Fault, 0=Normal)",
	  {
		  "RSVD",
		  "RSVD",
		  "HAMSA_VDDHRXTX_PCIE",
		  "HAMSA_AVDD_PCIE",
		  "OWL_W_TRVDD0P75",
		  "OWL_E_TRVDD0P75",
		  "OWL_W_TRVDD0P9",
		  "OWL_E_TRVDD0P9",
	  } },
	{ VR_POWER_FAULT_2_REG,
	  "VR Power Fault   (1:Power Fault, 0=Normal)",
	  {
		  "MAX_N_VDD",
		  "MAX_M_VDD",
		  "MAX_S_VDD",
		  "HAMSA_VDD",
		  "OWL_W_VDD",
		  "OWL_E_VDD",
		  "MEDHA0_VDD",
		  "MEDHA1_VDD",
	  } },
	{ VR_POWER_FAULT_3_REG,
	  "VR Power Fault   (1:Power Fault, 0=Normal)",
	  {
		  "VDDPHY_HBM1_HBM3_HBM5_HBM7",
		  "VPP_HBM1_HBM3_HBM5_HBM7",
		  "VDDQC_HBM1_HBM3_HBM5_HBM7",
		  "VDDQL_HBM1_HBM3_HBM5_HBM7",
		  "VDDPHY_HBM0_HBM2_HBM4_HBM6",
		  "VPP_HBM0_HBM2_HBM4_HBM6",
		  "VDDQC_HBM0_HBM2_HBM4_HBM6",
		  "VDDQL_HBM0_HBM2_HBM4_HBM6",
	  } },
	{ VR_POWER_FAULT_4_REG,
	  "VR Power Fault   (1:Power Fault, 0=Normal)",
	  {
		  "P1V5_W_RVDD",
		  "P1V5_E_RVDD",
		  "P0V9_OWL_W_PVDD",
		  "P0V9_OWL_E_PVDD",
		  "PLL_VDDA15_HBM5_HBM7",
		  "PLL_VDDA15_HBM1_HBM3",
		  "PLL_VDDA15_HBM4_HBM6",
		  "PLL_VDDA15_HBM0_HBM2",
	  } },
	{ VR_POWER_FAULT_5_REG,
	  "VR Power Fault   (1:Power Fault, 0=Normal)",
	  {
		  "PVDD1P5",
		  "P1V5_PLL_VDDA_SOC",
		  "P1V5_PLL_VDDA_OWL",
		  "LDO_IN_1V2",
		  "P1V8",
		  "P3V3",
		  "P5V",
		  "P12V_UBC_PWRGD",
	  } },
	{ VR_SMBUS_ALERT_EVENT_LOG_REG,
	  "VR SMBALRT , Status",
	  {
		  "RSVD",
		  "MAX_N_VDDRXTX_SMBALRT_N",
		  "VDDQC_VDDQL_0246_SMBALRT_N",
		  "MAX_M_VDDQC_1357_SMBALRT_N",
		  "OWL_W_SMBALRT_N",
		  "OWL_E_SMBALRT_N",
		  "MEDHA1_VDD_ALERT_R_N",
		  "MEDHA0_VDD_ALERT_R_N",
	  } }
};

const char *get_cpld_reg_name(uint8_t cpld_offset)
{
	for (int i = 0; i < ARRAY_SIZE(cpld_bit_name_table); i++) {
		if (cpld_bit_name_table[i].cpld_offset == cpld_offset) {
			return cpld_bit_name_table[i].reg_name;
		}
	}
	return "NA";
}

const char *get_cpld_bit_name(uint8_t cpld_offset, uint8_t bit_pos)
{
	for (int i = 0; i < ARRAY_SIZE(cpld_bit_name_table); i++) {
		if (cpld_bit_name_table[i].cpld_offset == cpld_offset) {
			if (bit_pos < 8 && cpld_bit_name_table[i].bit_name[bit_pos]) {
				return cpld_bit_name_table[i].bit_name[bit_pos];
			}
		}
	}
	return "NA";
}

void cmd_set_event(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_warn(shell, "Help: test log set_event <error_code_1> <error_code_2> ");
		shell_warn(shell, "error_code_1: high byte, error_code_2: low byte");
		return;
	}

	uint16_t error_code = ((strtol(argv[1], NULL, 16)) << 8) | (strtol(argv[2], NULL, 16));
	shell_print(shell, "Generate error code: 0x%x", error_code);

	error_log_event(error_code, LOG_ASSERT);

	return;
}

void cmd_log_dump(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 1) {
		shell_warn(shell, "Help: test log log_dump");
		return;
	}

	shell_print(shell, "Log number: %d", plat_log_get_num());

	shell_print(
		shell,
		"=============================      LOG DUMP START      =============================");
	for (int i = 0; i < plat_log_get_num(); i++) {
		plat_err_log_mapping log = { 0 };
		plat_log_read((uint8_t *)&log, FRU_LOG_SIZE, i + 1);

		uint8_t err_type = (log.err_code >> 13) & 0x07;

		shell_print(shell, "index %d:", log.index);
		shell_print(shell, "error_code: 0x%x", log.err_code);

		uint8_t cpld_offset = log.err_code & 0xFF;
		uint8_t bit_position = (log.err_code >> 8) & 0x07;

		const char *reg_name = get_cpld_reg_name(cpld_offset);
		const char *bit_name = get_cpld_bit_name(cpld_offset, bit_position);

		switch (err_type) {
		case CPLD_UNEXPECTED_VAL_TRIGGER_CAUSE:
			shell_print(shell, "\t%s", reg_name);
			shell_print(shell, "\t\t%s", bit_name);
			break;
		case POWER_ON_SEQUENCE_TRIGGER_CAUSE:
			shell_print(shell, "\tPOWER_ON_SEQUENCE_TRIGGER");
			break;
		case AC_ON_TRIGGER_CAUSE:
			shell_print(shell, "\tAC_ON");
			break;
		case DC_ON_TRIGGER_CAUSE:
			shell_print(shell, "\tDC_ON_DETECTED");
			break;
		case TEMPERATURE_TRIGGER_CAUSE:
			shell_print(shell, "\tTEMPERATURE_TRIGGER");
			uint8_t temp_sensor_num = log.err_code & 0xFF;
			//find name in temp_index_table
			for (int i = 0; i < TEMP_INDEX_MAX; i++) {
				if (temp_sensor_num == temp_index_table[i].sensor_id) {
					shell_print(shell, "\t\t%s",
						    temp_index_table[i].sensor_name);
					break;
				}
			}
			break;
		default:
			shell_print(shell, "Unknown error type: %d", err_type);
			break;
		}

		shell_print(shell, "sys_time: %lld ms", log.sys_time);
		shell_print(shell, "error_data:");
		shell_hexdump(shell, log.error_data, sizeof(log.error_data));
		shell_print(shell, "cpld register: start offset 0x%02x",
			    CPLD_REGISTER_1ST_PART_START_OFFSET);
		shell_hexdump(shell, log.cpld_dump, CPLD_REGISTER_1ST_PART_NUM);
		shell_print(
			shell,
			"====================================================================================");
	}

	shell_print(
		shell,
		"=============================       LOG DUMP END       =============================");

	return;
}

void cmd_test_read(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 4) {
		shell_warn(shell, "Help: test log test_read <offset_1> <offset_2> <length>");
		return;
	}

	uint16_t offset = ((strtol(argv[1], NULL, 16)) << 8) | (strtol(argv[2], NULL, 16));
	printf("offset = 0x%04X\n", offset);

	int length = strtol(argv[3], NULL, 16);

	uint8_t log_data[128] = { 0 };
	plat_eeprom_read(offset, log_data, length);
	printf("FRU_LOG_SIZE = %d\n", FRU_LOG_SIZE);

	shell_hexdump(shell, log_data, sizeof(uint8_t) * FRU_LOG_SIZE);

	return;
}

void cmd_log_clear(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 1) {
		shell_warn(shell, "Help: test log log_clear");
		return;
	}

	k_msleep(1000);

	plat_clear_log();
	shell_print(shell, "plat_clear_log finished!");

	return;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_blackbox_cmd, SHELL_CMD(get, NULL, "log_dump", cmd_log_dump),
			       SHELL_CMD(clear, NULL, "log_clear", cmd_log_clear),
			       SHELL_CMD(set_event, NULL, "set_event", cmd_set_event),
			       SHELL_CMD(test_read, NULL, "test_read", cmd_test_read),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(blackbox, &sub_blackbox_cmd, "blackbox get/clear commands", NULL);
