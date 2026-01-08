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
#include "plat_i2c.h"
#include "plat_cpld.h"
#include "plat_adc.h"
#include "plat_power_capping.h"
#include "plat_pldm_sensor.h"
#include "shell_adc.h"

typedef struct {
	const char *name;
	uint8_t vr_idx;
	uint8_t lv;
	uint8_t adc_mapping_idx;
} power_capping_item_t;

static const power_capping_item_t power_capping_item_list[CAPPING_VR_IDX_MAX * CAPPING_LV_IDX_MAX] = {
	{ "MEDHA0_LV1", CAPPING_VR_IDX_MEDHA0, CAPPING_LV_IDX_LV1, 0xFF },
	{ "MEDHA0_LV2", CAPPING_VR_IDX_MEDHA0, CAPPING_LV_IDX_LV2, ADC_IDX_MEDHA0_1 },
	{ "MEDHA0_LV3", CAPPING_VR_IDX_MEDHA0, CAPPING_LV_IDX_LV3, ADC_IDX_MEDHA0_2 },
	{ "MEDHA1_LV1", CAPPING_VR_IDX_MEDHA1, CAPPING_LV_IDX_LV1, 0xFF },
	{ "MEDHA1_LV2", CAPPING_VR_IDX_MEDHA1, CAPPING_LV_IDX_LV2, ADC_IDX_MEDHA1_1 },
	{ "MEDHA1_LV3", CAPPING_VR_IDX_MEDHA1, CAPPING_LV_IDX_LV3, ADC_IDX_MEDHA1_2 },
};

static int cmd_power_capping_get_all(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t capping_source = get_power_capping_source();
	const char *volt_str = "MEDHA0_VDD_VOLT, MEDHA1_VDD_VOLT";
	const char *pwr_str = "MEDHA0_VDD_PWR, MEDHA1_VDD_PWR";
	const char *print_str = NULL;
	print_str = (capping_source == CAPPING_SOURCE_VR) ? pwr_str : volt_str;

	shell_print(shell, "==================Power Capping Info=================");
	shell_print(shell, "Method: %d ( 0:LOOK_UP_TABLE,  1:CREDIT_BASE )",
		    get_power_capping_method());
	shell_print(shell, "Source: %d ( 0:VR,  1:ADC )\n", capping_source);
	shell_print(shell, "VR Polling Telemetry: %s", print_str);
	shell_print(shell, "VR Polling Rate: %11d (ms)\n",
		    plat_pldm_sensor_get_quick_vr_poll_interval());
	shell_print(shell, "---------------------MEDHA0---------------------");
	shell_print(shell, "MEDHA0 LV1 time window: %4d (us)",
		    get_power_capping_time_w(CAPPING_VR_IDX_MEDHA0, CAPPING_LV_IDX_LV1));
	shell_print(shell, "MEDHA0 LV1 threshold: %6d (W), %4d (A)\n",
		    get_power_capping_threshold(CAPPING_VR_IDX_MEDHA0, CAPPING_LV_IDX_LV1),
		    get_power_capping_current_threshold(CAPPING_VR_IDX_MEDHA0));
	shell_print(shell, "MEDHA0 LV2 time window: %4d (ms)",
		    get_power_capping_time_w(CAPPING_VR_IDX_MEDHA0, CAPPING_LV_IDX_LV2));
	shell_print(shell, "MEDHA0 LV2 threshold: %6d (W)",
		    get_power_capping_threshold(CAPPING_VR_IDX_MEDHA0, CAPPING_LV_IDX_LV2));
	shell_print(shell, "MEDHA0 LV2 avg_power: %6d (W)\n",
		    get_power_capping_avg_power(CAPPING_VR_IDX_MEDHA0, CAPPING_LV_IDX_LV2));
	shell_print(shell, "MEDHA0 LV3 time window: %4d (ms)",
		    get_power_capping_time_w(CAPPING_VR_IDX_MEDHA0, CAPPING_LV_IDX_LV3));
	shell_print(shell, "MEDHA0 LV3 threshold: %6d (W)",
		    get_power_capping_threshold(CAPPING_VR_IDX_MEDHA0, CAPPING_LV_IDX_LV3));
	shell_print(shell, "MEDHA0 LV3 avg_power: %6d (W)\n",
		    get_power_capping_avg_power(CAPPING_VR_IDX_MEDHA0, CAPPING_LV_IDX_LV3));
	shell_print(shell, "---------------------MEDHA1---------------------");
	shell_print(shell, "MEDHA1 LV1 time window: %4d (us)",
		    get_power_capping_time_w(CAPPING_VR_IDX_MEDHA1, CAPPING_LV_IDX_LV1));
	shell_print(shell, "MEDHA1 LV1 threshold: %6d (W), %4d (A)\n",
		    get_power_capping_threshold(CAPPING_VR_IDX_MEDHA1, CAPPING_LV_IDX_LV1),
		    get_power_capping_current_threshold(CAPPING_VR_IDX_MEDHA1));
	shell_print(shell, "MEDHA1 LV2 time window: %4d (ms)",
		    get_power_capping_time_w(CAPPING_VR_IDX_MEDHA1, CAPPING_LV_IDX_LV2));
	shell_print(shell, "MEDHA1 LV2 threshold: %6d (W)",
		    get_power_capping_threshold(CAPPING_VR_IDX_MEDHA1, CAPPING_LV_IDX_LV2));
	shell_print(shell, "MEDHA1 LV2 avg_power: %6d (W)\n",
		    get_power_capping_avg_power(CAPPING_VR_IDX_MEDHA1, CAPPING_LV_IDX_LV2));
	shell_print(shell, "MEDHA1 LV3 time window: %4d (ms)",
		    get_power_capping_time_w(CAPPING_VR_IDX_MEDHA1, CAPPING_LV_IDX_LV3));
	shell_print(shell, "MEDHA1 LV3 threshold: %6d (W)",
		    get_power_capping_threshold(CAPPING_VR_IDX_MEDHA1, CAPPING_LV_IDX_LV3));
	shell_print(shell, "MEDHA1 LV3 avg_power: %6d (W)\n",
		    get_power_capping_avg_power(CAPPING_VR_IDX_MEDHA1, CAPPING_LV_IDX_LV3));
	return 0;
}

static int cmd_power_capping_set_method(const struct shell *shell, size_t argc, char **argv)
{
	if (!strcmp(argv[0], "LOOK_UP_TABLE")) {
		shell_print(shell, "Method set to LOOK_UP_TABLE");
		set_power_capping_method(CAPPING_M_LOOK_UP_TABLE);
	} else if (!strcmp(argv[0], "CREDIT_BASE")) {
		shell_print(shell, "Method set to CREDIT_BASE");
		set_power_capping_method(CAPPING_M_CREDIT_BASE);
	} else {
		shell_error(shell, "method should be <LOOK_UP_TABLE | CREDIT_BASE>");
		return -1;
	}

	return 0;
}

static int cmd_power_capping_set_source(const struct shell *shell, size_t argc, char **argv)
{
	if (!strcmp(argv[0], "VR")) {
		shell_print(shell, "Source set to VR");
		set_power_capping_source(CAPPING_SOURCE_VR);
	} else if (!strcmp(argv[0], "ADC")) {
		shell_print(shell, "Source set to ADC");
		set_power_capping_source(CAPPING_SOURCE_ADC);
	} else {
		shell_error(shell, "source should be <VR | ADC>");
		return -1;
	}

	return 0;
}

static int cmd_power_capping_set_time_window(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t idx = 0xff;
	for (uint8_t id = 0; id < ARRAY_SIZE(power_capping_item_list); id++) {
		if (!strcmp(argv[1], power_capping_item_list[id].name)) {
			idx = id;
			break;
		}
	}

	if (idx == 0xff) {
		shell_error(shell, "Here is the <MEDHA[X]_LV[Y]> list:");
		for (uint8_t i = 0; i < ARRAY_SIZE(power_capping_item_list); i++) {
			shell_print(shell, "%s", power_capping_item_list[i].name);
		}
		return -1;
	}

	uint16_t value = strtol(argv[2], NULL, 10);
	if (value < ADC_AVERGE_TIMES_MIN && value > ADC_AVERGE_TIMES_MAX) {
		shell_error(shell, "time should be less equal to %d", ADC_AVERGE_TIMES_MAX);
		return -1;
	}

	if (power_capping_item_list[idx].lv == CAPPING_LV_IDX_LV1) {
		uint8_t tmp_idx = 0;
		if (!find_cpld_lv1_time_window_idx_by_value(&tmp_idx, value)) {
			shell_error(shell, "For LV1, the time(us) should be:");
			shell_print(shell, "{ 0, 1, 3, 5, 10, 15, 20, 50 }");
			return -1;
		}
	}

	set_power_capping_time_w(power_capping_item_list[idx].vr_idx,
				 power_capping_item_list[idx].lv, value);
	shell_print(shell, "set time_w %s to %d", power_capping_item_list[idx].name, value);

	return 0;
}

static int cmd_power_capping_set_threshold(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t idx = 0xff;
	for (uint8_t id = 0; id < ARRAY_SIZE(power_capping_item_list); id++) {
		if (!strcmp(argv[1], power_capping_item_list[id].name)) {
			idx = id;
			break;
		}
	}

	if (idx == 0xff) {
		shell_error(shell, "Here is the <MEDHA[X]_LV[Y]> list:");
		for (uint8_t i = 0; i < ARRAY_SIZE(power_capping_item_list); i++) {
			shell_print(shell, "%s", power_capping_item_list[i].name);
		}
		return -1;
	}

	uint16_t value = strtol(argv[2], NULL, 10);
	set_power_capping_threshold(power_capping_item_list[idx].vr_idx,
				    power_capping_item_list[idx].lv, value);
	shell_print(shell, "set threshold %s to %d", power_capping_item_list[idx].name, value);

	return 0;
}

static int cmd_power_capping_set_polling_rate(const struct shell *shell, size_t argc, char **argv)
{
	uint32_t poll_rate = strtoul(argv[1], NULL, 10);

	plat_pldm_sensor_set_quick_vr_poll_interval(poll_rate);
	shell_print(shell, "set polling rate to %d", poll_rate);

	return 0;
}

static int cmd_power_capping_debug(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t idx = 0xff;
	for (uint8_t id = 0; id < ARRAY_SIZE(power_capping_item_list); id++) {
		if (!strcmp(argv[1], power_capping_item_list[id].name)) {
			if (power_capping_item_list[id].adc_mapping_idx != 0xFF) {
				idx = id;
				break;
			} else {
				shell_error(shell, "Not support LV1");
				return -1;
			}
		}
	}

	if (idx == 0xff) {
		shell_error(shell, "Should input: <MEDHA[X]_LV[Y]>");
		return -1;
	}

	if (get_power_capping_source() == CAPPING_SOURCE_ADC) {
		shell_adc_get_averge_val(shell, power_capping_item_list[idx].adc_mapping_idx);
		shell_adc_get_buf(shell, power_capping_item_list[idx].adc_mapping_idx);
	} else if (get_power_capping_source() == CAPPING_SOURCE_VR) {
		uint16_t avg_pwr = get_adc_averge_val(power_capping_item_list[idx].adc_mapping_idx);
		shell_warn(shell, "%s VR PWR:  %d(W)", power_capping_item_list[idx].name, avg_pwr);
		shell_print(shell, "");

		uint16_t *buf = get_adc_buf(power_capping_item_list[idx].adc_mapping_idx);
		uint16_t len = get_power_capping_time_w(power_capping_item_list[idx].vr_idx,
							power_capping_item_list[idx].lv);
		shell_warn(shell, "%s VR PWR buf_raw (len=%d)(unit=W): ",
			   power_capping_item_list[idx].name, len);
		for (uint16_t i = 0; i < len; i++) {
			shell_fprintf(shell, SHELL_NORMAL, "%5d ", buf[i]);
			if ((i + 1) % 10 == 0) {
				shell_print(shell, "");
			}
		}
		shell_print(shell, "");
	}

	return 0;
}

static void power_capping_name_get_for_set_cmd(size_t idx, struct shell_static_entry *entry)
{
	entry->syntax = NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;

	if (idx < ARRAY_SIZE(power_capping_item_list)) {
		entry->syntax = power_capping_item_list[idx].name;
	}
}

SHELL_DYNAMIC_CMD_CREATE(power_capping_name, power_capping_name_get_for_set_cmd);

/* level 3 */
SHELL_STATIC_SUBCMD_SET_CREATE(
	set_method_subcmds,
	SHELL_CMD_ARG(LOOK_UP_TABLE, NULL, "power_capping set method <LOOK_UP_TABLE | CREDIT_BASE>",
		      cmd_power_capping_set_method, 1, 0),
	SHELL_CMD_ARG(CREDIT_BASE, NULL, "power_capping set method <LOOK_UP_TABLE | CREDIT_BASE>",
		      cmd_power_capping_set_method, 1, 0),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(set_source_subcmds,
			       SHELL_CMD_ARG(VR, NULL, "power_capping set source <VR | ADC>",
					     cmd_power_capping_set_source, 1, 0),
			       SHELL_CMD_ARG(ADC, NULL, "power_capping set source <VR | ADC>",
					     cmd_power_capping_set_source, 1, 0),
			       SHELL_SUBCMD_SET_END);

/* level 2 */
SHELL_STATIC_SUBCMD_SET_CREATE(get_subcmds,
			       SHELL_CMD(all, NULL, "power_capping get all",
					 cmd_power_capping_get_all),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	set_subcmds,
	SHELL_CMD_ARG(method, &set_method_subcmds,
		      "power_capping set method <LOOK_UP_TABLE | CREDIT_BASE>", NULL, 2, 0),
	SHELL_CMD_ARG(source, &set_source_subcmds, "power_capping set source <VR | ADC>", NULL, 2,
		      0),
	SHELL_CMD_ARG(time_window, &power_capping_name,
		      "power_capping set time_window <MEDHA[X]_LV[Y]> <time>",
		      cmd_power_capping_set_time_window, 3, 0),
	SHELL_CMD_ARG(threshold, &power_capping_name,
		      "power_capping set threshold <MEDHA[X]_LV[Y]> <threshold>",
		      cmd_power_capping_set_threshold, 3, 0),
	SHELL_CMD_ARG(polling_rate, NULL, "power_capping set polling_rate <time (ms)>",
		      cmd_power_capping_set_polling_rate, 2, 0),
	SHELL_SUBCMD_SET_END);

/* level 1 */
SHELL_STATIC_SUBCMD_SET_CREATE(
	power_capping_subcmds, SHELL_CMD(get, &get_subcmds, "power_capping get all", NULL),
	SHELL_CMD(set, &set_subcmds,
		  "power_capping set <method | source | time_window | threshold | polling_rate>",
		  NULL),
	SHELL_CMD_ARG(debug, &power_capping_name, "power_capping debug <MEDHA[X]_LV[Y]>",
		      cmd_power_capping_debug, 2, 0),
	SHELL_SUBCMD_SET_END);
/* Root level */
SHELL_CMD_REGISTER(
	power_capping, &power_capping_subcmds,
	"power_capping get all | power_capping set <method | source | time_window | threshold | polling_rate>",
	NULL);
