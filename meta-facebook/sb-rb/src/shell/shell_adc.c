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

#include "plat_adc.h"

void cmd_adc_poll_get(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t flag = 0;
	flag = adc_get_poll_flag() ? 1 : 0;

	shell_warn(shell, "adc polling %s", flag ? "enable" : "disable");
}
void cmd_adc_poll_set(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t onoff = strtoul(argv[1], NULL, 10);

	adc_set_poll_flag(onoff);

	shell_warn(shell, "set adc polling %s", onoff ? "enable" : "disable");
}

void cmd_adc_get_averge_times(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t idx = strtoul(argv[1], NULL, 10);
	if (idx >= ADC_IDX_MAX) {
		shell_warn(shell, "adc invalid idx %d", idx);
		return;
	}

	shell_warn(shell, "adc %d averge time %d", idx, get_adc_averge_times(idx));
}
void cmd_adc_set_averge_times(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t idx = strtoul(argv[1], NULL, 10);
	uint16_t time = strtoul(argv[2], NULL, 10);

	if (idx >= ADC_IDX_MAX) {
		shell_warn(shell, "adc invalid idx %d", idx);
		return;
	}

	if (time < ADC_AVERGE_TIMES_MIN && time > ADC_AVERGE_TIMES_MAX) {
		shell_warn(shell, "adc invalid averge time %d", time);
		return;
	}

	adc_set_averge_times(idx, time);

	shell_warn(shell, "set adc %d averge time to %d", idx, time);
}

void cmd_adc_get_averge_val(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t idx = strtoul(argv[1], NULL, 10);
	if (idx >= ADC_IDX_MAX) {
		shell_warn(shell, "adc invalid idx %d", idx);
		return;
	}

	// real val(A) = raw data(mV) * 2 * mV_to_A
	shell_warn(shell, "adc %d val: %f(A)", idx, adc_raw_mv_to_apms(get_adc_averge_val(idx)));
}

void cmd_adc_get_buf_raw(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t idx = strtoul(argv[1], NULL, 10);
	if (idx >= ADC_IDX_MAX) {
		shell_warn(shell, "adc invalid idx %d", idx);
		return;
	}

	uint8_t len = get_adc_averge_times(idx);
	uint16_t *buf = get_adc_buf(idx);
	shell_warn(shell, "adc %d buf(len=%d)(unit=V): ", idx, len);
	for (uint8_t i = 0; i < len; i++) {
		shell_fprintf(shell, SHELL_NORMAL, "%04d ", buf[i]);
		if ((i + 1) % 16 == 0) {
			shell_print(shell, "");
		}
	}

	if (len % 16 != 0) {
		shell_print(shell, "");
	}
}
void cmd_adc_get_buf(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t idx = strtoul(argv[1], NULL, 10);
	if (idx >= ADC_IDX_MAX) {
		shell_warn(shell, "adc invalid idx %d", idx);
		return;
	}

	uint8_t len = get_adc_averge_times(idx);
	uint16_t *buf = get_adc_buf(idx);
	shell_warn(shell, "adc %d buf(len=%d)(unit=A)): ", idx, len);
	for (uint8_t i = 0; i < len; i++) {
		shell_fprintf(shell, SHELL_NORMAL, "%f ", adc_raw_mv_to_apms(buf[i]));
		if ((i + 1) % 10 == 0) {
			shell_print(shell, "");
		}
	}

	if (len % 10 != 0) {
		shell_print(shell, "");
	}
}

void cmd_adc_get_ucr(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t idx = strtoul(argv[1], NULL, 10);
	if (idx >= ADC_IDX_MAX) {
		shell_warn(shell, "adc invalid idx %d", idx);
		return;
	}

	shell_warn(shell, "adc %d ucr %d(%s)", idx, get_adc_ucr(idx),
		   (get_adc_ucr_status(idx) ? "ucr" : "normal"));
}
void cmd_adc_set_ucr(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t idx = strtoul(argv[1], NULL, 10);
	uint16_t ucr = strtoul(argv[2], NULL, 10);
	if (idx >= ADC_IDX_MAX) {
		shell_warn(shell, "adc invalid idx %d", idx);
		return;
	}

	set_adc_ucr(idx, ucr);
	shell_warn(shell, "set adc %d ucr to %d", idx, ucr);
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_adc_poll_cmds,
			       SHELL_CMD(get, NULL, "adc polling get", cmd_adc_poll_get),
			       SHELL_CMD(set, NULL, "adc polling set", cmd_adc_poll_set),
			       SHELL_SUBCMD_SET_END);
SHELL_STATIC_SUBCMD_SET_CREATE(sub_adc_averge_times_cmds,
			       SHELL_CMD(get, NULL, "get adc averge times",
					 cmd_adc_get_averge_times),
			       SHELL_CMD(set, NULL, "set adc averge times",
					 cmd_adc_set_averge_times),
			       SHELL_SUBCMD_SET_END);
SHELL_STATIC_SUBCMD_SET_CREATE(sub_adc_ucr_cmds,
			       SHELL_CMD(get, NULL, "get adc ucr val", cmd_adc_get_ucr),
			       SHELL_CMD(set, NULL, "set adc ucr val", cmd_adc_set_ucr),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_adc_test_cmds, SHELL_CMD(polling, &sub_adc_poll_cmds, "adc polling cmds", NULL),
	SHELL_CMD(averge_times, &sub_adc_averge_times_cmds, "adc averge times cmds", NULL),
	SHELL_CMD(val, NULL, "get adc averge val", cmd_adc_get_averge_val),
	SHELL_CMD(buf_raw, NULL, "get adc buf raw data", cmd_adc_get_buf_raw),
	SHELL_CMD(buf, NULL, "get adc buf", cmd_adc_get_buf),
	SHELL_CMD(ucr, &sub_adc_ucr_cmds, "adc ucr cmds", NULL), SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(adc_test, &sub_adc_test_cmds, "adc test commands", NULL);
