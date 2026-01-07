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
#include "plat_power_capping.h"

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

	switch (idx) {
	case ADC_IDX_MEDHA0_1:
		set_power_capping_time_w(CAPPING_VR_IDX_MEDHA0, CAPPING_LV_IDX_LV2, time);
		break;
	case ADC_IDX_MEDHA1_1:
		set_power_capping_time_w(CAPPING_VR_IDX_MEDHA1, CAPPING_LV_IDX_LV2, time);
		break;
	case ADC_IDX_MEDHA0_2:
		set_power_capping_time_w(CAPPING_VR_IDX_MEDHA0, CAPPING_LV_IDX_LV3, time);
		break;
	case ADC_IDX_MEDHA1_2:
		set_power_capping_time_w(CAPPING_VR_IDX_MEDHA1, CAPPING_LV_IDX_LV3, time);
		break;
	default:
		break;
	}

	shell_warn(shell, "set adc %d averge time to %d", idx, time);
}

void shell_adc_get_averge_val(const struct shell *shell, uint8_t idx)
{
	float vref = 0;
	uint8_t adc_type = get_adc_type();
	if (adc_type == ADI_AD4058)
		vref = get_ad4058_vref();
	else if (adc_type == TIC_ADS7066)
		vref = get_ads7066_vref();
	else
		shell_error(shell, "invalid adc type %d", adc_type);
	uint8_t lv = (idx == 0 || idx == 1) ? 2 : 3;

	shell_warn(shell, "LV%d MEDHA%d CURR: %f(A)", lv, idx % 2,
		   adc_raw_mv_to_apms(get_adc_averge_val(idx), vref));

	float pwr = get_adc_vr_pwr(idx);
	shell_warn(shell, "LV%d MEDHA%d PWR:  %f(W)", lv, idx % 2, pwr);

	float vol = get_vr_vol_sum(idx);
	uint16_t len = get_adc_averge_times(idx);
	shell_warn(shell, "LV%d MEDHA%d VOLT: %f(V)", lv, idx % 2, vol / len);
	shell_print(shell, "");
}

void cmd_adc_get_averge_val(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t idx = strtoul(argv[1], NULL, 10);
	if (idx >= ADC_IDX_MAX) {
		shell_warn(shell, "adc invalid idx %d", idx);
		return;
	}

	shell_adc_get_averge_val(shell, idx);
}

void shell_adc_get_buf_raw(const struct shell *shell, uint8_t idx)
{
	uint8_t lv = (idx == 0 || idx == 1) ? 2 : 3;
	uint16_t len = get_adc_averge_times(idx);
	uint16_t *buf = get_adc_buf(idx);
	shell_warn(shell, "LV%d MEDHA%d ADC buf_raw (len=%d)(unit=V): ", lv, idx % 2, len);
	for (uint16_t i = 0; i < len; i++) {
		shell_fprintf(shell, SHELL_NORMAL, "0x%04x ", buf[i]);
		if ((i + 1) % 10 == 0) {
			shell_print(shell, "");
		}
	}
	shell_print(shell, "\n");
	shell_warn(shell, "LV%d MEDHA%d VR Volt buf_raw (len=%d)(unit=V)): ", lv, idx % 2, len);
	uint16_t *vr_buf = get_vr_buf(idx);
	for (uint16_t i = 0; i < len; i++) {
		shell_fprintf(shell, SHELL_NORMAL, "0x%x ", vr_buf[i]);
		if ((i + 1) % 10 == 0) {
			shell_print(shell, "");
		}
	}

	if (len % 10 != 0) {
		shell_print(shell, "");
	}
}

void cmd_adc_get_buf_raw(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t idx = strtoul(argv[1], NULL, 10);
	if (idx >= ADC_IDX_MAX) {
		shell_warn(shell, "adc invalid idx %d", idx);
		return;
	}

	shell_adc_get_buf_raw(shell, idx);
}

void shell_adc_get_buf(const struct shell *shell, uint8_t idx)
{
	uint8_t lv = (idx == 0 || idx == 1) ? 2 : 3;
	uint16_t len = get_adc_averge_times(idx);
	uint16_t *buf = get_adc_buf(idx);
	float vref = 0;
	uint8_t adc_type = get_adc_type();
	if (adc_type == ADI_AD4058)
		vref = get_ad4058_vref();
	else if (adc_type == TIC_ADS7066)
		vref = get_ads7066_vref();
	else
		shell_error(shell, "invalid adc type %d", adc_type);

	shell_warn(shell, "LV%d MEDHA%d CURR (len=%d)(unit=A)): ", lv, idx % 2, len);
	for (uint16_t i = 0; i < len; i++) {
		shell_fprintf(shell, SHELL_NORMAL, "%f ", adc_raw_mv_to_apms(buf[i], vref));
		if ((i + 1) % 10 == 0) {
			shell_print(shell, "");
		}
	}
	shell_print(shell, "");

	shell_warn(shell, "LV%d MEDHA%d PWR (len=%d)(unit=W)): ", lv, idx % 2, len);
	uint16_t *vr_buf = get_vr_buf(idx);
	for (uint16_t i = 0; i < len; i++) {
		// average pwr = average voltage * average current
		// P = V * I
		float pwr_buf = uint16_voltage_transfer_to_float(vr_buf[i]) *
				adc_raw_mv_to_apms(buf[i], vref);
		shell_fprintf(shell, SHELL_NORMAL, "%f ", pwr_buf);
		if ((i + 1) % 10 == 0) {
			shell_print(shell, "");
		}
	}
	shell_print(shell, "");

	shell_warn(shell, "LV%d MEDHA%d VOLT (len=%d)(unit=V)): ", lv, idx % 2, len);
	for (uint16_t i = 0; i < len; i++) {
		float tmp = uint16_voltage_transfer_to_float(vr_buf[i]);
		shell_fprintf(shell, SHELL_NORMAL, "%f ", tmp);
		if ((i + 1) % 10 == 0) {
			shell_print(shell, "");
		}
	}
	shell_print(shell, "");
}

void cmd_adc_get_buf(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t idx = strtoul(argv[1], NULL, 10);
	if (idx >= ADC_IDX_MAX) {
		shell_warn(shell, "adc invalid idx %d", idx);
		return;
	}

	shell_adc_get_buf(shell, idx);
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

	switch (idx) {
	case ADC_IDX_MEDHA0_1:
		set_power_capping_threshold(CAPPING_VR_IDX_MEDHA0, CAPPING_LV_IDX_LV2, ucr);
		break;
	case ADC_IDX_MEDHA1_1:
		set_power_capping_threshold(CAPPING_VR_IDX_MEDHA1, CAPPING_LV_IDX_LV2, ucr);
		break;
	case ADC_IDX_MEDHA0_2:
		set_power_capping_threshold(CAPPING_VR_IDX_MEDHA0, CAPPING_LV_IDX_LV3, ucr);
		break;
	case ADC_IDX_MEDHA1_2:
		set_power_capping_threshold(CAPPING_VR_IDX_MEDHA1, CAPPING_LV_IDX_LV3, ucr);
		break;
	default:
		break;
	}
	shell_warn(shell, "set adc %d ucr to %d", idx, ucr);
}

void cmd_adc_get_good_status(const struct shell *shell, size_t argc, char **argv)
{
	shell_info(shell, "%02x %02x", get_adc_good_status(ADC_RB_IDX_MEDHA0),
		   get_adc_good_status(ADC_RB_IDX_MEDHA1));
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
	SHELL_CMD(get_good_status, NULL, "get adc good status", cmd_adc_get_good_status),
	SHELL_CMD(ucr, &sub_adc_ucr_cmds, "adc ucr cmds", NULL), SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(adc_test, &sub_adc_test_cmds, "adc test commands", NULL);
