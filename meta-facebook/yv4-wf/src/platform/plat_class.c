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
#include <stdio.h>
#include <logging/log.h>
#include "libutil.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_class);

struct adc_info adc[NUMBER_OF_ADC_CHANNEL] = {
	{ 0x10, 0 },  { 0x10, 16 },  { 0x14, 0 },  { 0x14, 16 },  { 0x18, 0 },	{ 0x18, 16 },
	{ 0x1C, 0 },  { 0x1C, 16 },  { 0x110, 0 }, { 0x110, 16 }, { 0x114, 0 }, { 0x114, 16 },
	{ 0x118, 0 }, { 0x118, 16 }, { 0x11C, 0 }, { 0x11C, 16 }
};

struct board_rev_mappting_table board_rev_table[] = {
	{ 1.25, 0.05, BOARD_POC }, // range: +-5%
	{ 1.5, 0.05, BOARD_EVT }, // range: +-5%
};

static uint8_t board_revision = UNKNOWN;

bool get_adc_voltage(int channel, float *voltage)
{
	CHECK_NULL_ARG_WITH_RETURN(voltage, false);

	if (channel >= NUMBER_OF_ADC_CHANNEL) {
		LOG_ERR("Invalid ADC channel-%d", channel);
		return false;
	}

	uint32_t raw_value, reg_value;
	long unsigned int engine_control = 0x0;
	float reference_voltage = 0.0f;

	/* Get ADC reference voltage from Aspeed chip
	 * ADC000: Engine Control
	 * [7:6] Reference Voltage Selection
	 * 00b - 2.5V / 01b - 1.2V / 10b and 11b - External Voltage
	 */
	reg_value = sys_read32(AST1030_ADC_BASE_ADDR + engine_control);
	switch (reg_value & (BIT(7) | BIT(6))) {
	case REF_VOL_2_5V:
		reference_voltage = 2.5;
		break;
	case REF_VOL_1_2V:
		reference_voltage = 1.2;
		break;
	default:
		LOG_ERR("unsupported the external reference voltage");
		return false;
	}

	// Read ADC raw value
	reg_value = sys_read32(AST1030_ADC_BASE_ADDR + adc[channel].offset);
	raw_value = (reg_value >> adc[channel].shift) & 0x3FF; // 10-bit(0x3FF) resolution

	// Real voltage = raw data * reference voltage / 2 ^ resolution(10)
	*voltage = (raw_value * reference_voltage) / 1024;

	return true;
}

uint8_t get_board_revision()
{
	return board_revision;
}

int init_platform_config()
{
	// WF BIC judges the board revision according the ADC-1(0-based) voltage.
	float voltage = 0.0;
	bool ret = get_adc_voltage(CHANNEL_1, &voltage);
	if (!ret) {
		LOG_ERR("Failed to get board revision");
		return -1;
	}

	for (int cnt = 0; cnt < ARRAY_SIZE(board_rev_table); cnt++) {
		float typical_voltage = board_rev_table[cnt].voltage;
		float range_val = board_rev_table[cnt].range_val;

		LOG_DBG("stage %d: range %d~%d", cnt,
			(int)(typical_voltage * (1 - range_val) * 1000),
			(int)(typical_voltage * (1 + range_val) * 1000));
		LOG_DBG("voltage %d", (int)voltage * 1000);

		if ((voltage <= typical_voltage * (1 + range_val)) &&
		    (voltage >= typical_voltage * (1 - range_val))) {
			board_revision = board_rev_table[cnt].board_rev;
			break;
		}
	}

	return 0;
}
