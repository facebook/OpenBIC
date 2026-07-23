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

struct blade_config_mapping_table blade_config_table[] = {
	{ 1.5, BLADE_CONFIG_VOLTAGE_RANGE, BLADE_CONFIG_with_ASIC },
	{ 1.25, BLADE_CONFIG_VOLTAGE_RANGE, BLADE_CONFIG_UNKNOWN },
	{ 1.0, BLADE_CONFIG_VOLTAGE_RANGE, BLADE_CONFIG_UNKNOWN },
	{ 0.75, BLADE_CONFIG_VOLTAGE_RANGE, BLADE_CONFIG_without_ASIC },
	{ 0.5, BLADE_CONFIG_VOLTAGE_RANGE, BLADE_CONFIG_UNKNOWN },
	{ 0.0, BLADE_CONFIG_VOLTAGE_RANGE, BLADE_CONFIG_UNKNOWN },
};

struct board_rev_mappting_table board_rev_table[] = {
	{ 1.25, 0.05, BOARD_POC }, // range: +-5%
	{ 0.75, 0.05, BOARD_POC2 }, // range: +-5%
	{ 1.5, 0.05, BOARD_EVT }, // range: +-5%
	{ 1.0, 0.1, BOARD_DVT }, // range: +-10%
	{ 0.5, 0.05, BOARD_PVT }, // range: +-5%
	{ 0.0, 0.05, BOARD_MP }, // range: +-5%
};

static uint8_t board_revision = UNKNOWN;
static uint8_t blade_configuration = BLADE_CONFIG_UNKNOWN;

static uint32_t adc_get_sample_period_us(void)
{
	uint32_t clk_ctrl = sys_read32(AST1030_ADC_BASE_ADDR + 0x00C); // ADC00C
	uint32_t divisor = clk_ctrl & 0xFFFF; // [15:0]

	// Period(ADC clock) = PCLK_period * 2 * (divisor+1)
	// Sample period = Period(ADC clock) * 12
	uint64_t adc_clk_period_ns = (uint64_t)ADC_PCLK_NS * 2 * (divisor + 1);
	uint64_t sample_period_ns = adc_clk_period_ns * 12;

	return (uint32_t)(sample_period_ns / 1000) + 1; // convert to us, round up
}

bool get_adc_voltage(int channel, float *voltage)
{
	CHECK_NULL_ARG_WITH_RETURN(voltage, false);

	if (channel >= NUMBER_OF_ADC_CHANNEL) {
		LOG_ERR("Invalid ADC channel-%d", channel);
		return false;
	}

	uint32_t reg_value;
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

	uint32_t sample_period_us = adc_get_sample_period_us();

	// Settle: wait at least 2 full sample periods before first read
	k_usleep(sample_period_us * 2);

	uint32_t sum_raw = 0;

	for (int i = 0; i < ADC_SAMPLE_COUNT; i++) {
		reg_value = sys_read32(AST1030_ADC_BASE_ADDR + adc[channel].offset);
		uint32_t raw_value = (reg_value >> adc[channel].shift) & 0x3FF;
		sum_raw += raw_value;

		// Wait for a genuinely new conversion before next read
		k_usleep(sample_period_us);
	}

	float avg_raw = (float)sum_raw / ADC_SAMPLE_COUNT;
	*voltage = (avg_raw * reference_voltage) / 1024.0f;

	return true;
}

bool get_blade_config(uint8_t *blade_config)
{
	/* Determine blade configuration using ADC0 voltage
	 * Since the BMC also have FRU acessibility,
	 * avoid reading FRU to prevent unexpected behavior.
	 */

	CHECK_NULL_ARG_WITH_RETURN(blade_config, false);

	float voltage = 0.0f;

	if (get_adc_voltage(CHANNEL_0, &voltage) == false) {
		LOG_ERR("Failed to get blade config");
		*blade_config = BLADE_CONFIG_UNKNOWN;
		return false;
	}

	*blade_config = BLADE_CONFIG_UNKNOWN;

	for (int i = 0; i < ARRAY_SIZE(blade_config_table); i++) {
		float typical_voltage = blade_config_table[i].voltage;
		float range_val = blade_config_table[i].range_val;

		if ((voltage <= typical_voltage + range_val) &&
		    (voltage >= typical_voltage - range_val)) {
			*blade_config = blade_config_table[i].blade_config;
			LOG_INF("Blade config: 0x%02x, voltage: %dmV", *blade_config,
				(int)(voltage * 1000));
			return true;
		}
	}

	LOG_ERR("Unknown blade config voltage: %dmV", (int)(voltage * 1000));
	return false;
}

uint8_t get_board_revision()
{
	return board_revision;
}

uint8_t get_blade_configuration()
{
	return blade_configuration;
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

	ret = get_blade_config(&blade_configuration);
	if (!ret) {
		LOG_ERR("Failed to get blade configuration");
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
			LOG_INF("Board stage %d", board_revision);
			break;
		}
	}

	return 0;
}
