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
#include <stdlib.h>
#include <string.h>
#include <logging/log.h>

#include "libutil.h"
#include "plat_gpio.h"
#include "plat_i2c.h"
#include "plat_sensor_table.h"
#include "sensor.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_class);

#define NUMBER_OF_ADC_CHANNEL 16
#define AST1030_ADC_BASE_ADDR 0x7e6e9000

static uint8_t vr_type = VR_UNKNOWN;
static uint8_t power_monitor_ic_type = POWER_IC_UNKNOWN;
static uint8_t hsc_type = HSC_UNKNOWN;

/* ADC information for each channel
 * offset: register offset
 * shift: data of channel
 */
struct ADC_INFO {
	long unsigned int offset;
	int shift;
};

struct ADC_INFO adc_info[NUMBER_OF_ADC_CHANNEL] = {
	{ 0x10, 0 },  { 0x10, 16 },  { 0x14, 0 },  { 0x14, 16 },  { 0x18, 0 },	{ 0x18, 16 },
	{ 0x1C, 0 },  { 0x1C, 16 },  { 0x110, 0 }, { 0x110, 16 }, { 0x114, 0 }, { 0x114, 16 },
	{ 0x118, 0 }, { 0x118, 16 }, { 0x11C, 0 }, { 0x11C, 16 }
};

enum ADC_REF_VOL_SELECTION {
	REF_VOL_2_5V = 0x0, // 2.5V reference voltage selection
	REF_VOL_1_2V = 0x40 // 1.2V reference voltage selection
};

bool get_adc_voltage(int channel, float *voltage)
{
	CHECK_NULL_ARG_WITH_RETURN(voltage, false)

	if (channel >= NUMBER_OF_ADC_CHANNEL) {
		LOG_ERR("Invalid ADC channel-%d", channel);
		return false;
	}

	uint32_t raw_value, reg_value;
	float reference_voltage = 0.0f;

	/* Get ADC reference voltage from Aspeed chip
	 * ADC000: Engine Control
	 * [7:6] Reference Voltage Selection
	 * 00b - 2.5V / 01b - 1.2V / 10b and 11b - External Voltage
	 */
	reg_value = sys_read32(AST1030_ADC_BASE_ADDR);
	switch (reg_value & (BIT(7) | BIT(6))) {
	case REF_VOL_2_5V:
		reference_voltage = 2.5;
		break;
	case REF_VOL_1_2V:
		reference_voltage = 1.2;
		break;
	default:
		LOG_ERR("Unsupported external reference voltage");
		return false;
	}

	// Read ADC raw value
	reg_value = sys_read32(AST1030_ADC_BASE_ADDR + adc_info[channel].offset);
	raw_value =
		(reg_value >> adc_info[channel].shift) & BIT_MASK(10); // 10-bit(0x3FF) resolution

	// Real voltage = raw data * reference voltage / 2 ^ resolution(10)
	*voltage = (raw_value * reference_voltage) / 1024;

	return true;
}

GT_STAGE_REVISION_ID get_stage_by_rev_id()
{
	return (gpio_get(REV_ID0) | (gpio_get(REV_ID1) << 1) | (gpio_get(REV_ID2) << 2) |
		(gpio_get(BOARD_ID0) << 3));
}

uint8_t get_hsc_type()
{
	return hsc_type;
}

uint8_t get_vr_type()
{
	return vr_type;
}

uint8_t get_power_moniter_ic_type()
{
	return power_monitor_ic_type;
}

/**
 * Follow the schematic diagram, the HSC device can be distinguished by ADC11 (HSC_TYPE_ADC_R).
 * 0.0V(+/- 15%), the hotswap model is MP5990.
 * 1.0V(+/- 15%), the hotswap model is LTC4282.
 * 1.5V(+/- 15%), the hotswap model is LTC4286.
 */
void init_platform_hsc_config()
{
	float voltage_hsc_type_adc = 0;

	if (!get_adc_voltage(HSC_TYPE_ADC_CHANNEL, &voltage_hsc_type_adc)) {
		LOG_ERR("Failed to get hsc type by ADC");
		return;
	}

	if (voltage_hsc_type_adc < 0.15) {
		LOG_INF("The HSC is MP5990, (%.3f V)", voltage_hsc_type_adc);
		hsc_type = HSC_MP5990;
	} else if ((voltage_hsc_type_adc > 1.0 - (1.0 * 0.15)) &&
		   (voltage_hsc_type_adc < 1.0 + (1.0 * 0.15))) {
		LOG_INF("The HSC is LTC4282, (%.3f V)", voltage_hsc_type_adc);
		hsc_type = HSC_LTC4282;
	} else if ((voltage_hsc_type_adc > 1.5 - (1.5 * 0.15)) &&
		   (voltage_hsc_type_adc < 1.5 + (1.5 * 0.15))) {
		LOG_INF("The HSC is LTC4286, (%.3f V)", voltage_hsc_type_adc);
		hsc_type = HSC_LTC4286;
	} else {
		LOG_ERR("Unknown hotswap model type, (%.3f V)", voltage_hsc_type_adc);
	}
}

/**
 * Follow the schematic diagram, the VR chip can be distinguished by ADC12 (VR_TYPE_ADC_R).
 * 0.0V(+/- 15%), the VR chip is ISL69259.
 * 0.5V(+/- 15%), the VR chip is XDPE12284.
 */
void init_platform_vr_config()
{
	float voltage_vr_type_adc = 0;

	if (!get_adc_voltage(VR_TYPE_ADC_CHANNEL, &voltage_vr_type_adc)) {
		LOG_ERR("Failed to get VR type by ADC");
		return;
	}

	if (voltage_vr_type_adc < 0.15) {
		LOG_INF("The VR is RENESAS ISL69259, (%.3f V)", voltage_vr_type_adc);
		vr_type = VR_RNS_ISL69259;
	} else if ((voltage_vr_type_adc > 0.5 - (0.5 * 0.15)) &&
		   (voltage_vr_type_adc < 0.5 + (0.5 * 0.15))) {
		LOG_INF("The VR is INFINEON XDPE12284, (%.3f V)", voltage_vr_type_adc);
		vr_type = VR_INF_XDPE12284;
	} else if ((voltage_vr_type_adc > 1 - (1 * 0.15)) &&
		   (voltage_vr_type_adc < 1 + (1 * 0.15))) {
		LOG_INF("The VR is MPS MP2971, (%.3f V)", voltage_vr_type_adc);
		vr_type = VR_MPS_MPS2971;
	} else {
		LOG_ERR("Unknown VR type, (%.3f V)", voltage_vr_type_adc);
	}
}

/**
 * Follow the schematic diagram, the power monitor ic can be distinguished by ADC13 (ADC_TYPE_ADC_R).
 * 0.0V(+/- 15%), the power monitor ic is ISL28022.
 * 0.5V(+/- 15%), the power monitor ic is INA230.
 */
void init_platform_power_ic_config()
{
	float voltage_power_ic_type_adc = 0;

	if (!get_adc_voltage(POWER_IC_TYPE_ADC_CHANNEL, &voltage_power_ic_type_adc)) {
		LOG_ERR("Failed to get power monitor IC type by ADC");
		return;
	}

	if (voltage_power_ic_type_adc < 0.15) {
		LOG_INF("The power monitor IC is RENESAS ISL28022, (%.3f V)",
			voltage_power_ic_type_adc);
		power_monitor_ic_type = POWER_IC_ISL28022;
	} else if ((voltage_power_ic_type_adc > 0.5 - (0.5 * 0.15)) &&
		   (voltage_power_ic_type_adc < 0.5 + (0.5 * 0.15))) {
		LOG_INF("The power monitor IC is TI INA230, (%.3f V)", voltage_power_ic_type_adc);
		power_monitor_ic_type = POWER_IC_INA230;
	} else {
		LOG_ERR("Unknown power monitor IC type, (%.3f V)", voltage_power_ic_type_adc);
	}
}

void init_platform_config()
{
	init_platform_hsc_config();
	init_platform_vr_config();
	init_platform_power_ic_config();
}