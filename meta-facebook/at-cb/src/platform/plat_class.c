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
#include "hal_i2c.h"
#include "plat_fru.h"
#include "plat_class.h"
#include "common_i2c_mux.h"

LOG_MODULE_REGISTER(plat_class);

#define NUMBER_OF_ADC_CHANNEL 16
#define AST1030_ADC_BASE_ADDR 0x7e6e9000

/* ADC information for each channel
 * offset: register offset
 * shift: data of channel
 */
struct ADC_INFO {
	uint32_t offset;
	uint8_t shift;
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

struct ASIC_CARD_INFO asic_card_info[ASIC_CARD_COUNT] = {
  [0] = { .bus = I2C_BUS7,
    .mux_addr = ASIC_CARD_1_6_MUX_ADDR,
    .mux_channel = PCA9548A_CHANNEL_0,
    .card_status = ASIC_CARD_UNKNOWN_STATUS,
    .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
    .device_channel = PCA9546A_CHANNEL_0,
    .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
    .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [1] = { .bus = I2C_BUS7,
    .mux_addr = ASIC_CARD_1_6_MUX_ADDR,
    .mux_channel = PCA9548A_CHANNEL_1,
    .card_status = ASIC_CARD_UNKNOWN_STATUS,
    .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
    .device_channel = PCA9546A_CHANNEL_0,
    .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
    .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [2] = { .bus = I2C_BUS7,
    .mux_addr = ASIC_CARD_1_6_MUX_ADDR,
    .mux_channel = PCA9548A_CHANNEL_2,
    .card_status = ASIC_CARD_UNKNOWN_STATUS,
    .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
    .device_channel = PCA9546A_CHANNEL_0,
    .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
    .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [3] = { .bus = I2C_BUS7,
    .mux_addr = ASIC_CARD_1_6_MUX_ADDR,
    .mux_channel = PCA9548A_CHANNEL_3,
    .card_status = ASIC_CARD_UNKNOWN_STATUS,
    .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
    .device_channel = PCA9546A_CHANNEL_0,
    .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
    .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [4] = { .bus = I2C_BUS7,
    .mux_addr = ASIC_CARD_1_6_MUX_ADDR,
    .mux_channel = PCA9548A_CHANNEL_4,
    .card_status = ASIC_CARD_UNKNOWN_STATUS,
    .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
    .device_channel = PCA9546A_CHANNEL_0,
    .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
    .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [5] = { .bus = I2C_BUS7,
    .mux_addr = ASIC_CARD_1_6_MUX_ADDR,
    .mux_channel = PCA9548A_CHANNEL_5,
    .card_status = ASIC_CARD_UNKNOWN_STATUS,
    .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
    .device_channel = PCA9546A_CHANNEL_0,
    .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
    .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [6] = { .bus = I2C_BUS8,
    .mux_addr = ASIC_CARD_7_12_MUX_ADDR,
    .mux_channel = PCA9548A_CHANNEL_0,
    .card_status = ASIC_CARD_UNKNOWN_STATUS,
    .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
    .device_channel = PCA9546A_CHANNEL_0,
    .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
    .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [7] = { .bus = I2C_BUS8,
    .mux_addr = ASIC_CARD_7_12_MUX_ADDR,
    .mux_channel = PCA9548A_CHANNEL_1,
    .card_status = ASIC_CARD_UNKNOWN_STATUS,
    .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
    .device_channel = PCA9546A_CHANNEL_0,
    .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
    .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [8] = { .bus = I2C_BUS8,
    .mux_addr = ASIC_CARD_7_12_MUX_ADDR,
    .mux_channel = PCA9548A_CHANNEL_2,
    .card_status = ASIC_CARD_UNKNOWN_STATUS,
    .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
    .device_channel = PCA9546A_CHANNEL_0,
    .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
    .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [9] = { .bus = I2C_BUS8,
    .mux_addr = ASIC_CARD_7_12_MUX_ADDR,
    .mux_channel = PCA9548A_CHANNEL_3,
    .card_status = ASIC_CARD_UNKNOWN_STATUS,
    .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
    .device_channel = PCA9546A_CHANNEL_0,
    .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
    .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [10] = { .bus = I2C_BUS8,
     .mux_addr = ASIC_CARD_7_12_MUX_ADDR,
     .mux_channel = PCA9548A_CHANNEL_4,
     .card_status = ASIC_CARD_UNKNOWN_STATUS,
     .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
     .device_channel = PCA9546A_CHANNEL_0,
     .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
     .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [11] = { .bus = I2C_BUS8,
     .mux_addr = ASIC_CARD_7_12_MUX_ADDR,
     .mux_channel = PCA9548A_CHANNEL_5,
     .card_status = ASIC_CARD_UNKNOWN_STATUS,
     .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
     .device_channel = PCA9546A_CHANNEL_0,
     .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
     .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
};

bool get_adc_voltage(int channel, float *voltage)
{
	CHECK_NULL_ARG_WITH_RETURN(voltage, false)

	if (channel >= NUMBER_OF_ADC_CHANNEL) {
		LOG_ERR("Invalid ADC channel-%d", channel);
		return false;
	}

	uint32_t raw_value = 0;
	uint32_t reg_value = 0;
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

void check_asic_card_status()
{
	int ret = 0;
	int index = 0;
	int device_index = 0;
	mux_config i2c_mux = { 0 };
	uint8_t i2c_dev[I2C_BUFF_SIZE] = { 0 };
	uint8_t dev_count = 0;

	for (index = 0; index < ASIC_CARD_COUNT; ++index) {
		memset(&i2c_mux, 0, sizeof(i2c_mux));
		memset(&i2c_dev, 0, sizeof(i2c_dev));

		i2c_mux.bus = asic_card_info[index].bus;
		i2c_mux.target_addr = asic_card_info[index].mux_addr;
		i2c_mux.channel = asic_card_info[index].mux_channel;

		ret = set_mux_channel(i2c_mux);
		if (ret != true) {
			LOG_ERR("Switch ASIC%d mux fail", index);
			continue;
		}

		i2c_scan(i2c_mux.bus, i2c_dev, &dev_count);
		for (device_index = 0; device_index < dev_count; ++device_index) {
			if (i2c_dev[device_index] == (ACCL_FRU_ADDR << 1)) {
				asic_card_info[index].card_status = ASIC_CARD_PRESENT;
				break;
			}
		}

		if (device_index >= dev_count) {
			asic_card_info[index].card_status = ASIC_CARD_NOT_PRESENT;
		}
	}
}
