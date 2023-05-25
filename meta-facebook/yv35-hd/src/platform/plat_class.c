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

#include "plat_class.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "libutil.h"
#include "plat_gpio.h"
#include "plat_i2c.h"
#include "plat_sensor_table.h"
#include "sensor.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(plat_class);

#define CPLD_ADDR 0x21 // 7-bit address
#define CPLD_CLASS_TYPE_REG 0x05
#define CPLD_2OU_EXPANSION_CARD_REG 0x06
#define CPLD_BOARD_REV_ID_REG 0x08
#define CPLD_1OU_CARD_DETECTION 0x09
#define I2C_DATA_SIZE 5
#define NUMBER_OF_ADC_CHANNEL 16
#define AST1030_ADC_BASE_ADDR 0x7e6e9000

static uint8_t system_class = SYS_CLASS_1;
static uint8_t board_revision = 0x3F;
static uint8_t hsc_module = HSC_MODULE_UNKNOWN;
static CARD_STATUS _1ou_status = { false, TYPE_1OU_UNKNOWN };
static CARD_STATUS _2ou_status = { false, TYPE_2OU_UNKNOWN };

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

struct _1OU_CARD_MAPPING_TABLE {
	float voltage;
	uint8_t condition;
	uint8_t card_type;
};

/* The condition is used for 1OU card mapping table
 * "LOWER" means the voltage is lower than the setting value
 * "HIGHER" means the voltage is higher than the setting value
 * "RANGE" means the voltage is within a range
 */
enum CONDITION {
	LOWER = 0x0,
	HIGHER = 0x01,
	RANGE = 0x02,
};

struct _1OU_CARD_MAPPING_TABLE _1ou_card_mapping_table[] = {
	{ 0.3, LOWER, TYPE_1OU_SI_TEST_CARD },	 { 0.5, RANGE, TYPE_1OU_EXP_WITH_6_M2 },
	{ 0.75, RANGE, TYPE_1OU_RAINBOW_FALLS }, { 1.0, RANGE, TYPE_1OU_VERNAL_FALLS_WITH_TI },
	{ 1.26, RANGE, TYPE_1OU_WAIMANO_FALLS }, { 1.5, RANGE, TYPE_1OU_EXP_WITH_NIC },
	{ 2.0, RANGE, TYPE_1OU_EXP_WITH_E1S },
};

uint8_t get_system_class()
{
	return system_class;
}

CARD_STATUS get_1ou_status()
{
	return _1ou_status;
}

CARD_STATUS get_2ou_status()
{
	return _2ou_status;
}

uint8_t get_board_revision()
{
	return board_revision;
}

uint8_t get_hsc_module()
{
	return hsc_module;
}

bool get_adc_voltage(int channel, float *voltage)
{
	if (!voltage) {
		return false;
	}

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
	raw_value = (reg_value >> adc_info[channel].shift) & 0x3FF; // 10-bit(0x3FF) resolution

	// Real voltage = raw data * reference voltage / 2 ^ resolution(10)
	*voltage = (raw_value * reference_voltage) / 1024;

	return true;
}

void init_platform_config()
{
	I2C_MSG i2c_msg;
	uint8_t retry = 3;

	hsc_module = ((gpio_get(HSC_TYPE_1) << 1) & 0x02) | (gpio_get(HSC_TYPE_0) & 0x01);

	if (hsc_module == HSC_MODULE_MP5990) {
		gpio_set(HSC_OCP_GPIO3_R, GPIO_HIGH);
	}
	/* According hardware design, BIC can check the class type through GPIOs.
	 * The board ID is "0000" if the class type is class1.
	 * The board ID is "0001" if the class type is calss2.
	 */
	if (gpio_get(BOARD_ID0) == GPIO_HIGH) {
		system_class = SYS_CLASS_2;
	} else {
		system_class = SYS_CLASS_1;
	}

	uint8_t tx_len, rx_len;
	uint8_t class_type = 0x0;
	char *data = (uint8_t *)malloc(I2C_DATA_SIZE * sizeof(uint8_t));
	if (data == NULL) {
		LOG_ERR("data allocation failed.");
		return;
	}

	/* Read the expansion present from CPLD's class type register
	 * CPLD Class Type Register(05h)
	 * Bit[7:4] - Board ID(0000b: Class-1, 0001b: Class-2)
	 * Bit[3] - 2ou x8/x16 Riser Expansion Present
	 * Bit[2] - 1ou Expansion Present Pin
	 * Bit[1:0] - Reserved
	 */
	tx_len = 1;
	rx_len = 1;
	memset(data, 0, I2C_DATA_SIZE);
	data[0] = CPLD_CLASS_TYPE_REG;
	i2c_msg = construct_i2c_message(I2C_BUS1, CPLD_ADDR, tx_len, data, rx_len);
	if (!i2c_master_read(&i2c_msg, retry)) {
		class_type = i2c_msg.data[0];
		_1ou_status.present = ((class_type & BIT(2)) ? false : true);
		_2ou_status.present = ((class_type & BIT(3)) ? false : true);
	} else {
		LOG_ERR("Failed to read expansion present from CPLD");
	}
	/* Set the class type to CPLD's class type register(the bit[4] of offset 05h) */
	tx_len = 2;
	rx_len = 0;
	memset(data, 0, I2C_DATA_SIZE);
	data[0] = CPLD_CLASS_TYPE_REG;
	data[1] = (((system_class - 1) << 4) & 0x10) | class_type;
	i2c_msg = construct_i2c_message(I2C_BUS1, CPLD_ADDR, tx_len, data, rx_len);
	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Failed to set class type to CPLD)");
	}

	/* Get the board revision to CPLD's board rev id reg(the bit[3:0] of offset 08h)
	 * CPLD Board REV ID Register(08h)
	 * Bit[7:6] - Reserved
	 * Bit[5:0] - Board revision
	 */
	tx_len = 1;
	rx_len = 1;
	memset(data, 0, I2C_DATA_SIZE);
	data[0] = CPLD_BOARD_REV_ID_REG;
	i2c_msg = construct_i2c_message(I2C_BUS1, CPLD_ADDR, tx_len, data, rx_len);
	int ret = i2c_master_read(&i2c_msg, retry);
	if (ret == 0) {
		board_revision = i2c_msg.data[0] & 0x3F;
	} else {
		LOG_ERR("Failed to read board ID from CPLD");
	}
	printk("BIC class type(class-%d), 1ou present status(%d), 2ou present status(%d), board revision(0x%x)\n",
	       system_class, (int)_1ou_status.present, (int)_2ou_status.present, board_revision);

	/* BIC judges the 1OU card type according the ADC-6(0-based) voltage.
	 * The 1OU card type is
	 *  - "SI test board" if voltage is lower than 0.3V
	 *  - "Expansion with 6 M.2" if the voltage is 0.5V(+/- 5%)
	 *  - "Rainbow falls ( CXL with 4 DDR4 DIMMs)" if the voltage is 0.75V(+/- 5%)
	 *  - "Vernal falls (4 E1S, with TI chip)" if the voltage is 1.0V(+/- 5%)
	 *  - "Waimano falls (CXL with 2 DDR4 DIMMs+2 E1S)" if the voltage is 1.26V(+/- 5%)
	 *  - "Expansion with NIC" if the voltage is 1.5V(+/- 5%)
	 * And then, BIC sets the type to CL CPLD register(slave address 21h, register 09h)
	 * CPLD register 09h - 1OU Card Detection
	 *  - 00h: 1OU SI test card
	 *  - 01h: Expansion with 6 M.2
	 *  - 02h: Rainbow falls ( CXL with 4 DDR4 DIMMs)
	 *  - 03h: Vernal falls (with TI chip)
	 *  - 04h: Vernal falls (with AST1030 chip)
	 *  - 05h: Kahuna Falls
	 *  - 06h: Waimano falls (CXL with 2 DDR4 DIMMs+2 E1S)
	 *  - 07h: Expansion with NIC
	 */
	if (_1ou_status.present) {
		float voltage;
		if (get_adc_voltage(CHANNEL_6, &voltage)) {
			int cnt = 0;
			for (; cnt < ARRAY_SIZE(_1ou_card_mapping_table); cnt++) {
				float typical_voltage = _1ou_card_mapping_table[cnt].voltage;
				switch (_1ou_card_mapping_table[cnt].condition) {
				case LOWER:
					if (voltage <= typical_voltage) {
						_1ou_status.card_type =
							_1ou_card_mapping_table[cnt].card_type;
					}
					break;
				case HIGHER:
					if (voltage >= typical_voltage) {
						_1ou_status.card_type =
							_1ou_card_mapping_table[cnt].card_type;
					}
					break;
				case RANGE:
					if ((voltage >
					     typical_voltage - (typical_voltage * 0.07)) &&
					    (voltage <
					     typical_voltage + (typical_voltage * 0.07))) {
						_1ou_status.card_type =
							_1ou_card_mapping_table[cnt].card_type;
					}
					break;
				default:
					LOG_ERR("Unknown condition 0x%x",
						_1ou_card_mapping_table[cnt].condition);
					break;
				}

				if (_1ou_status.card_type != TYPE_1OU_UNKNOWN) {
					tx_len = 2;
					rx_len = 0;
					memset(data, 0, I2C_DATA_SIZE);
					data[0] = CPLD_1OU_CARD_DETECTION;
					data[1] = _1ou_status.card_type;
					i2c_msg = construct_i2c_message(I2C_BUS1, CPLD_ADDR, tx_len,
									data, rx_len);
					if (i2c_master_write(&i2c_msg, retry)) {
						LOG_ERR("Failed to set 1OU card detection to CPLD register(0x%x)",
							data[0]);
					}
					break;
				}
			}
			if (cnt == ARRAY_SIZE(_1ou_card_mapping_table)) {
				LOG_ERR("Unknown the 1OU card type, the voltage of ADC channel-6 is %fV",
					voltage);
			}
		}
	} else {
		// If 1ou card not present, disable ADC6(CARD_TYPE_EXP) and set it to GPIO to avoid floating
		uint32_t read_value = 0;
		// Disable ADC channel 6: ADC000[22]
		read_value = sys_read32(REG_ADC_BASE + 0x000);
		read_value = CLEARBIT(read_value, 22);
		sys_write32(read_value, REG_ADC_BASE + 0x000);

		// Multi-function pin ctl #5: SCU430[30] is GPIT6
		read_value = sys_read32(REG_SCU + 0x430);
		read_value = SETBIT(read_value, 30);
		sys_write32(read_value, REG_SCU + 0x430);

		// Enable internal PD
		read_value = sys_read32(REG_SCU + 0x630);
		read_value = CLEARBIT(read_value, 30);
		sys_write32(read_value, REG_SCU + 0x630);
	}

	if (_2ou_status.present) {
		if (_1ou_status.card_type == TYPE_1OU_EXP_WITH_E1S) {
			_2ou_status.card_type = TYPE_1OU_EXP_WITH_E1S;
		} else {
			tx_len = 1;
			rx_len = 1;
			memset(data, 0, I2C_DATA_SIZE);
			data[0] = CPLD_2OU_EXPANSION_CARD_REG;
			i2c_msg = construct_i2c_message(I2C_BUS1, CPLD_ADDR, tx_len, data, rx_len);
			if (!i2c_master_read(&i2c_msg, retry)) {
				switch (i2c_msg.data[0]) {
				case TYPE_2OU_DPV2_8:
				case TYPE_2OU_DPV2_16:
				case (TYPE_2OU_DPV2_8 | TYPE_2OU_DPV2_16):
					_2ou_status.card_type = i2c_msg.data[0];
					break;
				default:
					_2ou_status.card_type = TYPE_2OU_UNKNOWN;
					LOG_ERR("Unknown the 2OU card type, the card type read from CPLD is 0x%x",
						i2c_msg.data[0]);
					break;
				}
			}
		}
	}

	SAFE_FREE(data);
}
