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

#include <logging/log.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hal_gpio.h"
#include "hal_i2c.h"
#include "libutil.h"
#include "plat_gpio.h"
#include "plat_i2c.h"
#include "rg3mxxb12.h"
#include "p3h284x.h"
LOG_MODULE_REGISTER(plat_class);

#define NUMBER_OF_ADC_CHANNEL 16

static uint8_t system_class = SYS_CLASS_1;
static uint8_t board_revision = 0x0F;
static uint8_t hsc_module = HSC_MODULE_UNKNOWN;
static CARD_STATUS _1ou_status = { false, TYPE_1OU_UNKNOWN };
static CARD_STATUS _2ou_status = { false, TYPE_2OU_UNKNOWN };
static uint16_t i3c_hub_type = I3C_HUB_TYPE_UNKNOWN;
static uint16_t exp_i3c_hub_type = I3C_HUB_TYPE_UNKNOWN;

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

uint16_t get_i3c_hub_type()
{
	return i3c_hub_type;
}

uint16_t get_exp_i3c_hub_type()
{
	return exp_i3c_hub_type;
}

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
	{ 0.3, LOWER, TYPE_1OU_SI_TEST_CARD },
	{ 0.5, RANGE, TYPE_1OU_EXP_WITH_6_M2 },
	{ 0.75, RANGE, TYPE_1OU_RAINBOW_FALLS },
	{ 1.0, RANGE, TYPE_1OU_VERNAL_FALLS_WITH_TI },
	{ 1.26, RANGE, TYPE_1OU_NIAGARA_FALLS },
	{ 1.5, RANGE, TYPE_1OU_EXP_WITH_NIC },
	{ 1.75, RANGE, TYPE_1OU_VERNAL_FALLS_WITH_AST },
	{ 2.0, RANGE, TYPE_1OU_OLMSTED_POINT },
};

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
	reg_value = sys_read32(AST1030_ADC_BASE_ADDR + adc_info[channel].offset);
	raw_value = (reg_value >> adc_info[channel].shift) & 0x3FF; // 10-bit(0x3FF) resolution

	// Real voltage = raw data * reference voltage / 2 ^ resolution(10)
	*voltage = (raw_value * reference_voltage) / 1024;

	return true;
}

void init_platform_config()
{
	int retry = 5;
	int status;
	I2C_MSG msg;
	/* Initialize board revision
	 * BIC gets borad revision id from SB CPLD register 08h.
	 */
	msg.bus = SB_CPLD_BUS;
	msg.target_addr = SB_CPLD_ADDR;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = SB_CPLD_REG_BOARD_REVISION_ID;
	status = i2c_master_read(&msg, retry);
	if (status)
		LOG_ERR("failed to get board revision from cpld, ret %d", status);
	else
		board_revision = msg.data[0] & 0x7;

	/* Initialize class type
	 * BIC gets class type from SB CPLD register 05h.
	 */
	uint8_t class_type = 0x0;
	msg.bus = SB_CPLD_BUS;
	msg.target_addr = SB_CPLD_ADDR;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = SB_CPLD_REG_BOARD_CLASS_TYPE;
	status = i2c_master_read(&msg, retry);
	if (status)
		LOG_ERR("failed to get class type from cpld, ret %d", status);
	class_type = msg.data[0];

	_1ou_status.present = false;

	if (!(class_type & BIT(2))) {
		if ((class_type & BIT(0)) && (class_type & BIT(1)) && (class_type & BIT(4))) {
			float epsilon = 0.001;
			int count = 0;
			for (; count < ARRAY_SIZE(_1ou_card_mapping_table); count++) {
				if (((_1ou_card_mapping_table[count].voltage - 0.3) <
				     epsilon) && // Check if the voltage in the mapping table matches 0.3V.
				    (_1ou_card_mapping_table[count].condition ==
				     LOWER)) { //If BIT2 is low and BIT0,1,4 are high, SI test card need to be changed to E1S card when ADC6 is lower than 0.3V
					_1ou_card_mapping_table[count].card_type =
						TYPE_1OU_EXP_WITH_E1S;
					_1ou_status.present = true;
					break;
				}
			}
			if (_1ou_card_mapping_table[count].card_type != TYPE_1OU_EXP_WITH_E1S) {
				LOG_ERR("Failed to initialize 1OU card mapping table because E1S card doesn't add in it");
			}
		} else if (!(class_type & BIT(0)) && !(class_type & BIT(1)) &&
			   !(class_type & BIT(4))) { //BIT0,1,2,4 are low when inserting 1OU card.
			_1ou_status.present = true;
		} else {
			LOG_ERR("Failed to check 1OU card's presence because the register 05h's value is abnormal. Value: 0x%02x",
				class_type);
		}
	}

	_2ou_status.present = ((class_type & BIT(3)) ? false : true);

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
	 *    08h: OPA/OPB
	 */
	if (_1ou_status.present) {
		float voltage;
		bool success = get_adc_voltage(CHANNEL_6, &voltage);
		if (success) {
			for (int cnt = 0; cnt < ARRAY_SIZE(_1ou_card_mapping_table); cnt++) {
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
					     typical_voltage - (typical_voltage * 0.05)) &&
					    (voltage <
					     typical_voltage + (typical_voltage * 0.05))) {
						_1ou_status.card_type =
							_1ou_card_mapping_table[cnt].card_type;
					}
					break;
				default:
					LOG_ERR("unknown condition 0x%x",
						_1ou_card_mapping_table[cnt].condition);
					break;
				}

				if (_1ou_status.card_type != TYPE_1OU_UNKNOWN) {
					msg.tx_len = 2;
					msg.rx_len = 0;
					msg.data[0] = SB_CPLD_REG_1OU_CARD_DETECTION;
					msg.data[1] = _1ou_status.card_type;
					status = i2c_master_write(&msg, retry);
					if (status)
						LOG_ERR("failed to set 1ou card type, ret %d",
							status);
					break;
				}
				if ((cnt == ARRAY_SIZE(_1ou_card_mapping_table)) &&
				    (_1ou_status.card_type == TYPE_1OU_UNKNOWN)) {
					LOG_ERR("unknown the 1OU card type, the voltage of ADC channel-6 is %fV",
						voltage);
				}
			}
		}
	}

	if (_2ou_status.present) {
		if (_1ou_status.card_type == TYPE_1OU_OLMSTED_POINT) {
			_2ou_status.card_type = TYPE_1OU_OLMSTED_POINT;
		}
	}

	/* Initialize HSC module type
	 * The HCS ADM1278 is used if BIC GPIOA0 "HSC_TYPE" is H.
	 * Else, the HCS module type is MP5990.
	 */
	hsc_module = (gpio_get(HSC_TYPE) == GPIO_HIGH) ? HSC_MODULE_ADM1278 : HSC_MODULE_MP5990;

	LOG_INF("BIC class type(class-%d), 1ou present status(%d), 2ou present status(%d), board revision(0x%x)\n",
		system_class, (int)_1ou_status.present, (int)_2ou_status.present, board_revision);
}

void init_i3c_hub_type(void)
{
	if (rg3mxxb12_get_device_info(I2C_BUS8, &exp_i3c_hub_type)) {
		LOG_INF("Expansion I3C hub type: rg3mxxb12");
	} else if (p3h284x_get_device_info(I2C_BUS8, &exp_i3c_hub_type)) {
		LOG_INF("Expansion I3C hub type: p3h284x");
	} else {
		LOG_ERR("Expansion I3C hub get device type fail");
	}

	if (rg3mxxb12_get_device_info_i3c(I3C_BUS4, &i3c_hub_type) &&
	    (i3c_hub_type == RG3M87B12_DEVICE_INFO)) {
		LOG_INF("I3C hub type: rg3mxxb12");
	} else if (p3h284x_get_device_info_i3c(I3C_BUS4, &i3c_hub_type)) {
		LOG_INF("I3C hub type: p3h284x");
	} else {
		LOG_ERR("I3C hub get device type fail");
	}
}
