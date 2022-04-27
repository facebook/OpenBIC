#include "plat_class.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hal_gpio.h"
#include "hal_i2c.h"
#include "libutil.h"
#include "plat_gpio.h"
#include "plat_i2c.h"

#define CPLD_ADDR 0x21 // 7-bit address
#define CPLD_CLASS_TYPE_REG 0x05
#define CPLD_2OU_EXPANSION_CARD_REG 0x06
#define CPLD_BOARD_REV_ID_REG 0x08
#define I2C_DATA_SIZE 5

static uint8_t system_class = SYS_CLASS_1;
static bool is_1ou_present = false;
static bool is_2ou_present = false;
static uint8_t __attribute__((unused)) card_type_1ou = 0;
static uint8_t card_type_2ou = TYPE_UNKNOWN;
static uint8_t board_revision = 0x0F;

uint8_t get_system_class()
{
	return system_class;
}

bool get_1ou_status()
{
	return is_1ou_present;
}

bool get_2ou_status()
{
	return is_2ou_present;
}

uint8_t get_board_revision()
{
	return board_revision;
}

uint8_t get_2ou_cardtype()
{
	return card_type_2ou;
}

float get_hsc_type_adc_voltage()
{
	uint32_t adc_base_address = 0x7e6e9000, adc7_raw, reg_val;
	long unsigned int engine_control = 0x0, adc_data_of_ch7_and_6 = 0x1C;
	float reference_voltage = 0.0f;
	uint8_t reference_voltage_selection;

	/* Get ADC reference voltage from Aspeed chip
	 * ADC000: Engine Control
	 * [7:6] Reference Voltage Selection
	 * 00b - 2.5V / 01b - 1.2V / 10b and 11b - External Voltage
	 */
	reg_val = sys_read32(adc_base_address + engine_control);
	reference_voltage_selection = (reg_val >> 6) & 0x3;
	if (reference_voltage_selection == 0b00) {
		reference_voltage = 2.5;
	} else if (reference_voltage_selection == 0b01) {
		reference_voltage = 1.2;
	} else {
		printf("Not supported the external reference voltage\n");
	}

	/* Read ADC channel-7 raw value
	 * ADC01C: Data of Channel 7 and 6
	 * [25:16] Data of channel 7
	 */
	reg_val = sys_read32(adc_base_address + adc_data_of_ch7_and_6);
	adc7_raw = (reg_val & 0x3FF0000) >> 16;

	// Real voltage = raw data * reference voltage / 2 ^ resolution(10)
	return ((adc7_raw * reference_voltage) / (1024));
}
