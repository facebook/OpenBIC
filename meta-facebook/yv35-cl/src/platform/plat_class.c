#include "plat_class.h"

#include <stdio.h>

#include "hal_gpio.h"
#include "hal_i2c.h"
#include "plat_gpio.h"

static bool bic_class = sys_class_1;
static bool is_1ou_present = 0;
static bool is_2ou_present = 0;
static uint8_t card_type_2ou = 0;

bool get_bic_class()
{
	return bic_class;
}

bool get_1ou_status()
{
	return is_1ou_present;
}

bool get_2ou_status()
{
	return is_2ou_present;
}

uint8_t get_2ou_cardtype()
{
	return card_type_2ou;
}

void init_platform_config()
{
	I2C_MSG i2c_msg;
	uint8_t retry = 3;

	if (gpio_get(BOARD_ID0)) { // HW design: class1 board_ID 0000, class2 board_ID 0001
		bic_class = sys_class_2;
	} else {
		bic_class = sys_class_1;
	}

	i2c_msg.bus = i2c_bus_to_index[1];
	i2c_msg.slave_addr = 0x42 >> 1;
	i2c_msg.rx_len = 0x0;
	i2c_msg.tx_len = 0x2;
	i2c_msg.data[0] = 0x5;
	i2c_msg.data[1] = (bic_class << 4) & 0x10; // set CPLD class in reg 0x5, bit 4
	if (i2c_master_write(&i2c_msg, retry)) {
		printf("Set CPLD class type fail\n");
	} else {
		i2c_msg.rx_len = 0x1;
		i2c_msg.tx_len = 0x1;
		i2c_msg.data[0] = 0x5;
		if (!i2c_master_read(&i2c_msg, retry)) {
			is_1ou_present = (i2c_msg.data[0] & 0x4 ? 0 : 1);
			is_2ou_present = (i2c_msg.data[0] & 0x8 ? 0 : 1);

			if ((i2c_msg.data[0] & 0x10) != bic_class) {
				printf("Set class type %x but read %x\n", bic_class,
				       (i2c_msg.data[0] & 0x10));
			}
		} else {
			printf("Read expansion present from CPLD error\n");
		}
	}
	printk("bic class type : %d  1ou present status : %d  2ou present status : %d\n",
	       bic_class + 1, is_1ou_present, is_2ou_present);

	if (is_2ou_present) {
		i2c_msg.data[0] = 0x6;
		if (!i2c_master_read(&i2c_msg, retry)) {
			if ((i2c_msg.data[0] == type_2ou_dpv2)) {
				card_type_2ou = type_2ou_dpv2;
			} else if (i2c_msg.data[0] == type_2ou_spe) {
				card_type_2ou = type_2ou_spe;
			} else if (i2c_msg.data[0] == type_2ou_exp) {
				card_type_2ou = type_2ou_exp;
			} else if (i2c_msg.data[0] == type_2ou_dpv2_8) { // in case the SKU exist
				card_type_2ou = type_2ou_dpv2_8;
			} else if (i2c_msg.data[0] == type_2ou_dpv2_16) { // in case the SKU exist
				card_type_2ou = type_2ou_dpv2_16;
			}
		}
	}
}
