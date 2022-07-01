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
#define CPLD_CLASS_TYPE_REG 0x0D
#define CPLD_2OU_EXPANSION_CARD_REG 0x13
#define I2C_DATA_SIZE 5

static uint8_t system_class = SYS_CLASS_1;
static CARD_STATUS _1ou_status = { false, TYPE_1OU_UNKNOWN };
static CARD_STATUS _2ou_status = { false, TYPE_2OU_UNKNOWN };

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

void init_platform_config()
{
	I2C_MSG i2c_msg;
	uint8_t retry = 3;

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
		printf("[%s] Failed to allocate memory\n", __func__);
		return;
	}
	
	/* Read the expansion present from CPLD's class type register
	 * CPLD Class Type Register(0Dh)
	 * Bit[7:4] - Reserved
	 * Bit[3] - 2ou x8/x16 Riser Expansion Present
	 * Bit[2] - 1ou Expansion Present Pin
	 * Bit[1:0] - Board ID(0000b: Class-1, 0001b: Class-2)
	 */
	tx_len = 1;
	rx_len = 1;
	memset(data, 0, I2C_DATA_SIZE);
	data[0] = CPLD_CLASS_TYPE_REG;
	i2c_msg = construct_i2c_message(I2C_BUS2, CPLD_ADDR, tx_len, data, rx_len);
	if (!i2c_master_read(&i2c_msg, retry)) {
		class_type = i2c_msg.data[0];
		_1ou_status.present = ((class_type & BIT(2)) ? false : true);
		_2ou_status.present = ((class_type & BIT(3)) ? false : true);
	} else {
		printf("Failed to read expansion present from CPLD\n");
	}

	/* Set the class type to CPLD's class type register(the bit[1:0] of offset 0Dh) 
	   CPLD will set different UART topology according to class type*/
	tx_len = 2;
	rx_len = 0;
	memset(data, 0, I2C_DATA_SIZE);
	data[0] = CPLD_CLASS_TYPE_REG;
	data[1] = system_class | class_type;
	i2c_msg = construct_i2c_message(I2C_BUS2, CPLD_ADDR, tx_len, data, rx_len);
	if (i2c_master_write(&i2c_msg, retry)) {
		printf("Failed to set class type to CPLD)\n");
	}

	if (_2ou_status.present) {
		tx_len = 1;
		rx_len = 1;
		memset(data, 0, I2C_DATA_SIZE);
		data[0] = CPLD_2OU_EXPANSION_CARD_REG;
		i2c_msg = construct_i2c_message(I2C_BUS2, CPLD_ADDR, tx_len, data, rx_len);
		if (!i2c_master_read(&i2c_msg, retry)) {
			switch (i2c_msg.data[0]) {
			case TYPE_2OU_EXP:
			case TYPE_2OU_EXP_E1S:
			case TYPE_2OU_HSM:
				_2ou_status.card_type = i2c_msg.data[0];
				break;
			default:
				_2ou_status.card_type = TYPE_2OU_UNKNOWN;
				printf("Unknown the 2OU card type, the card type read from CPLD is 0x%x\n",
				       i2c_msg.data[0]);
				break;
			}
		}
	}
	SAFE_FREE(data);
}
