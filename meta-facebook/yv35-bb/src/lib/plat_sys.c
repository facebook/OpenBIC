#include "plat_sys.h"

#include <stdio.h>
#include <stdlib.h>

#include "util_sys.h"
#include "ipmi.h"
#include "plat_i2c.h"
#include "plat_gpio.h"

int pal_submit_12v_cycle_slot(ipmi_msg *msg)
{
	uint8_t retry = 3, isolator_num;
	I2C_MSG i2c_msg;

	i2c_msg.bus = CPLD_IO_I2C_BUS;
	i2c_msg.target_addr = CPLD_IO_I2C_ADDR;
	i2c_msg.tx_len = 2;

	if (msg->InF_source == SLOT1_BIC) {
		isolator_num = FM_BIC_SLOT1_ISOLATED_EN_R;
		i2c_msg.data[0] = CPLD_IO_REG_OFS_HSC_EN_SLOT1; // offset

	} else if (msg->InF_source == SLOT3_BIC) {
		isolator_num = FM_BIC_SLOT3_ISOLATED_EN_R;
		i2c_msg.data[0] = CPLD_IO_REG_OFS_HSC_EN_SLOT3; // offset
	} else {
		return NOT_SUPPORT_12V_CYCLE_SLOT;
	}

	// Before slot 12V off, disable isolator
	i2c_msg.data[1] = 0x00;
	gpio_set(isolator_num, GPIO_LOW);
	if (i2c_master_write(&i2c_msg, retry) < 0) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return SLOT_OFF_FAILED;
	}

	k_msleep(2000);

	// After slot 12V on, enable isolator
	i2c_msg.data[1] = 0x01;
	if (i2c_master_write(&i2c_msg, retry) < 0) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return SLOT_ON_FAILED;
	}
	gpio_set(isolator_num, GPIO_HIGH);

	msg->completion_code = CC_SUCCESS;
	return SUCCESS_12V_CYCLE_SLOT;
}
