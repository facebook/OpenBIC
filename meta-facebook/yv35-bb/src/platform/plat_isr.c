#include "plat_isr.h"

#include "libipmi.h"
#include "plat_isr.h"

#include "kcs.h"
#include "power_status.h"
#include "sensor.h"
#include "snoop.h"
#include "plat_gpio.h"
#include "plat_ipmi.h"
#include "plat_sensor_table.h"
#include "oem_1s_handler.h"
#include "plat_i2c.h"

void sled_cycle_work_handler(struct k_work *item);

K_WORK_DELAYABLE_DEFINE(sled_cycle_work, sled_cycle_work_handler);

void ISR_PWROK_SLOT1()
{
	set_BIC_slot_isolator(PWROK_STBY_BIC_SLOT1_R, FM_BIC_SLOT1_ISOLATED_EN_R);
}

void ISR_PWROK_SLOT3()
{
	set_BIC_slot_isolator(PWROK_STBY_BIC_SLOT3_R, FM_BIC_SLOT3_ISOLATED_EN_R);
}

void ISR_SLED_CYCLE()
{
	uint8_t bb_btn_status = GPIO_HIGH;

	bb_btn_status = gpio_get(BB_BUTTON_BMC_BIC_N_R);
	// press sled cycle button
	if (bb_btn_status == GPIO_LOW) {
		k_work_schedule(&sled_cycle_work, K_SECONDS(MAX_PRESS_SLED_BTN_TIME_s));

		// release sled cycle button
	} else if (bb_btn_status == GPIO_HIGH) {
		k_work_cancel_delayable(&sled_cycle_work);
	}
}

void set_BIC_slot_isolator(uint8_t pwr_state_gpio_num, uint8_t isolator_gpio_num)
{
	int ret = 0;
	uint8_t slot_pwr_status = GPIO_LOW;

	slot_pwr_status = gpio_get(pwr_state_gpio_num);

	if (slot_pwr_status == GPIO_HIGH) {
		ret = gpio_set(isolator_gpio_num, GPIO_HIGH);
	} else if (slot_pwr_status == GPIO_LOW) {
		ret = gpio_set(isolator_gpio_num, GPIO_LOW);
	}

	if (ret < 0) {
		printf("failed to set slot isolator due to set gpio %d is failed\n",
		       isolator_gpio_num);
	}
}

void sled_cycle_work_handler(struct k_work *item)
{
	set_sled_cycle();
}

void set_sled_cycle()
{
	uint8_t retry = 5;
	I2C_MSG msg;

	msg.bus = CPLD_IO_I2C_BUS;
	msg.target_addr = CPLD_IO_I2C_ADDR;
	msg.tx_len = 2;
	msg.data[0] = CPLD_IO_REG_OFS_SLED_CYCLE; // offset
	msg.data[1] = 0x01; // value

	if (i2c_master_write(&msg, retry) < 0) {
		printf("sled cycle fail\n");
	}
}
