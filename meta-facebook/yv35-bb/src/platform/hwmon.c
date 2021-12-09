#include "plat_func.h"
#include "plat_gpio.h"
#include "plat_i2c.h"


struct k_delayed_work sled_cycle_work;

K_DELAYED_WORK_DEFINE(sled_cycle_work, sled_cycle_work_handler);

void ISR_sled_cycle() {
  uint8_t bb_btn_status = GPIO_HIGH;

  bb_btn_status = gpio_get(BB_BUTTON_BMC_BIC_N_R);
  // press sled cycle button
  if (bb_btn_status == GPIO_LOW) {
    k_delayed_work_submit(&sled_cycle_work, K_SECONDS(MAX_PRESS_SLED_BTN_TIME_s));

  // release sled cycle button
  } else if (bb_btn_status == GPIO_HIGH) {
    k_delayed_work_cancel(&sled_cycle_work);
  }
}


void sled_cycle_work_handler(struct k_work *item) {
  set_sled_cycle();
}

void set_sled_cycle() {
  uint8_t retry = 5;
  I2C_MSG msg;

  msg.bus = CPLD_IO_I2C_BUS;
  msg.slave_addr = CPLD_IO_I2C_ADDR;
  msg.tx_len = 2;
  msg.data[0] = CPLD_IO_REG_OFS_SLED_CYCLE; // offset
  msg.data[1] = 0x01; // value

  if (i2c_master_write(&msg, retry) < 0) {
    printk("sled cycle fail\n");
  }
}
