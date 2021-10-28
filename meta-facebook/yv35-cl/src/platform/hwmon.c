#include "plat_func.h"
#include "plat_gpio.h"
#include "plat_i2c.h"

static bool is_DC_on = 0;
static bool is_post_complete = 0;
static bool bic_class = 0; // 0: class1 , 1: class2
static bool is_1ou_present = 0;
static bool is_2ou_present = 0;

void ISR_slp3(uint32_t tmp0, uint32_t tmp1) {
  printk("slp3\n");
}

void ISR_post_complete() {
  set_post_status();
}

void ISR_DC_on() {
  set_DC_status();
}

void set_DC_status() {
  is_DC_on = gpio_get(PWRGD_SYS_PWROK);
  printk("set dc status %d\n", is_DC_on);
}

bool get_DC_status() {
  return is_DC_on;
}

void set_post_status() {
  is_post_complete = !(gpio_get(FM_BIOS_POST_CMPLT_BMC_N));
  printk("set is_post_complete %d\n", is_post_complete);
}

bool get_post_status() {
  return is_post_complete;
}

void set_sys_config() {
  I2C_MSG i2c_msg;
  uint8_t retry = 3;
  i2c_msg.bus = i2c_bus_to_index[1];
  i2c_msg.slave_addr = 0x42 >> 1 ;
  i2c_msg.rx_len = 0x1;
  i2c_msg.tx_len = 0x1;
  i2c_msg.data[0] = 0x5;

  if ( !i2c_master_read(&i2c_msg, retry) ) {
    bic_class = ( i2c_msg.data[0] & 0x1 ? 0 : 1 );
    is_1ou_present = ( i2c_msg.data[0] & 0x4 ? 0 : 1 );
    is_2ou_present = ( i2c_msg.data[0] & 0x8 ? 0 : 1 );    
  }else {
    printk( "I2C bus to CPLD error\n" );
  }
  printk( "bic class type : %d  1ou present status : %d  2ou present status : %d\n" , bic_class+1 , is_1ou_present , is_2ou_present );
}

bool get_bic_class() {
  return bic_class;
}

bool get_1ou_status() {
  return is_1ou_present;
}

bool get_2ou_status() {
  return is_2ou_present;
}
