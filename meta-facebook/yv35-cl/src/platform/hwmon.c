#include "plat_func.h"
#include "plat_gpio.h"
#include "plat_i2c.h"
#include "plat_ipmi.h"
#include "hal_snoop.h"

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

void ISR_BMC_PRDY(){
  send_gpio_interrupt(H_BMC_PRDY_BUF_N);
}

void ISR_PWRGD_CPU(){
  send_gpio_interrupt(PWRGD_CPU_LVC3);
}

void ISR_PLTRST(){
  send_gpio_interrupt(RST_PLTRST_BUF_N);
}

void ISR_DBP_PRSNT(){
  send_gpio_interrupt(FM_DBP_PRESENT_N);
}

void set_SCU_setting() {
  sys_write32(0xffffffff, 0x7e6e2610);
  sys_write32(0xffffffff, 0x7e6e2614);
  sys_write32(0x30000000, 0x7e6e2618);
  sys_write32(0x00000F04, 0x7e6e261c);
}

void set_DC_status() {
  is_DC_on = gpio_get(PWRGD_SYS_PWROK);
  printk("set dc status %d\n", is_DC_on);

  if ( is_DC_on ){
    snoop_start_thread();
  }else{
    free_snoop_buffer();
  }
}

bool get_DC_status() {
  return is_DC_on;
}

void set_post_status() {
  is_post_complete = !(gpio_get(FM_BIOS_POST_CMPLT_BMC_N));
  printk("set is_post_complete %d\n", is_post_complete);

  if ( is_post_complete ){
    snoop_abort_thread();
  }
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

void send_gpio_interrupt(uint8_t gpio_num)
{
  ipmb_error status;
  ipmi_msg msg;
  uint8_t gpio_val;

  printk("Send gpio interrupt %d to BMC\n", gpio_num);
  gpio_val = gpio_get(gpio_num);

  msg.data_len = 5;
  msg.InF_source = Self_IFs;
  msg.InF_target = BMC_IPMB_IFs;
  msg.netfn = NETFN_OEM_1S_REQ;
  msg.cmd = CMD_OEM_SEND_INTERRUPT_TO_BMC;

  msg.data[0] = WW_IANA_ID & 0xFF;
  msg.data[1] = (WW_IANA_ID >> 8) & 0xFF;
  msg.data[2] = (WW_IANA_ID >> 16) & 0xFF;
  msg.data[3] = gpio_num;
  msg.data[4] = gpio_val;

  status = ipmb_read(&msg, IPMB_inf_index_map[msg.InF_target]);
  if (status == ipmb_error_failure) {
    printk("Fail to post msg to txqueue for gpio %d interrupt\n", gpio_num);
  } else if (status == ipmb_error_get_messageQueue) {
    printk("No response from bmc for gpio %d interrupt\n", gpio_num);
  }
}
