#include "plat_func.h"
#include "plat_gpio.h"
#include "plat_i2c.h"
#include "plat_ipmi.h"
#include "hal_snoop.h"
#include "plat_def.h"

static bool is_DC_on = 0;
static bool is_post_complete = 0;
static bool bic_class = sys_class_1;
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

  if ( gpio_get(BOARD_ID0) ) { // HW design: class1 board_ID 0000, class2 board_ID 0001
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
  if ( i2c_master_write(&i2c_msg, retry) ) {
    printk("Set CPLD class type fail\n");
  } else {
    i2c_msg.rx_len = 0x1;
    i2c_msg.tx_len = 0x1;
    i2c_msg.data[0] = 0x5;
    if ( !i2c_master_read(&i2c_msg, retry) ) {
      is_1ou_present = ( i2c_msg.data[0] & 0x4 ? 0 : 1 );
      is_2ou_present = ( i2c_msg.data[0] & 0x8 ? 0 : 1 );

      if ( (i2c_msg.data[0] & 0x10) != bic_class ) {
        printk("Set class type %x but read %x\n", bic_class, (i2c_msg.data[0] & 0x10));
      }
    }else {
      printk("Read expansion present from CPLD error\n");
    }

    printk( "bic class type : %d  1ou present status : %d  2ou present status : %d\n" , bic_class+1 , is_1ou_present , is_2ou_present );
  }
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
  msg.cmd = CMD_OEM_1S_SEND_INTERRUPT_TO_BMC;

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

void enable_asd_gpio_interrupt() {
  gpio_interrupt_conf(H_BMC_PRDY_BUF_N, GPIO_INT_EDGE_FALLING);
  gpio_interrupt_conf(PWRGD_CPU_LVC3, GPIO_INT_EDGE_BOTH);
  gpio_interrupt_conf(RST_PLTRST_BUF_N, GPIO_INT_EDGE_BOTH);
  gpio_interrupt_conf(FM_DBP_PRESENT_N, GPIO_INT_EDGE_BOTH);
}

void disable_asd_gpio_interrupt() {
  gpio_interrupt_conf(H_BMC_PRDY_BUF_N, GPIO_INT_DISABLE);
  gpio_interrupt_conf(PWRGD_CPU_LVC3, GPIO_INT_DISABLE);
  gpio_interrupt_conf(RST_PLTRST_BUF_N, GPIO_INT_DISABLE);
  gpio_interrupt_conf(FM_DBP_PRESENT_N, GPIO_INT_DISABLE);
}
