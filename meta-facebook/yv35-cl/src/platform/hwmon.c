#include "plat_func.h"
#include "plat_gpio.h"
#include "plat_i2c.h"
#include "plat_ipmi.h"
#include "hal_snoop.h"
#include "kcs.h"
#include "libipmi.h"
#include "sensor_def.h"
#include "plat_def.h"

static bool is_DC_on = 0;
static bool is_post_complete = 0;
static bool bic_class = sys_class_1;
static bool is_1ou_present = 0;
static bool is_2ou_present = 0;
static uint8_t card_type_1ou = 0;
static uint8_t card_type_2ou = 0;

#define PROC_FAIL_STACK_SIZE 2000
#define CATERR_THREAD_STACK_SIZE 2000

#define PROC_FAIL_START_DELAY_SECOND 10
#define CATERR_START_DELAY_SECOND 2

K_THREAD_STACK_DEFINE(proc_fail_thread, PROC_FAIL_STACK_SIZE);
struct k_thread proc_fail_thread_handler;
K_THREAD_STACK_DEFINE(CatErr_thread, CATERR_THREAD_STACK_SIZE);
struct k_thread CatErr_thread_handler;

static void proc_fail_handler(void*, void*, void*);
static void CatErr_handler(void*, void*, void*);

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

void ISR_PWRGD_CPU() {
  static k_tid_t proc_fail_tid = NULL;

  if (gpio_get(PWRGD_CPU_LVC3) == 1) {
    /* start thread proc_fail_handler after 10 seconds */
    if (proc_fail_tid != NULL) {
      k_thread_abort(proc_fail_tid);
      proc_fail_tid = NULL;
    }
    proc_fail_tid = k_thread_create(&proc_fail_thread_handler, proc_fail_thread,
                                    K_THREAD_STACK_SIZEOF(proc_fail_thread),
                                    proc_fail_handler, NULL, NULL, NULL,
                                    CONFIG_MAIN_THREAD_PRIORITY, 0, K_SECONDS(PROC_FAIL_START_DELAY_SECOND));
    k_thread_name_set(&proc_fail_thread_handler, "proc_fail_thread");
  } else {
    if (proc_fail_tid != NULL) {
      k_thread_abort(proc_fail_tid);
      proc_fail_tid = NULL;
    }
    reset_kcs_ok();
    reset_postcode_ok();
  }
  send_gpio_interrupt(PWRGD_CPU_LVC3);
}

void ISR_CATERR() {
  static k_tid_t CatErr_tid = NULL;

  if ((gpio_get(RST_PLTRST_BUF_N) == 1)) {
    /* start thread CatErr_handler after 2 seconds */
    if (CatErr_tid != NULL) {
      k_thread_abort(CatErr_tid);
      CatErr_tid = NULL;
    }
    CatErr_tid = k_thread_create(&CatErr_thread_handler, CatErr_thread,
                                K_THREAD_STACK_SIZEOF(CatErr_thread),
                                CatErr_handler, NULL, NULL, NULL,
                                CONFIG_MAIN_THREAD_PRIORITY, 0, K_SECONDS(CATERR_START_DELAY_SECOND));
    k_thread_name_set(&CatErr_thread_handler, "CatErr_thread");
  }
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

uint8_t get_2ou_cardtype() {
  return card_type_2ou;
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
    } else {
      printk("Read expansion present from CPLD error\n");
    }
  }
  printk( "bic class type : %d  1ou present status : %d  2ou present status : %d\n" , bic_class+1 , is_1ou_present , is_2ou_present );

  if ( is_2ou_present ) {
    i2c_msg.data[0] = 0x6;
    if ( !i2c_master_read(&i2c_msg, retry) ) {
      if ( (i2c_msg.data[0] == type_2ou_dpv2) ) {
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

void enable_PRDY_interrupt() {
  gpio_interrupt_conf(H_BMC_PRDY_BUF_N, GPIO_INT_EDGE_FALLING);
}

void disable_PRDY_interrupt() {
  gpio_interrupt_conf(H_BMC_PRDY_BUF_N, GPIO_INT_DISABLE);
}

static void proc_fail_handler(void *arug0, void *arug1, void *arug2) {
  /* if have not received kcs and post code, add FRB3 event log. */
  if ((get_kcs_ok() == 0) && (get_postcode_ok() == 0)) {
    addsel_msg_t sel_msg;
    bool ret = 0;

    memset(&sel_msg, 0, sizeof(addsel_msg_t));

    sel_msg.snr_type = IPMI_SENSOR_TYPE_PROCESSOR;
    sel_msg.snr_number = SENSOR_NUM_PROC_FAIL;
    sel_msg.evt_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
    sel_msg.evt_data1 = IPMI_EVENT_OFFSET_PROCESSOR_FRB3;
    sel_msg.evt_data2 = 0xFF;
    sel_msg.evt_data3 = 0xFF;
    ret = add_sel_evt_record(&sel_msg);
    if (!ret) {
      printk("Fail to assert FRE3 event log.\n");
    }
  }
}

static void CatErr_handler(void *arug0, void *arug1, void *arug2) {
  if ((gpio_get(RST_PLTRST_BUF_N) == 1) || (gpio_get(PWRGD_SYS_PWROK) == 1)) {
    addsel_msg_t sel_msg;
    bool ret = 0;

    memset(&sel_msg, 0, sizeof(addsel_msg_t));

    sel_msg.snr_type = IPMI_SENSOR_TYPE_PROCESSOR;
    sel_msg.snr_number = SENSOR_NUM_CATERR;
    sel_msg.evt_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
    /* MCERR: one pulse, IERR: keep low */
    if (gpio_get(FM_CPU_RMCA_CATERR_LVT3_N) == 1) {
      sel_msg.evt_data1 = IPMI_EVENT_OFFSET_PROCESSOR_MCERR;
    } else {
      sel_msg.evt_data1 = IPMI_EVENT_OFFSET_PROCESSOR_IERR;
    }
    sel_msg.evt_data2 = 0xFF;
    sel_msg.evt_data3 = 0xFF;
    ret = add_sel_evt_record(&sel_msg);
    if (!ret) {
      printk("Fail to assert CatErr event log.\n");
    }
  }
}
