#include "plat_func.h"
#include "plat_gpio.h"
#include "plat_i2c.h"
#include "plat_ipmi.h"
#include "hal_snoop.h"
#include "kcs.h"
#include "cmsis_os2.h"
#include "libipmi.h"
#include "sensor_def.h"

static bool is_DC_on = 0;
static bool is_post_complete = 0;
static bool bic_class = 0; // 0: class1 , 1: class2
static bool is_1ou_present = 0;
static bool is_2ou_present = 0;

#define PROC_FAIL_STACK_SIZE 1024
#define CATERR_THREAD_STACK_SIZE 1024

#define PROC_FAIL_START_DELAY_SECOND 10
#define CATERR_START_DELAY_SECOND 2

K_THREAD_STACK_DEFINE(proc_fail_thread, PROC_FAIL_STACK_SIZE);
struct k_thread proc_fail_thread_handler;
K_THREAD_STACK_DEFINE(CatErr_thread, CATERR_THREAD_STACK_SIZE);
struct k_thread CatErr_thread_handler;

static void proc_fail_handler(void*, void*, void*);
static void CatErr_handler(void*, void*, void*);

void SLP3_handler() {
  addsel_msg_t sel_msg;
  if(gpio_get(FM_SLPS3_PLD_N) && (gpio_get(PWRGD_SYS_PWROK) == 0)) {
    sel_msg.snr_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
    sel_msg.evt_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
    sel_msg.snr_number = SENSOR_NUM_SYSTEM_STATUS;
    sel_msg.evt_data1 = IPMI_OEM_EVENT_OFFSET_SYS_VRWATCHDOG;
    sel_msg.evt_data2 = 0xFF;
    sel_msg.evt_data3 = 0xFF;
    if (!add_sel_evt_record(&sel_msg)) {
      printk("VR watchdog timeout addsel fail\n");
    }
  }
}

K_WORK_DELAYABLE_DEFINE(SLP3_work, SLP3_handler);
void ISR_slp3() {
  if (gpio_get(FM_SLPS3_PLD_N)) {
    printk("slp3\n");
    k_work_schedule(&SLP3_work, K_MSEC(10000));
  } else {
    if (k_work_cancel_delayable(&SLP3_work) != 0) {
      printk("Cancel SLP3 delay work fail\n");
    }
  }
}

void ISR_post_complete() {
  set_post_status();
}

void ISR_DC_on() {
  set_DC_status();
  if (is_DC_on == 0) {
    addsel_msg_t sel_msg;
    if (gpio_get(FM_SLPS3_PLD_N) && gpio_get(RST_RSMRST_BMC_N)) {
      sel_msg.snr_type = IPMI_OEM_SENSOR_TYPE_OEM_C3;
      sel_msg.evt_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
      sel_msg.snr_number = SENSOR_NUM_POWER_ERROR;
      sel_msg.evt_data1 = IPMI_OEM_EVENT_OFFSET_SYS_PWROK_FAIL;
      sel_msg.evt_data2 = 0xFF;
      sel_msg.evt_data3 = 0xFF;
      if (!add_sel_evt_record(&sel_msg)) {
        printk("System PWROK failure addsel fail\n");
      }
    }
  }
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
                                    osPriorityBelowNormal, 0, K_SECONDS(PROC_FAIL_START_DELAY_SECOND));
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
                                osPriorityBelowNormal, 0, K_SECONDS(CATERR_START_DELAY_SECOND));
    k_thread_name_set(&CatErr_thread_handler, "CatErr_thread");
  }
}

void ISR_PLTRST(){
  send_gpio_interrupt(RST_PLTRST_BUF_N);
}

void ISR_DBP_PRSNT(){
  send_gpio_interrupt(FM_DBP_PRESENT_N);
}

void ISR_SOC_THMALTRIP() {
  addsel_msg_t sel_msg;
  if (gpio_get(RST_PLTRST_PLD_N)) {
    sel_msg.evt_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
    sel_msg.snr_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
    sel_msg.snr_number = SENSOR_NUM_SYSTEM_STATUS;
    sel_msg.evt_data1 = IPMI_OEM_EVENT_OFFSET_SYS_THERMAL_TRIP;
    sel_msg.evt_data2 = 0xFF;
    sel_msg.evt_data3 = 0xFF;
    if (!add_sel_evt_record(&sel_msg)) {
      printk("SOC Thermal trip addsel fail\n");
    }
  }
}

void ISR_SYS_THROTTLE() {
  addsel_msg_t sel_msg;
  if (gpio_get(RST_PLTRST_PLD_N) && gpio_get(PWRGD_SYS_PWROK)) {
    if (gpio_get(FM_CPU_BIC_PROCHOT_LVT3_N)) {
      sel_msg.evt_type = IPMI_OEM_EVENT_TYPE_DEASSART;
    } else {
      sel_msg.evt_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
    }
    sel_msg.snr_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
    sel_msg.snr_number = SENSOR_NUM_SYSTEM_STATUS;
    sel_msg.evt_data1 = IPMI_OEM_EVENT_OFFSET_SYS_THROTTLE;
    sel_msg.evt_data2 = 0xFF;
    sel_msg.evt_data3 = 0xFF;
    if (!add_sel_evt_record(&sel_msg)) {
      printk("System Throttle addsel fail\n");
    }
  }
}

void ISR_PCH_THMALTRIP() {
  addsel_msg_t sel_msg;
  static bool is_PCH_assert = 0;
  if (gpio_get(FM_PCHHOT_N) == 0) {
    if (gpio_get(RST_PLTRST_PLD_N) && is_post_complete && (is_PCH_assert == 0)) {
      sel_msg.evt_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
      is_PCH_assert = 1;
    }
  } else if (gpio_get(FM_PCHHOT_N) && (is_PCH_assert == 1)) {
    sel_msg.evt_type = IPMI_OEM_EVENT_TYPE_DEASSART;
    is_PCH_assert = 0;
  } else {
    return;
  }
  sel_msg.snr_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
  sel_msg.snr_number = SENSOR_NUM_SYSTEM_STATUS;
  sel_msg.evt_data1 = IPMI_OEM_EVENT_OFFSET_SYS_PCHHOT;
  sel_msg.evt_data2 = 0xFF;
  sel_msg.evt_data3 = 0xFF;
  if (!add_sel_evt_record(&sel_msg)) {
    printk("PCH Thermal trip addsel fail\n");
  }
}

void ISR_HSC_OC() {
  addsel_msg_t sel_msg;
  if (gpio_get(RST_RSMRST_BMC_N)) {
    if (gpio_get(FM_HSC_TIMER)) {
      sel_msg.evt_type = IPMI_OEM_EVENT_TYPE_DEASSART;
    } else {
      sel_msg.evt_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
    }
    sel_msg.snr_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
    sel_msg.snr_number = SENSOR_NUM_SYSTEM_STATUS;
    sel_msg.evt_data1 = IPMI_OEM_EVENT_OFFSET_SYS_HSCTIMER;
    sel_msg.evt_data2 = 0xFF;
    sel_msg.evt_data3 = 0xFF;
    if (!add_sel_evt_record(&sel_msg)) {
      printk("HSC OC addsel fail\n");
    }
  }
}

void ISR_CPU_MEMHOT() {
  addsel_msg_t sel_msg;
  if (gpio_get(RST_PLTRST_PLD_N) && gpio_get(PWRGD_SYS_PWROK)) {
    if (gpio_get(H_CPU_MEMHOT_OUT_LVC3_N)) {
      sel_msg.evt_type = IPMI_OEM_EVENT_TYPE_DEASSART;
    } else {
      sel_msg.evt_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
    }
    sel_msg.snr_type = IPMI_OEM_SENSOR_TYPE_CPU_DIMM_HOT;
    sel_msg.snr_number = SENSOR_NUM_CPUDIMM_HOT;
    sel_msg.evt_data1 = IPMI_OEM_EVENT_OFFSET_DIMM_HOT;
    sel_msg.evt_data2 = 0xFF;
    sel_msg.evt_data3 = 0xFF;
    if (!add_sel_evt_record(&sel_msg)) {
      printk("CPU MEM HOT addsel fail\n");
    }
  }
}   

void ISR_CPUVR_HOT() {
  addsel_msg_t sel_msg;
  if (gpio_get(RST_PLTRST_PLD_N) && gpio_get(PWRGD_SYS_PWROK)) {
    if (gpio_get(IRQ_CPU0_VRHOT_N)) {
      sel_msg.evt_type = IPMI_OEM_EVENT_TYPE_DEASSART;
    } else {
      sel_msg.evt_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
    }
    sel_msg.snr_type = IPMI_OEM_SENSOR_TYPE_CPU_DIMM_VR_HOT;
    sel_msg.snr_number = SENSOR_NUM_VR_HOT;
    sel_msg.evt_data1 = IPMI_OEM_EVENT_OFFSET_CPU_VR_HOT;
    sel_msg.evt_data2 = 0xFF;
    sel_msg.evt_data3 = 0xFF;
    if (!add_sel_evt_record(&sel_msg)) {
      printk("CPU VR HOT addsel fail\n");
    }
  }
}

void ISR_PCH_PWRGD() {
  addsel_msg_t sel_msg;
  if (gpio_get(FM_SLPS3_PLD_N)) {
    sel_msg.snr_type = IPMI_OEM_SENSOR_TYPE_OEM_C3;
    sel_msg.evt_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
    sel_msg.snr_number = SENSOR_NUM_POWER_ERROR;
    sel_msg.evt_data1 = IPMI_OEM_EVENT_OFFSET_PCH_PWROK_FAIL;
    sel_msg.evt_data2 = 0xFF;
    sel_msg.evt_data3 = 0xFF;
    if (!add_sel_evt_record(&sel_msg)) {
      printk("PCH PWROK failure addsel fail\n");
    }
  }
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
