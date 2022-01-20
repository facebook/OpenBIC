#include <zephyr.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/printk.h>
#include <device.h>
#include "hal_snoop.h"
#include "ipmi.h"
#include "ipmb.h"

const struct device *snoop_dev;
uint8_t *snoop_data;
static uint8_t *snoop_read_buffer;
int snoop_read_num[2] = {0};
static bool proc_postcode_ok = false;

K_THREAD_STACK_DEFINE( snoop_thread , SNOOP_STACK_SIZE );
struct k_thread snoop_thread_handler;
k_tid_t snoop_tid;

struct k_mutex snoop_mutex;

void snoop_init(){
  snoop_dev = device_get_binding(DT_LABEL(DT_NODELABEL(snoop)));
  if (!snoop_dev) {
    printk("No snoop device found\n");
    return;
  }

  if (k_mutex_init(&snoop_mutex)) {
    printk("<error> Snoop mutex init - failed!\n");
    return;
  }

  return;
}

void copy_snoop_read_buffer( uint8_t offset ,int size_num, uint8_t *buffer ){
  if ( ! k_mutex_lock(&snoop_mutex, K_MSEC(1000)) ){
    memcpy( &buffer[0] , &snoop_read_buffer[offset] , size_num - offset );
    memcpy( &buffer[ size_num - offset ] , &snoop_read_buffer[0] , offset );
  }else{
    printk("copy snoop buffer lock fail\n");
  }
  if ( k_mutex_unlock(&snoop_mutex) ){
    printk("copy snoop buffer unlock fail\n");
  }
}

bool get_postcode_ok() {
  return proc_postcode_ok;
}

void reset_postcode_ok() {
  proc_postcode_ok = false;
}

typedef struct bic_send_postcode_work {
    struct k_work work_id;
    ipmi_msg work_msg;
    uint8_t msg_idx;
    uint8_t msg_over_flag;
} bic_send_postcode_work_t;

void bic_send_postcode(struct k_work *work) {
  bic_send_postcode_work_t *work_info = CONTAINER_OF(work, bic_send_postcode_work_t, work_id);

  if ( ipmb_read(&(work_info->work_msg), IPMB_inf_index_map[work_info->work_msg.InF_target]) )
    printk("<error> POST CODE transfer to BMC success with byte[%d-%d]: 0x%x failed!\n",  work_info->msg_idx, work_info->msg_over_flag, work_info->work_msg.data[4]);

  free(work_info);
}

void snoop_read(){
  int rc;
  snoop_read_buffer = malloc( sizeof(uint8_t) * SNOOP_MAX_LEN );
  if (!snoop_read_buffer)
    return;

  while (1) {
    rc = snoop_aspeed_read(snoop_dev, 0, snoop_data, true);
    if (rc == 0){
      proc_postcode_ok = true;
      if ( ! k_mutex_lock(&snoop_mutex, K_MSEC(1000)) ){
        if (snoop_read_num[0] == SNOOP_MAX_LEN) {
          snoop_read_num[0] = 0;
          snoop_read_num[1] = 1;
        }

        snoop_read_buffer[ snoop_read_num[0]++ ] = *snoop_data;

        ipmi_msg msg;
        memset(&msg, 0, sizeof(ipmi_msg));

        msg.InF_source = Self_IFs;
        msg.InF_target = BMC_IPMB_IFs;
        msg.netfn = 0x38;
        msg.cmd = 0x08;
        msg.data[0] = 0x9C;
        msg.data[1] = 0x9C;
        msg.data[2] = 0x00;
        msg.data[3] = 1;
        msg.data[4] = *snoop_data;
        msg.data_len = 5;

        bic_send_postcode_work_t *new_job;
        new_job = malloc(sizeof(bic_send_postcode_work_t));
        if (!new_job) {
          printk("<error> bic_send_postcode_work malloc fail\n");
          continue;
        }
        memset(new_job, 0, sizeof(bic_send_postcode_work_t));
        new_job->msg_idx = snoop_read_num[0];
        new_job->msg_over_flag = snoop_read_num[1];
        new_job->work_msg = msg;

        k_work_init(&new_job->work_id, bic_send_postcode);
        k_work_submit(&new_job->work_id);
      }else{
        printk("snoop read lock fail\n");
      }
      if ( k_mutex_unlock(&snoop_mutex) ){
        printk("snoop read unlock fail\n");
      }
    }
  }
}

void snoop_start_thread(){
  snoop_init();
  if ( snoop_tid != NULL ){
    k_thread_abort(snoop_tid);
  }
  snoop_tid = k_thread_create(&snoop_thread_handler, snoop_thread,
                              K_THREAD_STACK_SIZEOF(snoop_thread),
                              snoop_read, NULL, NULL, NULL,
                              CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
  k_thread_name_set(&snoop_thread_handler, "snoop_thread");
}
