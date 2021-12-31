#include <zephyr.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/printk.h>
#include <device.h>
#include "cmsis_os.h"
#include "hal_snoop.h"

const struct device *snoop_dev;
uint8_t *snoop_data;
static uint8_t *snoop_read_buffer;
int snoop_read_num = 0;
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
        snoop_read_buffer[ snoop_read_num % SNOOP_MAX_LEN ] = *snoop_data;
        snoop_read_num++;
      }else{
        printk("snoop read lock fail\n");
      }
      if ( k_mutex_unlock(&snoop_mutex) ){
        printk("snoop read unlock fail\n");
      }
    }
  }
}

void free_snoop_buffer(){
  if ( snoop_read_buffer != NULL ){
    free(snoop_read_buffer);
  }
  snoop_read_num = 0;
}

void snoop_start_thread(){
  snoop_init();
  if ( snoop_tid != NULL ){
    k_thread_abort(snoop_tid);
  }
  snoop_tid = k_thread_create(&snoop_thread_handler, snoop_thread,
                              K_THREAD_STACK_SIZEOF(snoop_thread),
                              snoop_read, NULL, NULL, NULL,
                              osPriorityBelowNormal, 0, K_NO_WAIT);
  k_thread_name_set(&snoop_thread_handler, "snoop_thread");
}

void snoop_abort_thread(){
  k_thread_abort(snoop_tid);
}
