#include <zephyr.h>
#include <sys/printk.h>
#include <stdio.h>
#include <string.h>
#include "usb.h"
#include "ipmi.h"
#include "pal.h"

void pal_usb_handler(uint8_t *rx_buff,int rx_len) {
  ipmi_msg_cfg current_msg;

  if (DEBUG_USB) {
    printk("USB: len %d, req: %x %x ID: %x %x %x target: %x offset: %x %x %x %x len: %x %x\n", 
      rx_len, rx_buff[0], rx_buff[1], rx_buff[2], rx_buff[3], rx_buff[4], rx_buff[5], rx_buff[6], 
      rx_buff[7], rx_buff[8], rx_buff[9], rx_buff[10], rx_buff[11]);
  }

  current_msg.buffer.netfn = rx_buff[0] >> 2;
  current_msg.buffer.cmd = rx_buff[1];
  current_msg.buffer.InF_source = BMC_USB_IFs;
  current_msg.buffer.data_len = rx_len - 2; // skip netfn, cmd
  memcpy( &current_msg.buffer.data[0], &rx_buff[2], current_msg.buffer.data_len);
  while (k_msgq_put(&ipmi_msgq, &current_msg, K_NO_WAIT) != 0) {
    k_msgq_purge(&ipmi_msgq);
    printf("KCS retrying put ipmi msgq\n");
  }

  return;
}
