#include <zephyr.h>
#include <sys/printk.h>
#include <stdio.h>
#include <string.h>
#include "usb.h"
#include "ipmi.h"
#include "pal.h"

#define FWUPDATE_HEADER_SIZE 12
#define SIZE_NETFN_CMD 2

void pal_usb_handler(uint8_t *rx_buff,int rx_len) {
  uint16_t record_offset;
  static ipmi_msg_cfg current_msg;
  static bool fwupdate_keep_data = 0;
  static uint16_t keep_data_len = 0;
  static uint16_t fwupdate_data_len = 0;

  if (DEBUG_USB) {
    printk("USB: len %d, req: %x %x ID: %x %x %x target: %x offset: %x %x %x %x len: %x %x\n", 
      rx_len, rx_buff[0], rx_buff[1], rx_buff[2], rx_buff[3], rx_buff[4], rx_buff[5], rx_buff[6], 
      rx_buff[7], rx_buff[8], rx_buff[9], rx_buff[10], rx_buff[11]);
  }

  // USB driver must receive 64 byte package from bmc
  // it takes 512 + 64 byte package to receive ipmi command + 512 byte image data
  // if cmd fw_update, record next usb package as image until receive complete data
  if ( (rx_buff[0] == (NETFN_OEM_1S_REQ << 2) ) && (rx_buff[1] == CMD_OEM_1S_FW_UPDATE) ) {
    fwupdate_keep_data = true;
  }

  if ( fwupdate_keep_data ) {
    if ( (keep_data_len + rx_len) > IPMI_DATA_MAX_LENGTH ) {
      printk("usb fw update recv data over ipmi buff size %d, keep %d, recv %d\n", IPMI_DATA_MAX_LENGTH, keep_data_len, rx_len);
      keep_data_len = 0;
      fwupdate_keep_data = false;
      return;
    } else if (!keep_data_len) { // only fill up ipmb buffer from first package
      current_msg.buffer.netfn = rx_buff[0] >> 2;
      current_msg.buffer.cmd = rx_buff[1];
      current_msg.buffer.InF_source = BMC_USB_IFs;
      current_msg.buffer.data_len = rx_len - SIZE_NETFN_CMD;
      fwupdate_data_len = ( (rx_buff[11] << 8) | rx_buff[10] );
      memcpy( &current_msg.buffer.data[0], &rx_buff[SIZE_NETFN_CMD], (rx_len - SIZE_NETFN_CMD));
      keep_data_len = rx_len - FWUPDATE_HEADER_SIZE;
    } else {
      record_offset = keep_data_len + FWUPDATE_HEADER_SIZE - SIZE_NETFN_CMD;
      memcpy( &current_msg.buffer.data[record_offset], &rx_buff[0], rx_len);
      current_msg.buffer.data_len += rx_len;
      keep_data_len += rx_len;
    }
    if (keep_data_len == fwupdate_data_len) {
      while (k_msgq_put(&ipmi_msgq, &current_msg, K_NO_WAIT) != 0) {
        k_msgq_purge(&ipmi_msgq);
        printf("KCS retrying put ipmi msgq\n");
      }
      keep_data_len = 0;
      fwupdate_data_len = 0;
      fwupdate_keep_data = false;
    }
  } else {
    current_msg.buffer.netfn = rx_buff[0] >> 2;
    current_msg.buffer.cmd = rx_buff[1];
    current_msg.buffer.InF_source = BMC_USB_IFs;
    current_msg.buffer.data_len = rx_len - SIZE_NETFN_CMD;
    memcpy( &current_msg.buffer.data[0], &rx_buff[2], current_msg.buffer.data_len);
    while (k_msgq_put(&ipmi_msgq, &current_msg, K_NO_WAIT) != 0) {
      k_msgq_purge(&ipmi_msgq);
      printf("KCS retrying put ipmi msgq\n");
    }
  }

  return;
}
