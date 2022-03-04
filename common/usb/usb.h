#ifndef USB_H
#define USB_H

#include "ipmb.h"

#define DEBUG_USB 0
#define USB_HANDLER_STACK_SIZE 2000
#define RX_BUFF_SIZE    64
#define RING_BUF_SIZE   576

void usb_dev_init(void);
void usb_slavedev_init(void);
void USB_write(ipmi_msg *ipmi_resp);

#endif
