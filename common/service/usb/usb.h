#ifndef USB_H
#define USB_H

#ifdef CONFIG_USB

#define DEBUG_USB 0
#define USB_HANDLER_STACK_SIZE 2048
#define RX_BUFF_SIZE 64
#define RING_BUF_SIZE 576

#define FWUPDATE_HEADER_SIZE 12
#define SIZE_NETFN_CMD 2

#include "ipmb.h"

void usb_targetdev_init(void);
void usb_write_by_ipmi(ipmi_msg *ipmi_resp);
void usb_dev_init(void);

#endif

#endif
