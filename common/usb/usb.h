#ifndef USB_H
#define USB_H

#define DEBUG_USB 0
#define USB_HANDLER_STACK_SIZE 2000
#define RX_BUFF_SIZE    64
#define RING_BUF_SIZE   576


void usb_slavedev_init(void);

#endif
