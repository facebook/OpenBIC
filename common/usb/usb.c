#include <device.h>
#include <drivers/uart.h>
#include <usb/usb_device.h>
#include <logging/log.h>
#include <sys/ring_buffer.h>
#include "ipmi.h"
#include "usb.h"
#include "pal.h"

RING_BUF_DECLARE(ringbuf, RING_BUF_SIZE);

static struct k_sem usbhandle_sem;
static const struct device *dev;

struct k_thread USB_handler;
K_KERNEL_STACK_MEMBER(USB_handler_stack, USB_HANDLER_STACK_SIZE);

void USB_write(ipmi_msg *ipmi_resp) {
  uint8_t tx_buf[RX_BUFF_SIZE];
  struct ipmi_response *resp = (struct ipmi_response *)tx_buf;

  pack_ipmi_resp(resp, ipmi_resp);

  if ( DEBUG_USB ) {
    int i;
    printk("usb resp: %x %x %x, ", resp->netfn, resp->cmd, resp->cmplt_code);
    for (i = 0; i < ipmi_resp->data_len; i++)
      printk("0x%x ", resp->data[i]);
    printk("\n");
  }
  uart_fifo_fill(dev, tx_buf, ipmi_resp->data_len + 3); // return netfn + cmd + comltcode + data
}

static void USB_handle(void *arug0, void *arug1, void *arug2)
{
  uint8_t rx_buff[RX_BUFF_SIZE];
  int rx_len;
  int i;

  while (1) {
    k_sem_take(&usbhandle_sem, K_FOREVER);
    rx_len = ring_buf_get(&ringbuf, rx_buff, sizeof(rx_buff));
    if (!rx_len) {
      k_msleep(10);
      continue;
    }

    if ( DEBUG_USB ) {
      printk("Print Data: ");
      for (i = 0; i < rx_len; i++)
        printk("0x%x ", rx_buff[i]);
      printk("\n");
    }
    pal_usb_handler(rx_buff, rx_len);
  }
}

static void interrupt_handler(const struct device *dev, void *user_data)
{
  uint8_t rx_buff[RX_BUFF_SIZE];
  int recv_len, rx_len;

  ARG_UNUSED(user_data);

  while (uart_irq_is_pending(dev) && uart_irq_rx_ready(dev)) {
    recv_len = uart_fifo_read(dev, rx_buff, sizeof(rx_buff));

    if (recv_len) {
      rx_len = ring_buf_put(&ringbuf, rx_buff, recv_len);
      if (rx_len < recv_len) {
        printk("Drop %u bytes\n", recv_len - rx_len);
      } else {
        k_sem_give(&usbhandle_sem);
      }
    }
  }
}

void usb_dev_init(void) {
  int ret;

  ret = usb_enable(NULL);
  if (ret) {
    printk("Failed to enable USB");
    return;
  }

  dev = device_get_binding("CDC_ACM_0");
  if (!dev) {
    printk("CDC ACM device not found");
    return;
  }

  uart_irq_callback_set(dev, interrupt_handler);

  /* Enable rx interrupts */
  uart_irq_rx_enable(dev);

  k_thread_create(&USB_handler, USB_handler_stack,
                  K_THREAD_STACK_SIZEOF(USB_handler_stack),
                  USB_handle,
                  NULL, NULL, NULL,
                  CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
  k_thread_name_set(&USB_handler, "USB_handler");

  uint8_t init_sem_count = RING_BUF_SIZE / RX_BUFF_SIZE;
  k_sem_init(&usbhandle_sem, 0, init_sem_count);
}
