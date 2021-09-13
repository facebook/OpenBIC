/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <kernel.h>
#include <cmsis_os2.h>
#include <sys/printk.h>
#include "worker.h"
#include "sensor.h"
#include "hal_i2c.h"
#include "hal_gpio.h"
#include "ipmi.h"
#include "kcs.h"

void main(void)
{
  printk("Hello yv35 cl\n");

  util_init_timer();
  util_init_I2C();

  gpio_init();
  sensor_init();
  FRU_init();
  ipmi_init();
  kcs_init();
  usb_dev_init();

  ipmi_msg msg;
  while(0) {
    msg.data_len = 3;
    msg.InF_source = Self_IFs;
    msg.InF_target = BMC_IPMB_IFs;
    msg.netfn = NETFN_OEM_1S_REQ;
    msg.cmd = CMD_OEM_GET_GPIO;

    msg.data[0] = 0x9c;
    msg.data[1] = 0x9c;
    msg.data[2] = 0x0;
    ipmb_read(&msg, 0);
k_msleep(5);
  }
}
