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

void device_init(){
  adc_init();
  peci_init();
}

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
  device_init();
}

