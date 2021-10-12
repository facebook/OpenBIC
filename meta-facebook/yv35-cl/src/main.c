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
#include "plat_gpio.h"
#include "ipmi_def.h"
#include "ipmi.h"
#include "kcs.h"

void device_init(){
  adc_init();
  peci_init();
}

void set_sys_status() {
  gpio_set(BIC_READY, GPIO_HIGH);
}

void main(void)
{
  uint8_t proj_stage = (FIRMWARE_REVISION_1 & 0xf0) >> 4;
  printk("Hello, wellcome to yv35 craterlake POC %d\n", FIRMWARE_REVISION_2);

  util_init_timer();
  util_init_I2C();

  gpio_init();
  sensor_init();
  FRU_init();
  ipmi_init();
  kcs_init();
  usb_dev_init();
  device_init();
  set_sys_status();
}

