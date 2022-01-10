/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <kernel.h>
#include <cmsis_os2.h>
#include <sys/printk.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "plat_gpio.h"
#include "ipmi_def.h"
#include "ipmi.h"

void device_init() {
  adc_init();
  hsc_init();
}

void set_sys_status() {
  gpio_set(BIC_READY_R, GPIO_HIGH);
}

void main(void)
{
  uint8_t proj_stage = (FIRMWARE_REVISION_1 & 0xf0) >> 4;
  printk("Hello, welcome to yv35 baseboard %x%x.%x.%x\n", BIC_FW_YEAR_MSB, BIC_FW_YEAR_LSB, BIC_FW_WEEK, BIC_FW_VER);

  util_init_timer();
  util_init_I2C();

  sensor_init();
  FRU_init();
  ipmi_init();
  usb_dev_init();
  device_init();
  set_sys_status();

}

#define DEF_PROJ_GPIO_PRIORITY 61

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME",
        &gpio_init, NULL,
        NULL, NULL,
        POST_KERNEL, DEF_PROJ_GPIO_PRIORITY,
        NULL);
