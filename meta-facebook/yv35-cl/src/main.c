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
#include "kcs.h"
#include "plat_func.h"

void device_init() {
  adc_init();
  peci_init();
}

void set_sys_status() {
  gpio_set(FM_SPI_PCH_MASTER_SEL_R, GPIO_LOW);
  gpio_set(BIC_READY, GPIO_HIGH);
  set_DC_status();
  set_post_status();
  set_SCU_setting();
}

void main(void)
{
  uint8_t proj_stage = (FIRMWARE_REVISION_1 & 0xf0) >> 4;
  printk("Hello, wellcome to yv35 craterlake POC %d\n", FIRMWARE_REVISION_2);

  util_init_timer();
  util_init_I2C();

  set_sys_config();
  disable_asd_gpio_interrupt();
  sensor_init();
  FRU_init();
  ipmi_init();
  kcs_init();
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
