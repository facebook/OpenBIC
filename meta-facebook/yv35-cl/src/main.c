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
#include "ipmi.h"
#include "usb.h"
#include "kcs.h"
#include "power_status.h"
#include "plat_class.h"
#include "plat_sys.h"

void device_init()
{
	adc_init();
	peci_init();
	hsc_init();
}

void switch_spi_mux()
{
	gpio_set(FM_SPI_PCH_MASTER_SEL_R, GPIO_LOW);
}

void set_sys_status()
{
	gpio_set(BIC_READY, GPIO_HIGH);
	set_DC_status(PWRGD_SYS_PWROK);
	set_DC_on_delayed_status();
	set_DC_off_delayed_status();
	set_post_status(FM_BIOS_POST_CMPLT_BMC_N);
	set_post_thread();
}

void main(void)
{
	uint8_t proj_stage = (FIRMWARE_REVISION_1 & 0xf0) >> 4;
	printk("Hello, wellcome to yv35 craterlake %x%x.%x.%x\n", BIC_FW_YEAR_MSB, BIC_FW_YEAR_LSB,
	       BIC_FW_WEEK, BIC_FW_VER);

	util_init_timer();
	util_init_I2C();

	init_platform_config();
	disable_PRDY_interrupt();
	sensor_init();
	FRU_init();
	ipmi_init();
	kcs_init();
	usb_dev_init();
	device_init();
	set_sys_status();
	scu_init();
	set_ME_restore();
}

#define DEF_PROJ_GPIO_PRIORITY 78

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);

#define SWITCH_SPI_MUX_PRIORITY 81 // right after spi driver init

DEVICE_DEFINE(PRE_SWITCH_SPI_MUX, "PRE_SWITCH_SPI_MUX_NAME", &switch_spi_mux, NULL, NULL, NULL,
	      POST_KERNEL, SWITCH_SPI_MUX_PRIORITY, NULL);
