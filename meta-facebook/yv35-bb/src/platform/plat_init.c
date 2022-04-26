#include "util_sys.h"
#include "hal_gpio.h"
#include "plat_gpio.h"
#include "plat_fan.h"
#include <plat_def.h>

void pal_pre_init()
{
	init_fan_mode();

	// Due to BB CPLD bind HSC device need times
	// wait HSC ready before sensor read
	k_msleep(HSC_DEVICE_READY_DELAY_MS);
}

void pal_set_sys_status()
{
	set_sys_ready_pin(BIC_READY_R);
}

#define DEF_PROJ_GPIO_PRIORITY 61

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);
