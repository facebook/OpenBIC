#include "hal_gpio.h"
#include "plat_gpio.h"
#include "plat_class.h"
#include "plat_isr.h"
#include "plat_power_seq.h"
#include "power_status.h"

void pal_pre_init()
{
	init_platform_config();
}

void pal_set_sys_status()
{
	set_MB_DC_status(FM_POWER_EN);
	set_DC_status(PWRGD_CARD_PWROK);
	control_power_sequence();
}

#define DEF_PROJ_GPIO_PRIORITY 61

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);
