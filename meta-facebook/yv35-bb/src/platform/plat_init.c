#include "hal_gpio.h"
#include "adc.h"
#include "adm1278.h"
#include "plat_gpio.h"

void pal_device_init()
{
	adc_init();
	adm1278_init();
}

void pal_set_sys_status()
{
	gpio_set(BIC_READY_R, GPIO_HIGH);
}

#define DEF_PROJ_GPIO_PRIORITY 61

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);
