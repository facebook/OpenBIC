#include "hal_gpio.h"
#include "plat_gpio.h"
#include "plat_class.h"
#include "power_status.h"



void pal_pre_init()
{
	
}

void pal_set_sys_status()
{

}

#define DEF_PROJ_GPIO_PRIORITY 61

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);
