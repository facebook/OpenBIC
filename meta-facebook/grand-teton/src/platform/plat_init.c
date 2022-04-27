#include "hal_gpio.h"
#include "hal_peci.h"
#include "power_status.h"
#include "util_sys.h"
#include "plat_class.h"
#include "plat_gpio.h"

SCU_CFG scu_cfg[] = {
	//register    value
};

void pal_pre_init()
{
}

void pal_post_init()
{
}

void pal_set_sys_status()
{
  gpio_set(BIC_SYS_READY_N, GPIO_LOW);
}

#define DEF_PROJ_GPIO_PRIORITY 78

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);
