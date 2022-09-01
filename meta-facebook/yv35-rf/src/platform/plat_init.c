#include "hal_gpio.h"
#include "plat_gpio.h"
#include "expansion_board.h"
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
	set_DC_on_delayed_status();
	set_DC_off_delayed_status();
}

void pal_post_init()
{
	k_usleep(100);

	gpio_set(ASIC_DEV_RST_N, GPIO_HIGH);
}

#define DEF_PROJ_GPIO_PRIORITY 61

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);
