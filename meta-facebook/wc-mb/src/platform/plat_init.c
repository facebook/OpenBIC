#include "hal_gpio.h"
#include "hal_peci.h"
#include "power_status.h"
#include "util_sys.h"
#include "plat_class.h"
#include "plat_gpio.h"

SCU_CFG scu_cfg[] = {
	//register    value
	/* Set GPIOC0-C1 and GPIOC6-C7 to passthrough mode after gpio init */
	{ 0x7e6e24b0, 0x00C30000 },
	/* Set GPIOA/B/C/D internal pull-up/down after gpio init */
	{ 0x7e6e2610, 0xFFFFFFFF },
	/* Set GPIOF/G/H internal pull-up/down after gpio init */
	{ 0x7e6e2614, 0xFFFFFFFF },
	/* Set GPIOJ/K/L internal pull-up/down after gpio init */
	{ 0x7e6e2618, 0xF0000000 },
	/* Set GPIOM/N/OP internal pull-up/down after gpio init */
	{ 0x7e6e261c, 0x0000000B },
	/* Set GPIOQ/R/S internal pull-up/down after gpio init */
	{ 0x7e6e2630, 0x00000007 },
	/* Set GPIOU/V/X internal pull-up/down after gpio init */
	{ 0x7e6e2634, 0x0000007D },
};

void pal_pre_init()
{
	init_platform_config();
	disable_PRDY_interrupt();
	scu_init(scu_cfg, sizeof(scu_cfg) / sizeof(SCU_CFG));
}

void pal_post_init()
{
	init_me_firmware();
}

void pal_set_sys_status()
{
	set_DC_status(PWRGD_SYS_PWROK);
	set_DC_on_delayed_status();
	set_DC_off_delayed_status();
	set_post_status(FM_BIOS_POST_CMPLT_BIC_N);
	set_CPU_power_status(PWRGD_CPU_LVC3);
	set_post_thread();
	set_sys_ready_pin(FM_BIC_READY);
}

#define DEF_PROJ_GPIO_PRIORITY 78

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);
