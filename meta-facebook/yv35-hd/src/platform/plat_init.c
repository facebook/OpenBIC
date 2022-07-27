#include "hal_gpio.h"
#include "power_status.h"
#include "util_sys.h"
#include "plat_gpio.h"
#include "plat_class.h"
#include "snoop.h"
#include "apml.h"

SCU_CFG scu_cfg[] = {
	//register    value
	{ 0x7e6e2610, 0xffffffff },
	{ 0x7e6e2614, 0xffffffff },
	{ 0x7e6e2618, 0xdc000000 },
	{ 0x7e6e261c, 0x00000F32 },
};

void pal_pre_init()
{
	init_platform_config();
	scu_init(scu_cfg, sizeof(scu_cfg) / sizeof(SCU_CFG));
	apml_init();
}

void pal_set_sys_status()
{
	set_DC_status(PWRGD_CPU_LVC3);
	set_DC_on_delayed_status();
	set_post_status(FM_BIOS_POST_CMPLT_BIC_N);
	set_CPU_power_status(PWRGD_CPU_LVC3);
	set_sys_ready_pin(BIC_READY);
}

#define DEF_PROJ_GPIO_PRIORITY 78

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);
