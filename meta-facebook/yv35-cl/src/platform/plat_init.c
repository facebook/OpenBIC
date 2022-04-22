#include "hal_gpio.h"
#include "hal_peci.h"
#include "power_status.h"
#include "util_sys.h"
#include "plat_class.h"
#include "plat_gpio.h"

SCU_CFG scu_cfg[] = {
	//register    value
	{ 0x7e6e2610, 0xffffffff },
	{ 0x7e6e2614, 0xffffffff },
	{ 0x7e6e2618, 0x30000000 },
	{ 0x7e6e261c, 0x00000F04 },
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
	set_post_status(FM_BIOS_POST_CMPLT_BMC_N);
	set_CPU_power_status(PWRGD_CPU_LVC3);
	set_post_thread();
	set_sys_ready_pin(BIC_READY);
}

int switch_spi_mux(const struct device *args)
{
	gpio_set(FM_SPI_PCH_MASTER_SEL_R, GPIO_LOW);
	return 1;
}

#define DEF_PROJ_GPIO_PRIORITY 78

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);

#define SWITCH_SPI_MUX_PRIORITY 81 // right after spi driver init

DEVICE_DEFINE(PRE_SWITCH_SPI_MUX, "PRE_SWITCH_SPI_MUX_NAME", &switch_spi_mux, NULL, NULL, NULL,
	      POST_KERNEL, SWITCH_SPI_MUX_PRIORITY, NULL);
