#include <stdio.h>
#include "log.h"
#include "irq_aspeed.h"
#include "flash_api.h"
#include "serial_api.h"
#include "usb_api.h"
#include "gpio_aspeed.h"
#include "sgpiom_aspeed.h"
#include "board_device.h"
#include "pinctrl_aspeed.h"
#include "timer_aspeed.h"
#include "wait.h"
#include "cache_aspeed.h"
#include "scu_info_aspeed.h"
#include "version.h"

extern uint32_t __ram_start;
extern uint32_t __ram_end;
extern uint32_t __ram_nc_start;
extern uint32_t __ram_nc_end;

serial_t stdio_uart = { .device = &STDIO_UART_DEVICE};

gpio_t gpio[] = {
#ifdef CONFIG_DEVICE_GPIO
	{.device = &gpio0, .init = aspeed_gpio_init},
#endif
#ifdef CONFIG_DEVICE_SGPIOM
	{.device = &sgpiom0, .init = aspeed_sgpiom_init},
#endif
};

usb_t usb[] = {
#ifdef CONFIG_DEVICE_USB
	{.device = &usb_dev},
#endif
};

#ifdef CONFIG_DEVICE_FMC_SPI
spi_t fmc_spi[] = {
	{.device = &fmc_dev},
	{.device = &spi1_dev},
	{.device = &spi2_dev},
};
#endif

void board_early_init(void)
{
	int i;
	aspeed_irq_init();
	aspeed_wait_init_timer(0);
	serial_init_direct(&stdio_uart, NULL);
	/* Setting all of function group */
	aspeed_pinctrl_init();
	aspeed_cache_init(1);
	for (i = 0; i < sizeof(gpio) / sizeof(gpio[0]); i++) {
		gpio[i].init(&gpio[i]);
	}
	for (i = 0; i < sizeof(usb) / sizeof(usb[0]); i++) {
		usb_init(&usb[i]);
	}

#ifdef CONFIG_DEVICE_FMC_SPI
	aspeed_flash_probe(&fmc_spi[1], 0);
#endif
}

void board_show_info(void)
{
	printf("-----------------------------------------------------------\n");
	printf("board name: %s\n", CONFIG_BOARD_NAME, GIT_VERSION);
	printf("version: %s\n", GIT_VERSION);
	aspeed_print_soc_id();
	aspeed_print_sysrst_info();
	aspeed_print_security_info();
	aspeed_print_2nd_wdt_mode();
	aspeed_print_fmc_aux_ctrl();
	aspeed_print_spi1_abr_mode();
	aspeed_print_spi1_aux_ctrl();
	aspeed_print_spi_strap_mode();
	aspeed_print_espi_mode();
	printf("build date: %s - %s\n", __DATE__, __TIME__);
	printf("-----------------------------------------------------------\n");
	log_info("    cached SRAM: %08x - %08x\n", (uint32_t)&__ram_start, (uint32_t)&__ram_end - 1);
	log_info("non-cached SRAM: %08x - %08x\n", (uint32_t)&__ram_nc_start, (uint32_t)&__ram_nc_end - 1);
}
