#include <stdio.h>
#include "pinctrl_aspeed.h"
#include "irq_aspeed.h"
#include "serial_api.h"
#include "usb_api.h"
#include "gpio_aspeed.h"
#include "sgpiom_aspeed.h"
#include "board_device.h"
#include "ipi_aspeed.h"
#include "cache_aspeed.h"
#include "scu_info_aspeed.h"

extern uint32_t __ram_start;
extern uint32_t __ram_end;
extern uint32_t __ram_nc_start;
extern uint32_t __ram_nc_end;

serial_t stdio_uart = { .device = &STDIO_UART_DEVICE};
#if CONFIG_DEVICE_IPI	
ipi_t mailbox = {.device = &ipi};
#endif

gpio_t gpio[] = {
#ifdef CONFIG_DEVICE_GPIO
	{.device = &gpio0, .init = aspeed_gpio_init},
	{.device = &gpio1, .init = aspeed_gpio_init},
#endif
#ifdef CONFIG_DEVICE_SGPIOM
	{.device = &sgpiom0, .init = aspeed_sgpiom_init},
	{.device = &sgpiom1, .init = aspeed_sgpiom_init},
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

#define SOC_REV_AST2605_HI		0x05010103
#define SOC_REV_AST2605_LO		0x05020103

void aspeed_serial_pinmux_setting(uint32_t fun_id)
{
	struct aspeed_fun_desc *fun_desc = aspeed_fun_desc_table[fun_id];
	struct aspeed_sig_desc *sig_desc;
	struct aspeed_sig_en *sig_en;
	uint16_t sig_id;
	int sig_number;
	int sig_idx;
	int sig_en_number;
	int sig_en_idx;
	sig_number = fun_desc->nsignal;
	for (sig_idx = 0; sig_idx < sig_number; sig_idx++) {
		sig_id = fun_desc->sig_id_list[sig_idx];
		sig_desc = aspeed_sig_desc_table[sig_id];
		sig_en_number = sig_desc->nsig_en;
		for (sig_en_idx = 0; sig_en_idx < sig_en_number; sig_en_idx++) {
			sig_en = &sig_desc->sig_en[sig_en_idx];
			if (sig_en->op) {
				SCU_WR(sig_en->offset, SCU_RD(sig_en->offset) & ~BIT(sig_en->bits));
			} else {
				SCU_WR(sig_en->offset, SCU_RD(sig_en->offset) | BIT(sig_en->bits));
			}
		}
	}
}

void board_early_init(void)
{
	int i;
	aspeed_irq_init();
	aspeed_serial_pinmux_setting(GET_FUN_ID(CONCAT(UART, CONFIG_STDIO_UART)));
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
#if CONFIG_DEVICE_IPI	
	aspeed_ipi_init(&mailbox);
#endif
}

void board_show_info(void)
{
	printf("-----------------------------------------------------------\n");
	printf("board name: %s\n", CONFIG_BOARD_NAME);	
	aspeed_print_soc_id();
	aspeed_print_sysrst_info();
	aspeed_print_security_info();
	aspeed_print_2nd_wdt_mode();
	if (aspeed_print_fmc_aux_ctrl)
		aspeed_print_fmc_aux_ctrl();
	if (aspeed_print_spi1_abr_mode)
		aspeed_print_spi1_abr_mode();
	if (aspeed_print_spi1_aux_ctrl)
		aspeed_print_spi1_aux_ctrl();
	aspeed_print_spi_strap_mode();
	aspeed_print_espi_mode();
	printf("build time: %s %s\n", __TIME__, __DATE__);
	printf("-----------------------------------------------------------\n");
}