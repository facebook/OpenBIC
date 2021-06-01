#include <stdio.h>
#include "log.h"
#include "irq_aspeed.h"
#include "serial_api.h"
#include "board_device.h"
#include "timer_aspeed.h"
#include "wait.h"
#include "cache_aspeed.h"

extern uint32_t __ram_start;
extern uint32_t __ram_end;
extern uint32_t __ram_nc_start;
extern uint32_t __ram_nc_end;

serial_t stdio_uart = { .device = &STDIO_UART_DEVICE};

void board_early_init(void)
{
	aspeed_irq_init();
	aspeed_wait_init_timer(0);
	serial_init_direct(&stdio_uart, NULL);
	aspeed_cache_init(1);
}


void board_show_info(void)
{
	printf("-----------------------------------------------------------\n");
	printf("Aspeed miniBMC\n");
	printf("board name: %s\n", CONFIG_BOARD_NAME);
	printf("build time: %s %s\n", __TIME__, __DATE__);
	printf("-----------------------------------------------------------\n");
	log_info("    cached SRAM: %08x - %08x\n", (uint32_t)&__ram_start, (uint32_t)&__ram_end - 1);
	log_info("non-cached SRAM: %08x - %08x\n", (uint32_t)&__ram_nc_start, (uint32_t)&__ram_nc_end - 1);
}