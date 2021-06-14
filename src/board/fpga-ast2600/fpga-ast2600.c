#include <stdio.h>
#include "common.h"
#include "uart_aspeed.h"
#include "clk_aspeed.h"
#include "device.h"
#include "serial_api.h"

#define STDIO_UART_DEVICE			CONCAT(uart, CONFIG_STDIO_UART)

DECLARE_DEV_CLK(uart6, SCU_BASE + 0x94, SCU_BASE + 0x90, BIT(23));
DECLARE_DEV_RESET(uart6, 0, 0, 0);
DECLARE_DEV(uart6, ASPEED_DEV_UART6, UART6_BASE, NULL);

serial_t stdio_uart = { .device = &STDIO_UART_DEVICE};

void board_early_init(void)
{
	serial_init_direct(&stdio_uart, NULL);
}


void board_show_info(void)
{
	printf("-----------------------------------------------------------\n");
	printf("Aspeed miniBMC\n");
	printf("board name: %s\n", CONFIG_BOARD_NAME);
	printf("build time: %s %s\n", __TIME__, __DATE__);
	printf("-----------------------------------------------------------\n");
}