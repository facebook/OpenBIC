#include <stdio.h>
#include "pinctrl_aspeed.h"
#include "irq_aspeed.h"
#include "serial_api.h"
#include "board_device.h"
#include "ipi_aspeed.h"
#include "cache_aspeed.h"

extern uint32_t __ram_start;
extern uint32_t __ram_end;
extern uint32_t __ram_nc_start;
extern uint32_t __ram_nc_end;

serial_t stdio_uart = { .device = &STDIO_UART_DEVICE};
#if CONFIG_DEVICE_IPI	
ipi_t mailbox = {.device = &ipi};
#endif

#define SOC_REV_AST2605_HI		0x05010103
#define SOC_REV_AST2605_LO		0x05020103
void init_soc_by_revision(void)
{
	uint32_t revision[2];

	revision[0] = SCU_RD(0x04);
	revision[1] = SCU_RD(0x14);

	printf("rev id: %08x %08x\n", revision[0], revision[1]);

	/* if AST2605, de-assert Cortex-A7 */
    if ((revision[0] == SOC_REV_AST2605_HI) &&
        (revision[1] == SOC_REV_AST2605_LO)) {
    	printf("Deassert Primary processor reset\n");
    	SCU_WR(0x504, BIT(0));
    }
}

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
	aspeed_irq_init();
	aspeed_serial_pinmux_setting(GET_FUN_ID(CONCAT(UART, CONFIG_STDIO_UART)));
	serial_init_direct(&stdio_uart, NULL);
	/* Setting all of function group */
	aspeed_pinctrl_init();
	aspeed_cache_init(1);
#if CONFIG_DEVICE_IPI	
	aspeed_ipi_init(&mailbox);
#endif

#if CONFIG_AST2600A2
	init_soc_by_revision();
#endif
}

void board_show_info(void)
{
	printf("-----------------------------------------------------------\n");
	printf("Aspeed miniBMC\n");
	printf("board name: %s\n", CONFIG_BOARD_NAME);
	printf("build time: %s %s\n", __TIME__, __DATE__);
	printf("-----------------------------------------------------------\n");
}