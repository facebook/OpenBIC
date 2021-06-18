/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include "cmsis_compiler.h"
#include "common.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "FreeRTOS_CLI.h"
#include "log.h"
#include "wait.h"
#ifdef CONFIG_DEVICE_WDT
#include "wdt_aspeed.h"
#endif
#include "pal.h"

#ifdef CONFIG_LIBRARIES_CMBACKTRACE
#include "cm_backtrace.h"
#ifdef CONFIG_AST2600_SERIES
	#define FIRMWARE_NAME "ast2600"
	#define HARDWARE_VERSION "a1"
#else
	#define FIRMWARE_NAME "ast1030"
	#define HARDWARE_VERSION "a0"
#endif 
#define SOFTWARE_VERSION "v1.0"
#endif

//Task stack size section
#define CONSOLE_CLI_STACK_SIZE	(configMINIMAL_STACK_SIZE * 2UL)

//Task priority section
#define CONSOLE_CLI_TASK_PRIORITY	( tskIDLE_PRIORITY )//(configMAX_PRIORITIES - 2)

extern void vRegisterSampleCLICommands( void );
extern void vUARTCommandConsoleStart( uint16_t usStackSize, UBaseType_t uxPriority );
#ifdef CONFIG_CMD_MEM
extern void register_mem_commands(void);
#endif
#ifdef CONFIG_CMD_CLK
extern void register_clk_commands(void);
#endif
#ifdef CONFIG_DEMO_TASK_SW
extern void demo_task_sw_init(void);
#endif
#ifdef CONFIG_DEMO_I3C
extern void demo_i3c_init(void);
#endif
#ifdef CONFIG_DEMO_I2C
extern void demo_i2c_init(void);
#endif
#ifdef CONFIG_DEMO_ESPI
extern void demo_espi_init(void);
#endif
#ifdef CONFIG_DEMO_PECI
extern void demo_peci_init(void);
#endif
#ifdef CONFIG_DEMO_ADC
extern void demo_adc_init(void);
#endif
#ifdef CONFIG_DEMO_KCS
extern void demo_kcs_init(void);
#endif
#ifdef CONFIG_DEMO_BT
extern void demo_bt_init(void);
#endif
#ifdef CONFIG_DEMO_SNOOP
extern void demo_snoop_init(void);
#endif
#ifdef CONFIG_DEMO_PCC
extern void demo_pcc_init(void);
#endif
#ifdef CONFIG_DEMO_USB
extern void demo_usb_init(void);
#endif

#ifdef CONFIG_DEMO_PWM_TACH
extern void demo_pwm_tach_init(void);
#endif

#ifdef CONFIG_DEMO_JTAG
extern void demo_jtag_init(void);
#endif

#ifdef CONFIG_DEMO_GPIO
extern void demo_gpio_init(void);
#endif

#ifdef CONFIG_DEMO_CMB
extern void demo_cmb_init(void);
#endif

#ifdef CONFIG_DEMO_NET
extern void demo_net_init(void);
#endif

#ifdef CONFIG_DEMO_FMC_SPI
extern void demo_fmc_spi_flash_init(void);
#endif

#ifdef CONFIG_DEMO_VUART
extern void demo_vuart_init(void);
#endif

__WEAK void board_early_init(void)
{

}

__WEAK void board_show_info(void)
{

}

void vApplicationTickHook (void)
{
#ifdef CONFIG_DEVICE_WDT
	wdt_reload();
#endif
}

void vApplicationDaemonTaskStartupHook(void)
{
#ifdef CONFIG_DEVICE_WDT
	wdt_init();
	wdt_enable();
#endif
}

int main()
{
#ifdef CONFIG_LIBRARIES_LOG
	log_init();
#endif

	board_early_init();
	board_show_info();

	osKernelInitialize();
#ifdef CONFIG_LIBRARIES_CMBACKTRACE
	cm_backtrace_init(FIRMWARE_NAME, HARDWARE_VERSION, SOFTWARE_VERSION);
#endif

#ifdef CONFIG_DEMO_GPIO
	demo_gpio_init();
#endif

#ifdef CONFIG_DEMO_CMB
	demo_cmb_init();
#endif

#ifdef CONFIG_DEMO_TASK_SW
	demo_task_sw_init();
#endif

#ifdef CONFIG_DEMO_I3C
	demo_i3c_init();
#endif	

#ifdef CONFIG_DEMO_I2C
	demo_i2c_init();
#endif

#ifdef CONFIG_DEMO_ESPI
	demo_espi_init();
#endif

#ifdef CONFIG_DEMO_PECI
	demo_peci_init();
#endif

#ifdef CONFIG_DEMO_ADC
	demo_adc_init();
#endif

#ifdef CONFIG_DEMO_KCS
	demo_kcs_init();
#endif

#ifdef CONFIG_DEMO_BT
	demo_bt_init();
#endif

#ifdef CONFIG_DEMO_SNOOP
	demo_snoop_init();
#endif

#ifdef CONFIG_DEMO_PCC
	demo_pcc_init();
#endif

#ifdef CONFIG_DEMO_PWM_TACH
	demo_pwm_tach_init();
#endif

#ifdef CONFIG_DEMO_JTAG
	demo_jtag_init();
#endif

#ifdef CONFIG_DEMO_USB
	demo_usb_init();
#endif

#ifdef CONFIG_DEMO_FMC_SPI
	demo_fmc_spi_flash_init();
#endif

#ifdef CONFIG_DEMO_RSA
  demo_rsa_init();
#endif

#ifdef CONFIG_DEMO_NET
	demo_net_init();
#endif

#ifdef CONFIG_DEMO_VUART
  demo_vuart_init();
#endif

	vUARTCommandConsoleStart(CONSOLE_CLI_STACK_SIZE, CONSOLE_CLI_TASK_PRIORITY);
	vRegisterSampleCLICommands();
#ifdef CONFIG_CMD_MEM
	register_mem_commands();
#endif
#ifdef CONFIG_CMD_CLK
	register_clk_commands();
#endif
	pal_BIC_init();

	aspeed_wait_init_timer(1);
	osKernelStart();
	/* Should not get here unless we did not have enough memory to start the
	 scheduler. */
	for (;;)
		;
	return 0;
}
