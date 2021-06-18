/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS_CLI.h"
#include "common.h"
#include "memory_map.h"
#include "device.h"
#include "device_id.h"
#include "clk_aspeed.h"

static BaseType_t do_clk_dump(char *pcWriteBuffer, size_t xWriteBufferLen,
			    const char *pcCommandString) 
{
	//const char *pcParameter;
	//BaseType_t xParameterStringLength, xReturn;
	//static UBaseType_t uxParameterNumber = 0;
	uint32_t i, ret;
	struct aspeed_device_s uart_test;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes
	the write buffer length is adequate, so does not check for buffer
	overflows. */
	(void)pcCommandString;
	(void)xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	for (i = ASPEED_DEV_UART0; i <= ASPEED_DEV_UART12; i++) {
		uart_test.dev_id = i;
 		ret = aspeed_clk_get_uart_clk(&uart_test);
		printf("uart%d clk: %u\n", i - ASPEED_DEV_UART0, ret);
	}

	printf("hclk:%d Hz\n", aspeed_clk_get_hclk());
	printf("pclk:%d Hz\n", aspeed_clk_get_pclk());
#ifdef CONFIG_AST2600_SERIES
	printf("apb2:%d Hz\n", aspeed_clk_get_apb2());
#endif

	//uxParameterNumber = 0;
	pcWriteBuffer[0] = 0x00;

	return pdFALSE;
}

static const CLI_Command_Definition_t clk_dump = 
{
	"clk_dump",
	"\r\nclk_dump:\r\n dump values of the system clock frequency\r\n",
	do_clk_dump,
	0
};

void register_clk_commands(void)
{
	FreeRTOS_CLIRegisterCommand(&clk_dump);
}
