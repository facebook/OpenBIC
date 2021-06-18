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
#include "wdt_aspeed.h"
#include "memory_map.h"

void print_buf(void *buf, uint32_t addr, uint32_t length)
{
	int i;
	size_t s_addr = 10, s_data = 9;

	for (i = 0; i < length; i++) {
		if (0 == i % 4) {
			sprintf(buf, "\r\n%08x: ", addr);
			buf += s_addr + 2;
		}
		sprintf(buf, "%08x ", readl(addr));
		addr += 4;
		buf += s_data;
	}
	sprintf(buf, "\r\n");
}

static BaseType_t do_mem_md(char *pcWriteBuffer, size_t xWriteBufferLen,
			    const char *pcCommandString) 
{
	const char *pcParameter;
	BaseType_t xParameterStringLength, xReturn;
	static UBaseType_t uxParameterNumber = 0;
	static uint32_t addr = 0, length = 0;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes
	the write buffer length is adequate, so does not check for buffer
	overflows. */
	(void)pcCommandString;
	(void)xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	if (uxParameterNumber == 0) {
		/* The first time the function is called after the command has
		been entered just a header string is returned. */
		sprintf(pcWriteBuffer, "\r");

		/* Next time the function is called the first parameter will be
		echoed back. */
		uxParameterNumber = 1U;

		/* There is more data to be returned as no parameters have been
		echoed back yet. */
		xReturn = pdPASS;
	} else {
		/* Obtain the parameter string. */
		pcParameter = FreeRTOS_CLIGetParameter(
		    pcCommandString,	    /* The command string itself. */
		    uxParameterNumber,	    /* Return the next parameter. */
		    &xParameterStringLength /* Store the parameter string
						   length. */
		);

		if (pcParameter != NULL) {
			/* Return the parameter string. */
			memset(pcWriteBuffer, 0x00, xWriteBufferLen);
			switch (uxParameterNumber) {
			case 1:
				addr = strtoul(pcParameter, NULL, 16);
				break;
			case 2:
				length = strtoul(pcParameter, NULL, 16);
				break;
			}

			if (uxParameterNumber == 2) {
				print_buf(pcWriteBuffer, addr, length);
				xReturn = pdFALSE;
				uxParameterNumber = 0;
			} else {
				/* There might be more parameters to return
				 * after this one. */
				xReturn = pdTRUE;
				uxParameterNumber++;
			}

		} else {
			/* No more parameters were found.  Make sure the write
			buffer does not contain a valid string. */
			pcWriteBuffer[0] = 0x00;

			/* No more data to return. */
			xReturn = pdFALSE;

			/* Start over the next time this command is executed. */
			uxParameterNumber = 0;
		}
	}

	return xReturn;
}
static const CLI_Command_Definition_t mem_md = 
{
	"md",
	"\r\nmd:\r\n memory display. usage: md address [# of objects]\r\n",
	do_mem_md,
	2
};

static BaseType_t do_mem_mw(char *pcWriteBuffer, size_t xWriteBufferLen,
			    const char *pcCommandString) 
{
	const char *pcParameter;
	BaseType_t xParameterStringLength, xReturn;
	static UBaseType_t uxParameterNumber = 0;
	static uint32_t addr = 0, value = 0;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes
	the write buffer length is adequate, so does not check for buffer
	overflows. */
	(void)pcCommandString;
	(void)xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	if (uxParameterNumber == 0) {
		/* The first time the function is called after the command has
		been entered just a header string is returned. */
		sprintf(pcWriteBuffer, "\r");

		/* Next time the function is called the first parameter will be
		echoed back. */
		uxParameterNumber = 1U;

		/* There is more data to be returned as no parameters have been
		echoed back yet. */
		xReturn = pdPASS;
	} else {
		/* Obtain the parameter string. */
		pcParameter = FreeRTOS_CLIGetParameter(
		    pcCommandString,	    /* The command string itself. */
		    uxParameterNumber,	    /* Return the next parameter. */
		    &xParameterStringLength /* Store the parameter string
						   length. */
		);

		if (pcParameter != NULL) {
			/* Return the parameter string. */
			memset(pcWriteBuffer, 0x00, xWriteBufferLen);
			switch (uxParameterNumber) {
			case 1:
				addr = strtoul(pcParameter, NULL, 16);
				break;
			case 2:
				value = strtoul(pcParameter, NULL, 16);
				break;
			}

			if (uxParameterNumber == 2) {
				writel(value, addr);
				xReturn = pdFALSE;
				uxParameterNumber = 0;
			} else {
				/* There might be more parameters to return
				 * after this one. */
				xReturn = pdTRUE;
				uxParameterNumber++;
			}

		} else {
			/* No more parameters were found.  Make sure the write
			buffer does not contain a valid string. */
			pcWriteBuffer[0] = 0x00;

			/* No more data to return. */
			xReturn = pdFALSE;

			/* Start over the next time this command is executed. */
			uxParameterNumber = 0;
		}
	}

	return xReturn;
}

static const CLI_Command_Definition_t mem_mw = 
{
	"mw",
	"\r\nmw:\r\n memory write. usage: mw address value\r\n",
	do_mem_mw,
	2
};

static BaseType_t do_wdt_reset(char *pcWriteBuffer, size_t xWriteBufferLen,
			    const char *pcCommandString) 
{
	const char *pcParameter;
	BaseType_t xParameterStringLength, xReturn;
	static UBaseType_t uxParameterNumber = 0;
	uint32_t value = 0;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes
	the write buffer length is adequate, so does not check for buffer
	overflows. */
	(void)pcCommandString;
	(void)xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	if (uxParameterNumber == 0) {
		/* The first time the function is called after the command has
		been entered just a header string is returned. */
		sprintf(pcWriteBuffer, "\r");

		/* Next time the function is called the first parameter will be
		echoed back. */
		uxParameterNumber = 1U;

		/* There is more data to be returned as no parameters have been
		echoed back yet. */
		xReturn = pdPASS;
	} else {
		/* Obtain the parameter string. */
		pcParameter = FreeRTOS_CLIGetParameter(
		    pcCommandString,	    /* The command string itself. */
		    uxParameterNumber,	    /* Return the next parameter. */
		    &xParameterStringLength /* Store the parameter string
						   length. */
		);

		if (pcParameter != NULL) {
			/* Return the parameter string. */
			memset(pcWriteBuffer, 0x00, xWriteBufferLen);
			switch (uxParameterNumber) {
			case 1:
				value = strtoul(pcParameter, NULL, 16);
				wdt_set_timeout(value);
				wdt_reload();
				wdt_enable();
			default:
				xReturn = pdFALSE;
				uxParameterNumber = 0;
				break;

			}
		} else {
			/* No more parameters were found.  Make sure the write
			buffer does not contain a valid string. */
			pcWriteBuffer[0] = 0x00;

			/* No more data to return. */
			xReturn = pdFALSE;

			/* Start over the next time this command is executed. */
			uxParameterNumber = 0;
		}
	}

	return xReturn;
}

static const CLI_Command_Definition_t wdt_reset = 
{
	"reset",
	"\r\nreset:\r\n system reset. usage: reset <mask>.  ex: reset 1 -> reset CPU\r\n",
	do_wdt_reset,
	1
};

void register_mem_commands(void)
{
	FreeRTOS_CLIRegisterCommand(&mem_md);
	FreeRTOS_CLIRegisterCommand(&mem_mw);
	FreeRTOS_CLIRegisterCommand(&wdt_reset);
}