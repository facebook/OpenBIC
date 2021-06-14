/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include "cmsis_os.h"
#include "FreeRTOS_CLI.h"
#include "cm_backtrace.h"
#include "log.h"

void fault_test_by_unalign(void) {
    volatile int * p;
    volatile int value;

    p = (int *) 0x00;
    value = *p;
    printf("addr:0x%02X value:0x%08X\r\n", (int) p, value);

    p = (int *) 0x04;
    value = *p;
    printf("addr:0x%02X value:0x%08X\r\n", (int) p, value);

    p = (int *) 0x03;
    value = *p;
    printf("addr:0x%02X value:0x%08X\r\n", (int) p, value);
}

void fault_test_by_div0(void) {
    int x, y, z;

    x = 10;
    y = 0;
    z = x / y;
    printf("z:%d\n", z);
}

void call_back_trace(void){
    uint32_t call_stack[16] = {0};
    size_t i, depth = 0;
    depth = cm_backtrace_call_stack(call_stack, sizeof(call_stack), cmb_get_sp());
	printf("Show more call stack info by run: addr2line -e XXX.elf -a -f ");
    for (i = 0; i < depth; i++) {
        printf("%08x ", call_stack[i]);
    }
    printf("\n");
}

static BaseType_t do_cmb(char *pcWriteBuffer, size_t xWriteBufferLen,
			    const char *pcCommandString)
{
	const char *pcParameter;
	BaseType_t xParameterStringLength, xReturn;
	static UBaseType_t uxParameterNumber = 0;
	static uint8_t cmd, ExpectedNumberOfParameters;

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
					if( strncmp( pcParameter, "div0", strlen( "div0" ) ) == 0 ){
						cmd = 0;
						ExpectedNumberOfParameters = 1;
					}
					else if( strncmp( pcParameter, "unalign", strlen( "unalign" ) ) == 0 ){
						cmd = 1;
						ExpectedNumberOfParameters = 1;
					}
					else if( strncmp( pcParameter, "bt", strlen( "bt" ) ) == 0 ){
						cmd = 2;
						ExpectedNumberOfParameters = 1;
					}
					else{
						cmd = 0xff;
						ExpectedNumberOfParameters = 1;
					}
					break;
			}

			if (uxParameterNumber == ExpectedNumberOfParameters) {
				if (cmd == 0)
			    	fault_test_by_div0();
				else if (cmd == 1)
					fault_test_by_unalign();
				else if (cmd == 2)
				    call_back_trace();
				else log_warn("Command not found\n");
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

static const CLI_Command_Definition_t cmb_cmd = 
{
	"cmb",
	"\r\ncmb:\n \
	usage: \r\n \
	cmb command: \n \
		cmb div0\r\n \
		cmb unalign\r\n \
		cmb bt\r\n \
        ",
	do_cmb,
	-1
};


void demo_cmb_init(void)
{
    FreeRTOS_CLIRegisterCommand(&cmb_cmd);
}