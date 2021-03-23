/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include "cmsis_os.h"
#include "board_device.h"
#include "objects.h"
#include "peci_aspeed.h"
#include "FreeRTOS_CLI.h"
#include "log.h"

peci_t o_peci;
osThreadId_t tid_task;
osThreadAttr_t tattr_task;

static hal_status_t peci_ping(uint32_t address)
{
    peci_xfer_msg_t ping;
    uint8_t exp_wfcs, cap_wfcs;
    hal_status_t status;
    ping.addr = address;
    ping.tx_len    = 0x0;
    ping.rx_len    = 0x0;
    status = aspeed_peci_xfer(&o_peci, &ping);
    if(status != HAL_OK)
    {
        log_error("peci transfer error: %d\n", status);
        return status;
    }
    exp_wfcs = aspeed_get_expected_fcs(&o_peci, PECI_WFCS);
    cap_wfcs = aspeed_get_captured_fcs(&o_peci, PECI_WFCS);
    if (exp_wfcs != cap_wfcs)
    {
        log_error("wfcs check error: expected = 0x%08x captured = 0x%08x\n", exp_wfcs, cap_wfcs);
        return HAL_ERROR;
    }
    printf("ok\n");
    return status;
}

static hal_status_t peci_common_xfer(peci_xfer_msg_t *msg)
{
    uint8_t i;
    uint8_t exp_wfcs, cap_wfcs;
    uint8_t exp_rfcs, cap_rfcs;
    hal_status_t status;
    status = aspeed_peci_xfer(&o_peci, msg);
    if(status != HAL_OK)
    {
        log_error("peci transfer error: %d\n", status);
        return status;
    }
    exp_wfcs = aspeed_get_expected_fcs(&o_peci, PECI_WFCS);
    cap_wfcs = aspeed_get_captured_fcs(&o_peci, PECI_WFCS);
    if (exp_wfcs != cap_wfcs)
    {
        log_error("wfcs check error: expected = 0x%08x captured = 0x%08x\n", exp_wfcs, cap_wfcs);
        return HAL_ERROR;
    }
    exp_rfcs = aspeed_get_expected_fcs(&o_peci, PECI_RFCS);
    cap_rfcs = aspeed_get_captured_fcs(&o_peci, PECI_RFCS);
    if (exp_rfcs != cap_rfcs)
    {
        log_error("rfcs check error: expected = 0x%08x captured = 0x%08x\n", exp_rfcs, cap_rfcs);
        return HAL_ERROR;
    }
    for(i = 0; i < msg->rx_len; i++) 
        printf("%x ", msg->rx_buf[i]);
    printf("\n");
    return status;
}

static hal_status_t peci_getdib(uint32_t address)
{
    peci_xfer_msg_t getdib;
    getdib.addr = address;
    getdib.tx_len    = 0x1;
    getdib.rx_len    = 0x8;
    getdib.tx_buf[0] = 0xf7;
    return peci_common_xfer(&getdib);
}

static hal_status_t peci_gettmp(uint32_t address)
{
    peci_xfer_msg_t gettmp;
    gettmp.addr = address;
    gettmp.tx_len    = 0x1;
    gettmp.rx_len    = 0x2;
    gettmp.tx_buf[0] = 0x1;
    return peci_common_xfer(&gettmp);
}

static hal_status_t peci_rdpkgcfg(uint32_t address)
{
    peci_xfer_msg_t rdpkgcfg;
    rdpkgcfg.addr = address;
    rdpkgcfg.tx_len    = 0x5;
    rdpkgcfg.rx_len    = 0x5;
    rdpkgcfg.tx_buf[0] = 0xa1;
    rdpkgcfg.tx_buf[1] = 0x0;
    rdpkgcfg.tx_buf[2] = 0x0;
    rdpkgcfg.tx_buf[3] = 0x0;
    rdpkgcfg.tx_buf[4] = 0x0;
    return peci_common_xfer(&rdpkgcfg);
}

static BaseType_t do_peci(char *pcWriteBuffer, size_t xWriteBufferLen,
			    const char *pcCommandString)
{
	const char *pcParameter;
	BaseType_t xParameterStringLength, xReturn;
	static UBaseType_t uxParameterNumber = 0;
	static uint8_t cmd, ExpectedNumberOfParameters;
	static uint32_t address = 0;

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
					if( strncmp( pcParameter, "ping", strlen( "ping" ) ) == 0 ){
						cmd = 0;
						ExpectedNumberOfParameters = 2;
					}
					else if( strncmp( pcParameter, "getdib", strlen( "getdib" ) ) == 0 ){
						cmd = 1;
						ExpectedNumberOfParameters = 2;
					}
					else if( strncmp( pcParameter, "gettmp", strlen( "gettmp" ) ) == 0 ){
						cmd = 2;
						ExpectedNumberOfParameters = 2;
					}
                    else if( strncmp( pcParameter, "rdpkgcfg", strlen( "rdpkgcfg" ) ) == 0 ){
						cmd = 3;
						ExpectedNumberOfParameters = 2;
					}
					else{
						cmd = 0xff;
						ExpectedNumberOfParameters = 1;
					}
					break;
				case 2:
					address = strtoul(pcParameter, NULL, 16);
					break;
			}

			if (uxParameterNumber == ExpectedNumberOfParameters) {
				if(cmd == 0)
			    	peci_ping(address);
				else if(cmd == 1)
					peci_getdib(address);
				else if(cmd == 2)
				    peci_gettmp(address);
                else if(cmd == 3)
				    peci_rdpkgcfg(address);
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

// static void peci_task(void *argv)
// {
//     o_peci.device = &peci;
//     aspeed_peci_init(&o_peci);
    
//     while (1) {
//         peci_ping();
//         osDelay(1000);
//         peci_getdib();
//         osDelay(1000);
//         peci_gettmp();
//         osDelay(1000);
//     }
// }

static const CLI_Command_Definition_t peci_cmd = 
{
	"peci",
	"\r\npeci:\n \
	usage: \r\n \
	peci command: \n \
		peci ping [addr]\r\n \
		peci getdib [addr]\r\n \
		peci gettmp [addr]\r\n \
		peci rdpkgcfg [addr] \r\n \
        ",
	do_peci,
	-1
};

void demo_peci_init(void)
{
    // tattr_task.name = "demo_peci";
    // tattr_task.priority = osPriorityBelowNormal;
    // tattr_task.stack_size = 1024;
    
    // tid_task = osThreadNew(peci_task, NULL, &tattr_task);
    o_peci.device = &peci;
    aspeed_peci_init(&o_peci);

    FreeRTOS_CLIRegisterCommand(&peci_cmd);
}