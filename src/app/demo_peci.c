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
	ping.tx_len = 0x0;
	ping.rx_len = 0x0;
	ping.has_aw_fcs = 0x0;
	status = aspeed_peci_xfer(&o_peci, &ping);
	if (status != HAL_OK) {
		log_error("peci transfer error: %d\n", status);
		return status;
	}
	exp_wfcs = aspeed_get_expected_fcs(&o_peci, PECI_WFCS);
	cap_wfcs = aspeed_get_captured_fcs(&o_peci, PECI_WFCS);
	if (exp_wfcs != cap_wfcs) {
		log_error("wfcs check error: expected = 0x%08x captured = 0x%08x\n",
				  exp_wfcs, cap_wfcs);
		return HAL_ERROR;
	}
	printf("ok\n");
	return status;
}

/* Completion Code mask to check retry needs */
#define PECI_DEV_CC_RETRY_CHECK_MASK 0xf0

/* Device Specific Completion Code (CC) Definition */
#define PECI_DEV_CC_SUCCESS 0x40
#define PECI_DEV_CC_NEED_RETRY 0x80
#define PECI_DEV_CC_OUT_OF_RESOURCE 0x81
#define PECI_DEV_CC_UNAVAIL_RESOURCE 0x82
#define PECI_DEV_CC_INVALID_REQ 0x90
#define PECI_DEV_CC_MCA_ERROR 0x91
#define PECI_DEV_CC_CATASTROPHIC_MCA_ERROR 0x93
#define PECI_DEV_CC_FATAL_MCA_DETECTED 0x94
#define PECI_DEV_CC_PARITY_ERROR_ON_GPSB_OR_PMSB 0x98
#define PECI_DEV_CC_PARITY_ERROR_ON_GPSB_OR_PMSB_IERR 0x9B
#define PECI_DEV_CC_PARITY_ERROR_ON_GPSB_OR_PMSB_MCA 0x9C

static hal_status_t peci_common_xfer(peci_xfer_msg_t *msg, bool do_retry)
{
	uint8_t i;
	uint8_t exp_wfcs, cap_wfcs;
	uint8_t exp_rfcs, cap_rfcs;
	hal_status_t status;
	for (;;) {
		status = aspeed_peci_xfer(&o_peci, msg);
		if (status != HAL_OK) {
			log_error("peci transfer error: %d\n", status);
			return status;
		}
		if (!do_retry)
			break;
		if ((msg->rx_buf[0] & PECI_DEV_CC_RETRY_CHECK_MASK) !=
			PECI_DEV_CC_NEED_RETRY)
			break;
	}
	exp_wfcs = aspeed_get_expected_fcs(&o_peci, PECI_WFCS);
	cap_wfcs = aspeed_get_captured_fcs(&o_peci, PECI_WFCS);
	if (exp_wfcs != cap_wfcs) {
		log_error("wfcs check error: expected = 0x%08x captured = 0x%08x\n",
				  exp_wfcs, cap_wfcs);
		return HAL_ERROR;
	}
	exp_rfcs = aspeed_get_expected_fcs(&o_peci, PECI_RFCS);
	cap_rfcs = aspeed_get_captured_fcs(&o_peci, PECI_RFCS);
	if (exp_rfcs != cap_rfcs) {
		log_error("rfcs check error: expected = 0x%08x captured = 0x%08x\n",
				  exp_rfcs, cap_rfcs);
		return HAL_ERROR;
	}
	for (i = 0; i < msg->rx_len; i++)
		printf("%x ", msg->rx_buf[i]);
	printf("\n");
	return status;
}

static hal_status_t peci_getdib(uint32_t address)
{
	peci_xfer_msg_t getdib;
	hal_status_t ret;
	getdib.addr = address;
	getdib.tx_len = 0x1;
	getdib.rx_len = 0x8;
	getdib.has_aw_fcs = 0x0;
	getdib.tx_buf = pvPortMalloc(getdib.tx_len);
	getdib.rx_buf = pvPortMalloc(getdib.rx_len);
	getdib.tx_buf[0] = 0xf7;
	ret = peci_common_xfer(&getdib, 0);
	free(getdib.tx_buf);
	free(getdib.rx_buf);
	return ret;
}

static hal_status_t peci_gettmp(uint32_t address)
{
	peci_xfer_msg_t gettmp;
	hal_status_t ret;
	gettmp.addr = address;
	gettmp.tx_len = 0x1;
	gettmp.rx_len = 0x2;
	gettmp.has_aw_fcs = 0x0;
	gettmp.tx_buf = pvPortMalloc(gettmp.tx_len);
	gettmp.rx_buf = pvPortMalloc(gettmp.rx_len);
	gettmp.tx_buf[0] = 0x1;
	ret = peci_common_xfer(&gettmp, 0);
	free(gettmp.tx_buf);
	free(gettmp.rx_buf);
	return ret;
}

static hal_status_t peci_rdpkgcfg(uint32_t address, uint8_t u8Index,
								  uint16_t u16Param, uint8_t u8ReadLen)
{
	peci_xfer_msg_t rdpkgcfg;
	hal_status_t ret;
	rdpkgcfg.addr = address;
	rdpkgcfg.tx_len = 0x5;
	rdpkgcfg.rx_len = u8ReadLen;
	rdpkgcfg.has_aw_fcs = 0x0;
	rdpkgcfg.tx_buf = pvPortMalloc(rdpkgcfg.tx_len);
	rdpkgcfg.rx_buf = pvPortMalloc(rdpkgcfg.rx_len);
	rdpkgcfg.tx_buf[0] = 0xa1;
	rdpkgcfg.tx_buf[1] = 0x0;
	rdpkgcfg.tx_buf[2] = u8Index;
	rdpkgcfg.tx_buf[3] = u16Param;
	rdpkgcfg.tx_buf[4] = u16Param >> 8;
	ret = peci_common_xfer(&rdpkgcfg, 1);
	free(rdpkgcfg.tx_buf);
	free(rdpkgcfg.rx_buf);
	return ret;
}

static hal_status_t peci_wrpkgcfg(uint32_t address, uint8_t u8Index,
								  uint16_t u16Param, uint32_t u32Value,
								  uint8_t u8WriteLen)
{
	peci_xfer_msg_t wrpkgcfg;
	hal_status_t ret;
	uint8_t i;
	wrpkgcfg.addr = address;
	wrpkgcfg.tx_len = u8WriteLen;
	wrpkgcfg.rx_len = 0x1;
	wrpkgcfg.has_aw_fcs = 0x1;
	wrpkgcfg.tx_buf = pvPortMalloc(wrpkgcfg.tx_len + 1);
	wrpkgcfg.rx_buf = pvPortMalloc(wrpkgcfg.rx_len);
	wrpkgcfg.tx_buf[0] = 0xa5;
	wrpkgcfg.tx_buf[1] = 0x0;
	wrpkgcfg.tx_buf[2] = u8Index;
	wrpkgcfg.tx_buf[3] = u16Param;
	wrpkgcfg.tx_buf[4] = u16Param >> 8;
	for (i = 0; i < u8WriteLen - 6; i++) {
		wrpkgcfg.tx_buf[i + 5] = u32Value >> (i << 3);
	}
	ret = peci_common_xfer(&wrpkgcfg, 1);
	free(wrpkgcfg.tx_buf);
	free(wrpkgcfg.rx_buf);
	return ret;
}

static BaseType_t do_peci(char *pcWriteBuffer, size_t xWriteBufferLen,
						  const char *pcCommandString)
{
	const char *pcParameter;
	BaseType_t xParameterStringLength, xReturn;
	static UBaseType_t uxParameterNumber = 0;
	static uint8_t cmd, ExpectedNumberOfParameters;
	static uint32_t address = 0;
	static uint8_t u8Index, length;
	static uint16_t u16Param;
	static uint32_t u32Value;

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
				pcCommandString, /* The command string itself. */
				uxParameterNumber, /* Return the next parameter. */
				&xParameterStringLength /* Store the parameter string
						   length. */
		);

		if (pcParameter != NULL) {
			/* Return the parameter string. */
			memset(pcWriteBuffer, 0x00, xWriteBufferLen);
			switch (uxParameterNumber) {
			case 1:
				if (strncmp(pcParameter, "ping", strlen("ping")) == 0) {
					cmd = 0;
					ExpectedNumberOfParameters = 2;
				} else if (strncmp(pcParameter, "getdib", strlen("getdib")) ==
						   0) {
					cmd = 1;
					ExpectedNumberOfParameters = 2;
				} else if (strncmp(pcParameter, "gettmp", strlen("gettmp")) ==
						   0) {
					cmd = 2;
					ExpectedNumberOfParameters = 2;
				} else if (strncmp(pcParameter, "rdpkgcfg",
								   strlen("rdpkgcfg")) == 0) {
					cmd = 3;
					ExpectedNumberOfParameters = 5;

				} else if (strncmp(pcParameter, "wrpkgcfg",
								   strlen("wrpkgcfg")) == 0) {
					cmd = 4;
					ExpectedNumberOfParameters = 6;
				} else {
					cmd = 0xff;
					ExpectedNumberOfParameters = 1;
				}
				break;
			case 2:
				address = strtoul(pcParameter, NULL, 16);
				break;
			case 3:
				length = strtoul(pcParameter, NULL, 16);
				break;
			case 4:
				u8Index = strtoul(pcParameter, NULL, 16);
				break;
			case 5:
				u16Param = strtoul(pcParameter, NULL, 16);
				break;
			case 6:
				u32Value = strtoul(pcParameter, NULL, 16);
				break;
			}

			if (uxParameterNumber == ExpectedNumberOfParameters) {
				if (cmd == 0)
					peci_ping(address);
				else if (cmd == 1)
					peci_getdib(address);
				else if (cmd == 2)
					peci_gettmp(address);
				else if (cmd == 3)
					peci_rdpkgcfg(address, u8Index, u16Param, length);
				else if (cmd == 4)
					peci_wrpkgcfg(address, u8Index, u16Param, u32Value, length);
				else
					log_warn("Command not found\n");
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

static const CLI_Command_Definition_t peci_cmd = { "peci", "\r\npeci:\n \
	usage: \r\n \
	peci command: \n \
		peci ping [addr]\r\n \
		peci getdib [addr]\r\n \
		peci gettmp [addr]\r\n \
		peci rdpkgcfg [addr] [rx_len] [index] [param] \r\n \
		peci wrpkgcfg [addr] [tx_len] [index] [param] [value]\r\n",
												   do_peci, -1 };

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