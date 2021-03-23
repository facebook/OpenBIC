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
#include "mdio_aspeed.h"
#include "mac_aspeed.h"
#include "cache_aspeed.h"
#include "clk_aspeed.h"
#include "phy.h"

#define NETTEST_CTRL_EXT		0
#define NETTEST_CTRL_PHY		1
#define NETTEST_CTRL_MAC		2
#define NETTEST_CTRL_TX			3

#if CONFIG_AST1030_SERIES
mdio_t mdio_obj = { .device = &mdio0 };
phy_t phy_obj = { .link = 0, .mdio = &mdio_obj, .loopback = 0 };
mac_t mac_obj = { .device = &mac0, .phy = &phy_obj };
#else
mdio_t mdio_obj = { .device = &mdio2, .phy_addr = 0 };
phy_t phy_obj = { .link = 0, .mdio = &mdio_obj, .loopback = 0 };
mac_t mac_obj = { .device = &mac2, .phy = &phy_obj };
#endif

#define PKT_PER_TEST	4
mac_txdes_t txdes[PKT_PER_TEST] NON_CACHED_BSS_ALIGN16;
mac_rxdes_t rxdes[PKT_PER_TEST] NON_CACHED_BSS_ALIGN16;

static int nettest(uint32_t speed, uint32_t control)
{
	int i;
	uint32_t rxlen;
	hal_status_t status;
	hal_status_t ret = HAL_OK;

	uint8_t *rx_pkt_buf[PKT_PER_TEST];
	uint8_t *tx_pkt_buf[PKT_PER_TEST];
	uint8_t *ptr;
	uint8_t mac_addr[6] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05};

#ifdef CONFIG_AST2600_SERIES
	writel(0x10004077, SCU_BASE + 0x240);
	writel(0x8000003b, SCU_BASE + 0x244);
	clrsetbits(SCU_BASE + 0x304, GENMASK(23, 16), 0x74 << 16);
	clrsetbits(SCU_BASE + 0x310, GENMASK(26, 24), 0 << 24);
	clrsetbits(SCU_BASE + 0x340, GENMASK(31, 28), 0x9 << 28);
#else
	clrsetbits(SCU_BASE + 0x310, GENMASK(26, 24), 1 << 24);
	clrsetbits(SCU_BASE + 0x310, GENMASK(19, 16), 9 << 16);
	/* bit[31]: 1=select HPLL as clock source
	 * bit[23:20]: 7=HPLL clock / 8 = 1000/8 = 125M
	 */
	clrsetbits(SCU_BASE + 0x310, BIT(31) | GENMASK(23,20), 7 << 20);
	/* bit[31]: 1=select internal clock source */
	setbits(SCU_BASE + 0x350, BIT(31));
#endif

	for (i = 0; i < PKT_PER_TEST; i++) {
		tx_pkt_buf[i] = pvPortMallocNc(0x600);
		rx_pkt_buf[i] = pvPortMallocNc(0x600);
	}
	
	log_debug("address of txdes: %08x\n", (uint32_t)&txdes[0]);
	log_debug("address of rxdes: %08x\n", (uint32_t)&rxdes[0]);

	mac_obj.txdes = txdes;
	mac_obj.rxdes = rxdes;
	mac_obj.n_txdes = PKT_PER_TEST;
	mac_obj.n_rxdes = PKT_PER_TEST;
	mac_obj.rx_pkt_buf = &rx_pkt_buf[0];
	mac_obj.mac_addr = mac_addr;
	
	aspeed_mac_init(&mac_obj);
	if (control == NETTEST_CTRL_MAC) {
		aspeed_mac_set_loopback(&mac_obj, 1);
	} else {
		aspeed_mac_set_loopback(&mac_obj, 0);
	}

	ptr = &tx_pkt_buf[0][0];
	for (i = 0; i < 6; i++)
		*ptr++ = 0xff;
	for (i = 0; i < 6; i++)
		*ptr++ = mac_addr[i];
	
	*ptr++ = 0x00;
	*ptr++ = 0x60;

	for (i = 0; i < 60-14; i++)
		*ptr++ = i;

	for (i = 0; i < PKT_PER_TEST; i++)
		aspeed_mac_xmit(&mac_obj, tx_pkt_buf[0], 60);

	osDelay(1);

	for (i = 0; i < PKT_PER_TEST; i++) {
		status = aspeed_mac_recv(&mac_obj, (void **)&rx_pkt_buf[i], &rxlen);
		if (status != HAL_OK)
			ret = status;

#if 0
		if (status == HAL_OK) {
			printf("RX packet %d: length=%d addr=%08x\n", i, rxlen,
				   (uint32_t)rx_pkt_buf[i]);
			for (int j = 0; j < rxlen; j++) {
				printf("%02x ", rx_pkt_buf[i][j]);
				if ((j % 8) == 7)
					printf("\n");
			}
		}
#endif
	}

	for (i = 0; i < PKT_PER_TEST; i++) {
		vPortFreeNc(tx_pkt_buf[i]);
		vPortFreeNc(rx_pkt_buf[i]);
	}
	return ret;
}
static BaseType_t do_nettest(char *pcWriteBuffer, size_t xWriteBufferLen,
			    const char *pcCommandString)
{
	const char *pcParameter;
	BaseType_t xParameterStringLength, xReturn;
	static UBaseType_t uxParameterNumber = 0;
	static uint32_t speed = 0;
	static uint32_t control = 0;
	static uint32_t interface = 1;
	int i, j;

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
					speed = strtoul(pcParameter, NULL, 10);
					break;
				case 2:
					control = NETTEST_CTRL_EXT;
					if (strncmp(pcParameter, "phy", strlen("phy")) == 0) {
						control = NETTEST_CTRL_PHY;
					} else if (strncmp(pcParameter, "mac", strlen("mac")) == 0) {
						control = NETTEST_CTRL_MAC;
					}
					break;
				case 3:
					interface = 1;
					if (strncmp(pcParameter, "rmii", strlen("rmii")) == 0) {
						interface = 0;
					}
					break;
			}

			if (uxParameterNumber == 3) {
				printf("nettest %d %d %d\n", speed, control, interface);
				phy_obj.duplex = 1;
				phy_obj.speed = speed;
				if (control == NETTEST_CTRL_PHY)
					phy_obj.loopback = 1;
				else
					phy_obj.loopback = 0;

#ifdef CONFIG_AST1030_SERIES
				if (interface)
					writel(BIT(0), SCU_BASE + 0x510);
				else
					writel(BIT(0), SCU_BASE + 0x514);
#endif
				phy_bcm54616_config(&phy_obj);

				printf("   ");
				for (i = 0; i < 0x40; i++)
					printf("%1x", i & 0xf);
				
				printf("\n");

				for (i = 0; i < 0x40; i++) {
					printf("%02x:", i);
					for (j = 0; j < 0x40; j++) {
						aspeed_clk_set_rgmii_delay(mac_obj.device->dev_id, 0, i, j);
						aspeed_clk_set_rgmii_delay(mac_obj.device->dev_id, 1, i, j);
						aspeed_clk_set_rgmii_delay(mac_obj.device->dev_id, 2, i, j);
						if (nettest(speed, control) == HAL_OK)
							printf("o");
						else
							printf("x");
					}
					printf("\n");
				}

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

static const CLI_Command_Definition_t nettest_cmd = {
    "nettest",
    "\r\nnettest:\r\n Ethernet loopback test.\r\n  nettest [speed] [ctrl]\r\n\
    [speed]: 1000, 100, or 10\r\n\
    [ctrl] : ext = external loopback\r\n\
           : phy = PHY loopback\r\n\
           : mac = MAC loopback\r\n\
           : tx  = TX only\r\n",
    do_nettest,
	3
};

static BaseType_t do_mdio(char *pcWriteBuffer, size_t xWriteBufferLen,
			    const char *pcCommandString)
{
	const char *pcParameter;
	BaseType_t xParameterStringLength, xReturn;
	static UBaseType_t uxParameterNumber = 0;
	static uint8_t cmd, ExpectedNumberOfParameters;
	static uint32_t phy_addr = 0;
	static uint32_t reg_addr = 0;
	static uint32_t data;

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
					if( strncmp( pcParameter, "r", strlen( "r" ) ) == 0 ){
						cmd = 0;
						ExpectedNumberOfParameters = 3;
					} else if( strncmp( pcParameter, "w", strlen( "w" ) ) == 0 ){
						cmd = 1;
						ExpectedNumberOfParameters = 4;
					} else{
						cmd = 0xff;
						ExpectedNumberOfParameters = 1;
					}
					break;
				case 2:
					phy_addr = strtoul(pcParameter, NULL, 16);
					break;
				case 3:
					reg_addr = strtoul(pcParameter, NULL, 16);
					break;
				case 4:
					data = strtoul(pcParameter, NULL, 16);
					break;
			}

			if (uxParameterNumber == ExpectedNumberOfParameters) {
				if(cmd == 0)
					printf("phy %02x reg %04x: %04x\n", phy_addr, reg_addr, aspeed_mdio_read(&mdio_obj, phy_addr, reg_addr));
				else if(cmd == 1)
					aspeed_mdio_write(&mdio_obj, phy_addr, reg_addr, data);
				else
					printf("mdio #%d is under testing\n", mdio_obj.device->dev_id - ASPEED_DEV_MDIO0);

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


static const CLI_Command_Definition_t mdio_cmd = {
    "mdio",
    "\r\nmdio:\r\n mdio read/write command.\r\n  mdio r [phy_addr] [reg_addr]\r\n  mdio w [phy_addr] [reg_addr] [data]\r\n",
    do_mdio, -1
};

void demo_net_init(void)
{
#if CONFIG_DEMO_MDIO	
    aspeed_mdio_init(&mdio_obj);
	FreeRTOS_CLIRegisterCommand(&mdio_cmd);
#endif
    FreeRTOS_CLIRegisterCommand(&nettest_cmd);
}