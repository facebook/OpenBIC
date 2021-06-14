/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <errno.h>
#include "cmsis_os.h"
#include "board_device.h"
#include "objects.h"
#include "espi_aspeed.h"

extern aspeed_device_t espi_dev;

osThreadId_t espi_tid;
osThreadAttr_t espi_tattr;

static void espi_task(void *arg)
{
	int i, j, rc;
	uint8_t buf[ESPI_PKT_LEN_MAX];
	struct aspeed_espi_xfer xfer;
	struct espi_comm_hdr *hdr;

	espi_t espi = { .device = &espi_dev };

	for (i = 0; i < ESPI_CH_NUM; ++i) {
		espi.ch_isr[i].handler = NULL;
		espi.ch_isr[i].arg = NULL;
		espi.ch_reset_isr[i].handler = NULL;
		espi.ch_reset_isr[i].arg = NULL;
	}

	aspeed_espi_init(&espi);

	xfer.pkt = buf;
	hdr = (struct espi_comm_hdr *)xfer.pkt;

	while (1) {
		rc = aspeed_espi_oob_get_rx(&xfer);
		if (rc) {
			if (rc != -ENODATA)
				printf("failed to get OOB packet, rc=%d\n", rc);
		}
		else {
			printf("cyc=0x%02x, tag=0x%02x, len=0x%04x\n",
					hdr->cyc,
					hdr->tag,
					(hdr->len_h << 8) | hdr->len_l);
			for (i = sizeof(*hdr), j = 0; i < xfer.pkt_len; ++i, ++j) {
				if (j && (j % 16) == 0)
					printf("\n");
				printf("%02x ", xfer.pkt[i]);
			}
			printf("\n");
		}

		rc = aspeed_espi_flash_get_rx(&xfer);
		if (rc) {
			if (rc != -ENODATA)
				printf("failed to get flash RX packet, rc=%d\n", rc);
		}
		else {
			printf("cyc=0x%02x, tag=0x%02x, len=0x%04x\n",
					hdr->cyc,
					hdr->tag,
					(hdr->len_h << 8) | hdr->len_l);

			hdr->cyc = 0xf;
			hdr->tag = 0x5;
			hdr->len_h = 0x0;
			hdr->len_l = 0x4;

			i = sizeof(*hdr);

			xfer.pkt[i++] = 0x41;
			xfer.pkt[i++] = 0x42;
			xfer.pkt[i++] = 0x43;
			xfer.pkt[i++] = 0x44;

			rc = aspeed_espi_flash_put_tx(&xfer);
			if (rc)
				printf("failed to put flash TX packet, rc=%d\n", rc);
		}

		osDelay(1000);
	}
}

void demo_espi_init(void)
{
	espi_tattr.name = "demo_espi";
	espi_tattr.priority = osPriorityBelowNormal;
	espi_tattr.stack_size = 8192;

	espi_tid = osThreadNew(espi_task, NULL, &espi_tattr);
}
