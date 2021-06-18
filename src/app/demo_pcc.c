/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "cmsis_os.h"
#include "board_device.h"
#include "objects.h"
#include "pcc_aspeed.h"

extern aspeed_device_t pcc_dev;

osThreadId_t pcc_tid;
osThreadAttr_t pcc_tattr;

static void pcc_task(void *arg)
{
	int i, rc;
	uint8_t buf[256];
	pcc_t pcc = { .device = &pcc_dev };

	aspeed_pcc_init(&pcc);
	while (1) {
		rc = aspeed_pcc_read(&pcc, buf, sizeof(buf));
		if (rc < 0) {
			if (rc != -ENODATA)
				printf("failed to read PCC data, rc=%d\n", rc);
			osDelay(1000);
			continue;
		}

		printf("PCC:\n");
		for (i = 0; i < rc; ++i) {
			if (i && ((i % 16) == 0))
				printf("\n");
			printf("%02x ", buf[i]);
		}
		printf("\n");
	}
}

void demo_pcc_init(void)
{
	pcc_tattr.name = "demo_pcc";
	pcc_tattr.priority = osPriorityBelowNormal;
	pcc_tattr.stack_size = 1024;

	pcc_tid = osThreadNew(pcc_task, NULL, &pcc_tattr);
}
