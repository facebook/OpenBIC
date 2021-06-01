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
#include "snoop_aspeed.h"

extern aspeed_device_t snoop_dev;

osThreadId_t snoop_tid;
osThreadAttr_t snoop_tattr;

static void snoop_task(void *arg)
{
	int i, rc;
	uint8_t buf[256];
	snoop_t snoop = { .device = &snoop_dev };

	aspeed_snoop_init(&snoop);
	while (1) {
		rc = aspeed_snoop_read(&snoop, 0, buf, sizeof(buf));
		if (rc < 0) {
			if (rc != -ENODATA)
				printf("failed to read snoop0 data, rc=%d\n", rc);
		}
		else {
			printf("Snoop0: \n");
			for (i = 0; i < rc; ++i) {
				if (i && ((i % 16) == 0))
					printf("\n");
				printf("%02x ", buf[i]);
			}
			printf("\n");
		}

		rc = aspeed_snoop_read(&snoop, 1, buf, sizeof(buf));
		if (rc < 0) {
			if (rc != -ENODATA)
				printf("failed to read snoop1 data, rc=%d\n", rc);
		}
		else {
			printf("Snoop1: \n");
			for (i = 0; i < rc; ++i) {
				if (i && ((i % 16) == 0))
					printf("\n");
				printf("%02x ", buf[i]);
			}
			printf("\n");
		}

		osDelay(1000);
	}
}

void demo_snoop_init(void)
{
	snoop_tattr.name = "demo_snoop";
	snoop_tattr.priority = osPriorityBelowNormal;
	snoop_tattr.stack_size = 1024;

	snoop_tid = osThreadNew(snoop_task, NULL, &snoop_tattr);
}
