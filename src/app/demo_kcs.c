/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"
#include "board_device.h"
#include "objects.h"
#include "kcs_aspeed.h"

extern aspeed_device_t kcs3_dev;

struct kcs_request {
	uint8_t netfn;
	uint8_t cmd;
	uint8_t data[0];
};

struct kcs_response {
	uint8_t netfn;
	uint8_t cmd;
	uint8_t cmplt_code;
	uint8_t data[0];
};

osThreadId_t kcs_tid;
osThreadAttr_t kcs_tattr;

static void kcs_task(void *arg)
{
	int i, rc;
	uint8_t ibuf[256];
	uint8_t obuf[256];

	struct kcs_request *req;
	struct kcs_response *res;

	kcs_t kcs3 = { .device = &kcs3_dev };
	
	aspeed_kcs_init(&kcs3);

	while (1) {
		rc = aspeed_kcs_read(&kcs3, ibuf, sizeof(ibuf));
		if (rc < 0) {
			printf("failed to read KCS data, rc=%d\n", rc);
			continue;
		}

		req = (struct kcs_request *)ibuf;
		printf("KCS read: netfn=0x%02x, cmd=0x%02x, data:\n",
				req->netfn, req->cmd);
		for (i = 2; i < rc; ++i) {
			if (i && (i % 16 == 0))
				printf("\n");
			printf("%02x ", ibuf[i]);
		}
		printf("\n");

		res = (struct kcs_response *)obuf;
		res->netfn = req->netfn;
		res->cmd = req->cmd;
		res->cmplt_code = 0x0;
		memcpy(res->data, req->data, rc - 2);

		rc = aspeed_kcs_write(&kcs3, obuf, rc + 1);
		if (rc < 0) {
			printf("failed to write KCS data, rc=%d\n", rc);
			continue;
		}

		osDelay(1000);
	}
}

void demo_kcs_init(void)
{
	kcs_tattr.name = "demo_kcs";
	kcs_tattr.priority = osPriorityBelowNormal;
	kcs_tattr.stack_size = 4096;

	kcs_tid = osThreadNew(kcs_task, NULL, &kcs_tattr);
}
