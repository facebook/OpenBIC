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
#include "bt_aspeed.h"

extern aspeed_device_t bt_dev;

struct bt_request {
	uint8_t len;
	uint8_t netfn;
	uint8_t seq;
	uint8_t cmd;
	uint8_t data[0];
};

struct bt_response {
	uint8_t len;
	uint8_t netfn;
	uint8_t seq;
	uint8_t cmd;
	uint8_t cmplt_code;
	uint8_t data[0];
};

osThreadId_t bt_tid;
osThreadAttr_t bt_tattr;

static void bt_task(void *arg)
{
	int i, rc;

	uint8_t ibuf[ASPEED_BT_BUF_SIZE];
	uint8_t obuf[ASPEED_BT_BUF_SIZE];

	struct bt_request *req;
	struct bt_response *res;

	bt_t bt = { .device = &bt_dev };

	aspeed_bt_init(&bt);

	while (1) {

		while (1) {
			rc = aspeed_bt_read(&bt, ibuf, sizeof(ibuf));
			if (rc >= 0)
				break;
			if (rc != -EAGAIN)
				printf("failed to read BT request, rc=%d\n", rc);
			osDelay(1000);
		}

		req = (struct bt_request *)ibuf;
		printf("BT read: length=0x%02x, netfn=0x%02x, seq=0x%02x, cmd=0x%02x, data:\n",
				req->len, req->netfn, req->seq, req->cmd);
		for (i = 4; i < rc; ++i) {
			if (i && (i % 16 ==0))
				printf("\n");
			printf("%02x ", ibuf[i]);
		}
		printf("\n");

		res = (struct bt_response *)obuf;
		res->len = req->len + 1;
		res->netfn = req->netfn;
		res->seq = req->seq;
		res->cmd = req->cmd;
		res->cmplt_code = 0x0;
		memcpy(res->data, req->data, rc - 4);

		while (1) {
			rc = aspeed_bt_write(&bt, obuf, rc + 1);
			if (rc >= 0)
				break;
			printf("failed to write BT response, rc=%d\n", rc);
			osDelay(1000);
		}
	}
}

void demo_bt_init(void)
{
	bt_tattr.name = "demo_bt";
	bt_tattr.priority = osPriorityBelowNormal;
	bt_tattr.stack_size = 4096;

	bt_tid = osThreadNew(bt_task, NULL, &bt_tattr);
}
