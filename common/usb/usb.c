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
#include "cache_aspeed.h"
#include "getopt.h"
#include "usb_api.h"
#include "log.h"
#include "FreeRTOS_CLI.h"

#define MAX_INPUT_LENGTH	50
#define MAX_OUTPUT_LENGTH	100

extern usb_t usb[];

static osEventFlagsId_t usb_task_event;
static osThreadId_t tid_taskUSB, tid_tmp;
static osThreadAttr_t tattr_taskUSB;
uint8_t *rx_buff;

static void usb_slave_task(void *argv) {
	int rx_len, wait_ret;

	rx_buff = pvPortMallocNc(1024);
	memset(rx_buff, 0, 1024);
	usb_acquire(&usb[0]);
	while (1) {
		rx_len = usb_read(&usb[0], 2, rx_buff, 1024);
    if (rx_len <= 0) {
      continue;
    }

    pal_usb_handler(rx_buff, rx_len);

		usb_write(&usb[0], 1, (uint8_t *)rx_buff, rx_len);
		wait_ret = osEventFlagsWait(usb_task_event, 0x00000001U, osFlagsWaitAll, 1);
		if (wait_ret == 1) {
      printf("*****USB TIMEOUT*****\n");
			usb_release(&usb[0]);
			vPortFreeNc(rx_buff);
			tid_tmp = tid_taskUSB;
			tid_taskUSB = NULL;
			osThreadTerminate(tid_tmp);
		}
	}
}

void usb_slavedev_init(void) {
  usb_task_event = osEventFlagsNew(NULL);
  tattr_taskUSB.name = "usb_slave_thread";
  tattr_taskUSB.priority = osPriorityBelowNormal;
  tattr_taskUSB.stack_size = 0x1000;

  if (tid_taskUSB) {
    log_warn("USB slave device is already running\n");
    return;
  }

  tid_taskUSB = osThreadNew(usb_slave_task, NULL, &tattr_taskUSB);
}
