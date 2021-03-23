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
static osThreadId_t tid_taskA, tid_tmp;
static osThreadAttr_t tattr_taskA;
static const CLI_Command_Definition_t usb_cmd;
static uint8_t *rx_buff;

static void usb_exec_task(void *argv)
{
	uint8_t *input_buff, *output_buff, *rx_buff;
	int rx_len, wait_ret;
	int inputIdx = 0;

	rx_buff = pvPortMallocNc(MAX_INPUT_LENGTH);
	input_buff = pvPortMallocNc(MAX_INPUT_LENGTH);
	output_buff = pvPortMallocNc(MAX_OUTPUT_LENGTH);
	memset(rx_buff, 0, MAX_INPUT_LENGTH);
	memset(input_buff, 0, MAX_INPUT_LENGTH);
	memset(output_buff, 0, MAX_OUTPUT_LENGTH);
	usb_acquire(&usb[0]);

	while (1) {
		rx_len = usb_read(&usb[0], 2, rx_buff, MAX_INPUT_LENGTH);
		for(int i = 0; i < rx_len; i++) {
			if (rx_buff[i] == '\r') {
				printf("\n");

				/* process command */
				FreeRTOS_CLIProcessCommand((const char *)input_buff, (char *)output_buff, MAX_OUTPUT_LENGTH);
				printf("%s", output_buff);

				inputIdx = 0;
				memset(input_buff, 0, MAX_INPUT_LENGTH);

			} else if (rx_buff[i] == '\b') {
				if (inputIdx > 0) {
					inputIdx--;
					input_buff[inputIdx] = ' ';
				}
			} else {
				if (inputIdx < MAX_INPUT_LENGTH) {
					printf("%c", rx_buff[i]);
					input_buff[inputIdx] = rx_buff[i];
					inputIdx++;
				}
			}
		}
		usb_write(&usb[0], 1, (uint8_t *)rx_buff, rx_len);
		wait_ret = osEventFlagsWait(usb_task_event, 0x00000001U, osFlagsWaitAll, 1);
		if (wait_ret == 1) {
			usb_release(&usb[0]);
			vPortFreeNc(rx_buff);
			vPortFreeNc(input_buff);
			vPortFreeNc(output_buff);
			tid_tmp = tid_taskA;
			tid_taskA = NULL;
			osThreadTerminate(tid_tmp);
		}
	}
}

static void usb_echo_task(void *argv)
{
	int rx_len, wait_ret;
	rx_buff = pvPortMallocNc(1024);
	memset(rx_buff, 0, 1024);
	usb_acquire(&usb[0]);
	while (1) {
		rx_len = usb_read(&usb[0], 2, rx_buff, 1024);
		for(int i = 0; i < rx_len; i++) {
			if(rx_buff[i] == '\0') {
				puts("null");
			} else if (rx_buff[i] == '\r') {
				printf("\n");
			} else {
				printf("%c", rx_buff[i]);
			}
		}
		usb_write(&usb[0], 1, (uint8_t *)rx_buff, rx_len);
		wait_ret = osEventFlagsWait(usb_task_event, 0x00000001U, osFlagsWaitAll, 1);
		if (wait_ret == 1) {
			usb_release(&usb[0]);
			vPortFreeNc(rx_buff);
			tid_tmp = tid_taskA;
			tid_taskA = NULL;
			osThreadTerminate(tid_tmp);
		}
	}
}

static void usb_cmd_handler(int argc, char *argv[])
{
	char option;
	bool run = 0, delete = 0, exec = 0;
	optind = 0;
	while ((option = getopt(argc, argv, "hred")) != (char)-1) {
		switch (option) {
			case 'h':
			case '?':
				printf("%s", usb_cmd.pcHelpString);
				return;
			case 'r':
				run = 1;
				break;
			case 'e':
				exec = 1;
				break;
			case 'd':
				delete = 1;
				break;
			default:
				log_warn("unknown option -%c", option);
				break;
			}
	}
	if (run) {
		usb_task_event = osEventFlagsNew(NULL);
		tattr_taskA.name = "demo_echo_usb";
		tattr_taskA.priority = osPriorityBelowNormal;
		tattr_taskA.stack_size = 0x1000;

		if (tid_taskA) {
			log_warn("USB demo is already running\n");
			return;
		}

		tid_taskA = osThreadNew(usb_echo_task, NULL, &tattr_taskA);

	} else if (exec) {
		usb_task_event = osEventFlagsNew(NULL);
		tattr_taskA.name = "demo_exec_usb";
		tattr_taskA.priority = osPriorityBelowNormal;
		tattr_taskA.stack_size = 0x1000;

		if (tid_taskA) {
			log_warn("USB demo is already running\n");
			return;
		}

		tid_taskA = osThreadNew(usb_exec_task, NULL, &tattr_taskA);

	} else if (delete) {
		if (tid_taskA)
			osEventFlagsSet(usb_task_event, 0x00000001U);
	}
}

CLI_FUNC_DECL(usb, usb_cmd_handler);

static const CLI_Command_Definition_t usb_cmd =
{
	"usb",
	"\r\nusb:\n \
	usage: \r\n \
		[-h]: Show this message\r\n \
		[-r]: Run usb echo daemon\r\n \
		[-e]: Run usb execute daemon\r\n \
		[-d]: Delete usb daemon\r\n",
	CLI_FUNC_SYM(usb),
	-1
};

void demo_usb_init(void)
{
	FreeRTOS_CLIRegisterCommand(&usb_cmd);
}
