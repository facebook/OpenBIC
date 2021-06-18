/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include "board_device.h"
#include "objects.h"
#include "i3c_aspeed.h"
#include "i3c_api.h"
#include "internal.h"
#include "FreeRTOS_CLI.h"
#include "log.h"
#include "cmsis_os.h"

#define I3C1_SELF_ADDR	0x52

osThreadId_t i3c_task_id;
osThreadAttr_t i3c_task_attr;

i3c_global_t i3c_g = {.device = &i3c_global};
i3c_t o_i3c_s = {	.device = &i3c1,
					.global = &i3c_g,
					.role = 1,
					.self_addr = I3C1_SELF_ADDR,
					.n_slaves = 0,
					.i2c_mode = 0,
					.bus_context = 128
				};


/**
 * case 2: stand alone aspeed slave
*/
#define DUMMY_DATA_SZ		ASPEED_I3C_TX_FIFO_SIZE
uint8_t dummy_data[DUMMY_DATA_SZ];

static void i3c_task(void *argv)
{
	uint32_t i;
	/* prepare i3c dummy data */
	for (i = 0; i < DUMMY_DATA_SZ; i++)
		dummy_data[i] = i;
	
	i3c_init(&o_i3c_s);

	while (1) {
		i3c_msg_t msg;
		uint32_t length;
		uint8_t *tmp;
		uint8_t addr;

		i3c_slave_receive(&o_i3c_s, &msg);
		print_i3c_msg(&msg);

		/* handle the write command:
		 *
		 * msg.buf[0]:  r/w address
		 * msg.buf[1]:  reserved
		 * msg.buf[2~]: wr data
		 */
		addr = msg.buf[0];
		if (msg.len > 2) {
			for (i = 0; i < msg.len - 2; i++)
				dummy_data[addr + i] = msg.buf[i + 2];
		}

		/* handle the read command */
		length = DUMMY_DATA_SZ - addr;
		if ((uint32_t)&dummy_data[addr] & 0x3) {
			/* copy if source address is not aligned */
			tmp = pvPortMallocNc(length);
			memcpy((void *)tmp, (void *)&dummy_data[addr], length);
			aspeed_i3c_slave_wr_resp(&o_i3c_s, tmp, length);
			vPortFreeNc(tmp);
		} else {
			aspeed_i3c_slave_wr_resp(&o_i3c_s, &dummy_data[addr], length);
		}
	}
	i3c_free(&o_i3c_s);
}

void demo_i3c_init(void)
{
	/* create a task for stress testing */
	i3c_task_attr.name = "demo_i3c";
	i3c_task_attr.priority = osPriorityBelowNormal;
	i3c_task_attr.stack_size = 1024;

	i3c_task_id = osThreadNew(i3c_task, NULL, &i3c_task_attr);

	i3c_cmd_register_obj(&o_i3c_s);
	/* register commands */
	FreeRTOS_CLIRegisterCommand(&i3c_cmd);
}