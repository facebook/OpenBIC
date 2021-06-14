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
#include "log.h"
#include "cmsis_os.h"

/**
 * Topologic: external loopback
 * 
 * I3C0-----------+
 *                |
 * I3C1-----------+
 * 
 */
#define I3C0_SELF_ADDR	0x08
#define I3C1_SELF_ADDR	0x52

i3c_slave_t slaves[1] = {
	{.static_addr = I3C1_SELF_ADDR, .assign_dynamic_addr = I3C1_SELF_ADDR, .i2c_mode = 0},
};

i3c_global_t i3c_g = {.device = &i3c_global};
i3c_t o_i3c_m = {	.device = &i3c0, 
					.global = &i3c_g,
					.role = 0, 
					.self_addr = I3C0_SELF_ADDR, 
					.n_slaves = 1, 
					.slaves = slaves,
					.i2c_mode = 0,
					.bus_context = 128
				};
i3c_t o_i3c_s = {	.device = &i3c1,
					.global = &i3c_g,
					.role = 1,
					.self_addr = I3C1_SELF_ADDR,
					.n_slaves = 0,
					.i2c_mode = 0,
					.bus_context = 128
				};

#ifdef CONFIG_DEVICE_I3CDMA
#include "i3cdma_aspeed.h"
aspeed_device_t i3cdma;
i3cdma_t o_i3c_dma = { .device = &i3cdma, .in_used = 0 };
i3cdma_desc_t i3c_m_txdma = { .dma = &o_i3c_dma };
i3cdma_desc_t i3c_m_rxdma = { .dma = &o_i3c_dma };
//i3c_t o_i3c_m = {.device = &DEMO_I3CMST_DEV, .role = 0, .tx_dma_en = 1, .  rx_dma_en = 0, .tx_dma_desc = &i3c_m_txdma, .rx_dma_desc = &i3c_m_rxdma  };
#endif

osThreadId_t i3c_task_id;
osThreadAttr_t i3c_task_attr;
osThreadId_t i3c_master_task_id;
osThreadAttr_t i3c_master_task_attr;


void i3c_test_task(void *argv)
{
	i3c_usr_xfer_t usr_xfer;
	int i, has_error, loop = 0;
	uint8_t buf[128];
	i3c_msg_t msg;

	usr_xfer.rnw = 0;
	usr_xfer.data.in = buf;
	usr_xfer.len = 124;

	if ((o_i3c_m.device->init == 0) && (o_i3c_s.device->init == 0)) {
		printf("device not init\n");
		return;
	}

	osThreadSuspend(i3c_task_id);
	osThreadSuspend(i3c_master_task_id);

	while (1) {
		for (i = 0; i < 124; i++) {
			buf[i] = i;
		}

		/* master device issues transfers */
		aspeed_i3c_priv_xfer(&o_i3c_m, 0,  &usr_xfer, 1);
		i3c_slave_receive(&o_i3c_s, &msg);

		has_error = 0;
		for (i = 0; i < 124; i++) {
			if (msg.buf[i] != buf[i]) {
				has_error = 1;
				printf("loop#%d: data mismatch @[%d] tx:%02x rx:%02x\n", loop, i, buf[i], msg.buf[i]);
			}
		}

		if ((0 == has_error) && (0 == loop % 10))
			printf("loop#%d: master -> slave  pass\n", loop);

		osDelay(100);

		/* slave device issues IBI with data */
		aspeed_i3c_slave_issue_sir(&o_i3c_s, usr_xfer.data.in, usr_xfer.len);
		i3c_ibi_receive(&o_i3c_m, &msg);

		has_error = 0;
		for (i = 0; i < 123; i++) {
			if (msg.buf[i + 1] != buf[i]) {
				printf("loop#%d: data mismatch @[%d] tx:%02x rx:%02x\n", loop, i, buf[i], msg.buf[i + 1]);
			}
		}

		if ((0 == has_error) && (0 == loop % 10))
			printf("loop#%d: slave  -> master pass\n", loop);

		loop++;
		osDelay(100);
	}

	osThreadResume(i3c_task_id);
	osThreadResume(i3c_master_task_id);

}

osThreadId_t i3c_test_id;
osThreadAttr_t i3c_test_attr;

void i3c_cmd_test_begin_hook(void)
{
	i3c_test_attr.name = "i3c stress";
	i3c_test_attr.priority = osPriorityBelowNormal;
	i3c_test_attr.stack_size = 2048;
	i3c_test_id = osThreadNew(i3c_test_task, NULL, &i3c_test_attr);
}

void i3c_cmd_test_finish_hook(void)
{
	osThreadTerminate(i3c_test_id);
}

static void i3c_master_task(void *argv)
{
	i3c_msg_t msg;

	while (1) {
		i3c_ibi_receive(&o_i3c_m, &msg);
		
		printf("\nmaster receives %d bytes of ibi data\n", msg.len);
		print_i3c_msg(&msg);
	}
}

static void i3c_task(void *argv)
{
	i3c_msg_t msg;
	
	i3c_init(&o_i3c_s);
	i3c_init(&o_i3c_m);

	/* create master task */
	i3c_master_task_attr.name = "i3c master";
	i3c_master_task_attr.priority = osPriorityBelowNormal;
	i3c_master_task_attr.stack_size = 2048;

	i3c_master_task_id = osThreadNew(i3c_master_task, NULL, &i3c_master_task_attr);

	/* slave task */
	while (1) {
		i3c_slave_receive(&o_i3c_s, &msg);

		printf("\nslave receives %d bytes of wr data\n", msg.len);
		print_i3c_msg(&msg);
	}

	osThreadTerminate(i3c_master_task_id);
	i3c_free(&o_i3c_m);
	i3c_free(&o_i3c_s);
}

void demo_i3c_init(void)
{
	/* create a task for stress testing */
	i3c_task_attr.name = "i3c slave";
	i3c_task_attr.priority = osPriorityBelowNormal;
	i3c_task_attr.stack_size = 2048;

	i3c_task_id = osThreadNew(i3c_task, NULL, &i3c_task_attr);

	/* register commands */
	i3c_cmd_register_obj(&o_i3c_m);
	i3c_cmd_register_obj(&o_i3c_s);
	FreeRTOS_CLIRegisterCommand(&i3c_cmd);
}