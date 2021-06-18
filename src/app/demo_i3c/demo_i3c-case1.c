/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include "board_device.h"
#include "objects.h"
#include "i3c_api.h"
#include "internal.h"
#include "i3c_aspeed.h"
#include "FreeRTOS_CLI.h"
#include "log.h"
#include "cmsis_os.h"

/**
 * IMX3102 MR64
 * MR64[7]: 1=enable master port 0 & port 1
 * MR64[6]: 1=enable slave port
*/
#define IMX3102_MR64						64
#define IMX3102_MR64_MASTER_PORT_1_EN		BIT(7)
#define IMX3102_MR64_SLAVE_PORT_EN			BIT(6)
#define IMX3102_MR64_DEF					(IMX3102_MR64_MASTER_PORT_1_EN | IMX3102_MR64_SLAVE_PORT_EN)
/**
 * IMX3102 MR65
 * MR65[7]: 0=master port0, 1=master port1
 * MR65[6]: 1=enable slave port
*/
#define IMX3102_MR65						65
#define IMX3102_MR65_INPUT_MASTER_SELECT	BIT(7)
#define IMX3102_MR65_SLAVE_PORT_EN_0		BIT(6)

/**
 *                             ASPEED I3C daughter card
 *          +-------------------------------------------------------------
 *          |
 * I3C0-----+-master port0---\                           /----SPD5118 @50
 *          |                 IMX3102 @0f --slave port--+
 * I3C1-----+-master port1---/                           \----SPD5118 @51
 *          |
 * I3C2-----+-master port0---\                           /----SPD5118 @52
 *          |                 IMX3102 @0f --slave port--+
 * I3C3-----+-master port1---/                           \----SPD5118 @53
 *          |
 *          +-------------------------------------------------------------
 */
#define DEVICE_IMX3102						0
#define DEVICE_SPD5118_0					1
#define DEVICE_SPD5118_1					2
#define N_DEVICES							3

i3c_slave_t slaves_on_bus_0_1[N_DEVICES] = {
	/* IMX3102 */
	{.static_addr = 0x0f, .assign_dynamic_addr = 0x0f, .i2c_mode = 0},
	/* SPD5118 */
	{.static_addr = 0x50, .assign_dynamic_addr = 0x50, .i2c_mode = 0},
	{.static_addr = 0x51, .assign_dynamic_addr = 0x51, .i2c_mode = 0},
};

i3c_slave_t slaves_on_bus_2_3[N_DEVICES] = {
	/* IMX3102 */
	{.static_addr = 0x0f, .assign_dynamic_addr = 0x0f, .i2c_mode = 0},
	/* SPD5118 */
	{.static_addr = 0x52, .assign_dynamic_addr = 0x52, .i2c_mode = 0},
	{.static_addr = 0x53, .assign_dynamic_addr = 0x53, .i2c_mode = 0},
};

i3c_global_t i3c_g = {.device = &i3c_global};
i3c_t i3c_adapter[4] = {
	{ .device = &i3c0, .global = &i3c_g, .role = 0, .self_addr = 0x8, .i2c_mode = 0, .bus_context = 128, .n_slaves = N_DEVICES, .slaves = &slaves_on_bus_0_1[0] },
	{ .device = &i3c1, .global = &i3c_g, .role = 0, .self_addr = 0x8, .i2c_mode = 0, .bus_context = 128, .n_slaves = N_DEVICES, .slaves = &slaves_on_bus_0_1[0] },
	{ .device = &i3c2, .global = &i3c_g, .role = 0, .self_addr = 0x8, .i2c_mode = 0, .bus_context = 128, .n_slaves = N_DEVICES, .slaves = &slaves_on_bus_2_3[0] },
	{ .device = &i3c3, .global = &i3c_g, .role = 0, .self_addr = 0x8, .i2c_mode = 0, .bus_context = 128, .n_slaves = N_DEVICES, .slaves = &slaves_on_bus_2_3[0] },
};

osThreadId_t i3c_task_id;
osThreadAttr_t i3c_task_attr;

static void i3c_task(void *argv)
{
	uint8_t data[2] = { IMX3102_MR64_DEF, IMX3102_MR65_SLAVE_PORT_EN_0 };

	i3c_init(&i3c_adapter[0]);
	i3c_init(&i3c_adapter[2]);

	/* enable slave ports */
	i3c_spd_write(&i3c_adapter[0], DEVICE_IMX3102, IMX3102_MR64, 0x2, data);
	i3c_spd_write(&i3c_adapter[2], DEVICE_IMX3102, IMX3102_MR64, 0x2, data);

	/* handover to master #1 and master #3, then init the masters */
	i3c_spd_byte_write(&i3c_adapter[0], DEVICE_IMX3102, IMX3102_MR65, IMX3102_MR65_SLAVE_PORT_EN_0 | IMX3102_MR65_INPUT_MASTER_SELECT);
	i3c_spd_byte_write(&i3c_adapter[2], DEVICE_IMX3102, IMX3102_MR65, IMX3102_MR65_SLAVE_PORT_EN_0 | IMX3102_MR65_INPUT_MASTER_SELECT);
	i3c_init(&i3c_adapter[1]);
	i3c_init(&i3c_adapter[3]);

	/* switch back to master #0 and master #2 */
	i3c_spd_byte_write(&i3c_adapter[1], DEVICE_IMX3102, IMX3102_MR65, IMX3102_MR65_SLAVE_PORT_EN_0);
	i3c_spd_byte_write(&i3c_adapter[3], DEVICE_IMX3102, IMX3102_MR65, IMX3102_MR65_SLAVE_PORT_EN_0);

	osThreadFlagsWait(0x1, osFlagsWaitAny | osFlagsNoClear, osWaitForever);

	while (1) {
		printf("i3c0 read imx3102    mr0:%02x", i3c_spd_byte_read(&i3c_adapter[0], DEVICE_IMX3102, 0));
		printf("i3c0 read spd5118[0] mr0:%02x", i3c_spd_byte_read(&i3c_adapter[0], DEVICE_SPD5118_0, 0));
		printf("i3c0 read spd5118[1] mr0:%02x", i3c_spd_byte_read(&i3c_adapter[0], DEVICE_SPD5118_1, 0));

		printf("i3c2 read imx3102    mr0:%02x", i3c_spd_byte_read(&i3c_adapter[2], DEVICE_IMX3102, 0));
		printf("i3c2 read spd5118[0] mr0:%02x", i3c_spd_byte_read(&i3c_adapter[2], DEVICE_SPD5118_0, 0));
		printf("i3c2 read spd5118[1] mr0:%02x", i3c_spd_byte_read(&i3c_adapter[2], DEVICE_SPD5118_1, 0));

		i3c_spd_byte_write(&i3c_adapter[0], DEVICE_IMX3102, IMX3102_MR65, IMX3102_MR65_SLAVE_PORT_EN_0 | IMX3102_MR65_INPUT_MASTER_SELECT);
		i3c_spd_byte_write(&i3c_adapter[2], DEVICE_IMX3102, IMX3102_MR65, IMX3102_MR65_SLAVE_PORT_EN_0 | IMX3102_MR65_INPUT_MASTER_SELECT);
		
		printf("i3c1 read imx3102    mr0:%02x", i3c_spd_byte_read(&i3c_adapter[1], DEVICE_IMX3102, 0));
		printf("i3c1 read spd5118[0] mr0:%02x", i3c_spd_byte_read(&i3c_adapter[1], DEVICE_SPD5118_0, 0));
		printf("i3c1 read spd5118[1] mr0:%02x", i3c_spd_byte_read(&i3c_adapter[1], DEVICE_SPD5118_1, 0));

		printf("i3c3 read imx3102    mr0:%02x", i3c_spd_byte_read(&i3c_adapter[3], DEVICE_IMX3102, 0));
		printf("i3c3 read spd5118[0] mr0:%02x", i3c_spd_byte_read(&i3c_adapter[3], DEVICE_SPD5118_0, 0));
		printf("i3c3 read spd5118[1] mr0:%02x", i3c_spd_byte_read(&i3c_adapter[3], DEVICE_SPD5118_1, 0));
		i3c_spd_byte_write(&i3c_adapter[1], DEVICE_IMX3102, IMX3102_MR65, IMX3102_MR65_SLAVE_PORT_EN_0);
		i3c_spd_byte_write(&i3c_adapter[3], DEVICE_IMX3102, IMX3102_MR65, IMX3102_MR65_SLAVE_PORT_EN_0);
		osDelay(1000);
	}

	for (int i = 0; i < 4; i++)
		i3c_free(&i3c_adapter[i]);
}

void demo_i3c_init(void)
{
	/* create a task for stress testing */
	i3c_task_attr.name = "demo_i3c";
	i3c_task_attr.priority = osPriorityBelowNormal;
	i3c_task_attr.stack_size = 2048;

	i3c_task_id = osThreadNew(i3c_task, NULL, &i3c_task_attr);

	/* register commands */
	i3c_cmd_register_obj(&i3c_adapter[0]);
	i3c_cmd_register_obj(&i3c_adapter[1]);
	i3c_cmd_register_obj(&i3c_adapter[2]);
	i3c_cmd_register_obj(&i3c_adapter[3]);
	FreeRTOS_CLIRegisterCommand(&i3c_cmd);
}