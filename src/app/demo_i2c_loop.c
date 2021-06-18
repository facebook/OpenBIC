/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include "cmsis_os.h"
#include "board_device.h"
#include "objects.h"
#include "i2c_aspeed.h"

//i2c master
osThreadId_t tid_taskA;
osThreadAttr_t tattr_taskA;

//i2c slave
osThreadId_t tid_taskB;
osThreadAttr_t tattr_taskB;

//i2c0 as master
static void i2c0_task(void *argv)
{
	extern aspeed_device_t i2c0;

	uint8_t xfer_buf[0x40];
    i2c_t o_i2c0 = {.device = &i2c0};
	uint8_t addr = 0x3a;
	int cnt = 0;

	printf("i2c0_task \n");

	o_i2c0.mode = DMA_MODE;
	i2c_init(&o_i2c0);

/*
[AST /]$ i2cdetect -y 7
0 1 2 3 4 5 6 7 8 9 a b c d e f
00: -- -- -- -- -- -- 09 -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- 2e --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: 50 51 52 53 54 55 56 57 -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
*/
#if 0
	//i2cdetect bus 0 : addr write 0 to scan bus
	printf("i2cdetect bus 0 \n");
	printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
	printf("00: -- -- -- ");

	for(addr = 0x3; addr < 0x77; addr++) {
		if(!(addr % 0x10))
			printf("\n%02x: ", addr);

		if(i2c_write(&o_i2c0, addr, NULL, 0, I2C_M_STOP))
			printf("-- ");
		else
			printf("%02x ", addr);
	}
	printf("\n");
#endif
	printf("Send mqueue \n");
	while (1) {
		//master mqueue write 
		xfer_buf[0] = cnt;
		xfer_buf[1] = 0x11;
		xfer_buf[2] = 0x22;
		xfer_buf[3] = 0x33;
		xfer_buf[4] = 0x44;
		xfer_buf[5] = 0x55;
		xfer_buf[6] = 0x66;
		xfer_buf[7] = 0x77;
		xfer_buf[8] = 0x88;
		xfer_buf[9] = 0x99;
		xfer_buf[0xa] = 0xaa;
		xfer_buf[0xb] = 0xbb;
		xfer_buf[0xc] = 0xcc;
		xfer_buf[0xd] = 0xdd;
		xfer_buf[0xe] = 0xee;
		xfer_buf[0xf] = 0xff;
		i2c_write(&o_i2c0, addr, xfer_buf, 0x10, I2C_M_STOP);
		osDelay(1000);
		cnt++;
	}
}


static void i2c1_task(void *argv)
{
	extern aspeed_device_t i2c1;

	int length;
	uint8_t mqueue_buf[0x40];
    i2c_t o_i2c1 = {.device = &i2c1};

	o_i2c1.mode = DMA_MODE;
	i2c_init(&o_i2c1);

	i2c_slave_address(&o_i2c1, 0, 0x3a);
	i2c_slave_mode(&o_i2c1, 1);

	while (1) {
		//slave read 
		length = i2c_slave_mqueue_read(&o_i2c1, mqueue_buf);
		if(length) printf("\n smbus read [%d] : ", length);
		for(int i = 0; i < length; i++) {
			printf("%02x ", mqueue_buf[i]);
		}
		if(length) printf("\n");

		osDelay(1000);
	}

}


void demo_i2c_init(void)
{
	extern aspeed_device_t i2c_global;
    i2c_global_t g_i2c = {.device = &i2c_global};

	//fpga debug
//	writel(0x1, 0x7e6e21e0);
	i2c_global_init(&g_i2c);

	//i2c0 <-> i2c1 loop test
	tattr_taskA.name = "demo_i2c0";
	tattr_taskA.priority = osPriorityBelowNormal;
	tattr_taskA.stack_size = 0x1000;

	tattr_taskB.name = "demo_i2c1";
	tattr_taskB.priority = osPriorityBelowNormal;
	tattr_taskB.stack_size = 0x1000;

	tid_taskA = osThreadNew(i2c0_task, NULL, &tattr_taskA);
	tid_taskB = osThreadNew(i2c1_task, NULL, &tattr_taskB);
	
}
