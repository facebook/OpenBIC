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
#include "FreeRTOS_CLI.h"
#include "log.h"
#include "cmsis_os.h"

#define I2C_DETECT_CMD_STR	"detect"
#define I2C_DUMP_CMD_STR	"dump"
#define I2C_GET_CMD_STR		"get"
#define I2C_SET_CMD_STR		"set"
#define I2C_SLAVE_CMD_STR	"slave"
#define I2C_MQUEUE_CMD_STR	"mqueue"
#define I2C_LOOP_CMD_STR	"loop"

#define IS_DETECT_CMD(x) (strncmp(x, I2C_DETECT_CMD_STR, 6) == 0)
#define IS_DUMP_CMD(x) (strncmp(x, I2C_DUMP_CMD_STR, 4) == 0)
#define IS_GET_CMD(x) (strncmp(x, I2C_GET_CMD_STR, 3) == 0)
#define IS_SET_CMD(x) (strncmp(x, I2C_SET_CMD_STR, 3) == 0)
#define IS_SLAVE_CMD(x) (strncmp(x, I2C_SLAVE_CMD_STR, 5) == 0)
#define IS_MQUEUE_CMD(x) (strncmp(x, I2C_MQUEUE_CMD_STR, 6) == 0)
#define IS_LOOP_CMD(x) (strncmp(x, I2C_LOOP_CMD_STR, 4) == 0)


#ifdef TARGET_EVB_AST2600	
i2c_t i2c[1];
#else
i2c_t i2c[8];
#endif

static osEventFlagsId_t i2c_loop_task_event;
static osThreadId_t tid_taskI2C;
static osThreadAttr_t tattr_taskI2C;

struct i2c_loop_info {
	uint32_t mbus_no;
	uint32_t sbus_no;

};

static void i2c_loop_task(void *argv)
{
	struct i2c_loop_info *i2c_loop = argv;
	uint32_t s_addr = 0x10;
	int i = 0;
	uint32_t xfer_length = 0x20;
	uint8_t *test_data = pvPortMallocNc(xfer_length);	

	printf("====== i2c loop master [%d] <==> slave [%d] ===== \n", i2c_loop->mbus_no, i2c_loop->sbus_no);
	printf("====== i2c[%d] slave enable addr[%x] ===== \n", i2c_loop->sbus_no, s_addr);
	i2c_slave_address(&i2c[i2c_loop->sbus_no], 0, s_addr);
	i2c_slave_mode(&i2c[i2c_loop->sbus_no], 1);

	while(1) {
//		printf("i2c write [%x] len %d : ", s_addr, xfer_length);
		for(i = 0; i < xfer_length; i++) {
			test_data[i] = i;
//			printf("%x ", test_data[i]);
		}
//		printf("\n");
		if(i2c_write(&i2c[i2c_loop->mbus_no], s_addr, test_data, xfer_length, I2C_M_STOP)) {
			printf("error \n");
	//		break;
		}

		for(i = 0; i < xfer_length; i++) {
			test_data[i] = 0;
	//		printf("%x ", test_data[i]);
		}

		//slave read 
//		printf("====== i2c[%d] slave mqueue read ===== \n", i2c_loop->sbus_no);
		xfer_length = i2c_slave_mqueue_read(&i2c[i2c_loop->sbus_no], test_data);
//		if(xfer_length) printf("\n smbus read [%d] : ", xfer_length);
		for(i = 0; i < xfer_length; i++) {
//			printf("%02x ", test_data[i]);
			if(test_data[i] != i)
				printf("FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFail \n");
		}
//		if(xfer_length) printf("\n");

		osDelay(1000);
		
	}
	vPortFreeNc(test_data);


}

static BaseType_t do_i2c(char *pcWriteBuffer, size_t xWriteBufferLen,
						 const char *pcCommandString)
{
	const char *pcParameter;
	BaseType_t xParameterStringLength, xReturn;
	static UBaseType_t uxParameterNumber = 0;
	static uint8_t cmd, ExpectedNumberOfParameters;
	static uint32_t bus_no, slave_addr, offset = 0, length = 0, i = 0;
	static uint32_t wr_index = 0;
	static uint8_t *pdata;
	static uint32_t mbus_no, sbus_no;
	static struct i2c_loop_info loop;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes
	the write buffer length is adequate, so does not check for buffer
	overflows. */
	(void)pcCommandString;
	(void)xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	if (uxParameterNumber == 0) {
		/* The first time the function is called after the command has
		been entered just a header string is returned. */
		sprintf(pcWriteBuffer, "\r");

		/* Next time the function is called the first parameter will be
		echoed back. */
		uxParameterNumber = 1U;

		/* There is more data to be returned as no parameters have been
		echoed back yet. */
		xReturn = pdPASS;
	} else {
		/* Obtain the parameter string. */
		pcParameter = FreeRTOS_CLIGetParameter(
				pcCommandString, /* The command string itself. */
				uxParameterNumber, /* Return the next parameter. */
				&xParameterStringLength /* Store the parameter string
						   length. */
		);

		if (pcParameter != NULL) {
			/* Return the parameter string. */
			memset(pcWriteBuffer, 0x00, xWriteBufferLen);
			switch (uxParameterNumber) {
			case 1:
				if (IS_DETECT_CMD(pcParameter)) {
					cmd = 0;
					ExpectedNumberOfParameters = 2;
				} else if (IS_DUMP_CMD(pcParameter)) {
					cmd = 1;
					ExpectedNumberOfParameters = 3;
				} else if (IS_GET_CMD(pcParameter)) {
					cmd = 2;
					ExpectedNumberOfParameters = 5;
				} else if (IS_SET_CMD(pcParameter)) {
					cmd = 3;
					ExpectedNumberOfParameters = 5;
				} else if (IS_SLAVE_CMD(pcParameter)) {
					cmd = 4;
					ExpectedNumberOfParameters = 3;
				} else if (IS_MQUEUE_CMD(pcParameter)) {
					cmd = 5;
					ExpectedNumberOfParameters = 2;
				} else if (IS_LOOP_CMD(pcParameter)) {
					cmd = 6;
					ExpectedNumberOfParameters = 3;
				} else {
					cmd = 0xff;
					ExpectedNumberOfParameters = 1;
				}
				break;
			case 2:
				bus_no = strtoul(pcParameter, NULL, 16);
				mbus_no = bus_no;
				break;
			case 3:
				slave_addr = strtoul(pcParameter, NULL, 16);
				sbus_no = slave_addr;
				break;
			case 4:
				offset = strtoul(pcParameter, NULL, 16);
				break;
			case 5:
				length = strtoul(pcParameter, NULL, 16);
				if(cmd == 3) {
					ExpectedNumberOfParameters += length;
					wr_index = 0;
				}
				break;
			default:
				if(cmd == 3) {
					if(uxParameterNumber == 6) {
						pdata = pvPortMallocNc(length + 2);
						pdata[0] = offset;
					}
					pdata[1 + wr_index] = strtoul(pcParameter, NULL, 16);
					wr_index++;
				}
				break;
			}

			if (uxParameterNumber == ExpectedNumberOfParameters) {
				if(i2c[bus_no].device) {
					switch (cmd) {
					case 0:	//detect
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
						//i2cdetect bus# slave_addr addr write 0 to scan bus
						printf("===== i2c[%d] detect ===== \n", bus_no);
						printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
						printf("00: -- -- -- ");
						uint8_t addr;
						for(addr = 0x3; addr < 0x77; addr++) {
							if(!(addr % 0x10))
								printf("\n%02x: ", addr);

							if(i2c_write(&i2c[bus_no], addr, NULL, 0, I2C_M_STOP))
								printf("-- ");
							else
								printf("%02x ", addr);
						}
						printf("\n");
						break;
					case 1:	//dump 
						pdata = pvPortMallocNc(0x100);
						printf("====== i2c[%d] dump addr [%x] ===== \n", bus_no, slave_addr);
						pdata[0] = 0x00;
						if(i2c_write(&i2c[bus_no], slave_addr, pdata, 1, 0)) {
							printf("error \n");
							break;
						}
						i2c_read(&i2c[bus_no], slave_addr, pdata, 0x100, I2C_M_STOP);

						printf("       0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");

						for(offset = 0x0; offset < 0x100; offset++) {
							if(!(offset % 0x10))
								printf("\n%04x: ", offset);
							
							printf("%02x ", pdata[offset]);
						}
						printf("\n");
						printf("=================================================\n");					
						vPortFreeNc(pdata);
						break;
					case 2:	//i2c get
						pdata = pvPortMallocNc(length);
						pdata[0] = offset;
						if(i2c_write(&i2c[bus_no], slave_addr, pdata, 1, 0)) {
							printf("error \n");
							break;
						}
						i2c_read(&i2c[bus_no], slave_addr, pdata, length, I2C_M_STOP);

						printf("====== i2c[%d] get addr[%x] offset[%x] size[%d] ===== \n", bus_no, slave_addr, offset, length);
						for(i = 0; i < length; i++) {
							printf("%02x ", pdata[i]);
						}
						printf("\n");
						printf("===================================================\n");					
						vPortFreeNc(pdata);
						break;
					case 3: //i2c set
						printf("i2c set [%x] offset %x len %d : ",slave_addr, offset, length);
						for(i = 0; i < length + 1; i++)
							printf("%x ", pdata[i]);
						printf("\n");
						if(i2c_write(&i2c[bus_no], slave_addr, pdata, length + 1, I2C_M_STOP)) {
							printf("error \n");
							break;
						}
						vPortFreeNc(pdata);
						break;
					case 4: //i2c slave enable
						if(slave_addr) {
							printf("====== i2c[%d] slave enable addr[%x] ===== \n", bus_no, slave_addr);
							i2c_slave_address(&i2c[bus_no], 0, slave_addr);
							i2c_slave_mode(&i2c[bus_no], 1);
						} else {
							printf("====== i2c[%d] slave disable ===== \n", bus_no, slave_addr);
							i2c_slave_mode(&i2c[bus_no], 0);
						}
						break;
					case 5: //i2c slave mqueue read
						pdata = pvPortMallocNc(I2C_SLAVE_BUFF_SIZE);
						//slave read 
						printf("====== i2c[%d] slave mqueue read ===== \n", bus_no);
						length = i2c_slave_mqueue_read(&i2c[bus_no], pdata);
						if(length) printf("\n smbus read [%d] : ", length);
						for(int i = 0; i < length; i++) {
							printf("%02x ", pdata[i]);
						}
						if(length) printf("\n");
						
						vPortFreeNc(pdata);
						break;
					case 6:	//i2c loop test
						loop.mbus_no = mbus_no;
						loop.sbus_no = sbus_no;
						i2c_loop_task_event = osEventFlagsNew(NULL);
						tattr_taskI2C.name = "demo_i2c_loop";
						tattr_taskI2C.priority = osPriorityBelowNormal;
						tattr_taskI2C.stack_size = 0x1000;
						
						tid_taskI2C = osThreadNew(i2c_loop_task, &loop, &tattr_taskI2C);
					
						break;
					default:
						printf("=== default xxxxxxxxxxxxxx\n");
						if (cmd == 3) {
							printf("todo ~~~ \n");
						}
						break;
					}
				}else 
					printf("bus driver not install \n");
					
				xReturn = pdFALSE;
				uxParameterNumber = 0;
			} else {
				/* There might be more parameters to return
				 * after this one. */
				xReturn = pdTRUE;
				uxParameterNumber++;
			}
			
		} else {
			/* No more parameters were found.  Make sure the write
			buffer does not contain a valid string. */
			pcWriteBuffer[0] = 0x00;

			/* No more data to return. */
			xReturn = pdFALSE;

			/* Start over the next time this command is executed. */
			uxParameterNumber = 0;
		}
	}

	return xReturn;
}

static const CLI_Command_Definition_t i2c_cmd =
{
	"i2c",
	"\r\ni2c:\r\n i2c master commands.\r\n\
	- write data  : i2c set [bus#] [dev addr] [offset] [size] [data]\r\n\
	- read data   : i2c get [bus#] [dev addr] [offset] [size]\r\n\
	- dump data   : i2c dump [slave id] [offset] [length]\r\n\
	- detect : i2c detect [bus#]\r\n\
	- mqueue : i2c mqueue [bus#]\r\n",
	do_i2c,
	-1
};

void demo_i2c_init(void)
{
	
#ifdef TARGET_EVB_AST2600
	printf("demo_i2c_init AST2600 EVB \n");
	i2c[0].device = &i2c7;
#else
	printf("demo_i2c_init AST1030 EVB \n");
	//for ast1030 evb
	i2c[0].device = &i2c0;
	i2c[1].device = &i2c1;
	i2c[2].device = &i2c2;
	i2c[3].device = &i2c3;
	i2c[4].device = &i2c4;
	i2c[5].device = &i2c5;
	i2c[6].device = &i2c8;
	i2c[7].device = &i2c9;
#endif


#ifdef TARGET_EVB_AST2600
	i2c_init(&i2c[0]);
#else
	//for ast1030 evb demo
	i2c_init(&i2c[0]);
	i2c_init(&i2c[1]);
	i2c_init(&i2c[2]);
	i2c_init(&i2c[3]);
	i2c_init(&i2c[4]);
	i2c_init(&i2c[5]);
	i2c_init(&i2c[6]);
	i2c_init(&i2c[7]);
#endif

	FreeRTOS_CLIRegisterCommand(&i2c_cmd);
}
