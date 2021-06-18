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
#define I2C_PFR_CMD_STR		"pfr"
#define I2C_CHANGE_CLK_STR	"clock"
#define I2C_MAILBOX_STR		"mail"
#define I2C_MAILBOX_ADDR_STR "maddr"

#define IS_DETECT_CMD(x) (strncmp(x, I2C_DETECT_CMD_STR, 6) == 0)
#define IS_DUMP_CMD(x) (strncmp(x, I2C_DUMP_CMD_STR, 4) == 0)
#define IS_GET_CMD(x) (strncmp(x, I2C_GET_CMD_STR, 3) == 0)
#define IS_SET_CMD(x) (strncmp(x, I2C_SET_CMD_STR, 3) == 0)
#define IS_SLAVE_CMD(x) (strncmp(x, I2C_SLAVE_CMD_STR, 5) == 0)
#define IS_MQUEUE_CMD(x) (strncmp(x, I2C_MQUEUE_CMD_STR, 6) == 0)
#define IS_LOOP_CMD(x) (strncmp(x, I2C_LOOP_CMD_STR, 4) == 0)
#define IS_PFR_CMD(x) (strncmp(x, I2C_PFR_CMD_STR, 3) == 0)
#define IS_CLOCK_CMD(x) (strncmp(x, I2C_CHANGE_CLK_STR, 5) == 0)
#define IS_MAILBOX_CMD(x) (strncmp(x, I2C_MAILBOX_STR, 4) == 0)
#define IS_MAILBOX_ADDR_CMD(x) (strncmp(x, I2C_MAILBOX_ADDR_STR, 5) == 0)

#define I2C_THREAD_COUNT 	5
#define I2C_REMAINER_SEED 	8

#define ENABLE 	1
#define DISABLE   0 	

// PFR flags
#define I2C_PFR_SNOOP			1
#define I2C_PFR_MAILBOX			2
#define I2C_PFR_MAILBOX_FIFO	4
#define I2C_PFR_WHITE_LIST		8

// Commad define
#define I2C_DETECT_CMD		0
#define I2C_DUMP_CMD		1
#define I2C_GET_CMD		2
#define I2C_SET_CMD		3
#define I2C_SLAVE_CMD		4
#define I2C_MQUEUE_CMD		5
#define I2C_LOOP_CMD		6
#define I2C_PFR_CMD			7
#define I2C_CLOCK_CMD		8
#define I2C_MAILBOX_CMD		9
#define I2C_MAILBOX_ADDR_CMD		10

#ifdef TARGET_EVB_AST2600	
i2c_t i2c[1];
#else
//i2c_t i2c[8];
i2c_t i2c[16];
#endif

static osEventFlagsId_t i2c_loop_task_event[I2C_THREAD_COUNT];
static osThreadId_t tid_taskI2C[I2C_THREAD_COUNT];
static osThreadAttr_t tattr_taskI2C[I2C_THREAD_COUNT];
static char i2c_task_name[] = "i2cl";
static char i2c_task_count[] = "01234";
static uint8_t task_list[I2C_THREAD_COUNT] = {0};

struct i2c_loop_info {
	uint32_t mbus_no;
	uint32_t sbus_no;	
	uint32_t spbus_no;
	uint8_t mode;
	uint8_t pfr_feature;
	uint32_t loop_count;
	uint8_t loop_infinite;
	uint8_t task_list;
};

static void i2c_loop_task(void *argv)
{
	struct i2c_loop_info *i2c_loop = argv;
	uint32_t s_addr = 0x10;
	int i = 0;
	uint32_t xfer_length = 0x20;
	uint8_t *send_data = pvPortMallocNc(xfer_length);
	uint8_t *recv_data =  pvPortMallocNc(xfer_length);
	uint32_t loop_execute = 0x0;
	uint8_t index_value = 0;

	printf("====== i2c loop master [%d] <==> slave [%d] ===== \n", i2c_loop->mbus_no, i2c_loop->sbus_no);
	printf("====== i2c[%d] slave enable addr[%x] ===== \n", i2c_loop->sbus_no, s_addr);

	if((send_data == NULL)||(recv_data == NULL)){
		printf("====== i2c loop could not allocte NC buffer ====== \n");
		printf("====== ABORT ====== \n");
		goto LOOP_EXIT;
	}

	if(i2c_loop->loop_infinite)
		printf("====== i2c loop infinite ===== \n");
	else
		printf("====== i2c[%d] loop %d times===== \n", i2c_loop->mbus_no, i2c_loop->loop_count);

	i2c_slave_address(&i2c[i2c_loop->sbus_no], 0, s_addr, ENABLE);

	// Set master / slave everytime.
	i2c_slave_mode(&i2c[i2c_loop->sbus_no], ENABLE);

	// test mode change
	i2c[i2c_loop->mbus_no].mode = i2c_loop->mode;


#ifdef I2C_SNOOP_MODE
	// pfr feature setting
	if(i2c_loop->pfr_feature & I2C_PFR_SNOOP){
		printf("====== i2c[%d] snoop enable ===== \n", i2c_loop->spbus_no);			
		i2c_slave_address(&i2c[i2c_loop->spbus_no], 0, s_addr, ENABLE);
		i2c_slave_mode(&i2c[i2c_loop->spbus_no], ENABLE);		
		i2c_snoop_mode(&i2c[i2c_loop->spbus_no],ENABLE);

	} else {
		if(i2c_loop->spbus_no != 0xff) {			
			printf("====== i2c[%d] snoop disable ===== \n", i2c_loop->spbus_no); 		
			i2c_slave_address(&i2c[i2c_loop->spbus_no], 0, s_addr, DISABLE);
			i2c_slave_mode(&i2c[i2c_loop->spbus_no], DISABLE);				
			i2c_snoop_mode(&i2c[i2c_loop->spbus_no],DISABLE);
		}
	}
#endif 

	switch (i2c_loop->mode) {
		case DMA_MODE:
			printf("I2C DMA\n");
			break;
		case BUFF_MODE:
			printf("I2C BUF\n");
			break;			
		default:
			printf("I2C BYTE\n");
			break;
	}

	while(1) {			
		//printf("c : %d\n",loop_execute);
		//printf("index add : %x\n",(index_value * 0x10));
		
		//printf("i2c write [%x] len %d : ", s_addr, xfer_length);
		for(i = 0; i < xfer_length; i++) {
			send_data[i] = i + (index_value * 0x10);
			//printf("%x ", send_data[i]);
		}
		//printf("\n");
		
		if(i2c_write(&i2c[i2c_loop->mbus_no], s_addr, send_data, xfer_length, I2C_M_STOP)) {
			printf("error \n");
			//break;
		}
		
		//slave read 
		//printf("====== i2c[%d] slave mqueue read ===== \n", i2c_loop->sbus_no);
		xfer_length = i2c_slave_mqueue_read(&i2c[i2c_loop->sbus_no], recv_data);
		//if(xfer_length) printf("\n smbus read [%d] : ", xfer_length);
		for(i = 0; i < xfer_length; i++) {
			//printf("%02x ", recv_data[i]);
			if(recv_data[i] != send_data[i]) {
				printf("Fail at expected: %d ,loop: %d, recv: %x, send: %x\n",i,loop_execute,recv_data[i],send_data[i]);
			}
			else {
				if(((loop_execute % 100)==0x0)&&(i==0))
					printf("I2C PASS on %d at %d\n",i2c_loop->mbus_no,loop_execute);				
			}
		}
		
		printf("I2C PASS on %d at %d\n",i2c_loop->mbus_no,loop_execute);				

		osDelay(1000);

#ifdef I2C_SNOOP_MODE
		if(i2c_loop->pfr_feature & I2C_PFR_SNOOP)
			i2c_snoop_read(&i2c[i2c_loop->spbus_no], (xfer_length+1)<<1);
#endif

		loop_execute++;

		index_value = loop_execute % I2C_REMAINER_SEED;
		
		// I2C loop break rule
		if(!(i2c_loop->loop_infinite)) {
			if(loop_execute == (i2c_loop->loop_count))
				break;
		}
		
	}

LOOP_EXIT:
	// Disable slave id occpuy
	i2c_slave_address(&i2c[i2c_loop->sbus_no], 0, s_addr, DISABLE);

	if(recv_data != NULL)
		vPortFreeNc(recv_data);
	
	if(send_data != NULL)
		vPortFreeNc(send_data);
	
#ifdef I2C_SNOOP_MODE
	// Disable snoop
	if(i2c_loop->pfr_feature & I2C_PFR_SNOOP){		
		i2c_slave_address(&i2c[i2c_loop->spbus_no], 0, s_addr, DISABLE);
		i2c_slave_mode(&i2c[i2c_loop->spbus_no], DISABLE);				
		i2c_snoop_mode(&i2c[i2c_loop->spbus_no],DISABLE);
	}
#endif

	// Clear list occupy
	task_list[i2c_loop->task_list] = 0;
	
	printf("i2c_loop_task leave\n");

	osThreadExit();
}

static BaseType_t do_i2c(char *pcWriteBuffer, size_t xWriteBufferLen,
						 const char *pcCommandString)
{
	const char *pcParameter;
	BaseType_t xParameterStringLength, xReturn;
	static UBaseType_t uxParameterNumber = 0;
	static uint8_t cmd, ExpectedNumberOfParameters;
	static uint32_t bus_no, slave_addr, offset = 0, length = 0, i = 0;
	static uint32_t slave_addr1 = 0, slave_addr2 = 0;
	static uint32_t wr_index = 0;
	static uint8_t *pdata = NULL;
	static uint32_t mbus_no, sbus_no, spbus_no;
	static uint8_t mode = 0, enable = 0;
	static uint32_t clock = 0;	
	static uint8_t pfr = 0;
	static uint32_t loop_count = 0;
	static uint8_t loop_infinite = 0;
	static uint32_t base = 0;
	static struct i2c_loop_info loop[5] = {0};
	static char task_name[8];

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
					cmd = I2C_DETECT_CMD;
					ExpectedNumberOfParameters = 2;
				} else if (IS_DUMP_CMD(pcParameter)) {
					cmd = I2C_DUMP_CMD;
					ExpectedNumberOfParameters = 3;
				} else if (IS_GET_CMD(pcParameter)) {
					cmd = I2C_GET_CMD;
					ExpectedNumberOfParameters = 5;
				} else if (IS_SET_CMD(pcParameter)) {
					cmd = I2C_SET_CMD;
					ExpectedNumberOfParameters = 5;
				} else if (IS_SLAVE_CMD(pcParameter)) {
					cmd = I2C_SLAVE_CMD;
					ExpectedNumberOfParameters = 5;
				} else if (IS_MQUEUE_CMD(pcParameter)) {
					cmd = I2C_MQUEUE_CMD;
					ExpectedNumberOfParameters = 2;
				} else if (IS_LOOP_CMD(pcParameter)) {
					cmd = I2C_LOOP_CMD;
					ExpectedNumberOfParameters = 5;
				}else if (IS_PFR_CMD(pcParameter)) {
					cmd = I2C_PFR_CMD;
					ExpectedNumberOfParameters = 7;
				}else if (IS_CLOCK_CMD(pcParameter)) {
					cmd = I2C_CLOCK_CMD;
					ExpectedNumberOfParameters = 3;
				} else if (IS_MAILBOX_CMD(pcParameter)) {
					cmd = I2C_MAILBOX_CMD;
					ExpectedNumberOfParameters = 6;
				} else if (IS_MAILBOX_ADDR_CMD(pcParameter)) {
					cmd = I2C_MAILBOX_ADDR_CMD;
					ExpectedNumberOfParameters = 6;
				} else {
					cmd = 0xff;
					ExpectedNumberOfParameters = 1;
				}
				break;
			case 2:
				bus_no = strtoul(pcParameter, NULL, 10);
				mbus_no = bus_no;
				break;
			case 3:
				slave_addr = strtoul(pcParameter, NULL, 16);
				sbus_no = strtoul(pcParameter, NULL, 10);
				if(cmd == I2C_CLOCK_CMD) {
					clock = (uint32_t)strtoul(pcParameter, NULL, 10);
				} else if (cmd == I2C_MAILBOX_CMD) {
					enable = (uint32_t)strtoul(pcParameter, NULL, 10);
				}
				break;
			case 4:
				offset = strtoul(pcParameter, NULL, 16);
				if((cmd == I2C_LOOP_CMD)||(cmd == I2C_MAILBOX_CMD)) {
					mode = (uint8_t)offset;
				} else if ((cmd == I2C_SLAVE_CMD)||(cmd == I2C_MAILBOX_ADDR_CMD)) {
					slave_addr1 = offset;
				}
				break;
			case 5:
				if(cmd == I2C_SET_CMD) {					
					length = strtoul(pcParameter, NULL, 16);
					ExpectedNumberOfParameters += length;
					wr_index = 0;
				} else if ((cmd == I2C_LOOP_CMD) || (cmd == I2C_PFR_CMD)) {
					loop_infinite = 0;
					loop_count = strtoul(pcParameter, NULL, 10);
					if (loop_count == 0) {
						loop_infinite = 1;
					}
				} else if ((cmd == I2C_SLAVE_CMD)||(cmd == I2C_MAILBOX_ADDR_CMD)) {
					slave_addr2 = strtoul(pcParameter, NULL, 16);
				} else if (cmd == I2C_MAILBOX_CMD) {
					base = strtoul(pcParameter, NULL, 10);
				}
				break;
				
			case 6:
				if(cmd == I2C_SET_CMD) {
					if(uxParameterNumber == 6) {
						pdata = pvPortMallocNc(length + 2);
						if(pdata == NULL){
							printf("====== i2c set could not allocte NC buffer size 0x%x ====== \n",(length + 2));
							printf("====== ABORT ====== \n");
							return;
						}
						pdata[0] = offset;
					}
					pdata[1 + wr_index] = strtoul(pcParameter, NULL, 16);
					wr_index++;
				} else if (cmd == I2C_PFR_CMD) {
					pfr = strtoul(pcParameter, NULL, 16);
				} else if (cmd == I2C_MAILBOX_ADDR_CMD) {
					mode = strtoul(pcParameter, NULL, 16);
					printf("size : %x\n",mode);
				} else if (cmd == I2C_MAILBOX_CMD) {
					length = strtoul(pcParameter, NULL, 10);
				}
				break;
			case 7:
				if(cmd == I2C_SET_CMD) {
					pdata[1 + wr_index] = strtoul(pcParameter, NULL, 16);
					wr_index++;
				} else if(cmd == I2C_PFR_CMD) {
					spbus_no = strtoul(pcParameter, NULL, 10);
				}
				break;
			default:
				if(cmd == I2C_SET_CMD) {
					pdata[1 + wr_index] = strtoul(pcParameter, NULL, 16);
					wr_index++;
				}
				break;
			}

			if (uxParameterNumber == ExpectedNumberOfParameters) {
				if(i2c[bus_no].device) {
					switch (cmd) {
					case I2C_DETECT_CMD:	//detect
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
					case I2C_DUMP_CMD:	//dump 
						pdata = pvPortMallocNc(0x100);
						if(pdata == NULL){
							printf("====== i2c dump could not allocte NC buffer size 0x100 ====== \n");
							printf("====== ABORT ====== \n");
							return;
						}
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
					case I2C_GET_CMD:	//i2c get
						pdata = pvPortMallocNc(length);
						if(pdata == NULL){
							printf("====== i2c get could not allocte NC buffer size 0x%x ====== \n",length);
							printf("====== ABORT ====== \n");
							return;
						}
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
					case I2C_SET_CMD: //i2c set
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
					case I2C_SLAVE_CMD: //i2c slave enable						
						if((slave_addr)||(slave_addr1)||(slave_addr2)) {
							// slave addr
							if(slave_addr){
								printf("====== i2c[%d] slave enable addr[%x] ===== \n", bus_no, slave_addr);
								i2c_slave_address(&i2c[bus_no], 0, slave_addr,ENABLE);
							}
							else{
								printf("====== i2c[%d] slave disable addr ===== \n", bus_no);
								i2c_slave_address(&i2c[bus_no], 0, slave_addr,DISABLE);
							}

							// slave addr1
							if(slave_addr1){
								printf("====== i2c[%d] slave enable addr1[%x] ===== \n", bus_no, slave_addr1);
								i2c_slave_address(&i2c[bus_no], 1, slave_addr1,ENABLE);
							}
							else{
								printf("====== i2c[%d] slave disable addr1 ===== \n", bus_no);
								i2c_slave_address(&i2c[bus_no], 1, slave_addr1,DISABLE);
							}

							// slave addr2
							if(slave_addr2){
								printf("====== i2c[%d] slave enable addr2[%x] ===== \n", bus_no, slave_addr2);
								i2c_slave_address(&i2c[bus_no], 2, slave_addr2,ENABLE);
							}
							else{
								printf("====== i2c[%d] slave disable addr2 ===== \n", bus_no);
								i2c_slave_address(&i2c[bus_no], 2, slave_addr2,DISABLE);
							}

							// enable slave mode
							i2c_slave_mode(&i2c[bus_no], ENABLE);
						} else {
							// disable whole slave id setting
							i2c_slave_address(&i2c[bus_no], 0, slave_addr,DISABLE);
							i2c_slave_address(&i2c[bus_no], 1, slave_addr,DISABLE);
							i2c_slave_address(&i2c[bus_no], 2, slave_addr,DISABLE);
							printf("====== i2c[%d] slave disable ===== \n", bus_no, slave_addr);
							i2c_slave_mode(&i2c[bus_no], DISABLE);
						}
						break;
#ifdef I2C_MAILBOX_MODE	
						case I2C_MAILBOX_CMD: //i2c slave enable
							if(enable) {
								i2c_slave_mode(&i2c[bus_no], ENABLE);
								printf("====== i2c[%d] mailbox slave enable ===== \n", bus_no);
							} else {
								i2c_slave_mode(&i2c[bus_no], DISABLE);
								printf("====== i2c[%d] mailbox slave disable ===== \n", bus_no);
							}
							
							if(mode)
								printf("====== with internal SRAM ===== \n");
							else							
								printf("====== with external DMA ===== \n");
							
							i2c_mailbox_mode(&i2c[bus_no], enable, mode, base, (uint16_t)length);
						break;
						case I2C_MAILBOX_ADDR_CMD: //i2c slave enable	
							if((slave_addr)||(slave_addr1)||(slave_addr2)) {
								// slave addr
								if(slave_addr){
									printf("====== i2c[%d] mailbox slave enable addr[%x] ===== \n", bus_no, slave_addr);
									i2c_mailbox_address(&i2c[bus_no], 0, mode, slave_addr,ENABLE);
								}
								else{
									printf("====== i2c[%d] mailbox slave disable addr ===== \n", bus_no);
									i2c_mailbox_address(&i2c[bus_no], 0, mode, slave_addr,DISABLE);
								}
						
								// slave addr1
								if(slave_addr1){
									printf("====== i2c[%d] mailbox slave enable addr1[%x] ===== \n", bus_no, slave_addr1);
									i2c_mailbox_address(&i2c[bus_no], 1, mode, slave_addr1,ENABLE);
								}
								else{
									printf("====== i2c[%d] mailbox slave disable addr1 ===== \n", bus_no);
									i2c_mailbox_address(&i2c[bus_no], 1, mode, slave_addr1,DISABLE);
								}
						
								// slave addr2
								if(slave_addr2){
									printf("====== i2c[%d] mailbox slave enable addr2[%x] ===== \n", bus_no, slave_addr2);
									i2c_mailbox_address(&i2c[bus_no], 2, mode, slave_addr2,ENABLE);
								}
								else{
									printf("====== i2c[%d] mailbox slave disable addr2 ===== \n", bus_no);
									i2c_mailbox_address(&i2c[bus_no], 2, mode, slave_addr2,DISABLE);
								}						
							} else {
								// disable whole slave id setting
								i2c_mailbox_address(&i2c[bus_no], 0, mode, slave_addr,DISABLE);
								i2c_mailbox_address(&i2c[bus_no], 1, mode, slave_addr,DISABLE);
								i2c_mailbox_address(&i2c[bus_no], 2, mode, slave_addr,DISABLE);
								printf("====== i2c[%d] mailbox slave disable ===== \n", bus_no, slave_addr);
							}
							break;							
#endif
					case I2C_MQUEUE_CMD: //i2c slave mqueue read
						pdata = pvPortMallocNc(I2C_SLAVE_BUFF_SIZE);
						if(pdata == NULL){
							printf("====== i2c mqueue could not allocte NC buffer size 0x%x ====== \n", I2C_SLAVE_BUFF_SIZE);
							printf("====== ABORT ====== \n");
							return;
						}
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
					case I2C_LOOP_CMD:	//i2c loop test
					case I2C_PFR_CMD:	//i2c pfr test
						// copy string for task name
						strcpy(task_name,i2c_task_name);
						uint8_t size = strlen(task_name);
						uint8_t find = 0, i = 0;

						// find a empty task list
						for(i=0;i<I2C_THREAD_COUNT;i++){

							if(task_list[i] == 0) {
								task_list[i] = 1;
								find = 1;
								break;
							}
						}

						// threads are used
						if(find == 0) {							
							printf("i2c task list is full \n");
							break;
						}
												
						// fill parameters and create loop thread
						loop[i].mbus_no = mbus_no;
						loop[i].sbus_no = sbus_no;
						loop[i].mode = mode;
						loop[i].loop_count = loop_count;
						loop[i].loop_infinite = loop_infinite;
						loop[i].task_list = i;

						if(cmd == I2C_PFR_CMD){
							loop[i].pfr_feature = pfr;							
							loop[i].spbus_no = spbus_no;
						} else {
							loop[i].pfr_feature = 0x0;							
							loop[i].spbus_no = 0xff;
						}
						
						task_name[size] = i2c_task_count[i];
						task_name[size+1] = '\0';
						//printf("task name is %s\n",task_name);
						
						i2c_loop_task_event[i] = osEventFlagsNew(NULL);
						tattr_taskI2C[i].name = task_name;
						tattr_taskI2C[i].priority = osPriorityBelowNormal;
						tattr_taskI2C[i].stack_size = 0x2000;
						//tattr_taskI2C[i].stack_size = 40 * 1024;
											
						tid_taskI2C[i] = osThreadNew(i2c_loop_task, &loop[i], &tattr_taskI2C[i]);
						break;
						
					case I2C_CLOCK_CMD:	//i2c change clock
						// Clock change range is 1K ~ 5MHz
						if((clock == 0)||(clock>5000)){
							
							printf("The clock value is over bus behavior!!\n");
							printf("bus: %d has be changed as 400KHz\n",bus_no);
							clock = 400;
						}

						printf("bus: %d has been changed as %dKHz\n",bus_no, clock);
						
						i2c[bus_no].bus_frequency = clock*1000;
						i2c_frequency(&i2c[bus_no],clock);
						
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

//- pfr		 : i2c loop [M bus#] [S bus#] [mode] [counts] [PFR] [SP bus#]\n\
//- mailbox	 : i2c mail [bus#] [enable] [internal] [base] [length]\n\
//- mailbox addr : i2c maddr [bus#] [S addr] [S addr1] [S addr2][type]\n\

static const CLI_Command_Definition_t i2c_cmd =
{
	"i2c",
	"\r\ni2c:\r\n i2c master commands.\r\n\
	- write data : i2c set [bus#] [dev addr] [offset] [size] [data]\r\n\
	- read data  : i2c get [bus#] [dev addr] [offset] [size]\r\n\
	- dump data  : i2c dump [bus#] [dev addr] \r\n\
	- detect     : i2c detect [bus#]\r\n\
	- mqueue     : i2c mqueue [bus#]\r\n\
	- clock      : i2c clock [bus#] [clock(KHz)]\r\n\
	- loop       : i2c loop [M bus#] [S bus#] [mode] [counts] n\
 i2c slave commands.\r\n\
	- slave      : i2c slave [bus#] [S addr] [S addr1] [S addr2]\r\n",
	do_i2c,
	-1
};

void demo_i2c_init(void)
{
	static uint8_t i = 0, count = 0;

#ifdef TARGET_EVB_AST2600
	printf("demo_i2c_init AST2600 EVB \n");
	i2c[0].device = &i2c7;
	count = 1;
#elif defined I2C_FPGA
	printf("demo_i2c_init AST1060 FPGA \n");
	//for ast1060 fpga
	i2c[0].device = &i2c0;
	i2c[1].device = &i2c1;
	i2c[2].device = &i2c2;
	i2c[3].device = &i2c3;
	i2c[4].device = &i2c4;
	i2c[5].device = &i2c5;
	i2c[6].device = &i2c6;
	i2c[7].device = &i2c7;
	i2c[8].device = &i2c8;
	i2c[9].device = &i2c9;
	i2c[10].device = &i2c10;
	i2c[11].device = &i2c11;
	i2c[12].device = &i2c12;
	i2c[13].device = &i2c13;
	i2c[14].device = &i2c14;
	i2c[15].device = &i2c15;
	count = 16;
#elif TARGET_EVB_AST1030
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
	count = 8;
#endif

	printf("count : %d \n",count);

	// initial i2c by differnt count
	for(i = 0;i<count;i++)
	{
		i2c_init(&i2c[i]);
	}

	FreeRTOS_CLIRegisterCommand(&i2c_cmd);
}
