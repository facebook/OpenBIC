/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include "objects.h"
#include "i3c_aspeed.h"
#include "i3c_api.h"
#include "log.h"
#include "internal.h"
#include "getopt.h"

typedef struct i3c_adaptor_s {
	uint32_t nobjs;
	i3c_t *objs[8];
} i3c_adaptor_t;
i3c_adaptor_t i3c_adaptor = { .nobjs = 0 };

#define MAX_USR_XFERS	8
#define MAX_USR_DATA_SIZE	MAX(ASPEED_I3C_TX_FIFO_SIZE, ASPEED_I3C_RX_FIFO_SIZE)

static i3c_usr_xfer_t usr_xfers[MAX_USR_XFERS];
static uint8_t data_bufs[MAX_USR_XFERS][MAX_USR_DATA_SIZE];

void i3c_cmd_test_begin_hook(void);
void i3c_cmd_test_finish_hook(void);

__WEAK void i3c_cmd_test_begin_hook(void)
{

}

__WEAK void i3c_cmd_test_finish_hook(void)
{

}

void print_i3c_msg(i3c_msg_t *msg)
{
	int i;
	for (i = 0; i < msg->len; i++) {
		if ((i & 0xf) == 0)
			printf("\n[%02x] ", i);

		printf("%02x ", msg->buf[i]);
	}
	printf("\n\n");
}

void i3c_cmd_register_obj(i3c_t *obj)
{
	i3c_adaptor.objs[i3c_adaptor.nobjs++] = obj;
	if (i3c_adaptor.nobjs == 8)
		log_error("exceed MAX I3C objects\n");
}

static uint32_t args_to_wdata(char *arg, uint8_t *buf)
{
	char *data_ptrs[MAX_USR_DATA_SIZE];
	int i = 0, len = 0;
	
	data_ptrs[i] = strtok(arg, ",");
	while (data_ptrs[i] && i < MAX_USR_DATA_SIZE - 1)
		data_ptrs[++i] = strtok(NULL, ",");

	for (len = 0; len < i; len++)
		buf[len] = strtoul(data_ptrs[len], NULL, 16);

	return len;
}

static void i3c_cmd_handler(int argc, char *argv[])
{
	i3c_t *obj = NULL;
	int nxfer = 0;
	int i, j;
	uint32_t slave_idx;
	char opt;

	optind = 0;
	while ((opt = getopt(argc, argv, "o:s:w:r:t:ih")) != (char)-1) {
		switch (opt) {
		case 'o':
			i = atoi(optarg);
			if (i >= i3c_adaptor.nobjs)
				return;
			obj = i3c_adaptor.objs[i];
			log_debug("i3c object index: %d\n", i);
			break;
		case 's':
			slave_idx = atoi(optarg);
			log_debug("slave device index: %d\n", slave_idx);
			break;
		case 'w':
			memset(&data_bufs[nxfer][0], 0, MAX_USR_DATA_SIZE);
			usr_xfers[nxfer].rnw = 0;
			usr_xfers[nxfer].data.in = &data_bufs[nxfer][0];
			usr_xfers[nxfer].len = args_to_wdata(optarg, usr_xfers[nxfer].data.in);
			nxfer++;
			break;
		case 'r':
			usr_xfers[nxfer].rnw = 1;
			usr_xfers[nxfer].data.out = &data_bufs[nxfer][0];
			usr_xfers[nxfer].len = atoi(optarg);
			nxfer++;
			break;
		case 'i':
			for (i = 0; i < i3c_adaptor.nobjs; i++) {
				i3c_t *o = i3c_adaptor.objs[i];
				printf("\ni3c object %d @%02x (%s)\n", i, o->self_addr, (o->role == 0) ? "master" : "slave");
				if (o->role == 0) {
					for (j = 0; j < o->n_slaves; j++)
						printf("\tslave %d @%02x\n", j, o->slaves[j].dynamic_addr);
				}
			}
			break;
		case 't':
			i = atoi(optarg);
			if (i) {
				printf("enable i3c test task\n");
				i3c_cmd_test_begin_hook();
			} else {
				printf("disable i3c test task\n");
				i3c_cmd_test_finish_hook();
			}
			break;
		case 'h':
		default:
			printf("%s", i3c_cmd.pcHelpString);
			return;
		}
	}

	if (nxfer) {
		log_debug("%d transfers\n", nxfer);
		for (i = 0; i < nxfer; i++) {
			log_debug("xfr[%d]: rnw=%d len=%d\n", i, usr_xfers[i].rnw, usr_xfers[i].len);
		}
		
		if (obj->role == 0) {
			/* master device issues transfers */
			aspeed_i3c_priv_xfer(obj, slave_idx,  usr_xfers, nxfer);
		} else {
			/* slave device issues IBI with data */
			for (i = 0; i < nxfer; i++)
				aspeed_i3c_slave_issue_sir(obj, usr_xfers[i].data.in, usr_xfers[i].len);
		}

		for (i = 0; i < nxfer; i++) {
			if (usr_xfers[i].rnw)
				for (j = 0; j < usr_xfers[i].len; j++)
					printf("%02x\n", usr_xfers[i].data.out[j]);
		}
	}
}
CLI_FUNC_DECL(i3c, i3c_cmd_handler);

const CLI_Command_Definition_t i3c_cmd = {
    "i3c",
    "\r\ni3c:\r\n i3c commands.\r\n\
	-h                : help, show this message \r\n\
	-o <object index> : the index of the i3c object\r\n\
	-s <slave index>  : the index of the slave device to be r/w (if i3c object is a master device)\r\n\
	-r <length>       : length of the read data in byte\r\n\
	-w <data block>   : write data, seperated by ','\r\n\
	-t <enable>       : stress test enable\r\n\
	-i                : info of the i3c buses\r\n",
    CLI_FUNC_SYM(i3c),
	-1
};