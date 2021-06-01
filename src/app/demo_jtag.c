/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include "stdlib.h"
#include "cmsis_os.h"
#include "board_device.h"
#include "objects.h"
#include "jtag_aspeed.h"
#include "getopt.h"
#include "FreeRTOS_CLI.h"
#include "log.h"

jtag_t jtag[2];
static const CLI_Command_Definition_t jtag_cmd;
static osThreadId_t tid_jtag_task;
static osThreadAttr_t tattr_jtag_task;

static void jtag_stress(void *argv)
{
	uint8_t *tdi_buffer;
	uint32_t bit_len = 0, byte_len = 0, value, count = 0;
	uint32_t lattice_id = 0x12b5043;
	int chip = 1;
	for (;;) {
		if ((count & 0xf) == 0) {
			log_info("JTAG read id %d times\n", count);
		}
		aspeed_jtag_target_init(&jtag[chip]);
		bit_len = 8;
		byte_len = (bit_len + 7) >> 3;
		tdi_buffer = malloc(byte_len);
		value = 0xe0;
		aspeed_jtag_ir_scan(&jtag[chip], bit_len, (uint8_t *)&value, tdi_buffer,
							TAP_IDLE);
		free(tdi_buffer);
		bit_len = 32;
		byte_len = (bit_len + 7) >> 3;
		tdi_buffer = malloc(byte_len);
		value = 0x0;
		aspeed_jtag_dr_scan(&jtag[chip], bit_len, (uint8_t *)&value, tdi_buffer,
							TAP_IDLE);
		if (memcmp(tdi_buffer, &lattice_id, 4))
			log_error("Read ID error: expect 0x%08x get 0x%08x\n", lattice_id,
					  tdi_buffer);
		free(tdi_buffer);
		osDelay(1000);
		count++;
	}
}

static void jtag_cmd_handler(int argc, char *argv[])
{
	char option;
	bool is_ir_scan = 0, reset = 0, stress = 0;
	int chip = -1, i;
	uint32_t bit_len = 0, byte_len = 0, value = 0, frequency = 8000000;
	uint8_t *tdi_buffer;
	optind = 0;
	while ((option = getopt(argc, argv, "hrc:idl:v:f:s")) != (char)-1) {
		switch (option) {
			case 'h':
			case '?':
				printf("%s", jtag_cmd.pcHelpString);
				return;
			case 'r':
				reset = 1;
				break;
			case 'c':
				chip = atoi(optarg);
				break;
			case 'i':
				is_ir_scan = 1;
				break;
			case 'd':
				is_ir_scan = 0;
				break;
			case 'l':
				bit_len = atoi(optarg);
				break;
			case 'v':
				value = strtoul(optarg, NULL, 16);
				break;
			case 'f':
				frequency = atoi(optarg);
				break;
			case 's':
				stress = 1;
				break;
			default:
				log_warn("unknown option -%c", option);
				break;
			}
	}
	if (stress) {
		tattr_jtag_task.name = "jtag_stress";
		tattr_jtag_task.priority = osPriorityBelowNormal;
		tattr_jtag_task.stack_size = 0x1000;

		tid_jtag_task = osThreadNew(jtag_stress, NULL, &tattr_jtag_task);
		return;
	}
	if (chip == -1) {
		log_warn("-c [chip id] is requirment\n");
		return;
	}
	if (frequency != 8000000)
		aspeed_jtag_set_clk(&jtag[chip], frequency);
	if (reset)
		aspeed_jtag_target_init(&jtag[chip]);
	else {
		byte_len = (bit_len + 7) >> 3;
		tdi_buffer = malloc(byte_len);
		if (is_ir_scan) {
			aspeed_jtag_ir_scan(&jtag[chip], bit_len, (uint8_t *)&value,
								tdi_buffer, TAP_IDLE);
			for (i = byte_len - 1; i >= 0; i--) {
				printf("%x", tdi_buffer[i]);
			}
			printf("\n");
		} else {
			aspeed_jtag_dr_scan(&jtag[chip], bit_len, (uint8_t *)&value,
								tdi_buffer, TAP_IDLE);
			for (i = byte_len - 1; i >= 0; i--) {
				printf("%x", tdi_buffer[i]);
			}
			printf("\n");
		}
		free(tdi_buffer);
	}

}

CLI_FUNC_DECL(jtag, jtag_cmd_handler);

static const CLI_Command_Definition_t jtag_cmd = 
{
	"jtag",
	"\r\njtag:\n \
	usage: \r\n \
		[-h]: Show this message\r\n \
		[-c <chip id>]: Select jtag chip (requirment)\r\n \
		[-r]: reset target device \r\n \
		[-i]: IR-scan\r\n \
		[-d]: DR-scan\r\n \
		[-l <length>]: Data length \r\n \
		[-v <value>(hex)]: Data value\r\n \
		[-s]: stress task run\r\n \
		[-f <TCK frequency>]: set jtag tck frequency \r\n",
	CLI_FUNC_SYM(jtag),
	-1
};

#ifdef CONFIG_SVF
extern void demo_svf_init(void);
#endif

void demo_jtag_init(void)
{

	jtag[0].device = &jtag0;
	jtag[1].device = &jtag1;
	aspeed_jtag_init(&jtag[0]);
	aspeed_jtag_init(&jtag[1]);

	FreeRTOS_CLIRegisterCommand(&jtag_cmd);
#ifdef CONFIG_SVF
	demo_svf_init();
#endif
}
