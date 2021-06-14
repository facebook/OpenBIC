/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include "cmsis_os.h"
#include "board_device.h"
#include "objects.h"
#include "getopt.h"
#include "adc_aspeed.h"
#include "FreeRTOS_CLI.h"
#include "log.h"

#define ALLOW_ERROR_VALUE 125
#define MEASURE_VOLTAGE 1800

adc_t adc[2];
static const CLI_Command_Definition_t adc_cmd;

static uint32_t adc_read(uint32_t index, uint32_t channel)
{
	return aspeed_adc_read(&adc[index], channel);
}

static uint32_t adc_read_mv(uint32_t index, uint32_t channel)
{
	return aspeed_adc_read_mv(&adc[index], channel);
}

static void adc_stress(void *argv)
{
	uint32_t mv, count = 0;
	int chip = -1, number = -1;
	for (;;) {
		for (chip = 0; chip <= 1; chip++) {
			for (number = 0; number <= 7; number++) {
				mv = adc_read_mv(chip, number);
				if (mv > MEASURE_VOLTAGE + ALLOW_ERROR_VALUE)
					log_error("ADC%d:%d %d > %d\n", chip, number, mv,
							  MEASURE_VOLTAGE + ALLOW_ERROR_VALUE);
				if (mv < MEASURE_VOLTAGE - ALLOW_ERROR_VALUE)
					log_error("ADC%d:%d %d < %d\n", chip, number, mv,
							  MEASURE_VOLTAGE - ALLOW_ERROR_VALUE);
				if ((count & 0xf) == 0) {
					log_info("ADC%d:%d %dmv\n", chip, number, mv);
				}
			}
		}
		osDelay(1000);
		count++;
	}
}

static osThreadId_t tid_adc_task;
static osThreadAttr_t tattr_adc_task;

static void adc_cmd_handler(int argc, char *argv[])
{
	char option;
	bool raw_data = 0, info = 0, stress = 0;
	int chip = -1, number = -1;
	optind = 0;
	while ((option = getopt(argc, argv, "hc:n:ris")) != (char)-1) {
		switch (option) {
			case 'h':
			case '?':
				printf("%s", adc_cmd.pcHelpString);
				return;
			case 'c':
				chip = atoi(optarg);
				break;
			case 'n':
				number = atoi(optarg);
				break;
			case 'r':
				raw_data = 1;
				break;
			case 'i':
				info = 1;
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
		tattr_adc_task.name = "adc_stress";
		tattr_adc_task.priority = osPriorityBelowNormal;
		tattr_adc_task.stack_size = 0x1000;

		tid_adc_task = osThreadNew(adc_stress, NULL, &tattr_adc_task);
		return;
	}
	if (chip == -1 || number == -1) {
		log_warn("-c [chip id] and -n [adc channel number] is requirment\n");
		return;
	}
	if (raw_data)
		printf("%d\n", adc_read(chip, number));
	else
		printf("%dmv\n", adc_read_mv(chip, number));
		
	if (info) {
		/* TODO: Show adc status */
	}
}

CLI_FUNC_DECL(adc, adc_cmd_handler);

static const CLI_Command_Definition_t adc_cmd = 
{
	"adc",
	"\r\nadc:\n \
	usage: \r\n \
		[-h]: Show this message\r\n \
		[-c <chip id>]: Select adc chip (requirment)\r\n \
		[-n <adc channel number>]: Select adc channel (requirment)\r\n \
		[-r]: Output format with raw data 0~2^10 \r\n \
		[-s]: stress task run \r\n \
		[-i]: Get adc status\r\n",
	CLI_FUNC_SYM(adc),
	-1
};

void demo_adc_init(void)
{
	adc[0].device = &adc0;
	adc[1].device = &adc1;
	aspeed_adc_init(&adc[0]);
	aspeed_adc_init(&adc[1]);

	FreeRTOS_CLIRegisterCommand(&adc_cmd);
}