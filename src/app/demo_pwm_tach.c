/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include "cmsis_os.h"
#include "board_device.h"
#include "objects.h"
#include "pwm_tach_aspeed.h"
#include "FreeRTOS_CLI.h"
#include "getopt.h"
#include "log.h"

pwm_t pwm[16];
tach_t tach[16];
static const CLI_Command_Definition_t pwm_cmd;
static const CLI_Command_Definition_t tach_cmd;

void show_pwm_info(void)
{
    uint16_t index, pwm_status;
	pwm_status = aspeed_g_pwm_tach_get_pwm_status(&g_pwm_tach);
    for (index = 0; index < 16; index++)
	{
		printf("pwm%d: [%s]\n", index, (pwm_status & (1 << index)) ? ("used") : ("un-used"));
	}
}

static void pwm_cmd_handler(int argc, char *argv[])
{
	char option;
	bool get = 0, set = 0, info = 0, freq_set = 0;
	int channel = 0, duty, freq = 0;
	optind = 0;
	while ((option = getopt(argc, argv, "hc:s:f:gi")) != (char)-1) {
		switch (option) {
			case 'h':
				printf("%s", pwm_cmd.pcHelpString);
				return;
			case 'c':
				channel = atoi(optarg);
				break;
			case 'g':
				get = 1;
				break;
			case 's':
				set = 1;
				duty = atoi(optarg);
				break;
			case 'f':
				freq_set = 1;
				freq = atoi(optarg);
				break;
			case 'i':
				info = 1;
				break;
			case '?':
				return;
			default:
				printf("unknown option -%c", option);
				break;
			}
	}
	if (set) {
		aspeed_pwm_set_duty(&pwm[channel], duty);
	} else if (get) {
		printf("%d\n", aspeed_pwm_get_duty(&pwm[channel]));
	} else if (freq_set) {
		aspeed_pwm_set_freq(&pwm[channel], freq);
	} else if (info) {
		show_pwm_info();
	}
}

CLI_FUNC_DECL(pwm, pwm_cmd_handler);

static const CLI_Command_Definition_t pwm_cmd = 
{
	"pwm",
	"\r\npwm:\n \
	usage: \r\n \
		[-c <channel number>]: Select pwm channel (default:0)\r\n \
		[-s <duty percent>]: Set pwm duty\r\n \
		[-g]: Get pwm duty\r\n \
		[-i]: Get pwm status\r\n \
		[-f <frequency>]: Set pwm frequecy\r\n",
	CLI_FUNC_SYM(pwm),
	-1
};

void show_tach_info(void)
{
    uint16_t index, tach_status;
    tach_status = aspeed_g_pwm_tach_get_tach_status(&g_pwm_tach);
    for (index = 0; index < 16; index++)
	{
		printf("tach%d: [%s]\n", index, (tach_status & (1 << index)) ? ("used") : ("un-used"));
	}
}
void tach_cb(void *para)
{
    tach_t *obj = (tach_t *)para;
	aspeed_tach_int_cb_unhook(obj);
    log_info("Trigger tach threshold interrupt\n");
}

static void tach_cmd_handler(int argc, char *argv[])
{
	char option;
	bool get = 0, mon = 0, info = 0;
	int channel = 0, threshold = 1800, mon_condition = 0;
	optind = 0;
	while ((option = getopt(argc, argv, "hc:gt:m:i")) != (char)-1) {
		switch (option) {
			case 'h':
				printf("%s", tach_cmd.pcHelpString);
				return;
			case 'c':
				channel = atoi(optarg);
				break;
			case 'g':
				get = 1;
				break;
			case 't':
				threshold = atoi(optarg);
				break;
			case 'm':
				mon = 1;
				mon_condition = atoi(optarg);
				break;
			case 'i':
				info = 1;
				break;
			case '?':
				return;
			default:
				printf("unknown option -%c", option);
				break;
			}
	}
	if (get) {
		printf("%d\n", aspeed_tach_get_rpm(&tach[channel]));
	} else if (mon) {
		aspeed_tach_int_cb_hook(&tach[channel], threshold ,mon_condition, tach_cb, &tach[channel]);
	} else if (info) {
		show_tach_info();
	}
}

CLI_FUNC_DECL(tach, tach_cmd_handler);

static const CLI_Command_Definition_t tach_cmd = 
{
	"tach",
	"\r\ntach:\n \
	usage: \r\n \
		[-c <channel number>]: Select tach channel (default:0)\r\n \
		[-g]: Get tach rpm\r\n \
		[-t <threshold>]: set tach threshold (unit: rpm, default: 1800)\r\n \
		[-m <condition>]: set monitor condition and start monitor (0:less/1:more)\r\n \
		[-i]: Get tach status\r\n",
	CLI_FUNC_SYM(tach),
	-1
};

void demo_pwm_tach_init(void)
{
	pwm[0].device = &pwm0;
	pwm[1].device = &pwm1;
	pwm[2].device = &pwm2;
	pwm[3].device = &pwm3;
	pwm[4].device = &pwm4;
	pwm[5].device = &pwm5;
	pwm[6].device = &pwm6;
	pwm[7].device = &pwm7;
	pwm[8].device = &pwm8;
	pwm[9].device = &pwm9;
	pwm[10].device = &pwm10;
	pwm[11].device = &pwm11;
	pwm[12].device = &pwm12;
	pwm[13].device = &pwm13;
	pwm[14].device = &pwm14;
	pwm[15].device = &pwm15;
	aspeed_pwm_init(&pwm[0]);
	aspeed_pwm_init(&pwm[1]);
	aspeed_pwm_init(&pwm[2]);
	aspeed_pwm_init(&pwm[3]);
	aspeed_pwm_init(&pwm[4]);
	aspeed_pwm_init(&pwm[5]);
	aspeed_pwm_init(&pwm[6]);
	aspeed_pwm_init(&pwm[7]);
	aspeed_pwm_init(&pwm[8]);
	aspeed_pwm_init(&pwm[9]);
	aspeed_pwm_init(&pwm[10]);
	aspeed_pwm_init(&pwm[11]);
	aspeed_pwm_init(&pwm[12]);
	aspeed_pwm_init(&pwm[13]);
	aspeed_pwm_init(&pwm[14]);
	aspeed_pwm_init(&pwm[15]);

	tach[0].device = &tach0;
	tach[1].device = &tach1;
	tach[2].device = &tach2;
	tach[3].device = &tach3;
	tach[4].device = &tach4;
	tach[5].device = &tach5;
	tach[6].device = &tach6;
	tach[7].device = &tach7;
	tach[8].device = &tach8;
	tach[9].device = &tach9;
	tach[10].device = &tach10;
	tach[11].device = &tach11;
	tach[12].device = &tach12;
	tach[13].device = &tach13;
	tach[14].device = &tach14;
	tach[15].device = &tach15;
	aspeed_tach_init(&tach[0]);
	aspeed_tach_init(&tach[1]);
	aspeed_tach_init(&tach[2]);
	aspeed_tach_init(&tach[3]);
	aspeed_tach_init(&tach[4]);
	aspeed_tach_init(&tach[5]);
	aspeed_tach_init(&tach[6]);
	aspeed_tach_init(&tach[7]);
	aspeed_tach_init(&tach[8]);
	aspeed_tach_init(&tach[9]);
	aspeed_tach_init(&tach[10]);
	aspeed_tach_init(&tach[11]);
	aspeed_tach_init(&tach[12]);
	aspeed_tach_init(&tach[13]);
	aspeed_tach_init(&tach[14]);
	aspeed_tach_init(&tach[15]);

	FreeRTOS_CLIRegisterCommand(&pwm_cmd);
	FreeRTOS_CLIRegisterCommand(&tach_cmd);
}
