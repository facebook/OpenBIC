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
#include "gpio_aspeed.h"
#include "sgpiom_aspeed.h"
#include "FreeRTOS_CLI.h"
#include "log.h"
#include "hal_gpio.h"

extern gpio_t gpio[];

static const CLI_Command_Definition_t gpio_cmd;

void gpio_cb(void *para, uint32_t gpio_num)
{
	gpio_t *obj = (gpio_t *)para;
	log_info("Trigger gpio%d interrupt\n", gpio_num);
	obj->int_cb_unhook(obj, gpio_num);
}

static void gpio_cmd_handler(int argc, char *argv[])
{
	char option;
  bool get = 0, get_dir = 0, set = 0, dir = 0, info = 0, mon = 0;
	int set_v = 0, dir_v = 0, mon_type = 0;
	int chip = -1, number = -1;
	optind = 0;
	while ((option = getopt(argc, argv, "hac:n:s:d:m:gio")) != (char)-1) {
		switch (option) {
			case 'h':
			case '?':
				printf("%s", gpio_cmd.pcHelpString);
				return;
			case 'a':
				gpio_show();
				return;
			case 'c':
				chip = atoi(optarg);
				break;
			case 'n':
				number = atoi(optarg);
				break;
			case 's':
				set = 1;
				set_v = atoi(optarg);
				break;
			case 'd':
				dir = 1;
				dir_v = atoi(optarg);
				break;
			case 'm':
				mon = 1;
				mon_type = atoi(optarg);
				break;
			case 'g':
				get = 1;
				break;
			case 'i':
				info = 1;
				break;
      case 'o':
        get_dir = 1;
        break;
			default:
				log_warn("unknown option -%c", option);
				break;
			}
	}
	if (chip == -1) {
		log_warn("-c <chip id> is requirment\n");
		return;
	}

	if (info) {
		gpio[chip].info(&gpio[chip]);
		return;
	}

	if (number == -1) {
		log_warn("-n <gpio number> is requirment\n");
		return;
	}

	if (set) {
		gpio[chip].set_direction(&gpio[chip], number, GPIO_OUTPUT);
		gpio[chip].set(&gpio[chip], number, set_v);
	} else if (get) {
		gpio[chip].set_direction(&gpio[chip], number, GPIO_INPUT);
		printf("%d\n", gpio[chip].get(&gpio[chip], number));
  } else if (get_dir) {
    printf("%s\n", gpio[chip].get_direction(&gpio[chip], number) ? "out" : "in");
	} else if (dir) {
		gpio[chip].set_direction(&gpio[chip], number, dir_v);
	} else if (mon) {
		gpio[chip].set_direction(&gpio[chip], number,
									GPIO_INPUT);
		if (mon_type >= 5)
			gpio[chip].int_cb_unhook(&gpio[chip], number);
		else
			gpio[chip].int_cb_hook(&gpio[chip], number, mon_type, gpio_cb,
								   &gpio[chip]);
	}
}


CLI_FUNC_DECL(gpio, gpio_cmd_handler);

static const CLI_Command_Definition_t gpio_cmd = 
{
	"gpio",
	"\r\ngpio:\n \
	usage: \r\n \
		[-h]: Show this message\r\n \
		[-c <chip id>]: Select gpio chip (requirment)\r\n \
		[-n <gpio number>]: Select gpio number (requirment)\r\n \
		[-s <value>]: Set gpio value\r\n \
    [-o]: Get gpio direction\r\n \
		[-d <direction>]: Set gpio direction, 0:Input/1:Output\r\n \
		[-m <monitor type>]: Monitor gpio value change\r\n \
		monitor type: \r\n \
			0: Falling edge\r\n \
			1: Raising edge\r\n \
			2: Level low\r\n \
			3: Level high\r\n \
			4: Dual edge\r\n \
			>=5: Monitor disable\r\n \
		[-g]: Get gpio value\r\n \
		[-i]: Get gpio status\r\n",
	CLI_FUNC_SYM(gpio),
	-1
};

void demo_gpio_init(void)
{
	FreeRTOS_CLIRegisterCommand(&gpio_cmd);
}
