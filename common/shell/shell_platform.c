/*
    NAME: PLATFORM COMMAND
    FILE: shell_platform.c
    DESCRIPTION: OEM commands including gpio, sensor relative function access.
    CHIP/OS: AST1030 - Zephyr
    Note:
    (1) User command table 
          [topic]               [description]               [support]   [command]
        * GPIO                                              O
            * List group        List gpios' info in group   o           platform gpio list_group <gpio_device>
            * List all          List all gpios' info        o           platform gpio list_all
            * List multifnctl   List multi-fn-ctl regs      o           platform gpio multifnctl
            * Get               Get one gpio info           o           platform gpio get <gpio_num>
            * Set value         Set one gpio value          o           platform gpio set val <gpio_num> <value>
            * Set direction     Set one gpio direction      x           TODO
            * Set INT type      Set one gpio interrupt T    o           platform gpio set int_type <gpio_num> <type>
        * SENSOR                                            O
            * List all          List all sensors' info      o           platform sensor list_all
            * Get               Get one sensor info         o           platform sensor get <sensor_num>
            * Set polling en    Set one sensor polling      x           TODO
            * Set mbr           Set one sensor mbr          x           TODO
            * Set threshold     Set one sensor threshold    x           TODO
    (2) Some hard code features need to be modified if CHIP is different
    (3) GPIO List all/Get/Set value/Set direction are not protected by PINMASK_RESERVE_CHECK. It might cause system-hang problem!
    (4) "List multifnctl" command only lists relative registers mentioned in chip spec
        chapter "5.Multi-function Pins Mapping and Control" 
*/

#include "commands/gpio_shell.h"
#include "commands/info_shell.h"
#include "commands/sensor_shell.h"

#include <zephyr.h>
#include <sys/printk.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <shell/shell.h>
#include <device.h>
#include <devicetree.h>

/* Include VERSION */
#include "plat_version.h"

/* Include GPIO */
#include <drivers/gpio.h>
#include "plat_gpio.h"
#include "hal_gpio.h"

/* Include SENSOR */
#include "sensor.h"
#include "sdr.h"

/* Include config settings */
#include "shell_platform.h"

/* Sensor sub command */

/* MAIN command */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_platform_cmds,
			       SHELL_CMD(info, NULL, "Platform info.", cmd_info_print),
			       SHELL_CMD(gpio, &sub_gpio_cmds, "GPIO relative command.", NULL),
			       SHELL_CMD(sensor, &sub_sensor_cmds, "SENSOR relative command.",
					 NULL),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(platform, &sub_platform_cmds, "Platform commands", NULL);
