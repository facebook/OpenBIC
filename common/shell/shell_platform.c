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

#include <zephyr.h>
#include <sys/printk.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <shell/shell.h>
#include <device.h>
#include <devicetree.h>

/* Include GPIO */
#include <drivers/gpio.h>
#include "plat_gpio.h"
#include "hal_gpio.h"

/* Include SENSOR */
#include "sensor.h"

/* Include config settings */
#include "shell_platform.h"

/*********************************************************************************************************
 * TOOL SECTION
**********************************************************************************************************/
/* 
    Command GPIO 
*/
#if PINMASK_RESERVE_CHECK
enum CHECK_RESV { CHECK_BY_GROUP_IDX, CHECK_BY_GLOBAL_IDX };
static int gpio_check_reserve(const struct device *dev, int gpio_idx, enum CHECK_RESV mode)
{
	if (!dev) {
		return 1;
	}

	const struct gpio_driver_config *const cfg = (const struct gpio_driver_config *)dev->config;

	if (!cfg->port_pin_mask) {
		return 1;
	}

	int gpio_idx_in_group;
	if (mode == CHECK_BY_GROUP_IDX) {
		gpio_idx_in_group = gpio_idx;
	} else if (mode == CHECK_BY_GLOBAL_IDX) {
		gpio_idx_in_group = gpio_idx % GPIO_GROUP_SIZE;
	} else {
		return 1;
	}

	if ((cfg->port_pin_mask & (gpio_port_pins_t)BIT(gpio_idx_in_group)) == 0U) {
		return 1;
	}

	return 0;
}
#endif

static int gpio_access_cfg(const struct shell *shell, int gpio_idx, enum GPIO_ACCESS mode,
			   int *data)
{
	if (!shell) {
		return 1;
	}

	if (gpio_idx >= GPIO_CFG_SIZE || gpio_idx < 0) {
		shell_error(shell, "gpio_access_cfg - gpio index out of bound!");
		return 1;
	}

	switch (mode) {
	case GPIO_READ:
		if (gpio_cfg[gpio_idx].is_init == DISABLE) {
			return 1;
		}

		uint32_t g_val = sys_read32(GPIO_GROUP_REG_ACCESS[gpio_idx / 32]);
		uint32_t g_dir = sys_read32(GPIO_GROUP_REG_ACCESS[gpio_idx / 32] + 0x4);

		char *pin_prop = (gpio_cfg[gpio_idx].property == OPEN_DRAIN) ? "OD" : "PP";
		char *pin_dir = (gpio_cfg[gpio_idx].direction == GPIO_INPUT) ? "input" : "output";

		char *pin_dir_reg = "I";
		if (g_dir & BIT(gpio_idx % 32))
			pin_dir_reg = "O";

		int val = gpio_get(gpio_idx);
		if (val == 0 || val == 1) {
			shell_print(shell, "[%-3d] %-35s: %-3s | %-6s(%s) | %d(%d)", gpio_idx,
				    gpio_name[gpio_idx], pin_prop, pin_dir, pin_dir_reg, val,
				    GET_BIT_VAL(g_val, gpio_idx % 32));
		} else {
			shell_print(shell, "[%-3d] %-35s: %-3s | %-6s(%s) | %s", gpio_idx,
				    gpio_name[gpio_idx], pin_prop, pin_dir, pin_dir_reg, "resv");
		}

		break;

	case GPIO_WRITE:
		if (!data) {
			shell_error(shell, "gpio_access_cfg - GPIO_WRITE value empty!");
			return 1;
		}

		if (*data != 0 && *data != 1) {
			shell_error(
				shell,
				"gpio_access_cfg - GPIO_WRITE value should only accept 0 or 1!");
			return 1;
		}

		if (gpio_set(gpio_idx, *data)) {
			shell_error(shell, "gpio_access_cfg - GPIO_WRITE failed!");
			return 1;
		}

		break;

	default:
		shell_error(shell, "gpio_access_cfg - No such mode %d!", mode);
		break;
	}

	return 0;
}

static int gpio_get_group_idx_by_dev_name(const char *dev_name)
{
	if (!dev_name) {
		return -1;
	}

	int group_idx = -1;
	for (int i = 0; i < ARRAY_SIZE(GPIO_GROUP_NAME_LST); i++) {
		if (!strcmp(dev_name, GPIO_GROUP_NAME_LST[i]))
			group_idx = i;
	}

	return group_idx;
}

static const char *gpio_get_name(const char *dev_name, int pin_num)
{
	if (!dev_name) {
		return NULL;
	}

	int name_idx = -1;
	name_idx = pin_num + 32 * gpio_get_group_idx_by_dev_name(dev_name);

	if (name_idx == -1) {
		return NULL;
	}

	if (name_idx >= GPIO_CFG_SIZE) {
		return "Undefined";
	}

	return gpio_name[name_idx];
}

/* 
    Command SENSOR 
*/
static bool sensor_access_check(uint8_t sensor_num)
{
	bool (*access_checker)(uint8_t);
	access_checker = sensor_config[sensor_config_index_map[sensor_num]].access_checker;

	return (access_checker)(sensor_config[sensor_config_index_map[sensor_num]].num);
}

static int sensor_get_idx_by_sensor_num(uint16_t sensor_num)
{
	for (int sen_idx = 0; sen_idx < SDR_COUNT; sen_idx++) {
		if (sensor_num == sensor_config[sen_idx].num)
			return sen_idx;
	}

	return -1;
}

static int sensor_access(const struct shell *shell, int sensor_num, enum SENSOR_ACCESS mode)
{
	if (!shell) {
		return 1;
	}

	if (sensor_num >= SENSOR_NUM_MAX || sensor_num < 0) {
		return 1;
	}

	switch (mode) {
	/* Get sensor info by "sensor_config" table */
	case SENSOR_READ:;
		int sen_idx = sensor_get_idx_by_sensor_num(sensor_num);
		if (sen_idx == -1) {
			shell_error(shell, "No such sensor number!");
			return 1;
		}
		char *check_access =
			(sensor_access_check(sensor_config[sen_idx].num) == true) ? "O" : "X";
		shell_print(shell, "[0x%-2x] %-35s: %-10s | access[%s] | %-30s | %-8d",
			    sensor_config[sen_idx].num, "Unsupported name",
			    sensor_type_name[sensor_config[sen_idx].type], check_access,
			    sensor_status_name[sensor_config[sen_idx].cache_status],
			    sensor_config[sen_idx].cache);
		break;

	case SENSOR_WRITE:
		/* TODO */
		break;

	default:
		break;
	}

	return 0;
}

/*********************************************************************************************************
 * COMMAND FUNCTION SECTION
**********************************************************************************************************/
/* 
    Command header 
*/
static int cmd_info_print(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(
		shell,
		"========================{SHELL COMMAND INFO}========================================");
	shell_print(shell, "* NAME:          Platform command");
	shell_print(shell, "* DESCRIPTION:   Commands that could be used to debug or validate.");
	shell_print(shell, "* DATE/VERSION:  none");
	shell_print(shell, "* CHIP/OS:       AST1030 - Zephyr");
	shell_print(shell, "* Note:          none");
	shell_print(
		shell,
		"========================{SHELL COMMAND INFO}========================================");
	return 0;
}

/* 
    Command GPIO
*/
static void cmd_gpio_cfg_list_group(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_warn(shell, "Help: platform gpio list_group <gpio_device>");
		return;
	}

	const struct device *dev;
	dev = device_get_binding(argv[1]);

	if (!dev) {
		shell_error(shell, "Device [%s] not found!", argv[1]);
		return;
	}

	int g_idx = gpio_get_group_idx_by_dev_name(dev->name);
	int max_group_pin = num_of_pin_in_one_group_lst[g_idx];

	uint32_t g_val = sys_read32(GPIO_GROUP_REG_ACCESS[g_idx]);
	uint32_t g_dir = sys_read32(GPIO_GROUP_REG_ACCESS[g_idx] + 0x4);

	for (int index = 0; index < max_group_pin; index++) {
		if (gpio_cfg[g_idx * 32 + index].is_init == DISABLE) {
			shell_print(shell, "[%-3d][%s %-3d] %-35s: -- | %-9s | NA",
				    g_idx * 32 + index, dev->name, index, "gpio_disable", "i/o");
			continue;
		}

#if PINMASK_RESERVE_CHECK
		/* avoid pin_mask from devicetree "gpio-reserved" */
		if (gpio_check_reserve(dev, index, CHECK_BY_GROUP_IDX)) {
			shell_print(shell, "[%-3d][%s %-3d] %-35s: -- | %-9s | NA",
				    g_idx * 32 + index, dev->name, index, "gpio_reserve", "i/o");
			continue;
		}
#endif
		char *pin_dir = "output";
		if (gpio_cfg[g_idx * 32 + index].direction == GPIO_INPUT) {
			pin_dir = "input";
		}

		char *pin_dir_reg = "I";
		if (g_dir & BIT(index)) {
			pin_dir_reg = "O";
		}

		char *pin_prop =
			(gpio_cfg[g_idx * 32 + index].property == OPEN_DRAIN) ? "OD" : "PP";

		int rc;
		rc = gpio_pin_get(dev, index);
		if (rc >= 0) {
			shell_print(shell, "[%-3d][%s %-3d] %-35s: %2s | %-6s(%s) | %d(%d)",
				    g_idx * 32 + index, dev->name, index,
				    gpio_get_name(dev->name, index), pin_prop, pin_dir, pin_dir_reg,
				    rc, GET_BIT_VAL(g_val, index));
		} else {
			shell_error(shell, "[%-3d][%s %-3d] %-35s: %2s | %-6s | err[%d]",
				    g_idx * 32 + index, dev->name, index,
				    gpio_get_name(dev->name, index), pin_prop, pin_dir, rc);
		}
	}

	return;
}

static void cmd_gpio_cfg_list_all(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 1) {
		shell_warn(shell, "Help: platform gpio list_all");
		return;
	}

	for (int gpio_idx = 0; gpio_idx < GPIO_CFG_SIZE; gpio_idx++)
		gpio_access_cfg(shell, gpio_idx, GPIO_READ, NULL);

	return;
}

static void cmd_gpio_cfg_get(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_warn(shell, "Help: platform gpio get <gpio_idx>");
		return;
	}

	int gpio_index = strtol(argv[1], NULL, 10);
	if (gpio_access_cfg(shell, gpio_index, GPIO_READ, NULL))
		shell_error(shell, "gpio[%d] get failed!", gpio_index);

	return;
}

static void cmd_gpio_cfg_set_val(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_warn(shell, "Help: platform gpio set val <gpio_idx> <data>");
		return;
	}

	int gpio_index = strtol(argv[1], NULL, 10);
	int data = strtol(argv[2], NULL, 10);

	if (gpio_access_cfg(shell, gpio_index, GPIO_WRITE, &data))
		shell_error(shell, "gpio[%d] --> %d ,failed!", gpio_index, data);
	else
		shell_print(shell, "gpio[%d] --> %d ,success!", gpio_index, data);

	return;
}

static void cmd_gpio_cfg_set_int_type(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_warn(shell, "Help: platform gpio set int_type <gpio_idx> <type>");
		shell_warn(
			shell,
			"     type: [0]disable [1]edge-rise [2]edge-fall [3]edge-both [4]low [5]high");
		return;
	}

	int gpio_index = strtol(argv[1], NULL, 10);
	int type_idx = strtol(argv[2], NULL, 10);

	if (type_idx >= ARRAY_SIZE(int_type_table) || type_idx < 0) {
		shell_error(shell, "Wrong index of type!");
		shell_warn(
			shell,
			"type: [0]disable [1]edge-rise [2]edge-fall [3]edge-both [4]low [5]high");
		return;
	}

	if (gpio_interrupt_conf(gpio_index, int_type_table[type_idx]))
		shell_error(shell, "gpio[%d] --> type[%d] failed!", gpio_index, type_idx);
	else
		shell_print(shell, "gpio[%d] --> type[%d] success!", gpio_index, type_idx);

	return;
}

static void cmd_gpio_muti_fn_ctl_list(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 1) {
		shell_warn(shell, "Help: platform gpio multifnctl");
		return;
	}

	printf("[   REG    ]  hi                                      lo\n");
	for (int lst_idx = 0; lst_idx < ARRAY_SIZE(GPIO_MULTI_FUNC_PIN_CTL_REG_ACCESS); lst_idx++) {
		uint32_t cur_status = sys_read32(GPIO_MULTI_FUNC_PIN_CTL_REG_ACCESS[lst_idx]);
		printf("[0x%x]", GPIO_MULTI_FUNC_PIN_CTL_REG_ACCESS[lst_idx]);
		for (int i = 32; i > 0; i--) {
			if (!(i % 4)) {
				printf(" ");
			}
			if (!(i % 8)) {
				printf(" ");
			}
			printf("%d", (int)GET_BIT_VAL(cur_status, i - 1));
		}
		printf("\n");
	}

	shell_print(shell, "\n");
}

/*
    Command SENSOR
*/
static void cmd_sensor_cfg_list_all(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 1) {
		shell_warn(shell, "Help: platform sensor list_all");
		return;
	}

	shell_print(
		shell,
		"---------------------------------------------------------------------------------");
	for (int sen_idx = 0; sen_idx < SDR_COUNT; sen_idx++)
		sensor_access(shell, sensor_config[sen_idx].num, SENSOR_READ);

	shell_print(
		shell,
		"---------------------------------------------------------------------------------");
}

static void cmd_sensor_cfg_get(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_warn(shell, "Help: platform sensor get <sensor_num>");
		return;
	}

	int sen_num = strtol(argv[1], NULL, 16);

	sensor_access(shell, sen_num, SENSOR_READ);
}

/*********************************************************************************************************
 * COMMAND DECLARE SECTION
**********************************************************************************************************/

/* GPIO sub command */
static void device_gpio_name_get(size_t idx, struct shell_static_entry *entry)
{
	const struct device *dev = shell_device_lookup(idx, GPIO_DEVICE_PREFIX);

	if (entry == NULL) {
		printf("device_gpio_name_get passed null entry\n");
		return;
	}

	entry->syntax = (dev != NULL) ? dev->name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}
SHELL_DYNAMIC_CMD_CREATE(gpio_device_name, device_gpio_name_get);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_gpio_set_cmds,
			       SHELL_CMD(val, NULL, "Set pin value.", cmd_gpio_cfg_set_val),
			       SHELL_CMD(int_type, NULL, "Set interrupt pin type.",
					 cmd_gpio_cfg_set_int_type),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_gpio_cmds,
	SHELL_CMD(list_group, &gpio_device_name, "List all GPIO config from certain group.",
		  cmd_gpio_cfg_list_group),
	SHELL_CMD(list_all, NULL, "List all GPIO config.", cmd_gpio_cfg_list_all),
	SHELL_CMD(get, NULL, "Get GPIO config", cmd_gpio_cfg_get),
	SHELL_CMD(set, &sub_gpio_set_cmds, "Set GPIO config", NULL),
	SHELL_CMD(multifnctl, NULL, "List all GPIO multi-function control regs.",
		  cmd_gpio_muti_fn_ctl_list),
	SHELL_SUBCMD_SET_END);

/* Sensor sub command */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_sensor_cmds,
			       SHELL_CMD(list_all, NULL, "List all SENSOR config.",
					 cmd_sensor_cfg_list_all),
			       SHELL_CMD(get, NULL, "Get SENSOR config", cmd_sensor_cfg_get),
			       SHELL_SUBCMD_SET_END);

/* MAIN command */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_platform_cmds,
			       SHELL_CMD(note, NULL, "Note list.", cmd_info_print),
			       SHELL_CMD(gpio, &sub_gpio_cmds, "GPIO relative command.", NULL),
			       SHELL_CMD(sensor, &sub_sensor_cmds, "SENSOR relative command.",
					 NULL),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(platform, &sub_platform_cmds, "Platform commands", NULL);
