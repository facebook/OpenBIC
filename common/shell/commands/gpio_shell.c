/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "gpio_shell.h"
#include "hal_gpio.h"
#include "plat_gpio.h"
#include <drivers/gpio.h>
#include <stdio.h>

/*
 * Constants
 *
 * These constants are in the source file instead of the header file due to build problems 
 * with using the SHELL_STATIC_SUBCMD_SET_CREATE macro. 
 */

#define GET_BIT_VAL(val, n) ((val & BIT(n)) >> (n))
#define GPIO_DEVICE_PREFIX "GPIO0_"

int num_of_pin_in_one_group_lst[GPIO_GROUP_NUM] = { 32, 32, 32, 32, 32, 16 };
char GPIO_GROUP_NAME_LST[GPIO_GROUP_NUM][10] = { "GPIO0_A_D", "GPIO0_E_H", "GPIO0_I_L",
					       "GPIO0_M_P", "GPIO0_Q_T", "GPIO0_U_V" };
enum GPIO_ACCESS { GPIO_READ, GPIO_WRITE };

gpio_flags_t int_type_table[] = { GPIO_INT_DISABLE,   GPIO_INT_EDGE_RISING, GPIO_INT_EDGE_FALLING,
				  GPIO_INT_EDGE_BOTH, GPIO_INT_LEVEL_LOW,   GPIO_INT_LEVEL_HIGH };

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
 * Command GPIO 
 */
void cmd_gpio_cfg_list_group(const struct shell *shell, size_t argc, char **argv)
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

void cmd_gpio_cfg_list_all(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 1 && argc != 2) {
		shell_warn(shell, "Help: platform gpio list_all <key_word(optional)>");
		return;
	}

	char *key_word = NULL;
	if (argc == 2)
		key_word = argv[1];

	for (int gpio_idx = 0; gpio_idx < GPIO_CFG_SIZE; gpio_idx++) {
		if (key_word && !strstr(gpio_name[gpio_idx], key_word))
			continue;
		gpio_access_cfg(shell, gpio_idx, GPIO_READ, NULL);
	}

	return;
}

void cmd_gpio_cfg_get(const struct shell *shell, size_t argc, char **argv)
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

void cmd_gpio_cfg_set_val(const struct shell *shell, size_t argc, char **argv)
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

void cmd_gpio_cfg_set_int_type(const struct shell *shell, size_t argc, char **argv)
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

void cmd_gpio_muti_fn_ctl_list(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 1) {
		shell_warn(shell, "Help: platform gpio multifnctl");
		return;
	}

	printf("[   REG    ]  hi                                      lo\n");
	for (int lst_idx = 0; lst_idx < GPIO_MULTI_FUNC_CFG_SIZE; lst_idx++) {
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

/* GPIO sub command */
void device_gpio_name_get(size_t idx, struct shell_static_entry *entry)
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
