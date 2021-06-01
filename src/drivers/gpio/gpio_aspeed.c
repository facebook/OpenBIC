/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include "common.h"
#include "util.h"
#include "hal_def.h"
#include "reset_aspeed.h"
#include "clk_aspeed.h"
#include "pinctrl_aspeed.h"
#include "cmsis_os.h"
#include <string.h>
#include "gpio_aspeed.h"
#include "log.h"
#include "irq_aspeed.h"

uint16_t gpio_offset_data[] = {
	[0] = offsetof(gpio_register_t, group0_data),
	[1] = offsetof(gpio_register_t, group1_data),
	[2] = offsetof(gpio_register_t, group2_data),
	[3] = offsetof(gpio_register_t, group3_data),
	[4] = offsetof(gpio_register_t, group4_data),
	[5] = offsetof(gpio_register_t, group5_data),
	[6] = offsetof(gpio_register_t, group6_data),
};

uint16_t gpio_offset_dirction[] = {
	[0] = offsetof(gpio_register_t, group0_dir),
	[1] = offsetof(gpio_register_t, group1_dir),
	[2] = offsetof(gpio_register_t, group2_dir),
	[3] = offsetof(gpio_register_t, group3_dir),
	[4] = offsetof(gpio_register_t, group4_dir),
	[5] = offsetof(gpio_register_t, group5_dir),
	[6] = offsetof(gpio_register_t, group6_dir),
};

uint16_t gpio_offset_int_status[] = {
	[0] = offsetof(gpio_register_t, group0_int_status),
	[1] = offsetof(gpio_register_t, group1_int_status),
	[2] = offsetof(gpio_register_t, group2_int_status),
	[3] = offsetof(gpio_register_t, group3_int_status),
	[4] = offsetof(gpio_register_t, group4_int_status),
	[5] = offsetof(gpio_register_t, group5_int_status),
	[6] = offsetof(gpio_register_t, group6_int_status),
};

static void aspeed_gpio_isr_handle(aspeed_device_t *gpio)
{
	aspeed_gpio_priv_t *priv = (aspeed_gpio_priv_t *)gpio->private;
	aspeed_gpio_adapter_t *gpio_adapter;
	uint32_t group_num = DIV_ROUND_UP(priv->gpio_num, 32);
	uint32_t group_idx;
	uint32_t gpio_number;
	uint32_t int_index, int_pendding;
	gpio_int_status_register_t *int_reg;
	for (group_idx = 0; group_idx < group_num; group_idx++) {
		int_reg = (gpio_int_status_register_t
						   *)(gpio->base + gpio_offset_int_status[group_idx]);
		int_pendding = int_reg->value;
		int_index = 0;
		log_debug("group%d interrupt pendding = 0x%08x\n", group_idx,
				  int_pendding);
		while (int_pendding) {
			if (int_pendding & 0x1) {
				gpio_number = (group_idx << 3) + int_index;
				gpio_adapter = &priv->adapter[gpio_number];
				if (gpio_adapter->cb != NULL) {
					gpio_adapter->cb(gpio_adapter->para, gpio_number);
				}
				int_reg->value = BIT(int_index);
			}
			int_index++;
			int_pendding >>= 1;
		}
	}
}

static void aspeed_gpio_isr(void)
{
	uint32_t n_irq = aspeed_irq_get_current_irq();
	aspeed_device_t *gpio = aspeed_irq_get_isr_context(n_irq);
	aspeed_gpio_isr_handle(gpio);
}

static void aspeed_gpio_init_cmd_src_sel(gpio_t *obj)
{
	aspeed_device_t *gpio = obj->device;
	gpio_register_t *gpio_register = (volatile gpio_register_t *)gpio->base;
	gpio_register->cmd_src_sel.fields.mst1 = GPIO_SEL_PRI;
	gpio_register->cmd_src_sel.fields.mst2 = GPIO_SEL_LPC;
	gpio_register->cmd_src_sel.fields.mst3 = GPIO_SEL_SSP;
	gpio_register->cmd_src_sel.fields.mst4 = GPIO_SEL_PRI;
	gpio_register->cmd_src_sel.fields.mst5 = GPIO_SEL_PRI;
	gpio_register->cmd_src_sel.fields.lock = 1;
}

static hal_status_t aspeed_gpio_cmd_source_init(gpio_t *obj)
{
	aspeed_device_t *gpio = obj->device;
	aspeed_gpio_priv_t *priv = (aspeed_gpio_priv_t *)gpio->private;
	uint8_t gpio_group_num = DIV_ROUND_UP(priv->gpio_num, 8);
	uint32_t gpio_group_occupy, gpio_byte_idx;
	uint32_t cmd_source = priv->cmd_source & GENMASK(gpio_group_num, 0);
	for (gpio_byte_idx = 0; gpio_byte_idx < gpio_group_num; gpio_byte_idx++) {
		gpio_group_occupy = (cmd_source >> gpio_byte_idx) & 0x1;
		if (gpio_group_occupy) {
			log_debug("Occupy GPIO GROUP %d for minibmc\n", gpio_byte_idx);
#if defined(CONFIG_AST1030_SERIES)
			aspeed_gpio_cmd_src_set(obj, gpio_byte_idx * 8, GPIO_CMD_SRC_ARM);
#else
			aspeed_gpio_cmd_src_set(obj, gpio_byte_idx * 8, GPIO_CMD_SRC_MST3);
#endif
		}
	}
	return HAL_OK;
}

hal_status_t aspeed_gpio_cmd_src_set(gpio_t *obj, int gpio_number,
									 uint8_t cmd_src)
{
	aspeed_device_t *gpio = obj->device;
	aspeed_gpio_priv_t *priv = (aspeed_gpio_priv_t *)gpio->private;
	gpio_register_t *gpio_register = (volatile gpio_register_t *)gpio->base;
	gpio_index_register_t index;
	if (gpio_number >= priv->gpio_num) {
		log_error("invalie gpio #%d \n", gpio_number);
		return HAL_ERROR;
	}
	index.value = 0;
	index.fields.index_type = GPIO_CMD_SRC;
	index.fields.index_command = GPIO_INDEX_WRITE;
	index.fields.index_number = gpio_number;
	index.fields.index_data = cmd_src;
	gpio_register->index.value = index.value;
	return HAL_OK;
}

hal_status_t aspeed_gpio_int_cb_hook(gpio_t *obj, int gpio_number,
									 uint8_t int_type, gpio_cb_t cb, void *para)
{
	aspeed_device_t *gpio = obj->device;
	aspeed_gpio_priv_t *priv = (aspeed_gpio_priv_t *)gpio->private;
	gpio_register_t *gpio_register = (volatile gpio_register_t *)gpio->base;
	aspeed_gpio_adapter_t *gpio_adapter;
	gpio_index_register_t index;
	index.fields.index_type = GPIO_INTERRUPT;
	index.fields.index_command = GPIO_INDEX_WRITE;
	index.fields.index_data = BIT(0) | (int_type << 1);
	index.fields.index_number = gpio_number;
	if (gpio_number >= priv->gpio_num) {
		log_error("invalie gpio #%d \n", gpio_number);
		return HAL_ERROR;
	}
	gpio_adapter = &priv->adapter[gpio_number];
	gpio_adapter->cb = cb;
	gpio_adapter->para = para;
	gpio_adapter->int_type = int_type;
	log_debug("gpio index = 0x%08x\n", index.value);
	gpio_register->index.value = index.value;
	return HAL_OK;
}

hal_status_t aspeed_gpio_int_cb_unhook(gpio_t *obj, int gpio_number)
{
	aspeed_device_t *gpio = obj->device;
	aspeed_gpio_priv_t *priv = (aspeed_gpio_priv_t *)gpio->private;
	gpio_register_t *gpio_register = (volatile gpio_register_t *)gpio->base;
	aspeed_gpio_adapter_t *gpio_adapter;
	gpio_index_register_t index;
	index.fields.index_type = GPIO_INTERRUPT;
	index.fields.index_command = GPIO_INDEX_WRITE;
	index.fields.index_data = 0x0;
	index.fields.index_number = gpio_number;
	if (gpio_number >= priv->gpio_num) {
		log_error("invalie gpio #%d \n", gpio_number);
		return HAL_ERROR;
	}
	gpio_adapter = &priv->adapter[gpio_number];
	gpio_adapter->cb = NULL;
	gpio_adapter->para = NULL;
	log_debug("gpio index = 0x%08x\n", index.value);
	gpio_register->index.value = index.value;
	return HAL_OK;
}

hal_status_t aspeed_gpio_pin_request(gpio_t *obj, int gpio_number)
{
	/*TODO: Implement pinmux setting of gpio */
	return HAL_OK;
}

int aspeed_gpio_get(gpio_t *obj, int gpio_number)
{
	aspeed_device_t *gpio = obj->device;
	aspeed_gpio_priv_t *priv = (aspeed_gpio_priv_t *)gpio->private;
	gpio_data_value_register_t *data_reg;
	if (gpio_number >= priv->gpio_num) {
		log_error("invalie gpio #%d \n", gpio_number);
		return -HAL_ERROR;
	}
	uint32_t group_idx = gpio_number >> 5;
	uint32_t byte_idx = (gpio_number & 0x1f) >> 3;
	uint32_t bit_idx = (gpio_number & 0x7);
	uint8_t ret_value;
	log_debug("gpio %d at group%d, set%d, bit%d\n", gpio_number, group_idx,
			  byte_idx, bit_idx);
	data_reg = (gpio_data_value_register_t *)(gpio->base +
											  gpio_offset_data[group_idx]);
	ret_value = (data_reg->fields.data[byte_idx] & BIT(bit_idx)) >> bit_idx;
	return ret_value;
}

hal_status_t aspeed_gpio_set(gpio_t *obj, int gpio_number, int val)
{
	aspeed_device_t *gpio = obj->device;
	aspeed_gpio_priv_t *priv = (aspeed_gpio_priv_t *)gpio->private;
	gpio_register_t *gpio_register = (volatile gpio_register_t *)gpio->base;
	gpio_index_register_t index;
	if (gpio_number >= priv->gpio_num) {
		log_error("invalie gpio #%d \n", gpio_number);
		return HAL_ERROR;
	}
	index.fields.index_type = GPIO_DATA;
	index.fields.index_command = GPIO_INDEX_WRITE;
	index.fields.index_data = val;
	index.fields.index_number = gpio_number;
	log_debug("gpio index = 0x%08x\n", index.value);
	gpio_register->index.value = index.value;
	return HAL_OK;
}

int aspeed_gpio_get_direction(gpio_t *obj, int gpio_number)
{
	aspeed_device_t *gpio = obj->device;
	aspeed_gpio_priv_t *priv = (aspeed_gpio_priv_t *)gpio->private;
	gpio_direction_register_t *dir_reg;
	if (gpio_number >= priv->gpio_num) {
		log_error("invalie gpio #%d \n", gpio_number);
		return HAL_ERROR;
	}
	uint32_t group_idx = gpio_number >> 5;
	uint32_t byte_idx = (gpio_number & 0x1f) >> 3;
	uint32_t bit_idx = (gpio_number & 0x7);
	uint8_t ret_value;

	log_debug("gpio %d at group%d, set%d, bit%d\n", gpio_number, group_idx,
			  byte_idx, bit_idx);
	dir_reg = (gpio_direction_register_t *)(gpio->base +
											gpio_offset_dirction[group_idx]);
	ret_value = (dir_reg->fields.direction[byte_idx] & BIT(bit_idx)) >> bit_idx;
	return ret_value;
}

hal_status_t aspeed_gpio_set_direction(gpio_t *obj, int gpio_number, int direct)
{
	aspeed_device_t *gpio = obj->device;
	aspeed_gpio_priv_t *priv = (aspeed_gpio_priv_t *)gpio->private;
	gpio_register_t *gpio_register = (volatile gpio_register_t *)gpio->base;
	gpio_index_register_t index;
	if (gpio_number >= priv->gpio_num) {
		log_error("invalie gpio #%d \n", gpio_number);
		return HAL_ERROR;
	}
	index.fields.index_type = GPIO_DIRECTION;
	index.fields.index_command = GPIO_INDEX_WRITE;
	index.fields.index_data = direct;
	index.fields.index_number = gpio_number;
	log_debug("gpio index = 0x%08x\n", index.value);
	gpio_register->index.value = index.value;
	return HAL_OK;
}

hal_status_t aspeed_gpio_set_config(gpio_t *obj, int gpio_number, int config)
{
	/*TODO: Implement gpio configure: debounce time, reset tolerant */
	return HAL_OK;
}

hal_status_t aspeed_gpio_info(gpio_t *obj)
{
	aspeed_device_t *gpio = obj->device;
	aspeed_gpio_priv_t *priv = (aspeed_gpio_priv_t *)gpio->private;
	printf("type:gpio@%08x\n", gpio->base);
	printf("number:%d\n", priv->gpio_num);
	return HAL_OK;
}

hal_status_t aspeed_gpio_init(gpio_t *obj)
{
	aspeed_device_t *gpio = obj->device;
	aspeed_gpio_priv_t *priv = (aspeed_gpio_priv_t *)gpio->private;
	uint32_t gpio_idx;
	hal_status_t ret;

	if (gpio->init) {
		log_error("gpio@%08x already occupied\n", gpio->base);
		return HAL_BUSY;
	}
	priv->adapter = malloc(priv->gpio_num * sizeof(aspeed_gpio_adapter_t));
	if (priv->adapter == NULL) {
		log_error("GPIO adapter malloc %d byte error\n",
				  priv->gpio_num * sizeof(aspeed_gpio_adapter_t));
		return HAL_ERROR;
	}
	memset(priv->adapter, 0, priv->gpio_num * sizeof(aspeed_gpio_adapter_t));

	aspeed_gpio_init_cmd_src_sel(obj);
	aspeed_gpio_cmd_source_init(obj);

	for (gpio_idx = 0; gpio_idx < priv->gpio_num; gpio_idx++) {
		ret = aspeed_gpio_int_cb_unhook(obj, gpio_idx);
		if (ret != HAL_OK) {
			log_error("GPIO%d cb unhook error\n", gpio_idx);
			return HAL_ERROR;
		}
		ret = aspeed_gpio_set_direction(obj, gpio_idx, GPIO_INPUT);
		if (ret != HAL_OK) {
			log_error("GPIO%d set input mode error\n", gpio_idx);
			return HAL_ERROR;
		}
		priv->adapter[gpio_idx].evt_id = osEventFlagsNew(NULL);
		if (priv->adapter[gpio_idx].evt_id == NULL)
			log_error("fail to create evt_id\n");
	}

	if (priv->irq)
		aspeed_irq_register(priv->irq, (uint32_t)aspeed_gpio_isr, (void *)gpio);
	else {
		log_error("IRQ number %d is invalid", priv->irq);
		return HAL_ERROR;
	}

	obj->set = aspeed_gpio_set;
	obj->get = aspeed_gpio_get;
	obj->get_direction = aspeed_gpio_get_direction;
	obj->set_direction = aspeed_gpio_set_direction;
	obj->int_cb_hook = aspeed_gpio_int_cb_hook;
	obj->int_cb_unhook = aspeed_gpio_int_cb_unhook;
	obj->cmd_src_set = aspeed_gpio_cmd_src_set;
	obj->info = aspeed_gpio_info;

	gpio->init = 1;
	return HAL_OK;
}