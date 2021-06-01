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
#include "cmsis_os.h"
#include <string.h>
#include "sgpiom_aspeed.h"
#include "log.h"
#include "irq_aspeed.h"

uint16_t sgpiom_offset_gather[] = {
	[0] = offsetof(sgpiom_register_t, gather[0]),
	[1] = offsetof(sgpiom_register_t, gather[1]),
	[2] = offsetof(sgpiom_register_t, gather[2]),
	[3] = offsetof(sgpiom_register_t, gather4),
};

static void aspeed_sgpiom_isr_handle(aspeed_device_t *sgpiom)
{
    aspeed_sgpiom_priv_t *priv = (aspeed_sgpiom_priv_t *)sgpiom->private;
	aspeed_sgpiom_adapter_t *gpio_adapter;
	uint32_t group_num = DIV_ROUND_UP(priv->gpio_num, 32);
	uint32_t group_idx;
	uint32_t gpio_number;
	uint32_t int_index, int_pendding;
	sgpiom_gather_register_t *gather_reg;
	sgpiom_int_status_register_t *int_reg;
	for (group_idx = 0; group_idx < group_num; group_idx++) {
		gather_reg =
				(sgpiom_gather_register_t *)(sgpiom->base +
											 sgpiom_offset_gather[group_idx]);
		int_reg = &gather_reg->int_status;
		int_pendding = int_reg->value;
		int_index = 0;
		log_debug("group%d interrupt pendding = 0x%08x\n", group_idx,
				  int_pendding);
		while (int_pendding) {
			if (int_pendding & 0x1){
				gpio_number = (group_idx << 3) + int_index;
				gpio_adapter = &priv->adapter[gpio_number];
				if(gpio_adapter->cb != NULL)
				{
					gpio_adapter->cb(gpio_adapter->para, gpio_number);
				}
				int_reg->value = BIT(int_index);
			}
			int_index++;
			int_pendding >>= 1;
		}
	}
}

static void aspeed_sgpiom_isr(void)
{
	uint32_t n_irq = aspeed_irq_get_current_irq();
	aspeed_device_t *sgpiom = aspeed_irq_get_isr_context(n_irq);
	aspeed_sgpiom_isr_handle(sgpiom);
}

hal_status_t aspeed_sgpiom_int_cb_hook(gpio_t *obj, int gpio_number, uint8_t int_type, gpio_cb_t cb, void *para)
{
    aspeed_device_t *sgpiom = obj->device;
    aspeed_sgpiom_priv_t *priv = (aspeed_sgpiom_priv_t *)sgpiom->private;
	sgpiom_gather_register_t *gather_reg;
	aspeed_sgpiom_adapter_t *gpio_adapter;
	uint32_t group_idx = gpio_number >> 5;
	uint32_t bit = BIT(gpio_number & 0x1f);

    if (gpio_number >= priv->gpio_num) {
		log_error("invalie sgpiom #%d \n", gpio_number);
		return HAL_ERROR;
	}
	gpio_adapter = &priv->adapter[gpio_number];
    gpio_adapter->cb = cb;
	gpio_adapter->para = para;
	gpio_adapter->int_type = int_type;
	gather_reg = (sgpiom_gather_register_t *)(sgpiom->base +
											  sgpiom_offset_gather[group_idx]);
	gather_reg->int_en.value |= bit;
	gather_reg->int_sens_type[0].value &= ~(bit);
	gather_reg->int_sens_type[1].value &= ~(bit);
	gather_reg->int_sens_type[2].value &= ~(bit);
	gather_reg->int_sens_type[0].value |= (int_type & 0x1) ? bit : 0;
	gather_reg->int_sens_type[1].value |= (int_type & 0x2) ? bit : 0;
	gather_reg->int_sens_type[2].value |= (int_type & 0x4) ? bit : 0;
	return HAL_OK;
}

hal_status_t aspeed_sgpiom_int_cb_unhook(gpio_t *obj, int gpio_number)
{
    aspeed_device_t *sgpiom = obj->device;
    aspeed_sgpiom_priv_t *priv = (aspeed_sgpiom_priv_t *)sgpiom->private;
	aspeed_sgpiom_adapter_t *gpio_adapter;
	sgpiom_gather_register_t *gather_reg;
	uint32_t group_idx = gpio_number >> 5;
	uint32_t bit = BIT(gpio_number & 0x1f);

    if (gpio_number >= priv->gpio_num) {
		log_error("invalie sgpiom #%d \n", gpio_number);
		return HAL_ERROR;
	}
	gpio_adapter = &priv->adapter[gpio_number];
    gpio_adapter->cb = NULL;
	gpio_adapter->para = NULL;
    gather_reg = (sgpiom_gather_register_t *)(sgpiom->base +
											  sgpiom_offset_gather[group_idx]);
	gather_reg->int_en.value &= ~(bit);
	return HAL_OK;
}

int aspeed_sgpiom_get(gpio_t *obj, int gpio_number)
{
	aspeed_device_t *sgpiom = obj->device;
    aspeed_sgpiom_priv_t *priv = (aspeed_sgpiom_priv_t *)sgpiom->private;
	sgpiom_gather_register_t *gather_reg;
	uint32_t group_idx = gpio_number >> 5;
	uint32_t byte_idx = (gpio_number & 0x1f) >> 3;
	uint32_t bit_idx = (gpio_number & 0x7);

	if (gpio_number >= priv->gpio_num) {
		log_error("invalie sgpiom #%d \n", gpio_number);
		return -HAL_ERROR;
	}
	uint8_t ret_value;
	log_debug("sgpiom %d at group%d, set%d, bit%d\n", gpio_number, group_idx,
			  byte_idx, bit_idx);
	gather_reg = (sgpiom_gather_register_t *)(sgpiom->base +
											  sgpiom_offset_gather[group_idx]);
	ret_value = (gather_reg->data.fields.data[byte_idx] & BIT(bit_idx)) >> bit_idx;
	return ret_value;
}

hal_status_t aspeed_sgpiom_set(gpio_t *obj, int gpio_number, int val)
{
	aspeed_device_t *sgpiom = obj->device;
    aspeed_sgpiom_priv_t *priv = (aspeed_sgpiom_priv_t *)sgpiom->private;
	sgpiom_register_t *sgpiom_reg = (sgpiom_register_t *)sgpiom->base;
    sgpiom_gather_register_t *gather_reg;
	uint32_t temp_data;
	uint32_t group_idx = gpio_number >> 5;
	uint32_t bit = BIT(gpio_number & 0x1f);
	if (gpio_number >= priv->gpio_num) {
		log_error("invalie sgpiom #%d \n", gpio_number);
		return HAL_ERROR;
	}
	gather_reg = (sgpiom_gather_register_t *)(sgpiom->base +
											  sgpiom_offset_gather[group_idx]);
	temp_data = sgpiom_reg->rd_data[group_idx].value;
	if (val)
		temp_data |= bit;
	else
		temp_data &= ~(bit);
	gather_reg->data.value = temp_data;
	return HAL_OK;
}

hal_status_t aspeed_sgpiom_set_config(gpio_t *obj, int gpio_number, int config)
{
	/*TODO: Implement sgpiom configure: debounce time, reset tolerant */
	return HAL_OK;
}

hal_status_t aspeed_sgpiom_info(gpio_t *obj)
{
	aspeed_device_t *sgpiom = obj->device;
    aspeed_sgpiom_priv_t *priv = (aspeed_sgpiom_priv_t *)sgpiom->private;
	printf("type:sgpiom@%08x\n", sgpiom->base);
	printf("number:%d\n", priv->gpio_num);
	return HAL_OK;
}

uint16_t aspeed_sgpiom_get_clk_div(gpio_t *obj)
{
	aspeed_device_t *sgpiom = obj->device;
    aspeed_sgpiom_priv_t *priv = (aspeed_sgpiom_priv_t *)sgpiom->private;
#ifdef CONFIG_AST2600_SERIES
	uint32_t src_clk = aspeed_clk_get_apb2();
#else
	uint32_t src_clk = aspeed_clk_get_pclk();
#endif
	uint32_t target_clk = priv->target_clock;
	uint16_t div;
	div = ((src_clk >> 1) / target_clk) - 1;
	return div;
}

hal_status_t aspeed_sgpiom_set_direction(gpio_t *obj, int gpio_number, int direct)
{
	/* 
	 * SGPIO doesn't have the concept of direction. Use dummy function to bypass.
	 */
	return HAL_OK;
}

hal_status_t aspeed_sgpiom_init(gpio_t *obj)
{
	aspeed_device_t *sgpiom = obj->device;
    aspeed_sgpiom_priv_t *priv = (aspeed_sgpiom_priv_t *)sgpiom->private;
	sgpiom_register_t *sgpiom_reg = (sgpiom_register_t *)sgpiom->base;
	uint32_t gpio_idx;
	hal_status_t ret;

	if (sgpiom->init) {
	    log_error("sgpiom@%08x already occupied\n", sgpiom->base);
	    return HAL_BUSY;
	}
	priv->adapter = malloc(priv->gpio_num * sizeof(aspeed_sgpiom_adapter_t));
    if (priv->adapter == NULL) {
        log_error("sgpiom adapter malloc %d byte error\n", priv->gpio_num * sizeof(aspeed_sgpiom_adapter_t));
	    return HAL_ERROR;
    }
	memset(priv->adapter, 0, priv->gpio_num * sizeof(aspeed_sgpiom_adapter_t));

	for (gpio_idx = 0; gpio_idx < priv->gpio_num; gpio_idx++) {
		ret = aspeed_sgpiom_int_cb_unhook(obj, gpio_idx);
        if (ret != HAL_OK) {
			log_error("sgpiom%d cb unhook error\n", gpio_idx);
			return HAL_ERROR;
		}
		priv->adapter[gpio_idx].evt_id = osEventFlagsNew(NULL);
		if (priv->adapter[gpio_idx].evt_id == NULL)
            log_error("fail to create evt_id\n");
	}
	
	if (priv->irq)
		aspeed_irq_register(priv->irq, (uint32_t)aspeed_sgpiom_isr, (void *)sgpiom);
	else {
		log_error("IRQ number %d is invalid", priv->irq);
		return HAL_ERROR;
	}
	sgpiom_reg->config.fields.enable = 1;
	sgpiom_reg->config.fields.numbers = DIV_ROUND_UP(priv->gpio_num, 8);
	sgpiom_reg->config.fields.division = aspeed_sgpiom_get_clk_div(obj);

	obj->set = aspeed_sgpiom_set;
	obj->get = aspeed_sgpiom_get;
	obj->set_direction = aspeed_sgpiom_set_direction;
	obj->int_cb_hook = aspeed_sgpiom_int_cb_hook;
	obj->int_cb_unhook = aspeed_sgpiom_int_cb_unhook;
	obj->info = aspeed_sgpiom_info;

	sgpiom->init = 1;
	return HAL_OK;
}