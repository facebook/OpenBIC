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
#include "hal_vw_gpio.h"
#include <logging/log.h>
#include <stdlib.h>
#include <string.h>
#include "libutil.h"

LOG_MODULE_REGISTER(hal_vw_gpio);

static const struct device *espi_dev;
static struct espi_callback vw_cb;
static vw_gpio *vw_gpio_cfg;
static uint8_t vw_gpio_size = 0;

bool vw_gpio_get(int number, uint8_t *value)
{
	CHECK_NULL_ARG_WITH_RETURN(value, false);

#ifdef CONFIG_ESPI_ASPEED
	uint32_t reg_value = sys_read32(AST_ESPI_BASE + AST_ESPI_GPIO_DIR);
	value[0] = GETBIT(reg_value, number) ? VW_GPIO_OUTPUT : VW_GPIO_INPUT;
	reg_value = sys_read32(AST_ESPI_BASE + AST_ESPI_GPIO_VAL);
	value[1] = GETBIT(reg_value, number) ? VW_GPIO_HIGH : VW_GPIO_LOW;
	return true;
#else
	return false;
#endif
}

bool vw_gpio_set(int number, uint8_t value)
{
#ifdef CONFIG_ESPI_ASPEED
	// Check if the vw gpio is master input and bic output pin
	uint32_t reg_value = sys_read32(AST_ESPI_BASE + AST_ESPI_GPIO_DIR);
	if (GETBIT(reg_value, number))
		return false;

	uint8_t i;
	for (i = 0; i < vw_gpio_size; i++) {
		if ((vw_gpio_cfg[i].number == number) && (vw_gpio_cfg[i].is_enabled) &&
		    (vw_gpio_cfg[i].direction == VW_GPIO_OUTPUT)) {
			reg_value = sys_read32(AST_ESPI_BASE + AST_ESPI_GPIO_VAL);
			if (value == VW_GPIO_HIGH)
				reg_value = SETBIT(reg_value, number);
			else
				reg_value = CLEARBIT(reg_value, number);
			sys_write32(reg_value, AST_ESPI_BASE + AST_ESPI_GPIO_VAL);
			return true;
		}
	}
#endif
	return false;
}

#ifdef CONFIG_ESPI_ASPEED
static void ast_vw_gpio_scan(void)
{
	uint8_t i;
	uint32_t reg_dir = sys_read32(AST_ESPI_BASE + AST_ESPI_GPIO_DIR);
	uint32_t reg_val = sys_read32(AST_ESPI_BASE + AST_ESPI_GPIO_VAL);
	for (i = 0; i < vw_gpio_size; i++) {
		if (!vw_gpio_cfg[i].is_enabled)
			continue;

		if (vw_gpio_cfg[i].direction == VW_GPIO_INPUT) {
			/* Check if controller direction setting
			 * ESPI9C: GPIO direction of virtual wire channel
			 */
			uint8_t gpio_value;
			if (GETBIT(reg_dir, vw_gpio_cfg[i].number) != VW_GPIO_INPUT) {
				gpio_value = VW_GPIO_UNKNOWN;
				LOG_ERR("the vgpio setting at controller side is incorrect.");
				return;
			} else {
				gpio_value = GETBIT(reg_val, vw_gpio_cfg[i].number) ? VW_GPIO_HIGH :
										      VW_GPIO_LOW;
			}

			/* Get virtual gpio value
			 * ESPI09C: GPIO through virtual wire channel
			 */
			if (gpio_value != vw_gpio_cfg[i].value) {
				vw_gpio_cfg[i].value = gpio_value;
				if (vw_gpio_cfg[i].int_cb) {
					vw_gpio_cfg[i].int_cb(gpio_value);
				}
			}
		}
	}
}
#endif

/* Handler for virtual GPIO from eSPI virtual wire channel */
static void vw_handler(const struct device *dev, struct espi_callback *cb, struct espi_event event)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);

#ifdef CONFIG_ESPI_ASPEED
	if (event.evt_type == ESPI_BUS_EVENT_VWIRE_RECEIVED) {
		/* Check if the virtual wire GPIO interrupt is happened
		 * ESPI008: Interrupt Status
		 *   bit[9] - Virtual Wire GPIO Event
		 */
		if (!GETBIT(event.evt_data, 9))
			return;

		ast_vw_gpio_scan();
	}
#endif
}

void vw_gpio_reset()
{
	uint8_t i;
	for (i = 0; i < vw_gpio_size; i++) {
		if (vw_gpio_cfg[i].value != VW_GPIO_LOW) {
			vw_gpio_cfg[i].value = VW_GPIO_LOW;
			if (vw_gpio_cfg[i].int_cb) {
				vw_gpio_cfg[i].int_cb(VW_GPIO_LOW);
			}
		}
	}
}

bool vw_gpio_init(vw_gpio *config, uint8_t size)
{
	CHECK_NULL_ARG_WITH_RETURN(config, false);

	espi_dev = device_get_binding("ESPI");
	if (!espi_dev) {
		LOG_ERR("failed to get espi device");
		return false;
	}

	CHECK_ARG_WITH_RETURN(vw_gpio_cfg != NULL, false);
	vw_gpio_cfg = (vw_gpio *)malloc(size * sizeof(vw_gpio));
	if (!vw_gpio_cfg) {
		LOG_ERR("failed to allocate memory");
		return false;
	}
	memcpy(vw_gpio_cfg, config, size * sizeof(vw_gpio));
	vw_gpio_size = size;

	espi_init_callback(&vw_cb, vw_handler, ESPI_BUS_EVENT_VWIRE_RECEIVED);
	if (espi_add_callback(espi_dev, &vw_cb) < 0) {
		LOG_ERR("failed to add espi callback function");
		SAFE_FREE(vw_gpio_cfg);
		vw_gpio_size = 0;
		return false;
	}

	// Initialize vGPIO status during BIC bootup
#ifdef CONFIG_ESPI_ASPEED
	ast_vw_gpio_scan();
	return true;
#endif

	SAFE_FREE(vw_gpio_cfg);
	vw_gpio_size = 0;
	return false;
}
