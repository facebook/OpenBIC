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
 *
 * Real GPIO status-monitoring subsystem, ported to mainline Zephyr's
 * gpio API from scratch (not common/hal/hal_gpio.c - see the top-level
 * README for why that file isn't buildable against upstream Zephyr).
 *
 * This EVK has no on-board sensor to read, so SW2 (the "mon0" alias)
 * stands in for what would be a real platform status input on an
 * actual BIC target (power-good, host presence, etc.) - same GPIO
 * edge-interrupt + debounce pattern, real hardware, just a stand-in
 * signal source.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "plat_gpio.h"

LOG_MODULE_REGISTER(plat_gpio, LOG_LEVEL_INF);

#define DEBOUNCE_MS 30

static const struct gpio_dt_spec mon0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback mon0_cb_data;
static struct k_work_delayable mon0_debounce_work;
static atomic_t mon0_state;

/* Board hook, __weak so subsystems that care about SW2 transitions
 * (e.g. plat_pldm_monitor.c, which turns them into PLDM state-sensor
 * events) can observe them without this file depending on them.
 * Runs in the system workqueue, after debounce. */
__weak void plat_gpio_mon0_changed(int level)
{
	ARG_UNUSED(level);
}

static void mon0_debounce_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	int level = gpio_pin_get_dt(&mon0);

	if (level < 0) {
		LOG_ERR("mon0 read failed: %d", level);
		return;
	}

	if (atomic_set(&mon0_state, level) != level) {
		LOG_INF("mon0 (SW2) state changed: %s", level ? "asserted" : "deasserted");
		plat_gpio_mon0_changed(level);
	}
}

static void mon0_isr(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	k_work_reschedule(&mon0_debounce_work, K_MSEC(DEBOUNCE_MS));
}

void plat_gpio_init(void)
{
	int ret;

	if (!gpio_is_ready_dt(&mon0)) {
		LOG_ERR("mon0 device not ready");
		return;
	}

	ret = gpio_pin_configure_dt(&mon0, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("mon0 configure failed: %d", ret);
		return;
	}

	ret = gpio_pin_interrupt_configure_dt(&mon0, GPIO_INT_EDGE_BOTH);
	if (ret < 0) {
		LOG_ERR("mon0 interrupt configure failed: %d", ret);
		return;
	}

	k_work_init_delayable(&mon0_debounce_work, mon0_debounce_handler);
	gpio_init_callback(&mon0_cb_data, mon0_isr, BIT(mon0.pin));
	gpio_add_callback(mon0.port, &mon0_cb_data);

	atomic_set(&mon0_state, gpio_pin_get_dt(&mon0));
	LOG_INF("mon0 (SW2) monitoring started, initial state: %s",
		atomic_get(&mon0_state) ? "asserted" : "deasserted");
}

int plat_gpio_mon0_get(void)
{
	return atomic_get(&mon0_state);
}
