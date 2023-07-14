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

#include <string.h>
#include <logging/log.h>
#include "plat_gpio.h"
#include "plat_power_status.h"
#include "libutil.h"

LOG_MODULE_REGISTER(plat_power_status);

#define BIC_HB_HIGH_INTERVAL_MSEC 500
#define BIC_HB_LOW_INTERVAL_MSEC 500

#define BIC_HB_THREAD_STACK_SIZE 256

struct k_thread bic_hb_thread;
K_KERNEL_STACK_MEMBER(bic_hb_thread_stack, BIC_HB_THREAD_STACK_SIZE);

struct k_thread mpro_hb_thread;
K_KERNEL_STACK_MEMBER(mpro_hb_thread_stack, BIC_HB_THREAD_STACK_SIZE);
static bool is_mpro_ready = false;
void set_mpro_status()
{
	is_mpro_ready = (gpio_get(S0_BMC_GPIOA5_FW_BOOT_OK) == 1) ? true : false;
	LOG_WRN("MPRO_STATUS: %s", (is_mpro_ready) ? "on" : "off");
}

bool get_mpro_status()
{
	return is_mpro_ready;
}

bool mpro_access(uint8_t sensor_num)
{
	return get_mpro_status();
}

void bic_hb_handler(void *arug0, void *arug1, void *arug2)
{
	ARG_UNUSED(arug0);
	ARG_UNUSED(arug1);
	ARG_UNUSED(arug2);

	while (1) {
		gpio_set(BMC_GPIOD4_SW_HBLED, GPIO_HIGH);
		k_msleep(BIC_HB_HIGH_INTERVAL_MSEC);
		gpio_set(BMC_GPIOD4_SW_HBLED, GPIO_LOW);
		k_msleep(BIC_HB_LOW_INTERVAL_MSEC);
	}
}

void bic_heart_beat_init()
{
	k_thread_create(&bic_hb_thread, bic_hb_thread_stack,
			K_THREAD_STACK_SIZEOF(bic_hb_thread_stack), bic_hb_handler, NULL, NULL,
			NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&bic_hb_thread, "bic_hb_thread");
}

uint8_t mpro_hb_cnt = 0;
void mpro_padding()
{
	if (mpro_hb_cnt == 64)
		mpro_hb_cnt = 0;
	mpro_hb_cnt++;
}

void mpro_hb_handler(void *arug0, void *arug1, void *arug2)
{
	ARG_UNUSED(arug0);
	ARG_UNUSED(arug1);
	ARG_UNUSED(arug2);

	static uint8_t rec_mpro_hb_cnt = 0;
	while (1) {
		k_msleep(5000);

		if (get_mpro_status() == false)
			continue;

		if (mpro_hb_cnt == rec_mpro_hb_cnt) {
			LOG_DBG("MPRO HANG UP!!");
			continue;
		}

		rec_mpro_hb_cnt = mpro_hb_cnt;
	}
}

void mpro_heart_beat_check_init()
{
	k_thread_create(&mpro_hb_thread, mpro_hb_thread_stack,
			K_THREAD_STACK_SIZEOF(mpro_hb_thread_stack), mpro_hb_handler, NULL, NULL,
			NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&mpro_hb_thread, "mpro_hb_thread");
}
