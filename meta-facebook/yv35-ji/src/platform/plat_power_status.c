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

#include <stdlib.h>
#include "hal_i2c.h"
#include "plat_i2c.h"
#include "plat_gpio.h"
#include "plat_class.h"
#include "pmbus.h"
#include "plat_mctp.h"
#include "plat_sensor_table.h"
#include "power_status.h"
#include "plat_power_status.h"
#include "logging/log.h"

LOG_MODULE_REGISTER(plat_power_status);

K_MUTEX_DEFINE(post_status_mutex);

static bool is_post_end_work_done = false;
void reset_post_end_work_status()
{
	is_post_end_work_done = false;
}

struct rt8848c_cfg_tune {
	uint8_t reg;
	uint8_t val;
};

struct rt8848c_cfg_tune rt8848c_cfg_tune[] = {
	{ 0x00, 0x00 }, // unlock register 0x01~0x5F
	{ 0x54, 0x6B }, // CCM Ramp Hold Voltage for Jitter improvement (original:0x67)
	{ 0x4F, 0x19 }, // Internal Ramp Magnitude for jitter improvement , 700mV (original:0x13)
	{ 0x4D, 0x03 }, // [1:0]=11 , enable per-phase OCP function and digital flag detection
	{ 0x11, 0x1F }, // Current balance gain 32/16 for phase1
	{ 0x13, 0x00 }, // Current balance gain 1/16 for phase2
	{ 0x15, 0x1F }, // Current balance gain 32/16 for phase3
	{ 0x17, 0x00 }, // Current balance gain 1/16 for phase4
	{ 0x19, 0x1F }, // Current balance gain 32/16 for phase5
	{ 0x1B, 0x00 }, // Current balance gain 1/16 for phase6
	{ 0x1D, 0x1F }, // Current balance gain 32/16 for phase7
	{ 0x1F, 0x00 }, // Current balance gain 1/16 for phase8
	{ 0x12, 0x12 }, // Current balance offset -45mV for phase1
	{ 0x14, 0x00 }, // Current balance offset 15mV for phase2
	{ 0x16, 0x01 }, // Current balance offset 30mV for phase3
	{ 0x18, 0x00 }, // Current balance offset 15mV for phase4
	{ 0x1A, 0x12 }, // Current balance offset -45mV for phase5
	{ 0x1C, 0x01 }, // Current balance offset 30mV for phase6
	{ 0x1E, 0x00 }, // Current balance offset 15mV for phase7
	{ 0x20, 0x01 }, // Current balance offset 30mV for phase8
	{ 0x00, 0xA6 }, // lock register 0x01~0x5F
};

static void set_rt8848c_config()
{
	/* switch VR bus to BIC */
	gpio_set(BIC_CPLD_VRD_MUX_SEL, GPIO_LOW);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;
	i2c_msg.bus = CPUVDD_I2C_BUS;
	i2c_msg.target_addr = (RT8848C_I2C_ADDR >> 1);

	for (int i = 0; i < ARRAY_SIZE(rt8848c_cfg_tune); i++) {
		i2c_msg.tx_len = 2;
		i2c_msg.data[0] = rt8848c_cfg_tune[i].reg;
		i2c_msg.data[1] = rt8848c_cfg_tune[i].val;
		if (i2c_master_write(&i2c_msg, retry)) {
			LOG_ERR("Failed to config RT8848C while writing register 0x%02X to 0x%02X",
				rt8848c_cfg_tune[i].reg, rt8848c_cfg_tune[i].val);
			break;
		}
	}

	/* switch VR bus to CPU */
	gpio_set(BIC_CPLD_VRD_MUX_SEL, GPIO_HIGH);

	LOG_WRN("RT8848C configuration done!");
}

K_TIMER_DEFINE(send_cmd_timer, send_cmd_to_dev, NULL);
void handle_post_work(struct k_work *work)
{
	if (k_mutex_lock(&post_status_mutex, K_MSEC(1000))) {
		LOG_WRN("post status mutex lock failed");
		return;
	}

	if ((get_board_revision() >= SYS_BOARD_PVT) && is_post_end_work_done)
		goto exit;

	k_timer_start(&send_cmd_timer, K_NO_WAIT, K_NO_WAIT);

	/* Optimize RT8848C */
	if (get_oth_module() == OTH_MODULE_SECOND)
		set_rt8848c_config();

	/* work around */
	if (get_board_revision() < SYS_BOARD_EVT)
		goto exit;

	if (modify_sensor_cfg() == false) {
		LOG_ERR("Failed to modify sensor cfg!");
	}

	is_post_end_work_done = true;
exit:
	if (k_mutex_unlock(&post_status_mutex)) {
		LOG_ERR("post status mutex unlock failed");
	}
}
K_WORK_DEFINE(handle_post_end_work, handle_post_work);

void handle_post_action()
{
	k_work_submit(&handle_post_end_work);
}

void handle_post_status(bool status, bool need_change)
{
	if (get_board_revision() >= SYS_BOARD_PVT) {
		set_post_status(FPGA_CPU_BOOT_DONE_L);
	} else {
		if (need_change == true)
			gpio_set(VIRTUAL_BIOS_POST_COMPLETE_L, status);

		set_post_status(VIRTUAL_BIOS_POST_COMPLETE_L);
	}
}

/* work around for INF vr power-on */
void handle_tda38741_work_around()
{
	if ((get_oth_module() == OTH_MODULE_SECOND) && (get_DC_status() == false)) {
		/* switch VR bus to BIC */
		gpio_set(BIC_CPLD_VRD_MUX_SEL, GPIO_LOW);

		uint8_t retry = 8;
		I2C_MSG msg = { 0 };

		msg.bus = CPUDVDD_I2C_BUS;
		msg.target_addr = CPUDVDD_I2C_ADDR >> 1;
		msg.tx_len = 1;
		msg.rx_len = 3;
		msg.data[0] = PMBUS_MFR_ID;

		if (i2c_master_read(&msg, retry))
			LOG_WRN("TDA38741 patting failed!");
		else
			LOG_INF("TDA38741 patting done!");

		/* switch VR bus to CPU */
		gpio_set(BIC_CPLD_VRD_MUX_SEL, GPIO_HIGH);
	}
}

#define CPLD_REG_E1S_PRSNT_STATE 0x12
#define CPLD_REG_RETIMER_PRSNT_STATE 0x13
#define PWR_STAT_MON_THREAD_STACK_SIZE 1024

struct k_thread power_status_monitor_thread;
K_KERNEL_STACK_MEMBER(power_status_monitor_stack, PWR_STAT_MON_THREAD_STACK_SIZE);

void power_status_monitor_handler(void *arug0, void *arug1, void *arug2)
{
	static uint8_t last_e1s_prsnt_state = 0xFF;
	static uint8_t last_retimer_prsnt_state = 0xFF;

	while (1) {
		uint8_t retry = 3;
		I2C_MSG msg = { 0 };

		msg.bus = I2C_BUS1;
		msg.target_addr = CPLD_I2C_ADDR >> 1;
		msg.tx_len = 1;
		msg.rx_len = 1;
		msg.data[0] = CPLD_REG_E1S_PRSNT_STATE;

		if (i2c_master_read(&msg, retry) == 0) {
			uint8_t e1s_prsnt_state = msg.data[0] & 0x01;

			if (last_e1s_prsnt_state != e1s_prsnt_state) {
				LOG_INF("E1S presence state: %d", e1s_prsnt_state);
				gpio_set(VIRTUAL_E1S_PRSNT_L, e1s_prsnt_state);
				last_e1s_prsnt_state = e1s_prsnt_state;
			}
		}

		msg.tx_len = 1;
		msg.rx_len = 1;
		msg.data[0] = CPLD_REG_RETIMER_PRSNT_STATE;

		if (i2c_master_read(&msg, retry) == 0) {
			uint8_t retimer_prsnt_state = msg.data[0] & 0x01;

			if (last_retimer_prsnt_state != retimer_prsnt_state) {
				LOG_INF("RETIMER presence state: %d", retimer_prsnt_state);
				gpio_set(VIRTUAL_RETIMER_PG, retimer_prsnt_state);
				last_retimer_prsnt_state = retimer_prsnt_state;
			}
		}

		k_sleep(K_MSEC(3000));
	}
}

void power_status_monitor()
{
	k_thread_create(&power_status_monitor_thread, power_status_monitor_stack,
			K_THREAD_STACK_SIZEOF(power_status_monitor_stack),
			power_status_monitor_handler, NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY,
			0, K_NO_WAIT);
	k_thread_name_set(&power_status_monitor_thread, "power_monitor_thread");
}
