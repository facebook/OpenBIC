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

static void set_rt8848c_config()
{
	/* switch VR bus to BIC */
	gpio_set(BIC_CPLD_VRD_MUX_SEL, GPIO_LOW);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;
	i2c_msg.bus = CPUVDD_I2C_BUS;
	i2c_msg.target_addr = (RT8848C_I2C_ADDR >> 1);

	/* unlock register 0x01~0x5F */
	i2c_msg.tx_len = 2;
	i2c_msg.data[0] = 0x00;
	i2c_msg.data[1] = 0x00;
	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Failed to unlock RT8848C register 0x01~0x5F");
		return;
	}

	/* CCM Ramp Hold Voltage for Jitter improvement (original:0x67) */
	i2c_msg.tx_len = 2;
	i2c_msg.data[0] = 0x54;
	i2c_msg.data[1] = 0x6B;
	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Failed to do jitter improvement on RT8848C register 0x%x", 0x54);
		return;
	}

	/* Internal Ramp Magnitude for jitter improvement , 700mV (original:0x13) */
	i2c_msg.tx_len = 2;
	i2c_msg.data[0] = 0x4F;
	i2c_msg.data[1] = 0x19;
	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Failed to do jitter improvement on RT8848C register 0x%x", 0x4F);
		return;
	}

	/* [1:0]=11 , enable per-phase OCP function and digital flag detection */
	i2c_msg.tx_len = 2;
	i2c_msg.data[0] = 0x4D;
	i2c_msg.data[1] = 0x03;
	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Failed to enable per-phase OCP function and digital flag detection on RT8848C");
		return;
	}

	/* lock register 0x01~0x5F */
	i2c_msg.tx_len = 2;
	i2c_msg.data[0] = 0x00;
	i2c_msg.data[1] = 0xA6;
	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Failed to unlock RT8848C register 0x01~0x5F");
		return;
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

	if (is_post_end_work_done)
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
