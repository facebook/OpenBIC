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

#include <stdint.h>
#include <logging/log.h>
#include "plat_ssif.h"
#include "plat_i2c.h"
#include "plat_gpio.h"
#include "plat_mctp.h"
#include "plat_sensor_table.h"
#include "plat_class.h"
#include "power_status.h"
#include "ssif.h"

LOG_MODULE_REGISTER(plat_ssif);

struct ssif_init_cfg ssif_cfg_table[] = {
	{ SSIF_I2C_BUS, SSIF_I2C_ADDR, 0x0A },
};

void pal_ssif_alert_trigger(uint8_t status)
{
	LOG_DBG("trigger %d", status);
	gpio_set(I2C_SSIF_ALERT_L, status);
}

void ssif_init(void)
{
	ssif_device_init(ssif_cfg_table, ARRAY_SIZE(ssif_cfg_table));

	if (ssif_inst_get_by_bus(SSIF_I2C_BUS))
		LOG_WRN("SSIF ready!");
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
void handle_post_end_handler(struct k_work *work)
{
	k_timer_start(&send_cmd_timer, K_NO_WAIT, K_NO_WAIT);

	/* Pull low virtual bios complete pin */
	gpio_set(VIRTUAL_BIOS_POST_COMPLETE_L, GPIO_LOW);
	set_post_status(VIRTUAL_BIOS_POST_COMPLETE_L);

	uint8_t vr_module = get_oth_module();

	/* Optimize RT8848C */
	if (vr_module == OTH_MODULE_SECOND)
		set_rt8848c_config();

	/* work around */
	if (get_board_revision() < SYS_BOARD_EVT)
		return;

	if (modify_sensor_cfg() == false) {
		LOG_ERR("Failed to modify sensor cfg!");
	}
}

K_WORK_DEFINE(handle_post_end_work, handle_post_end_handler);
void pal_bios_post_complete()
{
	k_work_submit(&handle_post_end_work);
}
