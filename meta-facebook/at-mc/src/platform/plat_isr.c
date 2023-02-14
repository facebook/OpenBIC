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
#include "libutil.h"
#include "plat_i2c.h"
#include "plat_fru.h"
#include "plat_isr.h"
#include "plat_dev.h"
#include "plat_gpio.h"
#include "power_status.h"
#include "ioexp_tca9555.h"
#include "common_i2c_mux.h"
#include "plat_sensor_table.h"
#include "plat_mctp.h"
#include "plat_class.h"
#include "hal_gpio.h"
#include "util_worker.h"

typedef struct _cxl_work_info {
	bool is_init;
	uint8_t cxl_card_id;
	uint8_t cxl_channel;
	struct k_work_delayable set_eid_work;
} cxl_work_info;

cxl_work_info cxl_work_item[] = {
	{ .is_init = false, .cxl_card_id = CXL_CARD_1, .cxl_channel = PCA9848_CHANNEL_0 },
	{ .is_init = false, .cxl_card_id = CXL_CARD_2, .cxl_channel = PCA9848_CHANNEL_1 },
	{ .is_init = false, .cxl_card_id = CXL_CARD_3, .cxl_channel = PCA9848_CHANNEL_2 },
	{ .is_init = false, .cxl_card_id = CXL_CARD_4, .cxl_channel = PCA9848_CHANNEL_3 },
	{ .is_init = false, .cxl_card_id = CXL_CARD_5, .cxl_channel = PCA9848_CHANNEL_4 },
	{ .is_init = false, .cxl_card_id = CXL_CARD_6, .cxl_channel = PCA9848_CHANNEL_5 },
	{ .is_init = false, .cxl_card_id = CXL_CARD_7, .cxl_channel = PCA9848_CHANNEL_6 },
	{ .is_init = false, .cxl_card_id = CXL_CARD_8, .cxl_channel = PCA9848_CHANNEL_7 },
};

LOG_MODULE_REGISTER(plat_isr);

void cxl_set_eid_work_handler(struct k_work *work_item)
{
	bool ret = false;

	struct k_work_delayable *dwork = k_work_delayable_from_work(work_item);
	cxl_work_info *work_info = CONTAINER_OF(dwork, cxl_work_info, set_eid_work);

	/** MEB mux for cxl channels **/
	mux_config meb_mux = { 0 };
	meb_mux.bus = MEB_CXL_BUS;
	meb_mux.target_addr = CXL_FRU_MUX0_ADDR;
	meb_mux.channel = work_info->cxl_channel;

	/** CXL mux for sensor channels **/
	mux_config cxl_mux = { 0 };
	cxl_mux.bus = MEB_CXL_BUS;
	cxl_mux.target_addr = CXL_FRU_MUX1_ADDR;
	cxl_mux.channel = CXL_CONTROLLER_MUX_CHANNEL;

	struct k_mutex *meb_mutex = get_i2c_mux_mutex(meb_mux.bus);

	/** Mutex lock bus **/
	if (k_mutex_lock(meb_mutex, K_MSEC(MUTEX_LOCK_INTERVAL_MS))) {
		LOG_ERR("mutex locked failed bus%u", meb_mux.bus);
		return;
	}

	/** Enable mux channel **/
	ret = set_mux_channel(meb_mux);
	if (ret == false) {
		k_mutex_unlock(meb_mutex);
		return;
	}

	ret = set_mux_channel(cxl_mux);
	if (ret == false) {
		k_mutex_unlock(meb_mutex);
		return;
	}

	/** Set endpoint id **/
	set_cxl_endpoint(MCTP_EID_CXL, work_info->cxl_card_id);

	/** mutex unlock bus **/
	k_mutex_unlock(meb_mutex);
}

void init_cxl_set_eid_work()
{
	uint8_t index = 0;
	for (index = 0; index < ARRAY_SIZE(cxl_work_item); ++index) {
		if (cxl_work_item[index].is_init != true) {
			k_work_init_delayable(&(cxl_work_item[index].set_eid_work),
					      cxl_set_eid_work_handler);
			cxl_work_item[index].is_init = true;
		}
	}
}

int cxl_device_reset()
{
	int ret = 0;
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	/** Read cxl U15 ioexp output port status **/
	msg.bus = MEB_CXL_BUS;
	msg.target_addr = CXL_IOEXP_U15_ADDR;
	msg.rx_len = 1;
	msg.tx_len = 1;
	msg.data[0] = TCA9555_OUTPUT_PORT_REG_0;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Unable to read ioexp bus: %u addr: 0x%02x", msg.bus, msg.target_addr);
		return -1;
	}

	/** Button press high **/
	uint8_t button_press_high = msg.data[0] | CXL_IOEXP_DEV_RESET_BIT;
	uint8_t button_press_low = msg.data[0] & (~CXL_IOEXP_DEV_RESET_BIT);

	memset(&msg, 0, sizeof(I2C_MSG));
	msg.bus = MEB_CXL_BUS;
	msg.target_addr = CXL_IOEXP_U15_ADDR;
	msg.tx_len = 2;
	msg.data[0] = TCA9555_OUTPUT_PORT_REG_0;
	msg.data[1] = button_press_high;

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Unable to write ioexp to press button high bus: %u addr: 0x%02x", msg.bus,
			msg.target_addr);
		return -1;
	}

	/** Button press low **/
	memset(&msg, 0, sizeof(I2C_MSG));
	msg.bus = MEB_CXL_BUS;
	msg.target_addr = CXL_IOEXP_U15_ADDR;
	msg.tx_len = 2;
	msg.data[0] = TCA9555_OUTPUT_PORT_REG_0;
	msg.data[1] = button_press_low;

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Unable to write ioexp to press button low bus: %u addr: 0x%02x", msg.bus,
			msg.target_addr);
		return -1;
	}

	k_msleep(CXL_IOEXP_BUTTON_PRESS_DELAY_MS);

	/** Button press high **/
	memset(&msg, 0, sizeof(I2C_MSG));
	msg.bus = MEB_CXL_BUS;
	msg.target_addr = CXL_IOEXP_U15_ADDR;
	msg.tx_len = 2;
	msg.data[0] = TCA9555_OUTPUT_PORT_REG_0;
	msg.data[1] = button_press_high;

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Unable to write ioexp to press button high  bus: %u addr: 0x%02x", msg.bus,
			msg.target_addr);
		return -1;
	}

	return 0;
}

int cxl_pe_reset_control(uint8_t cxl_card_id)
{
	int ret = 0;
	bool is_mb_reset_pin_change = false;
	uint8_t retry = 5;
	uint8_t u15_output_status = 0;
	I2C_MSG msg = { 0 };

	static uint8_t mb_reset_status = 0;

	/** Read cxl U15 ioexp input port0 status **/
	msg.bus = MEB_CXL_BUS;
	msg.target_addr = CXL_IOEXP_U15_ADDR;
	msg.rx_len = 1;
	msg.tx_len = 1;
	msg.data[0] = TCA9555_INPUT_PORT_REG_0;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Unable to read ioexp bus: %u addr: 0x%02x", msg.bus, msg.target_addr);
		return -1;
	}

	if (mb_reset_status != (msg.data[0] & CXL_IOEXP_MB_RESET_BIT)) {
		is_mb_reset_pin_change = true;
	}

	mb_reset_status = msg.data[0] & CXL_IOEXP_MB_RESET_BIT;

	/** Read cxl U15 ioexp input port1 status **/
	memset(&msg, 0, sizeof(I2C_MSG));
	msg.bus = MEB_CXL_BUS;
	msg.target_addr = CXL_IOEXP_U15_ADDR;
	msg.rx_len = 1;
	msg.tx_len = 1;
	msg.data[0] = TCA9555_INPUT_PORT_REG_1;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Unable to read ioexp bus: %u addr: 0x%02x", msg.bus, msg.target_addr);
		return -1;
	}

	/** Read cxl U15 ioexp output port status **/
	memset(&msg, 0, sizeof(I2C_MSG));
	msg.bus = MEB_CXL_BUS;
	msg.target_addr = CXL_IOEXP_U15_ADDR;
	msg.rx_len = 1;
	msg.tx_len = 1;
	msg.data[0] = TCA9555_OUTPUT_PORT_REG_0;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Unable to read ioexp bus: %u addr: 0x%02x", msg.bus, msg.target_addr);
		return -1;
	}

	u15_output_status = msg.data[0];

	/** Set asic pe-reset status to MB reset status **/
	memset(&msg, 0, sizeof(I2C_MSG));
	msg.bus = MEB_CXL_BUS;
	msg.target_addr = CXL_IOEXP_U15_ADDR;
	msg.tx_len = 2;
	msg.data[0] = TCA9555_OUTPUT_PORT_REG_0;
	if (mb_reset_status) {
		msg.data[1] = u15_output_status | CXL_IOEXP_ASIC_PERESET_BIT;

		if (is_mb_reset_pin_change == true) {
			k_work_schedule_for_queue(&plat_work_q,
						  &cxl_work_item[cxl_card_id].set_eid_work,
						  K_MSEC(CXL_DRIVE_READY_DELAY_MS));
		}
	} else {
		msg.data[1] = u15_output_status & (~CXL_IOEXP_ASIC_PERESET_BIT);

		if (is_mb_reset_pin_change == true) {
			k_work_cancel_delayable(&cxl_work_item[cxl_card_id].set_eid_work);
		}
	}

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Unable to write ioexp bus: %u addr: 0x%02x", msg.bus, msg.target_addr);
		return -1;
	}

	return 0;
}

void check_ioexp_status(uint8_t cxl_card_id)
{
	int ret = 0;
	uint8_t retry = 5;
	uint8_t u17_input_port0_status = 0;
	I2C_MSG msg = { 0 };

	/** Read cxl U17 ioexp input port0 status **/
	msg.bus = MEB_CXL_BUS;
	msg.target_addr = CXL_IOEXP_U17_ADDR;
	msg.rx_len = 1;
	msg.tx_len = 1;
	msg.data[0] = TCA9555_INPUT_PORT_REG_0;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Unable to read ioexp bus: %u addr: 0x%02x", msg.bus, msg.target_addr);
		return;
	}

	u17_input_port0_status = msg.data[0];

	/** Read cxl U17 ioexp input port1 status **/
	memset(&msg, 0, sizeof(I2C_MSG));
	msg.bus = MEB_CXL_BUS;
	msg.target_addr = CXL_IOEXP_U17_ADDR;
	msg.rx_len = 1;
	msg.tx_len = 1;
	msg.data[0] = TCA9555_INPUT_PORT_REG_1;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Unable to read ioexp bus: %u addr: 0x%02x", msg.bus, msg.target_addr);
		return;
	}

	/** Check CXL controller power good **/
	if ((u17_input_port0_status & CXL_IOEXP_CONTROLLER_PWRGD_VAL) ==
	    CXL_IOEXP_CONTROLLER_PWRGD_VAL) {
		ret = cxl_device_reset();
		if (ret != 0) {
			LOG_ERR("CXL device reset fail");
		}
	}

	ret = cxl_pe_reset_control(cxl_card_id);
	if (ret != 0) {
		LOG_ERR("CXL pr-reset control fail");
	}
}

void cxl_ioexp_alert(cxl_work_info cxl_info)
{
	bool ret = false;

	/** MEB mux for cxl channels **/
	mux_config meb_mux = { 0 };
	meb_mux.bus = MEB_CXL_BUS;
	meb_mux.target_addr = CXL_FRU_MUX0_ADDR;
	meb_mux.channel = cxl_info.cxl_channel;

	/** CXL mux for sensor channels **/
	mux_config cxl_mux = { 0 };
	cxl_mux.bus = MEB_CXL_BUS;
	cxl_mux.target_addr = CXL_FRU_MUX1_ADDR;
	cxl_mux.channel = CXL_IOEXP_MUX_CHANNEL;

	struct k_mutex *meb_mutex = get_i2c_mux_mutex(meb_mux.bus);

	/** Mutex lock bus **/
	if (k_mutex_lock(meb_mutex, K_MSEC(MUTEX_LOCK_INTERVAL_MS))) {
		LOG_ERR("mutex locked failed bus%u", meb_mux.bus);
		return;
	}

	/** Enable mux channel **/
	ret = set_mux_channel(meb_mux);
	if (ret == false) {
		k_mutex_unlock(meb_mutex);
		return;
	}

	ret = set_mux_channel(cxl_mux);
	if (ret == false) {
		k_mutex_unlock(meb_mutex);
		return;
	}

	/** Check io expander **/
	check_ioexp_status(cxl_info.cxl_card_id);

	/** Initial ioexp U14 and U16 **/
	cxl_single_ioexp_init(IOEXP_U14);
	cxl_single_ioexp_init(IOEXP_U16);

	/** mutex unlock bus **/
	k_mutex_unlock(meb_mutex);
}

void ISR_NORMAL_PWRGD()
{
	set_DC_status(MEB_NORMAL_PWRGD_BIC);

	if (gpio_get(MEB_NORMAL_PWRGD_BIC) == HIGH_INACTIVE) {
		uint8_t index = 0;
		for (index = 0; index < CXL_CARD_8; ++index) {
			set_cxl_eid_flag(index, CLEAR_EID_FLAG);
		}
	}
}

void ISR_CXL_IOEXP_ALERT0()
{
	cxl_ioexp_alert(cxl_work_item[CXL_CARD_1]);
}

void ISR_CXL_IOEXP_ALERT1()
{
	cxl_ioexp_alert(cxl_work_item[CXL_CARD_2]);
}

void ISR_CXL_IOEXP_ALERT2()
{
	cxl_ioexp_alert(cxl_work_item[CXL_CARD_3]);
}

void ISR_CXL_IOEXP_ALERT3()
{
	cxl_ioexp_alert(cxl_work_item[CXL_CARD_4]);
}

void ISR_CXL_IOEXP_ALERT4()
{
	cxl_ioexp_alert(cxl_work_item[CXL_CARD_5]);
}

void ISR_CXL_IOEXP_ALERT5()
{
	cxl_ioexp_alert(cxl_work_item[CXL_CARD_6]);
}

void ISR_CXL_IOEXP_ALERT6()
{
	cxl_ioexp_alert(cxl_work_item[CXL_CARD_7]);
}

void ISR_CXL_IOEXP_ALERT7()
{
	cxl_ioexp_alert(cxl_work_item[CXL_CARD_8]);
}
