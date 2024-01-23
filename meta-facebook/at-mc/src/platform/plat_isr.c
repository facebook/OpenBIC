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
#include "plat_sys.h"
#include "libipmi.h"
#include "ipmi.h"
#include "plat_ipmi.h"
#include "plat_pldm_monitor.h"
#include "pldm_state_set.h"

bool is_interrupt_ongoing = false;

cxl_work_info cxl_work_item[] = {
	{ .is_init = false,
	  .cxl_card_id = CXL_CARD_1,
	  .cxl_channel = PCA9848_CHANNEL_6,
	  .is_device_reset = false,
	  .is_mb_reset = false,
	  .is_pe_reset = false },
	{ .is_init = false,
	  .cxl_card_id = CXL_CARD_2,
	  .cxl_channel = PCA9848_CHANNEL_7,
	  .is_device_reset = false,
	  .is_mb_reset = false,
	  .is_pe_reset = false },
	{ .is_init = false,
	  .cxl_card_id = CXL_CARD_3,
	  .cxl_channel = PCA9848_CHANNEL_4,
	  .is_device_reset = false,
	  .is_mb_reset = false,
	  .is_pe_reset = false },
	{ .is_init = false,
	  .cxl_card_id = CXL_CARD_4,
	  .cxl_channel = PCA9848_CHANNEL_5,
	  .is_device_reset = false,
	  .is_mb_reset = false,
	  .is_pe_reset = false },
	{ .is_init = false,
	  .cxl_card_id = CXL_CARD_5,
	  .cxl_channel = PCA9848_CHANNEL_3,
	  .is_device_reset = false,
	  .is_mb_reset = false,
	  .is_pe_reset = false },
	{ .is_init = false,
	  .cxl_card_id = CXL_CARD_6,
	  .cxl_channel = PCA9848_CHANNEL_2,
	  .is_device_reset = false,
	  .is_mb_reset = false,
	  .is_pe_reset = false },
	{ .is_init = false,
	  .cxl_card_id = CXL_CARD_7,
	  .cxl_channel = PCA9848_CHANNEL_1,
	  .is_device_reset = false,
	  .is_mb_reset = false,
	  .is_pe_reset = false },
	{ .is_init = false,
	  .cxl_card_id = CXL_CARD_8,
	  .cxl_channel = PCA9848_CHANNEL_0,
	  .is_device_reset = false,
	  .is_mb_reset = false,
	  .is_pe_reset = false },
};

LOG_MODULE_REGISTER(plat_isr);

void cxl_set_eid_work_handler(struct k_work *work_item)
{
	bool ret = false;
	uint8_t retry = 5;
	uint8_t index = 0;

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

	for (index = 0; index < retry; ++index) {
		/** Mutex lock bus **/
		if (k_mutex_lock(meb_mutex, K_MSEC(MUTEX_LOCK_INTERVAL_MS))) {
			LOG_ERR("mutex locked failed on bus1, cxl id: 0x%x",
				work_info->cxl_card_id);
			continue;
		}

		/** Enable mux channel **/
		ret = set_mux_channel(meb_mux, MUTEX_LOCK_ENABLE);
		if (ret == false) {
			k_mutex_unlock(meb_mutex);
			continue;
		}

		ret = set_mux_channel(cxl_mux, MUTEX_LOCK_ENABLE);
		if (ret == false) {
			k_mutex_unlock(meb_mutex);
			continue;
		}

		/** Set endpoint id **/
		ret = get_set_cxl_endpoint(work_info->cxl_card_id, MCTP_EID_CXL);
		if (ret != true) {
			LOG_ERR("Fail to set eid, cxl id: 0x%x", work_info->cxl_card_id);
			k_mutex_unlock(meb_mutex);
			continue;
		}

		if (pm8702_table[work_info->cxl_card_id].is_init != true) {
			ret = pal_init_pm8702_info(work_info->cxl_card_id);
			if (ret != true) {
				LOG_ERR("Initial cxl id: 0x%x info fail", work_info->cxl_card_id);
				k_mutex_unlock(meb_mutex);
				continue;
			}
		}

		/** mutex unlock bus **/
		k_mutex_unlock(meb_mutex);
		break;
	}
}

void add_sel_log_to_bmc_handler(struct k_work *work_item)
{
	int ret;

	struct k_work_delayable *dwork = k_work_delayable_from_work(work_item);
	cxl_work_info *work_info = CONTAINER_OF(dwork, cxl_work_info, perst_add_sel_work);
	ipmi_msg msg = { 0 };

	common_addsel_msg_t sel_msg;
	sel_msg.InF_target = MCTP;
	sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
	if (work_info->is_pe_reset) {
		sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
	} else {
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
	}
	sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
	sel_msg.event_data1 = work_info->cxl_card_id;
	sel_msg.event_data2 = work_info->is_mb_reset;
	sel_msg.event_data3 = work_info->is_pe_reset;

	pal_construct_ipmi_add_sel_msg(&msg, &sel_msg);

	ret = pal_pldm_send_ipmi_request(&msg, MCTP_EID_BMC);
	if (ret < 0) {
		LOG_ERR("Failed to send GPIO interrupt event to BMC, cxl id(%d) ret(%d)",
			work_info->cxl_card_id, ret);
	}
}

void init_cxl_work()
{
	uint8_t index = 0;
	for (index = 0; index < ARRAY_SIZE(cxl_work_item); ++index) {
		if (cxl_work_item[index].is_init != true) {
			k_work_init_delayable(&(cxl_work_item[index].device_reset_work),
					      cxl_ioexp_alert_handler);
			k_work_init_delayable(&(cxl_work_item[index].set_eid_work),
					      cxl_set_eid_work_handler);
			k_work_init_delayable(&(cxl_work_item[index].perst_add_sel_work),
					      add_sel_log_to_bmc_handler);

			cxl_work_item[index].is_init = true;
		}
	}
}

int set_cxl_device_reset_pin(uint8_t val, bool is_mutex)
{
	int ret = 0;
	uint8_t retry = 5;
	uint8_t set_val = 0;
	I2C_MSG msg = { 0 };

	/** Read cxl U15 ioexp output port status **/
	msg.bus = MEB_CXL_BUS;
	msg.target_addr = CXL_IOEXP_U15_ADDR;
	msg.rx_len = 1;
	msg.tx_len = 1;
	msg.data[0] = TCA9555_OUTPUT_PORT_REG_0;

	if (is_mutex) {
		ret = i2c_master_read(&msg, retry);
	} else {
		ret = i2c_master_read_without_mutex(&msg, retry);
	}
	if (ret != 0) {
		LOG_ERR("Unable to read ioexp bus: %u addr: 0x%02x", msg.bus, msg.target_addr);
		return -1;
	}

	switch (val) {
	case HIGH_ACTIVE:
		set_val = msg.data[0] | CXL_IOEXP_DEV_RESET_BIT;
		break;
	case HIGH_INACTIVE:
		set_val = msg.data[0] & (~CXL_IOEXP_DEV_RESET_BIT);
		break;
	default:
		LOG_ERR("Invalid set device reset pin val: 0x%x", val);
		return -1;
	}

	memset(&msg, 0, sizeof(I2C_MSG));
	msg.bus = MEB_CXL_BUS;
	msg.target_addr = CXL_IOEXP_U15_ADDR;
	msg.tx_len = 2;
	msg.data[0] = TCA9555_OUTPUT_PORT_REG_0;
	msg.data[1] = set_val;

	if (is_mutex) {
		ret = i2c_master_write(&msg, retry);
	} else {
		ret = i2c_master_write_without_mutex(&msg, retry);
	}
	if (ret != 0) {
		LOG_ERR("Unable to write ioexp to val: 0x%x, bus: %u, addr: 0x%02x", set_val,
			msg.bus, msg.target_addr);
		return -1;
	}

	return 0;
}

int check_cxl_power_status(bool is_mutex)
{
	int ret = 0;
	uint8_t retry = 5;
	uint8_t u17_input_port0_status = 0;
	uint8_t u17_input_port1_status = 0;
	I2C_MSG msg = { 0 };

	/** Read cxl U17 ioexp input port0 status **/
	msg.bus = MEB_CXL_BUS;
	msg.target_addr = CXL_IOEXP_U17_ADDR;
	msg.rx_len = 1;
	msg.tx_len = 1;
	msg.data[0] = TCA9555_INPUT_PORT_REG_0;

	if (is_mutex) {
		ret = i2c_master_read(&msg, retry);
	} else {
		ret = i2c_master_read_without_mutex(&msg, retry);
	}
	if (ret != 0) {
		LOG_ERR("Unable to read ioexp bus: %u addr: 0x%02x", msg.bus, msg.target_addr);
		return -1;
	}

	u17_input_port0_status = msg.data[0];

	/** Read cxl U17 ioexp input port1 status **/
	memset(&msg, 0, sizeof(I2C_MSG));
	msg.bus = MEB_CXL_BUS;
	msg.target_addr = CXL_IOEXP_U17_ADDR;
	msg.rx_len = 1;
	msg.tx_len = 1;
	msg.data[0] = TCA9555_INPUT_PORT_REG_1;

	if (is_mutex) {
		ret = i2c_master_read(&msg, retry);
	} else {
		ret = i2c_master_read_without_mutex(&msg, retry);
	}
	if (ret != 0) {
		LOG_ERR("Unable to read ioexp bus: %u addr: 0x%02x", msg.bus, msg.target_addr);
		return -1;
	}

	u17_input_port1_status = msg.data[0];

	/** Check CXL controller power good **/
	if ((u17_input_port0_status & CXL_IOEXP_CONTROLLER_PWRGD_VAL) ==
		    CXL_IOEXP_CONTROLLER_PWRGD_VAL &&
	    (u17_input_port1_status & CXL_IOEXP_DIMM_PWRGD_VAL) == CXL_IOEXP_DIMM_PWRGD_VAL) {
		return CXL_ALL_POWER_GOOD;
	} else {
		return CXL_NOT_ALL_POWER_GOOD;
	}
}

int cxl_pe_reset_control(uint8_t cxl_card_id)
{
	int ret = 0;
	uint8_t retry = 5;
	uint8_t mb_reset_status = 0;
	uint8_t u15_output_status = 0;
	I2C_MSG msg = { 0 };

	/** Read cxl U15 ioexp input port0 status **/
	msg.bus = MEB_CXL_BUS;
	msg.target_addr = CXL_IOEXP_U15_ADDR;
	msg.rx_len = 1;
	msg.tx_len = 1;
	msg.data[0] = TCA9555_INPUT_PORT_REG_0;

	ret = i2c_master_read_without_mutex(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Unable to read ioexp bus: %u addr: 0x%02x", msg.bus, msg.target_addr);
		return -1;
	}

	mb_reset_status = msg.data[0] & CXL_IOEXP_MB_RESET_BIT;

	/** Read cxl U15 ioexp input port1 status **/
	memset(&msg, 0, sizeof(I2C_MSG));
	msg.bus = MEB_CXL_BUS;
	msg.target_addr = CXL_IOEXP_U15_ADDR;
	msg.rx_len = 1;
	msg.tx_len = 1;
	msg.data[0] = TCA9555_INPUT_PORT_REG_1;

	ret = i2c_master_read_without_mutex(&msg, retry);
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

	ret = i2c_master_read_without_mutex(&msg, retry);
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
		cxl_work_item[cxl_card_id].is_mb_reset = true;
		msg.data[1] = u15_output_status | CXL_IOEXP_ASIC_PERESET_BIT;
		if (cxl_work_item[cxl_card_id].is_pe_reset != true) {
			k_work_schedule_for_queue(&plat_work_q,
						  &cxl_work_item[cxl_card_id].set_eid_work,
						  K_MSEC(CXL_DRIVE_READY_DELAY_MS));
		}
	} else {
		cxl_work_item[cxl_card_id].is_mb_reset = false;
		msg.data[1] = u15_output_status & (~CXL_IOEXP_ASIC_PERESET_BIT);
	}

	LOG_INF("[%s] cxl: 0x%x, mb_status: 0x%x, output_status: 0x%x, write: 0x%x", __func__,
		cxl_card_id, mb_reset_status, u15_output_status, msg.data[1]);

	ret = i2c_master_write_without_mutex(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Unable to write ioexp bus: %u addr: 0x%02x", msg.bus, msg.target_addr);
		return -1;
	} else {
		cxl_work_item[cxl_card_id].is_pe_reset = (mb_reset_status ? true : false);
	}

	if (debug_sel_mode == 1) {
		k_work_schedule_for_queue(&plat_work_q,
					  &cxl_work_item[cxl_card_id].perst_add_sel_work,
					  K_MSEC(CXL_DEBUG_SEL_DELAY_MS));
	}

	return 0;
}

bool check_ioexp_status(uint8_t cxl_card_id)
{
	int ret = 0;

	/** Check CXL controller power good **/
	ret = check_cxl_power_status(MUTEX_LOCK_DISENABLE);
	if (ret < 0) {
		LOG_ERR("Check CXL controller power fail");
		return false;
	}

	if (ret == CXL_ALL_POWER_GOOD) {
		if (cxl_work_item[cxl_card_id].is_device_reset != true) {
			LOG_INF("[%s] cxl: 0x%x, do device reset", __func__, cxl_card_id);
			ret = set_cxl_device_reset_pin(HIGH_ACTIVE, MUTEX_LOCK_DISENABLE);
			if (ret == 0) {
				cxl_work_item[cxl_card_id].is_device_reset = true;
			} else {
				LOG_ERR("CXL device reset fail");
			}
		}
	} else {
		set_cxl_device_reset_pin(HIGH_INACTIVE, MUTEX_LOCK_DISENABLE);
		cxl_work_item[cxl_card_id].is_device_reset = false;
	}

	ret = cxl_pe_reset_control(cxl_card_id);
	if (ret != 0) {
		LOG_ERR("CXL pr-reset control fail");
		return false;
	}

	return true;
}

void cxl_ioexp_alert_handler(struct k_work *work_item)
{
	bool ret = false;
	struct k_work_delayable *dwork = k_work_delayable_from_work(work_item);
	cxl_work_info *cxl_info = CONTAINER_OF(dwork, cxl_work_info, device_reset_work);

	LOG_INF("[%s] cxl: 0x%x", __func__, cxl_info->cxl_card_id);

	is_interrupt_ongoing = true;

	/** MEB mux for cxl channels **/
	mux_config meb_mux = { 0 };
	meb_mux.bus = MEB_CXL_BUS;
	meb_mux.target_addr = CXL_FRU_MUX0_ADDR;
	meb_mux.channel = cxl_info->cxl_channel;

	/** CXL mux for sensor channels **/
	mux_config cxl_mux = { 0 };
	cxl_mux.bus = MEB_CXL_BUS;
	cxl_mux.target_addr = CXL_FRU_MUX1_ADDR;
	cxl_mux.channel = CXL_IOEXP_MUX_CHANNEL;

	/** Enable mux channel **/
	ret = set_mux_channel(meb_mux, MUTEX_LOCK_DISENABLE);
	if (ret == false) {
		LOG_ERR("switch cxl %d meb mux failed", cxl_info->cxl_card_id);
		goto exit;
	}

	ret = set_mux_channel(cxl_mux, MUTEX_LOCK_DISENABLE);
	if (ret == false) {
		LOG_ERR("switch cxl %d cxl mux failed", cxl_info->cxl_card_id);
		goto exit;
	}

	/** Check io expander **/
	ret = check_ioexp_status(cxl_info->cxl_card_id);
	if (ret == false) {
		LOG_ERR("Check cxl %d io expander failed", cxl_info->cxl_card_id);
		goto exit;
	}

	/** Initial ioexp U14 **/
	ret = cxl_single_ioexp_alert_reset(IOEXP_U14, MUTEX_LOCK_DISENABLE);
	if (ret == false) {
		LOG_ERR("Check cxl %d io expander failed", cxl_info->cxl_card_id);
		goto exit;
	}

exit:
	meb_mux.channel = 0;
	ret = set_mux_channel(meb_mux, MUTEX_LOCK_DISENABLE);
	if (ret == false) {
		LOG_ERR("switch cxl %d meb mux failed", cxl_info->cxl_card_id);
	}
	is_interrupt_ongoing = false;
	return;
}

void check_ssd_power_good_timeout()
{
	uint8_t index = 0;
	uint8_t retry = 3;
	uint8_t status = 0;
	uint8_t bit_count = 8;
	I2C_MSG i2c_msg = { 0 };

	i2c_msg.bus = CPLD_BUS;
	i2c_msg.target_addr = CPLD_ADDR;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = CPLD_SSD_POWER_GOOD_TIMEOUT;
	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Get SSD power good timeout register value fail");
	} else {
		for (index = 0; index < bit_count; ++index) {
			if (i2c_msg.data[0] & BIT(index)) {
				status = ((index % 2) ?
						  PLDM_STATE_SET_OEM_DEVICE_12V_AUX_NO_POWER_GOOD :
						  PLDM_STATE_SET_OEM_DEVICE_3V3_AUX_NO_POWER_GOOD);
				plat_send_ssd_power_fault_event(index / 2, status);
			}
		}
	}
}

#define SET_RESET_SMB4_MUX_DELAY_MS 10
K_WORK_DELAYABLE_DEFINE(set_reset_smb4_mux_pin_work, set_reset_smb4_mux_pin);
#define SSD_POWER_GOOD_TIMEOUT_DELAY_MS 5000
K_WORK_DELAYABLE_DEFINE(check_ssd_power_good_timeout_work, check_ssd_power_good_timeout);
void ISR_NORMAL_PWRGD()
{
	uint8_t index = 0;
	set_DC_status(MEB_NORMAL_PWRGD_BIC);
	init_ssd_power_fault_work();

	if (gpio_get(MEB_NORMAL_PWRGD_BIC) == HIGH_INACTIVE) {
		for (index = 0; index < CXL_CARD_8; ++index) {
			set_cxl_eid_flag(index, CLEAR_EID_FLAG);
			clear_cxl_card_cache_value(index);
		}
		clear_ssd_power_fault_flag();
		if (k_work_cancel_delayable(&check_ssd_power_good_timeout_work) != 0) {
			LOG_ERR("Cancel check_ssd_power_good_timeout_work fail");
		}
	} else {
		for (index = 0; index < ARRAY_SIZE(cxl_work_item); ++index) {
			cxl_work_item[index].is_device_reset = false;
		}
		k_work_schedule_for_queue(&plat_work_q, &check_ssd_power_good_timeout_work,
					  K_MSEC(SSD_POWER_GOOD_TIMEOUT_DELAY_MS));
	}

	k_work_schedule_for_queue(&plat_work_q, &set_reset_smb4_mux_pin_work,
				  K_MSEC(SET_RESET_SMB4_MUX_DELAY_MS));
}

void ISR_CXL_IOEXP_ALERT0()
{
	k_work_schedule(&cxl_work_item[CXL_CARD_8].device_reset_work,
			K_MSEC(CXL_POWER_GOOD_DELAY_MS));
}

void ISR_CXL_IOEXP_ALERT1()
{
	k_work_schedule(&cxl_work_item[CXL_CARD_7].device_reset_work,
			K_MSEC(CXL_POWER_GOOD_DELAY_MS));
}

void ISR_CXL_IOEXP_ALERT2()
{
	k_work_schedule(&cxl_work_item[CXL_CARD_6].device_reset_work,
			K_MSEC(CXL_POWER_GOOD_DELAY_MS));
}

void ISR_CXL_IOEXP_ALERT3()
{
	k_work_schedule(&cxl_work_item[CXL_CARD_5].device_reset_work,
			K_MSEC(CXL_POWER_GOOD_DELAY_MS));
}

void ISR_CXL_IOEXP_ALERT4()
{
	k_work_schedule(&cxl_work_item[CXL_CARD_3].device_reset_work,
			K_MSEC(CXL_POWER_GOOD_DELAY_MS));
}

void ISR_CXL_IOEXP_ALERT5()
{
	k_work_schedule(&cxl_work_item[CXL_CARD_4].device_reset_work,
			K_MSEC(CXL_POWER_GOOD_DELAY_MS));
}

void ISR_CXL_IOEXP_ALERT6()
{
	k_work_schedule(&cxl_work_item[CXL_CARD_1].device_reset_work,
			K_MSEC(CXL_POWER_GOOD_DELAY_MS));
}

void ISR_CXL_IOEXP_ALERT7()
{
	k_work_schedule(&cxl_work_item[CXL_CARD_2].device_reset_work,
			K_MSEC(CXL_POWER_GOOD_DELAY_MS));
}
