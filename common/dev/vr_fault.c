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

#include <logging/log.h>
#include "vr_fault.h"

#include "pmbus.h"
#include "util_worker.h"
#include "hal_i2c.h"
#include "sensor.h"
#include "libipmi.h"

LOG_MODULE_REGISTER(vr_fault);

// PMBus command code 0x78 ~ 0x80
const uint8_t vr_pmbus_data_list[] = { PMBUS_STATUS_BYTE,	 PMBUS_STATUS_WORD,
				       PMBUS_STATUS_VOUT,	 PMBUS_STATUS_IOUT,
				       PMBUS_STATUS_INPUT,	 PMBUS_STATUS_TEMPERATURE,
				       PMBUS_STATUS_CML,	 PMBUS_STATUS_OTHER,
				       PMBUS_STATUS_MFR_SPECIFIC };

// Use for get VR vender type
__weak uint8_t pal_get_vr_vender_type()
{
	return 0;
}

// Use for record VR power fault event to BMC
__weak void pal_record_vr_power_fault(uint8_t event_type, uint8_t error_type, uint8_t vr_data1,
				      uint8_t vr_data2)
{
	return;
}

// Use for skip some PMBus command code, base on different VR vender and page
__weak bool pal_skip_pmbus_cmd_code(uint8_t vendor_type, uint8_t cmd, uint8_t page)
{
	return false;
}

void vr_pwr_fault_handler(struct k_work *work_item)
{
	uint8_t retry = 5;
	uint8_t cpld_vr_data = 0x00;
	uint8_t vr_vender_type = pal_get_vr_vender_type();

	// Read CPLD register check which VR power rail occur VR power fault
	I2C_MSG msg = { 0 };
	msg.bus = cpld_vr_reg_table.bus;
	msg.target_addr = cpld_vr_reg_table.addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = cpld_vr_reg_table.offset;
	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("Failed to get CPLD VR reg !, bus: 0x%x, addr: 0x%x, reg: 0x%x", msg.bus,
			msg.target_addr, cpld_vr_reg_table.offset);
		return;
	} else {
		cpld_vr_data = msg.data[0];
	}

	if (cpld_vr_data == VR_CPLD_NO_PWR_FAULT) {
		LOG_ERR("No VR power fault occured from CPLD VR reg !");
		return;
	}

	// Check can get VR vender type
	if (vr_vender_type == VR_TYPE_IS_UNKNOWN) {
		LOG_ERR("Failed to get VR vender type !");
		return;
	};

	disable_sensor_poll();
	k_msleep(VR_MONITOR_STOP_TIME); // wait for vr monitor stop

	// Check which VR power rail occur VR power fault
	for (int i = 0; i < vr_pwr_fault_table_size; i++) {
		// detect which VR power rail fault occur
		if (cpld_vr_data & vr_pwr_fault_table[i].bit) {
			// set to the correct VR page
			msg.bus = vr_pwr_fault_table[i].bus;
			msg.target_addr = vr_pwr_fault_table[i].addr;
			msg.tx_len = 2;
			msg.data[0] = PMBUS_PAGE;
			msg.data[1] = vr_pwr_fault_table[i].page;
			if (i2c_master_write(&msg, retry)) {
				LOG_ERR("Failed to set VR page !, bus: %d, addr: 0x%x, page: 0x%x",
					vr_pwr_fault_table[i].bus, vr_pwr_fault_table[i].addr,
					vr_pwr_fault_table[i].page);
				enable_sensor_poll();
				return;
			}

			for (int j = 0; j < ARRAY_SIZE(vr_pmbus_data_list); j++) {
				uint8_t pmbus_cmd = vr_pmbus_data_list[j];

				if (pal_skip_pmbus_cmd_code(vr_vender_type, pmbus_cmd,
							    vr_pwr_fault_table[i].page)) {
					continue;
				}

				memset(msg.data, 0, sizeof(msg.data)); // clear data buffer

				msg.tx_len = 1;
				msg.data[0] = pmbus_cmd;
				if (pmbus_cmd == PMBUS_STATUS_WORD) {
					msg.rx_len = 2;
				} else {
					msg.rx_len = 1;
				}

				if (i2c_master_read(&msg, retry)) {
					LOG_ERR("Failed to get VR reg !, bus: %d, addr: 0x%x, reg: 0x%x",
						msg.bus, msg.target_addr, pmbus_cmd);
					continue;
				} else {
					if (pmbus_cmd == PMBUS_STATUS_WORD) {
						uint8_t low_byte = msg.data[0];
						uint8_t high_byte = msg.data[1];

						pal_record_vr_power_fault(
							IPMI_EVENT_TYPE_SENSOR_SPECIFIC,
							vr_pwr_fault_table[i].event, pmbus_cmd,
							low_byte);
						pal_record_vr_power_fault(
							IPMI_EVENT_TYPE_SENSOR_SPECIFIC,
							vr_pwr_fault_table[i].event, pmbus_cmd,
							high_byte);
					} else {
						pal_record_vr_power_fault(
							IPMI_EVENT_TYPE_SENSOR_SPECIFIC,
							vr_pwr_fault_table[i].event, pmbus_cmd,
							msg.data[0]);
					}
				}
			}
		}
	}

	enable_sensor_poll();
}
