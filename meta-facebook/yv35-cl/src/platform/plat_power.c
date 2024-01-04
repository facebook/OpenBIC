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

#include "plat_power.h"
#include <logging/log.h>
#include "plat_class.h"
#include "plat_i2c.h"
#include "hal_i2c.h"
#include "plat_mctp.h"
#include "ipmi.h"
#include "sensor.h"

LOG_MODULE_REGISTER(plat_power);

uint8_t _1ou_m2_mapping_table[4] = { 4, 3, 2, 1 };

static uint8_t last_vpp_pwr_status;

void init_vpp_power_status()
{
	I2C_MSG i2c_msg;
	uint8_t retry = 3;

	// Read VPP power status from SB CPLD
	memset(&i2c_msg, 0, sizeof(I2C_MSG));
	i2c_msg.bus = I2C_BUS1;
	i2c_msg.target_addr = CPLD_ADDR;
	i2c_msg.data[0] = CPLD_1OU_VPP_POWER_STATUS;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read CPU VPP status, bus0x%x addr0x%x offset0x%x", i2c_msg.bus,
			i2c_msg.target_addr, i2c_msg.data[0]);
		return;
	}

	last_vpp_pwr_status = i2c_msg.data[0];
}

uint8_t get_last_vpp_power_status()
{
	return last_vpp_pwr_status;
}

uint8_t get_updated_vpp_power_status()
{
	init_vpp_power_status();
	return last_vpp_pwr_status;
}

int get_set_1ou_m2_power(ipmi_msg *msg, uint8_t device_id, uint8_t option)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, -1);
	uint32_t iana = IANA_ID;
	ipmb_error status;

	memset(msg, 0, sizeof(ipmi_msg));
	msg->InF_source = SELF;
	msg->InF_target = EXP1_IPMB;
	msg->netfn = NETFN_OEM_1S_REQ;
	msg->cmd = CMD_OEM_1S_GET_SET_M2;
	msg->data_len = 5;
	memcpy(&msg->data[0], (uint8_t *)&iana, 3);
	msg->data[3] = _1ou_m2_mapping_table[device_id];
	msg->data[4] = option;
	status = ipmb_read(msg, IPMB_inf_index_map[msg->InF_target]);
	if (status != IPMB_ERROR_SUCCESS) {
		LOG_ERR("Failed to set get 1OU E1.S power: status 0x%x, id %d, option 0x%x", status,
			device_id, option);
		return -1;
	}

	return 0;
}
