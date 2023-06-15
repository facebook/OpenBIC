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

#include <stdio.h>
#include <string.h>
#include <logging/log.h>
#include "libutil.h"
#include "ipmb.h"
#include "ipmi.h"
#include "plat_ipmb.h"
#include "plat_class.h"

LOG_MODULE_DECLARE(ipmb);

IPMB_config pal_IPMB_config_table[] = {
	// index, interface, channel, bus, channel_target_address, enable_status, self_address,
	// rx_thread_name, tx_thread_name
	{ BMC_IPMB_IDX, I2C_IF, BMC_IPMB, IPMB_BMC_BUS, BMC_I2C_ADDRESS, ENABLE, SELF_I2C_ADDRESS,
	  "RX_BMC_IPMB_TASK", "TX_BMC_IPMB_TASK" },
	{ EXP1_IPMB_IDX, I2C_IF, EXP1_IPMB, IPMB_EXP1_BUS, BIC1_I2C_ADDRESS, DISABLE,
	  SELF_I2C_ADDRESS, "RX_EPX1_IPMB_TASK", "TX_EXP1_IPMB_TASK" },
	{ EXP2_IPMB_IDX, I2C_IF, EXP2_IPMB, IPMB_EXP2_BUS, BIC2_I2C_ADDRESS, DISABLE,
	  SELF_I2C_ADDRESS, "RX_EPX2_IPMB_TASK", "TX_EXP2_IPMB_TASK" },
	{ RESERVED_IDX, RESERVED_IF, RESERVED, RESERVED_BUS, RESERVED_ADDRESS, DISABLE,
	  RESERVED_ADDRESS, "RESERVED_ATTR", "RESERVED_ATTR" },
};

bool pal_load_ipmb_config(void)
{
	CARD_STATUS _1ou_status = get_1ou_status();
	if (_1ou_status.present) {
		pal_IPMB_config_table[EXP1_IPMB_IDX].enable_status = ENABLE;
	}

	CARD_STATUS _2ou_status = get_2ou_status();
	if (_2ou_status.present) {
		// for reset of expansion board, enable ipmb and set i2c freq to 1Mhz
		if (_2ou_status.card_type == TYPE_1OU_EXP_WITH_E1S) {
			pal_IPMB_config_table[EXP2_IPMB_IDX].enable_status = ENABLE;
#ifdef CONFIG_I2C_IPMB_SLAVE
			pal_IPMB_config_table[EXP2_IPMB_IDX].channel = EXP3_IPMB;
#endif
		} else if ((_2ou_status.card_type & TYPE_2OU_DPV2_16) == TYPE_2OU_DPV2_16) {
			// for dpv2 sku, disable ipmb and set i2c freq to 400Khz for slave devices reading
			i2c_freq_set(pal_IPMB_config_table[EXP2_IPMB_IDX].bus, I2C_SPEED_FAST, 0);
			pal_IPMB_config_table[EXP2_IPMB_IDX].enable_status = DISABLE;
		} else {
			pal_IPMB_config_table[EXP2_IPMB_IDX].enable_status = ENABLE;
		}
	}

	memcpy(IPMB_config_table, pal_IPMB_config_table, sizeof(pal_IPMB_config_table));
	return true;
}

#ifdef CONFIG_I2C_IPMB_SLAVE
void pal_encode_response_bridge_cmd(ipmi_msg *bridge_msg, ipmi_msg_cfg *current_msg_rx,
				    IPMB_config *ipmb_cfg, IPMB_config *IPMB_config_tables)
{
	CHECK_NULL_ARG(bridge_msg);
	CHECK_NULL_ARG(current_msg_rx);
	CHECK_NULL_ARG(ipmb_cfg);
	CHECK_NULL_ARG(IPMB_config_tables);

	// handle the bridge commend from other fru
	if ((current_msg_rx->buffer.netfn == NETFN_OEM_1S_RES) &&
	    (current_msg_rx->buffer.cmd == CMD_OEM_1S_MSG_OUT)) {
		if (current_msg_rx->buffer.completion_code != CC_SUCCESS) {
			bridge_msg->data[0] = IANA_ID & 0xFF;
			bridge_msg->data[1] = (IANA_ID >> 8) & 0xFF;
			bridge_msg->data[2] = (IANA_ID >> 16) & 0xFF;
			bridge_msg->data[3] = current_msg_rx->buffer.completion_code;
			bridge_msg->data_len = current_msg_rx->buffer.data_len +
					       4; // add 4 byte len for bridge header
		} else {
			bridge_msg->data_len = current_msg_rx->buffer.data_len;
			memcpy(&bridge_msg->data[0], &current_msg_rx->buffer.data[0],
			       current_msg_rx->buffer.data_len);
		}
		bridge_msg->netfn = NETFN_OEM_1S_REQ; // Add bridge response header
		bridge_msg->cmd = CMD_OEM_1S_MSG_OUT;
		bridge_msg->completion_code = CC_SUCCESS;
		bridge_msg->seq = current_msg_rx->buffer.seq_source;
	} else {
		bridge_msg->data[0] =
			IANA_ID & 0xFF; // Move target response to bridge response data
		bridge_msg->data[1] = (IANA_ID >> 8) & 0xFF;
		bridge_msg->data[2] = (IANA_ID >> 16) & 0xFF;
		bridge_msg->data[3] = IPMB_config_tables[ipmb_cfg->index]
					      .channel; // return response source as request target
		bridge_msg->data[4] =
			current_msg_rx->buffer.netfn; // Move target response to bridge response data
		bridge_msg->data[5] = current_msg_rx->buffer.cmd;
		bridge_msg->data[6] = current_msg_rx->buffer.completion_code;
		bridge_msg->data_len =
			current_msg_rx->buffer.data_len + 7; // add 7 byte len for bridge header
		memcpy(&bridge_msg->data[7], &current_msg_rx->buffer.data[0],
		       current_msg_rx->buffer.data_len);
		bridge_msg->netfn = NETFN_OEM_1S_REQ; // Add bridge response header
		bridge_msg->cmd = CMD_OEM_1S_MSG_OUT;
		bridge_msg->completion_code = CC_SUCCESS;
		bridge_msg->seq = current_msg_rx->buffer.seq_source;
	}

	return;
}
#endif
