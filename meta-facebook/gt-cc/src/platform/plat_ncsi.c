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
#include "ncsi.h"
#include "mctp.h"
#include "plat_ncsi.h"

LOG_MODULE_REGISTER(plat_ncsi);

#define MELLANOX_SET_SELF_RECOVERY_SETTING_CHANNEL 0x1F
#define MELLANOX_IANA 0x19810000

uint8_t mellanox_cx7_set_self_recovery_setting(uint8_t mctp_dest_eid)
{
	mctp *mctp_inst = NULL;
	mctp_ext_params ext_params = { 0 };
	if (get_mctp_info_by_eid(mctp_dest_eid, &mctp_inst, &ext_params) == false) {
		LOG_ERR("Failed to get mctp info by eid 0x%x", mctp_dest_eid);
		return false;
	}

	int resp_len = sizeof(struct mellanox_set_self_recovery_setting_resp);
	struct mellanox_set_self_recovery_setting_resp resp = { 0 };

	uint16_t req_len = (uint8_t)sizeof(struct mellanox_set_self_recovery_setting_req) -
			   NCSI_CHECHSUM_LENGTH;
	struct mellanox_set_self_recovery_setting_req req = { 0 };

	req.iana[0] = MELLANOX_IANA & 0xFF;
	req.iana[1] = (MELLANOX_IANA >> 8) & 0xFF;
	req.iana[2] = (MELLANOX_IANA >> 16) & 0xFF;
	req.iana[3] = (MELLANOX_IANA >> 24) & 0xFF;
	req.command_rev = 0x00;
	req.command_id = 0x01;
	req.parameter = 0x28;
	req.mode = 0x01; // 0x00 - Self recovery disabled, 0x01 - Self recovery enabled
	req.checksum = 0x00000000; // responser will ignore the checksum if checksum is 0x00000000

	ncsi_msg msg = { 0 };
	memcpy(&msg.ext_params, &ext_params, sizeof(mctp_ext_params));

	msg.hdr.command = NCSI_COMMAND_OEM;
	msg.hdr.rq = NCSI_COMMAND_REQUEST;
	msg.hdr.channel_id = MELLANOX_SET_SELF_RECOVERY_SETTING_CHANNEL;
	msg.hdr.payload_length_high = (req_len & NCSI_PAYLOAD_LENGTH_HIGH_MASK) >> 8;
	msg.hdr.payload_length_low = req_len & NCSI_PAYLOAD_LENGTH_LOW_MASK;

	msg.buf = (uint8_t *)&req;
	msg.len = req_len;

	if (mctp_ncsi_read(mctp_inst, &msg, (uint8_t *)&resp, resp_len) == 0) {
		LOG_ERR("MCTP NCSI fail, eid = 0x%x, command: 0x%x", mctp_dest_eid,
			msg.hdr.command);
		return false;
	}

	if ((resp.response_code != NCSI_COMMAND_COMPLETED) || (resp.reason_code != NCSI_NO_ERROR)) {
		LOG_ERR("Failed to set self recovery setting, response_code = 0x%x, reason_code = 0x%x",
			resp.response_code, resp.reason_code);
		return false;
	}

	return true;
}

uint8_t mellanox_cx7_get_infiniband_link_status(uint8_t mctp_dest_eid, uint8_t *link_type,
						uint16_t *response_code, uint16_t *reason_code)
{
	mctp *mctp_inst = NULL;
	mctp_ext_params ext_params = { 0 };
	if (get_mctp_info_by_eid(mctp_dest_eid, &mctp_inst, &ext_params) == false) {
		LOG_ERR("Failed to get mctp info by eid 0x%x", mctp_dest_eid);
		return false;
	}

	int resp_len = sizeof(struct get_infiniband_link_status_resp);
	struct get_infiniband_link_status_resp resp = { 0 };

	uint16_t req_len =
		(uint8_t)sizeof(struct get_infiniband_link_status_req) - NCSI_CHECHSUM_LENGTH;
	struct get_infiniband_link_status_req req = { 0 };

	req.checksum = 0x00000000; // responser will ignore the checksum if checksum is 0x00000000

	ncsi_msg msg = { 0 };
	memcpy(&msg.ext_params, &ext_params, sizeof(mctp_ext_params));

	msg.hdr.command = NCSI_COMMAND_GET_INFINIBAND_LINK_STATUS;
	msg.hdr.rq = NCSI_COMMAND_REQUEST;
	msg.hdr.channel_id = 0x00;
	msg.hdr.payload_length_high = (req_len & NCSI_PAYLOAD_LENGTH_HIGH_MASK) >> 8;
	msg.hdr.payload_length_low = req_len & NCSI_PAYLOAD_LENGTH_LOW_MASK;

	msg.buf = (uint8_t *)&req;
	msg.len = req_len;

	if (mctp_ncsi_read(mctp_inst, &msg, (uint8_t *)&resp, resp_len) == 0) {
		LOG_ERR("MCTP NCSI get_infiniband_link_status fail, eid = 0x%x, command: 0x%x",
			mctp_dest_eid, msg.hdr.command);
		return false;
	}

	//response_code and reason_code is low byte first, switch low byte and high byte
	*response_code = (resp.response_code << 8) | (resp.response_code >> 8);
	*reason_code = (resp.reason_code << 8) | (resp.reason_code >> 8);

	if ((*response_code == NCSI_COMMAND_COMPLETED) && (*reason_code == NCSI_NO_ERROR))
		*link_type = resp.link_type;

	return true;
}

uint8_t mellanox_cx7_clear_initial_state(uint8_t mctp_dest_eid)
{
	mctp *mctp_inst = NULL;
	mctp_ext_params ext_params = { 0 };
	if (get_mctp_info_by_eid(mctp_dest_eid, &mctp_inst, &ext_params) == false) {
		LOG_ERR("Failed to get mctp info by eid 0x%x", mctp_dest_eid);
		return false;
	}

	int resp_len = sizeof(struct clear_initial_state_resp);
	struct clear_initial_state_resp resp = { 0 };

	uint16_t req_len = (uint8_t)sizeof(struct clear_initial_state_req) - NCSI_CHECHSUM_LENGTH;
	struct clear_initial_state_req req = { 0 };

	req.checksum = 0x00000000; // responser will ignore the checksum if checksum is 0x00000000

	ncsi_msg msg = { 0 };
	memcpy(&msg.ext_params, &ext_params, sizeof(mctp_ext_params));

	msg.hdr.command = NCSI_COMMAND_CLEAR_INITIAL_STATE;
	msg.hdr.rq = NCSI_COMMAND_REQUEST;
	msg.hdr.channel_id = 0x00;
	msg.hdr.payload_length_high = (req_len & NCSI_PAYLOAD_LENGTH_HIGH_MASK) >> 8;
	msg.hdr.payload_length_low = req_len & NCSI_PAYLOAD_LENGTH_LOW_MASK;

	msg.buf = (uint8_t *)&req;
	msg.len = req_len;

	if (mctp_ncsi_read(mctp_inst, &msg, (uint8_t *)&resp, resp_len) == 0) {
		LOG_ERR("MCTP NCSI clear_initial_state fail, eid = 0x%x, command: 0x%x",
			mctp_dest_eid, msg.hdr.command);
		return false;
	}

	if ((resp.response_code != NCSI_COMMAND_COMPLETED) || (resp.reason_code != NCSI_NO_ERROR)) {
		LOG_ERR("Failed to clear_initial_state, response_code = 0x%x, reason_code = 0x%x",
			resp.response_code, resp.reason_code);
		return false;
	}

	return true;
}
