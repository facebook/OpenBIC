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

#ifndef _PLAT_NCSI_H
#define _PLAT_NCSI_H

#ifdef __cplusplus
extern "C" {
#endif

struct mellanox_set_self_recovery_setting_req {
	uint8_t iana[4];
	uint8_t command_rev;
	uint8_t command_id;
	uint8_t parameter;
	uint32_t reserved;
	uint8_t mode;
	uint32_t checksum;
} __attribute__((packed));

struct mellanox_set_self_recovery_setting_resp {
	uint16_t response_code;
	uint16_t reason_code;
	uint32_t iana[4];
	uint8_t command_rev;
	uint8_t command_id;
	uint8_t parameter;
	uint32_t reserved;
	uint8_t mode;
	uint32_t checksum;
} __attribute__((packed));

uint8_t mellanox_cx7_set_self_recovery_setting(uint8_t mctp_dest_eid);

#ifdef __cplusplus
}
#endif

#endif /* _PLAT_NCSI_H */
