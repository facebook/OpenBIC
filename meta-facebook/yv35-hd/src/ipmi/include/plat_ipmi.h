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

#ifndef PLAT_IPMI_H
#define PLAT_IPMI_H

enum REQ_GET_CARD_TYPE {
	GET_1OU_CARD_TYPE = 0x0,
	GET_2OU_CARD_TYPE,
};

typedef struct oem_addsel_msg_t {
	uint8_t InF_target;
	uint8_t event_data[13];
} oem_addsel_msg_t;

bool plat_add_oem_sel_evt_record(oem_addsel_msg_t *sel_msg);

#endif
