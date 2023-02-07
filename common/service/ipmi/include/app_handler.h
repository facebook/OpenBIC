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

#ifndef APP_HANDLER_H
#define APP_HANDLER_H

#include "ipmi.h"
#include "plat_fru.h"

#ifndef BIC_FRU_DEV_ID
#define BIC_FRU_DEV_ID 0
#endif

#define GET_TEST_RESULT 0

typedef struct SELF_TEST_RESULT_STRUCT {
	uint8_t status;

	struct RESULT {
		uint8_t cannotAccessSelDev : 1;
		uint8_t cannotAccessSdrRepo : 1;
		uint8_t cannotAccessBmcFruDev : 1;
		uint8_t ipmbLinesDead : 1;
		uint8_t sdrRepoEmpty : 1;
		uint8_t internalCorrupt : 1;
		uint8_t updateFwCorrupt : 1;
		uint8_t opFwCorrupt : 1;
	} result;
} SELF_TEST_RESULT;

void APP_GET_DEVICE_ID(ipmi_msg *msg);
void APP_COLD_RESET(ipmi_msg *msg);
void APP_WARM_RESET(ipmi_msg *msg);
void APP_GET_SELFTEST_RESULTS(ipmi_msg *msg);
void APP_MASTER_WRITE_READ(ipmi_msg *msg);

#ifdef CONFIG_ESPI
void APP_GET_SYSTEM_GUID(ipmi_msg *msg);
#endif

void IPMI_APP_handler(ipmi_msg *msg);

#endif
