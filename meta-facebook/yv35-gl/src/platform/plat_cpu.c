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
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <drivers/peci.h>
#include "intel_peci.h"
#include "libutil.h"
#include "hal_peci.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(pal_cpu);

bool pal_get_cpu_time(uint8_t addr, uint8_t cmd, uint8_t readlen, uint32_t *run_time)
{
	uint8_t writelen = 0x05;
	uint8_t time_buf[4] = { 0x12, 0x1F, 0x00, 0x00 };
	int ret = 0;

	uint8_t *readbuf = (uint8_t *)malloc(readlen * sizeof(uint8_t));
	if (!readbuf) {
		LOG_ERR("Get cpu time fail to allocate readbuf memory");
		return false;
	}

	ret = peci_write(cmd, addr, readlen, readbuf, writelen, time_buf);
	if (ret) {
		LOG_ERR("Get cpu time peci write error");
		goto cleanup;
	}
	if (readbuf[0] != PECI_CC_RSP_SUCCESS) {
		if (readbuf[0] == PECI_CC_ILLEGAL_REQUEST) {
			LOG_ERR("Get cpu time unknown request");
		} else {
			LOG_ERR("Get cpu time peci control hardware, firmware or associated logic error");
		}
		goto cleanup;
	}

	*run_time = readbuf[4];
	*run_time = (*run_time << 8) | readbuf[3];
	*run_time = (*run_time << 8) | readbuf[2];
	*run_time = (*run_time << 8) | readbuf[1];

	SAFE_FREE(readbuf);
	return true;
cleanup:
	SAFE_FREE(readbuf);
	return false;
}
