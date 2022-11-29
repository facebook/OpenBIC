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
#include <plat_power_seq.h>

#include "hal_gpio.h"
#include "util_spi.h"
#include "util_sys.h"
#include "plat_gpio.h"

#define CXL_FLASH_TO_BIC 1
#define CXL_FLASH_TO_CXL 0

#define CXL_UPDATE_MAX_OFFSET 0x2000000

static bool switch_cxl_spi_mux(int gpio_status)
{
	if (gpio_status != CXL_FLASH_TO_BIC && gpio_status != CXL_FLASH_TO_CXL) {
		printf("[%s] Invalid argument\n", __func__);
		return false;
	}

	if (gpio_set(SPI_MASTER_SEL, gpio_status)) {
		printf("Fail to switch the flash to %s\n",
		       (gpio_status == CXL_FLASH_TO_BIC) ? "BIC" : "PIONEER");
		return false;
	}

	return true;
}

static bool control_flash_power(int power_state)
{
	int control_mode = 0;

	switch (power_state) {
	case POWER_OFF:
		control_mode = DISABLE_POWER_MODE;
		break;
	case POWER_ON:
		control_mode = ENABLE_POWER_MODE;
		break;
	default:
		return false;
	}

	for (int retry = 3;; retry--) {
		if (gpio_get(P1V8_ASIC_PG_R) == power_state) {
			return true;
		}

		if (!retry) {
			break;
		}
		control_power_stage(control_mode, P1V8_ASIC_EN_R);
		k_msleep(CHKPWR_DELAY_MSEC);
	}

	printf("Fail to %s the ASIC_1V8\n", (power_state == POWER_OFF) ? "disable" : "enable");

	return false;
}

uint8_t fw_update_cxl(uint32_t offset, uint16_t msg_len, uint8_t *msg_buf, bool sector_end)
{
	uint8_t ret = FWUPDATE_UPDATE_FAIL;
	set_CXL_update_status(POWER_ON);

	if (offset > CXL_UPDATE_MAX_OFFSET) {
		return FWUPDATE_OVER_LENGTH;
	}

	// Enable the P1V8_ASCI to power the flash
	if (control_flash_power(POWER_ON) == false) {
		return FWUPDATE_UPDATE_FAIL;
	}

	// Set high to choose the BIC as the host
	if (switch_cxl_spi_mux(CXL_FLASH_TO_BIC) == false) {
		printf("Fail to switch PIONEER flash to BIC\n");
		return FWUPDATE_UPDATE_FAIL;
	}

	ret = fw_update(offset, msg_len, msg_buf, sector_end, DEVSPI_SPI1_CS0);

	if (sector_end || ret != FWUPDATE_SUCCESS) {
		control_flash_power(POWER_OFF);
		switch_cxl_spi_mux(CXL_FLASH_TO_CXL);
		set_CXL_update_status(POWER_OFF);
	}

	return ret;
}
