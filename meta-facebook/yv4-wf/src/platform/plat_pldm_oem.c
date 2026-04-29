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

#include <zephyr.h>
#include <sys/printk.h>
#include <logging/log.h>
#include <logging/log_ctrl.h>
#include <stdlib.h>
#include <stdio.h>
#include "libutil.h"

#include "pldm_oem.h"

#include "plat_isr.h"
#include "plat_mctp.h"
#include "plat_spi.h"
#include "plat_pldm_oem.h"
#include "plat_gpio.h"
#include "ioexp_tca9555.h"
#include "util_spi.h"

LOG_MODULE_REGISTER(plat_pldm_oem);

static uint8_t plat_switch_flash_mux_wf(uint8_t cxl_id)
{
	uint8_t value = 0;
	if (get_ioe_value(ADDR_IOE1, TCA9555_OUTPUT_PORT_REG_0, &value) != 0) {
		return 1;
	}
	switch (cxl_id) {
	case WF_COMPNT_CXL1:
		// Switch SPI1 MUX to BIC
		value = SETBIT(value, IOE_P05);
		break;
	case WF_COMPNT_CXL2:
		// Switch SPI2 MUX to BIC
		value = SETBIT(value, IOE_P06);
		break;
	default:
		LOG_ERR("Unknown CXL component ID %d", cxl_id);
		return 1;
	}

	set_ioe_value(ADDR_IOE1, TCA9555_OUTPUT_PORT_REG_0, value);

	return 0;
}

static uint8_t plat_switch_flash_mux_cxl(uint8_t cxl_id)
{
	uint8_t value = 0;
	if (get_ioe_value(ADDR_IOE1, TCA9555_OUTPUT_PORT_REG_0, &value) != 0) {
		return 1;
	}
	switch (cxl_id) {
	case WF_COMPNT_CXL1:
		// Switch SPI1 MUX to CXL
		value = CLEARBIT(value, IOE_P05);
		break;
	case WF_COMPNT_CXL2:
		// Switch SPI2 MUX to CXL
		value = CLEARBIT(value, IOE_P06);
		break;
	default:
		LOG_ERR("Unknown CXL component ID %d", cxl_id);
		return 1;
	}

	set_ioe_value(ADDR_IOE1, TCA9555_OUTPUT_PORT_REG_0, value);
	return 0;
}

uint8_t read_flash_data_cmd(void *mctp_inst, uint8_t *req_buf, uint16_t req_len,
			    uint8_t instance_id, uint8_t *resp_buf, uint16_t *resp_len,
			    void *ext_params)
{
	ARG_UNUSED(mctp_inst);
	ARG_UNUSED(instance_id);
	ARG_UNUSED(ext_params);
	struct _cxl_read_flash_req *req = (struct _cxl_read_flash_req *)req_buf;
	struct _cxl_read_flash_resp *resp = (struct _cxl_read_flash_resp *)resp_buf;
	//check request length
	if (req_len < sizeof(struct _cxl_read_flash_req)) {
		resp->completion_code = PLDM_ERROR_INVALID_LENGTH;
		*resp_len = sizeof(struct _cxl_read_flash_resp);
		return PLDM_SUCCESS;
	}
	//check iana
	if (check_iana(req->iana) == PLDM_ERROR) {
		resp->completion_code = PLDM_ERROR_INVALID_DATA;
		*resp_len = 1;
		return PLDM_SUCCESS;
	}

	set_iana(resp->iana, sizeof(resp->iana));
	resp->completion_code = PLDM_SUCCESS;
	resp->data_len = req->data_len;
	static bool is_init = false;
	static const struct device *flash_dev = NULL;
	uint8_t ret = 0;
	// check request to switch MUX or read flash data
	switch (req->transfer_flag) {
	case READ_CXL_FLASH_START:
		ret = plat_switch_flash_mux_wf(req->cxl_comp_id);
		if (ret) {
			LOG_ERR("Failed to switch SPI MUX to WF BIC");
			resp->completion_code = PLDM_ERROR;
			break;
		}
		ret = pal_init_cxl_flash_device(req->cxl_comp_id, &flash_dev);
		if (ret) {
			LOG_ERR("Failed to init CXL flash");
			resp->completion_code = PLDM_ERROR;
			break;
		}
		is_init = true;
		break;
	case READ_CXL_FLASH_DATA:
		if (is_init == true && flash_dev != NULL) {
			ret = pal_dump_cxl_flash_data(resp->data, req->data_offset, req->data_len,
						      flash_dev, req->cxl_comp_id);
			if (ret) {
				resp->completion_code = PLDM_ERROR;
			}
		} else {
			LOG_ERR("Flash device and MUX is not ready");
			resp->completion_code = PLDM_ERROR;
		}
		break;
	case READ_CXL_FLASH_END:
		ret = plat_switch_flash_mux_cxl(req->cxl_comp_id);
		if (ret) {
			LOG_ERR("Failed to switch SPI MUX to CXL");
			resp->completion_code = PLDM_ERROR;
			break;
		}
		is_init = false;
		flash_dev = NULL;
		break;
	default:
		LOG_ERR("Unknown transfer_flag: 0x%x", req->transfer_flag);
		resp->completion_code = PLDM_ERROR_INVALID_DATA;
		break;
	}

	if (resp->completion_code != PLDM_SUCCESS) {
		resp->data_len = 0;
	}

	*resp_len = sizeof(struct _cxl_read_flash_resp) + resp->data_len;
	LOG_INF("resp len: %d", *resp_len);
	return PLDM_SUCCESS;
}