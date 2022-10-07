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

#ifdef CONFIG_PCC_ASPEED

#include <zephyr.h>
#include <device.h>
#include <stdlib.h>
#include <drivers/misc/aspeed/pcc_aspeed.h>
#include "ipmb.h"
#include "ipmi.h"
#include "libutil.h"
#include "pcc.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(pcc);

const struct device *pcc_dev;
static uint32_t pcc_read_buffer[PCC_BUFFER_LEN];
static uint16_t pcc_read_len = 0, pcc_read_index = 0;
static bool proc_4byte_postcode_ok = false;

uint16_t copy_pcc_read_buffer(uint16_t start, uint16_t length, uint8_t *buffer, uint16_t buffer_len)
{
	if ((buffer == NULL) || (buffer_len < (length * 4))) {
		return 0;
	}

	uint16_t current_index, i = 0;
	uint16_t current_read_len = pcc_read_len;
	uint16_t current_read_index = pcc_read_index;
	if (start < current_read_index) {
		current_index = current_read_index - start - 1;
	} else {
		current_index = current_read_index + PCC_BUFFER_LEN - start - 1;
	}

	for (; (i < length) && ((i + start) < current_read_len); i++) {
		buffer[4 * i] = pcc_read_buffer[current_index] & 0xFF;
		buffer[(4 * i) + 1] = (pcc_read_buffer[current_index] >> 8) & 0xFF;
		buffer[(4 * i) + 2] = (pcc_read_buffer[current_index] >> 16) & 0xFF;
		buffer[(4 * i) + 3] = (pcc_read_buffer[current_index] >> 24) & 0xFF;

		if (current_index == 0) {
			current_index = PCC_BUFFER_LEN - 1;
		} else {
			current_index--;
		}
	}
	return 4 * i;
}

void send_post_code_to_bmc()
{
	ipmi_msg *msg = (ipmi_msg *)malloc(sizeof(ipmi_msg));
	if (msg == NULL) {
		LOG_ERR("Memory allocation failed.");
		return;
	}

	memset(msg, 0, sizeof(ipmi_msg));
	msg->InF_source = SELF;
	msg->InF_target = BMC_IPMB;
	msg->netfn = NETFN_OEM_1S_REQ;
	msg->cmd = CMD_OEM_1S_SEND_4BYTE_POST_CODE_TO_BMC;
	msg->data_len = 8;
	msg->data[0] = IANA_ID & 0xFF;
	msg->data[1] = (IANA_ID >> 8) & 0xFF;
	msg->data[2] = (IANA_ID >> 16) & 0xFF;
	msg->data[3] = 4;
	copy_pcc_read_buffer(0, 1, &msg->data[4], 4);

	ipmb_error status = ipmb_read(msg, IPMB_inf_index_map[msg->InF_target]);
	SAFE_FREE(msg);
	if (status != IPMB_ERROR_SUCCESS) {
		LOG_ERR("Failed to send 4-byte post code to BMC, status %d.", status);
	}
}

K_WORK_DEFINE(send_post_code_work, send_post_code_to_bmc);

void pcc_rx_callback(const uint8_t *rb, uint32_t rb_sz, uint32_t st_idx, uint32_t ed_idx)
{
	/* The sequence of read data from pcc driver is:
	 * data[7:0], addr[3:0], data[7:0], addr[3:0], ......
	 * For example, we get data in the following order
	 * 0x12, 0x40, 0x34, 0x41, 0x56, 0x42, 0x78, 0x43, ......
	 * 0x40, 0x41, 0x42, 0x43 means the data comes from port 0x80, 0x81, 0x82, 0x83,
	 * and the 4-byte post code is 0x78563412
	 */
	static uint32_t four_byte_data = 0;
	uint8_t data, addr;
	uint32_t i = st_idx;
	proc_4byte_postcode_ok = true;

	do {
		data = rb[i];
		addr = rb[i + 1];
		four_byte_data |= data << (8 * (addr & 0x0F));
		if ((addr & 0x0F) == 0x03) {
			pcc_read_buffer[pcc_read_index] = four_byte_data;
			four_byte_data = 0;
			if (pcc_read_len < PCC_BUFFER_LEN) {
				pcc_read_len++;
			}
			pcc_read_index++;
			if (pcc_read_index == PCC_BUFFER_LEN) {
				pcc_read_index = 0;
			}
		}
		i = (i + 2) % rb_sz;
	} while (i != ed_idx);

	/* Send 4-byte post code to BMC */
	k_work_submit(&send_post_code_work);
}

void pcc_init()
{
	pcc_dev = device_get_binding(DT_LABEL(DT_NODELABEL(pcc)));
	if (!pcc_dev) {
		LOG_ERR("No pcc device found.");
		return;
	}
	/* set registers to enable pcc */
	uint32_t reg_data;
	sys_write32(0x00820080, 0x7E789090);

	reg_data = sys_read32(0x7E789100);
	sys_write32(reg_data | 0x0000C000, 0x7E789100);

	reg_data = sys_read32(0x7E789084);
	sys_write32((reg_data | 0x00080000) & ~0x0001ffff, 0x7E789084);

	if (pcc_aspeed_register_rx_callback(pcc_dev, pcc_rx_callback)) {
		LOG_ERR("Cannot register PCC RX callback.");
	}
	return;
}

void reset_pcc_buffer()
{
	pcc_read_len = 0;
	return;
}

bool get_4byte_postcode_ok()
{
	return proc_4byte_postcode_ok;
}

void reset_4byte_postcode_ok()
{
	proc_4byte_postcode_ok = false;
}

#endif
