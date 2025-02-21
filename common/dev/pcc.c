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
#include "libipmi.h"
#include "plat_sensor_table.h"
#include "plat_fru.h"
#include "util_sys.h"

#ifdef ENABLE_PLDM
#include "pldm_oem.h"
#include "plat_mctp.h"
#endif

#define PSB_POSTCODE_PREFIX 0xEE00
#define ABL_POSTCODE_PREFIX 0xEA00

#define AST1030_LPC_BASE_ADDR 0x7E789000
#define LPC_HICR6_REG (AST1030_LPC_BASE_ADDR + 0x84)
#define LPC_SNPWADR_REG (AST1030_LPC_BASE_ADDR + 0x90)
#define LPC_HICRB_REG (AST1030_LPC_BASE_ADDR + 0x100)
#define LPC_PCCR0_REG (AST1030_LPC_BASE_ADDR + 0x130)

#define PCCR0_EN_DMA_MODE BIT(14)
#define PCCR0_EN BIT(0)
#define POST_CODE_SIZE 4

LOG_MODULE_REGISTER(pcc);

K_THREAD_STACK_DEFINE(process_postcode_thread, PROCESS_POSTCODE_STACK_SIZE);
static struct k_thread process_postcode_thread_handler;

const struct device *pcc_dev;
static uint32_t pcc_read_buffer[PCC_BUFFER_LEN];
static uint16_t pcc_read_len = 0, pcc_read_index = 0;
static bool proc_4byte_postcode_ok = false;
static struct k_sem get_postcode_sem;

static uint8_t PSB_error_code_list[] = { 0x03, 0x04, 0x05, 0x0B, 0x10, 0x13, 0x14, 0x18, 0x22, 0x3E,
					 0x62, 0x64, 0x69, 0x6C, 0x6F, 0x78, 0x79, 0x7A, 0x7B, 0x7C,
					 0x7D, 0x7E, 0x7F, 0x80, 0x81, 0x82, 0x83, 0x92 };

static uint16_t ABL_error_code_list[] = { 0x3000, 0x3001, 0x3002, 0x3003, 0xE2A4, 0xE2A7, 0xE2A8,
					  0xE2AA, 0xE2AB, 0xE2AC, 0xE2AE, 0xE2B1, 0xE2B2, 0xE2E4,
					  0xE2B3, 0xE2B4, 0xE2B5, 0xE2B9, 0xE2BA, 0xE2BB, 0xE2BD,
					  0xE2BF, 0xE2CB, 0xE2CC, 0xE2CD, 0xE320, 0xE321, 0xE322,
					  0xE2E5, 0xE2EB, 0xE2EC, 0xE2AD, 0xE2ED, 0xE30E, 0xE2CA,
					  0xE2EF, 0xE2C2, 0xE2C3, 0xE2E3, 0xE2C6, 0xE310, 0xE2E7,
					  0xE32D, 0xE33E, 0xE328, 0xE345, 0xE32B, 0xE332 };

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

void check_PSB_error(uint32_t postcode)
{
	uint8_t error_code = postcode & 0xFF;
	int i = 0;
	for (; i < ARRAY_SIZE(PSB_error_code_list); i++) {
		if (error_code == PSB_error_code_list[i]) {
			break;
		}
	}
	if (i == ARRAY_SIZE(PSB_error_code_list)) {
		return;
	}

#ifndef ENABLE_PLDM
	/* Match PSB error */
	/* Add SEL */
	common_addsel_msg_t sel_msg;
	sel_msg.InF_target = BMC_IPMB;
	sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_PSB_ERROR;
	sel_msg.sensor_number = SENSOR_NUM_PSB_BOOT_ERROR;
	sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
	sel_msg.event_data1 = 0xEE;
	sel_msg.event_data2 = error_code;
	sel_msg.event_data3 = 0xFF;
	if (!common_add_sel_evt_record(&sel_msg)) {
		LOG_ERR("Failed to assert PSB event log, post code 0x%08x.", postcode);
	}

	/* write PSB error code to CPU EEPROM */
	EEPROM_ENTRY psb_inform = { 0 };
	psb_inform.data_len = PSB_ERROR_MAX_SIZE;
	memset(&psb_inform.data, 0xFF, EEPROM_WRITE_SIZE);
	psb_inform.data[4] = error_code;
	uint8_t checksum = 0;
	for (i = 0; i < (PSB_ERROR_MAX_SIZE - 1); i++) {
		checksum -= psb_inform.data[i];
	}
	psb_inform.data[PSB_ERROR_MAX_SIZE - 1] = checksum;

	if (!write_psb_inform(&psb_inform)) {
		LOG_ERR("Failed to write PSB to EEPROM, post code 0x%08x.", postcode);
	}
#endif
}

void check_ABL_error(uint32_t postcode)
{
	uint16_t error_code = postcode & 0xFFFF;
	int i = 0;
	for (; i < ARRAY_SIZE(ABL_error_code_list); i++) {
		if (error_code == ABL_error_code_list[i]) {
			break;
		}
	}
	if (i == ARRAY_SIZE(ABL_error_code_list)) {
		return;
	}
#ifndef ENABLE_PLDM
	ipmb_error status;
	ipmi_msg *msg = (ipmi_msg *)malloc(sizeof(ipmi_msg));
	if (msg == NULL) {
		LOG_ERR("Memory allocation failed.");
		return;
	}
	memset(msg, 0, sizeof(ipmi_msg));

	// record Unified SEL
	msg->data_len = 16;
	msg->InF_source = SELF;
	msg->InF_target = BMC_IPMB;
	msg->netfn = NETFN_STORAGE_REQ;
	msg->cmd = CMD_STORAGE_ADD_SEL;
	msg->data[0] = 0x00; // Record id byte 0, lsb
	msg->data[1] = 0x00; // Record id byte 1
	msg->data[2] = 0xFB; // Record Type: Unified SEL
	msg->data[4] = 0x00; // Timestamp
	msg->data[5] = 0x00; // Timestamp
	msg->data[6] = 0x00; // Timestamp
	msg->data[7] = 0x00; // Timestamp
	msg->data[9] = 0xFF; // DIMM Channel
	msg->data[10] = 0xFF; // DIMM Slot
	msg->data[11] = 0xFF; // Reserved

	if (error_code == 0xE310) {
		msg->data[3] = 0x2A; // General Information
		msg->data[8] = 0xFF; // DIMM Socket
		msg->data[12] = 0x07; // DIMM Error Type
		msg->data[13] = 0xFF; // Major code
		msg->data[14] = 0xFF; // Minor code
		msg->data[15] = 0xFF; // Minor code
	} else {
		msg->data[3] = 0x28; // General Information
		msg->data[8] = 0xFA; // Failure Event Type
		msg->data[12] = 0xFF; // Failure Event Detail byte 3
		msg->data[13] = 0xFF; // Failure Event Detail byte 4
		msg->data[14] = error_code & 0xFF; // Failure Code byte 0
		msg->data[15] = (error_code >> 8) & 0xFF; // Failure Code byte 1
	}

	status = ipmb_read(msg, IPMB_inf_index_map[msg->InF_target]);
	if (status != IPMB_ERROR_SUCCESS) {
		LOG_ERR("Failed to record ABL SEL, post code 0x%08x, ret %d", postcode, status);
	}
	SAFE_FREE(msg);
#else
	// record Unified SEL
	uint8_t bmc_bus = I2C_BUS_BMC, bmc_interface = BMC_INTERFACE_I2C;;
	mctp_ext_params ext_params = { 0 };
	uint8_t data[16] = { 0 };

	data[0] = 0x00; // Record id byte 0, lsb
	data[1] = 0x00; // Record id byte 1
	data[2] = 0xFB; // Record Type: Unified SEL
	data[4] = 0x00; // Timestamp
	data[5] = 0x00; // Timestamp
	data[6] = 0x00; // Timestamp
	data[7] = 0x00; // Timestamp
	data[9] = 0xFF; // DIMM Channel
	data[10] = 0xFF; // DIMM Slot
	data[11] = 0xFF; // Reserved

	if (error_code == 0xE310) {
		data[3] = 0x2A; // General Information
		data[8] = 0xFF; // DIMM Socket
		data[12] = 0x07; // DIMM Error Type
		data[13] = 0xFF; // Major code
		data[14] = 0xFF; // Minor code
		data[15] = 0xFF; // Minor code
	} else {
		data[3] = 0x28; // General Information
		data[8] = 0xFA; // Failure Event Type
		data[12] = 0xFF; // Failure Event Detail byte 3
		data[13] = 0xFF; // Failure Event Detail byte 4
		data[14] = error_code & 0xFF; // Failure Code byte 0
		data[15] = (error_code >> 8) & 0xFF; // Failure Code byte 1
	}

	bmc_interface = pal_get_bmc_interface();
	if (bmc_interface == BMC_INTERFACE_I3C) {
		bmc_bus = I3C_BUS_BMC;
		ext_params.type = MCTP_MEDIUM_TYPE_TARGET_I3C;
		ext_params.i3c_ext_params.addr = I3C_STATIC_ADDR_BMC;
		ext_params.ep = MCTP_EID_BMC;
	} else {
		bmc_bus = I2C_BUS_BMC;
		ext_params.type = MCTP_MEDIUM_TYPE_SMBUS;
		ext_params.smbus_ext_params.addr = I2C_ADDR_BMC;
		ext_params.ep = MCTP_EID_BMC;
	}

	pldm_platform_event_message_req(find_mctp_by_bus(bmc_bus), ext_params, 0xFB, &data[2], 14);
#endif
}

#ifdef ENABLE_PLDM
bool pldm_send_post_code_to_bmc(uint16_t send_index)
{
	pldm_msg msg = { 0 };
	uint8_t bmc_bus = I2C_BUS_BMC, bmc_interface = BMC_INTERFACE_I2C;

	bmc_interface = pal_get_bmc_interface();
	if (bmc_interface == BMC_INTERFACE_I3C) {
		bmc_bus = I3C_BUS_BMC;
		msg.ext_params.type = MCTP_MEDIUM_TYPE_TARGET_I3C;
		msg.ext_params.i3c_ext_params.addr = I3C_STATIC_ADDR_BMC;
		msg.ext_params.ep = MCTP_EID_BMC;
	} else {
		bmc_bus = I2C_BUS_BMC;
		msg.ext_params.type = MCTP_MEDIUM_TYPE_SMBUS;
		msg.ext_params.smbus_ext_params.addr = I2C_ADDR_BMC;
		msg.ext_params.ep = MCTP_EID_BMC;
	}

	msg.hdr.pldm_type = PLDM_TYPE_OEM;
	msg.hdr.cmd = PLDM_OEM_WRITE_FILE_IO;
	msg.hdr.rq = 1;

	struct pldm_oem_write_file_io_req *ptr = (struct pldm_oem_write_file_io_req *)malloc(
		sizeof(struct pldm_oem_write_file_io_req) + (POST_CODE_SIZE * sizeof(uint8_t)));

	if (ptr == NULL) {
		LOG_ERR("Memory allocation failed.");
		return false;
	}

	ptr->cmd_code = POST_CODE;
	ptr->data_length = POST_CODE_SIZE;
	ptr->messages[0] = pcc_read_buffer[send_index] & 0xFF;
	ptr->messages[1] = (pcc_read_buffer[send_index] >> 8) & 0xFF;
	ptr->messages[2] = (pcc_read_buffer[send_index] >> 16) & 0xFF;
	ptr->messages[3] = (pcc_read_buffer[send_index] >> 24) & 0xFF;

	msg.buf = (uint8_t *)ptr;
	msg.len = sizeof(struct pldm_oem_write_file_io_req) + POST_CODE_SIZE;

	uint8_t resp_len = sizeof(struct pldm_oem_write_file_io_resp);
	uint8_t rbuf[resp_len];

	if (!mctp_pldm_read(find_mctp_by_bus(bmc_bus), &msg, rbuf, resp_len)) {
		SAFE_FREE(ptr);
		LOG_ERR("mctp_pldm_read fail");
		return false;
	}

	struct pldm_oem_write_file_io_resp *resp = (struct pldm_oem_write_file_io_resp *)rbuf;
	if (resp->completion_code != PLDM_SUCCESS) {
		LOG_ERR("Check reponse completion code fail %x", resp->completion_code);
	}

	SAFE_FREE(ptr);
	return true;
}
#endif

bool ipmi_send_post_code_to_bmc(uint16_t send_index)
{
	ipmi_msg *msg = (ipmi_msg *)malloc(sizeof(ipmi_msg));
	if (msg == NULL) {
		LOG_ERR("Memory allocation failed.");
		return false;
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
	msg->data[3] = POST_CODE_SIZE;
	msg->data[4] = pcc_read_buffer[send_index] & 0xFF;
	msg->data[5] = (pcc_read_buffer[send_index] >> 8) & 0xFF;
	msg->data[6] = (pcc_read_buffer[send_index] >> 16) & 0xFF;
	msg->data[7] = (pcc_read_buffer[send_index] >> 24) & 0xFF;
	ipmb_error status = ipmb_read(msg, IPMB_inf_index_map[msg->InF_target]);
	if (status != IPMB_ERROR_SUCCESS) {
		SAFE_FREE(msg);
		LOG_ERR("Failed to send 4-byte post code to BMC, status %d.", status);
		return false;
	}
	SAFE_FREE(msg)
	return true;
}

bool send_post_code_to_bmc(uint16_t send_index)
{
#ifdef ENABLE_PLDM
	return pldm_send_post_code_to_bmc(send_index);
#else
	return ipmi_send_post_code_to_bmc(send_index);
#endif
}

static void process_postcode(void *arvg0, void *arvg1, void *arvg2)
{
	uint16_t send_index = 0;
	while (1) {
		k_sem_take(&get_postcode_sem, K_FOREVER);
		uint16_t current_read_index = pcc_read_index;
		for (; send_index != current_read_index;
		     send_index = (send_index + 1) % PCC_BUFFER_LEN) {
			if (((pcc_read_buffer[send_index] >> 16) & BIT_MASK(16)) ==
			    PSB_POSTCODE_PREFIX) {
				check_PSB_error(pcc_read_buffer[send_index]);
			} else if (((pcc_read_buffer[send_index] >> 16) & BIT_MASK(16)) ==
				   ABL_POSTCODE_PREFIX) {
				check_ABL_error(pcc_read_buffer[send_index]);
			}

			send_post_code_to_bmc(send_index);

			k_yield();
		}
	}
}

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
	k_sem_give(&get_postcode_sem);
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

	reg_data = sys_read32(LPC_PCCR0_REG);
	sys_write32(reg_data & ~(PCCR0_EN_DMA_MODE | PCCR0_EN), LPC_PCCR0_REG);
	sys_write32(reg_data | (PCCR0_EN_DMA_MODE | PCCR0_EN), LPC_PCCR0_REG);

	sys_write32(0x00820080, LPC_SNPWADR_REG);

	reg_data = sys_read32(LPC_HICRB_REG);
	sys_write32(reg_data | 0x0000C000, LPC_HICRB_REG);

	reg_data = sys_read32(LPC_HICR6_REG);
	sys_write32((reg_data | 0x00080000) & ~0x0001ffff, LPC_HICR6_REG);

	k_sem_init(&get_postcode_sem, 0, 1);

	if (pcc_aspeed_register_rx_callback(pcc_dev, pcc_rx_callback)) {
		LOG_ERR("Cannot register PCC RX callback.");
	}

	k_thread_create(&process_postcode_thread_handler, process_postcode_thread,
			K_THREAD_STACK_SIZEOF(process_postcode_thread), process_postcode, NULL,
			NULL, NULL, K_PRIO_PREEMPT(1), 0, K_NO_WAIT);
	k_thread_name_set(&process_postcode_thread_handler, "process_postcode_thread");

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
