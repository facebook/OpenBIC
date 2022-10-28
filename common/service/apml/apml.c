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
#include "hal_i2c.h"
#include "apml.h"
#include "power_status.h"

#define RETRY_MAX 3
#define MAILBOX_COMPLETE_RETRY_MAX 200
#define APML_RESP_BUFFER_SIZE 10
#define APML_HANDLER_STACK_SIZE 1024
#define APML_MSGQ_LEN 32
#define WAIT_TIME_MS 10

struct k_msgq apml_msgq;
struct k_thread apml_thread;
char __aligned(4) apml_msgq_buffer[APML_MSGQ_LEN * sizeof(apml_msg)];
K_THREAD_STACK_DEFINE(apml_handler_stack, APML_HANDLER_STACK_SIZE);
apml_buffer apml_resp_buffer[APML_RESP_BUFFER_SIZE];

uint8_t apml_read_byte(uint8_t bus, uint8_t addr, uint8_t offset, uint8_t *read_data)
{
	if (read_data == NULL) {
		return APML_ERROR;
	}
	uint8_t retry = 5;
	I2C_MSG msg;
	msg.bus = bus;
	msg.target_addr = addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = offset;
	int ret = i2c_master_read(&msg, retry);

	if (ret) {
		return APML_ERROR;
	}
	*read_data = msg.data[0];
	return APML_SUCCESS;
}

uint8_t apml_write_byte(uint8_t bus, uint8_t addr, uint8_t offset, uint8_t write_data)
{
	uint8_t retry = 5;
	I2C_MSG msg;
	msg.bus = bus;
	msg.target_addr = addr;
	msg.tx_len = 2;
	msg.rx_len = 0;
	msg.data[0] = offset;
	msg.data[1] = write_data;

	if (i2c_master_write(&msg, retry)) {
		return APML_ERROR;
	}
	return APML_SUCCESS;
}

static bool wait_HwAlert_set(apml_msg *msg, uint8_t retry)
{
	if (msg == NULL) {
		return false;
	}
	uint8_t read_data = 0;
	for (uint8_t i = 0; i < retry; i++) {
		if (!apml_read_byte(msg->bus, msg->target_addr, SBRMI_STATUS, &read_data)) {
			if (read_data & 0x80) {
				return true;
			}
		}
		k_msleep(WAIT_TIME_MS);
	}
	return false;
}
/****************** MCA *********************/

static uint8_t write_MCA_request(apml_msg *msg)
{
	if (msg == NULL) {
		return APML_ERROR;
	}
	uint8_t retry = 5;
	I2C_MSG i2c_msg;
	i2c_msg.bus = msg->bus;
	i2c_msg.target_addr = msg->target_addr;
	i2c_msg.tx_len = 9;
	i2c_msg.data[0] = 0x73;
	i2c_msg.data[1] = 0x07;
	i2c_msg.data[2] = 0x08;
	i2c_msg.data[3] = 0x86;
	memcpy(&i2c_msg.data[4], msg->WrData, sizeof(mca_WrData));

	if (i2c_master_write(&i2c_msg, retry)) {
		return APML_ERROR;
	}
	return APML_SUCCESS;
}

static uint8_t read_MCA_response(apml_msg *msg)
{
	if (msg == NULL) {
		return APML_ERROR;
	}
	uint8_t retry = 5;
	I2C_MSG i2c_msg;
	i2c_msg.bus = msg->bus;
	i2c_msg.target_addr = msg->target_addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 10;
	i2c_msg.data[0] = 0x73;
	if (i2c_master_read(&i2c_msg, retry)) {
		return APML_ERROR;
	}

	memcpy(msg->RdData, &i2c_msg.data[1], sizeof(mca_RdData));
	return APML_SUCCESS;
}

static uint8_t access_MCA(apml_msg *msg)
{
	if (msg == NULL) {
		return APML_ERROR;
	}
	if (write_MCA_request(msg)) {
		printf("[%s] write MCA request failed.\n", __func__);
		return APML_ERROR;
	}

	if (!wait_HwAlert_set(msg, RETRY_MAX)) {
		printf("[%s] HwAlert not be set, retry %d times.\n", __func__, RETRY_MAX);
	}

	if (read_MCA_response(msg)) {
		printf("[%s] read response failed.\n", __func__);
		return APML_ERROR;
	}

	if (apml_write_byte(msg->bus, msg->target_addr, SBRMI_STATUS, 0x80)) {
		printf("[%s] clear HwAlert failed.\n", __func__);
		return APML_ERROR;
	}
	return APML_SUCCESS;
}

/****************** CPUID *******************/

static uint8_t write_CPUID_request(apml_msg *msg)
{
	if (msg == NULL) {
		return APML_ERROR;
	}
	uint8_t retry = 5;
	I2C_MSG i2c_msg;
	i2c_msg.bus = msg->bus;
	i2c_msg.target_addr = msg->target_addr;
	i2c_msg.tx_len = 10;
	i2c_msg.data[0] = 0x73;
	i2c_msg.data[1] = 0x08;
	i2c_msg.data[2] = 0x08;
	i2c_msg.data[3] = 0x91;
	memcpy(&i2c_msg.data[4], msg->WrData, sizeof(cpuid_WrData));

	if (i2c_master_write(&i2c_msg, retry)) {
		return APML_ERROR;
	}
	return APML_SUCCESS;
}

static uint8_t read_CPUID_response(apml_msg *msg)
{
	if (msg == NULL) {
		return APML_ERROR;
	}
	uint8_t retry = 5;
	I2C_MSG i2c_msg;
	i2c_msg.bus = msg->bus;
	i2c_msg.target_addr = msg->target_addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 10;
	i2c_msg.data[0] = 0x73;
	if (i2c_master_read(&i2c_msg, retry)) {
		return APML_ERROR;
	}

	memcpy(msg->RdData, &i2c_msg.data[1], sizeof(cpuid_RdData));
	return APML_SUCCESS;
}

static uint8_t access_CPUID(apml_msg *msg)
{
	if (msg == NULL) {
		return APML_ERROR;
	}
	if (write_CPUID_request(msg)) {
		printf("[%s] write CPUID request failed.\n", __func__);
		return APML_ERROR;
	}

	if (!wait_HwAlert_set(msg, RETRY_MAX)) {
		printf("[%s] HwAlert not be set, retry %d times.\n", __func__, RETRY_MAX);
	}

	if (read_CPUID_response(msg)) {
		printf("[%s] read CPUID response failed.\n", __func__);
		return APML_ERROR;
	}

	if (apml_write_byte(msg->bus, msg->target_addr, SBRMI_STATUS, 0x80)) {
		printf("[%s] clear HwAlert failed.\n", __func__);
		return APML_ERROR;
	}
	return APML_SUCCESS;
}

/****************** RMI Mailbox**************/

static bool check_mailbox_command_complete(apml_msg *msg, uint8_t retry)
{
	if (msg == NULL) {
		return false;
	}
	uint8_t read_data = 0;
	for (uint8_t i = 0; i < retry; i++) {
		if (!apml_read_byte(msg->bus, msg->target_addr, SBRMI_SOFTWARE_INTERRUPT,
				    &read_data)) {
			if (!(read_data & 0x01)) {
				return true;
			}
		}
		k_msleep(WAIT_TIME_MS);
	}
	return false;
}

static uint8_t write_mailbox_request(apml_msg *msg)
{
	if (msg == NULL) {
		return APML_ERROR;
	}
	/* indicates command be serviced by firmware */
	uint8_t read_data;
	if (apml_read_byte(msg->bus, msg->target_addr, SBRMI_INBANDMSG_INST7, &read_data)) {
		return APML_ERROR;
	}
	if (!(read_data & 0x80)) {
		if (apml_write_byte(msg->bus, msg->target_addr, SBRMI_INBANDMSG_INST7, 0x80)) {
			return APML_ERROR;
		}
	}

	/* write command and data */
	mailbox_WrData *wr_data = (mailbox_WrData *)msg->WrData;
	if (apml_write_byte(msg->bus, msg->target_addr, SBRMI_INBANDMSG_INST0, wr_data->command)) {
		return APML_ERROR;
	}
	for (uint8_t offset = SBRMI_INBANDMSG_INST1, i = 0; offset <= SBRMI_INBANDMSG_INST4;
	     offset++, i++) {
		if (apml_write_byte(msg->bus, msg->target_addr, offset, wr_data->data_in[i])) {
			return APML_ERROR;
		}
	}

	/* notify to execute requested command */
	if (apml_write_byte(msg->bus, msg->target_addr, SBRMI_SOFTWARE_INTERRUPT, 0x01)) {
		return APML_ERROR;
	}

	return APML_SUCCESS;
}

static uint8_t read_mailbox_response(apml_msg *msg)
{
	if (msg == NULL) {
		return APML_ERROR;
	}
	mailbox_RdData *rd_data = (mailbox_RdData *)msg->RdData;
	if (apml_read_byte(msg->bus, msg->target_addr, SBRMI_OUTBANDMSG_INST0, &rd_data->command)) {
		return APML_ERROR;
	}
	for (uint8_t offset = SBRMI_OUTBANDMSG_INST1, i = 0; offset <= SBRMI_OUTBANDMSG_INST4;
	     offset++, i++) {
		if (apml_read_byte(msg->bus, msg->target_addr, offset, &rd_data->data_out[i])) {
			return APML_ERROR;
		}
	}
	if (apml_read_byte(msg->bus, msg->target_addr, SBRMI_OUTBANDMSG_INST7,
			   &rd_data->error_code)) {
		return APML_ERROR;
	}
	return APML_SUCCESS;
}

static uint8_t access_RMI_mailbox(apml_msg *msg)
{
	if (msg == NULL) {
		return APML_ERROR;
	}
	int i = 0;

	if (!check_mailbox_command_complete(msg, RETRY_MAX)) {
		printf("[%s] previous command not complete.\n", __func__);
		return APML_ERROR;
	}

	if (write_mailbox_request(msg)) {
		printf("[%s] write request failed.\n", __func__);
		return APML_ERROR;
	}

	/* wait for SwAlertSts to be set */
	uint8_t status;
	for (i = 0; i < MAILBOX_COMPLETE_RETRY_MAX; i++) {
		if (apml_read_byte(msg->bus, msg->target_addr, SBRMI_STATUS, &status)) {
			printf("[%s] read SwAlertSts failed.\n", __func__);
			return APML_ERROR;
		}
		if (status & 0x02) {
			break;
		}
		k_msleep(WAIT_TIME_MS);
		if (!get_post_status()) {
			return APML_ERROR;
		}
	}
	if (i == MAILBOX_COMPLETE_RETRY_MAX) {
		printf("[%s] SwAlertSts not be set, retry %d times.\n", __func__,
		       MAILBOX_COMPLETE_RETRY_MAX);
		return APML_ERROR;
	}

	if (read_mailbox_response(msg)) {
		printf("[%s] read mailbox response failed.\n", __func__);
		return APML_ERROR;
	}

	/* clear SwAlertSts */
	if (apml_write_byte(msg->bus, msg->target_addr, SBRMI_STATUS, 0x02)) {
		printf("[%s] clear SwAlertSts failed.\n", __func__);
		return APML_ERROR;
	}
	return APML_SUCCESS;
}

void apml_request_callback(apml_msg *msg)
{
	static uint8_t i = 0;
	apml_resp_buffer[i].index = msg->ui32_arg & 0xFF;
	memcpy(&apml_resp_buffer[i].msg, msg, sizeof(apml_msg));
	i++;
	if (i >= APML_RESP_BUFFER_SIZE) {
		i = 0;
	}
}

uint8_t get_apml_response_by_index(apml_msg *msg, uint8_t index)
{
	if (msg == NULL) {
		return APML_ERROR;
	}
	for (int i = 0; i < APML_RESP_BUFFER_SIZE; i++) {
		if (apml_resp_buffer[i].index == index) {
			memcpy(msg, &apml_resp_buffer[i].msg, sizeof(apml_msg));
			return APML_SUCCESS;
		}
	}
	return APML_ERROR;
}

uint8_t apml_read(apml_msg *msg)
{
	if (msg == NULL) {
		printf("[%s] msg is NULL.\n", __func__);
		return APML_ERROR;
	}

	if (k_msgq_put(&apml_msgq, msg, K_NO_WAIT)) {
		printf("[%s] put msg to apml_msgq failed.\n", __func__);
		return APML_ERROR;
	}
	return APML_SUCCESS;
}

static void apml_handler(void *arvg0, void *arvg1, void *arvg2)
{
	uint8_t ret;
	apml_msg msg_data;
	while (1) {
		ret = APML_ERROR;
		memset(&msg_data, 0, sizeof(apml_msg));
		k_msgq_get(&apml_msgq, &msg_data, K_FOREVER);

		if (get_post_status() == false) {
			if (msg_data.error_cb_fn) {
				msg_data.error_cb_fn(&msg_data);
			}
			k_msleep(10);
			continue;
		}

		switch (msg_data.msg_type) {
		case APML_MSG_TYPE_MAILBOX:
			ret = access_RMI_mailbox(&msg_data);
			break;
		case APML_MSG_TYPE_CPUID:
			ret = access_CPUID(&msg_data);
			break;
		case APML_MSG_TYPE_MCA:
			ret = access_MCA(&msg_data);
			break;
		default:
			break;
		}

		if (ret) {
			printf("[%s] APML access failed, msg type %d.\n", __func__,
			       msg_data.msg_type);
			if (msg_data.error_cb_fn) {
				msg_data.error_cb_fn(&msg_data);
			}
		} else {
			if (msg_data.cb_fn) {
				msg_data.cb_fn(&msg_data);
			}
		}
		k_msleep(10);
	}
}

void apml_init()
{
	printf("apml_init\n");
	k_msgq_init(&apml_msgq, apml_msgq_buffer, sizeof(apml_msg), APML_MSGQ_LEN);

	k_thread_create(&apml_thread, apml_handler_stack, K_THREAD_STACK_SIZEOF(apml_handler_stack),
			apml_handler, NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&apml_thread, "APML_handler");
}
