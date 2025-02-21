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
#include <logging/log.h>
#include "libutil.h"
#include "plat_def.h"

#ifdef ENABLE_APML
#include "plat_apml.h"

LOG_MODULE_REGISTER(apml);

#define RETRY_MAX 3
#define CPUID_MCA_WAIT_MAX 10
#define MAILBOX_COMPLETE_RETRY_MAX 200
#define APML_RESP_BUFFER_SIZE 10
#define APML_HANDLER_STACK_SIZE 2048
#define APML_MSGQ_LEN 32
#define WAIT_TIME_MS 10
#define WAIT_CPUID_MCA_RESP_TIME_MS 50
#define RECOVERY_SBRMI_RETRY_MAX 5

struct k_msgq apml_msgq;
struct k_thread apml_thread;
char __aligned(4) apml_msgq_buffer[APML_MSGQ_LEN * sizeof(apml_msg)];
K_THREAD_STACK_DEFINE(apml_handler_stack, APML_HANDLER_STACK_SIZE);
apml_buffer apml_resp_buffer[APML_RESP_BUFFER_SIZE];
static bool is_fatal_error_happened;
static int command_code_len = SBRMI_CMD_CODE_LEN_DEFAULT;
static int apml_bus = APML_BUS_UNKNOWN;

uint8_t apml_read_byte(uint8_t bus, uint8_t addr, uint8_t offset, uint8_t *read_data)
{
	CHECK_NULL_ARG_WITH_RETURN(read_data, APML_ERROR);
	uint8_t retry = 5;
	I2C_MSG msg;
	msg.bus = bus;
	msg.target_addr = addr;
	msg.rx_len = 1;

	if ((command_code_len == SBRMI_CMD_CODE_LEN_TWO_BYTE) && (addr == SB_RMI_ADDR)) {
		msg.tx_len = 2;
		msg.data[0] = offset;
		msg.data[1] = 0x00;
	} else {
		msg.tx_len = 1;
		msg.data[0] = offset;
	}

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
	msg.rx_len = 0;

	if ((command_code_len == SBRMI_CMD_CODE_LEN_TWO_BYTE) && (addr == SB_RMI_ADDR)) {
		msg.tx_len = 3;
		msg.data[0] = offset;
		msg.data[1] = 0x00;
		msg.data[2] = write_data;
	} else {
		msg.tx_len = 2;
		msg.data[0] = offset;
		msg.data[1] = write_data;
	}

	if (i2c_master_write(&msg, retry)) {
		return APML_ERROR;
	}
	return APML_SUCCESS;
}

static bool wait_HwAlert_set(apml_msg *msg, uint8_t retry)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);
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
	CHECK_NULL_ARG_WITH_RETURN(msg, APML_ERROR);
	uint8_t retry = 5;
	I2C_MSG i2c_msg;
	i2c_msg.bus = msg->bus;
	i2c_msg.target_addr = msg->target_addr;

	if ((command_code_len == SBRMI_CMD_CODE_LEN_TWO_BYTE) &&
	    (msg->target_addr == SB_RMI_ADDR)) {
		i2c_msg.tx_len = 11;
		i2c_msg.data[0] = 0x73;
		i2c_msg.data[1] = 0x00;
		i2c_msg.data[2] = 0x08;
		i2c_msg.data[3] = 0x08;
		i2c_msg.data[4] = 0x86;
		memcpy(&i2c_msg.data[5], msg->WrData, sizeof(mca_WrData_TwoPOne));
	} else {
		i2c_msg.tx_len = 9;
		i2c_msg.data[0] = 0x73;
		i2c_msg.data[1] = 0x07;
		i2c_msg.data[2] = 0x08;
		i2c_msg.data[3] = 0x86;
		memcpy(&i2c_msg.data[4], msg->WrData, sizeof(mca_WrData));
	}

	if (i2c_master_write(&i2c_msg, retry)) {
		return APML_ERROR;
	}

	return APML_SUCCESS;
}

static uint8_t read_MCA_response(apml_msg *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, APML_ERROR);
	uint8_t retry = 5;
	I2C_MSG i2c_msg;
	i2c_msg.bus = msg->bus;
	i2c_msg.target_addr = msg->target_addr;
	i2c_msg.rx_len = 10;

	if ((command_code_len == SBRMI_CMD_CODE_LEN_TWO_BYTE) &&
	    (msg->target_addr == SB_RMI_ADDR)) {
		i2c_msg.tx_len = 2;
		i2c_msg.data[0] = 0x73;
		i2c_msg.data[1] = 0x00;
	} else {
		i2c_msg.tx_len = 1;
		i2c_msg.data[0] = 0x73;
	}

	if (i2c_master_read(&i2c_msg, retry)) {
		return APML_ERROR;
	}

	memcpy(msg->RdData, &i2c_msg.data[1], sizeof(mca_RdData));
	return APML_SUCCESS;
}

static uint8_t access_MCA(apml_msg *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, APML_ERROR);
	if (write_MCA_request(msg)) {
		LOG_ERR("Write MCA request failed.");
		return APML_ERROR;
	}

	if (!wait_HwAlert_set(msg, CPUID_MCA_WAIT_MAX)) {
		LOG_ERR("HwAlert not be set, retry %d times.", CPUID_MCA_WAIT_MAX);
		return APML_ERROR;
	}

	/* wait for CPU to fill out registers */
	k_msleep(WAIT_CPUID_MCA_RESP_TIME_MS);
	if (read_MCA_response(msg)) {
		LOG_ERR("Read response failed.");
		return APML_ERROR;
	}

	if (apml_write_byte(msg->bus, msg->target_addr, SBRMI_STATUS, 0x80)) {
		LOG_ERR("Clear HwAlert failed.");
		return APML_ERROR;
	}
	return APML_SUCCESS;
}

/****************** CPUID *******************/

static uint8_t write_CPUID_request(apml_msg *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, APML_ERROR);
	uint8_t retry = 5;
	I2C_MSG i2c_msg;
	i2c_msg.bus = msg->bus;
	i2c_msg.target_addr = msg->target_addr;

	if ((command_code_len == SBRMI_CMD_CODE_LEN_TWO_BYTE) &&
	    (msg->target_addr == SB_RMI_ADDR)) {
		i2c_msg.tx_len = 12;
		i2c_msg.data[0] = 0x73;
		i2c_msg.data[1] = 0x00;
		i2c_msg.data[2] = 0x09;
		i2c_msg.data[3] = 0x08;
		i2c_msg.data[4] = 0x91;
		memcpy(&i2c_msg.data[5], msg->WrData, sizeof(cpuid_WrData_TwoPOne));
	} else {
		i2c_msg.tx_len = 10;
		i2c_msg.data[0] = 0x73;
		i2c_msg.data[1] = 0x08;
		i2c_msg.data[2] = 0x08;
		i2c_msg.data[3] = 0x91;
		memcpy(&i2c_msg.data[4], msg->WrData, sizeof(cpuid_WrData));
	}

	if (i2c_master_write(&i2c_msg, retry)) {
		return APML_ERROR;
	}
	return APML_SUCCESS;
}

static uint8_t read_CPUID_response(apml_msg *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, APML_ERROR);
	uint8_t retry = 5;
	I2C_MSG i2c_msg;
	i2c_msg.bus = msg->bus;
	i2c_msg.target_addr = msg->target_addr;
	i2c_msg.rx_len = 10;

	if ((command_code_len == SBRMI_CMD_CODE_LEN_TWO_BYTE) &&
	    (msg->target_addr == SB_RMI_ADDR)) {
		i2c_msg.tx_len = 2;
		i2c_msg.data[0] = 0x73;
		i2c_msg.data[1] = 0x00;
	} else {
		i2c_msg.tx_len = 1;
		i2c_msg.data[0] = 0x73;
	}

	if (i2c_master_read(&i2c_msg, retry)) {
		return APML_ERROR;
	}

	memcpy(msg->RdData, &i2c_msg.data[1], sizeof(cpuid_RdData));
	return APML_SUCCESS;
}

static uint8_t access_CPUID(apml_msg *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, APML_ERROR);
	if (write_CPUID_request(msg)) {
		LOG_ERR("Write CPUID request failed.");
		return APML_ERROR;
	}

	if (!wait_HwAlert_set(msg, CPUID_MCA_WAIT_MAX)) {
		LOG_ERR("HwAlert not be set, retry %d times.", CPUID_MCA_WAIT_MAX);
		return APML_ERROR;
	}

	/* wait for CPU to fill out registers */
	k_msleep(WAIT_CPUID_MCA_RESP_TIME_MS);
	if (read_CPUID_response(msg)) {
		LOG_ERR("Read CPUID response failed.");
		return APML_ERROR;
	}

	if (apml_write_byte(msg->bus, msg->target_addr, SBRMI_STATUS, 0x80)) {
		LOG_ERR("Clear HwAlert failed.");
		return APML_ERROR;
	}
	return APML_SUCCESS;
}

/****************** RMI Mailbox**************/

static bool check_mailbox_command_complete(apml_msg *msg, uint8_t retry)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);
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
	CHECK_NULL_ARG_WITH_RETURN(msg, APML_ERROR);
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
	CHECK_NULL_ARG_WITH_RETURN(msg, APML_ERROR);
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
	CHECK_NULL_ARG_WITH_RETURN(msg, APML_ERROR);
	int i = 0;

	if (!check_mailbox_command_complete(msg, RETRY_MAX)) {
		LOG_ERR("Previous command not complete.");
		return APML_ERROR;
	}

	if (write_mailbox_request(msg)) {
		LOG_ERR("Write request failed.");
		return APML_ERROR;
	}

	is_fatal_error_happened = false;

	/* wait for SwAlertSts to be set */
	uint8_t status;
	for (i = 0; i < MAILBOX_COMPLETE_RETRY_MAX; i++) {
		/* For TURIN, wait for SoftwareInterrupt */
		if (command_code_len == SBRMI_CMD_CODE_LEN_TWO_BYTE) {
			if (apml_read_byte(msg->bus, msg->target_addr, SBRMI_SOFTWARE_INTERRUPT, &status)) {
				LOG_ERR("Read SoftwareInterrupt failed.");
				return APML_ERROR;
			}
			if ((status & 0x01) == 0) {
				break;
			}
		} else {
			if (apml_read_byte(msg->bus, msg->target_addr, SBRMI_STATUS, &status)) {
				LOG_ERR("Read SwAlertSts failed.");
				return APML_ERROR;
			}
			if (status & 0x02) {
				break;
			}
		}

		k_msleep(WAIT_TIME_MS);
		if (!get_post_status()) {
			return APML_ERROR;
		}
	}
	if (i == MAILBOX_COMPLETE_RETRY_MAX) {
		if (command_code_len == SBRMI_CMD_CODE_LEN_TWO_BYTE) {
			LOG_ERR("SoftwareInterrupt not be set, retry %d times.", MAILBOX_COMPLETE_RETRY_MAX);
		} else {
			LOG_ERR("SwAlertSts not be set, retry %d times.", MAILBOX_COMPLETE_RETRY_MAX);
		}
		return APML_ERROR;
	}

	if (is_fatal_error_happened) {
		LOG_ERR("Fatal error happened during mailbox waiting.");
		return APML_ERROR;
	}

	if (read_mailbox_response(msg)) {
		LOG_ERR("Read mailbox response failed.");
		return APML_ERROR;
	}

	/* clear SwAlertSts */
	if (command_code_len != SBRMI_CMD_CODE_LEN_TWO_BYTE) {
		if (apml_write_byte(msg->bus, msg->target_addr, SBRMI_STATUS, 0x02)) {
			LOG_ERR("Clear SwAlertSts failed.");
			return APML_ERROR;
		}
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
	CHECK_NULL_ARG_WITH_RETURN(msg, APML_ERROR);
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
	CHECK_NULL_ARG_WITH_RETURN(msg, APML_ERROR);
	if (k_msgq_put(&apml_msgq, msg, K_NO_WAIT)) {
		LOG_ERR("Put msg to apml_msgq failed.");
		return APML_ERROR;
	}
	return APML_SUCCESS;
}

__weak int pal_check_sbrmi_command_code_length()
{
	command_code_len = SBRMI_CMD_CODE_LEN_DEFAULT;
	return 0;
}

int set_sbrmi_command_code_len(uint8_t value)
{
	command_code_len = value;
	return 0;
}

int get_sbrmi_command_code_len()
{
	return command_code_len;
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
			LOG_ERR("APML access failed, msg type %d.", msg_data.msg_type);
			if (msg_data.error_cb_fn) {
				msg_data.error_cb_fn(&msg_data);
			}
			apml_recovery();
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
	LOG_DBG("apml_init");
	apml_bus = pal_get_apml_bus();
	if (apml_bus == APML_BUS_UNKNOWN) {
		LOG_ERR("Failed to set APML bus");
		return;
	}

	pal_check_sbrmi_command_code_length();

	k_msgq_init(&apml_msgq, apml_msgq_buffer, sizeof(apml_msg), APML_MSGQ_LEN);
	memset(apml_resp_buffer, 0xFF, sizeof(apml_resp_buffer));

	k_thread_create(&apml_thread, apml_handler_stack, K_THREAD_STACK_SIZEOF(apml_handler_stack),
			apml_handler, NULL, NULL, NULL, K_PRIO_PREEMPT(1), 0, K_NO_WAIT);
	k_thread_name_set(&apml_thread, "APML_handler");
}

void fatal_error_happened()
{
	is_fatal_error_happened = true;
}

void apml_recovery()
{
	uint8_t ret, read_data = 0x00;
	ret = apml_read_byte(apml_bus, SB_RMI_ADDR, SBRMI_REVISION, &read_data);

	if (ret) {
		LOG_ERR("Failed to read SBRMI revision.");
	}

	if ((ret || ((read_data != PLAT_SBRMI_REVISION) && (read_data != SBRMI_REV_BRTH))) &&
	    get_post_status()) {
		LOG_INF("Recovery SBRMI.");
		if (apml_read_byte(apml_bus, SB_TSI_ADDR, SBTSI_CONFIG, &read_data)) {
			LOG_ERR("Failed to read SBTSI config.");
			return;
		}

		if (apml_write_byte(apml_bus, SB_TSI_ADDR, SBTSI_CONFIG_WRITE,
				    read_data | RMI_SOFT_RESET_BIT)) {
			LOG_ERR("Failed to write SBTSI config.");
			return;
		}

		uint8_t i = 0;
		for (; i < RECOVERY_SBRMI_RETRY_MAX; i++) {
			read_data = 0;
			if (apml_read_byte(apml_bus, SB_TSI_ADDR, SBTSI_CONFIG, &read_data)) {
				LOG_ERR("Failed to read SBTSI config.");
				return;
			}

			if (!(read_data & RMI_SOFT_RESET_BIT)) {
				break;
			}
			k_msleep(10);
		}

		if (i == RECOVERY_SBRMI_RETRY_MAX) {
			LOG_ERR("RMISoftReset bit is not cleared.");
		}
	}
	return;
}

void disable_mailbox_completion_alert()
{
	uint8_t reg;
	/* Set MbCmplSwAlertEnable to 0 for TURIN CPU. It will allow us to poll */
	/* SoftwareInterrupt for mailbox completion instead of SwAlertSts.      */
	if (command_code_len == SBRMI_CMD_CODE_LEN_TWO_BYTE) {
		if (apml_read_byte(apml_bus, SB_RMI_ADDR, SBRMI_CONTROL, &reg)) {
			LOG_ERR("Failed to read SBRMI control.");
			return;
		} else {
			LOG_INF("SBRMI_CONTROL read");
		}
		reg &= ~(1 << 5); // write 0 to bit5 MbCmplSwAlertEnable
		if (apml_write_byte(apml_bus, SB_RMI_ADDR, SBRMI_CONTROL, reg)) {
			LOG_ERR("Failed to write SBRMI control.");
			return;
		} else {
			LOG_INF("SBRMI_CONTROL written");
		}
	}
}

void enable_alert_signal()
{
	uint8_t reg;

	if (command_code_len == SBRMI_CMD_CODE_LEN_TWO_BYTE) {
		if (apml_read_byte(apml_bus, SB_RMI_ADDR, SBRMI_CONTROL, &reg)) {
			LOG_ERR("Failed to read SBRMI control.");
			return;
		} else {
			LOG_INF("SBRMI_CONTROL read");
		}
		reg &= ~(1 << 0); // write 0 to bit0 AlertMask
		if (apml_write_byte(apml_bus, SB_RMI_ADDR, SBRMI_CONTROL, reg)) {
			LOG_ERR("Failed to write SBRMI control.");
			return;
		} else {
			LOG_INF("SBRMI_CONTROL written");
		}
	}
}

__weak uint8_t pal_get_apml_bus()
{
	return APML_BUS_UNKNOWN;
}

uint8_t apml_get_bus()
{
	return apml_bus;
}

#endif
