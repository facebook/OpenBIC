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

#ifndef APML_H
#define APML_H

#define APML_SUCCESS 0
#define APML_ERROR 1

enum APML_MSG_TYPE {
	APML_MSG_TYPE_MAILBOX,
	APML_MSG_TYPE_CPUID,
	APML_MSG_TYPE_MCA,
};

enum SBRMI_MAILBOX_CMD {
	SBRMI_MAILBOX_PKGPWR = 0x01,
	SBRMI_MAILBOX_REPORT_DIMM_POWER = 0x40,
	SBRMI_MAILBOX_REPORT_DIMM_TEMP = 0x41,
	SBRMI_MAILBOX_GET_DIMM_PWR = 0x47,
	SBRMI_MAILBOX_GET_DIMM_TEMP = 0x48,
};

enum SBTSI_REGISTER {
	SBTSI_CPU_TEMP_INT = 0x01,
	SBTSI_STATUS = 0x02,
	SBTSI_CONFIG = 0x03,
	SBTSI_HIGH_TEMP_INTEGER_THRESHOLD = 0x07,
	SBTSI_CPU_TEMP_DEC = 0x10,
};

enum SBRMI_MAILBOX_ERR_CODE {
	SBRMI_MAILBOX_NO_ERR = 0x00,
	SBRMI_MAILBOX_CMD_ABORT = 0x01,
	SBRMI_MAILBOX_UNKNOWN_CMD = 0x02,
	SBRMI_MAILBOX_INVALID_CORE = 0x03,
};

enum SBRMI_REGISTER {
	SBRMI_STATUS = 0x02,
	SBRMI_OUTBANDMSG_INST0 = 0x30,
	SBRMI_OUTBANDMSG_INST1 = 0x31,
	SBRMI_OUTBANDMSG_INST2 = 0x32,
	SBRMI_OUTBANDMSG_INST3 = 0x33,
	SBRMI_OUTBANDMSG_INST4 = 0x34,
	SBRMI_OUTBANDMSG_INST5 = 0x35,
	SBRMI_OUTBANDMSG_INST6 = 0x36,
	SBRMI_OUTBANDMSG_INST7 = 0x37,
	SBRMI_INBANDMSG_INST0 = 0x38,
	SBRMI_INBANDMSG_INST1 = 0x39,
	SBRMI_INBANDMSG_INST2 = 0x3A,
	SBRMI_INBANDMSG_INST3 = 0x3B,
	SBRMI_INBANDMSG_INST4 = 0x3C,
	SBRMI_INBANDMSG_INST5 = 0x3D,
	SBRMI_INBANDMSG_INST6 = 0x3E,
	SBRMI_INBANDMSG_INST7 = 0x3F,
	SBRMI_SOFTWARE_INTERRUPT = 0x40,
	SBRMI_RAS_STATUS = 0x4C,
};

typedef struct _mailbox_WrData_ {
	uint8_t command;
	uint8_t data_in[4];
} mailbox_WrData;

typedef struct _mailbox_RdData_ {
	uint8_t command;
	uint8_t data_out[4];
	uint8_t error_code;
} mailbox_RdData;

typedef struct _cpuid_WrData_ {
	uint8_t thread;
	uint8_t cpuid_func[4];
	uint8_t ecx_value;
} cpuid_WrData;

typedef struct _cpuid_RdData_ {
	uint8_t status;
	uint8_t data_out[8];
} cpuid_RdData;

typedef struct _mca_WrData_ {
	uint8_t thread;
	uint8_t register_addr[4];
} mca_WrData;

typedef struct _mca_RdData_ {
	uint8_t status;
	uint8_t data_out[8];
} mca_RdData;

typedef __aligned(4) struct _apml_msg_ {
	uint8_t msg_type;
	uint8_t bus;
	uint8_t target_addr;
	uint8_t WrData[7];
	uint8_t RdData[9];
	void (*cb_fn)(struct _apml_msg_ *msg);
	void (*error_cb_fn)(struct _apml_msg_ *msg);
	void *ptr_arg;
	uint32_t ui32_arg;
} __packed apml_msg;

typedef struct _apml_buffer_ {
	uint8_t index;
	apml_msg msg;
} apml_buffer;

uint8_t apml_read_byte(uint8_t bus, uint8_t addr, uint8_t offset, uint8_t *read_data);
uint8_t apml_write_byte(uint8_t bus, uint8_t addr, uint8_t offset, uint8_t write_data);
void apml_request_callback(apml_msg *msg);
uint8_t get_apml_response_by_index(apml_msg *msg, uint8_t index);
uint8_t apml_read(apml_msg *msg);
void apml_init();

#endif
