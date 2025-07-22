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

#ifndef _PLDM_OEM_H
#define _PLDM_OEM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pldm.h"
#include <stdint.h>

#define IANA_LEN 0x03
/* define for pldm oem event */
#define OEM_EVENT_LEN 0x05
#define EVENT_ASSERTED 0x01
#define EVENT_DEASSERTED 0x00

/* commands of pldm type 0x3F : PLDM_TYPE_OEM */
#define PLDM_OEM_CMD_ECHO 0x00
#define PLDM_OEM_IPMI_BRIDGE 0x01
#define PLDM_OEM_WRITE_FILE_IO 0x02
#define PLDM_OEM_READ_FILE_IO 0x03

#define POWER_CONTROL_LEN 0x01

enum cmd_type {
	POST_CODE = 0x00,
	BIOS_VERSION = 0x01,
	POWER_CONTROL = 0x02,
	HTTP_BOOT = 0X03,
	APML_ALERT = 0x04,
	EVENT_LOG = 0x05,
	CRASH_DUMP = 0x06,
	BOOT_ORDER = 0x07,
};

enum POWER_CONTROL_OPTION {
	SLED_CYCLE = 0x00,
	SLOT_12V_CYCLE = 0x01,
	SLOT_DC_CYCLE = 0x02,
	NIC0_POWER_CYCLE = 0x03,
	NIC1_POWER_CYCLE = 0x04,
	NIC2_POWER_CYCLE = 0x05,
	NIC3_POWER_CYCLE = 0x06,
	MAX_POWER_OPTION,
};

enum oem_event_type {
	CPU_THERMAL_TRIP = 0x00,
	HSC_OCP,
	P12V_STBY_UV,
	PMALERT_ASSERT,
	FAST_PROCHOT_ASSERT,
	FRB3_TIMER_EXPIRE,
	POWER_ON_SEQUENCE_FAIL,
	DIMM_PMIC_ERROR,
	ADDC_DUMP,
	BMC_COMES_OUT_COLD_RESET,
	BIOS_FRB2_WDT_EXPIRE,
	BIC_POWER_FAIL,
	CPU_POWER_FAIL,
	BMC_VBOOT_FAIL,
	BMC_REBOOT_REQUESTED,
	CHASSIS_POWER_ON_BY_NIC_INSERT,
	BLADE_POWER_CYCLE_BY_BLADE_BTN,
	CHASSIS_POWER_CYCLE_BY_SLED_BTN,
	HSC_FAULT,
	SYS_THROTTLE,
	VR_FAULT,
	SYS_MANAGEMENT_ERROR,
	POST_COMPLETED,
	FAN_ERROR,
	HDT_PRSNT_ASSERT,
	PLTRST_ASSERT,
	APML_ALERT_ASSERT,
	CXL1_HB,
	CXL2_HB,
	POST_STARTED,
	POST_ENDED,
	PROCHOT_TRIGGERED_BY_SENSOR_UCR,
	FRB2_WDT_HARD_RST,
	FRB2_WDT_PWR_DOWN,
	FRB2_WDT_PWR_CYCLE,
	OS_LOAD_WDT_EXPIRED,
	OS_LOAD_WDT_HARD_RST,
	OS_LOAD_WDT_PWR_DOWN,
	OS_LOAD_WDT_PWR_CYCLE,
	MTIA_VR_FAULT
};

enum vr_event_source {
	PVDDCR_CPU0 = 0x00,
	PVDDCR_SOC,
	PVDDCR_CPU1,
	PVDDIO,
	PVDD11_S3,
	PVDDQ_AB_ASIC1,
	P0V85_ASIC1,
	PVDDQ_CD_ASIC1,
	P0V8_ASIC1,
	PVDDQ_AB_ASIC2,
	P0V85_ASIC2,
	PVDDQ_CD_ASIC2,
	P0V8_ASIC2,
	P1V5_RETIMER_1,
	P0V9_STBY_1,
	P3V3_E1S_0,
	P3V3_E1S_1,
	P12V_E1S_0,
	P12V_E1S_1,
};

enum mtia_vr_event_source {
	MTIA_VR_P3V3 = 0,
	MTIA_VR_P0V85_PVDD,
	MTIA_VR_P0V75_PVDD_CH_N,
	MTIA_VR_P0V75_MAX_PHY_N,
	MTIA_VR_P0V75_PVDD_CH_S,
	MTIA_VR_P0V75_MAX_PHY_S,
	MTIA_VR_P0V75_TRVDD_ZONEA,
	MTIA_VR_P1V8_VPP_HBM0_HBM2_HBM4,
	MTIA_VR_P0V75_TRVDD_ZONEB,
	MTIA_VR_P0V4_VDDQL_HBM0_HBM2_HBM4,
	MTIA_VR_P1V1_VDDC_HBM0_HBM2_HBM4,
	MTIA_VR_P0V75_VDDPHY_HBM0_HBM2_HBM4,
	MTIA_VR_P0V9_TRVDD_ZONEA,
	MTIA_VR_P1V8_VPP_HBM1_HBM3_HBM5,
	MTIA_VR_P0V9_TRVDD_ZONEB,
	MTIA_VR_P0V4_VDDQL_HBM1_HBM3_HBM5,
	MTIA_VR_P1V1_VDDC_HBM1_HBM3_HBM5,
	MTIA_VR_P0V75_VDDPHY_HBM1_HBM3_HBM5,
	MTIA_VR_P0V8_VDDA_PCIE,
	MTIA_VR_P1V2_VDDHTX_PCIE,
	MTIA_VR_MAX,
};

enum READ_FILE_OPTION { READ_FILE_ATTR, READ_FILE_DATA };

struct _cmd_echo_req {
	uint8_t iana[IANA_LEN];
	uint8_t first_data;
} __attribute__((packed));

struct _cmd_echo_resp {
	uint8_t completion_code;
	uint8_t iana[IANA_LEN];
	uint8_t first_data;
} __attribute__((packed));

struct _ipmi_cmd_req {
	uint8_t iana[IANA_LEN];
	uint8_t netfn_lun;
	uint8_t cmd;
	uint8_t first_data;
} __attribute__((packed));

struct _ipmi_cmd_resp {
	uint8_t completion_code;
	uint8_t iana[IANA_LEN];
	uint8_t netfn_lun;
	uint8_t cmd;
	uint8_t ipmi_comp_code;
	uint8_t first_data;
} __attribute__((packed));

struct pldm_oem_write_file_io_req {
	uint8_t cmd_code;
	uint32_t data_length;
	uint8_t messages[];
} __attribute__((packed));

struct pldm_oem_write_file_io_resp {
	uint8_t completion_code;
} __attribute__((packed));

struct pldm_oem_read_file_data_info {
	uint8_t transfer_flag;
	uint16_t offset;
} __attribute__((packed));

struct pldm_oem_read_file_attr_info {
	uint16_t size;
	uint32_t crc32;
} __attribute__((packed));

struct pldm_oem_read_file_io_attr_req {
	uint8_t cmd_code;
	uint8_t read_option;
	uint8_t read_info_length;
} __attribute__((packed));

struct pldm_oem_read_file_io_data_req {
	uint8_t cmd_code;
	uint8_t read_option;
	uint8_t read_info_length;
	struct pldm_oem_read_file_data_info data;
} __attribute__((packed));

struct pldm_oem_read_file_io_attr_resp {
	uint8_t completion_code;
	uint8_t cmd_code;
	uint8_t read_option;
	uint8_t read_info_length;
	struct pldm_oem_read_file_attr_info attr;
	uint8_t read_info[];
} __attribute__((packed));

struct pldm_oem_read_file_io_data_resp {
	uint8_t completion_code;
	uint8_t cmd_code;
	uint8_t read_option;
	uint8_t read_info_length;
	struct pldm_oem_read_file_data_info data;
	uint8_t read_info[];
} __attribute__((packed));

struct pldm_addsel_data {
	uint8_t event_type;
	uint8_t assert_type;
	uint8_t event_data_1;
	uint8_t event_data_2;
	uint8_t event_data_3;
} __attribute__((packed));

uint8_t check_iana(const uint8_t *iana);
uint8_t set_iana(uint8_t *buf, uint8_t buf_len);
uint8_t send_event_log_to_bmc(struct pldm_addsel_data msg);

uint8_t pldm_oem_handler_query(uint8_t code, void **ret_fn);

#ifdef __cplusplus
}
#endif

#endif /* _PLDM_OEM_H */
