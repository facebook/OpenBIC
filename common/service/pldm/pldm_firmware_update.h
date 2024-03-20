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

#ifndef _PLDM_FIRMWARE_UPDATE_H_
#define _PLDM_FIRMWARE_UPDATE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "pldm.h"
#include "plat_def.h"

#define MAX_FWUPDATE_RSP_BUF_SIZE 256
#define MAX_IMAGE_MALLOC_SIZE (1024 * 64)

#define KEYWORD_VR_ISL69259 "isl69259"
#define KEYWORD_VR_XDPE12284C "xdpe12284c"
#define KEYWORD_VR_MP2856 "mp2856"
#define KEYWORD_VR_MP2857 "mp2857"
#define KEYWORD_VR_MP2971 "mp2971"
#define KEYWORD_VR_XDPE15284 "xdpe15284"
#define KEYWORD_VR_MP2985 "mp2985"
#define KEYWORD_VR_RAA229620 "raa229620"
#define KEYWORD_VR_RAA229621 "raa229621"

#ifndef KEYWORD_CPLD_LATTICE
#define KEYWORD_CPLD_LATTICE "LCMXO3-9400C"
#endif

#define KEYWORD_RETIMER_PT5161L "pt5161l"
#define RETIMER_PT5161L_FW_VER_LEN 4

static const char hex_to_ascii[] = { '0', '1', '2', '3', '4', '5', '6', '7',
				     '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };

#define PLDM_COMMON_ERR_STR 'E', 'R', 'R', 'O', 'R', ':'
#define PLDM_COMMON_ERR_CODE 0
#define PLDM_CREATE_ERR_STR_ARRAY(code)                                                            \
	{                                                                                          \
		PLDM_COMMON_ERR_STR, hex_to_ascii[code]                                            \
	}

#define CHECK_PLDM_FW_UPDATE_RESULT_WITH_RETURN(component_id, offset, length, val, ret_val)                \
	if (val != 0) {                                                                                    \
		LOG_ERR("Component id: 0x%x firmware update fail, status: %d, offset: 0x%x, length: 0x%x", \
			component_id, val, offset, length);                                                \
		return ret_val;                                                                            \
	}

/**
 * PLDM Firmware update commands
 */
enum pldm_firmware_update_commands {
	/* inventory commands */
	PLDM_FW_UPDATE_CMD_CODE_QUERY_DEVICE_IDENTIFIERS = 0x01,
	PLDM_FW_UPDATE_CMD_CODE_GET_FIRMWARE_PARAMETERS = 0x02,
	PLDM_FW_UPDATE_CMD_CODE_QUERY_DOWNSTREAM_IDENTIFIERS = 0x04,

	/* update commands */
	PLDM_FW_UPDATE_CMD_CODE_REQUEST_UPDATE = 0x10,
	PLDM_FW_UPDATE_CMD_CODE_PASS_COMPONENT_TABLE = 0x13,
	PLDM_FW_UPDATE_CMD_CODE_UPDATE_COMPONENT = 0x14,
	PLDM_FW_UPDATE_CMD_CODE_REQUEST_FIRMWARE_DATA = 0x15,
	PLDM_FW_UPDATE_CMD_CODE_TRANSFER_COMPLETE = 0x16,
	PLDM_FW_UPDATE_CMD_CODE_VERIFY_COMPLETE = 0x17,
	PLDM_FW_UPDATE_CMD_CODE_APPLY_COMPLETE = 0x18,
	PLDM_FW_UPDATE_CMD_CODE_ACTIVE_FIRMWARE = 0x1A,
	PLDM_FW_UPDATE_CMD_CODE_GET_STATUS = 0x1B,
	PLDM_FW_UPDATE_CMD_CODE_CANCEL_UPDATE_COMPONENT = 0x1C,
	PLDM_FW_UPDATE_CMD_CODE_CANCEL_UPDATE = 0x1D,
};

/**
 * PLDM Firmware update completion codes
 */
enum pldm_firmware_update_completion_codes {
	PLDM_FW_UPDATE_CC_NOT_IN_UPDATE_MODE = 0x80,
	PLDM_FW_UPDATE_CC_ALREADY_IN_UPDATE_MODE = 0x81,
	PLDM_FW_UPDATE_CC_DATA_OUT_OF_RANGE = 0x82,
	PLDM_FW_UPDATE_CC_INVALID_TRANSFER_LENGTH = 0x83,
	PLDM_FW_UPDATE_CC_INVALID_STATE_FOR_COMMAND = 0x84,
	PLDM_FW_UPDATE_CC_INCOMPLETE_UPDATE = 0x85,
	PLDM_FW_UPDATE_CC_BUSY_IN_BACKGROUND = 0x86,
	PLDM_FW_UPDATE_CC_CANCEL_PENDING = 0x87,
	PLDM_FW_UPDATE_CC_COMMAND_NOT_EXPECTED = 0x88,
	PLDM_FW_UPDATE_CC_RETRY_REQUEST_FW_DATA = 0x89,
	PLDM_FW_UPDATE_CC_UNABLE_TO_INITIATE_UPDATE = 0x8A,
	PLDM_FW_UPDATE_CC_ACTIVATION_NOT_REQUIRED = 0x8B,
	PLDM_FW_UPDATE_CC_SELF_CONTAINED_ACTIVATION_NOT_PERMITTED = 0x8C,
	PLDM_FW_UPDATE_CC_NO_DEVICE_METADATA = 0x8D,
	PLDM_FW_UPDATE_CC_RETRY_REQUEST_UPDATE = 0x8E,
	PLDM_FW_UPDATE_CC_NO_PACKAGE_DATA = 0x8F,
	PLDM_FW_UPDATE_CC_INVALID_TRANSFER_HANDLE = 0x90,
	PLDM_FW_UPDATE_CC_INVALID_TRANSFER_OPERATION_FLAG = 0x91,
	PLDM_FW_UPDATE_CC_ACTIVATE_PENDING_IMAGE_NOT_PERMITTED = 0x92,
	PLDM_FW_UPDATE_CC_PACKAGE_DATA_ERROR = 0x93
};

/**
 * PLDM Frimware update string type
 */
enum pldm_firmware_update_string_type {
	PLDM_COMP_VER_STR_TYPE_UNKNOWN = 0,
	PLDM_COMP_ASCII = 1,
	PLDM_COMP_UTF_8 = 2,
	PLDM_COMP_UTF_16 = 3,
	PLDM_COMP_UTF_16LE = 4,
	PLDM_COMP_UTF_16BE = 5,
};

/**
 * PLDM Frimware update state
 */
enum pldm_firmware_update_state {
	STATE_IDLE,
	STATE_LEARN_COMP,
	STATE_RDY_XFER,
	STATE_DOWNLOAD,
	STATE_VERIFY,
	STATE_APPLY,
	STATE_ACTIVATE,
};

/**
 * PLDM Frimware update aux state
 */
enum pldm_firmware_update_aux_state {
	STATE_AUX_INPROGRESS,
	STATE_AUX_SUCCESS,
	STATE_AUX_FAILED,
	STATE_AUX_NOT_IN_UPDATE, //fd is in IDLE/LC/RX state
};

/**
 * PLDM component classification
 */
enum { COMP_CLASS_TYPE_UNKNOWN = 0x0000,
       COMP_CLASS_TYPE_OTHER,
       COMP_CLASS_TYPE_DRIVER,
       COMP_CLASS_TYPE_CFG_SW,
       COMP_CLASS_TYPE_APP_SW,
       COMP_CLASS_TYPE_INSTR,
       COMP_CLASS_TYPE_FW_BIOS,
       COMP_CLASS_TYPE_DIAG_SW,
       COMP_CLASS_TYPE_OS,
       COMP_CLASS_TYPE_MW,
       COMP_CLASS_TYPE_FW,
       COMP_CLASS_TYPE_BIOS_FC,
       COMP_CLASS_TYPE_SP_SV_P,
       COMP_CLASS_TYPE_SW_BUNDLE,
       COMP_CLASS_TYPE_DOWNSTREAM = 0xFFFF,
       COMP_CLASS_TYPE_MAX = 0x10000,
};

/**
 * Common error codes in TransferComplete, VerifyComplete and ApplyComplete request
 */
enum pldm_firmware_update_common_error_codes {
	PLDM_FW_UPDATE_FD_ABORT = 0x03,
	PLDM_FW_UPDATE_TIME_OUT = 0x09,
	PLDM_FW_UPDATE_GENERIC_ERROR = 0x0A
};

/**
 * TransferResult values in the request of TransferComplete
 */
enum pldm_firmware_update_transfer_result_values {
	PLDM_FW_UPDATE_TRANSFER_SUCCESS = 0x00,
	/* Other values that are not currently used, and will be defined if they are
  used in the future. */
};

/**
 * VerifyResult values in the request of VerifyComplete
 */
enum pldm_firmware_update_verify_result_values {
	PLDM_FW_UPDATE_VERIFY_SUCCESS = 0x00,
	/* Other values that are not currently used, and will be defined if they are
  used in the future. */
};

/**
 * ApplyResult values in the request of ApplyComplete
 */
enum pldm_firmware_update_apply_result_values {
	PLDM_FW_UPDATE_APPLY_SUCCESS = 0x00,
	PLDM_FW_UPDATE_APPLY_SUCCESS_HAS_MODIFY_ACTIVATE_METHOD,
	PLDM_FW_UPDATE_APPLY_FAIL_WITH_MEMORY_WRITE_ISSUE,
	PLDM_FW_UPDATE_APPLY_TIMEOUT_OCCURRED = 0x09,
	PLDM_FW_UPDATE_APPLY_GENERIC_ERROR_OCCURRED,
	/* Other values that are not currently used, and will be defined if they are
  used in the future. */
};

enum pldm_firmware_update_transfer_operation_flag {
	PLDM_FW_UPDATE_GET_NEXT_PART,
	PLDM_FW_UPDATE_GET_FIRST_PART,
};

enum pldm_firmware_update_transfer_flag {
	PLDM_FW_UPDATE_TRANSFER_START = 0x01,
	PLDM_FW_UPDATE_TRANSFER_MIDDLE = 0x02,
	PLDM_FW_UPDATE_TRANSFER_END = 0x04,
	PLDM_FW_UPDATE_TRANSFER_START_AND_END = 0x05,
};

/**
 * component classification values define in PLDM firmware update specification
 * Table 27
 */
enum pldm_component_classification_values {
	PLDM_COMP_UNKNOWN = 0x0000,
	PLDM_COMP_OTHER = 0x0001,
	PLDM_COMP_DRIVER = 0x0002,
	PLDM_COMP_CONFIGURATION_SOFTWARE = 0x0003,
	PLDM_COMP_APPLICATION_SOFTWARE = 0x0004,
	PLDM_COMP_INSTRUMENTATION = 0x0005,
	PLDM_COMP_FIRMWARE_OR_BIOS = 0x0006,
	PLDM_COMP_DIAGNOSTIC_SOFTWARE = 0x0007,
	PLDM_COMP_OPERATING_SYSTEM = 0x0008,
	PLDM_COMP_MIDDLEWARE = 0x0009,
	PLDM_COMP_FIRMWARE = 0x000A,
	PLDM_COMP_BIOS_OR_FCODE = 0x000B,
	PLDM_COMP_SUPPORT_OR_SERVICEPACK = 0x000C,
	PLDM_COMP_SOFTWARE_BUNDLE = 0x000D,
	PLDM_COMP_DOWNSTREAM_DEVICE = 0xFFFF
};

/** @brief Descriptor types defined in PLDM firmware update specification
 *  DSP0267 Table 7 – Descriptor identifier table
 */
enum pldm_firmware_update_descriptor_types {
	PLDM_PCI_VENDOR_ID = 0x0000,
	PLDM_FWUP_IANA_ENTERPRISE_ID = 0x0001,
	PLDM_PCI_DEVICE_ID = 0x0100,
	PLDM_ASCII_MODEL_NUMBER_LONG_STRING = 0x0106,
	PLDM_ASCII_MODEL_NUMBER_SHORT_STRING = 0x0107,
	PLDM_FWUP_VENDOR_DEFINED = 0xFFFF
};

/** @brief Descriptor types length defined in PLDM firmware update specification
 *  DSP0267 Table 7 – Descriptor identifier table
 */
enum pldm_firmware_update_descriptor_types_length {
	PLDM_PCI_VENDOR_ID_LENGTH = 2,
	PLDM_FWUP_IANA_ENTERPRISE_ID_LENGTH = 4,
	PLDM_PCI_DEVICE_ID_LENGTH = 2,
	PLDM_ASCII_MODEL_NUMBER_LONG_STRING_LENGTH = 40,
	PLDM_ASCII_MODEL_NUMBER_SHORT_STRING_LENGTH = 10,
};

/**
 * Component response for PassComponentTable and UpdateComponent commands
 */
enum comp_rsp_code {
	/* Common defined (0x00 ~ 0x0B) */
	COMP_RSP_CAN_UPDATE = 0x00,
	COMP_RSP_FD_NOT_SUPPORT = 0x06,

	/* Vendor defined (0xD0 ~ 0xEF) */
	COMP_RSP_UNKNOWN_ERR = 0xD0,
};

/**
 * Component response for PassComponentTable and UpdateComponent commands
 */
enum comp_act_mdthod {
	COMP_ACT_AUTO = 0x0001,
	COMP_ACT_SELF = 0x0002,
	COMP_ACT_MED_RESET = 0x0004,
	COMP_ACT_SYS_REBOOT = 0x0008,
	COMP_ACT_DC_PWR_CYCLE = 0x0010,
	COMP_ACT_AC_PWR_CYCLE = 0x0020,
	COMP_ACT_SUPP_PEND_IMAGE = 0x0040,
	COMP_ACT_SUPP_PEND_COMP_IMG_SET = 0x0080,
};

typedef enum fd_update_interface {
	COMP_UPDATE_VIA_UNKNOWN,
	COMP_UPDATE_VIA_I2C,
	COMP_UPDATE_VIA_SPI,
	COMP_UPDATE_VIA_JTAG,
} fd_update_interface_t;

// typedef uint8_t (*pldm_fwupdate_func)(uint16_t comp_id, void *mctp_p, void *ext_params);
typedef uint8_t (*pldm_fwupdate_func)(void *fw_update_param);
typedef uint8_t (*pldm_act_func)(void *arg);
typedef uint8_t (*pldm_apply_work)(void *arg);
typedef bool (*pldm_get_fw_version_fn)(void *info_p, uint8_t *buf, uint8_t *len);
typedef struct pldm_fw_update_param {
	uint16_t comp_id;
	char *comp_version_str;
	uint8_t *data;
	uint32_t data_ofs;
	uint32_t data_len;
	uint32_t next_ofs;
	uint32_t next_len;
	fd_update_interface_t inf;
	uint8_t bus; //i2c/jtag
	uint8_t addr; //i2c
} pldm_fw_update_param_t;

typedef struct pldm_fw_update_info {
	bool enable;
	uint16_t comp_classification;
	uint16_t comp_identifier;
	uint8_t comp_classification_index;
	pldm_fwupdate_func pre_update_func;
	pldm_fwupdate_func update_func;
	pldm_fwupdate_func pos_update_func;
	fd_update_interface_t inf;
	uint16_t activate_method;
	pldm_apply_work self_apply_work_func;
	void *self_apply_work_arg;
	pldm_act_func self_act_func;
	pldm_get_fw_version_fn get_fw_version_fn;
	uint8_t *pending_version_p;
	char *comp_version_str;
} pldm_fw_update_info_t;
extern pldm_fw_update_info_t *comp_config;
extern uint8_t comp_config_count;

struct pldm_fw_update_cfg {
	uint32_t image_size;
	uint16_t max_buff_size;
};
extern struct pldm_fw_update_cfg fw_update_cfg;

/**
 * Structure representing fixed part of Request Update request
 */
struct pldm_request_update_req {
	uint32_t max_transfer_size;
	uint16_t num_of_comp;
	uint8_t max_outstanding_transfer_req;
	uint16_t pkg_data_len;
	uint8_t comp_image_set_ver_str_type;
	uint8_t comp_image_set_ver_str_len;
} __attribute__((packed));

/**
 * Structure representing Request Update response
 */
struct pldm_request_update_resp {
	uint8_t completion_code;
	uint16_t fd_meta_data_len;
	uint8_t fd_will_send_pkg_data;
} __attribute__((packed));

/**
 * Structure representing PassComponentTable request
 */
struct pldm_pass_component_table_req {
	uint8_t transfer_flag;
	uint16_t comp_classification;
	uint16_t comp_identifier;
	uint8_t comp_classification_index;
	uint32_t comp_comparison_stamp;
	uint8_t comp_ver_str_type;
	uint8_t comp_ver_str_len;
} __attribute__((packed));

/**
 * Structure representing PassComponentTable response
 */
struct pldm_pass_component_table_resp {
	uint8_t completion_code;
	uint8_t comp_resp;
	uint8_t comp_resp_code;
} __attribute__((packed));

/**
 * Structure representing UpdateComponent request
 */
struct pldm_update_component_req {
	uint16_t comp_classification;
	uint16_t comp_identifier;
	uint8_t comp_classification_index;
	uint32_t comp_comparison_stamp;
	uint32_t comp_image_size;
	uint32_t update_option_flags;
	uint8_t comp_ver_str_type;
	uint8_t comp_ver_str_len;
} __attribute__((packed));

/**
 * Structure representing UpdateComponent response
 */
struct pldm_update_component_resp {
	uint8_t completion_code;
	uint8_t comp_compatability_resp;
	uint8_t comp_compatability_resp_code;
	uint32_t update_option_flags_enabled;
	uint16_t time_before_req_fw_data;
} __attribute__((packed));

/**
 * Structure representing RequestFirmwareData request
 */
struct pldm_request_firmware_data_req {
	uint32_t offset;
	uint32_t length;
} __attribute__((packed));

/**
 * Structure representing ActivateFirmware request
 */
struct pldm_activate_firmware_req {
	uint8_t selfContainedActivationRequest;
} __attribute__((packed));

/**
 * Structure representing ActivateFirmware response
 */
struct pldm_activate_firmware_resp {
	uint8_t completion_code;
	uint16_t estimated;
} __attribute__((packed));

/**
 * Structure representing GetStatus response
 */
struct pldm_get_status_resp {
	uint8_t completion_code;
	uint8_t cur_state;
	uint8_t pre_state;
	uint8_t aux_state;
	uint8_t aux_state_status;
	uint8_t prog_percent;
	uint8_t reason_code;
	uint32_t update_op_flag_en;
} __attribute__((packed));

/**
 * Structure representing CancelUpdate response
 */
struct pldm_cancel_update_resp {
	uint8_t completion_code;
	uint8_t non_func_comp_ind;
	uint64_t non_func_comp_bitmap;
} __attribute__((packed));

/**
 * SelfContainedActivationRequest in the request of ActivateFirmware
 */
enum pldm_self_contained_activation_req {
	PLDM_NOT_ACTIVATE_SELF_CONTAINED_COMPONENTS = false,
	PLDM_ACTIVATE_SELF_CONTAINED_COMPONENTS = true
};

/**
 * Structure representing TransferComplete request
 */
struct pldm_transfer_complete_req {
	uint8_t transferResult;
} __attribute__((packed));

struct pldm_verify_complete_req {
	uint8_t verifyResult;
} __attribute__((packed));

struct pldm_apply_complete_req {
	uint8_t applyResult;
	uint16_t compActivationMethodsModification;
} __attribute__((packed));

/**
 * Structure representing QueryDeviceIdentifiers request
 */
struct pldm_query_device_identifiers_resp {
	uint8_t completion_code;
	uint32_t device_identifiers_len;
	uint8_t descriptor_count;
} __attribute__((packed));

/**
 *  Structure representing descriptor type, length and value
 */
struct pldm_descriptor_tlv {
	uint16_t descriptor_type;
	uint16_t descriptor_length;
	uint8_t descriptor_data[1];
} __attribute__((packed));

struct pldm_vendor_defined_descriptor_tlv {
	uint16_t descriptor_type;
	uint16_t descriptor_length;
	uint8_t vendor_define_title_type;
	uint8_t descriptor_title_length;
	uint8_t descriptor_data[1];
} __attribute__((packed));

struct pldm_descriptor_string {
	uint16_t descriptor_type;
	char *title_string;
	char *descriptor_data;
} __attribute__((packed));

struct pldm_get_firmware_parameters_resp {
	uint8_t completion_code;
	union {
		struct {
			uint8_t fail_recovery : 1;
			uint8_t fail_retry : 1;
			uint8_t func_during_update : 1;
			uint8_t partial_update : 1;
			uint8_t update_mode_restrict : 4;
			/* Bit [31:8] reserved */
			uint8_t : 8;
			uint16_t : 16;
		};
		uint32_t capabilities_during_update;
	};
	uint16_t comp_count;
	uint8_t active_comp_image_set_ver_str_type;
	uint8_t active_comp_image_set_ver_str_len;
	uint8_t pending_comp_image_set_ver_str_type;
	uint8_t pending_comp_image_set_ver_str_len;
} __attribute__((packed));

struct component_parameter_table {
	uint16_t comp_classification;
	uint16_t comp_identifier;
	uint8_t comp_classification_index;
	uint32_t active_comp_comparison_stamp;
	uint8_t active_comp_ver_str_type;
	uint8_t active_comp_ver_str_len;
	uint8_t active_comp_release_date[8];
	uint32_t pending_comp_comparison_stamp;
	uint8_t pending_comp_ver_str_type;
	uint8_t pending_comp_ver_str_len;
	uint8_t pending_comp_release_date[8];
	uint16_t comp_activation_methods;
	uint32_t capabilities_during_update;
} __attribute__((packed));

struct pldm_query_downstream_identifier_req {
	uint32_t datatransferhandle;
	uint8_t transferoperationflag;
} __attribute__((packed));

struct pldm_query_downstream_identifier_resp {
	uint8_t completion_code;
	uint32_t nextdatatransferhandle;
	uint8_t transferflag;
	uint32_t downstreamdevicelength;
	uint16_t numbwerofdownstreamdevice;
	uint16_t downstreamdeviceindex;
	uint8_t downstreamdescriptorcount;
} __attribute__((packed));

struct pldm_downstream_identifier_table {
	struct pldm_descriptor_string *descriptor;
	uint8_t descriptor_count;
};

uint8_t pldm_fw_update_handler_query(uint8_t code, void **ret_fn);
uint16_t pldm_fw_update_read(void *mctp_p, enum pldm_firmware_update_commands cmd, uint8_t *req,
			     uint16_t req_len, uint8_t *rbuf, uint16_t rbuf_len, void *ext_params);
uint8_t pldm_bic_update(void *fw_update_param);
uint8_t pldm_vr_update(void *fw_update_param);
uint8_t pldm_cpld_update(void *fw_update_param);
uint8_t pldm_retimer_update(void *fw_update_param);
uint8_t pldm_bic_activate(void *arg);

uint8_t plat_pldm_query_device_identifiers(const uint8_t *buf, uint16_t len, uint8_t *resp,
					   uint16_t *resp_len);
uint8_t plat_pldm_query_downstream_identifiers(const uint8_t *buf, uint16_t len, uint8_t *resp,
					       uint16_t *resp_len);

int get_descriptor_type_length(uint16_t type);
int get_device_single_descriptor_length(struct pldm_descriptor_string data);
int get_device_descriptor_total_length(struct pldm_descriptor_string *table, uint8_t table_count);
uint8_t fill_descriptor_into_buf(struct pldm_descriptor_string *descriptor, uint8_t *buf,
				 uint8_t *fill_length, uint16_t current_length);

#ifdef __cplusplus
}
#endif

#endif /* _PLDM_FIRMWARE_UPDATE_H_ */
