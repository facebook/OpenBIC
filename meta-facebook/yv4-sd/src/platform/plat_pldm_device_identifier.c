#include "libutil.h"
#include "pldm_firmware_update.h"
#include "plat_pldm_device_identifier.h"

struct pldm_descriptor_string PLDM_VR_PVDDCR_CPU1_DESCRIPTORS[] = {
	{
		.descriptor_type = PLDM_FWUP_IANA_ENTERPRISE_ID,
		.title_string = NULL,
		.descriptor_data = "0000A015",
	},
	{
		.descriptor_type = PLDM_FWUP_VENDOR_DEFINED,
		.title_string = "Platform",
		.descriptor_data = "Yosemite4",
	},
	{
		.descriptor_type = PLDM_FWUP_VENDOR_DEFINED,
		.title_string = "Board",
		.descriptor_data = "SentinelDome",
	},
	{
		.descriptor_type = PLDM_ASCII_MODEL_NUMBER_LONG_STRING,
		.title_string = NULL,
		.descriptor_data =
			"4d50535f565200000000000000000000000000000000000000000000000000000000000000000000",
	},
	{
		.descriptor_type = PLDM_ASCII_MODEL_NUMBER_SHORT_STRING,
		.title_string = NULL,
		.descriptor_data = "31000000000000000000",
	},
};

struct pldm_descriptor_string PLDM_VR_PVDD11_S3_DESCRIPTORS[] = {
	{
		.descriptor_type = PLDM_FWUP_IANA_ENTERPRISE_ID,
		.title_string = NULL,
		.descriptor_data = "0000A015",
	},
	{
		.descriptor_type = PLDM_FWUP_VENDOR_DEFINED,
		.title_string = "Platform",
		.descriptor_data = "Yosemite4",
	},
	{
		.descriptor_type = PLDM_FWUP_VENDOR_DEFINED,
		.title_string = "Board",
		.descriptor_data = "SentinelDome",
	},
	{
		.descriptor_type = PLDM_ASCII_MODEL_NUMBER_LONG_STRING,
		.title_string = NULL,
		.descriptor_data =
			"4d50535f565200000000000000000000000000000000000000000000000000000000000000000000",
	},
	{
		.descriptor_type = PLDM_ASCII_MODEL_NUMBER_SHORT_STRING,
		.title_string = NULL,
		.descriptor_data = "32000000000000000000",
	},
};

struct pldm_descriptor_string PLDM_VR_PVDDCR_CPU0_DESCRIPTORS[] = {
	{
		.descriptor_type = PLDM_FWUP_IANA_ENTERPRISE_ID,
		.title_string = NULL,
		.descriptor_data = "0000A015",
	},
	{
		.descriptor_type = PLDM_FWUP_VENDOR_DEFINED,
		.title_string = "Platform",
		.descriptor_data = "Yosemite4",
	},
	{
		.descriptor_type = PLDM_FWUP_VENDOR_DEFINED,
		.title_string = "Board",
		.descriptor_data = "SentinelDome",
	},
	{
		.descriptor_type = PLDM_ASCII_MODEL_NUMBER_LONG_STRING,
		.title_string = NULL,
		.descriptor_data =
			"4d50535f565200000000000000000000000000000000000000000000000000000000000000000000",
	},
	{
		.descriptor_type = PLDM_ASCII_MODEL_NUMBER_SHORT_STRING,
		.title_string = NULL,
		.descriptor_data = "33000000000000000000",
	},
};

struct pldm_descriptor_string PLDM_RETIMER_X16_DESCRIPTORS[] = {
	{
		.descriptor_type = PLDM_FWUP_IANA_ENTERPRISE_ID,
		.title_string = NULL,
		.descriptor_data = "0000A015",
	},
	{
		.descriptor_type = PLDM_FWUP_VENDOR_DEFINED,
		.title_string = "Platform",
		.descriptor_data = "Yosemite4",
	},
	{
		.descriptor_type = PLDM_FWUP_VENDOR_DEFINED,
		.title_string = "Board",
		.descriptor_data = "SentinelDome",
	},
	{
		.descriptor_type = PLDM_ASCII_MODEL_NUMBER_LONG_STRING,
		.title_string = NULL,
		.descriptor_data =
			"4153544552414c41425f526574696d65720000000000000000000000000000000000000000000000",
	},
	{
		.descriptor_type = PLDM_ASCII_MODEL_NUMBER_SHORT_STRING,
		.title_string = NULL,
		.descriptor_data = "31000000000000000000",
	},
};

struct pldm_descriptor_string PLDM_RETIMER_X8_DESCRIPTORS[] = {
	{
		.descriptor_type = PLDM_FWUP_IANA_ENTERPRISE_ID,
		.title_string = NULL,
		.descriptor_data = "0000A015",
	},
	{
		.descriptor_type = PLDM_FWUP_VENDOR_DEFINED,
		.title_string = "Platform",
		.descriptor_data = "Yosemite4",
	},
	{
		.descriptor_type = PLDM_FWUP_VENDOR_DEFINED,
		.title_string = "Board",
		.descriptor_data = "SentinelDome",
	},
	{
		.descriptor_type = PLDM_ASCII_MODEL_NUMBER_LONG_STRING,
		.title_string = NULL,
		.descriptor_data =
			"4153544552414c41425f526574696d65720000000000000000000000000000000000000000000000",
	},
	{
		.descriptor_type = PLDM_ASCII_MODEL_NUMBER_SHORT_STRING,
		.title_string = NULL,
		.descriptor_data = "32000000000000000000",
	},
};

struct pldm_descriptor_string PLDM_BIOS_DESCRIPTORS[] = {
	{
		.descriptor_type = PLDM_FWUP_IANA_ENTERPRISE_ID,
		.title_string = NULL,
		.descriptor_data = "0000A015",
	},
	{
		.descriptor_type = PLDM_FWUP_VENDOR_DEFINED,
		.title_string = "Platform",
		.descriptor_data = "Yosemite4",
	},
	{
		.descriptor_type = PLDM_FWUP_VENDOR_DEFINED,
		.title_string = "Board",
		.descriptor_data = "SentinelDome",
	},
	{
		.descriptor_type = PLDM_ASCII_MODEL_NUMBER_LONG_STRING,
		.title_string = NULL,
		.descriptor_data =
			"42494f53000000000000000000000000000000000000000000000000000000000000000000000000",
	},
};

struct pldm_downstream_identifier_table downstream_table[] = {
	{ .descriptor = PLDM_VR_PVDDCR_CPU1_DESCRIPTORS,
	  .descriptor_count = ARRAY_SIZE(PLDM_VR_PVDDCR_CPU1_DESCRIPTORS) },
	{ .descriptor = PLDM_VR_PVDD11_S3_DESCRIPTORS,
	  .descriptor_count = ARRAY_SIZE(PLDM_VR_PVDD11_S3_DESCRIPTORS) },
	{ .descriptor = PLDM_VR_PVDDCR_CPU0_DESCRIPTORS,
	  .descriptor_count = ARRAY_SIZE(PLDM_VR_PVDDCR_CPU0_DESCRIPTORS) },
	{ .descriptor = PLDM_RETIMER_X16_DESCRIPTORS,
	  .descriptor_count = ARRAY_SIZE(PLDM_RETIMER_X16_DESCRIPTORS) },
	{ .descriptor = PLDM_RETIMER_X8_DESCRIPTORS,
	  .descriptor_count = ARRAY_SIZE(PLDM_RETIMER_X8_DESCRIPTORS) },
	{ .descriptor = PLDM_BIOS_DESCRIPTORS,
	  .descriptor_count = ARRAY_SIZE(PLDM_BIOS_DESCRIPTORS) },
};

const uint8_t downstream_devices_count = ARRAY_SIZE(downstream_table);
