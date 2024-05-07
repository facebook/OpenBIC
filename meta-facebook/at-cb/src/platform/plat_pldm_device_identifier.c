#include "libutil.h"
#include "pldm_firmware_update.h"
#include "plat_pldm_device_identifier.h"

/* PLDM descriptors table */
struct pldm_descriptor_string PLDM_DEVICE_DESCRIPTOR_TABLE[] = {
	{
		.descriptor_type = PLDM_FWUP_IANA_ENTERPRISE_ID,
		.title_string = NULL,
		.descriptor_data = "0000A015",
	},
	{
		.descriptor_type = PLDM_FWUP_VENDOR_DEFINED,
		.title_string = "Platform",
		.descriptor_data = "Artemis",
	},
	{
		.descriptor_type = PLDM_FWUP_VENDOR_DEFINED,
		.title_string = "Board",
		.descriptor_data = "ColterBay",
	},
	{
		.descriptor_type = PLDM_FWUP_VENDOR_DEFINED,
		.title_string = "Stage",
		.descriptor_data = "PVT",
	},
	{
		.descriptor_type = PLDM_ASCII_MODEL_NUMBER_LONG_STRING,
		.title_string = NULL,
		.descriptor_data =
			"617374313033305f6269630000000000000000000000000000000000000000000000000000000000",
	},
	{
		.descriptor_type = PLDM_ASCII_MODEL_NUMBER_SHORT_STRING,
		.title_string = NULL,
		.descriptor_data = "31000000000000000000",
	},
};

struct pldm_descriptor_string ASIC_PSOC_DESCRIPTOR_TABLE[] = {
	{
		.descriptor_type = PLDM_FWUP_IANA_ENTERPRISE_ID,
		.title_string = NULL,
		.descriptor_data = "0000A015",
	},
	{
		.descriptor_type = PLDM_FWUP_VENDOR_DEFINED,
		.title_string = "Platform",
		.descriptor_data = "Artemis",
	},
	{
		.descriptor_type = PLDM_FWUP_VENDOR_DEFINED,
		.title_string = "Board",
		.descriptor_data = "ColterBay",
	},
	{
		.descriptor_type = PLDM_FWUP_VENDOR_DEFINED,
		.title_string = "Stage",
		.descriptor_data = "PVT",
	},
	{
		.descriptor_type = PLDM_ASCII_MODEL_NUMBER_LONG_STRING,
		.title_string = NULL,
		.descriptor_data =
			"62726f6164636f6d5f61736963000000000000000000000000000000000000000000000000000000",
	},
	{
		.descriptor_type = PLDM_ASCII_MODEL_NUMBER_SHORT_STRING,
		.title_string = NULL,
		.descriptor_data = "31000000000000000000",
	},
	{
		.descriptor_type = PLDM_PCI_VENDOR_ID,
		.title_string = NULL,
		.descriptor_data = "1D9B",
	},
	{
		.descriptor_type = PLDM_PCI_DEVICE_ID,
		.title_string = NULL,
		.descriptor_data = "0201",
	},
};

struct pldm_descriptor_string ASIC_QSPI_DESCRIPTOR_TABLE[] = {
	{
		.descriptor_type = PLDM_FWUP_IANA_ENTERPRISE_ID,
		.title_string = NULL,
		.descriptor_data = "0000A015",
	},
	{
		.descriptor_type = PLDM_FWUP_VENDOR_DEFINED,
		.title_string = "Platform",
		.descriptor_data = "Artemis",
	},
	{
		.descriptor_type = PLDM_FWUP_VENDOR_DEFINED,
		.title_string = "Board",
		.descriptor_data = "ColterBay",
	},
	{
		.descriptor_type = PLDM_FWUP_VENDOR_DEFINED,
		.title_string = "Stage",
		.descriptor_data = "PVT",
	},
	{
		.descriptor_type = PLDM_ASCII_MODEL_NUMBER_LONG_STRING,
		.title_string = NULL,
		.descriptor_data =
			"62726f6164636f6d5f61736963000000000000000000000000000000000000000000000000000000",
	},
	{
		.descriptor_type = PLDM_ASCII_MODEL_NUMBER_SHORT_STRING,
		.title_string = NULL,
		.descriptor_data = "32000000000000000000",
	},
	{
		.descriptor_type = PLDM_PCI_VENDOR_ID,
		.title_string = NULL,
		.descriptor_data = "1D9B",
	},
	{
		.descriptor_type = PLDM_PCI_DEVICE_ID,
		.title_string = NULL,
		.descriptor_data = "0201",
	},
};

struct pldm_descriptor_string ASIC_BOOT1_DESCRIPTOR_TABLE[] = {
	{
		.descriptor_type = PLDM_FWUP_IANA_ENTERPRISE_ID,
		.title_string = NULL,
		.descriptor_data = "0000A015",
	},
	{
		.descriptor_type = PLDM_FWUP_VENDOR_DEFINED,
		.title_string = "Platform",
		.descriptor_data = "Artemis",
	},
	{
		.descriptor_type = PLDM_FWUP_VENDOR_DEFINED,
		.title_string = "Board",
		.descriptor_data = "ColterBay",
	},
	{
		.descriptor_type = PLDM_FWUP_VENDOR_DEFINED,
		.title_string = "Stage",
		.descriptor_data = "PVT",
	},
	{
		.descriptor_type = PLDM_ASCII_MODEL_NUMBER_LONG_STRING,
		.title_string = NULL,
		.descriptor_data =
			"62726f6164636f6d5f61736963000000000000000000000000000000000000000000000000000000",
	},
	{
		.descriptor_type = PLDM_ASCII_MODEL_NUMBER_SHORT_STRING,
		.title_string = NULL,
		.descriptor_data = "33000000000000000000",
	},
	{
		.descriptor_type = PLDM_PCI_VENDOR_ID,
		.title_string = NULL,
		.descriptor_data = "1D9B",
	},
	{
		.descriptor_type = PLDM_PCI_DEVICE_ID,
		.title_string = NULL,
		.descriptor_data = "0201",
	},
};

const uint8_t bic_descriptors_count = ARRAY_SIZE(PLDM_DEVICE_DESCRIPTOR_TABLE);
const uint8_t asic_psoc_descriptors_count = ARRAY_SIZE(ASIC_PSOC_DESCRIPTOR_TABLE);
const uint8_t asic_qspi_descriptors_count = ARRAY_SIZE(ASIC_QSPI_DESCRIPTOR_TABLE);
const uint8_t asic_boot1_descriptors_count = ARRAY_SIZE(ASIC_BOOT1_DESCRIPTOR_TABLE);

struct pldm_downstream_identifier_table downstream_table[] = {
	{ .descriptor = ASIC_PSOC_DESCRIPTOR_TABLE,
	  .descriptor_count = asic_psoc_descriptors_count },
	{ .descriptor = ASIC_QSPI_DESCRIPTOR_TABLE,
	  .descriptor_count = asic_qspi_descriptors_count },
	{ .descriptor = ASIC_BOOT1_DESCRIPTOR_TABLE,
	  .descriptor_count = asic_boot1_descriptors_count },
};

const uint8_t downstream_table_count = ARRAY_SIZE(downstream_table);
