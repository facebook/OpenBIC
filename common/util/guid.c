#include <stdio.h>
#include <string.h>
#include "guid.h"
#include "plat_guid.h"

// size of 1 to satisfy compiler warning
__weak const EEPROM_CFG guid_config[1] = {};

uint8_t GUID_read(EEPROM_ENTRY *entry)
{
	uint8_t status;

	if (entry->config.dev_id >= MAX_GUID_ID) {
		printf("GUID read device ID %x not exist\n", entry->config.dev_id);
		return GUID_INVALID_ID;
	}

	if ((entry->offset + entry->data_len) >=
	    (GUID_START + GUID_SIZE)) { // Check data write out of range
		printf("GUID read out of range, type: %x, ID: %x\n", entry->config.dev_type,
		       entry->config.dev_id);
		return GUID_OUT_OF_RANGE;
	}

	memcpy(&entry->config, &guid_config[entry->config.dev_id],
	       sizeof(guid_config[entry->config.dev_id]));
	status = eeprom_read(entry);

	if (!status) {
		return GUID_FAIL_TO_ACCESS;
	}

	return GUID_READ_SUCCESS;
}

uint8_t GUID_write(EEPROM_ENTRY *entry)
{
	uint8_t status;

	if (entry->config.dev_id >= MAX_GUID_ID) {
		printf("GUID write device ID %x not exist\n", entry->config.dev_id);
		return GUID_INVALID_ID;
	}

	if ((entry->offset + entry->data_len) >=
	    (GUID_START + GUID_SIZE)) { // Check data write out of range
		printf("GUID write out of range, type: %x, ID: %x\n", entry->config.dev_type,
		       entry->config.dev_id);
		return GUID_OUT_OF_RANGE;
	}

	memcpy(&entry->config, &guid_config[entry->config.dev_id],
	       sizeof(guid_config[entry->config.dev_id]));
	status = eeprom_write(entry);

	if (!status) {
		return GUID_FAIL_TO_ACCESS;
	}

	return GUID_WRITE_SUCCESS;
}
