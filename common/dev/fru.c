#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fru.h"
#include "plat_fru.h"

EEPROM_CFG fru_config[FRU_CFG_NUM];

static uint8_t find_FRU_ID(uint8_t FRUID)
{
	uint8_t ret;

	for (ret = 0; ret < MAX_FRU_ID; ret++) {
		if (FRUID == fru_config[ret].dev_id) {
			break;
		}
	}

	if (ret == MAX_FRU_ID) { // FRU ID not found
		return -1;
	}

	return ret;
}

uint8_t get_FRU_access(uint8_t FRUID)
{
	uint8_t ID_No;

	ID_No = find_FRU_ID(FRUID);

	if (ID_No == -1) { // FRU ID not found
		return 0xFF;
	}

	return fru_config[ID_No].access;
}

uint16_t find_FRU_size(uint8_t FRUID)
{
	uint8_t ID_No;

	ID_No = find_FRU_ID(FRUID);

	if (ID_No == -1) { // FRU ID not found
		return 0xFFFF;
	}

	return fru_config[ID_No].max_size;
}

uint8_t FRU_read(EEPROM_ENTRY *entry)
{
	uint8_t status;

	if (entry->config.dev_id >= MAX_FRU_ID) { // check if FRU is defined
		printf("fru read device ID %x not exist\n", entry->config.dev_id);
		return FRU_INVALID_ID;
	}

	if ((entry->offset + entry->data_len) >=
	    (FRU_START + FRU_SIZE)) { // Check data write out of range
		printf("fru read out of range, type: %x, ID: %x\n", entry->config.dev_type,
		       entry->config.dev_id);
		return FRU_OUT_OF_RANGE;
	}

	memcpy(&entry->config, &fru_config[entry->config.dev_id],
	       sizeof(fru_config[entry->config.dev_id]));
	status = eeprom_read(entry);

	if (!status) {
		return FRU_FAIL_TO_ACCESS;
	}

	return FRU_READ_SUCCESS;
}

uint8_t FRU_write(EEPROM_ENTRY *entry)
{
	uint8_t status;

	if (entry->config.dev_id >= MAX_FRU_ID) { // check if FRU is defined
		printf("fru write device ID %x not exist\n", entry->config.dev_id);
		return FRU_INVALID_ID;
	}

	if ((entry->offset + entry->data_len) >=
	    (FRU_START + FRU_SIZE)) { // Check data write out of range
		printf("fru write out of range, type: %x, ID: %x\n", entry->config.dev_type,
		       entry->config.dev_id);
		return FRU_OUT_OF_RANGE;
	}

	memcpy(&entry->config, &fru_config[entry->config.dev_id],
	       sizeof(fru_config[entry->config.dev_id]));
	status = eeprom_write(entry);

	if (!status) {
		return FRU_FAIL_TO_ACCESS;
	}

	return FRU_WRITE_SUCCESS;
}

__weak void pal_load_fru_config(void)
{
	return;
}

void FRU_init(void)
{
	pal_load_fru_config();
}
