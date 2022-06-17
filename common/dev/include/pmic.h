#ifndef PMIC_H
#define PMIC_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>

//Memory write read request max data len
#define MAX_MEMORY_DATA 32

//PMIC write/read data len
#define PMIC_WRITE_DATA_LEN 14
#define PMIC_READ_DATA_LEN 12

//PMIC write/read memory command
#define CMD_SMBUS_READ_MEMORY 0x47
#define CMD_SMBUS_WRITE_MEMORY 0x48

//PMIC report power/total mode setting
#define SET_DEV_REPORT_POWER 0x45
#define SET_DEV_REPORT_TOTAL 0xC2

//PMIC total/power/ADC/record register address value
#define PMIC_TOTAL_INDIV_ADDR_VAL 0x0000001A
#define PMIC_PWR_CURR_ADDR_VAL 0x0000001B
#define PMIC_ADC_ADDR_VAL 0x00000030
#define PMIC_SWA_ADDR_VAL 0x0000000C

//PMIC enable ADC bit
#define PMIC_ENABLE_ADC_BIT BIT(7)

//PMIC PMIC setting delay msec
#define PMIC_COMMAND_DELAY_MSEC 250

//Send PMIC memory write read parameter
#define INTEL_ID 0x00000157
#define PMIC_ADDR_SIZE 0x1
#define PMIC_DATA_LEN 0x1

//PMIC total power uint
#define PMIC_TOTAL_POWER_MW 125

typedef struct _memory_write_read_req_ {
	uint32_t intel_id;
	uint8_t smbus_identifier;
	uint8_t smbus_address;
	uint8_t addr_size;
	uint8_t data_len;
	uint32_t addr_value;
	uint8_t write_data[MAX_MEMORY_DATA];
} memory_write_read_req;

extern uint8_t *compose_memory_write_read_req(uint8_t smbus_identifier, uint8_t smbus_address,
					      uint32_t addr_value, uint8_t *write_data,
					      uint8_t write_len);
extern int pmic_ipmb_transfer(int *total_pmic_power, uint8_t seq_source, uint8_t netFn,
			      uint8_t command, uint8_t source_inft, uint8_t target_inft,
			      uint16_t data_len, uint8_t *data);
#endif
