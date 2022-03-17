#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "plat_i2c.h"

#define CPLD_UPDATE_SIZE 0x80

#define ON_CHIP_FLASH_IP_DATA_REG 0x00000000
#define ON_CHIP_FLASH_IP_CSR_STATUS_REG 0x00200020

#define CFM_START_ADDR 0x27000
#define CFM_END_ADDR 0x49fff

// status register
#define BUSY_IDLE 0x00
#define BUSY_ERASE 0x01
#define BUSY_WRITE 0x02
#define BUSY_READ 0x03
#define READ_SUCCESS 0x04
#define WRITE_SUCCESS 0x08
#define ERASE_SUCCESS 0x10
#define STATUS_BIT_MASK 0x1F

void change_word_to_byte(uint8_t *output, int intput);
int get_register_via_i2c(int reg, int *val);
int set_register_via_i2c(int reg, int val);
int max10_reg_read(int address);
int max10_reg_write(int address, int data);
int max10_status_read(void);
int max10_write_flash_data(int address, int data);
void change_msg_to_buffer(uint8_t *buffer, int msg_len, uint8_t *msg);
int cpld_altera_fw_update(uint32_t offset, uint16_t msg_len, uint8_t *msg);
