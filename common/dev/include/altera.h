#ifndef ALTERA_H
#define ALTERA_H

#include <stdint.h>

#define CPLD_UPDATE_SIZE 0x80

// on-chip Flash IP
#define ON_CHIP_FLASH_IP_CSR_STATUS_REG 0x00200020
#define ON_CHIP_FLASH_IP_DATA_REG 0x00000000

// Dual-boot IP
#define DUAL_BOOT_IP_BASE 0x00200000
#define M04_CFM1_START_ADDR 0x00027000
#define M04_CFM1_END_ADDR 0x00049FFF

// status register
#define BUSY_IDLE 0x00
#define BUSY_ERASE 0x01
#define BUSY_WRITE 0x02
#define BUSY_READ 0x03
#define READ_SUCCESS 0x04
#define WRITE_SUCCESS 0x08
#define ERASE_SUCCESS 0x10
#define STATUS_BIT_MASK 0x1F

typedef struct _altera_max10_attr {
	uint8_t bus;
	uint8_t slave_addr;
	int update_start_addr;
	int update_end_addr;
} altera_max10_attr;

int change_word_to_byte(uint8_t *output, int intput);
int get_register_via_i2c(int reg, int *val);
int set_register_via_i2c(int reg, int val);
int max10_reg_read(int address);
int max10_reg_write(int address, int data);
int max10_status_read(void);
int max10_write_flash_data(int address, int data);
int cpld_altera_max10_fw_update(uint32_t offset, uint16_t msg_len, uint8_t *msg);
int pal_load_altera_max10_attr();

#endif
