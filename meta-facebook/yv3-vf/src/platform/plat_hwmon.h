#include <stdio.h>
#include <stdint.h>

#define I2C_ADDR_MB_CPLD (0x1E >> 1)

void BICup1secTickHandler();
int8_t mb_cpld_dev_prsnt_set(uint32_t idx, uint32_t val);