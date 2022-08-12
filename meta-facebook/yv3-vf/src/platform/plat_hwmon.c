#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "plat_m2.h"
#include "plat_gpio.h"
#include "plat_power_seq.h"
#include "plat_class.h"
#include "plat_util.h"
#include "plat_i2c.h"
#include "plat_sensor_table.h"

#include "plat_hwmon.h"

static void init_dev_prsnt_status(void)
{
	uint8_t i;
	for (i = M2_IDX_E_A; i < M2_IDX_E_MAX; i++) {
		if (!mb_cpld_dev_prsnt_set(i, m2_prsnt(i)))
			return;
	}
}

void BICup1secTickHandler()
{
	init_dev_prsnt_status();
}

int8_t mb_cpld_dev_prsnt_set(uint32_t idx, uint32_t val)
{
#define MB_CPLD_PRSNT_REG_1OU 0x05
	/*
 * device present offset
 * dev0, offset 4
 * dev1, offset 3
 * dev2, offset 2
 * dev3, offset 1
 */

	uint8_t reg = MB_CPLD_PRSNT_REG_1OU;
	uint8_t prsnt_ofs = idx + 1;
	uint8_t buf;

	uint8_t retry = 5;
	I2C_MSG msg;

	msg.bus = I2C_BUS_MB_CPLD;
	msg.target_addr = I2C_ADDR_MB_CPLD;
	msg.tx_len = sizeof(reg);
	msg.rx_len = sizeof(buf);
	memcpy(&msg.data[0], &reg, sizeof(reg));

	if (i2c_master_read(&msg, retry)) {
		printf("MB CPLD read failed!\n");
		return false;
	}

	buf = msg.data[0];
	WRITE_BIT(buf, prsnt_ofs, !val);

	msg.tx_len = sizeof(buf) + 1;
	msg.data[0] = reg;
	memcpy(&msg.data[1], &buf, sizeof(buf));

	if (i2c_master_write(&msg, retry)) {
		return false;
	}

	return true;
}
