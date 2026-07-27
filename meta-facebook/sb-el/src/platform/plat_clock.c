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
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
 #include "plat_clock.h"
#include "plat_i2c.h"
#include "plat_util.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(plat_clock);

/* function to read APLL lock status for CLK_GEN_100M_U86 */
uint8_t clk_100mhz_get_lock_status_u86(void)
{
    /* Switch to 2-byte address mode */
    #define SSI_GLOBAL_CNFG_REG 0x26
    #define SET_TWO_BYTE_ADDRESS 0x05
    uint8_t write_data = SET_TWO_BYTE_ADDRESS;
    if (!plat_i2c_write(I2C_BUS3, CLK_GEN_100M_U86_ADDR, SSI_GLOBAL_CNFG_REG, &write_data, 1)) {
        LOG_ERR("Failed to write 100MHz clock(U86) SSI 2-Byte address register");
        return 0xFF;
    }

    /* Read APLL lock status */
    #define U86_APLL_STS_REG_HSB 0x01
    #define U86_APLL_STS_REG_LSB 0x3F
	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = I2C_BUS3;
	i2c_msg.target_addr = CLK_GEN_100M_U86_ADDR; //7-bit
	i2c_msg.tx_len = 2;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = U86_APLL_STS_REG_HSB; //offset HSB
	i2c_msg.data[1] = U86_APLL_STS_REG_LSB; //offset LSB

	if (i2c_master_read_without_error_log(&i2c_msg, retry)) {
        LOG_ERR("Failed to read 100MHz clock(U86) APLL lock status");
		return 0xFF;
	}

    /* Switch back to 1-byte address mode */
    #define SET_ONE_BYTE_ADDRESS 0x01
    write_data = SET_ONE_BYTE_ADDRESS;
    if (!plat_i2c_write(I2C_BUS3, CLK_GEN_100M_U86_ADDR, SSI_GLOBAL_CNFG_REG, &write_data, 1)) {
        LOG_ERR("Failed to write 100MHz clock(U86) SSI 1-Byte address register");
        return 0xFF;
    }

	return i2c_msg.data[0] & 0x01; //bit0 is the APLL lock status
}

/* function to read APLL lock status for CLK_GEN_312_5M_U618 */
uint8_t clk_312_5mhz_get_lock_status_u618(void)
{
    #define U618_APLL_STS_REG_HSB 0x00
    #define U618_APLL_STS_REG_LSB 0xbd
	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = I2C_BUS3;
	i2c_msg.target_addr = CLK_GEN_312M_U618_ADDR; // 7-bit
	i2c_msg.tx_len = 2;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = U618_APLL_STS_REG_HSB; //offset HSB
	i2c_msg.data[1] = U618_APLL_STS_REG_LSB; //offset LSB
	if (i2c_master_read_without_error_log(&i2c_msg, retry)) {
        LOG_ERR("Failed to read 312.5MHz clock(U618) APLL lock status");
		return 0xFF; 
	}
	return i2c_msg.data[0] & 0x01; //bit 0 is the APLL lock status
}