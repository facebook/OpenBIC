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
#include "plat_cpld.h"
#include "pldm_oem.h"
#include "plat_log.h"

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

void check_clk_buf_loss_status(void)
{
	uint8_t clk_buf_loss_status = 0;
	if (!plat_read_cpld(CLK_100MHZ_BUF_LOSS_REG, &clk_buf_loss_status, 1)) {
		LOG_ERR("Failed to read 100MHz clock buffer loss status from CPLD");
		return;
	}
	/*
	Bit7: BUFF0_100M_LOSB_PLD
	Bit6: BUFF1_100M_LOSB_PLD
	Bit5: BUFF2_100M_LOSB_PLD
	if bit7 bit6 bit5 is 0 means fail
	*/
	if ((clk_buf_loss_status & 0xE0) != 0xE0) {
		if ((clk_buf_loss_status & BIT(7)) == 0) {
			uint16_t error_code =
				CLOCK_APLL_UNLOCK_EVENT_CAUSE | CLK_BUF0_100M_LOSB_PLD;
			error_log_event(error_code, LOG_ASSERT);
		}
		if ((clk_buf_loss_status & BIT(6)) == 0) {
			uint16_t error_code =
				CLOCK_APLL_UNLOCK_EVENT_CAUSE | CLK_BUF1_100M_LOSB_PLD;
			error_log_event(error_code, LOG_ASSERT);
		}
		if ((clk_buf_loss_status & BIT(5)) == 0) {
			uint16_t error_code =
				CLOCK_APLL_UNLOCK_EVENT_CAUSE | CLK_BUF2_100M_LOSB_PLD;
			error_log_event(error_code, LOG_ASSERT);
		}
	}
}

/* function to read APLL lock status for CLK_GEN_312_5M_U618 */
uint8_t clk_312_5mhz_get_lock_status_u618(void)
{
    #define U618_APLL_STS_REG_HSB 0x00
    #define U618_APLL_STS_REG_LSB 0xbd
	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = I2C_BUS3;
	i2c_msg.target_addr = CLK_GEN_312_5M_U618_ADDR; // 7-bit
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

/* get clock error data and send to bmc*/
bool clock_get_error_data(uint16_t error_code, uint8_t *data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	uint8_t clk_idx = error_code & 0xF;
	uint8_t lock_status = 0x00;
	uint8_t bmc_err_type = 0;
	bool ret = true;
	switch (clk_idx) {
		case CLK_100MHZ_ERR_IDX:
			lock_status = clk_100mhz_get_lock_status_u86();
			if (lock_status == 0xFF) {
				LOG_ERR("Failed to get 100MHz clock(U86) lock status");
				ret = false;
			}
			data[0] = lock_status;
			bmc_err_type = CLOCK_APLL_UNLOCK_EVENT;
			break;
		case CLK_312_5MHZ_ERR_IDX:
			lock_status = clk_312_5mhz_get_lock_status_u618();
			if (lock_status == 0xFF) {
				LOG_ERR("Failed to get 312.5MHz clock(U618) lock status");
				ret = false;
			}
			data[0] = lock_status;
			bmc_err_type = CLK_312_5M_APLL_UNLOCK_EVENT;
			break;
		case CLK_BUF0_100M_LOSB_PLD:
			bmc_err_type = CLK_BUF0_100M_LOSB_PLD_EVENT;
			if (!plat_read_cpld(CLK_100MHZ_BUF_LOSS_REG, &data[0], 1))
				ret = false;
			break;
		case CLK_BUF1_100M_LOSB_PLD:
			bmc_err_type = CLK_BUF1_100M_LOSB_PLD_EVENT;
			if (!plat_read_cpld(CLK_100MHZ_BUF_LOSS_REG, &data[0], 1))
				ret = false;
			break;
		case CLK_BUF2_100M_LOSB_PLD:
			bmc_err_type = CLK_BUF2_100M_LOSB_PLD_EVENT;
			if (!plat_read_cpld(CLK_100MHZ_BUF_LOSS_REG, &data[0], 1))
				ret = false;
			break;
		default:
			LOG_ERR("Unsupported clock error code: 0x%04x", error_code);
			return false;
	}
	packaged_bmc_log(ARKE_FAULT, bmc_err_type, data[0], 0);

	return ret;
}