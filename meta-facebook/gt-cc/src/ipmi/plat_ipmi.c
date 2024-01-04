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

#include "pldm.h"
#include "plat_ipmi.h"

#include <stdio.h>
#include <stdlib.h>
#include <drivers/spi_nor.h>
#include <drivers/flash.h>
#include <logging/log.h>

#include "libutil.h"
#include "ipmi.h"
#include "plat_ipmb.h"
#include "plat_gpio.h"
#include "plat_fru.h"
#include "plat_class.h"
#include "hal_jtag.h"
#include "eeprom.h"
#include "fru.h"
#include "sdr.h"
#include "app_handler.h"
#include "util_spi.h"
#include "sensor.h"
#include "pmbus.h"
#include "hal_i2c.h"
#include "i2c-mux-tca9548.h"
#include "plat_hook.h"
#include "plat_sensor_table.h"
#include "plat_mctp.h"
#include "pex89000.h"
#include "power_status.h"

#define GT_BIC_UPDATE_MAX_OFFSET KB(410)

LOG_MODULE_REGISTER(plat_ipmi);

struct bridge_compnt_info_s {
	uint8_t compnt_id;
	uint8_t i2c_bus;
	uint8_t i2c_addr;
	bool mux_present;
	uint8_t mux_addr;
	uint8_t mux_channel;
	struct k_mutex *bus_mutex;
};

extern struct k_mutex i2c_bus6_mutex;
extern struct k_mutex i2c_bus10_mutex;
struct bridge_compnt_info_s bridge_compnt_info[] = {
	[0] = { .compnt_id = GT_COMPNT_VR0,
		.i2c_bus = I2C_BUS6,
		.i2c_addr = PEX_0_1_P0V8_VR_ADDR,
		.mux_present = true,
		.mux_addr = 0xe0,
		.mux_channel = 6,
		.bus_mutex = &i2c_bus6_mutex },
	[1] = { .compnt_id = GT_COMPNT_VR1,
		.i2c_bus = I2C_BUS6,
		.i2c_addr = PEX_2_3_P0V8_VR_ADDR,
		.mux_present = true,
		.mux_addr = 0xe0,
		.mux_channel = 6,
		.bus_mutex = &i2c_bus6_mutex },
};

void OEM_1S_FW_UPDATE(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	/*********************************
	* Request Data
	*
	* Byte   0: [6:0] fw update target, [7] indicate last packet
	* Byte 1-4: offset, lsb first
	* Byte 5-6: length, lsb first
	* Byte 7-N: data
	***********************************/
	if (msg->data_len < 8) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t target = msg->data[0];
	uint8_t status = -1;
	uint32_t offset =
		((msg->data[4] << 24) | (msg->data[3] << 16) | (msg->data[2] << 8) | msg->data[1]);
	uint16_t length = ((msg->data[6] << 8) | msg->data[5]);

	if ((length == 0) || (length != msg->data_len - 7)) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	if ((target == GT_COMPNT_BIC) || (target == (GT_COMPNT_BIC | IS_SECTOR_END_MASK))) {
		// Expect BIC firmware size not bigger than 410k
		if (offset > GT_BIC_UPDATE_MAX_OFFSET) {
			msg->completion_code = CC_PARAM_OUT_OF_RANGE;
			return;
		}
		status = fw_update(offset, length, &msg->data[7], (target & IS_SECTOR_END_MASK),
				   DEVSPI_FMC_CS0);
	} else if (((target & WITHOUT_SENCTOR_END_MASK) == GT_COMPNT_PEX0) ||
		   ((target & WITHOUT_SENCTOR_END_MASK) == GT_COMPNT_PEX1) ||
		   ((target & WITHOUT_SENCTOR_END_MASK) == GT_COMPNT_PEX2) ||
		   ((target & WITHOUT_SENCTOR_END_MASK) == GT_COMPNT_PEX3)) {
		uint8_t flash_sel_pin[4] = { BIC_SEL_FLASH_SW0, BIC_SEL_FLASH_SW1,
					     BIC_SEL_FLASH_SW2, BIC_SEL_FLASH_SW3 };
		uint8_t flash_sel_base = BIC_SEL_FLASH_SW0 - GT_COMPNT_PEX0;
		uint8_t current_sel_pin = 0xFF;

		/* change mux to pex flash */
		for (int i = 0; i < ARRAY_SIZE(flash_sel_pin); i++) {
			if (flash_sel_base + (target & WITHOUT_SENCTOR_END_MASK) ==
			    flash_sel_pin[i]) {
				current_sel_pin = flash_sel_pin[i];
				gpio_set(current_sel_pin, GPIO_HIGH);
			} else {
				gpio_set(flash_sel_pin[i], GPIO_LOW);
			}
		}
		status = fw_update(offset, length, &msg->data[7], (target & IS_SECTOR_END_MASK),
				   DEVSPI_SPI1_CS0);

		if (current_sel_pin != 0xFF) {
			/* change mux to default */
			gpio_set(current_sel_pin, GPIO_LOW);
		}
	} else {
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	msg->data_len = 0;

	switch (status) {
	case FWUPDATE_SUCCESS:
		msg->completion_code = CC_SUCCESS;
		break;
	case FWUPDATE_OUT_OF_HEAP:
		msg->completion_code = CC_LENGTH_EXCEEDED;
		break;
	case FWUPDATE_OVER_LENGTH:
		msg->completion_code = CC_OUT_OF_SPACE;
		break;
	case FWUPDATE_REPEATED_UPDATED:
		msg->completion_code = CC_INVALID_DATA_FIELD;
		break;
	case FWUPDATE_UPDATE_FAIL:
		msg->completion_code = CC_TIMEOUT;
		break;
	case FWUPDATE_ERROR_OFFSET:
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		break;
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}
	if (status != FWUPDATE_SUCCESS) {
		LOG_ERR("firmware (0x%02X) update failed cc: %x", target, msg->completion_code);
	}

	return;
}

void OEM_1S_PEX_FLASH_READ(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 4) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	if (msg->data[0] > 3 || msg->data[0] < 0) {
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		return;
	}

	static const uint8_t flash_sel_pin[4] = { BIC_SEL_FLASH_SW0, BIC_SEL_FLASH_SW1,
						  BIC_SEL_FLASH_SW2, BIC_SEL_FLASH_SW3 };
	uint8_t read_len = msg->data[3];
	uint32_t addr = msg->data[1] | (msg->data[2] << 8);
	const struct device *flash_dev;
	uint8_t current_sel_pin = 0xFF;

	if (read_len > 64) {
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		return;
	}
	/* pull high select pin to active flash and other should keep low*/
	for (int i = 0; i < ARRAY_SIZE(flash_sel_pin); i++) {
		if (msg->data[0] == i) {
			current_sel_pin = flash_sel_pin[i];
			gpio_set(current_sel_pin, GPIO_HIGH);
		} else {
			gpio_set(flash_sel_pin[i], GPIO_LOW);
		}
	}

	flash_dev = device_get_binding("spi1_cs0");
	if (!flash_dev)
		goto exit;
	/* Due to the SPI in this project has mux so call this function to re-init*/
	if (spi_nor_re_init(flash_dev))
		goto exit;

	if (flash_read(flash_dev, addr, &msg->data[0], read_len) != 0)
		goto exit;

	if (current_sel_pin != 0xFF)
		gpio_set(current_sel_pin, GPIO_LOW);

	msg->data_len = read_len;
	msg->completion_code = CC_SUCCESS;
	return;

exit:
	if (current_sel_pin != 0xFF)
		gpio_set(current_sel_pin, GPIO_LOW);
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

void OEM_1S_GET_FPGA_USER_CODE(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t buffer[4] = { 0 };
	uint8_t ir_value = 0xc0;
	uint8_t dr_value = 0x00;
	const struct device *jtag_dev;

	jtag_dev = device_get_binding("JTAG0");

	if (!jtag_dev) {
		LOG_ERR("JTAG device not found");
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	gpio_set(JTAG_BIC_EN, GPIO_HIGH);

	if (jtag_tap_set(jtag_dev, TAP_RESET)) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	k_msleep(10);

	if (jtag_ir_scan(jtag_dev, 8, &ir_value, buffer, TAP_IDLE)) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	if (jtag_dr_scan(jtag_dev, 32, &dr_value, buffer, TAP_IDLE)) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	gpio_set(JTAG_BIC_EN, GPIO_LOW);

	memcpy(msg->data, buffer, 4);
	msg->data_len = 4;
	msg->completion_code = CC_SUCCESS;
}

void APP_GET_SELFTEST_RESULTS(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	EEPROM_ENTRY fru_entry;
	fru_entry.config.dev_id = 0;
	fru_entry.offset = 0;
	fru_entry.data_len = 8;
	fru_entry.config.mux_present = true;
	fru_entry.config.mux_addr = SWB_FRU_MUX_ADDR;
	fru_entry.config.mux_channel = SWB_FRU_MUX_CHAN;
	FRU_read(&fru_entry);
	uint8_t checksum = 0;
	for (uint8_t i = 0; i < fru_entry.data_len; i++) {
		checksum += fru_entry.data[i];
	}

	SELF_TEST_RESULT res;

	res.result.opFwCorrupt = 0;
	res.result.updateFwCorrupt = 0;
	res.result.sdrRepoEmpty = is_sdr_not_init;
	res.result.ipmbLinesDead = 0;

	if (checksum == 0) {
		res.result.cannotAccessBmcFruDev = 0;
		res.result.internalCorrupt = 0;
	} else {
		res.result.cannotAccessBmcFruDev = 1;
		res.result.internalCorrupt = 1;
	}

	res.result.cannotAccessSdrRepo = is_sdr_not_init;
	res.result.cannotAccessSelDev = 0;

	memcpy(&msg->data[1], &res.result, 1);
	// 55h = No error, 57h = Corrupted or inaccessible data or devices
	msg->data[0] = (msg->data[1] == 0x00) ? 0x55 : 0x57;
	msg->data_len = 2;
	msg->completion_code = CC_SUCCESS;

	return;
}

void OEM_1S_GET_FW_VERSION(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t component;
	ipmi_msg bridge_msg = { 0 };
	component = msg->data[0];

	if (component >= GT_COMPNT_MAX) {
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		return;
	}
	/* 
   * Return data format: 
   * data[0] = component id
   * data[1] = data length
   * data[2] - data[data length + 1] = firmware version
   */
	switch (component) {
	case GT_COMPNT_BIC:
		msg->data[0] = GT_COMPNT_BIC;
		msg->data[1] = 7;
		msg->data[2] = BIC_FW_YEAR_MSB;
		msg->data[3] = BIC_FW_YEAR_LSB;
		msg->data[4] = BIC_FW_WEEK;
		msg->data[5] = BIC_FW_VER;
		msg->data[6] = BIC_FW_platform_0;
		msg->data[7] = BIC_FW_platform_1;
		msg->data[8] = BIC_FW_platform_2;
		msg->data_len = 9;
		msg->completion_code = CC_SUCCESS;
		break;
	case GT_COMPNT_PEX0:
	case GT_COMPNT_PEX1:
	case GT_COMPNT_PEX2:
	case GT_COMPNT_PEX3: {
		/* Only can be read when DC is on */
		if (is_mb_dc_on() == false) {
			msg->completion_code = CC_PEX_NOT_POWER_ON;
			return;
		}
		const uint8_t pex_sensor_num_table[PEX_MAX_NUMBER] = { SENSOR_NUM_TEMP_PEX_0,
								       SENSOR_NUM_TEMP_PEX_1,
								       SENSOR_NUM_TEMP_PEX_2,
								       SENSOR_NUM_TEMP_PEX_3 };
		/* Physical Layer User Test Patterns, Byte 0 Register */
		int reading = 0x6080020c;

		uint8_t pex_sensor_num = pex_sensor_num_table[component - GT_COMPNT_PEX0];
		sensor_cfg *cfg = &sensor_config[sensor_config_index_map[pex_sensor_num]];
		pex89000_unit *p = (pex89000_unit *)cfg->priv_data;

		if (cfg->pre_sensor_read_hook) {
			if (cfg->pre_sensor_read_hook(cfg, cfg->pre_sensor_read_args) == false) {
				LOG_ERR("PEX%d pre-read failed!", component - GT_COMPNT_PEX0);
				msg->completion_code = CC_PEX_PRE_READING_FAIL;
				return;
			}
		}

		if (pex_access_engine(cfg->port, cfg->target_addr, p->idx, pex_access_register,
				      &reading)) {
			if (k_mutex_unlock(&i2c_bus10_mutex))
				LOG_ERR("pex%d mutex unlock failed on bus(%d)", p->idx, cfg->port);
			msg->completion_code = CC_PEX_ACCESS_FAIL;
			return;
		}

		/* Change version register to SBR because the old PEX firmware did not fill in version information at register 0x6080020c yet */
		if (((reading & 0xFF) == (component - GT_COMPNT_PEX0)) &&
		    ((reading >> 8) & 0xFF) == 0xCC) {
			if (pex_access_engine(cfg->port, cfg->target_addr, p->idx,
					      pex_access_sbr_ver, &reading)) {
				if (k_mutex_unlock(&i2c_bus10_mutex))
					LOG_ERR("pex%d mutex unlock failed on bus(%d)", p->idx,
						cfg->port);
				msg->completion_code = CC_PEX_ACCESS_FAIL;
				return;
			}
		}

		if (cfg->post_sensor_read_hook) {
			if (cfg->post_sensor_read_hook(cfg, cfg->post_sensor_read_args, NULL) ==
			    false) {
				LOG_ERR("PEX%d post-read failed!", component - GT_COMPNT_PEX0);
			}
		}

		memcpy(&msg->data[2], &reading, sizeof(reading));

		msg->data[0] = component;
		msg->data[1] = sizeof(reading);
		msg->data_len = sizeof(reading) + 2;
		msg->completion_code = bridge_msg.completion_code;
		break;
	}
	case GT_COMPNT_CPLD:
		bridge_msg.data_len = 0;
		OEM_1S_GET_FPGA_USER_CODE(&bridge_msg);
		memcpy(&msg->data[2], &bridge_msg.data[0], bridge_msg.data_len);
		msg->data[0] = component;
		msg->data[1] = bridge_msg.data_len;
		msg->data_len = bridge_msg.data_len + 2;
		msg->completion_code = bridge_msg.completion_code;
		break;
	case GT_COMPNT_VR0:
	case GT_COMPNT_VR1: {
		I2C_MSG i2c_msg = { 0 };
		uint8_t retry = 3;
		uint8_t buf[5] = { 0 };
		/* Assign VR 0/1 related sensor number to get information for accessing VR */
		uint8_t sensor_num = (component == GT_COMPNT_VR0) ? SENSOR_NUM_PEX_0_VR_TEMP :
								    SENSOR_NUM_PEX_2_VR_TEMP;
		sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];

		if (!tca9548_select_chan(cfg, &mux_conf_addr_0xe0[6])) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}
		/* Get bus and target address by sensor number in sensor configuration */
		i2c_msg.bus = cfg->port;
		i2c_msg.target_addr = cfg->target_addr;

		/* Write to PMBus command DMAADDR (0xC7) and configuration id address is 0x3F*/
		i2c_msg.tx_len = 3;
		i2c_msg.data[0] = 0xC7;
		i2c_msg.data[1] = 0x3F;
		i2c_msg.data[2] = 0x00;

		if (i2c_master_write(&i2c_msg, retry)) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}
		/* Read DMA data register */
		i2c_msg.tx_len = 2;
		i2c_msg.rx_len = 4;
		i2c_msg.data[0] = 0xC5;
		i2c_msg.data[1] = 0xC1;

		if (i2c_master_read(&i2c_msg, retry)) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}

		memcpy(buf, i2c_msg.data, 4);

		/* Write to PMBus command DMAADDR (0xC7) and number of available NVM slots address is 0xC2 */
		i2c_msg.tx_len = 3;
		i2c_msg.data[0] = 0xC7;
		i2c_msg.data[1] = 0xC2;
		i2c_msg.data[2] = 0x00;

		if (i2c_master_write(&i2c_msg, retry)) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}

		/* Read DMA data register */
		i2c_msg.tx_len = 2;
		i2c_msg.rx_len = 4;
		i2c_msg.data[0] = 0xC5;
		i2c_msg.data[1] = 0xC1;

		if (i2c_master_read(&i2c_msg, retry)) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}

		memcpy(&buf[4], i2c_msg.data, 1);

		msg->data[0] = component;
		msg->data[1] = 5;
		memcpy(&msg->data[2], &buf[0], sizeof(buf));
		msg->data_len = 7;
		msg->completion_code = CC_SUCCESS;
		break;
	};
	case GT_COMPNT_NIC0:
	case GT_COMPNT_NIC1:
	case GT_COMPNT_NIC2:
	case GT_COMPNT_NIC3:
	case GT_COMPNT_NIC4:
	case GT_COMPNT_NIC5:
	case GT_COMPNT_NIC6:
	case GT_COMPNT_NIC7: {
		uint8_t idx = component - GT_COMPNT_NIC0;

		if (!nic_vesion[idx].ptr) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}

		memcpy(&msg->data[2], nic_vesion[idx].ptr, nic_vesion[idx].length);

		msg->data[0] = component;
		msg->data[1] = nic_vesion[idx].length;
		msg->data_len = nic_vesion[idx].length + 2;
		msg->completion_code = CC_SUCCESS;
		break;
	}
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}
	return;
}

void OEM_1S_BRIDGE_I2C_MSG_BY_COMPNT(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len < 2) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t compnt_id = msg->data[0];
	uint8_t i;
	uint8_t retry = 5;
	I2C_MSG i2c_msg = { 0 };
	ipmi_msg bridge_msg = { 0 };
	struct bridge_compnt_info_s *p;

	for (i = 0; i < ARRAY_SIZE(bridge_compnt_info); i++) {
		p = bridge_compnt_info + i;

		if (p->compnt_id == compnt_id)
			break;
	}

	if (i == ARRAY_SIZE(bridge_compnt_info)) {
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	if (p->bus_mutex) {
		if (k_mutex_lock(p->bus_mutex, K_MSEC(100))) {
			LOG_ERR("The mutex lock failed on the bus(%d)", p->i2c_bus);
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}
	}

	if (p->mux_present) {
		i2c_msg.bus = p->i2c_bus;
		i2c_msg.target_addr = p->mux_addr >> 1;
		i2c_msg.tx_len = 1;
		i2c_msg.data[0] = (1 << p->mux_channel);

		if (i2c_master_write(&i2c_msg, retry)) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			LOG_ERR("Access to the I2C MUX failed on the bus(%d)", p->i2c_bus);
			goto unlock_exit;
		}
	}

	bridge_msg.data[0] = (p->i2c_bus << 1);
	bridge_msg.data[1] = (p->i2c_addr << 1);
	bridge_msg.data[2] = msg->data[1];
	memcpy(&bridge_msg.data[2], &msg->data[1], msg->data_len - 1);
	bridge_msg.data_len = msg->data_len + 1;

	APP_MASTER_WRITE_READ(&bridge_msg);

	msg->completion_code = bridge_msg.completion_code;
	msg->data_len = bridge_msg.data_len;
	memcpy(&msg->data[0], &bridge_msg.data[0], bridge_msg.data_len);

unlock_exit:
	if (p->bus_mutex) {
		if (k_mutex_unlock(p->bus_mutex))
			LOG_ERR("The mutex unlock failed on the bus(%d)", p->i2c_bus);
	}
	return;
}
