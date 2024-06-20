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
#include "plat_def.h"
#ifdef ENABLE_DS160PT801

#include <stdlib.h>
#include <logging/log.h>
#include "ds160pt801.h"
#include "util_spi.h"
#include "hal_i2c.h"
#include "libutil.h"
#include "sensor.h"

LOG_MODULE_REGISTER(dev_ds160pt801);

K_MUTEX_DEFINE(ds106pt801_mutex);

struct ds160pt801_ascii_data {
	uint8_t head_buf[9]; // ':' + 4bytes hex header
	uint8_t data_buf[64]; // max 32bytes hex data
	uint8_t checksum[2]; // 1bytes hex checksum
} __attribute__((packed));

typedef struct ds160pt801_hex_data {
	struct _head_buf { // 4bytes hex header
		uint8_t byte_cnt;
		uint16_t load_ofst;
		uint8_t rec_type;
	} head_buf;
	uint8_t data_buf[32]; // max 32bytes hex data
	uint8_t checksum; // 1bytes hex checksum
} ds160pt801_hex_data_t;

enum rec_type {
	HEADER_DATA_RECORD = 0x00,
	HEADER_END_OF_FILE = 0x01,
	HEADER_EXT_SEG_ADDR = 0x02,
	HEADER_START_SEG_ADDR = 0x03,
	HEADER_EXT_LIN_ADDR = 0x04,
	HEADER_START_LIN_ADDR = 0x05,
};

#define EEPROM_DATA_SZ_PER_TIME 8
#define DATA_PER_LINE (sizeof(struct ds160pt801_ascii_data) + 2)
#define MAX_LINE_CNT_PER_TIME 5

#ifndef RETIMER_EEPROM_ADDR
#define RETIMER_EEPROM_ADDR 0xA0 //8bit address
#define RETIMER_EEPROM_RD_ADDR 0xA1
#endif

/* TI VENDOR ID */
#define TI_VENDOR_ID 0x4172

/* DS160PT801 DEVICE ID */
#define DS160PT801_ID 0x24

/* ADDR MODE SWITCH */
#define ADDR_MODE_16_TO_8 0x0168
#define ADDR_MODE_8_TO_16 0xF9

enum {
	FROM_16_TO_8 = 0, //default
	FROM_8_TO_16 = 1,
};

/* --------- GLOBAL ---------- */
#define GLOBAL_REG_DEV_REV 0xF0
#define GLOBAL_REG_DEV_ID 0xF1
#define GLOBAL_REG_VEND_ID 0xF6
#define GLOBAL_REG_FUND_RST 0xFA
#define GLOBAL_REG_CHAN_SEL 0xFC
#define GLOBAL_REG_BLOCK_SEL 0xFF

/* --------- SHARE ---------- */
#define SHARE_REG_EEPROM_CTL 0x18
#define SHARE_REG_EEPROM_ADDR 0x19
#define SHARE_REG_EEPROM_OFST_LO 0x1A
#define SHARE_REG_EEPROM_OFST_HI 0x1B
#define SHARE_REG_EEPROM_BUFF 0x1C
#define SHARE_REG_EEPROM_CRC_CHECK 0x1D
#define SHARE_REG_EEPROM_DETECT 0x1E
#define SHARE_REG_EEPROM_STATE 0x1F
#define SHARE_RET_EEPROM_VER 0xDA

#define SHARE_REG_TEMP_CTL 0x78
#define SHARE_REG_TEMP 0x79
#define SHARE_REG_TEMP_NOM 0x7E

enum {
	EEPROM_I2C_SPEED_10K,
	EEPROM_I2C_SPEED_100K,
	EEPROM_I2C_SPEED_400K,
};

#define EEPROM_CTL_DEFAULT (BIT(6) | (EEPROM_I2C_SPEED_400K << 2))

static bool is_update_ongoing = false;

static bool ds160pt801_switch_addr_mode(I2C_MSG *i2c_msg, uint8_t mode)
{
	CHECK_NULL_ARG_WITH_RETURN(i2c_msg, false);

	switch (mode) {
	case FROM_16_TO_8:
		i2c_msg->data[0] = ADDR_MODE_16_TO_8 & 0xFF;
		i2c_msg->data[1] = (ADDR_MODE_16_TO_8 >> 8) & 0xFF;
		i2c_msg->data[2] = 0x00;
		i2c_msg->tx_len = 3;
		break;

	case FROM_8_TO_16:
		i2c_msg->data[0] = ADDR_MODE_8_TO_16;
		i2c_msg->data[1] = 0x01;
		i2c_msg->tx_len = 2;
		break;

	default:
		break;
	}

	if (i2c_master_write(i2c_msg, 3)) {
		LOG_ERR("Failed to switch ADDR mode %d", mode);
		return false;
	}

	return true;
}

bool ds160pt801_get_fw_version(I2C_MSG *i2c_msg, uint8_t *version)
{
	CHECK_NULL_ARG_WITH_RETURN(i2c_msg, false);
	CHECK_NULL_ARG_WITH_RETURN(version, false);

	bool ret = false;

	if (k_mutex_lock(&ds106pt801_mutex, K_MSEC(1000))) {
		LOG_ERR("Mutex lock failed");
		return false;
	}

	if (ds160pt801_switch_addr_mode(i2c_msg, FROM_16_TO_8) == false) {
		LOG_ERR("Failed to switch ADDR mode from 16bit to 8bit");
		return false;
	}

	i2c_msg->data[0] = SHARE_RET_EEPROM_VER;
	i2c_msg->tx_len = 1;
	i2c_msg->rx_len = 1;

	if (i2c_master_read(i2c_msg, 3)) {
		LOG_ERR("Failed to read REVISION register");
		goto unlock_exit;
	}

	*version = i2c_msg->data[0];

	ret = true;
unlock_exit:
	if (k_mutex_unlock(&ds106pt801_mutex)) {
		LOG_ERR("Mutex unlock failed");
	}

	return ret;
}

static bool ds160pt801_read_junction_temp(I2C_MSG *i2c_msg, double *avg_temperature)
{
	CHECK_NULL_ARG_WITH_RETURN(i2c_msg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(avg_temperature, SENSOR_UNSPECIFIED_ERROR);

	uint8_t tmp_byte = 0;
	bool ret = false;

	if (k_mutex_lock(&ds106pt801_mutex, K_MSEC(1000))) {
		LOG_ERR("Mutex lock failed");
		return false;
	}

	/* Step1. Enable DIE0 shared register set */
	i2c_msg->data[0] = GLOBAL_REG_BLOCK_SEL;
	i2c_msg->tx_len = 1;
	i2c_msg->rx_len = 1;

	if (i2c_master_read(i2c_msg, 3)) {
		LOG_ERR("Failed to read BLOCK SEL register");
		goto unlock_exit;
	}
	tmp_byte = i2c_msg->data[0];

	tmp_byte |= BIT(6);
	i2c_msg->data[0] = GLOBAL_REG_BLOCK_SEL;
	i2c_msg->data[1] = tmp_byte;
	i2c_msg->tx_len = 2;
	i2c_msg->rx_len = 0;

	if (i2c_master_write(i2c_msg, 3)) {
		LOG_ERR("Failed to enable die0");
		goto unlock_exit;
	}

	/* Step2. Freeze capture */
	i2c_msg->data[0] = SHARE_REG_TEMP_CTL;
	i2c_msg->tx_len = 1;
	i2c_msg->rx_len = 1;

	if (i2c_master_read(i2c_msg, 3)) {
		LOG_ERR("Failed to read TEMP CTL register");
		goto unlock_exit;
	}
	tmp_byte = i2c_msg->data[0];

	tmp_byte &= ~BIT(1);
	i2c_msg->data[0] = SHARE_REG_TEMP_CTL;
	i2c_msg->data[1] = tmp_byte;
	i2c_msg->tx_len = 2;
	i2c_msg->rx_len = 0;

	if (i2c_master_write(i2c_msg, 3)) {
		LOG_ERR("Failed to freeze capture");
		goto unlock_exit;
	}

	/* Step3. Disable temp sensor */
	tmp_byte &= ~BIT(2);
	i2c_msg->data[0] = SHARE_REG_TEMP_CTL;
	i2c_msg->data[1] = tmp_byte;
	i2c_msg->tx_len = 2;
	i2c_msg->rx_len = 0;

	if (i2c_master_write(i2c_msg, 3)) {
		LOG_ERR("Failed to disable temp sensor");
		goto unlock_exit;
	}

	/* Step4. Enable temp sensor */
	tmp_byte |= BIT(2);
	i2c_msg->data[0] = SHARE_REG_TEMP_CTL;
	i2c_msg->data[1] = tmp_byte;
	i2c_msg->tx_len = 2;
	i2c_msg->rx_len = 0;

	if (i2c_master_write(i2c_msg, 3)) {
		LOG_ERR("Failed to enable temp sensor");
		goto unlock_exit;
	}

	/* Step5. Reset temp sensor */
	tmp_byte &= ~BIT(0);
	i2c_msg->data[0] = SHARE_REG_TEMP_CTL;
	i2c_msg->data[1] = tmp_byte;
	i2c_msg->tx_len = 2;
	i2c_msg->rx_len = 0;

	if (i2c_master_write(i2c_msg, 3)) {
		LOG_ERR("Failed to reset temp sensor");
		goto unlock_exit;
	}

	/* Step6. Unfreeze capture */
	tmp_byte |= BIT(1);
	i2c_msg->data[0] = SHARE_REG_TEMP_CTL;
	i2c_msg->data[1] = tmp_byte;
	i2c_msg->tx_len = 2;
	i2c_msg->rx_len = 0;

	if (i2c_master_write(i2c_msg, 3)) {
		LOG_ERR("Failed to unfreeze capture");
		goto unlock_exit;
	}

	/* Step7. Read TEMP_NOM */
	i2c_msg->data[0] = SHARE_REG_TEMP_NOM;
	i2c_msg->tx_len = 1;
	i2c_msg->rx_len = 1;

	if (i2c_master_read(i2c_msg, 3)) {
		LOG_ERR("Failed to unfreeze capture");
		goto unlock_exit;
	}

	*avg_temperature = i2c_msg->data[0];

	/* Step8. Read TEMP */
	i2c_msg->data[0] = SHARE_REG_TEMP;
	i2c_msg->tx_len = 1;
	i2c_msg->rx_len = 1;

	if (i2c_master_read(i2c_msg, 3)) {
		LOG_ERR("Failed to unfreeze capture");
		goto unlock_exit;
	}

	*avg_temperature = 1.56 * (i2c_msg->data[0] - *avg_temperature) + 28.5;

	ret = true;
unlock_exit:
	if (k_mutex_unlock(&ds106pt801_mutex)) {
		LOG_ERR("Mutex unlock failed");
	}

	return ret;
}

bool ds160pt801_read_data(I2C_MSG *msg, uint16_t offset, uint8_t *rxbuf, uint16_t length)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);
	CHECK_NULL_ARG_WITH_RETURN(rxbuf, false);

	/* should have 8 byte to load in one time */
	if (length != 8) {
		LOG_WRN("Invalid data length %d", length);
		return false;
	}

	/* set EEPROM address config */

	msg->data[0] = SHARE_REG_EEPROM_ADDR;
	msg->data[1] = RETIMER_EEPROM_RD_ADDR;
	msg->tx_len = 2;
	if (i2c_master_write(msg, 3)) {
		LOG_ERR("Failed to set EEPROM address");
		return false;
	}

	/* write low byte offset */
	msg->data[0] = SHARE_REG_EEPROM_OFST_LO;
	msg->data[1] = offset & 0xFF;
	msg->tx_len = 2;
	if (i2c_master_write(msg, 3)) {
		LOG_ERR("Failed to write low byte offset");
		return false;
	}

	/* write high byte offset */
	msg->data[0] = SHARE_REG_EEPROM_OFST_HI;
	msg->data[1] = (offset >> 8) & 0xFF;
	msg->tx_len = 2;
	if (i2c_master_write(msg, 3)) {
		LOG_ERR("Failed to write high byte offset");
		return false;
	}

	/* trigger EEPROM read */
	msg->data[0] = SHARE_REG_EEPROM_CTL;
	msg->data[1] = 0x01;
	msg->tx_len = 2;
	if (i2c_master_write(msg, 3)) {
		LOG_ERR("Failed to trigger EEPROM read");
		return false;
	}

	k_msleep(50);

	/* reset EEPROM trigger */
	msg->data[0] = SHARE_REG_EEPROM_CTL;
	msg->data[1] = 0x00;
	msg->tx_len = 2;
	if (i2c_master_write(msg, 3)) {
		LOG_ERR("Failed to reset EEPROM trigger");
		return false;
	}

	k_msleep(50);

	/* read data */
	for (int i = 0; i < length; i++) {
		msg->data[0] = SHARE_REG_EEPROM_BUFF;
		msg->tx_len = 1;
		msg->rx_len = 1;
		if (i2c_master_read(msg, 3)) {
			LOG_ERR("Failed to data data [byte %d] at offset 0x%x", i, offset);
			return false;
		}
		rxbuf[i] = msg->data[0];
	}

	return true;
}

static bool ds160pt801_upload_data(I2C_MSG *msg, uint16_t offset, uint8_t *txbuf, uint16_t length)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);
	CHECK_NULL_ARG_WITH_RETURN(txbuf, false);

	/* should have 8 byte to load in one time */
	if (length != 8) {
		LOG_WRN("Invalid data length %d", length);
		return false;
	}

	LOG_DBG("Write data to EEPROM offset 0x%x", offset);
	LOG_HEXDUMP_DBG(txbuf, length, "Write data");

	/* set EEPROM address config */
	msg->data[0] = SHARE_REG_EEPROM_ADDR;
	msg->data[1] = RETIMER_EEPROM_ADDR;
	msg->tx_len = 2;
	if (i2c_master_write(msg, 3)) {
		LOG_ERR("Failed to set EEPROM address");
		return false;
	}

	/* write low byte offset */
	msg->data[0] = SHARE_REG_EEPROM_OFST_LO;
	msg->data[1] = offset & 0xFF;
	msg->tx_len = 2;
	if (i2c_master_write(msg, 3)) {
		LOG_ERR("Failed to write low byte offset");
		return false;
	}

	/* write high byte offset */
	msg->data[0] = SHARE_REG_EEPROM_OFST_HI;
	msg->data[1] = (offset >> 8) & 0xFF;
	msg->tx_len = 2;
	if (i2c_master_write(msg, 3)) {
		LOG_ERR("Failed to write high byte offset");
		return false;
	}

	/* write data */
	for (int i = 0; i < length; i++) {
		msg->data[0] = SHARE_REG_EEPROM_BUFF;
		msg->data[1] = txbuf[i];
		msg->tx_len = 2;
		if (i2c_master_write(msg, 3)) {
			LOG_ERR("Failed to write data [byte %d]0x%x at offset 0x%x", i, txbuf[i],
				offset);
			return false;
		}
	}

	int try = 0, retry = 3;
	for (try = 0; try < retry; try++) {
		/* check data write success */
		msg->data[0] = SHARE_REG_EEPROM_CTL;
		msg->tx_len = 1;
		msg->rx_len = 1;
		if (i2c_master_read(msg, 3)) {
			LOG_ERR("Failed to check EEPROM status");
			return false;
		}

		if ((msg->data[0] & BIT(5)) && (msg->data[0] & BIT(7))) {
			break;
		}

		LOG_WRN("Got unexpected EEPROM status 0x%x", msg->data[0]);
		k_msleep(50);
	}

	if (try == retry) {
		LOG_ERR("EEPROM status check failed");
		return false;
	}

	/* trigger EEPROM write */
	msg->data[0] = SHARE_REG_EEPROM_CTL;
	msg->data[1] = EEPROM_CTL_DEFAULT | BIT(0);
	msg->tx_len = 2;
	if (i2c_master_write(msg, 3)) {
		LOG_ERR("Failed to trigger EEPROM write");
		return false;
	}

	/* reset EEPROM trigger */
	msg->data[0] = SHARE_REG_EEPROM_CTL;
	msg->data[1] = EEPROM_CTL_DEFAULT;
	msg->tx_len = 2;
	if (i2c_master_write(msg, 3)) {
		LOG_ERR("Failed to reset EEPROM trigger");
		return false;
	}

	k_msleep(50);
	msg->data[0] = SHARE_REG_EEPROM_CTL;
	msg->tx_len = 1;
	msg->rx_len = 1;
	if (i2c_master_write(msg, 3)) {
		LOG_ERR("Failed to resd EEPROM stat");
		return false;
	}

	uint8_t read_buf[8] = { 0 };
	if (ds160pt801_read_data(msg, offset, read_buf, sizeof(read_buf)) == false) {
		LOG_ERR("Failed to read data after write");
		return false;
	}

	for (int i = 0; i < sizeof(read_buf); i++) {
		if (read_buf[i] != txbuf[i]) {
			LOG_ERR("Data mismatch at offset 0x%x", offset + i);
			return false;
		}
	}

	return true;
}

static bool ds160pt801_pre_update(I2C_MSG *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);

	/* Enable 8bit ADDR mode */
	if (ds160pt801_switch_addr_mode(msg, FROM_16_TO_8) == false) {
		LOG_ERR("Failed to switch ADDR mode from 16bit to 8bit");
		return false;
	}

	/* Show vendor info */
	msg->data[0] = GLOBAL_REG_VEND_ID;
	msg->tx_len = 1;
	msg->rx_len = 2;
	if (i2c_master_read(msg, 3)) {
		LOG_ERR("Failed to read device ID");
		return false;
	}

	uint16_t vendor_id = msg->data[0] | (msg->data[1] << 8);
	if (vendor_id != TI_VENDOR_ID) {
		LOG_ERR("Get invalid vendor id 0x%x", vendor_id);
		return false;
	}
	LOG_INF("* Vendor ID: 0x%x", msg->data[0] | (msg->data[1] << 8));

	/* Show device info */
	msg->data[0] = GLOBAL_REG_DEV_ID;
	msg->tx_len = 1;
	msg->rx_len = 1;
	if (i2c_master_read(msg, 3)) {
		LOG_ERR("Failed to read device ID");
		return false;
	}
	if (msg->data[0] != DS160PT801_ID) {
		LOG_ERR("Get invalid device id 0x%x", msg->data[0]);
		return false;
	}
	LOG_INF("* Device ID: 0x%x", msg->data[0]);

	/* Show fw revision */
	uint8_t fw_version = 0;
	if (ds160pt801_get_fw_version(msg, &fw_version) == false) {
		LOG_ERR("Failed to get firmware version");
		return false;
	}
	LOG_INF("* Firmware version: 0x%x", fw_version);

	/* deselect every channel */
	msg->data[0] = GLOBAL_REG_CHAN_SEL;
	msg->data[1] = 0x00;
	msg->data[2] = 0x00;
	msg->tx_len = 3;
	if (i2c_master_write(msg, 3)) {
		LOG_ERR("Failed to deslect every channel");
		return false;
	}

	/* select die0 share register */
	msg->data[0] = GLOBAL_REG_BLOCK_SEL;
	msg->data[1] = 0x40;
	msg->tx_len = 2;
	if (i2c_master_write(msg, 3)) {
		LOG_ERR("Failed to select die1 share register");
		return false;
	}

	/* check EEPROM at expected address */
	msg->data[0] = SHARE_REG_EEPROM_DETECT;
	msg->tx_len = 1;
	msg->rx_len = 1;
	if (i2c_master_read(msg, 3)) {
		LOG_ERR("Failed to detect EEPROM");
		return false;
	}

	if (msg->data[0] != RETIMER_EEPROM_ADDR) {
		LOG_WRN("Detect unexpected EEPROM addr 0x%x", msg->data[0]);

		/* set EEPROM address config */
		msg->data[0] = SHARE_REG_EEPROM_ADDR;
		msg->data[1] = RETIMER_EEPROM_ADDR;
		msg->tx_len = 2;
		if (i2c_master_write(msg, 3)) {
			LOG_ERR("Failed to set EEPROM address");
			return false;
		}
	}

	/* set EEPROM present */
	msg->data[0] = SHARE_REG_EEPROM_STATE;
	msg->data[1] = 0x20;
	msg->tx_len = 2;
	if (i2c_master_write(msg, 3)) {
		LOG_ERR("Failed to set EEPROM present");
		return false;
	}

	/* set EEPROM i2c speed */
	msg->data[0] = SHARE_REG_EEPROM_CTL;
	msg->data[1] = EEPROM_CTL_DEFAULT;
	msg->tx_len = 2;
	if (i2c_master_write(msg, 3)) {
		LOG_ERR("Failed to set EEPROM i2c speed");
		return false;
	}

	return true;
}

static bool ds160pt801_do_update(I2C_MSG *msg, uint16_t update_offset, uint8_t *txbuf,
				 uint16_t length, bool is_end)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);
	CHECK_NULL_ARG_WITH_RETURN(txbuf, false);

	if (update_offset == 0) {
		is_update_ongoing = true;
		if (!ds160pt801_pre_update(msg)) {
			LOG_ERR("Failed to do pre-update");
			return false;
		}
	}

	while (length) {
		if (length < EEPROM_DATA_SZ_PER_TIME) {
			LOG_WRN("Get data length %d is less than 8", length);
			length = 0;
			break;
		}

		if (ds160pt801_upload_data(msg, update_offset, txbuf, EEPROM_DATA_SZ_PER_TIME) ==
		    false)
			return false;

		length -= EEPROM_DATA_SZ_PER_TIME;
		txbuf += EEPROM_DATA_SZ_PER_TIME;
		update_offset += EEPROM_DATA_SZ_PER_TIME;
	}

	if (is_end)
		is_update_ongoing = false;

	return true;
}

static bool ds160pt801_parsing(uint8_t *txbuf, uint32_t length, ds160pt801_hex_data_t *line_data,
			       uint16_t *line_len)
{
	CHECK_NULL_ARG_WITH_RETURN(txbuf, false);
	CHECK_NULL_ARG_WITH_RETURN(line_data, false);
	CHECK_NULL_ARG_WITH_RETURN(line_len, false);

	*line_len = 0;

	int hi_val = 0;
	int lo_val = 0;

	for (int i = 0; i < length; i++) {
		if (((txbuf[i] == 0x0d) || txbuf[i] == 0x0a)) {
			// pass '\n'
			continue;
		}

		if (txbuf[i] != 0x3a) {
			LOG_ERR("Invlaid line header key %x, should be [:]", txbuf[i]);
			return false;
		}

		if ((i + 8) >= length) {
			LOG_ERR("Invlaid line header length, should be [:xxxxxxxx]");
			return false;
		}

		i += 1; //skip ':'
		line_data[*line_len].head_buf.byte_cnt =
			ascii_to_val(txbuf[i]) * 16 + ascii_to_val(txbuf[i + 1]);
		i += 2;
		uint8_t load_ofst_hi = ascii_to_val(txbuf[i]) * 16 + ascii_to_val(txbuf[i + 1]);
		uint8_t load_ofst_lo = ascii_to_val(txbuf[i + 2]) * 16 + ascii_to_val(txbuf[i + 3]);
		line_data[*line_len].head_buf.load_ofst = (load_ofst_hi << 8) | load_ofst_lo;
		i += 4;
		line_data[*line_len].head_buf.rec_type =
			ascii_to_val(txbuf[i]) * 16 + ascii_to_val(txbuf[i + 1]);
		i += 2;

		if (line_data[*line_len].head_buf.rec_type == HEADER_END_OF_FILE) {
			return true;
		} else if (line_data[*line_len].head_buf.rec_type == HEADER_DATA_RECORD) {
			if (line_data[*line_len].head_buf.byte_cnt >
			    ARRAY_SIZE(line_data[*line_len].data_buf)) {
				LOG_ERR("Data length over limit %ld",
					ARRAY_SIZE(line_data[*line_len].data_buf));
				return false;
			}

			if ((i + (line_data[*line_len].head_buf.byte_cnt * 2)) >= length) {
				LOG_ERR("Invlaid line data length, should be %d of data bytes",
					line_data[*line_len].head_buf.byte_cnt);
				return false;
			}

			for (int j = 0; j < line_data[*line_len].head_buf.byte_cnt * 2; j += 2) {
				hi_val = ascii_to_val(txbuf[i]);
				lo_val = ascii_to_val(txbuf[i + 1]);
				if (hi_val == -1 || lo_val == -1) {
					LOG_ERR("Got non-hex ascii byte");
					return false;
				}
				line_data[*line_len].data_buf[j / 2] = hi_val * 16 + lo_val;
				i += 2;
			}

			if ((i + 2) >= length) {
				LOG_ERR("Invlaid line checksum length");
				return false;
			}

			line_data[*line_len].checksum =
				ascii_to_val(txbuf[i]) * 16 + ascii_to_val(txbuf[i + 1]);
			i++;

			(*line_len)++;
		} else {
			LOG_ERR("Got unsupported record type 0x%x",
				line_data[*line_len].head_buf.rec_type);
			return false;
		}
	}

	return true;
}

uint8_t ds160pt801_fw_update(I2C_MSG *msg, uint32_t *offset, uint32_t *msg_len, uint8_t *msg_buf,
			     uint8_t flag)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, FWUPDATE_UPDATE_FAIL);
	CHECK_NULL_ARG_WITH_RETURN(offset, FWUPDATE_UPDATE_FAIL);
	CHECK_NULL_ARG_WITH_RETURN(msg_len, FWUPDATE_UPDATE_FAIL);
	CHECK_NULL_ARG_WITH_RETURN(msg_buf, FWUPDATE_UPDATE_FAIL);

	static uint16_t buf_offset = 0;
	uint8_t ret = FWUPDATE_NOT_SUPPORT;
	bool is_end = false;

	uint8_t *txbuf = (uint8_t *)malloc(SECTOR_SZ_256);
	if (!txbuf) { // Retry alloc
		k_msleep(100);
		txbuf = (uint8_t *)malloc(SECTOR_SZ_256);
		if (!txbuf) {
			LOG_ERR("Failed to allocate txbuf at 0x%x.", *offset);
			is_update_ongoing = false;
			return FWUPDATE_OUT_OF_HEAP;
		}
	}

	buf_offset = 0;

	if (*msg_len > SECTOR_SZ_256) {
		LOG_ERR("eeprom offset %x, recv data 0x%x over sector size 0x%x", *offset,
			buf_offset + *msg_len, SECTOR_SZ_256);
		ret = FWUPDATE_OVER_LENGTH;
		goto exit;
	}

	LOG_DBG("update offset %x , msg_len %d, flag 0x%x", *offset, *msg_len, flag);
	LOG_HEXDUMP_DBG(msg_buf, *msg_len, "update data");

	/* remove redundant data and also update next msg_len */
	*msg_len = (*msg_len / DATA_PER_LINE) * DATA_PER_LINE;
	/* update next offset */
	*offset += *msg_len;

	ds160pt801_hex_data_t line_data_list[MAX_LINE_CNT_PER_TIME] = { 0 };
	uint16_t line_cnt = 0;
	if (ds160pt801_parsing(msg_buf, *msg_len, line_data_list, &line_cnt) == false) {
		ret = FWUPDATE_UPDATE_FAIL;
		goto exit;
	}

	memcpy(&txbuf[buf_offset], msg_buf, *msg_len);
	buf_offset += *msg_len;

	if (flag & SECTOR_END_FLAG) {
		is_end = true;
	}

	if (k_mutex_lock(&ds106pt801_mutex, K_FOREVER)) {
		LOG_ERR("pt5161l mutex lock failed");
		ret = FWUPDATE_UPDATE_FAIL;
		goto exit;
	}

	for (int i = 0; i < line_cnt; i++) {
		LOG_DBG("* Offset 0x%x len %d checksum 0x%x", line_data_list[i].head_buf.load_ofst,
			line_data_list[i].head_buf.byte_cnt, line_data_list[i].checksum);
		LOG_HEXDUMP_DBG(&line_data_list[i].data_buf, line_data_list[i].head_buf.byte_cnt,
				"hex data");

		if (ds160pt801_do_update(msg, line_data_list[i].head_buf.load_ofst,
					 line_data_list[i].data_buf,
					 line_data_list[i].head_buf.byte_cnt, is_end) == false) {
			LOG_ERR("Failed to update DS160PT801 retimer");
			ret = FWUPDATE_UPDATE_FAIL;
			goto exit;
		}
	}

	ret = FWUPDATE_SUCCESS;
	k_msleep(10);

exit:
	if (ret != FWUPDATE_SUCCESS) {
		is_update_ongoing = false;
	}

	/* cover some scenarios couldn't be handled */
	if (is_end) {
		is_update_ongoing = false;
	}

	SAFE_FREE(txbuf);
	if (k_mutex_unlock(&ds106pt801_mutex)) {
		LOG_WRN("pt5161l mutex unlock failed");
	}

	return ret;
}

uint8_t ds160pt801_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	if (is_update_ongoing) {
		LOG_WRN("Firmware update is ongoing, skip reading sensor");
		return SENSOR_NOT_ACCESSIBLE;
	}

	I2C_MSG msg = { 0 };
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;

	if (ds160pt801_switch_addr_mode(&msg, FROM_16_TO_8) == false) {
		LOG_ERR("Failed to switch ADDR mode from 16bit to 8bit");
		return SENSOR_FAIL_TO_ACCESS;
	}

	double val = 0;

	switch (cfg->offset) {
	case DS160PT801_READ_TEMP:
		if (ds160pt801_read_junction_temp(&msg, &val) == false) {
			LOG_ERR("Failed to read junction temperature");
			return SENSOR_FAIL_TO_ACCESS;
		}

		break;
	default:
		LOG_ERR("Invalid sensor 0x%x offset 0x%x", cfg->num, cfg->offset);
		return SENSOR_NOT_FOUND;
	}

	sensor_val *sval = (sensor_val *)reading;
	memset(sval, 0, sizeof(*sval));

	sval->integer = (int)val & 0xFFFF;
	sval->fraction = (val - sval->integer) * 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t ds160pt801_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	cfg->read = ds160pt801_read;
	return SENSOR_INIT_SUCCESS;
}

#endif
