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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <sys/crc.h>
#include <logging/log.h>
#include "libutil.h"
#include "sensor.h"
#include "m88rt51632.h"
#include "hal_i2c.h"
#include "util_spi.h"

LOG_MODULE_REGISTER(dev_m88rt51632);
K_MUTEX_DEFINE(m88rt51632_mutex);
static bool is_update_ongoing = false;

static uint8_t cal_crc8_pec(uint8_t *crc_list, uint8_t len)
{
	CHECK_NULL_ARG_WITH_RETURN(crc_list, 0);
	return crc8(crc_list, len, 0x07, 0x00, false);
}

/* 
 *@addr : address to read
 *@data : return data
 */
static bool m88rt51632_smbus_read_byte(I2C_MSG *msg, uint16_t addr, uint32_t *data)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);
	uint8_t retry = 5;
	uint8_t pec_crc, crc_reslut;
	uint8_t crc_list[10];
	int slave_addr = msg->target_addr << 1;
	uint8_t addr_low = addr & 0xFF;
	uint8_t addr_high = addr >> 8;

	// read buffer is 3 + data bytes always
	// First byte is num.bytes read
	// Second and third bytes are address
	// Bytes 4 onwards is data (4 bytes)
	// 1 byte pec
	uint8_t read_buf_len = 3 + 4 + 1;

	msg->data[0] = I2C_READ_CCODE_START;
	msg->data[1] = I2C_READ_BYTCNT;
	msg->data[2] = addr_low;
	msg->data[3] = addr_high;

	crc_list[0] = slave_addr;
	memcpy(&(crc_list[1]), msg->data, 4);

	msg->data[4] = cal_crc8_pec(crc_list, 5);
	msg->tx_len = 5;

	if (i2c_master_write(msg, retry)) {
		LOG_ERR("montage write reading address %x failed", addr);
		return false;
	}

	memset(msg->data, 0, I2C_BUFF_SIZE);
	msg->data[0] = I2C_READ_CCODE_END;
	msg->tx_len = 1;
	msg->rx_len = read_buf_len;
	if (i2c_master_read(msg, retry)) {
		LOG_ERR("montage write i2c read ccode end failed");
		return false;
	}

	if (msg->data[0] != 6) {
		LOG_ERR("Read error!, length: %d", msg->data[0]);
		return false;
	}

	// PEC CRC
	crc_list[0] = slave_addr;
	crc_list[1] = I2C_READ_CCODE_END;
	crc_list[2] = slave_addr + 1;
	memcpy(&(crc_list[3]), msg->data, 7);

	pec_crc = msg->data[7]; // pec value

	crc_reslut = cal_crc8_pec(crc_list, 10);

	if (pec_crc != crc_reslut) {
		LOG_ERR("The read data pec_crc=0x%x is invalid, the right crc value=0x%x please check",
			pec_crc, crc_reslut);
		return false;
	}

	uint32_t reverse_data;
	convert_uint8_t_pointer_to_uint32_t(&reverse_data, &(msg->data[3]), 4, BIG_ENDIAN);
	*data = uint32_t_byte_reverse(reverse_data);

	return true;
}

static bool m88rt51632_smbus_write_byte(I2C_MSG *msg, uint16_t addr, uint32_t data)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);
	uint8_t retry = 3;
	int slave_addr = msg->target_addr << 1;
	uint8_t crc_list[9];
	uint8_t addr_low = addr & 0xFF;
	uint8_t addr_high = addr >> 8;
	uint32_t reverse_data = uint32_t_byte_reverse(data);

	/* Set buffer length based on number of bytes being written
     * 1 bytes i2c write command code
     * 2 bytes of address in calculation
     * 1 extra byte for num bytes in buffer
     * 4 bytes data
     * 1 byte pec
     */
	int write_num_bytes = 9;

	msg->data[0] = I2C_WRITE_CCODE_START;
	msg->data[1] = I2C_WRITE_BYTCNT;
	msg->data[2] = addr_low;
	msg->data[3] = addr_high;
	convert_uint32_t_to_uint8_t_pointer(reverse_data, &(msg->data[4]), 4, BIG_ENDIAN);

	//cal pec
	crc_list[0] = slave_addr;
	memcpy(&(crc_list[1]), msg->data, 8);

	msg->data[8] = cal_crc8_pec(crc_list, 9);
	msg->tx_len = write_num_bytes;

	if (i2c_master_write(msg, retry)) {
		LOG_ERR("montage write address %x data %x %x %x %X pec %xfailed", addr,
			msg->data[4], msg->data[5], msg->data[6], msg->data[7], msg->data[8]);
		return false;
	}

	return true;
}

bool m88rt51632_get_vendor_id(I2C_MSG *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);
	uint8_t retry = 5;
	bool ret = false;

	if (k_mutex_lock(&m88rt51632_mutex, K_MSEC(M88RT51632_MUTEX_LOCK_MS))) {
		LOG_ERR("m88rt51632 mutex lock failed");
		k_msleep(10);
		return false;
	}

	msg->tx_len = 4;
	msg->data[0] = 0x02; //COMMAND CODE = 0x02 END=0, START=1, FUNC=3'b000, PEC=0
	msg->data[1] = 0x02; //byte count
	msg->data[2] = 0x04; //vendor id lower offset
	msg->data[3] = 0x00; //vendor id upper offset

	if (i2c_master_write(msg, retry)) {
		LOG_ERR("Failed to set PCIE RETIMER vendor id offset");
		goto exit;
	}

	memset(msg->data, 0, I2C_BUFF_SIZE);
	msg->tx_len = 1;
	msg->rx_len = 7;
	msg->data[0] = 0x01; //COMMAND CODE = 0x01 END=1, START=0, FUNC=3'b000, PEC=0
	if (i2c_master_read(msg, retry)) {
		LOG_ERR("Failed to read PCIE RETIMER vendor id");
		goto exit;
	}

	ret = true;

exit:
	if (k_mutex_unlock(&m88rt51632_mutex)) {
		LOG_ERR("m88rt51632 mutex unlock failed");
	}
	return ret;
}

/**
 * montage_retimer_read() - read retimer
 * @addr: address to read
 * @data: return data from retimer.
 *
 * Return: true on success, or false.
 */
bool montage_retimer_read(I2C_MSG *msg, uint32_t addr, uint32_t *data)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);

	if (!m88rt51632_smbus_write_byte(msg, RETIMER_ADDRPORT, addr)) {
		LOG_ERR("montage reading set RETIMER_ADDRPORT with addr %x failed", addr);
		return false;
	}

	if (!m88rt51632_smbus_read_byte(msg, RETIMER_DATAPORT, data)) {
		LOG_ERR("montage reading RETIMER_DATAPORT failed");
		return false;
	}

	return true;
}

/**
 * montage_retimer_write() - write retimer
 * @addr:       address to write
 * @data:       data to write
 *
 * Return: true on success, or false.
 */
bool montage_retimer_write(I2C_MSG *msg, uint32_t addr, uint32_t data)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);

	if (!m88rt51632_smbus_write_byte(msg, RETIMER_ADDRPORT, addr)) {
		LOG_ERR("montage write RETIMER_ADDRPORT with addr %x failed", addr);
		return false;
	}

	if (!m88rt51632_smbus_write_byte(msg, RETIMER_DATAPORT, data)) {
		LOG_ERR("montage write RETIMER_ADDRPORT with data %x failed", data);
		return false;
	}

	return true;
}

bool m88rt51632_get_fw_version(I2C_MSG *msg, uint32_t *version)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);

	if (!montage_retimer_read(msg, RETIMER_VERSION, version)) {
		LOG_ERR("montage get pcie retimer version failed");
		return false;
	}

	return true;
}

uint8_t m88rt51632_do_update(I2C_MSG *msg, uint32_t start_offset, uint8_t *txbuf, uint16_t msg_len)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, FWUPDATE_UPDATE_FAIL);
	CHECK_NULL_ARG_WITH_RETURN(txbuf, FWUPDATE_UPDATE_FAIL);
	uint8_t index;
	uint32_t offset = M88RT51632_EEPROM_BASE_OFFSET + start_offset;

	for (index = 0; index < msg_len; ++index) {
		if (!montage_retimer_write(msg, offset, txbuf[index])) {
			LOG_ERR("write eeprom offset %x fail", offset);
			return FWUPDATE_UPDATE_FAIL;
		}
		offset++;
		k_msleep(10); // wait 10ms till burn EEPROM complete
	}
	return FWUPDATE_SUCCESS;
}

uint8_t m88rt51632_fw_update(I2C_MSG *msg, uint32_t offset, uint16_t msg_len, uint8_t *msg_buf,
			     uint8_t flag)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, FWUPDATE_UPDATE_FAIL);
	CHECK_NULL_ARG_WITH_RETURN(msg_buf, FWUPDATE_UPDATE_FAIL);
	static bool is_init = false;
	static uint8_t *txbuf = NULL;
	static uint32_t start_offset = 0;
	uint32_t ret = 0;

	if (!is_init) {
		SAFE_FREE(txbuf);
		txbuf = (uint8_t *)malloc(IMAGE_PACKAGE_SIZE);
		if (txbuf == NULL) { // Retry alloc
			k_msleep(100);
			txbuf = (uint8_t *)malloc(IMAGE_PACKAGE_SIZE);
		}
		if (txbuf == NULL) {
			LOG_ERR("eeprom offset %x, failed to allocate txbuf.", offset);
			return FWUPDATE_OUT_OF_HEAP;
		}
		is_init = true;
		is_update_ongoing = true;
		start_offset = offset;
		k_msleep(10);
	}

	if (msg_len > IMAGE_PACKAGE_SIZE) {
		LOG_ERR("eeprom offset %x, recv data 0x%x over sector size 0x%x", offset, msg_len,
			IMAGE_PACKAGE_SIZE);
		SAFE_FREE(txbuf);
		k_msleep(10);
		is_init = false;
		is_update_ongoing = false;
		return FWUPDATE_OVER_LENGTH;
	}

	memcpy(txbuf, msg_buf, msg_len);

	if ((msg_len == IMAGE_PACKAGE_SIZE) || (flag & SECTOR_END_FLAG)) {
		if (k_mutex_lock(&m88rt51632_mutex, K_FOREVER)) {
			LOG_ERR("m88rt51632 mutex lock failed");
			SAFE_FREE(txbuf);
			k_msleep(10);
			return FWUPDATE_UPDATE_FAIL;
		}

		ret = m88rt51632_do_update(msg, start_offset, txbuf, msg_len);

		if (k_mutex_unlock(&m88rt51632_mutex)) {
			LOG_ERR("m88rt51632 mutex unlock failed");
			SAFE_FREE(txbuf);
			return FWUPDATE_UPDATE_FAIL;
		}

		SAFE_FREE(txbuf);
		k_msleep(10);

		if (ret) {
			LOG_ERR("Failed to update PCIE retimer eeprom, status %d", ret);
			return FWUPDATE_UPDATE_FAIL;
		} else {
			LOG_INF("PCIE retimer %x update success", start_offset);
		}

		is_init = false;
		is_update_ongoing = false;

		return ret;
	}

	return FWUPDATE_SUCCESS;
}

uint8_t m88rt51632_get_temperature(I2C_MSG *msg, float *temperature0, float *temperature1)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(temperature0, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(temperature1, SENSOR_UNSPECIFIED_ERROR);

	uint32_t ts_ctl_add[2] = { 0xa10, 0xb10 };
	uint32_t ts_sts_add[2] = { 0xa18, 0xb18 };

	int i, j, timeout;
	uint32_t ts_sts, flag, ODR, ts_ctl;
	float res[2];

	if (k_mutex_lock(&m88rt51632_mutex, K_MSEC(M88RT51632_MUTEX_LOCK_MS))) {
		LOG_ERR("m88rt51632 mutex lock failed");
		k_msleep(10);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	for (i = 0; i < MAX_SENSORS; i++) {
		if (!montage_retimer_read(msg, ts_ctl_add[i], &ts_ctl)) {
			LOG_ERR("montage get temperature read addr %x failed", ts_ctl_add[i]);
			goto error;
		}
		ts_ctl = ts_ctl & 0xFFFFFFF8;

		if (!montage_retimer_write(msg, ts_ctl_add[i], ts_ctl)) {
			LOG_ERR("montage get temperature write ts_ctl %x failed", ts_ctl);
			goto error;
		}

		if (!montage_retimer_write(msg, ts_ctl_add[i], ts_ctl | 0x4)) {
			LOG_ERR("montage get temperature write ts_ctl | 0x4 %x failed",
				ts_ctl | 0x4);
			goto error;
		}

		if (!montage_retimer_write(msg, ts_ctl_add[i], ts_ctl | 0x5)) {
			LOG_ERR("montage get temperature write ts_ctl | 0x5 %x failed",
				ts_ctl | 0x5);
			goto error;
		}

		timeout = 0;

		for (j = 0; j < 100; j++) {
			if (!montage_retimer_read(msg, ts_sts_add[i], &ts_sts)) {
				LOG_ERR("montage get temperature read system status failed");
				goto error;
			}

			if (GETBIT(ts_sts, 16)) {
				break;
			}

			k_msleep(2);
			if (j == 99) {
				timeout = 1;
			}
		}

		if (timeout == 1) {
			LOG_ERR("read temperature timeout!");
			goto error;
		}

		if (!montage_retimer_read(msg, ts_sts_add[i], &ts_sts)) {
			LOG_ERR("montage get temperature read addr %x failed", ts_sts_add[i]);
			goto error;
		}

		flag = (ts_sts >> 12) & 0x1;
		ODR = ts_sts & 0xfff;

		if (flag == 0) {
			res[i] = +ODR / 16.000;
		} else {
			res[i] = -ODR / 16.000;
		}
	}

	*temperature0 = res[0];
	*temperature1 = res[1];

	if (k_mutex_unlock(&m88rt51632_mutex)) {
		LOG_ERR("m88rt51632 mutex unlock failed");
	}

	return SENSOR_READ_SUCCESS;

error:
	if (k_mutex_unlock(&m88rt51632_mutex)) {
		LOG_ERR("m88rt51632 mutex unlock failed");
	}

	return SENSOR_UNSPECIFIED_ERROR;
}

uint8_t m88rt51632_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	float temperature[2] = { 0 };
	float avg_temperature = 0;
	I2C_MSG msg = { 0 };
	uint8_t ret;

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;

	if (is_update_ongoing) {
		return SENSOR_NOT_ACCESSIBLE;
	}

	switch (cfg->offset) {
	case M88RT51632_TEMP_OFFSET:
		ret = m88rt51632_get_temperature(&msg, &(temperature[0]), &(temperature[1]));
		if (ret != SENSOR_READ_SUCCESS) {
			return ret;
		}
		avg_temperature = (temperature[0] + temperature[1]) / 2;
		break;
	default:
		LOG_ERR("Invalid sensor 0x%x offset 0x%x", cfg->num, cfg->offset);
		return SENSOR_NOT_FOUND;
	}

	sensor_val *sval = (sensor_val *)reading;
	memset(sval, 0, sizeof(*sval));

	sval->integer = (int)avg_temperature & 0xFFFF;
	sval->fraction = (avg_temperature - sval->integer) * 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t m88rt51632_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	if (cfg->init_args != NULL) {
		pt5161l_init_arg *init_args = (pt5161l_init_arg *)cfg->init_args;

		if (init_args->is_init) {
			goto exit;
		}

		init_args->is_init = true;
	}

exit:
	cfg->read = m88rt51632_read;
	return SENSOR_INIT_SUCCESS;
}
