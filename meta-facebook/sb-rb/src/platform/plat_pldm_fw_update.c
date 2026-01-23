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
#include <string.h>
#include <logging/log.h>
#include <sys/crc.h>

#include "pldm_firmware_update.h"
#include "plat_pldm_fw_update.h"
#include "plat_pldm_sensor.h"
#include "plat_class.h"
#include "plat_hook.h"
#include "mp2971.h"
#include "mp29816a.h"
#include "raa228249.h"
#include "drivers/i2c_npcm4xx.h"
#include "util_spi.h"
#include "plat_gpio.h"
#include "plat_cpld.h"
#include "plat_iris_smbus.h"
#include "plat_util.h"
#include "plat_i2c.h"

#define RESET_CPLD_ON 0x3F
#define RESET_CPLD_OFF 0x00
#define IRIS_BOOT0_IMG_SIZE 0x1FFFFB

LOG_MODULE_REGISTER(plat_fwupdate);

static uint8_t pldm_pre_vr_update(void *fw_update_param);
static uint8_t pldm_post_vr_update(void *fw_update_param);
static uint8_t pldm_pre_bic_update(void *fw_update_param);
static bool get_vr_fw_version(void *info_p, uint8_t *buf, uint8_t *len);
static bool get_boot0_hamsa_fw_version(void *info_p, uint8_t *buf, uint8_t *len);
static bool get_boot0_medha0_fw_version(void *info_p, uint8_t *buf, uint8_t *len);
static bool get_boot0_medha1_fw_version(void *info_p, uint8_t *buf, uint8_t *len);

static uint32_t crc_boot0[3] = { 0 };
static uint32_t version_boot0[3] = { 0 };
const struct device *i2c_dev;
uint8_t slave_id = HAMSA_BOOT1_ADDR;
static uint32_t write_addr = HAMSA_BOOT1_ASIC_MEM_ADDR;
bool update_flag = 0;
typedef struct {
	uint8_t firmware_comp_id;
	uint8_t plat_pldm_sensor_id;
	char sensor_name[MAX_AUX_SENSOR_NAME_LEN];
} compnt_mapping_sensor;

compnt_mapping_sensor vr_compnt_mapping_sensor_table[] = {
	{ COMPNT_VR_1, SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_TEMP_C, "ASIC_P0V85_MEDHA0_VDD" },
	{ COMPNT_VR_2, SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_TEMP_C, "ASIC_P0V85_MEDHA1_VDD" },
	{ COMPNT_VR_3, SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_TEMP_C, "ASIC_P0V9_OWL_E_TRVDD" },
	{ COMPNT_VR_4, SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_TEMP_C, "ASIC_P0V75_MAX_M_VDD" },
	{ COMPNT_VR_5, SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_TEMP_C, "ASIC_P0V75_OWL_E_VDD" },
	{ COMPNT_VR_6, SENSOR_NUM_ASIC_P1V1_VDDQC_HBM1357_TEMP_C, "ASIC_P1V1_VDDQC_HBM1357" },
	{ COMPNT_VR_7, SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_TEMP_C, "ASIC_P0V75_MAX_N_VDD" },
	{ COMPNT_VR_8, SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_TEMP_C,
	  "ASIC_P1V2_HAMSA_VDDHRXTX_PCIE" },
	{ COMPNT_VR_9, SENSOR_NUM_ASIC_P1V1_VDDQC_HBM0246_TEMP_C, "ASIC_P1V1_VDDQC_HBM0246" },
	{ COMPNT_VR_10, SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_TEMP_C, "ASIC_P0V4_VDDQL_HBM0246" },
	{ COMPNT_VR_11, SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_TEMP_C, "ASIC_P0V75_OWL_W_VDD" },
	{ COMPNT_VR_12, SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_TEMP_C, "ASIC_P0V9_OWL_W_TRVDD" },
	{ COMPNT_VR_3V3, SENSOR_NUM_P3V3_OSFP_TEMP_C, "P3V3_OSFP" },
};
static uint8_t pldm_pre_bic_update(void *fw_update_param)
{
	ARG_UNUSED(fw_update_param);

	/* Stop sensor polling */
	set_plat_sensor_polling_enable_flag(false);
	LOG_INF("Stop pldm sensor polling");
	return 0;
}
void spi_node_disable()
{
	gpio_set(SPI_HAMSA_MUX_IN1, 0);
	gpio_set(SPI_MEDHA0_MUX_IN1, 0);
	gpio_set(SPI_MEDHA1_MUX_IN1, 0);
	gpio_set(QSPI_CPLD_SEL_0, 0);
	gpio_set(QSPI_CPLD_SEL_1, 0);
}

void change_spi_node_to_hamsa()
{
	gpio_set(SPI_HAMSA_MUX_IN1, 1);
	gpio_set(QSPI_CPLD_SEL_0, 0);
	gpio_set(QSPI_CPLD_SEL_1, 0);
}

void change_spi_node_to_medha0()
{
	gpio_set(SPI_MEDHA0_MUX_IN1, 1);
	gpio_set(QSPI_CPLD_SEL_0, 1);
	gpio_set(QSPI_CPLD_SEL_1, 0);
}

void change_spi_node_to_medha1()
{
	gpio_set(SPI_MEDHA1_MUX_IN1, 1);
	gpio_set(QSPI_CPLD_SEL_0, 0);
	gpio_set(QSPI_CPLD_SEL_1, 1);
}

void set_cpld_reset_reg(uint8_t value)
{
	uint8_t temp_data = 0;
	plat_read_cpld(RESET, &temp_data, 1);
	LOG_DBG("cpld reset reg: 0x%x", temp_data);
	temp_data = value; // set cpld reset
	plat_write_cpld(RESET, &temp_data);
	// check cpld reset reg
	plat_read_cpld(RESET, &temp_data, 1);
	LOG_DBG("check cpld reset reg: 0x%x", temp_data);
}

static uint8_t pldm_pre_mtia_flash_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, 1);

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;
	set_cpld_reset_reg(RESET_CPLD_OFF);
	uint16_t spi_node = p->comp_id;
	LOG_DBG("MTIA flash comp id: 0x%x", p->comp_id);
	switch (spi_node) {
	case COMPNT_HAMSA:
		change_spi_node_to_hamsa();
		LOG_INF("change spi node to hamsa");
		break;
	case COMPNT_MEDHA0:
		change_spi_node_to_medha0();
		LOG_INF("change spi node to medha0");
		break;
	case COMPNT_MEDHA1:
		change_spi_node_to_medha1();
		LOG_INF("change spi node to medha1");
		break;
	default:
		LOG_ERR("Unsupported MTIA flash comp id: 0x%x", p->comp_id);
		set_cpld_reset_reg(RESET_CPLD_ON);
		return 1;
	}

	// re-init flash
	const struct device *flash_dev;
	flash_dev = device_get_binding("spi_fiu0_cs1");
	int rc = 0;
	rc = spi_nor_re_init(flash_dev);
	if (rc != 0) {
		LOG_ERR("spi_nor_re_init fail");
		set_cpld_reset_reg(RESET_CPLD_ON);
		return 1;
	}

	return 0;
}

uint8_t pldm_mtia_flash_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, 1);

	return pldm_fw_update(fw_update_param, DEVSPI_SPI1_CS1);
}

uint32_t plat_get_image_crc_checksum(uint8_t index)
{
	if (index >= BOOT0_MAX) {
		return 0;
	} else {
		return crc_boot0[index];
	}
}

uint32_t plat_get_image_version(uint8_t index)
{
	if (index >= BOOT0_MAX) {
		return 0;
	} else {
		return version_boot0[index];
	}
}

static uint8_t pldm_post_mtia_flash_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, 1);
	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;

	//read data back to calculate CRC32
	uint8_t *rxbuf = NULL;
	uint8_t *version_rxbuf = NULL;
	// image data is from 0x0 to 0x1FFFFB, 0x1FFFFC to 0x1FFFFF is CRC32
	int remain = IRIS_BOOT0_IMG_SIZE + 1;
	uint32_t offset = 0;
	uint32_t crc32 = 0;

	rxbuf = malloc(PLAT_CRC32_READ_SIZE);
	version_rxbuf = malloc(PLAT_CRC32_READ_SIZE);
	if (rxbuf == NULL) {
		LOG_ERR("Fail to allocate size %d", PLAT_CRC32_READ_SIZE);
	} else {
		while (remain > 0) {
			if (remain > PLAT_CRC32_READ_SIZE) {
				if (read_fw_image(offset, PLAT_CRC32_READ_SIZE, rxbuf,
						  DEVSPI_SPI1_CS1)) {
					LOG_ERR("Fail to read offset %x", offset);
					break;
				}
				crc32 = crc32_ieee_update(crc32, rxbuf, PLAT_CRC32_READ_SIZE);
				remain = remain - PLAT_CRC32_READ_SIZE;
				offset += PLAT_CRC32_READ_SIZE;
			} else {
				if (read_fw_image(offset, remain, rxbuf, DEVSPI_SPI1_CS1)) {
					LOG_ERR("Fail to read offset %x", offset);
					break;
				}
				crc32 = crc32_ieee_update(crc32, rxbuf, remain);
				remain = 0;
				break;
			}
		}
	}
	SAFE_FREE(rxbuf);
	if (read_fw_image(0x1FFFF8, 3, version_rxbuf, DEVSPI_SPI1_CS1)) {
		LOG_ERR("read flash : read_fw_image fail");
		set_cpld_reset_reg(RESET_CPLD_ON);
		SAFE_FREE(version_rxbuf);
		return 1;
	}
	uint32_t ver_value = version_rxbuf[0] << 16 | version_rxbuf[1] << 8 | version_rxbuf[2];

	LOG_DBG("version: 0x%x, crc32: 0x%x", ver_value, crc32);
	switch (p->comp_id) {
	case COMPNT_HAMSA:
		crc_boot0[BOOT0_HAMSA] = crc32;
		version_boot0[BOOT0_HAMSA] = ver_value;
		LOG_DBG("version: 0x%x, crc32: 0x%x", version_boot0[BOOT0_HAMSA],
			crc_boot0[BOOT0_HAMSA]);
		SAFE_FREE(version_rxbuf);
		break;
	case COMPNT_MEDHA0:
		crc_boot0[BOOT0_MEDHA0] = crc32;
		version_boot0[BOOT0_MEDHA0] = ver_value;
		LOG_DBG("version: 0x%x, crc32: 0x%x", version_boot0[BOOT0_MEDHA0],
			crc_boot0[BOOT0_MEDHA0]);
		SAFE_FREE(version_rxbuf);
		break;
	case COMPNT_MEDHA1:
		crc_boot0[BOOT0_MEDHA1] = crc32;
		version_boot0[BOOT0_MEDHA1] = ver_value;
		LOG_DBG("version: 0x%x, crc32: 0x%x", version_boot0[BOOT0_MEDHA1],
			crc_boot0[BOOT0_MEDHA1]);
		SAFE_FREE(version_rxbuf);
		break;
	default:
		break;
	}

	// disable spi node
	spi_node_disable();
	LOG_INF("Disable spi node");
	set_cpld_reset_reg(RESET_CPLD_ON);
	return 0;
}

bool plat_get_image_crc_checksum_from_flash(uint8_t index, uint8_t data_type, uint32_t *data)
{
	if (index != COMPNT_HAMSA && index != COMPNT_MEDHA0 && index != COMPNT_MEDHA1) {
		return false;
	} else {
		uint16_t spi_node = index;
		uint8_t flash_index = 0;
		LOG_DBG("Read flash comp id: 0x%x", index);
		switch (spi_node) {
		case COMPNT_HAMSA:
			change_spi_node_to_hamsa();
			LOG_DBG("change spi node to hamsa");
			flash_index = BOOT0_HAMSA;
			break;
		case COMPNT_MEDHA0:
			change_spi_node_to_medha0();
			LOG_DBG("change spi node to medha0");
			flash_index = BOOT0_MEDHA0;
			break;
		case COMPNT_MEDHA1:
			change_spi_node_to_medha1();
			LOG_DBG("change spi node to medha1");
			flash_index = BOOT0_MEDHA1;
			break;
		default:
			LOG_ERR("Unsupported read flash comp id: 0x%x", index);
			set_cpld_reset_reg(RESET_CPLD_ON);
			return false;
		}
		// re-init flash
		const struct device *flash_dev;
		flash_dev = device_get_binding("spi_fiu0_cs1");
		int rc = 0;
		rc = spi_nor_re_init(flash_dev);
		if (rc != 0) {
			LOG_ERR("read flash : spi_nor_re_init fail");
			set_cpld_reset_reg(RESET_CPLD_ON);
			return false;
		}
		//read data back to combine in Ver and CRC32
		uint8_t *ver_rxbuf = NULL;
		uint8_t *crc32_rxbuf = NULL;
		ver_rxbuf = malloc(128);
		crc32_rxbuf = malloc(128);
		/*
		0x1FFFF8 (Byte 0): VERSION_PATCH = 0x00
		0x1FFFF9 (Byte 1): VERSION_MINOR  = 0x06
		0x1FFFFA (Byte 2): VERSION_MAJOR  = 0x01
		
		0x1FFFFC: CRC32 byte 0 (LSB)
		0x1FFFFD: CRC32 byte 1      
		0x1FFFFE: CRC32 byte 2      
		0x1FFFFF: CRC32 byte 3 (MSB)
		
		VER : 01.06.00 | CRC32 : e9e2b0ba
		*/
		int test = read_fw_image(0x1FFFF8, 3, ver_rxbuf, DEVSPI_SPI1_CS1);
		LOG_DBG("Read flash test: %d", test);
		if (read_fw_image(0x1FFFF8, 3, ver_rxbuf, DEVSPI_SPI1_CS1)) {
			LOG_ERR("read flash : read_fw_image fail");
			set_cpld_reset_reg(RESET_CPLD_ON);
			SAFE_FREE(ver_rxbuf);
			SAFE_FREE(crc32_rxbuf);
			return false;
		}
		if (read_fw_image(0x1FFFFC, 4, crc32_rxbuf, DEVSPI_SPI1_CS1)) {
			LOG_ERR("read flash : read_fw_image fail");
			set_cpld_reset_reg(RESET_CPLD_ON);
			SAFE_FREE(ver_rxbuf);
			SAFE_FREE(crc32_rxbuf);
			return false;
		}

		uint32_t ver = 0;
		ver = ver_rxbuf[0] << 16 | ver_rxbuf[1] << 8 | ver_rxbuf[2];
		uint32_t crc32 = 0;
		crc32 = crc32_rxbuf[0] << 24 | crc32_rxbuf[1] << 16 | crc32_rxbuf[2] << 8 |
			crc32_rxbuf[3];
		LOG_DBG("VER : 0x%x | CRC32 : 0x%x", ver, crc32);
		switch (data_type) {
		case VERSION:
			set_cpld_reset_reg(RESET_CPLD_ON);
			*data = ver;
			LOG_DBG("version flash idx: %d", flash_index);
			version_boot0[flash_index] = ver;
			LOG_DBG("version: 0x%x", version_boot0[flash_index]);

			break;
		case CRC32:
			set_cpld_reset_reg(RESET_CPLD_ON);
			*data = crc32;
			LOG_DBG("crc flash idx: %d", flash_index);
			crc_boot0[flash_index] = crc32;
			LOG_DBG("crc: 0x%x", crc_boot0[flash_index]);

			break;
		default:
			LOG_ERR("Unsupported data type: 0x%x", data_type);
			set_cpld_reset_reg(RESET_CPLD_ON);
			SAFE_FREE(ver_rxbuf);
			SAFE_FREE(crc32_rxbuf);
			return false;
			break;
		}

		SAFE_FREE(ver_rxbuf);
		SAFE_FREE(crc32_rxbuf);
		set_cpld_reset_reg(RESET_CPLD_ON);
		return true;
	}
}

int sb_write_byte(uint8_t cmd, uint8_t data)
{
	return i2c_reg_write_byte(i2c_dev, slave_id, cmd, data);
}
int smbus_read_byte(uint8_t cmd, uint8_t *data)
{
	uint8_t rd;
	int ret;
	ret = i2c_write_read(i2c_dev, slave_id, &cmd, 1, &rd, 1);
	*data = rd;

	return ret;
}
static char *prt_mode(uint8_t mode)
{
	char *str = NULL;

	switch (mode) {
	case FASTBOOT_MODE:
		str = "fastboot";
		break;
	case CMRT_SIC_MODE:
		str = "cmrt_sic";
		break;
	case RECOVERY_MODE:
		str = "recovery";
		break;
	default:
		str = "unknown";
		break;
	}
	return str;
}
static int word_to_i2c_pkt(uint8_t *dst, uint32_t src)
{
	int data_index = 0;

	/* read the data from specified address */
	dst[data_index++] = (src >> 0) & BYTE_MASK;
	dst[data_index++] = (src >> 8) & BYTE_MASK;
	dst[data_index++] = (src >> 16) & BYTE_MASK;
	dst[data_index++] = (src >> 24) & BYTE_MASK;

	return data_index;
}
int sb_write_block(uint8_t slv_id, uint8_t cmd, uint8_t *data, uint32_t len)
{
	//return i2c_burst_write(i2c_dev, slv_id, cmd, data, len);
	return plat_i2c_write(I2C_BUS12, slave_id, cmd, data, len);
}
int sb_read_byte(uint8_t cmd, uint8_t *data)
{
	uint8_t rd;
	int ret;
	ret = i2c_write_read(i2c_dev, slave_id, &cmd, 1, &rd, 1);
	*data = rd;

	return ret;
}
uint32_t smbus_mode_query(void)
{
	uint8_t rd = 0;
	uint8_t cmd = SB_MODE_QUERY;
	int err = 0xffff;
	err = i2c_write_read(i2c_dev, slave_id, &cmd, 1, &rd, 1);

	LOG_INF("slave:0x%x in mode:%d, %s", slave_id, rd, prt_mode(rd));

	return err == 0 ? rd & 0x7 : err;
}
int sb_write_fwblock(uint32_t addr, uint32_t *data, uint32_t data_len)
{
	//smbus max payload for a transaction is 32
	uint8_t buf[SMBUS_MAX_PKT_LEN] = { 0 };
	uint8_t words = 0;
	buf[MSG_PKT_LEN_OFFSET] = ADDR_DATA_LEN_SZ + data_len;
	word_to_i2c_pkt(&buf[MSG_ADDR_OFFSET], addr);
	buf[MSG_DATA_LEN_OFFSET] = data_len;

	// if data length is multples of word size, packetize each word first
	while (data_len > 0) {
		word_to_i2c_pkt(&buf[MSG_RDWR_DATA_START + words * BYTES_PER_WORD],
				*(data + words));
		data_len -= BYTES_PER_WORD;
		words++;
	}

	//bytes to write = total pkt length(buf[0]) + pkt_len_byte_size(1B)
	return sb_write_block(slave_id, FW_DATA_WRITE, buf,
			      buf[MSG_PKT_LEN_OFFSET] + PKT_LEN_BYTE_SZ);
}

uint8_t pldm_pre_iris_boot_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, 1);
	update_flag = 0;
	uint8_t rd;
	write_addr = HAMSA_BOOT1_ASIC_MEM_ADDR;
	i2c_dev = device_get_binding("I2C_11");
	// check i2c device
	if (!i2c_dev) {
		LOG_ERR("Failed to get binding for I2C_11");
		update_flag = 1;
		return 1;
	}
	// pre do: check is download mode
	int err = smbus_mode_query();
	if (!err) {
		LOG_ERR("slave not in download mode");
		goto err_i2c;
	}
	// start: Send a FW_DL_START command using FW_CTRL_WRITE
	err = sb_write_byte(FW_CTRL_WRITE, FW_DL_START);
	if (err < 0) {
		LOG_ERR("Firmware download start request failed");
		goto err_i2c;
	}
	// send: send the firmware image to IRIS
	err = sb_write_fwblock(HAMSA_BOOT1_ADDR, &fw_update_cfg.image_size, 4);
	if (err == false) {
		LOG_ERR("sending fw addr-size failed");
		goto err_i2c;
	}

	LOG_INF("Waiting for Slave to get ready");
	// Wait for the slave firmware to get into download mode
	err = sb_read_byte(FW_CTRL_READ, &rd);
	if (err < 0) {
		LOG_ERR("slave status read failed. Abort Download");
		goto err_i2c;
	}
	LOG_INF("slave fw status read: 0x%x", rd);
	if (rd == FW_DL_SLV_RDY)
		LOG_INF("device ready to download(0x%x)", rd);
	else {
		LOG_ERR("device status error(0x%x)", rd);
		goto err_i2c;
	}
	return 0;

err_i2c:
	// send download abort command to device
	LOG_INF("partial download, abort now");
	err = sb_write_byte(FW_CTRL_WRITE, FW_DL_HST_ABRT);
	if (err < 0) {
		LOG_ERR("cmd write failed, %d", err);
	}

	LOG_INF("check abort status...");
	// read slave status and check of device aborted download
	err = sb_read_byte(FW_CTRL_READ, &rd);
	if (err < 0) {
		LOG_ERR("cmd status read failed, %d", err);
	}
	if (rd & FW_DL_SLV_ABRTD) {
		LOG_WRN("SMBus device 0x%x aborted download", slave_id);
	} else {
		LOG_ERR("SMBus device 0x%x failed to abort(0x%x)", slave_id, rd);
	}
	// check what smbus error occurred
	err = smbus_read_byte(FW_SMBUS_ERROR, &rd);
	if (err < 0)
		LOG_ERR("read error status failed, %d", err);
	else
		LOG_ERR("smbus error status = 0x%x\n", rd);
	update_flag = 1;
	return 1;
}
int iris_data_write(uint8_t *data, uint32_t data_size)
{
	int idx = 0;
	int err = 0;
	uint32_t size = data_size;
	uint32_t dst_addr = HAMSA_BOOT1_ASIC_MEM_ADDR;
	uint8_t *data_buf = data;
	uint8_t rd;
	uint32_t write_size = MAX_DATA_PKT_SIZE;
	while (size > 0) {
		int offset = (write_size * idx);

		if (size > MAX_DATA_PKT_SIZE) {
			write_size = MAX_DATA_PKT_SIZE;
			size = size - MAX_DATA_PKT_SIZE;
			idx++;
		} else {
			// Last chunk
			write_size = size;
			size = 0;
			idx++;
		}
		// if error,  retry up to MAX_RETRIES
		int tries = MAX_RETRIES;
		while (tries) {
			err = sb_write_fwblock(write_addr, (uint32_t *)(data_buf + offset),
					       write_size);
			if (err == true)
				break;
			tries--;
			LOG_INF("t%d ", tries);
		}
		// if fails after all retries, abort the download cmd
		if (tries == 0) {
			LOG_ERR("fw write fail at addr:0x%x,%dB, remaining:%dB", write_addr,
				write_size, size);
			LOG_WRN("aborting the firmware download\n");
			goto err_i2c_write_img;
		}

		// after first block, ensure fw state is set to progress
		if (write_addr == dst_addr) {
			LOG_INF("Verify status progress bit");
			// If the slave state is not valid, abort download cmd
			err = sb_read_byte(FW_CTRL_READ, &rd);
			if (err < 0) {
				LOG_ERR("status read failed. Abort Download\n");
				goto err_i2c_write_img;
			}

			if ((rd & FW_DL_SLV_PROG) == 0) {
				LOG_ERR("invalid fw status:0x%x\n", rd);
			} else
				LOG_INF("fw status set to progress(0x%x)\n", rd);
		}

		write_addr += write_size;
	}
	return 0;

err_i2c_write_img:
	return 1;
}
uint8_t pldm_iris_boot_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, 1);

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;

	CHECK_NULL_ARG_WITH_RETURN(p->data, 1);

	LOG_DBG("image size: %d", fw_update_cfg.image_size);
	LOG_DBG("data len: %d", p->data_len);

	uint8_t *data = p->data;
	uint32_t len = p->data_len;
	uint32_t sent = 0;

	while (sent < len) {
		uint32_t chunk = MAX_DATA_PKT_SIZE;
		if (sent + chunk > len)
			chunk = len - sent; // last chunk can be < 24 bytes

		bool ok = false;
		for (int retry = 0; retry < STATUS_RETRY_CNT; retry++) {
			if (!iris_data_write(data + sent, chunk)) {
				ok = true;
				break;
			}
		}

		if (!ok) {
			LOG_ERR("IRIS block transfer failed at PLDM offset %u", p->data_ofs + sent);
			return 1;
		}

		sent += chunk;
	}
	// update next PLDM offset / length
	p->next_ofs = p->data_ofs + len;
	if (p->next_ofs < fw_update_cfg.image_size) {
		uint32_t remain = fw_update_cfg.image_size - p->next_ofs;
		p->next_len = (remain > fw_update_cfg.max_buff_size) ? fw_update_cfg.max_buff_size :
								       remain;
	} else {
		p->next_len = 0;
	}

	return 0;
}

uint8_t pldm_post_iris_boot_update(void *fw_update_param)
{
	ARG_UNUSED(fw_update_param);
	if (update_flag == 1) {
		LOG_WRN("FW pre update failed, skip post update");
		return 0;
	}
	// finished: set the bit to inform device that download is complete
	int err = sb_write_byte(FW_CTRL_WRITE, FW_DL_FINISH);
	if (err < 0) {
		// retain original err code returned, i.e. set by ioctl
		LOG_ERR("Unable to notify download completion state, aborting");
		goto err_i2c;
	}
	LOG_INF("Sent firmware activate command");

	int check_idx = 0;
	uint8_t rd;
	/* Wait for the slave to complete firmware update */
	while (1) {
		rd = 0;
		err = sb_read_byte(FW_CTRL_READ, &rd);
		if (err < 0) {
			LOG_ERR("fw upgrade state read failed, aborting");
			goto err_i2c;
		}

		if (rd & FW_DL_SLV_DONE) {
			LOG_INF("Read status: FW update complete");
			break;
		}

		if (check_idx++ > STATUS_RETRY_CNT) {
			LOG_ERR("Invalid completion status(0x%x) read", rd);
			err = -1; // no errno, set card fw error
			goto err_i2c;
		}
	}
	LOG_INF("~ Firmware Update Completed ~\n");
	return 0;

err_i2c:
	// send download abort command to device
	LOG_INF("partial download, abort now");
	err = sb_write_byte(FW_CTRL_WRITE, FW_DL_HST_ABRT);
	if (err < 0) {
		LOG_ERR("cmd write failed, %d", err);
	}

	LOG_INF("check abort status...");
	// read slave status and check of device aborted download
	err = sb_read_byte(FW_CTRL_READ, &rd);
	if (err < 0) {
		LOG_ERR("cmd status read failed, %d", err);
	}
	if (rd & FW_DL_SLV_ABRTD) {
		LOG_WRN("SMBus device 0x%x aborted download", slave_id);
	} else {
		LOG_ERR("SMBus device 0x%x failed to abort(0x%x)", slave_id, rd);
	}
	// check what smbus error occurred
	err = smbus_read_byte(FW_SMBUS_ERROR, &rd);
	if (err < 0)
		LOG_ERR("read error status failed, %d", err);
	else
		LOG_ERR("smbus error status = 0x%x\n", rd);
	return 1;
}
#define ASIC_VERSION_BYTE 0x68
#define I2C_MAX_RETRY 3
bool get_fw_version_from_asic(uint8_t *data)
{
	I2C_MSG i2c_msg = { .bus = I2C_BUS12, .target_addr = 0x32 };
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 11;
	i2c_msg.data[0] = ASIC_VERSION_BYTE;
	i2c_master_read(&i2c_msg, I2C_MAX_RETRY);

	LOG_INF(" boot0 VER : %02d.%02d.%02d", i2c_msg.data[9], i2c_msg.data[8], i2c_msg.data[7]);
	uint32_t data_p = i2c_msg.data[9] << 16 | i2c_msg.data[8] << 8 | i2c_msg.data[7];
	memcpy(data, &data_p, 4);
	return true;
}

static bool get_boot1_fw_version(void *info_p, uint8_t *buf, uint8_t *len)
{
	bool ret = false;
	const char *remain_str_p = ", BOOT1: ";
	uint8_t *buf_p = buf;
	*len = 0;

	memcpy(buf_p, remain_str_p, strlen(remain_str_p));
	buf_p += strlen(remain_str_p);
	*len += strlen(remain_str_p);
	uint8_t *version_tmp = NULL;
	uint32_t version = get_fw_version_from_asic(version_tmp);

	*len += bin2hex((uint8_t *)&version, 4, buf_p, 4);
	buf_p += 4;

	ret = true;
	return ret;
}

static bool get_boot0_hamsa_fw_version(void *info_p, uint8_t *buf, uint8_t *len)
{
	CHECK_NULL_ARG_WITH_RETURN(info_p, false);
	CHECK_NULL_ARG_WITH_RETURN(buf, false);
	CHECK_NULL_ARG_WITH_RETURN(len, false);
	bool ret = false;
	const char *remain_str_p = "flash hamsa BOOT0: ";
	uint8_t *buf_p = buf;
	*len = 0;

	memcpy(buf_p, remain_str_p, strlen(remain_str_p));
	buf_p += strlen(remain_str_p);
	*len += strlen(remain_str_p);
	uint8_t *version = (uint8_t *)&version_boot0[BOOT0_HAMSA];
	LOG_INF("version_boot0[BOOT0_HAMSA]: %x", version_boot0[BOOT0_HAMSA]);
	// bin2hex: 3 bytes → 6 charsgit s
	int hex_len = bin2hex(version, 3, buf_p, 6);

	buf_p += hex_len;
	*len += hex_len;

	const char *space = " ";
	memcpy(buf_p, space, strlen(space));
	buf_p += strlen(space);
	*len += strlen(space);

	uint8_t *crc_checksum = (uint8_t *)&crc_boot0[BOOT0_HAMSA];
	int crc_hex_len = bin2hex(crc_checksum, 4, buf_p, 8);
	buf_p += crc_hex_len;
	*len += crc_hex_len;

	ret = true;
	return ret;
}
static bool get_boot0_medha0_fw_version(void *info_p, uint8_t *buf, uint8_t *len)
{
	CHECK_NULL_ARG_WITH_RETURN(info_p, false);
	CHECK_NULL_ARG_WITH_RETURN(buf, false);
	CHECK_NULL_ARG_WITH_RETURN(len, false);

	bool ret = false;
	const char *remain_str_p = "flash medha0 BOOT0: ";
	uint8_t *buf_p = buf;
	*len = 0;

	memcpy(buf_p, remain_str_p, strlen(remain_str_p));
	buf_p += strlen(remain_str_p);
	*len += strlen(remain_str_p);
	uint8_t *version = (uint8_t *)&version_boot0[BOOT0_MEDHA0];
	LOG_INF("version_boot0[BOOT0_MEDHA0]: %x", version_boot0[BOOT0_MEDHA0]);
	// bin2hex: 3 bytes → 6 chars
	int hex_len = bin2hex(version, 3, buf_p, 6);
	buf_p += hex_len;
	*len += hex_len;

	const char *space = " ";
	memcpy(buf_p, space, strlen(space));
	buf_p += strlen(space);
	*len += strlen(space);

	uint8_t *crc_checksum = (uint8_t *)&crc_boot0[BOOT0_MEDHA0];
	int crc_hex_len = bin2hex(crc_checksum, 4, buf_p, 8);
	buf_p += crc_hex_len;
	*len += crc_hex_len;

	ret = true;
	return ret;
}
static bool get_boot0_medha1_fw_version(void *info_p, uint8_t *buf, uint8_t *len)
{
	CHECK_NULL_ARG_WITH_RETURN(info_p, false);
	CHECK_NULL_ARG_WITH_RETURN(buf, false);
	CHECK_NULL_ARG_WITH_RETURN(len, false);
	bool ret = false;
	const char *remain_str_p = "flash medha1 BOOT0: ";
	uint8_t *buf_p = buf;
	*len = 0;
	memcpy(buf_p, remain_str_p, strlen(remain_str_p));
	buf_p += strlen(remain_str_p);
	*len += strlen(remain_str_p);
	uint8_t *version = (uint8_t *)&version_boot0[BOOT0_MEDHA1];
	LOG_INF("version_boot0[BOOT0_MEDHA1]: %x", version_boot0[BOOT0_MEDHA1]);
	// bin2hex: 3 bytes → 6 chars
	int hex_len = bin2hex(version, 3, buf_p, 6);
	buf_p += hex_len;
	*len += hex_len;

	const char *space = " ";
	memcpy(buf_p, space, strlen(space));
	buf_p += strlen(space);
	*len += strlen(space);

	uint8_t *crc_checksum = (uint8_t *)&crc_boot0[BOOT0_MEDHA1];
	int crc_hex_len = bin2hex(crc_checksum, 4, buf_p, 8);
	buf_p += crc_hex_len;
	*len += crc_hex_len;

	ret = true;
	return ret;
}
//clang-format off
#define VR_COMPONENT_DEF(comp_id)                                                                  \
	{                                                                                          \
		.enable = true, .comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,                 \
		.comp_identifier = comp_id, .comp_classification_index = 0x00,                     \
		.pre_update_func = pldm_pre_vr_update, .update_func = pldm_vr_update,              \
		.pos_update_func = pldm_post_vr_update, .inf = COMP_UPDATE_VIA_I2C,                \
		.activate_method = COMP_ACT_AC_PWR_CYCLE, .self_act_func = NULL,                   \
		.get_fw_version_fn = get_vr_fw_version, .self_apply_work_func = NULL,              \
		.comp_version_str = NULL,                                                          \
	}
// clang-format on

/* PLDM FW update table */
pldm_fw_update_info_t PLDMUPDATE_FW_CONFIG_TABLE[] = {
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = COMPNT_BIC,
		.comp_classification_index = 0x00,
		.pre_update_func = pldm_pre_bic_update,
		.update_func = pldm_bic_update,
		.pos_update_func = NULL,
		.inf = COMP_UPDATE_VIA_SPI,
		.activate_method = COMP_ACT_SELF,
		.self_act_func = pldm_bic_activate,
		.get_fw_version_fn = NULL,
		.self_apply_work_func = NULL,
		.comp_version_str = NULL,
	},
	VR_COMPONENT_DEF(COMPNT_VR_1),
	VR_COMPONENT_DEF(COMPNT_VR_2),
	VR_COMPONENT_DEF(COMPNT_VR_3),
	VR_COMPONENT_DEF(COMPNT_VR_4),
	VR_COMPONENT_DEF(COMPNT_VR_5),
	VR_COMPONENT_DEF(COMPNT_VR_6),
	VR_COMPONENT_DEF(COMPNT_VR_7),
	VR_COMPONENT_DEF(COMPNT_VR_8),
	VR_COMPONENT_DEF(COMPNT_VR_9),
	VR_COMPONENT_DEF(COMPNT_VR_10),
	VR_COMPONENT_DEF(COMPNT_VR_11),
	VR_COMPONENT_DEF(COMPNT_VR_12),
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = COMPNT_HAMSA,
		.comp_classification_index = 0x00,
		.pre_update_func = pldm_pre_mtia_flash_update,
		.update_func = pldm_mtia_flash_update,
		.pos_update_func = pldm_post_mtia_flash_update,
		.inf = COMP_UPDATE_VIA_SPI,
		.activate_method = COMP_ACT_SELF,
		.self_act_func = NULL,
		.get_fw_version_fn = get_boot0_hamsa_fw_version,
		.self_apply_work_func = NULL,
		.comp_version_str = NULL,
	},
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = COMPNT_MEDHA0,
		.comp_classification_index = 0x00,
		.pre_update_func = pldm_pre_mtia_flash_update,
		.update_func = pldm_mtia_flash_update,
		.pos_update_func = pldm_post_mtia_flash_update,
		.inf = COMP_UPDATE_VIA_SPI,
		.activate_method = COMP_ACT_SELF,
		.self_act_func = NULL,
		.get_fw_version_fn = get_boot0_medha0_fw_version,
		.self_apply_work_func = NULL,
		.comp_version_str = NULL,
	},
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = COMPNT_MEDHA1,
		.comp_classification_index = 0x00,
		.pre_update_func = pldm_pre_mtia_flash_update,
		.update_func = pldm_mtia_flash_update,
		.pos_update_func = pldm_post_mtia_flash_update,
		.inf = COMP_UPDATE_VIA_SPI,
		.activate_method = COMP_ACT_SELF,
		.self_act_func = NULL,
		.get_fw_version_fn = get_boot0_medha1_fw_version,
		.self_apply_work_func = NULL,
		.comp_version_str = NULL,
	},
	VR_COMPONENT_DEF(COMPNT_VR_3V3),
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = COMPNT_HAMSA_BOOT1,
		.comp_classification_index = 0x00,
		.pre_update_func = pldm_pre_iris_boot_update,
		.update_func = pldm_iris_boot_update,
		.pos_update_func = pldm_post_iris_boot_update,
		.inf = COMP_UPDATE_VIA_I2C,
		.activate_method = COMP_ACT_SELF,
		.self_act_func = NULL,
		.get_fw_version_fn = get_boot1_fw_version,
		.self_apply_work_func = NULL,
		.comp_version_str = NULL,
	},
};

uint8_t plat_pldm_query_device_identifiers(const uint8_t *buf, uint16_t len, uint8_t *resp,
					   uint16_t *resp_len)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, false);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);

	LOG_INF("pldm_query_device_identifiers");

	struct pldm_query_device_identifiers_resp *resp_p =
		(struct pldm_query_device_identifiers_resp *)resp;

	resp_p->completion_code = PLDM_SUCCESS;
	resp_p->descriptor_count = 0x03;

	uint8_t iana[PLDM_FWUP_IANA_ENTERPRISE_ID_LENGTH] = { 0x00, 0x00, 0xA0, 0x15 };

	// Set the device id for sd bic
	uint8_t deviceId[PLDM_PCI_DEVICE_ID_LENGTH] = { 0x00, 0x00 };

	uint8_t slotNumber = plat_get_eid() / 10;
	uint8_t slot[PLDM_ASCII_MODEL_NUMBER_SHORT_STRING_LENGTH] = { (char)(slotNumber + '0') };

	uint8_t total_size_of_iana_descriptor =
		sizeof(struct pldm_descriptor_tlv) + sizeof(iana) - 1;

	uint8_t total_size_of_device_id_descriptor =
		sizeof(struct pldm_descriptor_tlv) + sizeof(deviceId) - 1;

	uint8_t total_size_of_slot_descriptor =
		sizeof(struct pldm_descriptor_tlv) + sizeof(slot) - 1;

	if (sizeof(struct pldm_query_device_identifiers_resp) + total_size_of_iana_descriptor +
		    total_size_of_device_id_descriptor + total_size_of_slot_descriptor >
	    PLDM_MAX_DATA_SIZE) {
		LOG_ERR("QueryDeviceIdentifiers data length is over PLDM_MAX_DATA_SIZE define size %d",
			PLDM_MAX_DATA_SIZE);
		resp_p->completion_code = PLDM_ERROR;
		return PLDM_ERROR;
	}

	// Allocate data for tlv which including descriptors data
	struct pldm_descriptor_tlv *tlv_ptr = malloc(total_size_of_iana_descriptor);
	if (tlv_ptr == NULL) {
		LOG_ERR("Memory allocation failed!");
		return PLDM_ERROR;
	}

	tlv_ptr->descriptor_type = PLDM_FWUP_IANA_ENTERPRISE_ID;
	tlv_ptr->descriptor_length = PLDM_FWUP_IANA_ENTERPRISE_ID_LENGTH;
	memcpy(tlv_ptr->descriptor_data, iana, sizeof(iana));

	uint8_t *end_of_id_ptr =
		(uint8_t *)resp + sizeof(struct pldm_query_device_identifiers_resp);

	memcpy(end_of_id_ptr, tlv_ptr, total_size_of_iana_descriptor);
	free(tlv_ptr);

	tlv_ptr = malloc(total_size_of_device_id_descriptor);
	if (tlv_ptr == NULL) {
		LOG_ERR("Memory allocation failed!");
		return PLDM_ERROR;
	}

	tlv_ptr->descriptor_type = PLDM_PCI_DEVICE_ID;
	tlv_ptr->descriptor_length = PLDM_PCI_DEVICE_ID_LENGTH;
	memcpy(tlv_ptr->descriptor_data, deviceId, sizeof(deviceId));

	end_of_id_ptr += total_size_of_iana_descriptor;
	memcpy(end_of_id_ptr, tlv_ptr, total_size_of_device_id_descriptor);
	free(tlv_ptr);

	tlv_ptr = malloc(total_size_of_slot_descriptor);
	if (tlv_ptr == NULL) {
		LOG_ERR("Memory allocation failed!");
		return PLDM_ERROR;
	}

	tlv_ptr->descriptor_type = PLDM_ASCII_MODEL_NUMBER_SHORT_STRING;
	tlv_ptr->descriptor_length = PLDM_ASCII_MODEL_NUMBER_SHORT_STRING_LENGTH;
	memcpy(tlv_ptr->descriptor_data, slot, sizeof(slot));

	end_of_id_ptr += total_size_of_device_id_descriptor;
	memcpy(end_of_id_ptr, tlv_ptr, total_size_of_slot_descriptor);
	free(tlv_ptr);

	resp_p->device_identifiers_len = total_size_of_iana_descriptor +
					 total_size_of_device_id_descriptor +
					 total_size_of_slot_descriptor;

	*resp_len = sizeof(struct pldm_query_device_identifiers_resp) +
		    total_size_of_iana_descriptor + total_size_of_device_id_descriptor +
		    total_size_of_slot_descriptor;

	LOG_INF("pldm_query_device_identifiers done");
	return PLDM_SUCCESS;
}

void load_pldmupdate_comp_config(void)
{
	if (comp_config) {
		LOG_WRN("PLDM update component table has already been load");
		return;
	}

	comp_config_count = ARRAY_SIZE(PLDMUPDATE_FW_CONFIG_TABLE);
	comp_config = malloc(sizeof(pldm_fw_update_info_t) * comp_config_count);
	if (!comp_config) {
		LOG_ERR("comp_config malloc failed");
		return;
	}

	memcpy(comp_config, PLDMUPDATE_FW_CONFIG_TABLE, sizeof(PLDMUPDATE_FW_CONFIG_TABLE));
}

// vr update
static uint8_t pldm_pre_vr_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, 1);

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;
	if (get_asic_board_id() != ASIC_BOARD_ID_EVB && p->comp_id == COMPNT_VR_3V3) {
		LOG_ERR("only evb support 3V3 vr update");
		return 1;
	}

	/* Stop sensor polling */
	set_plat_sensor_polling_enable_flag(false);
	k_msleep(2000);

	uint8_t sensor_id = 0;
	char sensor_name[MAX_AUX_SENSOR_NAME_LEN] = { 0 };

	if (!find_sensor_id_and_name_by_firmware_comp_id(p->comp_id, &sensor_id, sensor_name)) {
		LOG_ERR("Can't find sensor id and name by comp id: 0x%x", p->comp_id);
		return 1;
	}

	sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);
	CHECK_NULL_ARG_WITH_RETURN(cfg, 1);

	/* Get bus and target address by sensor number in sensor configuration */
	p->bus = cfg->port;
	p->addr = cfg->target_addr;

	return 0;
}
static uint8_t pldm_post_vr_update(void *fw_update_param)
{
	ARG_UNUSED(fw_update_param);

	/* Start sensor polling */
	k_msleep(2000);
	set_plat_sensor_polling_enable_flag(true);

	return 0;
}
static bool get_vr_fw_version(void *info_p, uint8_t *buf, uint8_t *len)
{
	CHECK_NULL_ARG_WITH_RETURN(info_p, false);
	CHECK_NULL_ARG_WITH_RETURN(buf, false);
	CHECK_NULL_ARG_WITH_RETURN(len, false);

	pldm_fw_update_info_t *p = (pldm_fw_update_info_t *)info_p;

	bool ret = false;
	uint8_t sensor_id = 0;
	char sensor_name[MAX_AUX_SENSOR_NAME_LEN] = { 0 };

	if (is_mb_dc_on() == false)
		return ret;

	if (!find_sensor_id_and_name_by_firmware_comp_id(p->comp_identifier, &sensor_id,
							 sensor_name)) {
		LOG_ERR("Can't find sensor id and name by comp id: 0x%x", p->comp_identifier);
		return ret;
	}

	sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);
	CHECK_NULL_ARG_WITH_RETURN(cfg, ret);

	if ((cfg->pre_sensor_read_hook)) {
		if ((cfg->pre_sensor_read_hook)(cfg, cfg->pre_sensor_read_args) == false) {
			LOG_DBG("%d read vr fw pre hook fail!", sensor_id);
			return false;
		}
	};

	uint8_t vr_module = get_vr_module();
	uint32_t version = 0;
	uint16_t remain = 0xFFFF;
	switch (cfg->type) {
	case sensor_dev_mp2971:
		if (!mp2971_get_checksum(cfg->port, cfg->target_addr, &version)) {
			LOG_ERR("The VR MPS2971 version reading failed");
			goto err;
		}
		break;
	case sensor_dev_mp29816a:
		if (!mp29816a_get_fw_version(cfg->port, cfg->target_addr, &version)) {
			LOG_ERR("The VR MPS29816a version reading failed");
			goto err;
		}
		break;
	case sensor_dev_raa228249:
		if (!raa228249_get_crc(cfg->port, cfg->target_addr, &version)) {
			LOG_ERR("The VR RAA228249 version reading failed");
			goto err;
		}
		if (raa228249_get_remaining_wr(cfg->port, cfg->target_addr, (uint8_t *)&remain) <
		    0) {
			LOG_ERR("The VR RAA228249 remaining reading failed");
			goto err;
		}
		break;
	default:
		LOG_ERR("Unsupport VR type(%d)", cfg->type);
		goto err;
	}

	if (cfg->type == sensor_dev_mp2891 || cfg->type == sensor_dev_mp29816a)
		version = sys_cpu_to_be16(version);
	else if (cfg->type == sensor_dev_raa228249 || cfg->type == sensor_dev_mp2971)
		version = sys_cpu_to_be32(version);
	else
		LOG_ERR("Unsupport VR type(%d)", cfg->type);

	const char *vr_name[] = {
		[VR_MODULE_MPS] = "MPS ",
		[VR_MODULE_RNS] = "RNS ",
		[VR_MODULE_UNKNOWN] = NULL,
	};

	const char *remain_str_p = ", Remaining Write: ";
	uint8_t *buf_p = buf;
	const uint8_t *vr_name_p = vr_name[vr_module];
	*len = 0;

	if (!vr_name_p) {
		LOG_ERR("The pointer of VR string name is NULL");
		goto err;
	}

	memcpy(buf_p, vr_name_p, strlen(vr_name_p));
	buf_p += strlen(vr_name_p);
	*len += strlen(vr_name_p);

	if (cfg->type == sensor_dev_mp2891 || cfg->type == sensor_dev_mp29816a) {
		*len += bin2hex((uint8_t *)&version, 2, buf_p, 4);
		buf_p += 4;
	} else if (cfg->type == sensor_dev_raa228249 || cfg->type == sensor_dev_mp2971) {
		*len += bin2hex((uint8_t *)&version, 4, buf_p, 8);
		buf_p += 8;
	} else {
		LOG_ERR("Unsupport VR type(%d)", cfg->type);
	}

	memcpy(buf_p, remain_str_p, strlen(remain_str_p));
	buf_p += strlen(remain_str_p);
	*len += strlen(remain_str_p);

	if (remain != 0xFFFF) {
		uint8_t packed_remain = (uint8_t)((remain % 10) | (remain / 10 << 4));
		*len += bin2hex(&packed_remain, 1, buf_p, 2);
		buf_p += 2;
	} else {
		*len += bin2hex((uint8_t *)&remain, 2, buf_p, 4);
		buf_p += 4;
	}

	ret = true;

err:
	if ((cfg->post_sensor_read_hook)) {
		if ((cfg->post_sensor_read_hook)(cfg, cfg->post_sensor_read_args, 0) == false) {
			LOG_DBG("%d read vr fw post hook fail!", sensor_id);
			ret = false;
		}
	}
	return ret;
}

bool find_sensor_id_and_name_by_firmware_comp_id(uint8_t comp_identifier, uint8_t *sensor_id,
						 char *sensor_name)
{
	CHECK_NULL_ARG_WITH_RETURN(sensor_id, false);
	CHECK_NULL_ARG_WITH_RETURN(sensor_name, false);

	for (uint8_t i = 0; i < ARRAY_SIZE(vr_compnt_mapping_sensor_table); i++) {
		if (vr_compnt_mapping_sensor_table[i].firmware_comp_id == comp_identifier) {
			*sensor_id = vr_compnt_mapping_sensor_table[i].plat_pldm_sensor_id;
			strncpy(sensor_name, vr_compnt_mapping_sensor_table[i].sensor_name,
				MAX_AUX_SENSOR_NAME_LEN);
			return true;
		}
	}

	return false;
}

void plat_reset_prepare()
{
	const char *i2c_labels[] = { "I2C_0", "I2C_1", "I2C_2", "I2C_3", "I2C_4",  "I2C_5",
				     "I2C_6", "I2C_7", "I2C_8", "I2C_9", "I2C_10", "I2C_11" };

	for (int i = 0; i < ARRAY_SIZE(i2c_labels); i++) {
		const struct device *i2c_dev = device_get_binding(i2c_labels[i]);
		if (!i2c_dev) {
			LOG_ERR("Failed to get binding for %s", i2c_labels[i]);
			continue;
		}

		int ret = i2c_npcm_device_disable(i2c_dev);
		if (ret) {
			LOG_ERR("Failed to disable %s (ret=%d)", i2c_labels[i], ret);
		} else {
			LOG_INF("%s disabled", i2c_labels[i]);
		}
	}
}

void pal_warm_reset_prepare()
{
	LOG_INF("cmd platform warm reset prepare");
	plat_reset_prepare();
}
