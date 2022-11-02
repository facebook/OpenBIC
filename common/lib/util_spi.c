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

#include <zephyr.h>
#include <logging/log.h>
#include <shell/shell.h>
#include <drivers/flash.h>
#include <device.h>
#include <soc.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sys/util.h>
#include <drivers/spi_nor.h>
#include "cmsis_os2.h"
#include "util_spi.h"
#include "util_sys.h"
#include "libutil.h"
#include "ipmi.h"
#include <crypto/hash.h>

LOG_MODULE_REGISTER(util_spi);

static struct {
	char *name;
	bool isinit;
} flash_device_list[] = {
	[DEVSPI_FMC_CS0] = { "fmc_cs0", true },	   [DEVSPI_FMC_CS1] = { "fmc_cs1", false },
	[DEVSPI_SPI1_CS0] = { "spi1_cs0", false }, [DEVSPI_SPI1_CS1] = { "spi1_cs1", false },
	[DEVSPI_SPI2_CS0] = { "spi2_cs0", false }, [DEVSPI_SPI2_CS1] = { "spi2_cs1", false },
};

static int do_erase_write_verify(const struct device *flash_device, uint32_t op_addr,
				 uint8_t *write_buf, uint8_t *read_back_buf, uint32_t erase_sz)
{
	uint32_t ret = 0;

	ret = flash_erase(flash_device, op_addr, erase_sz);
	if (ret != 0) {
		LOG_ERR("Failed to erase %u.", op_addr);
		goto end;
	}

	ret = flash_write(flash_device, op_addr, write_buf, erase_sz);
	if (ret != 0) {
		LOG_ERR("Failed to write %u.", op_addr);
		goto end;
	}

	ret = flash_read(flash_device, op_addr, read_back_buf, erase_sz);
	if (ret != 0) {
		LOG_ERR("Failed to read %u.", op_addr);
		goto end;
	}

	if (memcmp(write_buf, read_back_buf, erase_sz) != 0) {
		ret = -EINVAL;
		LOG_ERR("Failed to write flash at 0x%x.", op_addr);
		LOG_HEXDUMP_ERR(write_buf, erase_sz, "to be written:");
		LOG_HEXDUMP_ERR(read_back_buf, erase_sz, "readback:");
		goto end;
	}

end:
	return ret;
}

static int do_update(const struct device *flash_device, off_t offset, uint8_t *buf, size_t len)
{
	int ret = 0;
	uint32_t flash_sz = flash_get_flash_size(flash_device);
	uint32_t sector_sz = flash_get_write_block_size(flash_device);
	uint32_t flash_offset = (uint32_t)offset;
	uint32_t remain, op_addr = 0, end_sector_addr;
	uint8_t *update_ptr = buf, *op_buf = NULL, *read_back_buf = NULL;
	bool update_it = false;

	if (flash_sz < flash_offset + len) {
		LOG_ERR("Update boundary exceeds flash size. (%u, %u, %u)", flash_sz, flash_offset,
			(unsigned int)len);
		ret = -EINVAL;
		goto end;
	}

	op_buf = (uint8_t *)malloc(sector_sz);
	if (op_buf == NULL) {
		LOG_ERR("Failed to allocate op_buf.");
		ret = -EINVAL;
		goto end;
	}

	read_back_buf = (uint8_t *)malloc(sector_sz);
	if (read_back_buf == NULL) {
		LOG_ERR("Failed to allocate read_back_buf.");
		ret = -EINVAL;
		goto end;
	}

	/* initial op_addr */
	op_addr = (flash_offset / sector_sz) * sector_sz;

	/* handle the start part which is not multiple of sector size */
	if (flash_offset % sector_sz != 0) {
		ret = flash_read(flash_device, op_addr, op_buf, sector_sz);
		if (ret != 0)
			goto end;

		remain = MIN(sector_sz - (flash_offset % sector_sz), len);
		memcpy((uint8_t *)op_buf + (flash_offset % sector_sz), update_ptr, remain);
		ret = do_erase_write_verify(flash_device, op_addr, op_buf, read_back_buf,
					    sector_sz);
		if (ret != 0)
			goto end;

		op_addr += sector_sz;
		update_ptr += remain;
	}

	end_sector_addr = (flash_offset + len) / sector_sz * sector_sz;
	/* handle body */
	for (; op_addr < end_sector_addr;) {
		ret = flash_read(flash_device, op_addr, op_buf, sector_sz);
		if (ret != 0)
			goto end;

		if (memcmp(op_buf, update_ptr, sector_sz) != 0)
			update_it = true;

		if (update_it) {
			ret = do_erase_write_verify(flash_device, op_addr, update_ptr,
						    read_back_buf, sector_sz);
			if (ret != 0)
				goto end;
		}

		op_addr += sector_sz;
		update_ptr += sector_sz;
	}

	/* handle remain part */
	if (end_sector_addr < flash_offset + len) {
		ret = flash_read(flash_device, op_addr, op_buf, sector_sz);
		if (ret != 0)
			goto end;

		remain = flash_offset + len - end_sector_addr;
		memcpy((uint8_t *)op_buf, update_ptr, remain);

		ret = do_erase_write_verify(flash_device, op_addr, op_buf, read_back_buf,
					    sector_sz);
		if (ret != 0)
			goto end;

		op_addr += remain;
	}

end:
	SAFE_FREE(op_buf);
	SAFE_FREE(read_back_buf);

	return ret;
}

#ifdef CONFIG_CRYPTO_ASPEED
#define HASH_DRV_NAME CONFIG_CRYPTO_ASPEED_HASH_DRV_NAME

uint8_t get_fw_sha256(uint8_t *msg_buf, uint32_t offset, uint32_t length, uint8_t flash_position)
{
	const struct device *flash_dev;
	uint8_t *buf = NULL;
	uint8_t ret = 0;
	bool need_free_section = false;

	if (flash_position >= ARRAY_SIZE(flash_device_list)) {
		return CC_PARAM_OUT_OF_RANGE;
	}

	if (msg_buf == NULL) {
		return CC_UNSPECIFIED_ERROR;
	}

	buf = (uint8_t *)malloc(length);
	if (buf == NULL) {
		LOG_ERR("Failed to allocate buf.");
		return CC_OUT_OF_SPACE;
	}

	flash_dev = device_get_binding(flash_device_list[flash_position].name);
	if (!flash_device_list[flash_position].isinit) {
		ret = spi_nor_re_init(flash_dev);
		if (ret != 0) {
			LOG_ERR("Failed to re-init flash, ret %d.", ret);
			ret = CC_UNSPECIFIED_ERROR;
			goto end;
		}
		flash_device_list[flash_position].isinit = true;
	}
	ret = flash_read(flash_dev, offset, buf, length);
	if (ret != 0) {
		LOG_ERR("Failed to read flash, ret %d.", ret);
		ret = CC_UNSPECIFIED_ERROR;
		goto end;
	}

	const struct device *dev = device_get_binding(HASH_DRV_NAME);

	uint8_t digest[SHA256_DIGEST_SIZE];
	struct hash_ctx ini;
	struct hash_pkt pkt;

	pkt.in_buf = buf;
	pkt.in_len = length;
	pkt.out_buf = digest;
	pkt.out_buf_max = sizeof(digest);

	ret = hash_begin_session(dev, &ini, HASH_SHA256);
	if (ret) {
		LOG_ERR("hash_begin_session error, ret %d.", ret);
		ret = CC_UNSPECIFIED_ERROR;
		goto end;
	}

	need_free_section = true;

	ret = hash_update(&ini, &pkt);
	if (ret) {
		LOG_ERR("hash_update error, ret %d.", ret);
		ret = CC_UNSPECIFIED_ERROR;
		goto end;
	}

	ret = hash_final(&ini, &pkt);
	if (ret) {
		LOG_ERR("hash_final error, ret %d.", ret);
		ret = CC_UNSPECIFIED_ERROR;
		goto end;
	}

	memcpy(msg_buf, &digest[0], sizeof(digest));
	ret = CC_SUCCESS;
end:
	SAFE_FREE(buf);

	if (need_free_section) {
		hash_free_session(dev, &ini);
	}
	return ret;
}
#endif

uint8_t fw_update(uint32_t offset, uint16_t msg_len, uint8_t *msg_buf, bool sector_end,
		  uint8_t flash_position)
{
	static bool is_init = 0;
	static uint8_t *txbuf = NULL;
	static uint32_t buf_offset = 0;
	uint32_t ret = 0;
	const struct device *flash_dev;

	if (!is_init) {
		SAFE_FREE(txbuf);
		txbuf = (uint8_t *)malloc(SECTOR_SZ_64K);
		if (txbuf == NULL) { // Retry alloc
			k_msleep(100);
			txbuf = (uint8_t *)malloc(SECTOR_SZ_64K);
		}
		if (txbuf == NULL) {
			LOG_ERR("SPI index %d, failed to allocate txbuf.", flash_position);
			return FWUPDATE_OUT_OF_HEAP;
		}
		is_init = 1;
		buf_offset = 0;
		k_msleep(10);
	}

	if ((buf_offset + msg_len) > SECTOR_SZ_64K) {
		LOG_ERR("SPI index %d, recv data 0x%x over sector size 0x%x", flash_position,
			buf_offset + msg_len, SECTOR_SZ_64K);
		SAFE_FREE(txbuf);
		k_msleep(10);
		is_init = 0;
		return FWUPDATE_OVER_LENGTH;
	}

	if ((offset % SECTOR_SZ_64K) != buf_offset) {
		LOG_ERR("SPI index %d, recorded offset 0x%x but updating 0x%x\n", flash_position,
			buf_offset, offset % SECTOR_SZ_64K);
		SAFE_FREE(txbuf);
		txbuf = NULL;
		k_msleep(10);
		is_init = 0;
		return FWUPDATE_REPEATED_UPDATED;
	}

	LOG_DBG("SPI index %d, update offset 0x%x 0x%x, msg_len %d, sector_end %d,"
		" msg_buf: 0x%2x 0x%2x 0x%2x 0x%2x\n",
		flash_position, offset, buf_offset, msg_len, sector_end, msg_buf[0], msg_buf[1],
		msg_buf[2], msg_buf[3]);

	memcpy(&txbuf[buf_offset], msg_buf, msg_len);
	buf_offset += msg_len;

	// Update fmc while collect 64k bytes data or BMC signal last image package with target | 0x80
	if ((buf_offset == SECTOR_SZ_64K) || sector_end) {
		flash_dev = device_get_binding(flash_device_list[flash_position].name);
		if (!flash_device_list[flash_position].isinit) {
			uint8_t rc = 0;
			rc = spi_nor_re_init(flash_dev);
			if (rc != 0) {
				return rc;
			}
			flash_device_list[flash_position].isinit = true;
		}
		uint8_t sector = 16;
		uint32_t txbuf_offset;
		uint32_t update_offset;

		for (int i = 0; i < sector; i++) {
			txbuf_offset = SECTOR_SZ_4K * i;
			update_offset = (offset / SECTOR_SZ_64K) * SECTOR_SZ_64K + txbuf_offset;
			ret = do_update(flash_dev, update_offset, &txbuf[txbuf_offset],
					SECTOR_SZ_4K);
			if (ret) {
				LOG_ERR("Failed to update SPI, status %d", ret);
				break;
			}
		}
		if (!ret) {
			LOG_INF("Update success");
		}
		SAFE_FREE(txbuf);
		k_msleep(10);
		is_init = 0;

		LOG_DBG("Update 0x%x, offset 0x%x, SECTOR_SZ_16K 0x%x\n",
			(offset / SECTOR_SZ_16K) * SECTOR_SZ_16K, offset, SECTOR_SZ_16K);

		if (sector_end &&
		    (flash_position == DEVSPI_FMC_CS0)) { // reboot bic itself after fw update
			submit_bic_warm_reset();
		}

		return ret;
	}

	return FWUPDATE_SUCCESS;
}

int read_fw_image(uint32_t offset, uint8_t msg_len, uint8_t *msg_buf, uint8_t flash_position)
{
	CHECK_NULL_ARG_WITH_RETURN(msg_buf, -EINVAL);

	const struct device *flash_dev;
	flash_dev = device_get_binding(flash_device_list[flash_position].name);

	if (!flash_device_list[flash_position].isinit) {
		int rc = 0;
		rc = spi_nor_re_init(flash_dev);
		if (rc != 0) {
			LOG_ERR("Failed to re-init flash, ret %d.", rc);
			return rc;
		}
		flash_device_list[flash_position].isinit = true;
	}
	uint32_t flash_sz = flash_get_flash_size(flash_dev);
	if (flash_sz < offset + msg_len) {
		LOG_ERR("Read boundary 0x%x exceeds flash size 0x%x.", offset + msg_len, flash_sz);
		return -EINVAL;
	}

	return flash_read(flash_dev, offset, msg_buf, msg_len);
}

__weak uint8_t fw_update_cxl(uint32_t offset, uint16_t msg_len, uint8_t *msg_buf, bool sector_end)
{
	return FWUPDATE_NOT_SUPPORT;
}

__weak int pal_get_bios_flash_position()
{
	return -1;
}

__weak int pal_get_prot_flash_position()
{
	return -1;
}

__weak bool pal_switch_bios_spi_mux(int gpio_status)
{
	return true;
}
