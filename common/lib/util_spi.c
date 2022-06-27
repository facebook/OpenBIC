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

static char *flash_device[6] = { "fmc_cs0",  "fmc_cs1",	 "spi1_cs0",
				 "spi1_cs1", "spi2_cs0", "spi2_cs1" };
static bool isInitialized = false;
static int do_erase_write_verify(const struct device *flash_device, uint32_t op_addr,
				 uint8_t *write_buf, uint8_t *read_back_buf, uint32_t erase_sz)
{
	uint32_t ret = 0;
	uint32_t i;

	ret = flash_erase(flash_device, op_addr, erase_sz);
	if (ret != 0) {
		printf("[%s][%d] erase failed at %u.\n", __func__, __LINE__, op_addr);
		goto end;
	}

	ret = flash_write(flash_device, op_addr, write_buf, erase_sz);
	if (ret != 0) {
		printf("[%s][%d] write failed at %u.\n", __func__, __LINE__, op_addr);
		goto end;
	}

	ret = flash_read(flash_device, op_addr, read_back_buf, erase_sz);
	if (ret != 0) {
		printf("[%s][%d] write failed at %u.\n", __func__, __LINE__, op_addr);
		goto end;
	}

	if (memcmp(write_buf, read_back_buf, erase_sz) != 0) {
		ret = -EINVAL;
		printf("ERROR: %s %d fail to write flash at 0x%x\n", __func__, __LINE__, op_addr);
		printf("to be written:\n");
		for (i = 0; i < 256; i++) {
			printf("%x ", write_buf[i]);
			if (i % 16 == 15)
				printf("\n");
		}

		printf("readback:\n");
		for (i = 0; i < 256; i++) {
			printf("%x ", read_back_buf[i]);
			if (i % 16 == 15)
				printf("\n");
		}

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
		printf("ERROR: update boundary exceeds flash size. (%u, %u, %u)\n", flash_sz,
		       flash_offset, (unsigned int)len);
		ret = -EINVAL;
		goto end;
	}

	op_buf = (uint8_t *)malloc(sector_sz);
	if (op_buf == NULL) {
		printf("heap full %d %u\n", __LINE__, sector_sz);
		ret = -EINVAL;
		goto end;
	}

	read_back_buf = (uint8_t *)malloc(sector_sz);
	if (read_back_buf == NULL) {
		printf("heap full %d %u\n", __LINE__, sector_sz);
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
			printf("spi index%x update buffer alloc fail\n", flash_position);
			return FWUPDATE_OUT_OF_HEAP;
		}
		is_init = 1;
		buf_offset = 0;
		k_msleep(10);
	}

	if ((buf_offset + msg_len) > SECTOR_SZ_64K) {
		printf("spi bus%x recv data %d over sector size %d\n", flash_position,
		       buf_offset + msg_len, SECTOR_SZ_64K);
		SAFE_FREE(txbuf);
		k_msleep(10);
		is_init = 0;
		return FWUPDATE_OVER_LENGTH;
	}

	if ((offset % SECTOR_SZ_64K) != buf_offset) {
		printf("spi bus%x recorded offset 0x%x but updating 0x%x\n", flash_position,
		       buf_offset, offset % SECTOR_SZ_64K);
		SAFE_FREE(txbuf);
		txbuf = NULL;
		k_msleep(10);
		is_init = 0;
		return FWUPDATE_REPEATED_UPDATED;
	}

	if (FW_UPDATE_DEBUG) {
		printf("spi bus%x update offset %x %x, msg_len %d, sector_end %d, msg_buf: %2x %2x %2x %2x\n",
		       flash_position, offset, buf_offset, msg_len, sector_end, msg_buf[0],
		       msg_buf[1], msg_buf[2], msg_buf[3]);
	}

	memcpy(&txbuf[buf_offset], msg_buf, msg_len);
	buf_offset += msg_len;

	// Update fmc while collect 64k bytes data or BMC signal last image package with target | 0x80
	if ((buf_offset == SECTOR_SZ_64K) || sector_end) {
		flash_dev = device_get_binding(flash_device[flash_position]);
		if (flash_position == DEVSPI_SPI1_CS0 && !isInitialized) {
			uint8_t rc = 0;
			rc = spi_nor_re_init(flash_dev);
			if (rc != 0) {
				return rc;
			}
			isInitialized = true;
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
				printf("SPI update fail status: %x\n", ret);
				break;
			}
		}
		if (!ret) {
			printf("Update success\n");
		}
		SAFE_FREE(txbuf);
		k_msleep(10);
		is_init = 0;

		if (FW_UPDATE_DEBUG) {
			printf("***update %x, offset %x, SECTOR_SZ_16K %x\n",
			       (offset / SECTOR_SZ_16K) * SECTOR_SZ_16K, offset, SECTOR_SZ_16K);
		}

		if (sector_end &&
		    (flash_position == DEVSPI_FMC_CS0)) { // reboot bic itself after fw update
			submit_bic_warm_reset();
		}

		return ret;
	}

	return FWUPDATE_SUCCESS;
}

uint8_t fw_update_cxl(uint8_t flash_position)
{
	int ret = 0;
	const struct device *flash_dev;

	flash_dev = device_get_binding(flash_device[flash_position]);
	ret = spi_nor_re_init(flash_dev);
	if (ret != 0) {
		return FWUPDATE_UPDATE_FAIL;
	}
	//TODO: do real update until know CXL fw update format and progress
	return FWUPDATE_SUCCESS;
}

__weak int pal_get_bios_flash_position()
{
	return -1;
}

__weak bool pal_switch_bios_spi_mux(int gpio_status)
{
	return false;
}

__weak int pal_get_cxl_flash_position()
{
	return -1;
}

__weak bool pal_switch_cxl_spi_mux()
{
	return false;
}
