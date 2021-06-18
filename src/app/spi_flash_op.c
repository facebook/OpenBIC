/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "common.h"
#include "flash_api.h"
#include "fmc_spi_err.h"
#include "objects.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint32_t aspeed_read_from_flash(spi_t spi, uint32_t cs,
			uint8_t *read_buf, uint32_t flash_offset,
			uint32_t read_sz)
{
	uint32_t ret = 0;
	fmc_spi_priv_t *priv = (fmc_spi_priv_t *)spi.device->private;
	flash_t *flash = priv->chipes[cs].ctx;
	uint32_t flash_sz = flash->total_sz;

	if (flash_sz < flash_offset + read_sz) {
		printf("ERROR: read boundary exceeds flash maximum size. (%d, %d, %d)\n",
			flash_sz, flash_offset, read_sz);
		ret = ERR_INVALID_READ_SZ;
		goto end;
	}

	//printf("reading %dB from %s cs %d... ", read_sz, priv->name, cs);

	aspeed_spi_flash_read(flash, flash_offset, read_sz, read_buf);

end:
	//printf(" %s.\n", ret ? "FAILED" : "done");

	return ret;
}

uint32_t aspeed_erase_flash_sectors(spi_t spi, uint32_t cs, uint32_t flash_offset, uint32_t len)
{
	uint32_t ret = 0;
	fmc_spi_priv_t *priv = (fmc_spi_priv_t *)spi.device->private;
	flash_t *flash = priv->chipes[cs].ctx;
	uint32_t sector_sz = flash->sector_sz;
	uint32_t op_offset;

	//printf("eraseing %dB from 0x%x of %s cs %d... ", len, offset, priv->name, cs);

	if (flash->total_sz < flash_offset + len) {
		printf("ERROR: erase boundary exceeds flash maximum size. (%d, %d, %d)\n",
			flash->total_sz, flash_offset, len);
		ret = ERR_INVALID_READ_SZ;
		goto end;
	}

	if ((flash_offset % sector_sz) != 0) {
		printf("ERROR: flash offset, 0x%x, is not multiple of (0x%xB)\n", flash_offset, sector_sz);
		return ERR_INVALID_ERASE_PARAMETER;
	}

	if ((len % sector_sz) != 0) {
		printf("ERROR: erase size, 0x%x, is not multiple of (%dB)\n", len, sector_sz);
		return ERR_INVALID_ERASE_PARAMETER;
	}

	for (op_offset = flash_offset; op_offset < flash_offset + len; op_offset += sector_sz) {
		aspeed_spi_flash_erase(flash, op_offset);
	}

end:
	return ret;
}

uint32_t aspeed_write_to_flash(spi_t spi, uint32_t cs, uint8_t *write_buf,
			uint32_t flash_offset, uint32_t len)
{
	uint32_t ret = 0;
	fmc_spi_priv_t *priv = (fmc_spi_priv_t *)spi.device->private;
	flash_t *flash = priv->chipes[cs].ctx;
	uint32_t page_sz = flash->page_sz;
	uint32_t write_sz, op_offset;
	uint8_t *tmp_ptr = write_buf;

	if ((flash_offset % page_sz) != 0)
		printf("warning: write offset, %x, is not multiple of page size, %x.\n", flash_offset, page_sz);

	if (flash->total_sz < flash_offset + len) {
		printf("ERROR: write boundary exceeds flash maximum size. (%d, %d, %d)\n",
			flash->total_sz, flash_offset, len);
		ret = ERR_INVALID_READ_SZ;
		goto end;
	}

	op_offset = flash_offset;

	while (len != 0) {
		if (page_sz <= len)
			write_sz = page_sz;
		else
			write_sz = len;

		aspeed_spi_flash_write(flash, op_offset, write_sz, tmp_ptr);

		op_offset += write_sz;
		tmp_ptr += write_sz;
		len -= write_sz;
	}

end:
	return ret;
}

/* aspeed_erase_write_verify is designed for erase -> write -> check
 * a single sector.
 */
uint32_t aspeed_erase_write_verify(spi_t spi, uint32_t cs,
			uint32_t op_addr, uint8_t *write_buf, uint8_t *read_back_buf,
			uint32_t erase_sz, bool show_process, uint32_t percent_current)
{
	uint32_t ret = 0;
	uint32_t i;

	if (show_process)
		printf("\b\b\b\be%2d%%", percent_current);

	aspeed_erase_flash_sectors(spi, cs, op_addr, erase_sz);

	if (show_process)
		printf("\b\b\b\bw%2d%%", percent_current);


	aspeed_write_to_flash(spi, cs, write_buf, op_addr, erase_sz);

	if (show_process)
		printf("\b\b\b\br%2d%%", percent_current);


	aspeed_read_from_flash(spi, cs, read_back_buf, op_addr, erase_sz);

	if (show_process)
		printf("\b\b\b\bc%2d%%", percent_current);

	if (memcmp(write_buf, read_back_buf, erase_sz) != 0) {
		ret = ERR_FAIL_TO_WRITE_TO_FLASH;
		printf("ERROR: %s %d fail to write flash at 0x%x\n",
				__func__, __LINE__, op_addr);
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

uint32_t aspeed_update_flash(spi_t spi, uint32_t cs,
			uint8_t *update_buf, uint32_t flash_offset, uint32_t len,
			bool show_progress)
{
	uint32_t ret = 0;
	fmc_spi_priv_t *priv = (fmc_spi_priv_t *)spi.device->private;
	flash_t *flash = priv->chipes[cs].ctx;
	uint32_t flash_sz = flash->total_sz;
	uint32_t sector_sz = flash->sector_sz;
	uint32_t remain, op_addr = 0, end_sector_addr;
	uint32_t percent_last = 0, percent_current = 0;
	uint8_t *op_buf = NULL, *update_ptr = update_buf, *read_back_buf = NULL;
	bool update_it = false;

	printf("writing %dB to %s cs %d... ", len, priv->name, cs);

	if (flash_sz < flash_offset + len) {
		printf("ERROR: update boundary exceeds flash maximum size. (%d, %d, %d)\n",
			flash_sz, flash_offset, len);
		ret = ERR_INVALID_READ_SZ;
		goto end;
	}

	op_buf = (uint8_t *)pvPortMalloc(sector_sz);
	if (op_buf == NULL) {
		printf("heap full %d %d\n", __LINE__, sector_sz);
		ret = ERR_INVALID_PTR;
		goto end;
	}

	read_back_buf = (uint8_t *)pvPortMalloc(sector_sz);
	if (read_back_buf == NULL) {
		printf("heap full %d %d\n", __LINE__, sector_sz);
		ret = ERR_INVALID_PTR;
		goto end;
	}

	if (show_progress) {
		printf(" ");
		printf("\b  0%%");
	}

	/* initial op_addr */
	op_addr = (flash_offset / sector_sz) * sector_sz;

	/* handle the start part which is not multiple of sector size */
	if (flash_offset % sector_sz != 0) {
		ret = aspeed_read_from_flash(spi, cs, op_buf, op_addr, sector_sz);
		if (ret != 0)
			goto end;

		remain = MIN(sector_sz - (flash_offset % sector_sz), len);
		memcpy((uint8_t *)op_buf + (flash_offset % sector_sz), update_ptr, remain);
		ret = aspeed_erase_write_verify(spi, cs, op_addr, op_buf, read_back_buf,
								sector_sz, show_progress, percent_current);
		if (ret != 0)
			goto end;

		op_addr += sector_sz;
		update_ptr += remain;
	}

	end_sector_addr = (flash_offset + len) / sector_sz * sector_sz;
	/* handle body */
	for (; op_addr < end_sector_addr;) {
		ret = aspeed_read_from_flash(spi, cs, op_buf, op_addr, sector_sz);
		if (ret != 0)
			goto end;

		if (show_progress)
			printf("\b\b\b\bc%2d%%", percent_current);

		if (memcmp(op_buf, update_ptr, sector_sz) != 0)
			update_it = true;

		if (update_it) {
			ret = aspeed_erase_write_verify(spi, cs, op_addr, update_ptr, read_back_buf,
								sector_sz, show_progress, percent_current);
			if (ret != 0)
				goto end;
		}

		if (show_progress) {
			percent_current = (uint32_t) ((op_addr - flash_offset) * 100 / len);
			if(percent_current != percent_last) {
				printf("\b\b\b\b %2d%%", percent_current);
				percent_last = percent_current;
			}
		}
		op_addr += sector_sz;
		update_ptr += sector_sz;
	}

	/* handle remain part */
	if (end_sector_addr < flash_offset + len) {
		if (show_progress)
			printf("\b\b\b\br%2d%%", percent_current);

		ret = aspeed_read_from_flash(spi, cs, op_buf, op_addr, sector_sz);
		if (ret != 0)
			goto end;

		remain = flash_offset + len - end_sector_addr;
		memcpy((uint8_t *)op_buf, update_ptr, remain);

		ret = aspeed_erase_write_verify(spi, cs, op_addr, op_buf, read_back_buf,
								sector_sz, show_progress, percent_current);
		if (ret != 0)
			goto end;

		op_addr += remain;
	}

	if (show_progress) {
		percent_current = (uint32_t) ((op_addr - flash_offset) * 100 / len);
		if(percent_current != percent_last) {
			printf("\b\b\b\b %2d%%", percent_current);
			percent_last = percent_current;
		}
	}

end:
	printf(" %s.\n", ret ? "FAILED" : "done");

	if (op_buf != NULL)
		vPortFree(op_buf);
	if (read_back_buf != NULL)
		vPortFree(read_back_buf);

	return ret;
}

