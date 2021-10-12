#include <zephyr.h>
#include <sys/printk.h>
#include <logging/log.h>
#include <shell/shell.h>
#include <drivers/flash.h>
#include <device.h>
#include <soc.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sys/util.h>
#include "cmsis_os2.h"
#include "util_spi.h"

static char *flash_device[6] = {
  "fmc_cs0",
  "fmc_cs1",
  "spi1_cs0",
  "spi1_cs1",
  "spi2_cs0",
  "spi2_cs1"
};

static int do_erase_write_verify(const struct device *flash_device,
      uint32_t op_addr, uint8_t *write_buf, uint8_t *read_back_buf,
      uint32_t erase_sz)
{
  uint32_t ret = 0;
  uint32_t i;

  ret = flash_erase(flash_device, op_addr, erase_sz);
  if (ret != 0) {
    printk("[%s][%d] erase failed at %d.\n",
      __func__, __LINE__, op_addr);
    goto end;
  }

  ret = flash_write(flash_device, op_addr, write_buf, erase_sz);
  if (ret != 0) {
    printk("[%s][%d] write failed at %d.\n",
      __func__, __LINE__, op_addr);
    goto end;
  }

  flash_read(flash_device, op_addr, read_back_buf, erase_sz);
  if (ret != 0) {
    printk("[%s][%d] write failed at %d.\n",
      __func__, __LINE__, op_addr);
    goto end;
  }

  if (memcmp(write_buf, read_back_buf, erase_sz) != 0) {
    ret = -EINVAL;
    printk("ERROR: %s %d fail to write flash at 0x%x\n",
        __func__, __LINE__, op_addr);
    printk("to be written:\n");
    for (i = 0; i < 256; i++) {
      printk("%x ", write_buf[i]);
      if (i % 16 == 15)
        printk("\n");
    }

    printk("readback:\n");
    for (i = 0; i < 256; i++) {
      printk("%x ", read_back_buf[i]);
      if (i % 16 == 15)
        printk("\n");
    }

    goto end;
  }

end:
  return ret;
}


static int do_update(const struct device *flash_device,
        off_t offset, uint8_t *buf, size_t len)
{
  int ret = 0;
  uint32_t flash_sz = flash_get_flash_size(flash_device);
  uint32_t sector_sz = flash_get_write_block_size(flash_device);
  uint32_t flash_offset = (uint32_t)offset;
  uint32_t remain, op_addr = 0, end_sector_addr;
  uint8_t *update_ptr = buf, *op_buf = NULL, *read_back_buf = NULL;
  bool update_it = false;

  if (flash_sz < flash_offset + len) {
    printk("ERROR: update boundary exceeds flash size. (%d, %d, %d)\n",
      flash_sz, flash_offset, len);
    ret = -EINVAL;
    goto end;
  }

  op_buf = (uint8_t *)k_malloc(sector_sz);
  if (op_buf == NULL) {
    printk("heap full %d %d\n", __LINE__, sector_sz);
    ret = -EINVAL;
    goto end;
  }

  read_back_buf = (uint8_t *)k_malloc(sector_sz);
  if (read_back_buf == NULL) {
    printk("heap full %d %d\n", __LINE__, sector_sz);
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
    ret = do_erase_write_verify(flash_device, op_addr, op_buf,
                read_back_buf, sector_sz);
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

    ret = do_erase_write_verify(flash_device, op_addr, op_buf,
                read_back_buf, sector_sz);
    if (ret != 0)
      goto end;

    op_addr += remain;
  }

end:

  if (op_buf != NULL)
    k_free(op_buf);
  if (read_back_buf != NULL)
    k_free(read_back_buf);

  return ret;
}

uint8_t fw_update(uint32_t offset, uint16_t msg_len, uint8_t *msg_buf, bool update_en, uint8_t spi_bus) {
  static bool is_init = 0;
  static uint8_t *txbuf = NULL;
  static uint32_t buf_offset = 0;
  uint32_t ret = 0;
  const struct device *flash_dev;

  if(!is_init) {
    if( (offset & 0x0000) != 0 ) {
      return fwupdate_error_offset;
    }

    if (txbuf != NULL) {
      free(txbuf);
      txbuf = NULL;
    }
    txbuf = (uint8_t *)malloc(sector_sz_64k);
    if(txbuf == NULL) { // Retry alloc
      k_msleep(100);
      txbuf = (uint8_t *)malloc(sector_sz_64k);
    }
    if(txbuf == NULL) {
      printk("spi index%x update buffer alloc fail\n", spi_bus);
      return fwupdate_out_of_heap;
    }
    is_init = 1;
    buf_offset = 0;
    k_msleep(10);
  }

  if( (buf_offset + msg_len) > sector_sz_64k ) {
    printk("spi bus%x recv data %d over sector size %d\n", spi_bus, buf_offset + msg_len, sector_sz_64k);
    free(txbuf);
    txbuf = NULL;
    k_msleep(10);
    is_init = 0;
    return fwupdate_over_length;
  }

  if( (offset % sector_sz_64k) != buf_offset  ) {
    printk("spi bus%x recorded offset 0x%x but updating 0x%x\n", spi_bus, buf_offset, offset % sector_sz_64k);
    free(txbuf);
    txbuf = NULL;
    k_msleep(10);
    is_init = 0;
    return fwupdate_repeated_updated;
  }

  if(FW_UPDATE_DEBUG) {
    printk("spi bus%x update offset %x %x, msg_len %d, update_en %d, msg_buf: %2x %2x %2x %2x\n", spi_bus, offset, buf_offset, msg_len, update_en, msg_buf[0], msg_buf[1], msg_buf[2], msg_buf[3]);
  }

  memcpy(&txbuf[buf_offset], msg_buf, msg_len);
  buf_offset += msg_len;

  if( (buf_offset == sector_sz_64k) || update_en ) {  // Update fmc while collect 64k bytes data or BMC signal last image package with target | 0x80
    flash_dev = device_get_binding(flash_device[spi_bus]);
    uint8_t sector = 16;
    uint32_t txbuf_offset;
    uint32_t update_offset;

    for (int i = 0; i < sector; i++) {
      txbuf_offset = sector_sz_4k * i;
      update_offset = (offset / sector_sz_64k) * sector_sz_64k + txbuf_offset;
      ret = do_update(flash_dev, update_offset, &txbuf[txbuf_offset], sector_sz_4k);
      if (ret) {
        printk("SPI update fail status: %x\n", ret);
        break;
      }
    }
    if (!ret) {
      printk("Update success\n");
    }
    free(txbuf);
    txbuf = NULL;
    k_msleep(10);
    is_init = 0;

    if(FW_UPDATE_DEBUG) {
      printk("***update %x, offset %x, sector_sz_16k %x\n", (offset / sector_sz_16k) * sector_sz_16k, offset, sector_sz_16k);
    }

    if ( update_en && spi_bus == (devspi_fmc_cs0) ) {  // reboot bic itself after fw update
      submit_bic_warm_reset();
    }

    return ret;
  }

  return fwupdate_success;
}
