/*
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "common.h"
#include "flash_api.h"
#include "objects.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fmc_spi_err.h"
#include "fw_update.h"
#include "bic-util.h"
#include "timer.h"

#define FW_UPDATE_DEBUG 0

extern spi_t fmc_spi[];

uint8_t fw_update(uint32_t offset, uint16_t msg_len, uint8_t *msg_buf, bool update_en, uint8_t spi_bus, uint8_t spi_cs) {
  static bool is_init = 0;
  static uint8_t *txbuf;
  static uint32_t buf_offset = 0;
  fmc_spi_priv_t *priv = (fmc_spi_priv_t *)fmc_spi[spi_bus].device->private;
  flash_info_t *info = flash_ids;
  uint32_t sector_sz = info->sector_sz;
  uint32_t ret = 0;
  uint32_t FreeHeapSize;

  if(!is_init) {
    if(FW_UPDATE_DEBUG) {
      printf("***init sector_sz %u, heap left %d\n", sector_sz, xPortGetFreeHeapSize());
    }

    taskENTER_CRITICAL(); // Get free heap size to make sure not pvPortMalloc fail
    FreeHeapSize = xPortGetFreeHeapSize();
    if(FreeHeapSize > sector_sz) {
      txbuf = (uint8_t*)pvPortMalloc(sector_sz);
    }
    taskEXIT_CRITICAL();
    osDelay(10);

    if(txbuf == NULL) {
      printf("spi bus%x update buffer alloc fail\n", spi_bus);
      return fwupdate_out_of_heap;
    }

    buf_offset = 0;
    is_init = 1;
  }

  if( (buf_offset + msg_len) > sector_sz ) {
    printf("spi bus%x recv data %u over sector size %u\n", spi_bus, buf_offset + msg_len, sector_sz);
    vPortFree(txbuf);
    osDelay(10);
    is_init = 0;
    return fwupdate_over_length;
  }

  if( (offset % sector_sz) != buf_offset  ) {
    printf("spi bus%x recorded offset 0x%x but updating 0x%x\n", spi_bus, buf_offset, offset % sector_sz);
    vPortFree(txbuf);
    osDelay(10);
    is_init = 0;
    return fwupdate_repeated_updated;
  }

  if(FW_UPDATE_DEBUG) {
    printf("spi bus%x update offset %x %x, msg_len %d, update_en %d, msg_buf: %2x %2x %2x %2x\n", spi_bus, offset, buf_offset, msg_len, update_en, msg_buf[0], msg_buf[1], msg_buf[2], msg_buf[3]);
  }

  memcpy(&txbuf[buf_offset], msg_buf, msg_len);
  buf_offset += msg_len;

  if( (buf_offset == sector_sz) || update_en ) {  // Update fmc while collect 64k bytes data or BMC signal last image package with target | 0x80
    configASSERT(txbuf);
    aspeed_flash_probe(&fmc_spi[spi_bus], spi_cs);
    ret = aspeed_update_flash(fmc_spi[spi_bus], spi_cs, txbuf, (offset / sector_sz) * sector_sz, buf_offset, false);
    vPortFree(txbuf);
    osDelay(10);
    is_init = 0;

    if(FW_UPDATE_DEBUG) {
      printf("***update %x, offset %x, sector_sz %x\n", (offset / sector_sz) * sector_sz, offset, sector_sz);
    }

    if ( update_en && spi_bus == (fmc_bus) ) {  // reboot bic itself after fw update
      bic_warm_reset();
    }

    if(ret) {
      printf("aspeed_update_flash update fail status: %x\n", ret);
      return fwupdate_update_fail;
    } else {
      return fwupdate_success;
    }
  }

  return fwupdate_success;
}
