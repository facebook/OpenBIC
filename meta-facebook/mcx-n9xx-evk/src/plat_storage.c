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
 *
 * Real persistent-storage subsystem: NVS on the on-board W25Q64 QSPI
 * flash chip's "storage" partition (already defined in the board's own
 * devicetree, no board-specific code needed here). Stand-in for the
 * config/calibration storage a real BIC needs to survive a reset -
 * FRDM-MCXN947's wallabmc project (same silicon, different board) had
 * to disable this because its external QSPI flash is off by default;
 * this EVK's is on by default, so it just works.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kvss/nvs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/logging/log.h>

#include "plat_storage.h"

LOG_MODULE_REGISTER(plat_storage, LOG_LEVEL_INF);

#define NVS_PARTITION storage_partition
#define NVS_PARTITION_DEVICE FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET FIXED_PARTITION_OFFSET(NVS_PARTITION)

#define BOOT_COUNT_ID 1

static struct nvs_fs fs;
static uint32_t boot_count;

void plat_storage_init(void)
{
	int ret;
	struct flash_pages_info info;

	fs.flash_device = NVS_PARTITION_DEVICE;
	if (!device_is_ready(fs.flash_device)) {
		LOG_ERR("storage flash device not ready");
		return;
	}

	fs.offset = NVS_PARTITION_OFFSET;
	ret = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
	if (ret < 0) {
		LOG_ERR("flash_get_page_info_by_offs failed: %d", ret);
		return;
	}

	fs.sector_size = info.size;
	fs.sector_count = 4U;

	ret = nvs_mount(&fs);
	if (ret < 0) {
		LOG_ERR("nvs_mount failed: %d", ret);
		return;
	}

	ret = nvs_read(&fs, BOOT_COUNT_ID, &boot_count, sizeof(boot_count));
	if (ret < 0) {
		/* First boot ever, or entry not found yet. */
		boot_count = 0;
	}

	boot_count++;
	ret = nvs_write(&fs, BOOT_COUNT_ID, &boot_count, sizeof(boot_count));
	if (ret < 0) {
		LOG_ERR("nvs_write failed: %d", ret);
		return;
	}

	LOG_INF("persistent boot count: %u", boot_count);
}

uint32_t plat_storage_boot_count(void)
{
	return boot_count;
}
