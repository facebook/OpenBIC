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

#ifndef PLAT_SPI_H
#define PLAT_SPI_H

enum wf_cxl_component {
	WF_COMPNT_CXL1 = 0x05,
	WF_COMPNT_CXL2 = 0x06,
};

int pal_dump_cxl_flash_data(uint8_t *read_buf, uint32_t offset, size_t length,
			    const struct device *flash_dev, uint8_t cxl_comp_id);
int pal_get_cxl_flash_position_by_id(uint8_t cxl_comp_id);
uint8_t pal_init_cxl_flash_device(uint8_t cxl_comp_id, const struct device **flash_dev);

#endif