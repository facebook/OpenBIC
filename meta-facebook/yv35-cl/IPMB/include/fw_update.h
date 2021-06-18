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

#ifndef FW_UPDATE_H
#define FW_UPDATE_H

#define fmc_bus 0
#define spi0_bus 0
#define spi1_bus 1
#define spi2_bus 2

#define fmc_cs 0
#define spi0_cs 0
#define spi1_cs 1
#define spi2_cs 2

uint8_t fw_update(uint32_t offset, uint16_t msg_len, uint8_t *msg_buf, bool update_en, uint8_t bus, uint8_t cs);

enum {
  fwupdate_success,
  fwupdate_out_of_heap,
  fwupdate_over_length,
  fwupdate_repeated_updated,
  fwupdate_update_fail,
};

#endif
