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

#ifndef PLAT_PLDM_H
#define PLAT_PLDM_H

#define BMC_PLDM_DATA_MAXIMUM 150
#define HTTP_BOOT_DATA_MAXIMUM 2200 //The current estimated maximum value of the BIOS

uint8_t plat_pldm_get_http_boot_data(uint8_t *httpBootData, uint16_t *httpBootDataLen);

#endif
