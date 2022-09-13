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

#ifndef ISL69254IRAZ_T_H
#define ISL69254IRAZ_T_H

#define isl69254iraz_t_checksum_length 4

bool isl69254iraz_t_get_checksum(uint8_t bus, uint8_t target_addr, uint8_t *checksum);
bool isl69254iraz_t_get_remaining_write(uint8_t bus, uint8_t target_addr, uint8_t *remain_write);
#endif
