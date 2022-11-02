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

#ifndef PCC_H
#define PCC_H

#ifdef CONFIG_PCC_ASPEED

#define PCC_STACK_SIZE 512
#define PCC_BUFFER_LEN 1024
#define PROCESS_POSTCODE_STACK_SIZE 2048

uint16_t copy_pcc_read_buffer(uint16_t start, uint16_t length, uint8_t *buffer,
			      uint16_t buffer_len);
void pcc_init();
void reset_pcc_buffer();
bool get_4byte_postcode_ok();
void reset_4byte_postcode_ok();

#endif

#endif
