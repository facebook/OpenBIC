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

#ifndef SNOOP_H
#define SNOOP_H

#ifdef CONFIG_SNOOP_ASPEED

#define SENDPOSTCODE_STACK_SIZE 2048
#define SNOOP_STACK_SIZE 512
#define SNOOP_MAX_LEN 244

enum POSTCODE_COPY_TYPES {
	COPY_ALL_POSTCODE,
	COPY_SPECIFIC_POSTCODE,
};

extern int snoop_read_num;
void copy_snoop_read_buffer(uint8_t offset, int size_num, uint8_t *buffer, uint8_t copy_mode);
bool get_postcode_ok();
void reset_postcode_ok();
void init_snoop_thread();
void abort_snoop_thread();
void init_send_postcode_thread();

#endif

#endif
