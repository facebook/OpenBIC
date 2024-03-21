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

#ifndef LIBUTIL_H
#define LIBUTIL_H

#include <stdint.h>
#include "ipmb.h"
#include "hal_i2c.h"

#define SAFE_FREE(p)                                                                               \
	if (p) {                                                                                   \
		free(p);                                                                           \
		p = NULL;                                                                          \
	}

#define SETBIT(x, y) (x | (1ULL << y))
#define SETBITS(x, y, z) (x | (y << z))
#define GETBIT(x, y) ((x & (1ULL << y)) > y)
#define CLEARBIT(x, y) (x & (~(1ULL << y)))
#define CLEARBITS(x, y, z)                                                                         \
	for (int i = y; i <= z; ++i) {                                                             \
		x = CLEARBIT(x, i);                                                                \
	}

#define SHELL_CHECK_NULL_ARG(arg_ptr)                                                              \
	if (arg_ptr == NULL) {                                                                     \
		shell_error(shell, "Parameter \"" #arg_ptr "\" passed in as NULL");                \
		return;                                                                            \
	}

#define SHELL_CHECK_NULL_ARG_WITH_RETURN(arg_ptr, ret_val)                                         \
	if (arg_ptr == NULL) {                                                                     \
		shell_error(shell, "Parameter \"" #arg_ptr "\" passed in as NULL");                \
		return ret_val;                                                                    \
	}

#define CHECK_NULL_ARG(arg_ptr)                                                                    \
	if (arg_ptr == NULL) {                                                                     \
		LOG_DBG("Parameter \"" #arg_ptr "\" passed in as NULL");                           \
		return;                                                                            \
	}

#define CHECK_NULL_ARG_WITH_RETURN(arg_ptr, ret_val)                                               \
	if (arg_ptr == NULL) {                                                                     \
		LOG_DBG("Parameter \"" #arg_ptr "\" passed in as NULL");                           \
		return ret_val;                                                                    \
	}

#define CHECK_ARG_WITH_RETURN(cmp, ret_val)                                                        \
	if (cmp) {                                                                                 \
		LOG_DBG("Parameter \"" #cmp "\" true");                                            \
		return ret_val;                                                                    \
	}

#define CHECK_MSGQ_INIT(msgq) CHECK_NULL_ARG((msgq)->buffer_start);

#define CHECK_MSGQ_INIT_WITH_RETURN(msgq, ret_val)                                                 \
	CHECK_NULL_ARG_WITH_RETURN((msgq)->buffer_start, ret_val);

#define CHECK_MUTEX_INIT(mutex) CHECK_NULL_ARG((mutex)->wait_q.waitq.head);

#define CHECK_MUTEX_INIT_WITH_RETURN(mutex, ret_val)                                               \
	CHECK_NULL_ARG_WITH_RETURN((mutex)->wait_q.waitq.head, ret_val);

#define SET_FLAG_WITH_RETURN(flag, set_val, ret_val)                                               \
	if (flag != true) {                                                                        \
		LOG_DBG("Set parameter \"" #flag "\" to true");                                    \
		flag = set_val;                                                                    \
		return ret_val;                                                                    \
	}

enum BIT_SETTING_READING {
	BIT_CLEAR = 0,
	BIT_SET = 1,
	BIT_GET = 2,
};

enum BIT_SETTING_READING_N {
	BIT_SET_N = 0,
	BIT_CLEAR_N = 1,
};

enum BYTE_ENDIAN {
	SMALL_ENDIAN = 0,
	BIG_ENDIAN = 1,
};

ipmi_msg construct_ipmi_message(uint8_t seq_source, uint8_t netFn, uint8_t command,
				uint8_t source_inft, uint8_t target_inft, uint16_t data_len,
				uint8_t *data);

I2C_MSG construct_i2c_message(uint8_t bus_id, uint8_t address, uint8_t tx_len, uint8_t *data,
			      uint8_t rx_len);

void reverse_array(uint8_t arr[], uint8_t size);
int ascii_to_val(uint8_t ascii_byte);
uint32_t uint32_t_byte_reverse(uint32_t data);
void convert_uint32_t_to_uint8_t_pointer(uint32_t data_32, uint8_t *data_8, uint8_t len,
					 uint8_t endian);
void convert_uint8_t_pointer_to_uint32_t(uint32_t *data_32, const uint8_t *data_8, uint8_t len,
					 uint8_t endian);
double power(double x, int y);
int uint8_t_to_dec_ascii_pointer(uint8_t val, uint8_t *result, uint8_t len);
int find_byte_data_in_buf(const uint8_t *buf, uint8_t byte_data, int start_index, int end_index);
void clear_bits(uint32_t *value, int start_bit, int end_bit);

#endif
