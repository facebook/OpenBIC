// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 * Copyright (c) 2011-2014, Wind River Systems, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
/*
 * history:
 * copy bit operation macros from zephyr project
 * https://github.com/zephyrproject-rtos/zephyr/blob/master/include/sys/util.h
 */
#include <stdint.h>

/** Number of bits in a long int. */
#define BITS_PER_LONG	(__CHAR_BIT__ * __SIZEOF_LONG__)

/**
 * @brief Create a contiguous bitmask starting at bit position @p l
 *        and ending at position @p h.
 */
#define GENMASK(h, l) \
	(((~0UL) - (1UL << (l)) + 1) & (~0UL >> (BITS_PER_LONG - 1 - (h))))

#define BIT(n)  (1 << (n))

/** @brief 0 if @p cond is true-ish; causes a compile error otherwise. */
#define ZERO_OR_COMPILE_ERROR(cond) ((int) sizeof(char[1 - 2 * !(cond)]) - 1)

#if defined(__cplusplus)
/* The built-in function used below for type checking in C is not
 * supported by GNU C++.
 */
#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))

#else /* __cplusplus */
/**
 * @brief Zero if @p array has an array type, a compile error otherwise
 *
 * This macro is available only from C, not C++.
 */
#define IS_ARRAY(array) \
	ZERO_OR_COMPILE_ERROR( \
		!__builtin_types_compatible_p(__typeof__(array), \
					      __typeof__(&(array)[0])))

/**
 * @brief Number of elements in the given @p array
 *
 * In C++, due to language limitations, this will accept as @p array
 * any type that implements <tt>operator[]</tt>. The results may not be
 * particulary meaningful in this case.
 *
 * In C, passing a pointer as @p array causes a compile error.
 */
#define ARRAY_SIZE(array) \
	((long) (IS_ARRAY(array) + (sizeof(array) / sizeof((array)[0]))))

#endif /* __cplusplus */

#ifndef MAX
#define MAX(a,b)	(((a) > (b)) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a,b)	(((a) < (b)) ? (a) : (b))
#endif

#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))

#define bswap_32(x) \
	((((x) & 0xff000000u) >> 24) | (((x) & 0x00ff0000u) >> 8) \
	| (((x) & 0x0000ff00u) << 8) | (((x) & 0x000000ffu) << 24))


int fls(int mask);
int ffs(int mask);

#define CONCAT1(x, y)		x ## y
#define CONCAT(x, y)		CONCAT1(x, y)

#define abs(x)  ( (x<0) ? -x : x )

#define reg_read_poll_timeout(map, reg, val, cond, sleep_tick, timeout_tick)   \
	({                                                                         \
		uint32_t __timeout_tick = (timeout_tick);                              \
		uint32_t __start = xTaskGetTickCount();                                \
		int __ret = 0;                                                         \
		for (;;) {                                                             \
			val.value = map->reg.value;                                        \
			if (cond)                                                          \
				break;                                                         \
			if ((xTaskGetTickCount() - __start) > __timeout_tick) {            \
				__ret = -1;                                                    \
				break;                                                         \
			}                                                                  \
			if (sleep_tick)                                                    \
				vTaskDelay(sleep_tick);                                        \
		}                                                                      \
		__ret;                                                                 \
	})
