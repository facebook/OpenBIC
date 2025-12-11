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

#include "plat_pcc.h"
#include "pcc.h"
#include <zephyr.h>
#include <logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(plat_pcc);

#define POSTCODE_PREFIX_BL          0xEE000000ul
#define POSTCODE_PREFIX_DDEE        0xDD000000ul
#define POSTCODE_PREFIX_ABL_ERROR   0xEA000000ul
#define POSTCODE_PREFIX_AGESA_ERROR 0xB0000000ul
#define POSTCODE_PREFIX_UNEXPECTED  0x00000000ul

#define GET_POSTCODE_PREFIX(code)   ((code) & 0xFF000000ul)
#define GET_HIGH_WORD(code)         (((code) >> 16) & 0xFFFF)
#define GET_LOW_WORD(code)          ((code) & 0xFFFF)
#define GET_24BIT_VALUE(code)       ((code) & 0x00FFFFFF)

#define BL_POSTCODE_MAX             0x009F
#define POSTCODE_EXCLUDE_ZERO       0x00000000
#define POSTCODE_EXCLUDE_ABCD       0x0000ABCD

static bool postcode_filter_enabled = true;


#define FILTERED_POSTCODE_BUFFER_SIZE 15

static uint32_t filtered_postcodes[FILTERED_POSTCODE_BUFFER_SIZE];
static uint8_t filtered_postcode_index = 0;
static uint8_t filtered_postcode_count = 0;
static struct k_mutex filtered_postcode_mutex;
static struct k_work store_filtered_postcode_work;
static uint32_t pending_filtered_postcode = 0;
static atomic_t pending_flag = ATOMIC_INIT(0);


// ABL Error codes for filtering (0xEA00xxxx)
static const uint16_t abl_filter_error_codes[] = {
	// APCB/Debug related
	0x0ACE,	0x0ACF,

	// Memory related errors
	0xBAAB,	0xBAAC,	0xBAAD,

	// ABL stage errors
	0xE0B5,	0xE0B6,	0xE0BC,	0xE0BD,

	// GMI/xGMI error codes
	0xB1C0,	0xB2C0,	0xB3C0,	0xB4C0,	0xB5C0,
	0xB6C0,	0xF1C0,	0xF2C0,	0xF3C0,	0xF4C0,
	0xF5C0,	0xF6C0,	0xF9C0,	0xFAC0,

	// Other ABL errors
	0xE0C5,	0xE0C6,	0xE0F9,	0xE106,	0xE10B,

	// E2xx series error codes
	0xE2A0,	0xE2A1,	0xE2A3,	0xE2A4,	0xE2A5,
	0xE2A6,	0xE2A7,	0xE2A8,	0xE2A9,	0xE2AA,
	0xE2AB,	0xE2AC,	0xE2AD,	0xE2AE,	0xE2B0,
	0xE2B1,	0xE2B2,	0xE2B3,	0xE2B4,	0xE2B5,
	0xE2B6,	0xE2B7,	0xE2B8,	0xE2B9,	0xE2BA,
	0xE2BB,	0xE2BC,	0xE2BD,	0xE2BE,	0xE2BF,
	0xEBC0,	0xE2C1,	0xE2C2,	0xE2C3,	0xE2C4,
	0xE2C5,	0xE2C6,	0xE2C7,	0xE2C8,	0xE2C9,
	0xE2CA,	0xE2CB,	0xE2CC,	0xE2CD,	0xE2CE,
	0xE2CF,	0xE2D0,	0xE2D1,	0xE2D2,	0xE2D3,
	0xE2D4,	0xE2D5,	0xE2D6,	0xE2D7,	0xE2D8,
	0xE2D9,	0xE2DA,	0xE2DB,	0xE2DC,	0xE2DD,
	0xE2DE,	0xE2DF,	0xE2E0,	0xE2E1,	0xE2E2,
	0xE2E3,	0xE2E4,	0xE2E5,	0xE2E7,	0xE2E8,
	0xE2EB,	0xE2EC,	0xE2ED,	0xE2EE,	0xE2EF,
	0xE2F0,	0xE2F1,	0xE2F2,	0xE2F3,	0xE2F4,
	0xE2F5,	0xE2F6,	0xE2F7,	0xE2F8,	0xE2F9,
	0xE2FA,	0xE2FB,	0xE2FC,	0xE2FD,	0xE2FE,
	0xE2FF,

	// E3xx series error codes
	0xE300,	0xE301,	0xE302,	0xE303,	0xE304,
	0xE305,	0xE306,	0xE307,	0xE308,	0xE309,
	0xE30A,	0xE30B,	0xE30C,	0xE30D,	0xE30E,
	0xE30F,	0xE310,	0xE311,	0xE312,	0xE313,
	0xE314,	0xE316,	0xE320,	0xE321,	0xE322,
	0xE327,	0xE328,	0xE329,	0xE32A,	0xE32B,
	0xE32C,	0xE32D,	0xE32E,	0xE32F,	0xE330,
	0xE331,	0xE332,	0xE333,	0xE334,	0xE335,
	0xE33C,	0xE33F,	0xE343,	0xE345,	0xE346,
	0xE347,	0xE360,
};

// AGESA Error codes for filtering (0xB0xxxxxx)
static const uint16_t agesa_filter_error_codes[] = {
	// CPM error codes
	0x0C01, 0x0C02,	0x0C03,

	// Memory/PMU errors
	0xA1F9,

	// PSP related errors
	0xA537, 0xA53B,	0xA53C, 0xA53D, 0xA53E,
	0xA53F,

	// Intrusion Detection errors
	0xA56A, 0xA56E, 0xA56F,

	// P2C mailbox errors
	0xA59F,
};

static inline bool binary_search_uint16(uint16_t code, const uint16_t *array, size_t size)
{
	size_t left = 0, right = size;
	
	while (left < right) {
		size_t mid = left + (right - left) / 2;
		
		if (array[mid] == code) {
			return true;
		}
		
		if (array[mid] < code) {
			left = mid + 1;
		} else {
			right = mid;
		}
	}
	
	return false;
}

static bool validate_bl_postcode(uint32_t postcode)
{
	uint16_t high_word = GET_HIGH_WORD(postcode);
	uint16_t low_word = GET_LOW_WORD(postcode);
	
	return (high_word == 0xEE00 && low_word <= BL_POSTCODE_MAX);
}

static bool validate_abl_error_postcode(uint32_t postcode)
{
	uint16_t high_word = GET_HIGH_WORD(postcode);
	uint16_t low_word = GET_LOW_WORD(postcode);
	
	if (high_word != 0xEA00) {
		return false;
	}
	
	return binary_search_uint16(low_word, abl_filter_error_codes,
	                            ARRAY_SIZE(abl_filter_error_codes));
}

static bool validate_agesa_error_postcode(uint32_t postcode)
{
	uint32_t value_24bit = GET_24BIT_VALUE(postcode);
	
	return binary_search_uint16(value_24bit, agesa_filter_error_codes,
	                            ARRAY_SIZE(agesa_filter_error_codes));
}

static void store_filtered_postcode_handler(struct k_work *work)
{
	uint32_t postcode = pending_filtered_postcode;
	atomic_set(&pending_flag, 0);	
	k_mutex_lock(&filtered_postcode_mutex, K_FOREVER);
	
	filtered_postcodes[filtered_postcode_index] = postcode;
	filtered_postcode_index = (filtered_postcode_index + 1) % 
	                          FILTERED_POSTCODE_BUFFER_SIZE;
	
	if (filtered_postcode_count < FILTERED_POSTCODE_BUFFER_SIZE) {
		filtered_postcode_count++;
	}
	
	k_mutex_unlock(&filtered_postcode_mutex);
}

void pcc_platform_store_postcode(uint32_t postcode)
{
	// Check if there's already a pending postcode
	if (atomic_set(&pending_flag, 1) != 0) {
		return;
	}

	pending_filtered_postcode = postcode;

	k_work_submit(&store_filtered_postcode_work);
}

void pcc_platform_filter_init(void)
{
	k_mutex_init(&filtered_postcode_mutex);
	k_work_init(&store_filtered_postcode_work, store_filtered_postcode_handler);
	
	memset(filtered_postcodes, 0, sizeof(filtered_postcodes));
	filtered_postcode_index = 0;
	filtered_postcode_count = 0;
	
	LOG_INF("Postcode filter initialized (filter %s)",
	        postcode_filter_enabled ? "enabled" : "disabled");
	
#ifdef CONFIG_DEBUG
	// Validate sorted arrays
	for (size_t i = 1; i < ARRAY_SIZE(abl_filter_error_codes); i++) {
		if (abl_filter_error_codes[i] <= abl_filter_error_codes[i-1]) {
			LOG_ERR("ABL codes not sorted at index %zu", i);
			k_panic();
		}
	}
	
	for (size_t i = 1; i < ARRAY_SIZE(agesa_filter_error_codes); i++) {
		if (agesa_filter_error_codes[i] <= agesa_filter_error_codes[i-1]) {
			LOG_ERR("AGESA codes not sorted at index %zu", i);
			k_panic();
		}
	}
	
	LOG_DBG("Filter arrays validated (ABL: %zu, AGESA: %zu)",
	        ARRAY_SIZE(abl_filter_error_codes),
	        ARRAY_SIZE(agesa_filter_error_codes));
#endif
}

bool pcc_platform_filter_postcode(uint32_t postcode)
{
	if(!postcode_filter_enabled) {
		return true;
	}

	uint32_t prefix = GET_POSTCODE_PREFIX(postcode);
	uint16_t high_word = GET_HIGH_WORD(postcode);
	
	switch (prefix) {
	case POSTCODE_PREFIX_BL:
		return validate_bl_postcode(postcode);
		
	case POSTCODE_PREFIX_UNEXPECTED:
		if (postcode == POSTCODE_EXCLUDE_ZERO || 
		    postcode == POSTCODE_EXCLUDE_ABCD) {
			return false;
		}
		return true;
		
	case POSTCODE_PREFIX_ABL_ERROR:
		return validate_abl_error_postcode(postcode);
		
	case POSTCODE_PREFIX_AGESA_ERROR:
		return validate_agesa_error_postcode(postcode);
		
	case POSTCODE_PREFIX_DDEE:
		return (high_word == 0xDDEE);
		
	default:
		return false;
	}
}

void plat_pcc_set_filter_enable(bool enable)
{
	postcode_filter_enabled = enable;
	LOG_INF("Postcode filter %s", enable ? "enabled" : "disabled");
}

bool plat_pcc_get_filter_enable(void)
{
	return postcode_filter_enabled;
}

uint8_t plat_pcc_copy_filtered_postcodes(uint32_t *buffer, uint8_t buffer_size)
{
	if (buffer == NULL) {
		LOG_ERR("Invalid buffer pointer");
		return 0;
	}
	
	if (buffer_size == 0) {
		LOG_WRN("Buffer size is zero");
		return 0;
	}
	
	if (buffer_size > FILTERED_POSTCODE_BUFFER_SIZE) {
		buffer_size = FILTERED_POSTCODE_BUFFER_SIZE;
	}
	
	k_mutex_lock(&filtered_postcode_mutex, K_FOREVER);
	
	uint8_t copy_count = (filtered_postcode_count < buffer_size) ? 
	                     filtered_postcode_count : buffer_size;
	
	for (uint8_t i = 0; i < copy_count; i++) {
		uint8_t src_idx = (filtered_postcode_index + 
		                   FILTERED_POSTCODE_BUFFER_SIZE - 1 - i) %
		                  FILTERED_POSTCODE_BUFFER_SIZE;
		buffer[i] = filtered_postcodes[src_idx];
	}
	
	k_mutex_unlock(&filtered_postcode_mutex);
	
	return copy_count;
}
