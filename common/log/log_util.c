#include <zephyr.h>
#include <sys/printk.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "log_util.h"

#define log_num_to_name(x) #x,

// clang-format off
const char *const log_name[] = {
	log_num_to_name(DEBUG_I2C)
	log_num_to_name(DEBUG_PECI)
	log_num_to_name(DEBUG_KCS)
	log_num_to_name(DEBUG_IPMB)
	log_num_to_name(DEBUG_IPMI)
	log_num_to_name(DEBUG_SENSOR)
	log_num_to_name(DEBUG_USB)
};
// clang-format on

static uint8_t log_cfg[DEBUG_MAX];

uint8_t is_log_en(log_type_t log_idx)
{
	if (log_idx >= DEBUG_MAX)
		return false;
	if (!log_cfg[log_idx])
		return false;
	return true;
}

uint8_t log_status_ctl(log_type_t log_idx, uint8_t status)
{
	if (log_idx >= DEBUG_MAX)
		return false;
	if (status != LOG_ENABLE && status != LOG_DISABLE)
		return false;
	log_cfg[log_idx] = status;
	return true;
}
