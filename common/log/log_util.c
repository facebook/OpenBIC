#include <zephyr.h>
#include <sys/printk.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "log_util.h"

#define log_name_to_num(x) #x,
const char *const log_name[] = { log_name_to_num(DEBUG_I2C) log_name_to_num(DEBUG_PECI)
					 log_name_to_num(DEBUG_KCS) log_name_to_num(DEBUG_IPMB)
						 log_name_to_num(DEBUG_IPMI)
							 log_name_to_num(DEBUG_SENSOR)
								 log_name_to_num(DEBUG_USB) };

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
