#include <zephyr.h>
#include <sys/printk.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "log_util.h"

#define log_name_to_num(x) #x,
const char *const log_name[] = { log_name_to_num(M_DEBUG_I2C) log_name_to_num(M_DEBUG_PECI)
					 log_name_to_num(M_DEBUG_KCS) log_name_to_num(M_DEBUG_IPMI)
						 log_name_to_num(M_DEBUG_IPMI)
							 log_name_to_num(M_DEBUG_SENSOR)
								 log_name_to_num(M_DEBUG_USB) };

static log_cfg_t log_cfg[] = {
	{ M_DEBUG_I2C, "DEBUG_I2C", LOG_DISABLE },
	{ M_DEBUG_PECI, "DEBUG_PECI", LOG_DISABLE },
	{ M_DEBUG_KCS, "DEBUG_KCS", LOG_DISABLE },
	{ M_DEBUG_IPMB, "DEBUG_IPMB", LOG_DISABLE },
	{ M_DEBUG_IPMI, "DEBUG_IPMI", LOG_DISABLE },
	{ M_DEBUG_SENSOR, "DEBUG_SENSOR", LOG_DISABLE },
	{ M_DEBUG_USB, "DEBUG_USB", LOG_DISABLE },
};

uint8_t is_log_en(log_type_t log_idx)
{
	if (log_cfg[log_idx].log_idx != log_idx) {
		printf("%s: log idx doesn't mach with table index, please check log table log_cfg.",
		       __func__);
		return false;
	}
	if (!log_cfg[log_idx].status)
		return false;
	return true;
}

uint8_t log_status_ctl(log_type_t log_idx, uint8_t status)
{
	if (log_cfg[log_idx].log_idx != log_idx) {
		printf("%s: log idx doesn't mach with table index, please check log table log_cfg.",
		       __func__);
		return false;
	}
	if (status != LOG_ENABLE && status != LOG_DISABLE)
		return false;
	log_cfg[log_idx].status = status;
	return true;
}

const int LOG_CONFIG_SIZE = ARRAY_SIZE(log_cfg);
