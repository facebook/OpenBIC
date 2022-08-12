#ifndef LOG_UTIL_H
#define LOG_UTIL_H

#include <stdbool.h>
#include <stdint.h>

#define LOG_ENABLE 1
#define LOG_DISABLE 0

typedef enum {
	DEBUG_FW_UPDATE,
	DEBUG_I2C,
	DEBUG_IPMB,
	DEBUG_IPMI,
	DEBUG_KCS,
	DEBUG_PECI,
	DEBUG_SENSOR,
	DEBUG_SNOOP,
	DEBUG_USB,
	DEBUG_MAX,
} log_type_t;

extern const char *const log_name[];

uint8_t is_log_en(log_type_t log_idx);
uint8_t log_status_ctl(log_type_t log_idx, uint8_t status);

#endif
