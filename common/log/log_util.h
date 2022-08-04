#ifndef LOG_UTIL_H
#define LOG_UTIL_H

#include <stdbool.h>
#include <stdint.h>

#define LOG_ENABLE 1
#define LOG_DISABLE 0

typedef enum {
	M_DEBUG_I2C,
	M_DEBUG_PECI,
	M_DEBUG_KCS,
	M_DEBUG_IPMB,
	M_DEBUG_IPMI,
	M_DEBUG_SENSOR,
	M_DEBUG_USB,
} log_type_t;

typedef struct _log_cfg__ {
	uint8_t log_idx;
	char name[15];
	uint8_t status;
} log_cfg_t;

extern const int LOG_CONFIG_SIZE;
extern const char *const log_name[];

uint8_t is_log_en(log_type_t log_idx);
uint8_t log_status_ctl(log_type_t log_idx, uint8_t status);

#endif
